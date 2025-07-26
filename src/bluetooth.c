#include "esp_log.h"
#include "esp_bt_device.h"
#include "nvs_flash.h"
#include <string.h>

#include "logging_config.h"
#include "bluetooth.h"
#include "elm327.h"
#include "gpio_control.h"

// Forward declarations for sound functions
extern void play_connection_sound(void);
extern void play_error_sound(void);

static const char *TAG = "BLE";

// Global BLE state variables
bool is_connected = false;
bool is_connecting = false;
bool is_scanning = false;
uint16_t gattc_if = ESP_GATT_IF_NONE;
uint16_t conn_id = 0xFFFF;  // Sentinel value for no connection

// Target ELM327 device address (VEEPEAK BLE ELM327)
esp_bd_addr_t target_elm327_addr = {0x66, 0x1E, 0x87, 0x02, 0x64, 0xC1};  // VEEPEAK ELM327

// GATT handles for UART service
uint16_t uart_service_handle = 0;
uint16_t tx_char_handle = 0;
uint16_t rx_char_handle = 0;
uint16_t rx_char_cccd_handle = 0;

// Service discovery globals
static uint16_t svc_start = 0, svc_end = 0;

// Nordic UART Service UUIDs (not used - using direct handles instead)
// static esp_bt_uuid_t uart_service_uuid = {
//     .len = ESP_UUID_LEN_128,
//     .uuid = {.uuid128 = {0x21, 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0x5E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x6E}}
// };

// TX and RX UUIDs will be discovered dynamically

static int connection_attempt = 0;

// Pending connection variables (for proper scan-stop-then-connect flow)
static esp_bd_addr_t pending_bda;
static esp_ble_addr_type_t pending_addr_type;
static bool connect_pending = false;

// Forward declarations
void restart_scan_task(void *pvParameters);
void scan_timeout_task(void *pvParameters);
void reconnection_watchdog_task(void *pvParameters);
void ble_recovery_task(void *pvParameters);
void poll_rx_char_task(void *pvParameters);

// Helper function to compare BD addresses
static bool bd_addr_equal(esp_bd_addr_t a, esp_bd_addr_t b) {
    return memcmp(a, b, ESP_BD_ADDR_LEN) == 0;
}

// GAP callback for BLE scanning
static void gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
            
            if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                char addr_str[18];
                snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                        scan_result->scan_rst.bda[0], scan_result->scan_rst.bda[1], 
                        scan_result->scan_rst.bda[2], scan_result->scan_rst.bda[3],
                        scan_result->scan_rst.bda[4], scan_result->scan_rst.bda[5]);
                
                // Log ALL discovered BLE devices for debugging
                LOG_INFO(TAG, "📱 BLE Device Found: %s (RSSI: %d, AddrType: %d)", 
                        addr_str, scan_result->scan_rst.rssi, scan_result->scan_rst.ble_addr_type);
                
                // Check if this is our target ELM327 device
                if (bd_addr_equal(scan_result->scan_rst.bda, target_elm327_addr)) {
                    if (is_connecting || connect_pending) {
                        ESP_LOGD(TAG, "🎯 Already connecting/pending to ELM327, ignoring duplicate");
                        return;
                    }
                    
                    LOG_INFO(TAG, "🎯 MATCH! This is our target VEEPEAK ELM327: %s", addr_str);
                    LOG_INFO(TAG, "📍 Device address type: %d, RSSI: %d", scan_result->scan_rst.ble_addr_type, scan_result->scan_rst.rssi);
                    
                    // Store connection info for later (after scan stops)
                    memcpy(pending_bda, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
                    pending_addr_type = scan_result->scan_rst.ble_addr_type;
                    connect_pending = true;
                    
                    LOG_INFO(TAG, "🛑 Stopping scan first, then will connect...");
                    esp_ble_gap_stop_scanning();  // Connection will happen in SCAN_STOP_COMPLETE_EVT
                } else {
                    // We already logged this device above, no need to log again
                }
            }
            break;
        }
        
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                LOG_BT(TAG, "🔍 BLE scan started successfully");
                is_scanning = true;
                
                // Create timeout task to restart scan if no device found
                xTaskCreate(scan_timeout_task, "scan_timeout", 2048, NULL, 3, NULL);
            } else {
                LOG_ERROR(TAG, "❌ Failed to start BLE scan: %d", param->scan_start_cmpl.status);
            }
            break;
            
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            LOG_BT(TAG, "🔍 BLE scan stopped");
            is_scanning = false;
            
            if (connect_pending) {
                // Scan stopped and we have a pending connection - now it's safe to connect
                connect_pending = false;
                is_connecting = true;
                
                char addr_str[18];
                snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                        pending_bda[0], pending_bda[1], pending_bda[2], 
                        pending_bda[3], pending_bda[4], pending_bda[5]);
                
                LOG_INFO(TAG, "🔗 Scan fully stopped - now connecting to VEEPEAK: %s", addr_str);
                LOG_INFO(TAG, "📍 Using address type: %d (direct connection)", pending_addr_type);
                
                // Clear white list before connecting
                esp_ble_gap_clear_whitelist();
                
                // Initiate direct connection (is_direct = true)
                esp_err_t ret = esp_ble_gattc_open(gattc_if, pending_bda, pending_addr_type, true);
                if (ret != ESP_OK) {
                    LOG_ERROR(TAG, "❌ Connection failed: %s", esp_err_to_name(ret));
                    is_connecting = false;
                } else {
                    LOG_INFO(TAG, "✅ BLE connection request sent - waiting for CONNECT_EVT...");
                    LOG_INFO(TAG, "⏰ Will timeout after 8 seconds if no response");
                }
            } else if (!is_connecting && !is_connected) {
                // Scan completed but device not found - restart scan
                ESP_LOGW(TAG, "🔄 ELM327 not found during scan - will retry in 5 seconds");
                xTaskCreate(restart_scan_task, "restart_scan", 2048, NULL, 3, NULL);
            }
            break;
            
        default:
            ESP_LOGD(TAG, "GAP BLE event: %d", event);
            break;
    }
}

// GATT client callback for connection and service discovery
static void gattc_callback(esp_gattc_cb_event_t event, esp_gatt_if_t gatt_if, esp_ble_gattc_cb_param_t *param) {
    // Simplified event logging to reduce stack usage
    if (event == ESP_GATTC_CONNECT_EVT || event == ESP_GATTC_DISCONNECT_EVT || 
        event == ESP_GATTC_NOTIFY_EVT || event == ESP_GATTC_SEARCH_CMPL_EVT) {
        LOG_INFO(TAG, "🔧 GATT Event: %d", event);
    }
    
    switch (event) {
        case ESP_GATTC_REG_EVT:
            LOG_BT(TAG, "GATT client registered, interface: %d", gatt_if);
            gattc_if = gatt_if;
            
            // Start scanning for ELM327 device
            start_ble_scan();
            break;
            
                case ESP_GATTC_CONNECT_EVT:
            conn_id = param->connect.conn_id;
            is_connecting = false;  // ✅ Clear connecting state
            is_connected = true;    // ✅ Set connected state
            
            LOG_BT(TAG, "✅ Connected to ELM327 BLE device (conn_id: %d)", conn_id);
            LOG_INFO(TAG, "🎉 VEEPEAK BLE connection successful!");
            
            // Request larger MTU for better throughput
            esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, conn_id);
            LOG_INFO(TAG, "📶 Requesting 247 byte MTU: %s", esp_err_to_name(mtu_ret));
            
            // Start service discovery to build attribute cache
            LOG_INFO(TAG, "🔍 Starting service discovery to build attribute cache...");
            esp_err_t ret = esp_ble_gattc_search_service(gattc_if, conn_id, NULL);
            if (ret != ESP_OK) {
                LOG_ERROR(TAG, "❌ Failed to start service discovery: %s", esp_err_to_name(ret));
            }
            break;
            
        case ESP_GATTC_OPEN_EVT:
            LOG_BT(TAG, "🔗 GATT OPEN event - status: %d", param->open.status);
            if (param->open.status == ESP_GATT_OK) {
                LOG_BT(TAG, "✅ GATT connection opened successfully!");
                conn_id = param->open.conn_id;
                
                // If this is the first connection event we receive (some devices send OPEN before CONNECT)
                if (is_connecting) {
                    is_connecting = false;
                    is_connected = true;
                    LOG_INFO(TAG, "🎉 VEEPEAK connection established via OPEN_EVT");
                }
            } else {
                LOG_ERROR(TAG, "❌ GATT open failed with status: %d", param->open.status);
                is_connecting = false;
                is_connected = false;
            }
            break;
            
        case ESP_GATTC_DISCONNECT_EVT:
            LOG_WARN(TAG, "🔴 Disconnected from ELM327 (reason: %d)", param->disconnect.reason);
            
            // Check if this was a BLE protocol failure (quick disconnect after connection)
            if (param->disconnect.reason == 0x13 || param->disconnect.reason == 0x16) {
                LOG_ERROR(TAG, "💥 BLE protocol incompatibility detected!");
                LOG_ERROR(TAG, "🔧 VEEPEAK may not be compatible with ESP32-S3 BLE stack");
                LOG_ERROR(TAG, "💡 Recommendation: Use ESP32 (Classic Bluetooth) instead");
                connection_attempt += 10; // Increase attempt count to reduce retry frequency
            }
            
            is_connected = false;
            is_connecting = false;
            conn_id = 0xFFFF;
            
            // Reset handles
            uart_service_handle = 0;
            tx_char_handle = 0;
            rx_char_handle = 0;
            rx_char_cccd_handle = 0;
            
            // Reset service discovery globals
            svc_start = 0;
            svc_end = 0;
            
            // Clear white list on disconnect to prevent "already in initiator white list" errors
            esp_err_t clear_ret = esp_ble_gap_clear_whitelist();
            if (clear_ret == ESP_OK) {
                LOG_INFO(TAG, "🧹 White list cleared on disconnect");
            }
            
            set_ecu_status(false);
            
            // Play error sound for disconnection
            play_error_sound();
            
            // Start reconnection attempts with longer delay for compatibility issues
            if (connection_attempt > 10) {
                LOG_WARN(TAG, "⏰ Multiple failures detected - waiting 30 seconds before retry");
                vTaskDelay(pdMS_TO_TICKS(30000));
            }
            
            xTaskCreate(reconnection_watchdog_task, "reconnect_watchdog", 4096, NULL, 2, NULL);
            break;
            
        case ESP_GATTC_SEARCH_RES_EVT: {
            // Check for UART service - test 128-bit Nordic UUID first, then fall back to 0xFFF0/0xFFE0
            bool is_uart_service = false;
            esp_bt_uuid_t *uuid = &param->search_res.srvc_id.uuid;
            
            if (uuid->len == ESP_UUID_LEN_128) {
                // Nordic UART Service: 49535343-FE7D-4AE5-8FA9-9FAFD205E455
                uint8_t nordic_uuid[16] = {0x55,0xE4,0x05,0xD2,0xAF,0x9F,0xA9,0x8F,
                                          0xE5,0x4A,0x7D,0xFE,0x43,0x53,0x53,0x49};
                if (memcmp(uuid->uuid.uuid128, nordic_uuid, 16) == 0) {
                    is_uart_service = true;
                    LOG_INFO(TAG, "📌 Found Nordic UART Service (128-bit)");
                }
            } else if (uuid->len == ESP_UUID_LEN_16) {
                // Legacy services: 0xFFF0 or 0xFFE0
                if (uuid->uuid.uuid16 == 0xFFF0) {
                    is_uart_service = true;
                    LOG_INFO(TAG, "📌 Found Legacy UART Service (FFF0)");
                } else if (uuid->uuid.uuid16 == 0xFFE0) {
                    is_uart_service = true;
                    LOG_INFO(TAG, "📌 Found Legacy UART Service (FFE0)");
                }
            }
            
            if (is_uart_service) {
                svc_start = param->search_res.start_handle;
                svc_end = param->search_res.end_handle;
                LOG_INFO(TAG, "📌 UART service found: start=0x%04X end=0x%04X", svc_start, svc_end);
            }
            break;
        }
            
        case ESP_GATTC_SEARCH_CMPL_EVT:
            LOG_INFO(TAG, "✅ Service discovery complete - attribute cache built");
            
            if (svc_start == 0) {
                LOG_ERROR(TAG, "❌ UART service not found during discovery");
                break;
            }
            
            // All UART characteristic UUIDs we accept (different VEEPEAK firmware versions)
            const esp_bt_uuid_t uart_char_uuids[] = {
                // Nordic UART RX: 49535343-6DAA-4D02-ABF6-19569ACA69FE
                { .len = ESP_UUID_LEN_128,
                  .uuid = { .uuid128 = 
                      {0xFE,0x69,0xCA,0x9A,0x56,0x19,0xF6,0xAB,
                       0x02,0x4D,0xA4,0x6D,0x43,0x53,0x53,0x49} } },
                // Nordic UART TX: 49535343-ACA3-481C-91EC-D85E28A60318  
                { .len = ESP_UUID_LEN_128,
                  .uuid = { .uuid128 = 
                      {0x18,0x03,0xA6,0x28,0x5E,0xD8,0xEC,0x91,
                       0x1C,0x48,0xA3,0xAC,0x43,0x53,0x53,0x49} } },
                // Legacy FFF1 (receive)
                { .len = ESP_UUID_LEN_16, .uuid = { .uuid16 = 0xFFF1 } },
                // Legacy FFF2 (transmit)
                { .len = ESP_UUID_LEN_16, .uuid = { .uuid16 = 0xFFF2 } },
                // Legacy FFE1 (combined RX/TX)
                { .len = ESP_UUID_LEN_16, .uuid = { .uuid16 = 0xFFE1 } }
            };
            
            LOG_INFO(TAG, "🔍 Looking for characteristics by UUID...");
            esp_gattc_char_elem_t char_elem;
            bool rx_found = false, tx_found = false;
            
            // Search for both RX and TX characteristics, checking properties to distinguish them
            for (size_t i = 0; i < sizeof(uart_char_uuids)/sizeof(uart_char_uuids[0]) && (!rx_found || !tx_found); ++i) {
                uint16_t count = 1;  // ← CRITICAL: Reset count before every call!
                esp_err_t ret = esp_ble_gattc_get_char_by_uuid(gattc_if, conn_id,
                                                              svc_start, svc_end,
                                                              uart_char_uuids[i], &char_elem, &count);
                if (ret == ESP_OK && count > 0) {
                    // Log detailed characteristic properties for debugging
                    LOG_INFO(TAG, "🔍 Found characteristic: handle=0x%04X, properties=0x%02X, UUID index=%zu", 
                             char_elem.char_handle, char_elem.properties, i);
                    LOG_INFO(TAG, "   Properties: Read=%d, Write=%d, WriteNR=%d, Notify=%d, Indicate=%d",
                             !!(char_elem.properties & ESP_GATT_CHAR_PROP_BIT_READ),
                             !!(char_elem.properties & ESP_GATT_CHAR_PROP_BIT_WRITE),
                             !!(char_elem.properties & ESP_GATT_CHAR_PROP_BIT_WRITE_NR),
                             !!(char_elem.properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY),
                             !!(char_elem.properties & ESP_GATT_CHAR_PROP_BIT_INDICATE));
                    
                    // Check characteristic properties to determine if it's RX or TX
                    if ((char_elem.properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) && !rx_found) {
                        rx_char_handle = char_elem.char_handle;
                        rx_found = true;
                        LOG_INFO(TAG, "✅ Assigned as RX characteristic (Notify capability) at handle: 0x%04X", rx_char_handle);
                        if (!(char_elem.properties & ESP_GATT_CHAR_PROP_BIT_READ)) {
                            LOG_WARN(TAG, "⚠️ RX characteristic does NOT support Read - polling will fail!");
                        } else {
                            LOG_INFO(TAG, "✅ RX characteristic supports Read - polling should work");
                        }
                    } else if ((char_elem.properties & (ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR)) && !tx_found) {
                        tx_char_handle = char_elem.char_handle;
                        tx_found = true;
                        LOG_INFO(TAG, "✅ Assigned as TX characteristic (Write capability) at handle: 0x%04X", tx_char_handle);
                    }
                }
            }
            
            if (!rx_found) {
                LOG_ERROR(TAG, "❌ RX characteristic not found in any known UUID");
                break;
            }
            
            if (!tx_found) {
                LOG_WARN(TAG, "⚠️ TX characteristic not found, may not be able to send commands");
            }
            
            // Find the CCCD handle for notifications
            LOG_INFO(TAG, "🔍 Looking for CCCD descriptor...");
            esp_bt_uuid_t cccd_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = { .uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG }
            };
            esp_gattc_descr_elem_t descr_elem;
            uint16_t count = 1;
            
            esp_err_t descr_ret = esp_ble_gattc_get_descr_by_char_handle(
                    gattc_if, conn_id, rx_char_handle, cccd_uuid, &descr_elem, &count);
            
            if (descr_ret == ESP_OK && count > 0) {
                rx_char_cccd_handle = descr_elem.handle;
                LOG_INFO(TAG, "✅ Found CCCD at handle: 0x%04X", rx_char_cccd_handle);
            } else {
                // Fallback to +1 (usually works)
                rx_char_cccd_handle = rx_char_handle + 1;
                LOG_WARN(TAG, "⚠️ CCCD not found in cache, trying fallback: 0x%04X", rx_char_cccd_handle);
                
                // Also try other possible CCCD handles for debugging
                LOG_INFO(TAG, "🔧 Alternative CCCD candidates:");
                LOG_INFO(TAG, "   RX+1: 0x%04X (using this)", rx_char_handle + 1);
                LOG_INFO(TAG, "   TX+1: 0x%04X", tx_char_handle + 1);
                LOG_INFO(TAG, "   Service end: 0x%04X", svc_end);
            }
            
            // Enable notifications - detailed logging for debugging
            uint8_t enable_ntf[2] = {0x01, 0x00};
            LOG_INFO(TAG, "🔧 CCCD Write Details:");
            LOG_INFO(TAG, "   RX char handle: 0x%04X", rx_char_handle);
            LOG_INFO(TAG, "   CCCD handle: 0x%04X", rx_char_cccd_handle);
            LOG_INFO(TAG, "   Writing bytes: 0x%02X 0x%02X (enable notifications)", enable_ntf[0], enable_ntf[1]);
            
            esp_err_t write_ret = esp_ble_gattc_write_char_descr(gattc_if, conn_id,
                                                               rx_char_cccd_handle,
                                                               sizeof(enable_ntf), enable_ntf,
                                                               ESP_GATT_WRITE_TYPE_RSP,
                                                               ESP_GATT_AUTH_REQ_NONE);
            if (write_ret == ESP_OK) {
                LOG_INFO(TAG, "✉️ Enabling notifications (CCCD 0x%04X)...", rx_char_cccd_handle);
            } else {
                LOG_ERROR(TAG, "❌ Failed to write CCCD: %s", esp_err_to_name(write_ret));
            }
            break;
            
        case ESP_GATTC_REG_FOR_NOTIFY_EVT:
            LOG_BT(TAG, "✅ Notifications registered successfully!");
            break;
            
        case ESP_GATTC_WRITE_DESCR_EVT:
            LOG_INFO(TAG, "📝 CCCD Write Result: handle=0x%04X, status=%d", param->write.handle, param->write.status);
            if (param->write.status == ESP_GATT_OK) {
                LOG_INFO(TAG, "✅ CCCD written - notifications enabled!");
                LOG_INFO(TAG, "🔔 RX notifications should now arrive at handle 0x%04X", rx_char_handle);
                
                // Skip polling since RX characteristic doesn't support reading
                LOG_INFO(TAG, "🚫 Polling disabled as RX characteristic does not support Read (status=0x02)");
                
                // Verify CCCD write by reading it back
                LOG_INFO(TAG, "🔍 Reading back CCCD to verify...");
                esp_err_t read_ret = esp_ble_gattc_read_char_descr(gattc_if, conn_id, rx_char_cccd_handle, ESP_GATT_AUTH_REQ_NONE);
                if (read_ret != ESP_OK) {
                    LOG_ERROR(TAG, "❌ Failed to read back CCCD: %s", esp_err_to_name(read_ret));
                }
                
                LOG_INFO(TAG, "🚀 Starting ELM327 initialization...");
                
                // Start ELM327 initialization
                xTaskCreate(initialize_elm327_task, "elm327_init", 4096, NULL, 2, NULL);
            } else {
                LOG_ERROR(TAG, "❌ Write CCCD failed, status 0x%02X", param->write.status);
            }
            break;
            
        case ESP_GATTC_CFG_MTU_EVT:
            LOG_INFO(TAG, "📶 MTU configured: %d bytes (status: %d)", param->cfg_mtu.mtu, param->cfg_mtu.status);
            break;
            
        case ESP_GATTC_NOTIFY_EVT: {
            // SUCCESS! Received notification from ELM327
            LOG_INFO(TAG, "🎉 SUCCESS! GATT NOTIFY received from handle 0x%04X", param->notify.handle);
            LOG_INFO(TAG, "   Expected RX handle: 0x%04X %s", rx_char_handle, 
                    (param->notify.handle == rx_char_handle) ? "(MATCH - PERFECT!)" : "(MISMATCH!)");
            LOG_INFO(TAG, "⬇️ NOTIFY (%d bytes): %.*s", param->notify.value_len, param->notify.value_len, param->notify.value);
            
            // Also log as hex for debugging
            LOG_INFO(TAG, "⬇️ NOTIFY HEX: ");
            for (int i = 0; i < param->notify.value_len; i++) {
                printf("%02X ", param->notify.value[i]);
            }
            printf("\n");
            
            // Forward data to ELM327 handler 
            handle_elm327_response((char *)param->notify.value, param->notify.value_len);
            break;
        }
        
        case ESP_GATTC_WRITE_CHAR_EVT:
            if (param->write.status != ESP_GATT_OK) {
                LOG_ERROR(TAG, "❌ Failed to write data: %d", param->write.status);
            }
            break;

        case ESP_GATTC_READ_CHAR_EVT:
            // Polling is disabled since RX characteristic doesn't support Read
            LOG_WARN(TAG, "⚠️ Unexpected READ_CHAR_EVT - polling disabled (RX characteristic read not supported)");
            if (param->read.status != ESP_GATT_OK) {
                LOG_INFO(TAG, "   Confirmed: Read failed with status=0x%02X (ESP_GATT_READ_NOT_PERMIT=0x02)", param->read.status);
            }
            break;
            
        case ESP_GATTC_READ_DESCR_EVT:
            LOG_INFO(TAG, "📖 CCCD Read Result: handle=0x%04X, status=%d", param->read.handle, param->read.status);
            if (param->read.status == ESP_GATT_OK) {
                LOG_INFO(TAG, "📖 CCCD value (%d bytes): ", param->read.value_len);
                printf("CCCD HEX: ");
                for (int i = 0; i < param->read.value_len; i++) {
                    printf("%02X ", param->read.value[i]);
                }
                printf("\n");
                if (param->read.value_len >= 2) {
                    uint16_t cccd_value = param->read.value[0] | (param->read.value[1] << 8);
                    LOG_INFO(TAG, "📖 CCCD value: 0x%04X %s", cccd_value, 
                            (cccd_value & 0x01) ? "(Notifications enabled)" : "(Notifications disabled)");
                }
            } else {
                LOG_ERROR(TAG, "❌ Failed to read CCCD: status=%d", param->read.status);
            }
            break;
            

        default:
            // Only log truly unhandled events
            LOG_WARN(TAG, "⚠️ Unhandled GATT Event: %d", event);
            break;
    }
}

// Write data to ELM327 via BLE UART
esp_err_t ble_uart_write(uint8_t *data, size_t len) {
    if (!is_connected || tx_char_handle == 0) {
        LOG_ERROR(TAG, "❌ BLE not connected or TX characteristic not found");
        return ESP_FAIL;
    }
    
    LOG_BT(TAG, "⬆️ SEND (%d bytes): %.*s", len, (int)len, (char *)data);
    
    return esp_ble_gattc_write_char(gattc_if, conn_id, tx_char_handle,
                                   len, data, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
}

// Start BLE scanning
void start_ble_scan(void) {
    if (is_scanning) {
        ESP_LOGD(TAG, "🔍 BLE scan already in progress");
        return;
    }
    
    // Check if BLE stack is ready
    if (gattc_if == ESP_GATT_IF_NONE) {
        LOG_ERROR(TAG, "❌ BLE stack not ready (gattc_if not registered)");
        return;
    }
    
    LOG_BT(TAG, "🔍 Starting BLE scan for ELM327...");
    
    esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,   // Use ACTIVE scan to wake up VEEPEAK
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,   // 50ms interval (standard value)
        .scan_window = 0x30,     // 30ms window (60% duty cycle)
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE  // Disable to see all advertisements
    };
    
    LOG_INFO(TAG, "📐 Scan params: interval=%.1fms, window=%.1fms, type=%s", 
             scan_params.scan_interval * 0.625, 
             scan_params.scan_window * 0.625,
             (scan_params.scan_type == BLE_SCAN_TYPE_ACTIVE) ? "ACTIVE" : "PASSIVE");
    
    // Wait for BLE stack to be fully ready
    vTaskDelay(pdMS_TO_TICKS(500));
    
    esp_err_t ret = esp_ble_gap_set_scan_params(&scan_params);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "❌ Failed to set scan parameters: %s (0x%X)", esp_err_to_name(ret), ret);
        is_scanning = false;
        return;
    }
    
    LOG_INFO(TAG, "✅ Scan parameters set successfully");
    
    // Wait for scan params to be applied
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ret = esp_ble_gap_start_scanning(0);  // 0 = scan indefinitely
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "❌ Failed to start scanning: %s", esp_err_to_name(ret));
        is_scanning = false;
        return;
    }
    
    // Start scan timeout task
    xTaskCreate(scan_timeout_task, "scan_timeout", 2048, NULL, 1, NULL);
}

// Initialize Bluetooth system
void bluetooth_init(void) {
    LOG_BT(TAG, "🚀 Initializing BLE system...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "❌ Failed to initialize BT controller: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "❌ Failed to enable BT controller: %s", esp_err_to_name(ret));
        return;
    }
    
    // Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "❌ Failed to init bluedroid: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "❌ Failed to enable bluedroid: %s", esp_err_to_name(ret));
        return;
    }
    
    // Register GAP callback
    ret = esp_ble_gap_register_callback(gap_callback);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "❌ Failed to register GAP callback: %s", esp_err_to_name(ret));
        return;
    }
    
    // Register GATT client
    ret = esp_ble_gattc_register_callback(gattc_callback);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "❌ Failed to register GATT client callback: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_ble_gattc_app_register(0);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "❌ Failed to register GATT client app: %s", esp_err_to_name(ret));
        return;
    }
    
    LOG_BT(TAG, "✅ BLE initialization complete");
    
    // Start reconnection watchdog
    xTaskCreate(reconnection_watchdog_task, "reconnect_watchdog", 4096, NULL, 2, NULL);
    
    // Start BLE recovery monitor
    xTaskCreate(ble_recovery_task, "ble_recovery", 3072, NULL, 1, NULL);
    LOG_BT(TAG, "🔧 BLE recovery monitor started");
}

// Handle connection failure
void handle_connection_failure(void) {
    connection_attempt++;
    LOG_WARN(TAG, "🔴 Connection attempt %d failed", connection_attempt);
    
    // Wait before retrying
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    if (!is_connected) {
        start_ble_scan();
    }
}

// Task to restart scanning after timeout
void restart_scan_task(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(5000));  // Wait 5 seconds
    
    if (!is_connected && !is_connecting) {
        LOG_INFO(TAG, "🔄 Restarting BLE scan...");
        start_ble_scan();
    }
    
    vTaskDelete(NULL);
}

// Task to handle scan timeout
void scan_timeout_task(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(18000));  // Wait 18 seconds (shorter than scan)
    
    if (is_scanning && !is_connecting) {
        LOG_WARN(TAG, "⏰ Scan timeout - stopping scan");
        esp_ble_gap_stop_scanning();
        is_scanning = false;
        
        // Restart scan after delay if not connected
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (!is_connected && !is_connecting) {
            LOG_INFO(TAG, "🔄 Restarting BLE scan...");
            start_ble_scan();
        }
    }
    
    vTaskDelete(NULL);
}

// Reconnection watchdog task
void reconnection_watchdog_task(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  // Check every 10 seconds
        
        if (!is_connected && !is_connecting && !is_scanning) {
            LOG_INFO(TAG, "🔄 Watchdog: Attempting reconnection...");
            start_ble_scan();
        }
    }
}

// BLE recovery task - monitors for stuck BLE stack and attempts recovery
void ble_recovery_task(void *pvParameters) {
    uint32_t last_activity_time = xTaskGetTickCount();
    uint32_t stuck_detection_count = 0;
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));  // Check every 5 seconds
        
        uint32_t current_time = xTaskGetTickCount();
        
        // Check if we're in a problematic state
        bool potentially_stuck = false;
        
        // If we've been connecting for more than 8 seconds, something's wrong
        if (is_connecting && (current_time - last_activity_time) > pdMS_TO_TICKS(8000)) {
            potentially_stuck = true;
            LOG_WARN(TAG, "🔧 BLE recovery: Connection stuck for >8s");
        }
        
        // If we've had recent activity, reset counters
        if (is_connected || (!is_connecting && !is_scanning)) {
            last_activity_time = current_time;
            stuck_detection_count = 0;
        }
        
        // If stuck, attempt recovery
        if (potentially_stuck) {
            stuck_detection_count++;
            LOG_WARN(TAG, "🚨 BLE stack appears stuck (attempt %lu/3)", (unsigned long)stuck_detection_count);
            
            if (stuck_detection_count >= 3) {
                LOG_ERROR(TAG, "🔄 Attempting BLE stack recovery...");
                
                // Force disconnect and reset state
                if (conn_id != 0xFFFF) {
                    esp_ble_gattc_close(gattc_if, conn_id);
                    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for close to complete
                }
                
                // Clear BLE white list to prevent "already in initiator white list" error
                esp_err_t ret = esp_ble_gap_clear_whitelist();
                if (ret == ESP_OK) {
                    LOG_INFO(TAG, "✅ BLE white list cleared");
                } else {
                    LOG_WARN(TAG, "⚠️ Failed to clear white list: %s", esp_err_to_name(ret));
                }
                
                is_connected = false;
                is_connecting = false;
                is_scanning = false;
                conn_id = 0xFFFF;
                
                // Reset all handles
                uart_service_handle = 0;
                tx_char_handle = 0;
                rx_char_handle = 0;
                rx_char_cccd_handle = 0;
                
                // Reset service discovery globals
                svc_start = 0;
                svc_end = 0;
                
                LOG_INFO(TAG, "💡 BLE state reset - will restart scanning");
                
                // Wait a bit then restart
                vTaskDelay(pdMS_TO_TICKS(5000));
                start_ble_scan();
                
                stuck_detection_count = 0;
                last_activity_time = xTaskGetTickCount();
            }
        }
    }
} 

// Polling task for RX characteristic (fallback if notifications don't work)
void poll_rx_char_task(void *pvParameters) {
    LOG_INFO(TAG, "🔄 Starting RX characteristic polling as fallback...");
    LOG_INFO(TAG, "📍 Polling RX handle: 0x%04X, conn_id: %d, gattc_if: %d", rx_char_handle, conn_id, gattc_if);
    
    uint32_t consecutive_failures = 0;
    const uint32_t max_consecutive_failures = 10;
    uint32_t total_attempts = 0;
    
    while (is_connected && rx_char_handle != 0 && consecutive_failures < max_consecutive_failures) {
        total_attempts++;
        
        esp_err_t ret = esp_ble_gattc_read_char(gattc_if, conn_id, rx_char_handle, ESP_GATT_AUTH_REQ_NONE);
        if (ret != ESP_OK) {
            consecutive_failures++;
            LOG_ERROR(TAG, "❌ Failed to initiate read (attempt %lu): %s", (unsigned long)total_attempts, esp_err_to_name(ret));
            LOG_ERROR(TAG, "   Consecutive failures: %lu/%lu", (unsigned long)consecutive_failures, (unsigned long)max_consecutive_failures);
            vTaskDelay(pdMS_TO_TICKS(500)); // Wait longer on error
            continue;
        }
        
        // Reset failure count on successful read initiation
        consecutive_failures = 0;
        
        vTaskDelay(pdMS_TO_TICKS(200)); // Poll every 200ms (slower than before)
    }
    
    if (consecutive_failures >= max_consecutive_failures) {
        LOG_ERROR(TAG, "🛑 Polling task stopped: too many consecutive failures (%lu)", (unsigned long)consecutive_failures);
    } else {
        LOG_INFO(TAG, "🛑 Polling task stopped (disconnected or no RX handle)");
    }
    
    LOG_INFO(TAG, "📊 Polling statistics: %lu total attempts, %lu final consecutive failures", 
             (unsigned long)total_attempts, (unsigned long)consecutive_failures);
    
    vTaskDelete(NULL);
} 