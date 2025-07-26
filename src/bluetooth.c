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
uint16_t conn_id = 0;

// Target ELM327 device address (VEEPEAK BLE ELM327)
esp_bd_addr_t target_elm327_addr = {0x66, 0x1E, 0x87, 0x02, 0x64, 0xC1};  // VEEPEAK ELM327

// GATT handles for UART service
uint16_t uart_service_handle = 0;
uint16_t tx_char_handle = 0;
uint16_t rx_char_handle = 0;
uint16_t rx_char_cccd_handle = 0;

// Nordic UART Service UUIDs
static esp_bt_uuid_t uart_service_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = {0x21, 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0x5E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x6E}}
};

// TX and RX UUIDs will be discovered dynamically

static int connection_attempt = 0;

// Forward declarations
void restart_scan_task(void *pvParameters);
void scan_timeout_task(void *pvParameters);
void reconnection_watchdog_task(void *pvParameters);

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
                
                // Check if this is our target ELM327 device
                if (bd_addr_equal(scan_result->scan_rst.bda, target_elm327_addr)) {
                    if (is_connecting) {
                        ESP_LOGD(TAG, "🎯 Already connecting to ELM327, ignoring duplicate discovery");
                        return;
                    }
                    
                    LOG_INFO(TAG, "🎯 Found target ELM327 BLE device: %s", addr_str);
                    LOG_BT(TAG, "Attempting BLE connection to ELM327...");
                    
                    is_connecting = true;
                    esp_ble_gap_stop_scanning();
                    
                    // Connect to the ELM327 device
                    esp_err_t ret = esp_ble_gattc_open(gattc_if, scan_result->scan_rst.bda, 
                                                     scan_result->scan_rst.ble_addr_type, true);
                    if (ret != ESP_OK) {
                        LOG_ERROR(TAG, "❌ Failed to initiate connection: %s", esp_err_to_name(ret));
                        is_connecting = false;
                    } else {
                        LOG_INFO(TAG, "✅ BLE connection initiated");
                    }
                } else {
                    ESP_LOGD(TAG, "📱 Found other BLE device: [%s] - skipping", addr_str);
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
            
            if (!is_connecting && !is_connected) {
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
    switch (event) {
        case ESP_GATTC_REG_EVT:
            LOG_BT(TAG, "GATT client registered, interface: %d", gatt_if);
            gattc_if = gatt_if;
            
            // Start scanning for ELM327 device
            start_ble_scan();
            break;
            
        case ESP_GATTC_CONNECT_EVT:
            conn_id = param->connect.conn_id;
            LOG_BT(TAG, "✅ Connected to ELM327 BLE device");
            LOG_INFO(TAG, "🔗 BLE connection established - discovering services...");
            
            // Start service discovery
            esp_ble_gattc_search_service(gatt_if, conn_id, &uart_service_uuid);
            break;
            
        case ESP_GATTC_DISCONNECT_EVT:
            LOG_WARN(TAG, "🔴 Disconnected from ELM327 (reason: %d)", param->disconnect.reason);
            is_connected = false;
            is_connecting = false;
            conn_id = 0;
            
            // Reset handles
            uart_service_handle = 0;
            tx_char_handle = 0;
            rx_char_handle = 0;
            rx_char_cccd_handle = 0;
            
            set_ecu_status(false);
            
            // Play error sound for disconnection
            play_error_sound();
            
            // Start reconnection attempts
            xTaskCreate(reconnection_watchdog_task, "reconnect_watchdog", 4096, NULL, 2, NULL);
            break;
            
        case ESP_GATTC_SEARCH_RES_EVT:
            // For simplicity, assume first service found is our UART service
            uart_service_handle = param->search_res.start_handle;
            LOG_BT(TAG, "🔍 Found service, handle: %d", uart_service_handle);
            break;
            
        case ESP_GATTC_SEARCH_CMPL_EVT:
            if (uart_service_handle != 0) {
                LOG_BT(TAG, "✅ Service discovery complete - finding characteristics...");
                // Get all characteristics of the UART service
                // For now, manually set characteristic handles (simplified approach)
                tx_char_handle = uart_service_handle + 2;  // Typically offset +2 for TX
                rx_char_handle = uart_service_handle + 3;  // Typically offset +3 for RX
                
                LOG_BT(TAG, "📤 Assuming TX characteristic handle: %d", tx_char_handle);
                LOG_BT(TAG, "📥 Assuming RX characteristic handle: %d", rx_char_handle);
                
                // Register for notifications on RX characteristic
                esp_ble_gattc_register_for_notify(gatt_if, target_elm327_addr, rx_char_handle);
            } else {
                LOG_ERROR(TAG, "❌ UART service not found");
                esp_ble_gattc_close(gatt_if, conn_id);
            }
            break;
            
        case ESP_GATTC_REG_FOR_NOTIFY_EVT:
            LOG_BT(TAG, "✅ Notifications enabled - BLE UART ready!");
            is_connected = true;
            is_connecting = false;
            
            // Play connection sound and start ELM327 initialization
            play_connection_sound();
            xTaskCreate(initialize_elm327_task, "elm327_init", 4096, NULL, 3, NULL);
            break;
            

            
        case ESP_GATTC_NOTIFY_EVT: {
            // Received data from ELM327
            LOG_BT(TAG, "⬇️ RECV (%d bytes): %.*s", param->notify.value_len, param->notify.value_len, param->notify.value);
            
            // Forward data to ELM327 handler
            handle_elm327_response((char *)param->notify.value, param->notify.value_len);
            break;
        }
        
        case ESP_GATTC_WRITE_CHAR_EVT:
            if (param->write.status != ESP_GATT_OK) {
                LOG_ERROR(TAG, "❌ Failed to write data: %d", param->write.status);
            }
            break;
            
        default:
            ESP_LOGD(TAG, "GATT client event: %d", event);
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
    
    LOG_BT(TAG, "🔍 Starting BLE scan for ELM327...");
    
    esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,  // 50ms
        .scan_window = 0x30,    // 30ms
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
    };
    
    esp_err_t ret = esp_ble_gap_set_scan_params(&scan_params);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "❌ Failed to set scan parameters: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_ble_gap_start_scanning(30);  // Scan for 30 seconds
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "❌ Failed to start scanning: %s", esp_err_to_name(ret));
    }
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
    vTaskDelay(pdMS_TO_TICKS(30000));  // Wait 30 seconds (scan duration)
    
    if (is_scanning && !is_connecting) {
        LOG_WARN(TAG, "⏰ Scan timeout - stopping scan");
        esp_ble_gap_stop_scanning();
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