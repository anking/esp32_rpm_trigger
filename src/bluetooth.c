#include "esp_log.h"
#include "esp_bt_device.h"
#include "nvs_flash.h"
#include <string.h>

#include "logging_config.h"
#include "bluetooth.h"
#include "elm327.h"
#include "gpio_control.h"

static const char *TAG = "BLUETOOTH";

// Global Bluetooth state variables
bool is_connected = false;
bool is_connecting = false;
bool is_searching = false;
uint32_t spp_handle = 0;
uint8_t target_elm327_bda[6] = ELM327_BT_ADDR;
static int connection_attempt = 0;

// GAP callback for device discovery
static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT: {
            char addr_str[18];
            snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                    param->disc_res.bda[0], param->disc_res.bda[1], param->disc_res.bda[2],
                    param->disc_res.bda[3], param->disc_res.bda[4], param->disc_res.bda[5]);
            
            // Check if this is our target ELM327 device
            if (memcmp(param->disc_res.bda, target_elm327_bda, 6) == 0) {
                if (is_connecting) {
                    ESP_LOGD(TAG, "🎯 Already connecting to ELM327, ignoring duplicate discovery");
                    break;
                }
                
                LOG_INFO(TAG, "Found ELM327: %s", addr_str);
                LOG_BT(TAG, "Attempting connection to ELM327...");
                
                is_connecting = true;  // Mark as connecting to prevent multiple attempts
                esp_bt_gap_cancel_discovery();
                
                // Give ELM327 time to be ready for connection
                LOG_BT(TAG, "Waiting 2 seconds before connection attempt...");
                vTaskDelay(pdMS_TO_TICKS(2000));
                
                // Connect directly to SCN 2 (discovered working channel)
                LOG_BT(TAG, "Connecting to ELM327 on SCN 2 (verified working channel)...");
                esp_err_t ret = esp_spp_connect(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_MASTER, 2, param->disc_res.bda);
                if (ret != ESP_OK) {
                    LOG_WARN(TAG, "SCN 2 failed (%s), trying SCN 1 fallback...", esp_err_to_name(ret));
                    vTaskDelay(pdMS_TO_TICKS(1000));  // Brief wait
                    ret = esp_spp_connect(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_MASTER, 1, param->disc_res.bda);
                    if (ret != ESP_OK) {
                        LOG_ERROR(TAG, "Both SCN 2 and SCN 1 failed: %s", esp_err_to_name(ret));
                        is_connecting = false;
                    } else {
                        LOG_INFO(TAG, "SCN 1 fallback connection initiated");
                    }
                } else {
                    LOG_INFO(TAG, "SCN 2 connection initiated");
                }
            } else {
                ESP_LOGD(TAG, "📱 Found other device: [%s] - skipping", addr_str);
            }
            break;
        }
        
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
                ESP_LOGI(TAG, "🔍 Device discovery stopped");
                if (!is_connecting && !is_connected) {
                    ESP_LOGI(TAG, "⏳ Connection attempt in progress - waiting...");
                }
            }
            break;
            
        default:
            ESP_LOGD(TAG, "GAP event: %d", event);
            break;
    }
}

// SPP callback for connection events and data
static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            LOG_BT(TAG, "SPP initialized");
            break;
            
        case ESP_SPP_START_EVT:
            LOG_BT(TAG, "SPP server started");
            break;
            
        case ESP_SPP_CL_INIT_EVT:
            LOG_BT(TAG, "SPP client initiated");
            break;
            
        case ESP_SPP_OPEN_EVT:
            LOG_INFO(TAG, "RFCOMM connection established");
            is_connected = true;
            is_connecting = false;
            is_searching = false;
            
            if (param) {
                spp_handle = param->open.handle;
            }
            led_set_connected(true);  // Turn on LED solid
            
            // Create task for delayed ELM327 initialization to prevent immediate disconnection
            LOG_VERBOSE(TAG, "Scheduling ELM327 initialization...");
            xTaskCreate(initialize_elm327_task, "elm327_init", 4096, NULL, 5, NULL);
            break;
            
        case ESP_SPP_CLOSE_EVT:
            LOG_WARN(TAG, "Bluetooth connection closed");
            is_connecting = false;   // Reset connection attempt state
            is_connected = false;    // No longer connected
            elm327_initialized = false;
            led_set_connected(false);  // Turn off LED
            
            handle_connection_failure();
            break;
            
        case ESP_SPP_DATA_IND_EVT:
            if (param && param->data_ind.data && param->data_ind.len > 0) {
                LOG_DEBUG(TAG, "Data received: %.*s", param->data_ind.len, param->data_ind.data);
                process_received_data((const char *)param->data_ind.data, param->data_ind.len);
            }
            break;
            
        case ESP_SPP_CONG_EVT:
            ESP_LOGW(TAG, "⚠️  SPP congestion occurred");
            break;
            
        default:
            ESP_LOGD(TAG, "SPP event: %d", event);
            break;
    }
}

// Handle connection failures with simple retry logic
void handle_connection_failure(void) {
    // Simple retry: SCN 2 first, then SCN 1, then restart discovery
    connection_attempt++;
    ESP_LOGI(TAG, "🔄 Connection failed - retry attempt #%d...", connection_attempt);
    
    vTaskDelay(pdMS_TO_TICKS(2000));  // Wait before retry
    
    if (connection_attempt % 3 == 1) {
        // Try SCN 2 (known working channel)
        ESP_LOGI(TAG, "📡 Retry: SCN 2 (primary channel)...");
        if (esp_spp_connect(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_MASTER, 2, target_elm327_bda) == ESP_OK) {
            is_connecting = true;
            ESP_LOGI(TAG, "✅ SCN 2 retry connection initiated");
            return;
        }
    } else if (connection_attempt % 3 == 2) {
        // Try SCN 1 (fallback)
        ESP_LOGI(TAG, "📡 Retry: SCN 1 (fallback channel)...");
        if (esp_spp_connect(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_MASTER, 1, target_elm327_bda) == ESP_OK) {
            is_connecting = true;
            ESP_LOGI(TAG, "✅ SCN 1 retry connection initiated");
            return;
        }
    } else {
        // Reset counter and restart discovery
        ESP_LOGI(TAG, "🔄 Retries exhausted, restarting discovery...");
        connection_attempt = 0;
    }
    
    // Restart discovery
    ESP_LOGI(TAG, "🔍 Restarting device discovery...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    start_device_discovery();
}

// Start device discovery
void start_device_discovery(void) {
    if (is_connecting || is_connected) {
        ESP_LOGD(TAG, "🔗 Already connecting/connected, skipping discovery");
        return;
    }
    
    is_searching = true;
    ESP_LOGI(TAG, "🔍 Starting device discovery for ELM327...");
    esp_err_t ret = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Device discovery started successfully");
        led_set_searching(true);  // Start LED search indicator
        ESP_LOGI(TAG, "🔍 Device discovery started - looking for ELM327...");
    } else {
        ESP_LOGE(TAG, "❌ Failed to start device discovery: %s", esp_err_to_name(ret));
        is_searching = false;
    }
}

// Initialize Bluetooth system
void bluetooth_init(void) {
    LOG_INFO(TAG, "Starting Bluetooth initialization...");
    
    // Release BLE memory since we only use Classic BT
    LOG_VERBOSE(TAG, "Releasing BLE memory (using Classic BT only)...");
    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "Failed to release BLE memory: %s", esp_err_to_name(ret));
        return;
    }
    
    // Initialize BT controller in Classic mode only
    LOG_VERBOSE(TAG, "Initializing BT controller (Classic mode)...");
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.mode = ESP_BT_MODE_CLASSIC_BT;  // Classic Bluetooth only
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Enable BT controller
    LOG_VERBOSE(TAG, "Enabling BT controller (Classic Bluetooth only)...");
    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Initialize Bluedroid
    LOG_VERBOSE(TAG, "Initializing Bluedroid...");
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Enable Bluedroid
    LOG_VERBOSE(TAG, "Enabling Bluedroid...");
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Log free heap before SPP init
    LOG_DEBUG(TAG, "Free heap before SPP: %lu bytes", esp_get_free_heap_size());
    
    // Initialize SPP (Serial Port Profile)
    LOG_VERBOSE(TAG, "Initializing SPP (Serial Port Profile)...");
    ret = esp_spp_init(ESP_SPP_MODE_CB);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "SPP init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Register SPP callback
    LOG_VERBOSE(TAG, "Registering SPP callback...");
    ret = esp_spp_register_callback(spp_callback);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "SPP callback register failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Register GAP callback for device discovery
    LOG_VERBOSE(TAG, "Registering GAP callback...");
    ret = esp_bt_gap_register_callback(gap_callback);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "GAP callback register failed: %s", esp_err_to_name(ret));
        return;
    }
    
    LOG_INFO(TAG, "Bluetooth initialization complete!");
} 