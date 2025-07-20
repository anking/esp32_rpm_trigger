#include "logging_config.h"
#include "esp_log.h"

// Configure ESP-IDF Bluetooth stack logging levels
void configure_esp_bt_logging(void) {
#if !ENABLE_ESP_BT_LOGS
    // Suppress noisy ESP-IDF Bluetooth stack logs completely
    esp_log_level_set("BT_RFCOMM", ESP_LOG_NONE);       // PORT_WriteDataCO, etc. (completely silent)
    esp_log_level_set("BT_L2CAP", ESP_LOG_NONE);        // L2CA_DataWrite, etc. (completely silent)
    esp_log_level_set("BT_BTM", ESP_LOG_NONE);          // BTM security logs (completely silent)
    esp_log_level_set("BT_HCI", ESP_LOG_NONE);          // HCI connection logs (completely silent)
    esp_log_level_set("BT_APPL", ESP_LOG_NONE);         // BT application logs (completely silent)
    esp_log_level_set("BTDM_INIT", ESP_LOG_INFO);       // Keep init logs but reduce noise
    esp_log_level_set("BT_SMP", ESP_LOG_NONE);          // Security Manager Protocol (completely silent)
    esp_log_level_set("BT_GAP", ESP_LOG_NONE);          // Generic Access Profile (completely silent)
    esp_log_level_set("BT_SDP", ESP_LOG_NONE);          // Service Discovery Protocol (completely silent)
    esp_log_level_set("BT_SPP", ESP_LOG_WARN);          // Serial Port Profile (keep warnings only)
    
    ESP_LOGI("LOGGING", "ESP-IDF Bluetooth stack logs completely suppressed (set to NONE level)");
#else
    // Keep all BT logs enabled for debugging
    esp_log_level_set("BT_RFCOMM", ESP_LOG_DEBUG);
    esp_log_level_set("BT_L2CAP", ESP_LOG_DEBUG);
    esp_log_level_set("BT_BTM", ESP_LOG_DEBUG);
    esp_log_level_set("BT_HCI", ESP_LOG_DEBUG);
    esp_log_level_set("BT_APPL", ESP_LOG_DEBUG);
    esp_log_level_set("BTDM_INIT", ESP_LOG_DEBUG);
    esp_log_level_set("BT_SMP", ESP_LOG_DEBUG);
    esp_log_level_set("BT_GAP", ESP_LOG_DEBUG);
    esp_log_level_set("BT_SDP", ESP_LOG_DEBUG);
    esp_log_level_set("BT_SPP", ESP_LOG_DEBUG);
    
    ESP_LOGI("LOGGING", "ESP-IDF Bluetooth stack logs enabled (DEBUG level)");
#endif
} 