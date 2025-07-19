#include "logging_config.h"
#include "esp_log.h"

// Configure ESP-IDF Bluetooth stack logging levels
void configure_esp_bt_logging(void) {
#if !ENABLE_ESP_BT_LOGS
    // Suppress noisy ESP-IDF Bluetooth stack logs
    esp_log_level_set("BT_RFCOMM", ESP_LOG_WARN);      // PORT_WriteDataCO, etc.
    esp_log_level_set("BT_L2CAP", ESP_LOG_WARN);       // L2CA_DataWrite, etc.
    esp_log_level_set("BT_BTM", ESP_LOG_WARN);         // BTM security logs
    esp_log_level_set("BT_HCI", ESP_LOG_WARN);         // HCI connection logs
    esp_log_level_set("BT_APPL", ESP_LOG_WARN);        // BT application logs
    esp_log_level_set("BTDM_INIT", ESP_LOG_INFO);      // Keep init logs but reduce noise
    esp_log_level_set("BT_SMP", ESP_LOG_WARN);         // Security Manager Protocol
    esp_log_level_set("BT_GAP", ESP_LOG_WARN);         // Generic Access Profile
    esp_log_level_set("BT_SDP", ESP_LOG_WARN);         // Service Discovery Protocol
    esp_log_level_set("BT_SPP", ESP_LOG_WARN);         // Serial Port Profile (keep warnings)
    
    ESP_LOGI("LOGGING", "ESP-IDF Bluetooth stack logs suppressed (set to WARN level)");
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