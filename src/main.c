#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

// Module includes
#include "logging_config.h"
#include "bluetooth.h"
#include "elm327.h"
#include "obd_data.h"
#include "gpio_control.h"

static const char *TAG = "OBD_CONTROLLER";

// Application main entry point
void app_main(void) {
    LOG_INFO(TAG, "ESP32 OBD-II Controller Starting");
    
    // Initialize NVS (required for Bluetooth)
    LOG_VERBOSE(TAG, "Initializing NVS flash...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    LOG_INFO(TAG, "NVS initialized");
    
    // Configure ESP-IDF logging levels (suppress noisy BT stack logs)
    configure_esp_bt_logging();
    
    // Initialize all modules
    LOG_VERBOSE(TAG, "Initializing system modules...");
    
    // Initialize GPIO and LED system
    gpio_init_system();
    
    // Initialize ELM327 system (semaphores, buffers)
    elm327_init_system();
    
    // Initialize OBD data system
    obd_data_init();
    
    // Initialize Bluetooth system
    bluetooth_init();
    
    // Log target ELM327 device
    LOG_INFO(TAG, "Looking for ELM327 device: [01:23:45:67:89:BA]");
    
    // Start device discovery immediately
    LOG_VERBOSE(TAG, "Starting device search...");
    vTaskDelay(pdMS_TO_TICKS(100));  // Brief delay for system stability
    start_device_discovery();
    
    // Create LED search indicator task
    LOG_VERBOSE(TAG, "Creating LED search task...");
    xTaskCreate(led_search_task, "led_search", 2048, NULL, 4, NULL);
    
    // Create OBD data polling task
    LOG_VERBOSE(TAG, "Creating OBD task...");
    xTaskCreate(obd_task, "obd_task", 4096, NULL, 5, NULL);
    
    LOG_INFO(TAG, "System initialization complete. Searching for ELM327...");
    
    // Main task complete - FreeRTOS scheduler handles everything from here
}
