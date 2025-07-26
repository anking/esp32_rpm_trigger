#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "logging_config.h"
#include "bluetooth.h"
#include "bt_test_stub.h"
#include "elm327.h"           // FIXED: Added for elm327_init_system()
#include "gpio_control.h"     // FIXED: Added for gpio_init_system()

static const char *TAG = "BT_TEST";

void run_bt_connection_test(void) {
    // Wait for logging system to be fully ready
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Force this log to appear first with basic printf
    printf("*** TEST STUB CALLED - BT_TEST_MODE IS ACTIVE ***\n");
    ESP_LOGI(TAG, "🚀 Starting VEEPEAK BLE connection test stub");
    ESP_LOGI(TAG, "TEST STUB: Step 1 - Function called successfully");

    // Initialise NVS (required before BT)
    ESP_LOGI(TAG, "TEST STUB: Step 2 - Initializing NVS");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Configure stack logging (use same helper as full app)
    configure_esp_bt_logging();

    // FIXED: Initialize ELM327 system to create semaphores (prevents NULL semaphore crash)
    ESP_LOGI(TAG, "TEST STUB: Step 3 - Initializing ELM327 system");
    elm327_init_system();
    ESP_LOGI(TAG, "TEST STUB: ELM327 system initialized - semaphores created");

    // FIXED: Initialize GPIO system to prevent crashes in set_ecu_status()
    ESP_LOGI(TAG, "TEST STUB: Step 4 - Initializing GPIO system");
    gpio_init_system();
    ESP_LOGI(TAG, "TEST STUB: GPIO system initialized");

    // Bring up Bluetooth controller / host and begin scanning
    ESP_LOGI(TAG, "TEST STUB: Step 5 - Calling bluetooth_init()");
    bluetooth_init();

    ESP_LOGI(TAG, "TEST STUB: Step 6 - bluetooth_init() includes scanning, no need to call start_ble_scan() again");
    
    ESP_LOGI(TAG, "TEST STUB: Step 7 - Test stub completed, monitoring for VEEPEAK...");
} 