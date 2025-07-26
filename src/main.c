#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"

// Module includes
#include "logging_config.h"
#include "bluetooth.h"
#include "elm327.h"
#include "obd_data.h"
#include "gpio_control.h"
#include "bt_test_stub.h"

// Enable this define to build a minimal firmware that only scans for the
// VEEPEAK adapter (no OBD, relays, etc.)
// FIXED: Now properly initializes ELM327 system to prevent semaphore crashes
// NOTE: Disable for full app testing with OBD data collection and relay control
#define BT_TEST_MODE 1

#if BT_TEST_MODE

static const char *TAG = "OBD_CONTROLLER";

// Application main entry point
void app_main(void) {
    // Wait for system to fully boot and logging to be ready
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Force this log to appear first
    printf("*** MAIN.C: BT_TEST_MODE PATH SELECTED ***\n");
    ESP_LOGI(TAG, "*** BT_TEST_MODE: Calling test stub ***");
    
    // Run the isolated BLE connection test and then idle forever
    run_bt_connection_test();

    // Idle loop – all work happens in BT tasks spawned by bluetooth_init()
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#else

static const char *TAG = "OBD_CONTROLLER";

// Application main entry point
void app_main(void) {
    // Force this log to appear first
    printf("*** MAIN.C: FULL APPLICATION PATH SELECTED ***\n");
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
    LOG_INFO(TAG, "Logging configured");
    
    // Configure task watchdog for BLE debugging
    LOG_INFO(TAG, "Configuring watchdog for BLE debugging...");
    esp_task_wdt_config_t watchdog_config = {
        .timeout_ms = 15000,        // 15 second timeout (longer for debugging)
        .idle_core_mask = 0,        // Don't monitor idle tasks  
        .trigger_panic = false      // Don't panic on timeout, just log
    };
    esp_err_t wdt_ret = esp_task_wdt_reconfigure(&watchdog_config);
    if (wdt_ret == ESP_OK) {
        LOG_INFO(TAG, "✅ Watchdog reconfigured: 15s timeout, no panic");
    } else {
        LOG_WARN(TAG, "⚠️  Watchdog reconfigure failed: %s", esp_err_to_name(wdt_ret));
    }
    
    // Initialize all modules
    LOG_INFO(TAG, "Initializing system modules...");
    
    // Initialize GPIO and LED system
    LOG_INFO(TAG, "Initializing GPIO...");
    gpio_init_system();
    LOG_INFO(TAG, "GPIO initialized");
    
    // Initialize ELM327 system (semaphores, buffers)
    LOG_INFO(TAG, "Initializing ELM327...");
    elm327_init_system();
    LOG_INFO(TAG, "ELM327 initialized");
    
    // Initialize OBD data system
    LOG_INFO(TAG, "Initializing OBD data...");
    obd_data_init();
    LOG_INFO(TAG, "OBD data initialized");
    
    // Initialize Bluetooth system
    LOG_INFO(TAG, "Initializing Bluetooth...");
    bluetooth_init();
    LOG_INFO(TAG, "Bluetooth initialized");
    
    // Create Bluetooth LED indicator task (after bluetooth init)
    LOG_INFO(TAG, "Creating Bluetooth LED task...");
    xTaskCreate(bluetooth_led_task, "bt_led", 2048, NULL, 4, NULL);
    LOG_INFO(TAG, "Bluetooth LED task created");
    
    // Log target ELM327 device
    LOG_INFO(TAG, "Looking for ELM327 device: [01:23:45:67:89:BA]");
    
    // Start device discovery immediately
    LOG_INFO(TAG, "Starting device search...");
    vTaskDelay(pdMS_TO_TICKS(100));  // Brief delay for system stability
    start_ble_scan();
    LOG_INFO(TAG, "Device search started");
    
    // Create OBD data polling task
    LOG_INFO(TAG, "Creating OBD task...");
    xTaskCreate(obd_task, "obd_task", 4096, NULL, 5, NULL);
    LOG_INFO(TAG, "OBD task created");
    
    LOG_INFO(TAG, "System initialization complete. Searching for ELM327...");
    
    // Main task complete - FreeRTOS scheduler handles everything from here
}

#endif
