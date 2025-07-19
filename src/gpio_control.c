#include "esp_log.h"

#include "logging_config.h"
#include "gpio_control.h"
#include "bluetooth.h"

static const char *TAG = "GPIO";

// GPIO state
bool gpio_status = false;

// Initialize GPIO system
void gpio_init_system(void) {
    LOG_VERBOSE(TAG, "Initializing GPIO...");
    
    // Configure LED pin as output
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret == ESP_OK) {
        // Start with LED off
        gpio_set_level(LED_PIN, 0);
        gpio_status = false;
        LOG_VERBOSE(TAG, "GPIO initialized (LED on pin %d)", LED_PIN);
    } else {
        LOG_ERROR(TAG, "Failed to initialize GPIO: %s", esp_err_to_name(ret));
    }
}

// Turn LED on
void led_on(void) {
    gpio_set_level(LED_PIN, 1);
    gpio_status = true;
}

// Turn LED off
void led_off(void) {
    gpio_set_level(LED_PIN, 0);
    gpio_status = false;
}

// Pulse LED for specified duration
void led_pulse(int duration_ms) {
    led_on();
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    led_off();
}

// Set LED state for searching mode
void led_set_searching(bool searching) {
    if (searching) {
        ESP_LOGD(TAG, "🔄 LED search mode enabled");
        // The led_search_task will handle the pulsing
    } else {
        ESP_LOGD(TAG, "🔄 LED search mode disabled");
        led_off();
    }
}

// Set LED state for connected mode
void led_set_connected(bool connected) {
    if (connected) {
        LOG_DEBUG(TAG, "LED: Connected (solid ON)");
        led_on();
    } else {
        LOG_DEBUG(TAG, "LED: Disconnected (OFF)");
        led_off();
    }
}

// LED search indicator task
void led_search_task(void *pv) {
    LOG_VERBOSE(TAG, "LED search indicator task started");
    
    while (1) {
        // Only pulse LED when searching and not connected
        if (is_searching && !is_connected && !is_connecting) {
            // Pulse LED to indicate searching
            led_on();
            vTaskDelay(pdMS_TO_TICKS(200));  // On for 200ms
            led_off();
            vTaskDelay(pdMS_TO_TICKS(800));  // Off for 800ms (1 second total cycle)
        } else if (is_connected) {
            // Keep LED solid on when connected
            if (!gpio_status) {
                led_on();
            }
            vTaskDelay(pdMS_TO_TICKS(1000));  // Check every second
        } else {
            // Turn off LED when not searching or connecting
            if (gpio_status) {
                led_off();
            }
            vTaskDelay(pdMS_TO_TICKS(500));  // Check every 500ms
        }
    }
} 