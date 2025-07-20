#include "esp_log.h"
#include "logging_config.h"
#include "gpio_control.h"
#include "bluetooth.h"

static const char *TAG = "GPIO";

// ECU connection status
static bool ecu_connected = false;

// LED burst control
static bool led_burst_active = false;
static volatile bool button_pressed = false;

// Forward declarations
static void button_isr_handler(void* arg);

// Initialize GPIO system
void gpio_init_system(void) {
    LOG_VERBOSE(TAG, "Initializing GPIO system...");
    
    // Configure output GPIO pins
    gpio_config_t out_conf = {};
    out_conf.intr_type = GPIO_INTR_DISABLE;
    out_conf.mode = GPIO_MODE_OUTPUT;
    out_conf.pin_bit_mask = (1ULL << BT_ECU_LED_PIN) | 
                           (1ULL << NOS_READY_PIN) | 
                           (1ULL << NOS_AUTO_INJ_PIN);
    out_conf.pull_down_en = 0;
    out_conf.pull_up_en = 0;
    
    esp_err_t ret = gpio_config(&out_conf);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "Failed to initialize output GPIO: %s", esp_err_to_name(ret));
        return;
    }
    
    // Configure button input pin
    gpio_config_t in_conf = {};
    in_conf.intr_type = GPIO_INTR_NEGEDGE;  // Trigger on falling edge (button press)
    in_conf.mode = GPIO_MODE_INPUT;
    in_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    in_conf.pull_down_en = 0;
    in_conf.pull_up_en = 1;  // Enable pull-up for button
    
    ret = gpio_config(&in_conf);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "Failed to initialize button GPIO: %s", esp_err_to_name(ret));
        return;
    }
    
    // Install GPIO ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {  // ESP_ERR_INVALID_STATE means already installed
        LOG_ERROR(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return;
    }
    
    // Add ISR handler for button
    ret = gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "Failed to add button ISR handler: %s", esp_err_to_name(ret));
        return;
    }
    
    // Initialize all output pins to LOW (off/inactive)
    gpio_set_level(BT_ECU_LED_PIN, 0);
    gpio_set_level(NOS_READY_PIN, 0);
    gpio_set_level(NOS_AUTO_INJ_PIN, 0);
    
    LOG_VERBOSE(TAG, "GPIO initialized - BT/ECU LED: %d, NOS Ready: %d, NOS Auto: %d, Button: %d", 
               BT_ECU_LED_PIN, NOS_READY_PIN, NOS_AUTO_INJ_PIN, BUTTON_PIN);
}

// Internal LED control (used only by bluetooth_led_task)
static void set_bt_ecu_led(bool active) {
    gpio_set_level(BT_ECU_LED_PIN, active ? 1 : 0);
    LOG_DEBUG(TAG, "BT/ECU LED: %s", active ? "ON (active)" : "OFF (inactive)");
}

// Set ECU connection status
void set_ecu_status(bool connected) {
    ecu_connected = connected;
    LOG_DEBUG(TAG, "ECU status: %s", connected ? "Connected" : "Disconnected");
}

// Button interrupt handler
static void IRAM_ATTR button_isr_handler(void* arg) {
    button_pressed = true;
}

// Trigger LED burst (3 short flashes)
void trigger_led_burst(void) {
    led_burst_active = true;
    LOG_DEBUG(TAG, "LED burst triggered");
    
    // Perform 3 short flashes
    for (int i = 0; i < 3; i++) {
        set_bt_ecu_led(true);
        vTaskDelay(pdMS_TO_TICKS(100));  // 100ms on
        set_bt_ecu_led(false);
        vTaskDelay(pdMS_TO_TICKS(100));  // 100ms off
    }
    
    led_burst_active = false;
    LOG_DEBUG(TAG, "LED burst complete");
}

// NOS readiness output control
void set_nos_ready(bool ready) {
    gpio_set_level(NOS_READY_PIN, ready ? 1 : 0);
    LOG_DEBUG(TAG, "NOS Ready: %s", ready ? "HIGH (ready)" : "LOW (not ready)");
}

// NOS auto injection control
void set_nos_auto_injection(bool active) {
    gpio_set_level(NOS_AUTO_INJ_PIN, active ? 1 : 0);
    LOG_DEBUG(TAG, "NOS Auto Injection: %s", active ? "HIGH (active)" : "LOW (inactive)");
}

// Bluetooth LED task - handles automatic LED control based on bluetooth and ECU status
void bluetooth_led_task(void *pv) {
    LOG_VERBOSE(TAG, "Bluetooth LED task started");
    
    while (1) {
        // Check for button press
        if (button_pressed) {
            button_pressed = false;  // Clear the flag
            trigger_led_burst();    // Execute LED burst
        }
        
        // Skip automatic LED control during burst
        if (led_burst_active) {
            vTaskDelay(pdMS_TO_TICKS(50));  // Short delay during burst
            continue;
        }
        
        if (is_connected && ecu_connected) {
            // Solid ON - both BT and ECU connected
            set_bt_ecu_led(true);
            vTaskDelay(pdMS_TO_TICKS(1000));  // Check every second
            
        } else if (is_connected && !ecu_connected) {
            // Fast flash - BT connected, ECU connecting/searching
            set_bt_ecu_led(true);
            vTaskDelay(pdMS_TO_TICKS(100));   // On for 100ms
            set_bt_ecu_led(false);
            vTaskDelay(pdMS_TO_TICKS(150));   // Off for 150ms (250ms cycle = fast)
            
        } else if (is_searching || is_connecting) {
            // Slow flash - BT connecting/searching
            set_bt_ecu_led(true);
            vTaskDelay(pdMS_TO_TICKS(300));   // On for 300ms
            set_bt_ecu_led(false);
            vTaskDelay(pdMS_TO_TICKS(700));   // Off for 700ms (1000ms cycle = slow)
            
        } else {
            // OFF - nothing connected
            set_bt_ecu_led(false);
            vTaskDelay(pdMS_TO_TICKS(500));   // Check every 500ms
        }
    }
} 