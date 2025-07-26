#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/ledc.h"
#include "driver/rmt.h"
#include "logging_config.h"
#include "gpio_control.h"
#include "bluetooth.h"

static const char *TAG = "GPIO";

// ECU connection status
static bool ecu_connected = false;

// LED burst control
static bool led_burst_active = false;
static volatile bool button_pressed = false;

// NOS system variables
static bool auto_injection_mode = false;
static bool nos_conditions_met = false;
static nvs_handle_t nvs_storage_handle;

// RGB LED control (RMT-based WS2812)
static bool rgb_led_initialized = false;
#define WS2812_T0H_NS (350)
#define WS2812_T0L_NS (1000)
#define WS2812_T1H_NS (1000)
#define WS2812_T1L_NS (350)
#define WS2812_RESET_US (280)

// Buzzer control
static bool buzzer_initialized = false;

// Forward declarations
static void button_isr_handler(void* arg);
static void save_auto_injection_mode(bool mode);
static bool load_auto_injection_mode(void);
static void init_rgb_led(void);
static void init_buzzer(void);

// Initialize GPIO system
void gpio_init_system(void) {
    LOG_VERBOSE(TAG, "Initializing GPIO system for ESP32-S3 Relay Board...");
    
    // Configure relay output GPIO pins
    gpio_config_t relay_conf = {};
    relay_conf.intr_type = GPIO_INTR_DISABLE;
    relay_conf.mode = GPIO_MODE_OUTPUT;
    relay_conf.pin_bit_mask = (1ULL << RELAY_1_PIN) | 
                             (1ULL << RELAY_2_PIN) | 
                             (1ULL << RELAY_3_PIN) | 
                             (1ULL << RELAY_4_PIN) | 
                             (1ULL << RELAY_5_PIN) | 
                             (1ULL << RELAY_6_PIN);
    relay_conf.pull_down_en = 0;
    relay_conf.pull_up_en = 0;
    
    esp_err_t ret = gpio_config(&relay_conf);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "Failed to initialize relay GPIO: %s", esp_err_to_name(ret));
        return;
    }
    
    // Configure button input pin
    gpio_config_t btn_conf = {};
    btn_conf.intr_type = GPIO_INTR_NEGEDGE;  // Trigger on falling edge (button press)
    btn_conf.mode = GPIO_MODE_INPUT;
    btn_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    btn_conf.pull_down_en = 0;
    btn_conf.pull_up_en = 1;  // Enable pull-up for button
    
    ret = gpio_config(&btn_conf);
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
    
    // Initialize all relays to OFF (inactive)
    gpio_set_level(RELAY_1_PIN, 0);
    gpio_set_level(RELAY_2_PIN, 0);
    gpio_set_level(RELAY_3_PIN, 0);
    gpio_set_level(RELAY_4_PIN, 0);
    gpio_set_level(RELAY_5_PIN, 0);
    gpio_set_level(RELAY_6_PIN, 0);
    
    // Initialize RGB LED
    init_rgb_led();
    
    // Initialize buzzer
    init_buzzer();
    
    LOG_VERBOSE(TAG, "GPIO initialized - Relays: %d,%d,%d,%d,%d,%d, RGB LED: %d, Buzzer: %d, Button: %d", 
               RELAY_1_PIN, RELAY_2_PIN, RELAY_3_PIN, RELAY_4_PIN, RELAY_5_PIN, RELAY_6_PIN,
               WS2812_RGB_LED_PIN, BUZZER_PIN, BUTTON_PIN);
    
    // Initialize NVS for persistent storage
    esp_err_t nvs_ret = nvs_open("gpio_storage", NVS_READWRITE, &nvs_storage_handle);
    if (nvs_ret != ESP_OK) {
        LOG_ERROR(TAG, "Failed to open NVS handle: %s", esp_err_to_name(nvs_ret));
    } else {
        // Load auto injection mode from storage
        auto_injection_mode = load_auto_injection_mode();
        LOG_INFO(TAG, "Auto injection mode loaded: %s", auto_injection_mode ? "ENABLED" : "DISABLED");
    }
    
    // Play startup sound
    play_startup_sound();
}

// Initialize RGB LED
static void init_rgb_led(void) {
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(WS2812_RGB_LED_PIN, RMT_CHANNEL_0);
    config.clk_div = 2;
    
    esp_err_t ret = rmt_config(&config);
    if (ret == ESP_OK) {
        ret = rmt_driver_install(config.channel, 0, 0);
    }
    
    if (ret == ESP_OK) {
        rgb_led_initialized = true;
        LOG_VERBOSE(TAG, "RGB LED initialized on GPIO %d using RMT", WS2812_RGB_LED_PIN);
        // Set initial color (off)
        set_rgb_led(0, 0, 0);
    } else {
        LOG_ERROR(TAG, "Failed to initialize RGB LED: %s", esp_err_to_name(ret));
    }
}

// Initialize buzzer
static void init_buzzer(void) {
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 1000,  // Default frequency
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return;
    }
    
    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,  // Start with buzzer off
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0,
    };
    
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
        return;
    }
    
    buzzer_initialized = true;
    LOG_VERBOSE(TAG, "Buzzer initialized on GPIO %d", BUZZER_PIN);
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

// Trigger LED burst (3 short flashes) OR toggle auto injection mode
void trigger_led_burst(void) {
    led_burst_active = true;
    
    // Toggle auto injection mode
    toggle_auto_injection_mode();
    
    // Perform 3 short flashes to indicate mode change
    LOG_DEBUG(TAG, "LED burst triggered - Auto injection mode: %s", 
              auto_injection_mode ? "ENABLED" : "DISABLED");
    
    for (int i = 0; i < 3; i++) {
        set_rgb_led(255, 255, 0); // Yellow for mode toggle (red + green)
        vTaskDelay(pdMS_TO_TICKS(100));  // 100ms on
        set_rgb_led(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(100));  // 100ms off
    }
    
    led_burst_active = false;
    LOG_DEBUG(TAG, "LED burst complete");
}

// NOS readiness output control (using relay!)
void set_nos_ready(bool ready) {
    gpio_set_level(NOS_READY_RELAY, ready ? 1 : 0);
    LOG_DEBUG(TAG, "NOS Ready Relay: %s", ready ? "ACTIVATED (ready)" : "DEACTIVATED (not ready)");
}

// NOS auto injection control (using relay!)
void set_nos_auto_injection(bool active) {
    gpio_set_level(NOS_AUTO_INJ_RELAY, active ? 1 : 0);
    LOG_DEBUG(TAG, "NOS Auto Injection Relay: %s", active ? "ACTIVATED (injecting)" : "DEACTIVATED (off)");
}

// NVS storage functions
static void save_auto_injection_mode(bool mode) {
    esp_err_t err = nvs_set_u8(nvs_storage_handle, "auto_inj_mode", mode ? 1 : 0);
    if (err == ESP_OK) {
        err = nvs_commit(nvs_storage_handle);
        if (err == ESP_OK) {
            LOG_DEBUG(TAG, "Auto injection mode saved: %s", mode ? "ENABLED" : "DISABLED");
        } else {
            LOG_ERROR(TAG, "Failed to commit auto injection mode: %s", esp_err_to_name(err));
        }
    } else {
        LOG_ERROR(TAG, "Failed to save auto injection mode: %s", esp_err_to_name(err));
    }
}

static bool load_auto_injection_mode(void) {
    uint8_t mode = 0;
    esp_err_t err = nvs_get_u8(nvs_storage_handle, "auto_inj_mode", &mode);
    
    if (err == ESP_OK) {
        LOG_DEBUG(TAG, "Auto injection mode loaded from NVS: %s", mode ? "ENABLED" : "DISABLED");
        return mode != 0;
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        LOG_DEBUG(TAG, "Auto injection mode not found in NVS, defaulting to DISABLED");
        return false;
    } else {
        LOG_ERROR(TAG, "Failed to load auto injection mode: %s", esp_err_to_name(err));
        return false;
    }
}

// Public functions for NOS system
void update_nos_system(uint32_t rpm, uint8_t throttle, uint8_t speed) {
    // Only update when everything is connected
    if (!is_connected || !ecu_connected) {
        // Reset outputs when disconnected
        set_nos_ready(false);
        if (auto_injection_mode) {
            set_nos_auto_injection(false);
        }
        nos_conditions_met = false;
        return;
    }
    
    // Check NOS conditions: Speed >20mph, RPM >3000, Throttle >40%
    bool conditions_met = (speed > 20) && (rpm > 3000) && (throttle > 40);
    
    // Update readiness pin (always active when conditions are met)
    set_nos_ready(conditions_met);
    
    // Update auto injection pin (only in auto mode when conditions are met)
    if (auto_injection_mode) {
        set_nos_auto_injection(conditions_met);
    } else {
        set_nos_auto_injection(false);  // Ensure it's off when not in auto mode
    }
    
    // Log state changes
    if (conditions_met != nos_conditions_met) {
        nos_conditions_met = conditions_met;
        LOG_INFO(TAG, "NOS conditions %s (Speed:%d, RPM:%lu, Throttle:%d%%)", 
                conditions_met ? "MET" : "NOT MET", speed, rpm, throttle);
        
        if (auto_injection_mode && conditions_met) {
            LOG_INFO(TAG, "🚀 AUTO INJECTION ACTIVATED!");
        }
    }
}

bool get_auto_injection_mode(void) {
    return auto_injection_mode;
}

void toggle_auto_injection_mode(void) {
    auto_injection_mode = !auto_injection_mode;
    save_auto_injection_mode(auto_injection_mode);
    
    LOG_INFO(TAG, "Auto injection mode %s", auto_injection_mode ? "ENABLED" : "DISABLED");
    
    // If disabled, turn off auto injection output immediately
    if (!auto_injection_mode) {
        set_nos_auto_injection(false);
    }
}

// Bluetooth LED task - handles automatic LED control based on bluetooth and ECU status
void bluetooth_led_task(void *pv) {
    LOG_VERBOSE(TAG, "Bluetooth LED task started");
    
    while (1) {
        // Check for button press
        if (button_pressed) {
            button_pressed = false;  // Clear the flag
            trigger_led_burst();    // Execute LED burst and toggle mode
        }
        
        // Skip automatic LED control during burst
        if (led_burst_active) {
            vTaskDelay(pdMS_TO_TICKS(50));  // Short delay during burst
            continue;
        }
        
        if (is_connected && ecu_connected) {
            // Both BT and ECU connected - SOLID GREEN
            if (auto_injection_mode) {
                // Auto injection mode: Solid green interrupted by 200ms OFF every second
                set_rgb_led(0, 255, 0);  // Green solid
                vTaskDelay(pdMS_TO_TICKS(800));   // On for 800ms
                set_rgb_led(0, 0, 0);    // Off for 200ms interrupt
                vTaskDelay(pdMS_TO_TICKS(200));   
            } else {
                // Normal mode: Solid green
                set_rgb_led(0, 255, 0);  // Solid green
                vTaskDelay(pdMS_TO_TICKS(1000));  // Check every second
            }
            
        } else if (is_connected && !ecu_connected) {
            // BT connected, ECU connecting - BLINK GREEN
            set_rgb_led(0, 255, 0);  // Green blink for ECU connecting
            vTaskDelay(pdMS_TO_TICKS(500));   // On for 500ms
            set_rgb_led(0, 0, 0);    // Off
            vTaskDelay(pdMS_TO_TICKS(500));   // Off for 500ms (1s cycle)
            
        } else if (is_scanning || is_connecting) {
            // BT connecting/scanning - BLINK BLUE
            set_rgb_led(0, 0, 255);  // Blue blink for BT connecting
            vTaskDelay(pdMS_TO_TICKS(500));   // On for 500ms
            set_rgb_led(0, 0, 0);    // Off
            vTaskDelay(pdMS_TO_TICKS(500));   // Off for 500ms (1s cycle)
            
        } else {
            // OFF - nothing connected
            set_rgb_led(0, 0, 0);    // LED off
            vTaskDelay(pdMS_TO_TICKS(500));   // Check every 500ms
        }
    }
} 

// RGB LED control function
void set_rgb_led(uint8_t red, uint8_t green, uint8_t blue) {
    if (!rgb_led_initialized) {
        LOG_DEBUG(TAG, "RGB LED not initialized");
        return;
    }
    
    // WS2812 expects GRB format, but swapping red/green to fix color issue
    uint32_t color = (red << 16) | (green << 8) | blue;
    
    rmt_item32_t data[24];  // 24 bits for one LED (8 bits each for G, R, B)
    
    for (int i = 0; i < 24; i++) {
        uint32_t bit = (color >> (23 - i)) & 1;
        if (bit) {
            // Send '1' bit
            data[i].level0 = 1;
            data[i].duration0 = 40; // ~800ns at 80MHz / 2
            data[i].level1 = 0;
            data[i].duration1 = 17; // ~350ns
        } else {
            // Send '0' bit
            data[i].level0 = 1;
            data[i].duration0 = 17; // ~350ns
            data[i].level1 = 0;
            data[i].duration1 = 40; // ~800ns
        }
    }
    
    rmt_write_items(RMT_CHANNEL_0, data, 24, true);
}

// Buzzer control function
void play_beep(uint16_t frequency, uint16_t duration_ms) {
    if (!buzzer_initialized) {
        LOG_DEBUG(TAG, "Buzzer not initialized");
        return;
    }
    
    // Set frequency
    ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, frequency);
    
    // Set duty cycle to 50% (4096 out of 8192 for 13-bit resolution)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 4096);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    
    // Play for specified duration
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    
    // Turn off buzzer
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// Sound effects
void play_startup_sound(void) {
    LOG_INFO(TAG, "🔊 Playing startup sound");
    set_rgb_led(0, 255, 0);  // Green
    play_beep(800, 200);
    play_beep(1000, 200);
    play_beep(1200, 300);
    set_rgb_led(0, 0, 0);    // Off
}

void play_connection_sound(void) {
    LOG_INFO(TAG, "🔊 Playing connection sound");
    set_rgb_led(0, 0, 255);  // Blue
    play_beep(1000, 100);
    play_beep(1200, 100);
    set_rgb_led(0, 0, 0);    // Off
}

void play_error_sound(void) {
    LOG_ERROR(TAG, "🔊 Playing error sound");
    set_rgb_led(255, 0, 0);  // Red
    play_beep(400, 200);
    play_beep(300, 200);
    play_beep(200, 300);
    set_rgb_led(0, 0, 0);    // Off
} 