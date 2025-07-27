#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ESP32-S3 Industrial Relay Board GPIO definitions
#define WS2812_RGB_LED_PIN  GPIO_NUM_38  // WS2812 RGB LED (onboard)
#define BUZZER_PIN          GPIO_NUM_21  // Passive buzzer (onboard)
#define BUTTON_PIN          GPIO_NUM_0   // Button input (BOOT button)

// Relay control pins (verified for ESP32-S3-Relay-6CH board)
#define RELAY_1_PIN         GPIO_NUM_1   // Relay CH1 control
#define RELAY_2_PIN         GPIO_NUM_2   // Relay CH2 control  
#define RELAY_3_PIN         GPIO_NUM_41  // Relay CH3 control
#define RELAY_4_PIN         GPIO_NUM_42  // Relay CH4 control
#define RELAY_5_PIN         GPIO_NUM_45  // Relay CH5 control
#define RELAY_6_PIN         GPIO_NUM_46  // Relay CH6 control

// NOS system relay assignments (using actual relays for better isolation)
#define NOS_READY_RELAY     RELAY_1_PIN  // NOS readiness relay (10A capability!)
#define NOS_AUTO_INJ_RELAY  RELAY_2_PIN  // NOS auto injection relay (10A capability!)

// LED indicator relay assignments
#define BT_STATUS_LED_RELAY    RELAY_3_PIN  // Bluetooth status LED relay
#define ECU_STATUS_LED_RELAY   RELAY_4_PIN  // ECU status LED relay  
#define NOS_STATUS_LED_RELAY   RELAY_5_PIN  // NOS readiness LED relay

// Function declarations
void gpio_init_system(void);
void bluetooth_led_task(void *pv);
void led_indicator_task(void *pv);

// Status update functions (for internal use by automatic LED control)
void set_ecu_status(bool connected);

// Button functions
void trigger_led_burst(void);

// NOS system functions
void update_nos_system(uint32_t rpm, uint8_t throttle, uint8_t speed);
bool get_auto_injection_mode(void);
void toggle_auto_injection_mode(void);

// NOS control outputs (now using relays!)
void set_nos_ready(bool ready);
void set_nos_auto_injection(bool active);

// Enhanced features for relay board
void set_rgb_led(uint8_t red, uint8_t green, uint8_t blue);
void play_beep(uint16_t frequency, uint16_t duration_ms);
void play_startup_sound(void);
void play_connection_sound(void);
void play_error_sound(void);

#endif // GPIO_CONTROL_H 