#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// GPIO pin definitions with specific purposes
#define BT_ECU_LED_PIN      GPIO_NUM_2   // Bluetooth/ECU status LED
#define NOS_READY_PIN       GPIO_NUM_4   // NOS readiness output
#define NOS_AUTO_INJ_PIN    GPIO_NUM_16  // NOS auto injection
#define BUTTON_PIN          GPIO_NUM_0   // Button input (BOOT button)

// Function declarations
void gpio_init_system(void);
void bluetooth_led_task(void *pv);

// Status update functions (for internal use by automatic LED control)
void set_ecu_status(bool connected);

// Button functions
void trigger_led_burst(void);

// NOS control outputs
void set_nos_ready(bool ready);
void set_nos_auto_injection(bool active);

#endif // GPIO_CONTROL_H 