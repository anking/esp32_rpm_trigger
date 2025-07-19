#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// GPIO pin definitions
#define LED_PIN GPIO_NUM_2

// GPIO state
extern bool gpio_status;

// Function declarations
void gpio_init_system(void);
void led_search_task(void *pv);

// LED control functions
void led_set_searching(bool searching);
void led_set_connected(bool connected);
void led_pulse(int duration_ms);
void led_on(void);
void led_off(void);

#endif // GPIO_CONTROL_H 