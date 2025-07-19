#ifndef ELM327_H
#define ELM327_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ELM327 state
extern bool elm327_initialized;
extern SemaphoreHandle_t connection_semaphore;

// RX Buffer for ELM327 responses
#define RX_BUFFER_SIZE 256
extern char rx_buffer[RX_BUFFER_SIZE];
extern uint16_t rx_buffer_len;

// Function declarations
void elm327_init_system(void);
void send_obd_command(const char *cmd);
void initialize_elm327(void);
void initialize_elm327_task(void *pv);
void process_received_data(const char *data, uint16_t len);

// ELM327 communication
esp_err_t elm327_send_command(const char *cmd);
void elm327_handle_response(const char *response);

#endif // ELM327_H 