#ifndef ELM327_H
#define ELM327_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ELM327 state
extern bool elm327_initialized;
extern bool ecu_connected;
extern bool response_received_flag;  // Flag set when any response is received
extern SemaphoreHandle_t connection_semaphore;

// RX Buffer for ELM327 responses
#define RX_BUFFER_SIZE 384   /* multi-PID lines are longer */
extern char rx_buffer[RX_BUFFER_SIZE];
extern uint16_t rx_buffer_len;

// Function declarations
void elm327_init_system(void);
void initialize_elm327(void);
void initialize_elm327_task(void *pv);
void handle_elm327_response(const char *data, size_t len);
void process_received_data(const char *data, uint16_t len);

// ECU connectivity verification
bool test_ecu_connectivity(void);
void verify_ecu_connection(void);
void check_ecu_disconnection(void);
void reset_ecu_connection(void);
void ecu_reconnection_task(void *pv);
void reset_ecu_error_counters(void);

// ELM327 communication
esp_err_t elm327_send_command(const char *cmd);
esp_err_t elm327_send_command_with_options(const char *cmd, bool wait_for_prompt);
void elm327_handle_response(const char *response);
void process_obd_response(const char *response);

#endif // ELM327_H 