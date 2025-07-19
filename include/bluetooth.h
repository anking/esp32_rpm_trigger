#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"

// Bluetooth connection states
extern bool is_connected;
extern bool is_connecting;
extern bool is_searching;
extern uint32_t spp_handle;

// ELM327 target device
#define ELM327_BT_ADDR {0x01, 0x23, 0x45, 0x67, 0x89, 0xBA}
extern uint8_t target_elm327_bda[6];

// Function declarations
void bluetooth_init(void);
void start_device_discovery(void);

// Connection management
void handle_connection_failure(void);
bool attempt_connection(uint8_t *bda, int scn);

#endif // BLUETOOTH_H 