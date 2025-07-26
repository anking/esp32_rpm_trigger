#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt.h"
#include "esp_bt_main.h"

// Bluetooth connection states
extern bool is_connected;
extern bool is_connecting;
extern bool is_scanning;
extern uint16_t gattc_if;
extern uint16_t conn_id;

// Nordic UART Service (common for ELM327 BLE adapters)
#define UART_SERVICE_UUID           0x6E400001, 0xB5A3, 0xF393, 0xE0A9, 0xE50E, 0x24DC, 0xCA9E, 0x21
#define UART_TX_CHAR_UUID           0x6E400002, 0xB5A3, 0xF393, 0xE0A9, 0xE50E, 0x24DC, 0xCA9E, 0x21  // Write to ELM327
#define UART_RX_CHAR_UUID           0x6E400003, 0xB5A3, 0xF393, 0xE0A9, 0xE50E, 0x24DC, 0xCA9E, 0x21  // Read from ELM327

// ELM327 target device (update this with your specific device's MAC)
extern esp_bd_addr_t target_elm327_addr;

// GATT handles for UART service  
extern uint16_t uart_service_handle;
extern uint16_t tx_char_handle;
extern uint16_t rx_char_handle;
extern uint16_t rx_char_cccd_handle;

// Function declarations
void bluetooth_init(void);
void start_ble_scan(void);
esp_err_t ble_uart_write(uint8_t *data, size_t len);

// Connection management
void handle_connection_failure(void);
bool attempt_ble_connection(esp_bd_addr_t addr);

#endif // BLUETOOTH_H 