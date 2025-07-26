/* BLE test stub header */
#ifndef BT_TEST_STUB_H
#define BT_TEST_STUB_H

#ifdef __cplusplus
extern "C" {
#endif

// Starts a simplistic BLE test: initializes Bluetooth and begins scanning for the
// VEEPEAK ELM327 adapter.  When the adapter is discovered the full MAC address is
// printed via ESP_LOGI in the GAP callback implemented in bluetooth.c.
void run_bt_connection_test(void);

#ifdef __cplusplus
}
#endif

#endif // BT_TEST_STUB_H 