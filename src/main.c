#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h" // Add this header for GAP functions
#include <string.h>
#include <stdio.h>
#include <inttypes.h>  // for PRIu32

#define LED_GPIO GPIO_NUM_2
#define RPM_THRESHOLD 3000
#define TAG "OBD_CONTROLLER"
#define RX_BUFFER_SIZE 256

// Replace this with your ELM327 device name or address
#define ELM327_BT_NAME "OBDII"

static uint32_t current_rpm = 0;
static uint8_t throttle_position = 0;  // Throttle position (0-100%)
static uint8_t vehicle_speed = 0;      // Speed in km/h
static char current_gear[4] = "N/A";
static bool gpio_status = false;
static bool is_connected = false;
static uint32_t spp_handle = 0;
static char rx_buffer[RX_BUFFER_SIZE];
static uint16_t rx_buffer_len = 0;

// Send an OBD command
void send_obd_command(const char *cmd) {
    if (is_connected) {
        esp_spp_write(spp_handle, strlen(cmd), (uint8_t *)cmd);
    }
}

// Parse RPM response from ELM327
uint32_t parse_rpm(const char *response) {
    // Example response: "41 0C 1A F8\r"
    int A, B;
    if (sscanf(response, "41 0C %x %x", &A, &B) == 2) {
        return ((A * 256) + B) / 4;
    }
    return 0;
}

// Parse throttle position response from ELM327
uint8_t parse_throttle(const char *response) {
    // Example response: "41 11 5A\r" (90% throttle)
    int A;
    if (sscanf(response, "41 11 %x", &A) == 1) {
        return (uint8_t)((A * 100) / 255);  // Convert to percentage
    }
    return 0;
}

// Parse vehicle speed response from ELM327
uint8_t parse_speed(const char *response) {
    // Example response: "41 0D 64\r" (100 km/h)
    int A;
    if (sscanf(response, "41 0D %x", &A) == 1) {
        return (uint8_t)A;  // Speed in km/h
    }
    return 0;
}

// Parse gear (if your ECU supports it)
void parse_gear(const char *response) {
    // Example: mock up, real logic depends on PID support
    strcpy(current_gear, "N"); // Default to Neutral
}

// Process received data
void process_received_data(const char *data, uint16_t len) {
    // Append new data to buffer
    if (rx_buffer_len + len < RX_BUFFER_SIZE) {
        memcpy(rx_buffer + rx_buffer_len, data, len);
        rx_buffer_len += len;
        rx_buffer[rx_buffer_len] = '\0';  // Null-terminate
    } else {
        ESP_LOGE(TAG, "Buffer overflow, clearing buffer");
        rx_buffer_len = 0;
        return;
    }

    // Process complete lines
    char *line = rx_buffer;
    char *end;
    while ((end = strchr(line, '\r')) != NULL) {
        *end = '\0';  // Terminate line at carriage return
        if (strncmp(line, "41 0C", 5) == 0) {
            current_rpm = parse_rpm(line);
        } else if (strncmp(line, "41 11", 5) == 0) {
            throttle_position = parse_throttle(line);
        } else if (strncmp(line, "41 0D", 5) == 0) {
            vehicle_speed = parse_speed(line);
        } else {
            parse_gear(line);  // Handle gear if supported
        }
        line = end + 1;  // Move to next line
    }

    // Shift remaining data to start of buffer
    if (line != rx_buffer) {
        uint16_t remaining = rx_buffer_len - (line - rx_buffer);
        memmove(rx_buffer, line, remaining);
        rx_buffer_len = remaining;
        rx_buffer[rx_buffer_len] = '\0';
    }
}

// Called every 250ms
void obd_task(void *pv) {
    while (1) {
        send_obd_command("010C\r"); // Request RPM
        send_obd_command("0111\r"); // Request Throttle Position
        send_obd_command("010D\r"); // Request Vehicle Speed
        // Optionally send gear PID if supported, e.g. 0101 or 012F

        // Control GPIO
        if (current_rpm >= RPM_THRESHOLD) {
            gpio_set_level(LED_GPIO, 1);
            gpio_status = true;
        } else {
            gpio_set_level(LED_GPIO, 0);
            gpio_status = false;
        }

        // Print status
        ESP_LOGI(TAG, "RPM: %" PRIu32 " | Throttle: %d%% | Speed: %d km/h | Gear: %s | GPIO2: %s",
            current_rpm, throttle_position, vehicle_speed, current_gear, gpio_status ? "ON" : "OFF");

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// Handle Bluetooth events
void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(TAG, "SPP initialized, starting discovery...");
            esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
            break;
        case ESP_SPP_OPEN_EVT:
            ESP_LOGI(TAG, "Connected to ELM327");
            is_connected = true;
            spp_handle = param->open.handle;
            // Initialize ELM327
            send_obd_command("ATZ\r");  // Reset
            vTaskDelay(pdMS_TO_TICKS(500));
            send_obd_command("ATE0\r"); // Echo off
            vTaskDelay(pdMS_TO_TICKS(100));
            send_obd_command("ATSP0\r"); // Auto protocol
            break;
        case ESP_SPP_DATA_IND_EVT:
            ESP_LOGI(TAG, "Data Received: %.*s", param->data_ind.len, param->data_ind.data);
            process_received_data((const char *)param->data_ind.data, param->data_ind.len);
            break;
        default:
            break;
    }
}

// Start Bluetooth SPP
void bluetooth_init(void) {
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);

    esp_bluedroid_init();
    esp_bluedroid_enable();

    esp_spp_register_callback(spp_callback);
    esp_spp_enhanced_init(ESP_SPP_MODE_CB);
}

void app_main(void)
{
    // Init GPIO
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    // Start Bluetooth
    bluetooth_init();

    // Start OBD polling task
    xTaskCreate(obd_task, "obd_task", 4096, NULL, 5, NULL);
}
