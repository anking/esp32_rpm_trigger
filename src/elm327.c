#include "esp_log.h"
#include "esp_spp_api.h"
#include <string.h>
#include <stdio.h>

#include "logging_config.h"
#include "elm327.h"
#include "bluetooth.h"
#include "obd_data.h"

static const char *TAG = "ELM327";

// ELM327 state variables
bool elm327_initialized = false;
SemaphoreHandle_t connection_semaphore;
char rx_buffer[RX_BUFFER_SIZE];
uint16_t rx_buffer_len = 0;

// Initialize ELM327 system (semaphore, etc.)
void elm327_init_system(void) {
    // Create semaphore for connection synchronization
    connection_semaphore = xSemaphoreCreateBinary();
    if (connection_semaphore == NULL) {
        ESP_LOGE(TAG, "❌ Failed to create connection semaphore");
        return;
    }
    
    // Initialize RX buffer
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    rx_buffer_len = 0;
    
    LOG_VERBOSE(TAG, "ELM327 system initialized");
}

// Send OBD command to ELM327
void send_obd_command(const char *cmd) {
    if (is_connected && elm327_initialized && spp_handle) {
        LOG_DEBUG(TAG, "Sending OBD command: %s", cmd);
        
        // Format command with carriage return
        char formatted_cmd[32];
        snprintf(formatted_cmd, sizeof(formatted_cmd), "%s\r", cmd);
        
        esp_err_t ret = esp_spp_write(spp_handle, strlen(formatted_cmd), (uint8_t *)formatted_cmd);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "⚠️ Failed to send command: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGW(TAG, "⚠️ Cannot send command - not connected or not initialized");
    }
}

// ELM327 specific command sending with error handling
esp_err_t elm327_send_command(const char *cmd) {
    if (!is_connected || !spp_handle) {
        ESP_LOGW(TAG, "⚠️ Not connected to ELM327");
        return ESP_ERR_INVALID_STATE;
    }
    
    char formatted_cmd[64];
    int len = snprintf(formatted_cmd, sizeof(formatted_cmd), "%s\r", cmd);
    
    esp_err_t ret = esp_spp_write(spp_handle, len, (uint8_t *)formatted_cmd);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "📤 Sent: %s", cmd);
    } else {
        ESP_LOGW(TAG, "⚠️ Failed to send '%s': %s", cmd, esp_err_to_name(ret));
    }
    
    return ret;
}

// Handle ELM327 responses
void elm327_handle_response(const char *response) {
    if (!response || strlen(response) == 0) {
        return;
    }
    
    ESP_LOGD(TAG, "📥 ELM327 response: %s", response);
    
    // Check for common ELM327 responses
    if (strstr(response, "ELM327")) {
        ESP_LOGI(TAG, "🔧 ELM327 device identified: %s", response);
    } else if (strstr(response, "OK")) {
        ESP_LOGD(TAG, "✅ Command acknowledged");
    } else if (strstr(response, "ERROR")) {
        ESP_LOGW(TAG, "⚠️ ELM327 error: %s", response);
    } else if (strstr(response, "UNABLE TO CONNECT")) {
        ESP_LOGD(TAG, "🔌 ELM327 cannot connect to ECU (normal when not in car)");
    } else if (strstr(response, "SEARCHING")) {
        ESP_LOGD(TAG, "🔍 ELM327 searching for ECU...");
    } else {
        // Process as potential OBD data
        vehicle_data_t *data = &vehicle_data;
        
        // Try to parse different OBD responses
        if (strstr(response, "41 0C")) {
            data->rpm = parse_rpm(response);
        } else if (strstr(response, "41 11")) {
            data->throttle_position = parse_throttle(response);
        } else if (strstr(response, "41 0D")) {
            data->vehicle_speed = parse_speed(response);
        }
    }
}

// Process received data from Bluetooth
void process_received_data(const char *data, uint16_t len) {
    if (!data || len == 0) {
        return;
    }
    
    // Add received data to buffer
    for (uint16_t i = 0; i < len && rx_buffer_len < (RX_BUFFER_SIZE - 1); i++) {
        char c = data[i];
        
        // Check for end of response (carriage return or newline)
        if (c == '\r' || c == '\n') {
            if (rx_buffer_len > 0) {
                rx_buffer[rx_buffer_len] = '\0';  // Null terminate
                
                ESP_LOGD(TAG, "Processing response: %s", rx_buffer);
                elm327_handle_response(rx_buffer);
                
                // Clear buffer for next response
                rx_buffer_len = 0;
                memset(rx_buffer, 0, RX_BUFFER_SIZE);
            }
        } else if (c >= 32 && c <= 126) {  // Printable ASCII characters
            rx_buffer[rx_buffer_len++] = c;
        }
    }
}

// Gentle ELM327 initialization to prevent disconnection
void initialize_elm327(void) {
    LOG_ELM(TAG, "Starting GENTLE ELM327 initialization...");
    
    // Wait for ELM327 to settle after connection
    LOG_ELM(TAG, "Waiting 3 seconds for ELM327 to settle...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Send reset command
    LOG_ELM(TAG, "Sending gentle ATZ (Reset)...");
    esp_err_t ret = elm327_send_command("ATZ");
    if (ret != ESP_OK) {
        LOG_WARN(TAG, "Failed to send ATZ");
        return;
    }
    
    // Wait for reset to complete
    LOG_ELM(TAG, "Waiting for ELM327 reset response...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Turn off echo
    LOG_ELM(TAG, "Sending ATE0 (Echo OFF)...");
    ret = elm327_send_command("ATE0");
    if (ret != ESP_OK) {
        LOG_WARN(TAG, "Failed to send ATE0");
        return;
    }
    
    // Wait for echo off response
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Mark as initialized
    elm327_initialized = true;
    LOG_INFO(TAG, "Basic ELM327 initialization complete!");
    
    // Signal that connection is ready
    xSemaphoreGive(connection_semaphore);
}

// ELM327 initialization task (runs in separate thread)
void initialize_elm327_task(void *pv) {
    LOG_ELM(TAG, "ELM327 initialization task started...");
    
    // Perform gentle initialization
    initialize_elm327();
    
    // Delete this task when done
    vTaskDelete(NULL);
} 