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

// Command pacing - wait for prompt before sending next command
static volatile bool elm_ready = false;
static uint8_t consecutive_fail = 0;

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
    
    // Wait for ELM327 to be ready (prompt detected) with timeout
    TickType_t start_time = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(2000);  // 2 second timeout
    
    while (!elm_ready) {
        vTaskDelay(pdMS_TO_TICKS(5));  // Wait in 5ms slots
        
        // Check for timeout to prevent watchdog issues
        if ((xTaskGetTickCount() - start_time) > timeout) {
            ESP_LOGW(TAG, "⚠️ Timeout waiting for ELM327 prompt, sending anyway");
            break;
        }
    }
    elm_ready = false;  // Clear flag before sending
    
    char formatted_cmd[32];
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
    
    ESP_LOGI(TAG, "📥 ELM327 RAW response: '%s'", response);
    
    // Check for common ELM327 responses
    if (strstr(response, "ELM327")) {
        ESP_LOGI(TAG, "🔧 ELM327 device identified: %s", response);
    } else if (strstr(response, "OK")) {
        ESP_LOGD(TAG, "✅ Command acknowledged");
    } else if (strstr(response, "CAN ERROR") || strstr(response, "NO DATA")) {
        ESP_LOGW(TAG, "⚠️ CAN/ECU error: %s", response);
        if (++consecutive_fail >= 3) {
            ESP_LOGW(TAG, "⚠️ 3 consecutive failures, backing off...");
            consecutive_fail = 0;
            // Slow-down: pause for 300ms
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        return;
    } else if (strstr(response, "ERROR")) {
        ESP_LOGW(TAG, "⚠️ ELM327 error: %s", response);
    } else if (strstr(response, "UNABLE TO CONNECT")) {
        ESP_LOGD(TAG, "🔌 ELM327 cannot connect to ECU (normal when not in car)");
    } else if (strstr(response, "SEARCHING")) {
        ESP_LOGD(TAG, "🔍 ELM327 searching for ECU...");
    } else {
        // Successfully received data - reset failure counter
        consecutive_fail = 0;
        
        // Process as potential OBD data
        /* Handle possible multi-PID payload */
        char response_copy[256];
        strncpy(response_copy, response, sizeof(response_copy) - 1);
        response_copy[sizeof(response_copy) - 1] = '\0';
        parse_multi_pid_line(response_copy);
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
        } else if (c == '>') {
            // Prompt detected - ELM327 is ready for next command
            elm_ready = true;
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
    
    // Configure ELM327 for Honda Civic (ISO 15765-4 29-bit, 500k)
    LOG_ELM(TAG, "Configuring protocol for Honda Civic...");
    
    // Turn off echo
    LOG_ELM(TAG, "Sending ATE0 (Echo OFF)...");
    ret = elm327_send_command("ATE0");
    if (ret != ESP_OK) {
        LOG_WARN(TAG, "Failed to send ATE0");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Try automatic protocol detection first
    LOG_ELM(TAG, "Sending AT SP 0 (Auto protocol detection)...");
    elm327_send_command("AT SP 0");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Allow long frames (>7 bytes)
    LOG_ELM(TAG, "Sending AT AL (Allow Long frames)...");
    elm327_send_command("AT AL");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Try broadcast first (more compatible)
    LOG_ELM(TAG, "Sending AT SH 7DF (Broadcast address)...");
    elm327_send_command("AT SH 7DF");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Enable ELM auto-formatting for ISO-TP
    LOG_ELM(TAG, "Sending AT CAF1 (Auto-format ISO-TP)...");
    elm327_send_command("AT CAF1");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Set shorter timeout (50ms instead of 100ms default)
    LOG_ELM(TAG, "Sending AT ST 32 (50ms timeout)...");
    elm327_send_command("AT ST 32");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Headers off for shorter replies
    LOG_ELM(TAG, "Sending ATH0 (Headers OFF)...");
    elm327_send_command("ATH0");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Test basic connectivity with a simple command
    LOG_ELM(TAG, "Testing connectivity with AT RV (voltage check)...");
    elm327_send_command("AT RV");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Check what protocol was detected
    LOG_ELM(TAG, "Checking detected protocol with AT DPN...");
    elm327_send_command("AT DPN");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Try a basic OBD test
    LOG_ELM(TAG, "Testing basic OBD with 0100 (Supported PIDs)...");
    elm327_send_command("0100");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    LOG_ELM(TAG, "If above shows CAN ERROR, check:");
    LOG_ELM(TAG, "1. Car ignition is ON");
    LOG_ELM(TAG, "2. Car engine is running");
    LOG_ELM(TAG, "3. OBD port connection is secure");
    
    // Mark as initialized
    elm327_initialized = true;
    elm_ready = true;  // Ready to accept commands
    LOG_INFO(TAG, "ELM327 initialization complete - diagnostics above show readiness!");
    
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