#include "esp_log.h"
#include "esp_spp_api.h"
#include <string.h>
#include <stdio.h>

#include "logging_config.h"
#include "elm327.h"
#include "bluetooth.h"
#include "obd_data.h"

static const char *TAG = "ELM327";

/*
 * ELM327 Communication Logging:
 * ⬆️ SEND: Commands sent TO ELM327
 * ⬇️ RECV: Responses received FROM ELM327  
 */

// ELM327 state variables
bool elm327_initialized = false;
bool ecu_connected = false;
SemaphoreHandle_t connection_semaphore;
char rx_buffer[RX_BUFFER_SIZE];
uint16_t rx_buffer_len = 0;

// Command pacing - wait for prompt before sending next command
static volatile bool elm_ready = false;
static uint8_t consecutive_fail = 0;

// Initialization phase tracking
static volatile bool initialization_in_progress = false;

// ECU disconnection detection
static uint8_t consecutive_ecu_failures = 0;
static uint8_t unable_to_connect_count = 0;
static uint8_t can_error_count = 0;
#define MAX_CONSECUTIVE_FAILURES 3
#define MAX_UNABLE_TO_CONNECT 2
#define MAX_CAN_ERRORS 5

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

// ELM327 specific command sending with error handling
esp_err_t elm327_send_command_with_options(const char *cmd, bool wait_for_prompt) {
    if (!is_connected || !spp_handle) {
        ESP_LOGW(TAG, "⚠️ Not connected to ELM327");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Always log outgoing commands
    ESP_LOGI(TAG, "⬆️ SEND: %s", cmd);
    
    // Wait for ELM327 to be ready (prompt detected) with timeout - only if requested
    if (wait_for_prompt) {
        TickType_t start_time = xTaskGetTickCount();
        TickType_t timeout = pdMS_TO_TICKS(2000);  // 2 second timeout
        
        while (!elm_ready) {
            vTaskDelay(pdMS_TO_TICKS(5));  // Wait in 5ms slots
            
            // Check for timeout to prevent watchdog issues
            if ((xTaskGetTickCount() - start_time) > timeout) {
                if (initialization_in_progress) {
                    ESP_LOGD(TAG, "🔧 ELM327 busy during initialization, proceeding with send");
                } else {
                    ESP_LOGW(TAG, "⚠️ Timeout waiting for prompt, proceeding with send");
                }
                break;
            }
        }
        elm_ready = false;  // Clear flag before sending
    } else {
        ESP_LOGD(TAG, "Skipping prompt wait for ECU test command");
    }
    
    char formatted_cmd[32];
    int len = snprintf(formatted_cmd, sizeof(formatted_cmd), "%s\r", cmd);
    
    esp_err_t ret = esp_spp_write(spp_handle, len, (uint8_t *)formatted_cmd);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠️ Failed to send '%s': %s", cmd, esp_err_to_name(ret));
    }
    
    return ret;
}

// Standard command sending (waits for prompt)
esp_err_t elm327_send_command(const char *cmd) {
    return elm327_send_command_with_options(cmd, true);
}

// Handle ELM327 responses
void elm327_handle_response(const char *response) {
    if (!response || strlen(response) == 0) {
        return;
    }
    
    ESP_LOGI(TAG, "⬇️ RECV: %s", response);
    
    // Check for common ELM327 responses
    if (strstr(response, "ELM327")) {
        ESP_LOGI(TAG, "🔧 ELM327 device identified: %s", response);
    } else if (strstr(response, "OK")) {
        ESP_LOGD(TAG, "✅ Command acknowledged");
    } else if (strstr(response, "CAN ERROR") || strstr(response, "NO DATA")) {
        ESP_LOGW(TAG, "⚠️ CAN/ECU error: %s", response);
        
        // Track ECU disconnection patterns
        consecutive_ecu_failures++;
        can_error_count++;
        
        if (++consecutive_fail >= 3) {
            ESP_LOGW(TAG, "⚠️ 3 consecutive failures, backing off...");
            consecutive_fail = 0;
            // Slow-down: pause for 300ms
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        
        // Check for ECU disconnection
        check_ecu_disconnection();
        return;
    } else if (strstr(response, "ERROR")) {
        ESP_LOGW(TAG, "⚠️ ELM327 error: %s", response);
        consecutive_ecu_failures++;
        check_ecu_disconnection();
    } else if (strstr(response, "UNABLE TO CONNECT")) {
        ESP_LOGD(TAG, "🔌 ELM327 cannot connect to ECU");
        
        // Track unable to connect failures
        consecutive_ecu_failures++;
        unable_to_connect_count++;
        
        // Check for ECU disconnection
        check_ecu_disconnection();
    } else if (strstr(response, "SEARCHING")) {
        ESP_LOGD(TAG, "🔍 ELM327 searching for ECU...");
    } else {
        // Successfully received some response - reset basic failure counter
        consecutive_fail = 0;
        
        // Process as potential OBD data
        process_obd_response(response);
    }
}

// Process OBD data responses
void process_obd_response(const char *response) {
    // Only reset ECU error counters for actual OBD responses (start with "41")
    if (strstr(response, "41 ") != NULL) {
        // This is a valid OBD Mode 1 response - parse it
        char response_copy[256];
        strncpy(response_copy, response, sizeof(response_copy) - 1);
        response_copy[sizeof(response_copy) - 1] = '\0';
        parse_multi_pid_line(response_copy);
        // Note: parse_multi_pid_line will call reset_ecu_error_counters() if successful
    }
}

// Process received data from Bluetooth
void process_received_data(const char *data, uint16_t len) {
    if (!data || len == 0) {
        return;
    }
    
    // Debug: Log raw received data
    ESP_LOGD(TAG, "Raw data received (%d bytes): %.*s", len, len, data);
    
    // Add received data to buffer
    for (uint16_t i = 0; i < len && rx_buffer_len < (RX_BUFFER_SIZE - 1); i++) {
        char c = data[i];
        
        // Check for end of response (carriage return)
        if (c == '\r') {
            if (rx_buffer_len > 0) {
                rx_buffer[rx_buffer_len] = '\0';  // Null terminate
                
                ESP_LOGD(TAG, "Processing response (len=%d): '%s'", rx_buffer_len, rx_buffer);
                elm327_handle_response(rx_buffer);
                
                // Clear buffer for next response
                rx_buffer_len = 0;
                memset(rx_buffer, 0, RX_BUFFER_SIZE);
            }
        } else if (c == '\n') {
            // Ignore line feed (often follows carriage return) - prevents duplicate processing
            ESP_LOGD(TAG, "Ignoring LF character");
            continue;
        } else if (c == '>') {
            // Prompt detected - ELM327 is ready for next command
            elm_ready = true;
            ESP_LOGD(TAG, "🔥 Prompt '>' detected, elm_ready=true");
        } else if (c >= 32 && c <= 126) {  // Printable ASCII characters
            rx_buffer[rx_buffer_len++] = c;
        } else {
            // Log any unexpected characters
            ESP_LOGD(TAG, "Unexpected char: 0x%02X (%d)", c, c);
        }
    }
}

// Test ECU connectivity with a basic OBD command
bool test_ecu_connectivity(void) {
    // Check prerequisites
    if (!is_connected) {
        ESP_LOGD(TAG, "❌ Bluetooth not connected - cannot test ECU");
        return false;
    }
    
    if (!elm327_initialized) {
        ESP_LOGD(TAG, "❌ ELM327 not initialized - cannot test ECU");
        return false;
    }
    
    ESP_LOGI(TAG, "🔍 Testing ECU connectivity...");
    
    // Clear any pending response flags and buffer
    elm_ready = false;
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    rx_buffer_len = 0;
    
    // Send basic OBD test command (get supported PIDs) - skip prompt wait for ECU testing
    esp_err_t ret = elm327_send_command_with_options("0100", false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "❌ Failed to send ECU test command");
        return false;
    }
    
    // Wait for response with timeout
    TickType_t start_time = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(4000);  // 4 second timeout (longer for first connection)
    
    while ((xTaskGetTickCount() - start_time) < timeout) {
        vTaskDelay(pdMS_TO_TICKS(50));  // Check every 50ms
        
        // Check if we got a valid OBD response (starts with "41 00")
        if (rx_buffer_len > 5) {
            if (strstr(rx_buffer, "41 00") != NULL) {
                ESP_LOGI(TAG, "✅ ECU connection verified! Supported PIDs detected");
                return true;
            }
            
            // Check for error responses
            if (strstr(rx_buffer, "UNABLE TO CONNECT") != NULL) {
                return false;
            }
            
            if (strstr(rx_buffer, "CAN ERROR") != NULL) {
                return false;
            }
            
            if (strstr(rx_buffer, "NO DATA") != NULL) {
                return false;
            }
            
            // If we got "SEARCHING..." continue waiting
            if (strstr(rx_buffer, "SEARCHING") != NULL) {
                // Clear buffer and continue waiting
                memset(rx_buffer, 0, RX_BUFFER_SIZE);
                rx_buffer_len = 0;
                continue;
            }
        }
    }
    
    ESP_LOGD(TAG, "❌ ECU test timeout - no valid response received");
    return false;
}

// Verify ECU connection with retries
void verify_ecu_connection(void) {
    ESP_LOGI(TAG, "🔗 === ECU CONNECTION VERIFICATION START ===");
    ESP_LOGI(TAG, "📡 Testing connection to vehicle ECU...");
    
    ecu_connected = false;
    int attempt = 1;
    
    while (!ecu_connected) {
        // Check if Bluetooth is still connected
        if (!is_connected) {
            ESP_LOGW(TAG, "🔴 Bluetooth disconnected during ECU verification - stopping");
            ecu_connected = false;
            return;
        }
        
        // Check if ELM327 is still initialized
        if (!elm327_initialized) {
            ESP_LOGW(TAG, "🔴 ELM327 no longer initialized - stopping ECU verification");
            ecu_connected = false;
            return;
        }
        
        ESP_LOGI(TAG, "🔄 ECU Connection Attempt #%d", attempt);
        
        if (test_ecu_connectivity()) {
            ecu_connected = true;
            ESP_LOGI(TAG, "✅ === ECU CONNECTION ESTABLISHED ===");
            ESP_LOGI(TAG, "🚗 Vehicle ECU is responding to OBD commands");
            ESP_LOGI(TAG, "🎯 System ready for OBD data polling!");
            break;
        }
        
        ESP_LOGD(TAG, "⏱️ ECU not ready, retrying in 2 seconds...");
        ESP_LOGD(TAG, "💡 Ensure: 1) Ignition ON  2) Engine running  3) OBD cable secure");
        
        vTaskDelay(pdMS_TO_TICKS(2000));  // Wait 2 seconds before retry
        attempt++;
        
        // Optional: Add max attempts if desired
        // if (attempt > 30) { // 1 minute of attempts
        //     ESP_LOGE(TAG, "❌ ECU connection failed after 30 attempts");
        //     break;
        // }
    }
}

// Check for ECU disconnection based on error patterns
void check_ecu_disconnection(void) {
    if (!ecu_connected) {
        return;  // Already disconnected
    }
    
    // Check if we've exceeded disconnection thresholds
    if (consecutive_ecu_failures >= MAX_CONSECUTIVE_FAILURES || 
        unable_to_connect_count >= MAX_UNABLE_TO_CONNECT ||
        can_error_count >= MAX_CAN_ERRORS) {
        
        ESP_LOGW(TAG, "🔴 ECU DISCONNECTION DETECTED!");
        ESP_LOGW(TAG, "├─ Consecutive failures: %d/%d", consecutive_ecu_failures, MAX_CONSECUTIVE_FAILURES);
        ESP_LOGW(TAG, "├─ Unable to connect: %d/%d", unable_to_connect_count, MAX_UNABLE_TO_CONNECT);
        ESP_LOGW(TAG, "├─ CAN errors: %d/%d", can_error_count, MAX_CAN_ERRORS);
        
        reset_ecu_connection();
    }
}

// Reset ECU connection and start reconnection process
void reset_ecu_connection(void) {
    ESP_LOGW(TAG, "🔄 Resetting ECU connection - will attempt reconnection");
    
    // Reset connection state
    ecu_connected = false;
    
    // Reset error counters
    consecutive_ecu_failures = 0;
    unable_to_connect_count = 0;
    can_error_count = 0;
    
    ESP_LOGI(TAG, "🔗 Starting ECU reconnection process...");
    
    // Start verification in separate task to avoid blocking
    xTaskCreate(ecu_reconnection_task, "ecu_reconnect", 4096, NULL, 5, NULL);
}

// ECU reconnection task (runs in separate thread)
void ecu_reconnection_task(void *pv) {
    ESP_LOGI(TAG, "ECU reconnection task started...");
    
    // Check if Bluetooth is still connected before attempting reconnection
    if (!is_connected) {
        ESP_LOGW(TAG, "🔴 Bluetooth disconnected - aborting ECU reconnection");
        vTaskDelete(NULL);
        return;
    }
    
    // Perform ECU connection verification
    verify_ecu_connection();
    
    // Delete this task when done
    vTaskDelete(NULL);
}

// Reset ECU error counters on successful data reception
void reset_ecu_error_counters(void) {
    consecutive_ecu_failures = 0;
    // Gradually reduce error counts on success (but keep some history for resilience)
    if (unable_to_connect_count > 0) unable_to_connect_count--;
    if (can_error_count > 0) can_error_count--;
}

// Gentle ELM327 initialization to prevent disconnection
void initialize_elm327(void) {
    // Mark initialization as in progress
    initialization_in_progress = true;
    
    ESP_LOGI(TAG, "🚀 === ELM327 INITIALIZATION SEQUENCE START ===");
    ESP_LOGI(TAG, "🔧 Configuring ELM327 for Honda Civic (ISO 15765-4, 29-bit, 500k)");
    
    // Wait for ELM327 to settle after connection
    ESP_LOGI(TAG, "⏱️ Step 1: Waiting 3 seconds for ELM327 to settle...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Send reset command
    ESP_LOGI(TAG, "🔄 Step 2: Sending reset command...");
    esp_err_t ret = elm327_send_command("ATZ");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "❌ Failed to send ATZ");
        initialization_in_progress = false;  // Clear flag on failure
        return;
    }
    
    // Wait for reset to complete
    ESP_LOGI(TAG, "⏱️ Step 3: Waiting for reset to complete...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Configure ELM327 for Honda Civic (ISO 15765-4 29-bit, 500k)
    ESP_LOGI(TAG, "⚙️ Step 4: Configuring ELM327 parameters...");
    
    // Turn off echo
    ESP_LOGI(TAG, "├─ Disabling echo...");
    ret = elm327_send_command("ATE0");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "❌ Failed to send ATE0");
        initialization_in_progress = false;  // Clear flag on failure
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Try automatic protocol detection first
    ESP_LOGI(TAG, "├─ Setting auto protocol detection...");
    elm327_send_command("AT SP 0");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Allow long frames (>7 bytes)
    ESP_LOGI(TAG, "├─ Enabling long frames...");
    elm327_send_command("AT AL");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Try broadcast first (more compatible)
    ESP_LOGI(TAG, "├─ Setting broadcast address...");
    elm327_send_command("AT SH 7DF");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Enable ELM auto-formatting for ISO-TP
    ESP_LOGI(TAG, "├─ Enabling auto-format ISO-TP...");
    elm327_send_command("AT CAF1");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Set shorter timeout (50ms instead of 100ms default)
    ESP_LOGI(TAG, "├─ Setting shorter timeout...");
    elm327_send_command("AT ST 32");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Headers off for shorter replies
    ESP_LOGI(TAG, "├─ Disabling headers...");
    elm327_send_command("ATH0");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Test basic connectivity with a simple command
    ESP_LOGI(TAG, "├─ Testing basic connectivity...");
    elm327_send_command("AT RV");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Check what protocol was detected
    ESP_LOGI(TAG, "├─ Checking detected protocol...");
    elm327_send_command("AT DPN");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Try a basic OBD test
    ESP_LOGI(TAG, "├─ Testing basic OBD...");
    elm327_send_command("0100");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "If above shows CAN ERROR, check:");
    ESP_LOGI(TAG, "1. Car ignition is ON");
    ESP_LOGI(TAG, "2. Car engine is running");
    ESP_LOGI(TAG, "3. OBD port connection is secure");
    
    // Mark as initialized
    elm327_initialized = true;
    elm_ready = true;  // Ready to accept commands
    initialization_in_progress = false;  // Initialization complete
    ESP_LOGI(TAG, "✅ === ELM327 INITIALIZATION SEQUENCE COMPLETE ===");
    
    // Signal that ELM327 is ready
    xSemaphoreGive(connection_semaphore);
    
    // Now verify connection to vehicle ECU
    verify_ecu_connection();
}

// ELM327 initialization task (runs in separate thread)
void initialize_elm327_task(void *pv) {
    LOG_ELM(TAG, "ELM327 initialization task started...");
    
    // Perform gentle initialization
    initialize_elm327();
    
    // Delete this task when done
    vTaskDelete(NULL);
} 