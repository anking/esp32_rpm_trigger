#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "logging_config.h"
#include "elm327.h"
#include "bluetooth.h"
#include "obd_data.h"
#include "gpio_control.h"

// External declarations from bluetooth.h
extern uint16_t tx_char_handle;

static const char *TAG = "ELM327";

// Helper function to check for line number prefix safely
static bool has_line_number_prefix(const char *str) {
    if (!str || strlen(str) < 2) return false;
    char first = str[0];
    char second = str[1];
    return first >= '0' && first <= '9' && second == ':';
}

// Helper function to check for Mode 1 response
static bool is_mode1_response(const char *str) {
    return str && strlen(str) >= 2 && strncmp(str, "41", 2) == 0;
}

/*
 * ELM327 Communication Logging:
 * ⬆️ SEND: Commands sent TO ELM327
 * ⬇️ RECV: Responses received FROM ELM327  
 */

// ELM327 state variables
bool elm327_initialized = false;
bool ecu_connected = false;
bool response_received_flag = false;  // Flag set when any response is received
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
static volatile bool ecu_test_response_received = false;
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
    if (!is_connected || tx_char_handle == 0) {
        LOG_WARN(TAG, "⚠️ Not connected to ELM327");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Command logging should respect ELM327 logging flag
    LOG_ELM(TAG, "⬆️ SEND: %s", cmd);
    
    // Wait for ELM327 to be ready (prompt detected) with timeout - only if requested
    if (wait_for_prompt) {
        TickType_t start_time = xTaskGetTickCount();
        TickType_t timeout = pdMS_TO_TICKS(2000);  // 2 second timeout
        
        while (!elm_ready) {
            vTaskDelay(pdMS_TO_TICKS(5));  // Wait in 5ms slots
            
            // Check for timeout to prevent watchdog issues
            if ((xTaskGetTickCount() - start_time) > timeout) {
                if (initialization_in_progress) {
                    LOG_DEBUG(TAG, "🔧 ELM327 busy during initialization, proceeding with send");
                } else {
                    LOG_WARN(TAG, "⚠️ Timeout waiting for prompt, proceeding with send");
                }
                break;
            }
        }
        elm_ready = false;  // Clear flag before sending
    } else {
        LOG_DEBUG(TAG, "Skipping prompt wait for ECU test command");
    }
    
    char formatted_cmd[32];
    int len = snprintf(formatted_cmd, sizeof(formatted_cmd), "%s\r", cmd);
    
    esp_err_t ret = ble_uart_write((uint8_t *)formatted_cmd, len);
    if (ret != ESP_OK) {
        LOG_WARN(TAG, "⚠️ Failed to send '%s': %s", cmd, esp_err_to_name(ret));
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
    
    // Response logging should respect ELM327 logging flag
    LOG_ELM(TAG, "⬇️ RECV: %s", response);
    
    // Always set the response received flag first
    response_received_flag = true;
    
    // Check for common ELM327 responses
    if (strstr(response, "ELM327")) {
        LOG_INFO(TAG, "🎉 VEEPEAK ELM327 device identified: %s", response);
        LOG_INFO(TAG, "🚀 SUCCESS! VEEPEAK firmware handshake complete - device is responsive!");
        if (strstr(response, "v2.")) {
            LOG_INFO(TAG, "📋 Detected VEEPEAK ELM327 version 2.x - fully compatible");
        }
    } else if (strstr(response, "OK")) {
        LOG_DEBUG(TAG, "✅ Command acknowledged");
    } else if (strcmp(response, "?") == 0) {
        LOG_DEBUG(TAG, "❓ VEEPEAK prompt/error response - device is responding to trigger");
        LOG_DEBUG(TAG, "🔧 This '?' response indicates VEEPEAK processed the trigger command");
    } else if (strstr(response, "CAN ERROR") || strstr(response, "NO DATA")) {
        LOG_WARN(TAG, "⚠️ CAN/ECU error: %s", response);
        
        // Track ECU disconnection patterns
        consecutive_ecu_failures++;
        can_error_count++;
        
        if (++consecutive_fail >= 3) {
            LOG_WARN(TAG, "⚠️ 3 consecutive failures, backing off...");
            consecutive_fail = 0;
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        
        check_ecu_disconnection();
        return;
    } else if (strstr(response, "ERROR")) {
        LOG_WARN(TAG, "⚠️ ELM327 error: %s", response);
        consecutive_ecu_failures++;
        check_ecu_disconnection();
    } else if (strstr(response, "UNABLE TO CONNECT")) {
        LOG_DEBUG(TAG, "🔌 ELM327 cannot connect to ECU");
        consecutive_ecu_failures++;
        unable_to_connect_count++;
        check_ecu_disconnection();
    } else if (strstr(response, "SEARCHING")) {
        LOG_DEBUG(TAG, "🔍 ELM327 searching for ECU...");
        LOG_WARN(TAG, "⚠️ ECU searching - no ECU response yet");
        consecutive_ecu_failures++;
        
        static int search_retry_count = 0;
        search_retry_count++;
        if (search_retry_count >= 3) {
            LOG_WARN(TAG, "🔄 Retrying ECU command after multiple SEARCHING responses...");
            search_retry_count = 0;
            vTaskDelay(pdMS_TO_TICKS(2000));
            
            esp_err_t ret = elm327_send_command("0100");
            if (ret != ESP_OK) {
                LOG_ERROR(TAG, "❌ Failed to retry ECU command: %s", esp_err_to_name(ret));
            }
        }
    } else {
        consecutive_fail = 0;
        
        
        // Remove line number prefix if present (e.g., "0:" or "1:")
        const char *data = response;
        if (has_line_number_prefix(response)) {
            data = response + 2;  // Skip the line number prefix
        }

        // Check for any valid Mode 1 response (41xx)
        if (is_mode1_response(data)) {
            ecu_test_response_received = true;
            LOG_DEBUG(TAG, "🎯 ECU test response detected: %s", response);
        }

        parse_multi_pid_line(response);
    }
}

// Handle ELM327 response received via BLE (bridge function)
void handle_elm327_response(const char *data, size_t len) {
    // Convert size_t to uint16_t for existing function
    uint16_t data_len = (len > UINT16_MAX) ? UINT16_MAX : (uint16_t)len;
    
    // Forward to existing data processing function
    process_received_data(data, data_len);
}

// Process received data from Bluetooth
void process_received_data(const char *data, uint16_t len) {
    if (!data || len == 0) {
        return;
    }
    
    LOG_DEBUG(TAG, "Raw data received (%d bytes): %.*s", len, len, data);
    
    // Process on both \r and \n for better compatibility with Honda ECUs
    for (uint16_t i = 0; i < len && rx_buffer_len < (RX_BUFFER_SIZE - 1); i++) {
        char c = data[i];
        
        if (c == '\r' || c == '\n') {
            if (rx_buffer_len > 0) {
                rx_buffer[rx_buffer_len] = '\0';  // Null terminate
                
                LOG_DEBUG(TAG, "Processing response (len=%d): '%s'", rx_buffer_len, rx_buffer);
                elm327_handle_response(rx_buffer);
                
                // Clear buffer for next response
                rx_buffer_len = 0;
                memset(rx_buffer, 0, RX_BUFFER_SIZE);
            }
        } else if (c == '>') {
            elm_ready = true;
            LOG_DEBUG(TAG, "🔥 Prompt '>' detected, elm_ready=true");
            
            if (rx_buffer_len > 0) {
                rx_buffer[rx_buffer_len] = '\0';
                LOG_DEBUG(TAG, "Processing prompt response: '%s'", rx_buffer);
                elm327_handle_response(rx_buffer);
                rx_buffer_len = 0;
                memset(rx_buffer, 0, RX_BUFFER_SIZE);
            }
            
            response_received_flag = true;
            LOG_DEBUG(TAG, "🔥 response_received_flag set to true (prompt detected)");
        } else if (c >= 32 && c <= 126) {
            rx_buffer[rx_buffer_len++] = c;
        } else {
            LOG_DEBUG(TAG, "Unexpected char: 0x%02X (%d)", c, c);
        }
    }
    
    if (rx_buffer_len > 0 || strstr(data, ">") != NULL) {
        response_received_flag = true;
        LOG_DEBUG(TAG, "🔥 response_received_flag set to true (data processed)");
    }
}

// Test ECU connectivity with a basic OBD command
bool test_ecu_connectivity(void) {
    // Check prerequisites
    if (!is_connected) {
        LOG_DEBUG(TAG, "❌ Bluetooth not connected - cannot test ECU");
        return false;
    }
    
    if (!elm327_initialized) {
        LOG_DEBUG(TAG, "❌ ELM327 not initialized - cannot test ECU");
        return false;
    }
    
    LOG_INFO(TAG, "🔍 Testing ECU connectivity...");
    
    // Clear any pending response flags and buffer
    elm_ready = false;
    ecu_test_response_received = false;
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    rx_buffer_len = 0;
    LOG_INFO(TAG, "Quick ECU verify: sending supported-PID request");
    esp_err_t ret;

    for (int retry = 0; retry < 2; retry++) {
        if (retry > 0) {
            LOG_INFO(TAG, "🔄 ECU test retry #%d", retry + 1);
            vTaskDelay(pdMS_TO_TICKS(2000)); // Wait between retries
        }
        
        ret = elm327_send_command_with_options("0100", false);
        if (ret != ESP_OK) {
            LOG_WARN(TAG, "❌ Failed to send ECU test command");
            continue;
        }
        
        TickType_t start_time = xTaskGetTickCount();
        TickType_t timeout = pdMS_TO_TICKS(15000); // 15s timeout
        
        while ((xTaskGetTickCount() - start_time) < timeout) {
            vTaskDelay(pdMS_TO_TICKS(50));
            
            if (ecu_test_response_received) {
                LOG_INFO(TAG, "✅ ECU connection verified! Supported PIDs detected");
                return true;
            }
            
            if (rx_buffer_len > 5) {
                if (strstr(rx_buffer, "SEARCHING") != NULL) {
                    LOG_DEBUG(TAG, "⏳ ECU still searching, continuing...");
                    // Clear buffer and continue waiting
                    memset(rx_buffer, 0, RX_BUFFER_SIZE);
                    rx_buffer_len = 0;
                    continue;
                }
                
                if (strstr(rx_buffer, "UNABLE TO CONNECT") != NULL ||
                    strstr(rx_buffer, "CAN ERROR") != NULL ||
                    strstr(rx_buffer, "NO DATA") != NULL) {
                    LOG_WARN(TAG, "❌ ECU test failed: %s", rx_buffer);
                    break; // Try next retry
                }
            }
        }
    }
    
    LOG_WARN(TAG, "⏰ ECU test timeout after retries - no response from vehicle");
    return false;
}

// Verify ECU connection with retries
void verify_ecu_connection(void) {
    LOG_INFO(TAG, "🔗 === ECU CONNECTION VERIFICATION START ===");
    LOG_INFO(TAG, "📡 Testing connection to vehicle ECU...");
    
    ecu_connected = false;
    int attempt = 1;
    const int MAX_ATTEMPTS = 10; // Limited to 10 attempts to prevent infinite loop
    
    while (!ecu_connected && attempt <= MAX_ATTEMPTS) {
        if (!is_connected) {
            LOG_WARN(TAG, "🔴 Bluetooth disconnected during ECU verification - stopping");
            ecu_connected = false;
            return;
        }
        
        if (!elm327_initialized) {
            LOG_WARN(TAG, "🔴 ELM327 no longer initialized - stopping ECU verification");
            ecu_connected = false;
            return;
        }
        
        LOG_INFO(TAG, "🔄 ECU Connection Attempt #%d", attempt);
        
        if (test_ecu_connectivity()) {
            ecu_connected = true;
            set_ecu_status(true);  // Notify GPIO module that ECU is connected
            LOG_INFO(TAG, "✅ === ECU CONNECTION ESTABLISHED ===");
            LOG_INFO(TAG, "🚗 Vehicle ECU is responding to OBD commands");
            LOG_INFO(TAG, "🎯 System ready for OBD data polling!");
            break;
        }
        
        LOG_DEBUG(TAG, "⏱️ ECU not ready, retrying in 2 seconds...");
        LOG_DEBUG(TAG, "💡 Ensure: 1) Ignition ON  2) Engine running  3) OBD cable secure");
        
        vTaskDelay(pdMS_TO_TICKS(2000));
        attempt++;
    }
    
    if (!ecu_connected) {
        LOG_ERROR(TAG, "❌ Failed to connect to ECU after %d attempts", MAX_ATTEMPTS);
        LOG_ERROR(TAG, "💡 Please check:");
        LOG_ERROR(TAG, "   - Vehicle ignition is ON");
        LOG_ERROR(TAG, "   - OBD cable is securely connected");
        LOG_ERROR(TAG, "   - Vehicle is compatible with OBD-II");
        LOG_ERROR(TAG, "   - Try cycling ignition OFF/ON and restart");
        LOG_ERROR(TAG, "   - Try unplugging/replugging OBD adapter");
        LOG_ERROR(TAG, "   - For Honda Civic 2018: Engine should be running");
    }
}

// Check for ECU disconnection based on error patterns
void check_ecu_disconnection(void) {
    if (!ecu_connected) {
        return;  // Already disconnected
    }
    
    // NOTE: ECU disconnection is now handled by the adaptive polling system
    // in obd_data.c which only triggers after multiple consecutive errors
    // at maximum delay (500ms). This provides more intelligent ECU health monitoring.
    
    // Legacy disconnection logic disabled - adaptive polling handles this better
    return;
    
    // Check if we've exceeded disconnection thresholds
    if (consecutive_ecu_failures >= MAX_CONSECUTIVE_FAILURES || 
        unable_to_connect_count >= MAX_UNABLE_TO_CONNECT ||
        can_error_count >= MAX_CAN_ERRORS) {
        
        LOG_WARN(TAG, "🔴 ECU DISCONNECTION DETECTED!");
        LOG_WARN(TAG, "├─ Consecutive failures: %d/%d", consecutive_ecu_failures, MAX_CONSECUTIVE_FAILURES);
        LOG_WARN(TAG, "├─ Unable to connect: %d/%d", unable_to_connect_count, MAX_UNABLE_TO_CONNECT);
        LOG_WARN(TAG, "├─ CAN errors: %d/%d", can_error_count, MAX_CAN_ERRORS);
        
        reset_ecu_connection();
    }
}

// Reset ECU connection and start reconnection process
void reset_ecu_connection(void) {
    LOG_WARN(TAG, "🔄 Resetting ECU connection - will attempt reconnection");
    
    ecu_connected = false;
    consecutive_ecu_failures = 0;
    unable_to_connect_count = 0;
    can_error_count = 0;
    
    LOG_INFO(TAG, "🔗 Starting ECU reconnection process...");
    
    xTaskCreate(ecu_reconnection_task, "ecu_reconnect", 6144, NULL, 5, NULL);
}

// ECU reconnection task
void ecu_reconnection_task(void *pv) {
    LOG_INFO(TAG, "ECU reconnection task started...");
    
    if (!is_connected) {
        LOG_WARN(TAG, "🔴 Bluetooth disconnected - aborting ECU reconnection");
        vTaskDelete(NULL);
        return;
    }
    
    verify_ecu_connection();
    vTaskDelete(NULL);
}

// Reset ECU error counters on successful data reception
void reset_ecu_error_counters(void) {
    consecutive_ecu_failures = 0;
    // Gradually reduce error counts on success (but keep some history for resilience)
    if (unable_to_connect_count > 0) unable_to_connect_count--;
    if (can_error_count > 0) can_error_count--;
}

// Gentle ELM327 initialization optimized for Honda Civic 2018
void initialize_elm327(void) {
    LOG_ELM(TAG, "Starting Honda-optimized ELM327 initialization...");
    
    // Wait for ELM327 to settle after connection
    LOG_ELM(TAG, "Waiting 3 seconds for ELM327 to settle...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Reset command
    LOG_ELM(TAG, "Sending ATZ (Reset)...");
    esp_err_t ret = elm327_send_command("ATZ");
    if (ret != ESP_OK) {
        LOG_WARN(TAG, "Failed to send ATZ");
        initialization_in_progress = false;
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for reset
    
    // Simple initialization sequence optimized for Honda
    LOG_ELM(TAG, "Configuring ELM327 for Honda Civic 2018...");
    
    // Minimal initialization sequence
    LOG_ELM(TAG, "Sending ATE0 (Echo OFF)...");
    elm327_send_command("ATE0");
    vTaskDelay(pdMS_TO_TICKS(1000));

    LOG_ELM(TAG, "Sending ATL0 (Linefeeds OFF)...");
    elm327_send_command("ATL0");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    LOG_ELM(TAG, "Sending ATS0 (Spaces OFF)...");
    elm327_send_command("ATS0");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    LOG_ELM(TAG, "Sending ATH0 (Headers OFF)...");
    elm327_send_command("ATH0");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    LOG_ELM(TAG, "Sending ATSP0 (Auto protocol)...");
    elm327_send_command("ATSP0");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Test voltage
    LOG_ELM(TAG, "Testing with AT RV (voltage check)...");
    elm327_send_command("AT RV");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Check protocol
    LOG_ELM(TAG, "Checking protocol with AT DPN...");
    elm327_send_command("AT DPN");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Test OBD
    LOG_ELM(TAG, "Testing OBD with 0100 (Supported PIDs)...");
    elm327_send_command("0100");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    LOG_ELM(TAG, "Honda Civic 2018 troubleshooting:");
    LOG_ELM(TAG, "1. Engine MUST be running (not just ignition ON)");
    LOG_ELM(TAG, "2. Press accelerator or turn on A/C to wake up PCM");
    LOG_ELM(TAG, "3. Hood should be open (Honda security feature)");
    LOG_ELM(TAG, "4. Wait 15s after engine start (PCM initialization)");
    LOG_ELM(TAG, "5. Try cycling ignition: OFF (10s) -> ON -> Start Engine");
    LOG_ELM(TAG, "6. Check raw CAN frames in logs for bus activity");
    
    // Mark as initialized
    elm327_initialized = true;
    elm_ready = true;
    // Note: ECU status will be set by verify_ecu_connection() when actually connected
    LOG_INFO(TAG, "ELM327 initialization complete - diagnostics above show readiness!");
    
    if (connection_semaphore != NULL) {
        LOG_INFO(TAG, "📡 Giving semaphore %p - ELM327 ready", connection_semaphore);
        xSemaphoreGive(connection_semaphore);
    } else {
        LOG_WARN(TAG, "⚠️ Skipping semaphore give - connection_semaphore not initialized (test mode?)");
    }
    
    verify_ecu_connection();
}

// ELM327 initialization task (runs in separate thread)
void initialize_elm327_task(void *pv) {
    LOG_ELM(TAG, "ELM327 initialization task started...");
    LOG_INFO(TAG, "🚀 === ELM327 INITIALIZATION START ===");
    LOG_INFO(TAG, "📋 VEEPEAK should be responsive after disable/enable sequence");
    
    // FIXED: Removed Phase 1 ATZ testing - old version didn't have it, directly called initialize_elm327
    // Additional stabilization after notification re-enable
    LOG_INFO(TAG, "⏱️ Waiting for VEEPEAK to stabilize after notification reset...");
    vTaskDelay(pdMS_TO_TICKS(1000));  // Reduced from 5000ms
    
    // Perform gentle initialization like old version
    initialize_elm327();
    
    // FIXED: Removed Phase 2 comprehensive init - merged into initialize_elm327
    // Delete this task when done
    vTaskDelete(NULL);
}