#include "esp_log.h"
#include <string.h>
#include <stdio.h>

#include "logging_config.h"
#include "elm327.h"
#include "bluetooth.h"
#include "obd_data.h"
#include "gpio_control.h"

// External declarations from bluetooth.h
extern uint16_t tx_char_handle;

static const char *TAG = "ELM327";

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
        
        if (strstr(response, "41 00") != NULL) {
            ecu_test_response_received = true;
            LOG_DEBUG(TAG, "🎯 ECU test response detected: %s", response);
        }
        
        process_obd_response(response);
    }
}

// Handle ELM327 response received via BLE (bridge function)
void handle_elm327_response(const char *data, size_t len) {
    // Convert size_t to uint16_t for existing function
    uint16_t data_len = (len > UINT16_MAX) ? UINT16_MAX : (uint16_t)len;
    
    // Forward to existing data processing function
    process_received_data(data, data_len);
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
    
    LOG_DEBUG(TAG, "Raw data received (%d bytes): %.*s", len, len, data);
    
    // Add received data to buffer
    for (uint16_t i = 0; i < len && rx_buffer_len < (RX_BUFFER_SIZE - 1); i++) {
        char c = data[i];
        
        if (c == '\r') {
            if (rx_buffer_len > 0) {
                rx_buffer[rx_buffer_len] = '\0';
                LOG_DEBUG(TAG, "Processing response (len=%d): '%s'", rx_buffer_len, rx_buffer);
                elm327_handle_response(rx_buffer);
                rx_buffer_len = 0;
                memset(rx_buffer, 0, RX_BUFFER_SIZE);
            }
        } else if (c == '\n') {
            LOG_DEBUG(TAG, "Ignoring LF character");
            continue;
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
    
    esp_err_t ret = elm327_send_command_with_options("0100", false);
    if (ret != ESP_OK) {
        LOG_WARN(TAG, "❌ Failed to send ECU test command");
        return false;
    }
    
    TickType_t start_time = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(4000);
    
    while ((xTaskGetTickCount() - start_time) < timeout) {
        vTaskDelay(pdMS_TO_TICKS(50));
        
        if (ecu_test_response_received) {
            LOG_INFO(TAG, "✅ ECU connection verified! Supported PIDs detected");
            return true;
        }
        
        if (rx_buffer_len > 5) {
            if (strstr(rx_buffer, "UNABLE TO CONNECT") != NULL ||
                strstr(rx_buffer, "CAN ERROR") != NULL ||
                strstr(rx_buffer, "NO DATA") != NULL) {
                return false;
            }
            
            if (strstr(rx_buffer, "SEARCHING") != NULL) {
                memset(rx_buffer, 0, RX_BUFFER_SIZE);
                rx_buffer_len = 0;
                continue;
            }
        }
    }
    
    LOG_DEBUG(TAG, "❌ ECU test timeout - no valid response received");
    return false;
}

// Verify ECU connection with retries
void verify_ecu_connection(void) {
    LOG_INFO(TAG, "🔗 === ECU CONNECTION VERIFICATION START ===");
    LOG_INFO(TAG, "📡 Testing connection to vehicle ECU...");
    
    ecu_connected = false;
    int attempt = 1;
    
    while (!ecu_connected) {
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

// Gentle ELM327 initialization
void initialize_elm327(void) {
    initialization_in_progress = true;
    
    connection_semaphore = xSemaphoreCreateBinary();
    if (connection_semaphore == NULL) {
        LOG_ERROR(TAG, "❌ Failed to create connection semaphore");
        initialization_in_progress = false;
        return;
    }
    
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    rx_buffer_len = 0;
    
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    esp_err_t ret = elm327_send_command("ATZ");
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "❌ Failed to send ATZ");
        initialization_in_progress = false;
        return;
    }
    
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ret = elm327_send_command("ATE0");
    if (ret != ESP_OK) {
        LOG_ERROR(TAG, "❌ Failed to send ATE0");
        initialization_in_progress = false;
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Send initialization commands
    const char *init_cmds[] = {
        "AT SP 0", "AT CRA", "AT AL", "AT SH 7DF", "AT CAF1",
        "AT ST 32", "ATH0", "AT RV", "AT DPN", "0100"
    };
    
    for (size_t i = 0; i < sizeof(init_cmds)/sizeof(init_cmds[0]); i++) {
        elm327_send_command(init_cmds[i]);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Try specific protocols
    elm327_send_command("AT SP 6");
    vTaskDelay(pdMS_TO_TICKS(500));
    elm327_send_command("0100");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    elm327_send_command("AT SP 7");
    vTaskDelay(pdMS_TO_TICKS(500));
    elm327_send_command("0100");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    elm327_send_command("AT SP 0");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    elm327_initialized = true;
    elm_ready = true;
    set_ecu_status(true);
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
    
    // Additional stabilization after notification re-enable (matching nRF Connect timing)
    LOG_INFO(TAG, "⏱️ Waiting 5 seconds for VEEPEAK to stabilize after notification reset...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Test standard command formats (should work after disable/enable sequence)
    LOG_INFO(TAG, "🔄 Phase 1: Testing ATZ commands (post-CCCD reset)...");
    
    const char *cmd_formats[] = {"ATZ\r", "ATZ\r\n", "ATZ\n"};
    const uint8_t max_retries = 3;
    bool response_received = false;
    
    for (size_t fmt_idx = 0; fmt_idx < sizeof(cmd_formats)/sizeof(cmd_formats[0]) && !response_received; fmt_idx++) {
        const char *cmd = cmd_formats[fmt_idx];
        LOG_INFO(TAG, "🔍 Testing command format %zu/3: %s", fmt_idx + 1, 
                fmt_idx == 0 ? "ATZ\\r\\n" : (fmt_idx == 1 ? "ATZ\\r" : "ATZ\\n"));
        
        for (uint8_t retry = 0; retry < max_retries && !response_received; retry++) {
            LOG_INFO(TAG, "🔄 Sending command (attempt %d/%d)...", retry + 1, max_retries);
            
            // Clear response flag before sending
            response_received_flag = false;
            elm_ready = false;
            
            esp_err_t ret = ble_uart_write((uint8_t *)cmd, strlen(cmd));
            if (ret != ESP_OK) {
                LOG_ERROR(TAG, "❌ Failed to send command: %s", esp_err_to_name(ret));
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            
            LOG_INFO(TAG, "✅ Command sent, waiting for response...");
            
            // Wait for response (up to 30 seconds per attempt - nRF Connect showed ~21s delay)
            TickType_t start_time = xTaskGetTickCount();
            TickType_t timeout = pdMS_TO_TICKS(30000);  // 30 second timeout per attempt
            
            while (xTaskGetTickCount() - start_time < timeout) {
                            if (response_received_flag) {
                response_received = true;
                LOG_INFO(TAG, "🎉 SUCCESS! VEEPEAK responded after CCCD reset! Command format: %s", 
                        fmt_idx == 0 ? "\\r" : (fmt_idx == 1 ? "\\r\\n" : "\\n"));
                LOG_INFO(TAG, "📋 Expected response patterns: '?' (error), 'ELM327 v2.x' (success), '>' (prompt)");
                break;
            }
                vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100ms
            }
            
            if (!response_received && retry < max_retries - 1) {
                LOG_WARN(TAG, "⚠️ No response to command, retrying...");
                vTaskDelay(pdMS_TO_TICKS(1000));  // Wait 1 second before retry
            }
        }
        
        if (!response_received) {
            LOG_WARN(TAG, "⚠️ No response with format: %s", 
                    fmt_idx == 0 ? "\\r\\n" : (fmt_idx == 1 ? "\\r" : "\\n"));
        }
    }
    
    // Try alternative commands if ATZ failed
    if (!response_received) {
        LOG_WARN(TAG, "⚠️ ATZ failed with all formats, trying alternative commands...");
        const char *alt_cmds[] = {"ATI\r", "ATI\r\n", "AT@1\r", "AT@1\r\n"};
        
        for (size_t i = 0; i < sizeof(alt_cmds)/sizeof(alt_cmds[0]) && !response_received; i++) {
            LOG_INFO(TAG, "🔄 Sending alternative command: %s", alt_cmds[i]);
            
            response_received_flag = false;
            esp_err_t ret = ble_uart_write((uint8_t *)alt_cmds[i], strlen(alt_cmds[i]));
            if (ret == ESP_OK) {
                TickType_t start_time = xTaskGetTickCount();
                while (!response_received_flag && (xTaskGetTickCount() - start_time < pdMS_TO_TICKS(30000))) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                if (response_received_flag) {
                    response_received = true;
                    LOG_INFO(TAG, "✅ Response received to alternative command: %s", alt_cmds[i]);
                }
            }
        }
    }
    
    if (!response_received) {
        LOG_ERROR(TAG, "❌ Failed to get response after all attempts (ATZ + alternatives)");
        LOG_ERROR(TAG, "💡 This indicates a communication problem:");
        LOG_ERROR(TAG, "   - VEEPEAK device may not be sending notifications");
        LOG_ERROR(TAG, "   - BLE GATT notifications may not be working"); 
        LOG_ERROR(TAG, "   - Device may require different initialization sequence");
        LOG_ERROR(TAG, "   - Vehicle OBD-II port may be inactive (ignition off, engine not running)");
        LOG_ERROR(TAG, "   - OBD-II connector may be loose or faulty");
        LOG_ERROR(TAG, "   - VEEPEAK firmware may be incompatible with this approach");
        goto initialization_failed;
    }
    
    // If we got a response, continue with full initialization
    LOG_INFO(TAG, "🎉 SUCCESS! Basic communication verified - proceeding with full initialization");
    
    // Send comprehensive initialization commands
    LOG_INFO(TAG, "🔧 Phase 2: Sending complete ELM327 initialization sequence...");
    
    const char *init_cmds[] = {
        "ATE0\r",       // Turn off echo
        "AT SP 0\r",    // Auto protocol detection
        "AT CRA\r",     // Reset all filters
        "AT AL\r",      // Allow long frames
        "AT SH 7DF\r",  // Set broadcast address
        "AT CAF1\r",    // Enable auto-format ISO-TP
        "AT ST 32\r",   // Set shorter timeout (50ms)
        "ATH0\r",       // Headers off
        "AT RV\r",      // Read voltage (test command)
        "AT DPN\r",     // Check detected protocol
        "0100\r"        // Basic OBD test - get supported PIDs
    };
    
    const char *cmd_descriptions[] = {
        "Disabling echo", "Setting auto protocol", "Resetting filters", "Enabling long frames",
        "Setting broadcast address", "Enabling auto-format", "Setting timeout", "Disabling headers",
        "Reading voltage", "Checking protocol", "Testing OBD (supported PIDs)"
    };
    
    for (size_t i = 0; i < sizeof(init_cmds)/sizeof(init_cmds[0]); i++) {
        LOG_INFO(TAG, "├─ %s (%s)...", cmd_descriptions[i], init_cmds[i]);
        response_received_flag = false;
        
        esp_err_t ret = ble_uart_write((uint8_t *)init_cmds[i], strlen(init_cmds[i]));
        if (ret == ESP_OK) {
            // Wait for response with longer timeout for all commands (nRF Connect showed ~21s delay)
            uint32_t timeout_ms = 30000;  // 30s for all commands based on nRF Connect findings
            TickType_t start_time = xTaskGetTickCount();
            
            while (!response_received_flag && (xTaskGetTickCount() - start_time < pdMS_TO_TICKS(timeout_ms))) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
            if (response_received_flag) {
                LOG_INFO(TAG, "├─ ✅ Response received");
            } else {
                LOG_WARN(TAG, "├─ ⚠️ No response, but continuing...");
            }
        } else {
            LOG_ERROR(TAG, "├─ ❌ Failed to send command: %s", esp_err_to_name(ret));
        }
        
        // Small delay between commands
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    // Mark as initialized
    elm327_initialized = true;
    elm_ready = true;
    set_ecu_status(true);
    initialization_in_progress = false;
    LOG_INFO(TAG, "✅ === ELM327 INITIALIZATION COMPLETE ===");
    
    // FIXED: Signal that ELM327 is ready (with NULL check to prevent crash)
    if (connection_semaphore != NULL) {
        LOG_INFO(TAG, "📡 Giving semaphore %p - ELM327 task complete", connection_semaphore);
        xSemaphoreGive(connection_semaphore);
    } else {
        LOG_WARN(TAG, "⚠️ Skipping semaphore give - connection_semaphore not initialized (test mode?)");
    }
    
    // Now verify connection to vehicle ECU
    verify_ecu_connection();
    
    // Delete this task when done
    vTaskDelete(NULL);
    
initialization_failed:
    LOG_ERROR(TAG, "❌ ELM327 initialization failed");
    initialization_in_progress = false;  // Clear flag on failure
    elm327_initialized = false;
    vTaskDelete(NULL);
} 