#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "logging_config.h"
#include "obd_data.h"
#include "elm327.h"
#include "bluetooth.h"
#include "gpio_control.h"

static const char *TAG = "OBD_DATA";

// Adaptive polling delay constants
#define MIN_COMMAND_DELAY_MS 175        // Minimum delay between commands
#define MAX_COMMAND_DELAY_MS 500        // Maximum delay between commands  
#define DELAY_INCREASE_MS 50            // Increase delay by this amount on failure
#define DELAY_DECREASE_MS 5            // Decrease delay by this amount on success
#define RESPONSE_TIMEOUT_MS 1000        // Wait up to 1 second for response
#define MAX_ERRORS_AT_MAX_DELAY 10      // Consider ECU disconnected after this many errors at max delay

#define HEXBYTE_TO_INT(ptr)  ((uint8_t)strtol((ptr), NULL, 16))

// Global vehicle data instance
vehicle_data_t vehicle_data = {
    .rpm = 0,
    .throttle_position = 0,
    .vehicle_speed = 0
};

// Timestamp tracking for data freshness (in FreeRTOS ticks)
static volatile TickType_t rpm_last_update = 0;
static volatile TickType_t throttle_last_update = 0;
static volatile TickType_t speed_last_update = 0;

// PID data length lookup (in hex chars, including PID itself)
// returns number of hex chars in DATA *excluding* the PID byte
static size_t get_pid_data_length(uint8_t pid) {
    switch (pid) {
        case 0x0C: return 4;  // RPM: 2 data bytes (4 hex chars)
        case 0x0D: return 2;  // Speed: 1 data byte
        case 0x11: return 2;  // Throttle: 1 data byte
        case 0x0F: return 2;  // IAT: 1 data byte
        case 0x05: return 2;  // ECT: 1 data byte
        case 0x0B: return 2;  // MAP: 1 data byte
        case 0x10: return 4;  // MAF: 2 data bytes (4 hex chars)
        case 0x0E: return 2;  // Timing Advance: 1 data byte
        default:   return 2;  // Default: 1 data byte
    }
}

// Parse multi-PID response line
void parse_multi_pid_line(char *line)
{
    /* Example CAN frame response:
       First line:    "008"            - Number of data bytes in hex
       Frame 0:       "0:410C0B381122" - Mode 41 + RPM (0C 0B38) + Throttle (11 22)
       Frame 1:       "1:0D005555555555" - Speed (0D 00) + padding
       
       Frame format:
       41 0C 0B 38 11 22 - Frame 0
       ^^ ^^ ^^ ^^ ^^ ^^
       |  |  |____|  |__|
       |  |    RPM   Throttle (PID 11, value 22)
       |  PID 0C
       Mode 41

       0D 00 55 55 55 55 - Frame 1
       ^^ ^^ ^^
       |  |  padding
       |  Speed value (00)
       PID 0D */

    static int expected_bytes = 0;  // Total hex chars we expect (2 per byte)
    static int received_bytes = 0;  // Hex chars received so far
    bool data_parsed = false;       // Track if any valid data was parsed
    char hex[3] = {0};             // Buffer for 2 hex chars + null terminator

    // Check if this is a byte count line (e.g., "008")
    // Only treat as byte count if ≤3 chars AND all hex digits
    if (strlen(line) <= 3 && 
        strspn(line, "0123456789ABCDEFabcdef") == strlen(line)) {
        expected_bytes = strtol(line, NULL, 16);  // Read as hex
        expected_bytes *= 2;  // Convert to hex char count
        received_bytes = 0;   // Reset counter for new response
        LOG_DEBUG(TAG, "Expecting %d hex chars of OBD data", expected_bytes);
        return;
    }

    // Skip line number prefix if present (e.g., "0:" or "1:")
    if (line[0] >= '0' && line[0] <= '9' && line[1] == ':') {
        line += 2;  // Skip "0:" or "1:" prefix
    }

    // Skip any leading spaces
    while (*line == ' ') line++;

    // Check for Mode 1 response in first frame
    if (strncmp(line, "41", 2) == 0) {
        // Skip the mode byte (41)
        line += 2;
        received_bytes += 2;

        // Process the rest of the frame
        while (strlen(line) >= 2) {  // Need at least PID
            // Exit if we've received all expected bytes
            if (expected_bytes > 0 && received_bytes >= expected_bytes) break;

            // Get PID
            strncpy(hex, line, 2); hex[2]='\0';
            uint8_t pid = HEXBYTE_TO_INT(hex);
            line += 2;
            received_bytes += 2;

            // Get data length for this PID
            size_t data_len = get_pid_data_length(pid);
            if (strlen(line) < data_len) break;  // Not enough data

            switch (pid) {
                case 0x0C: {  // RPM (needs two bytes)
                    strncpy(hex, line, 2); hex[2]='\0';
                    uint8_t msb = HEXBYTE_TO_INT(hex);
                    strncpy(hex, line + 2, 2); hex[2]='\0';
                    uint8_t lsb = HEXBYTE_TO_INT(hex);
                    uint16_t raw = (msb << 8) | lsb;
                    vehicle_data.rpm = raw / 4;
                    rpm_last_update = xTaskGetTickCount();
                    data_parsed = true;
                    LOG_DEBUG(TAG, "RPM updated: %lu (from %02X %02X)", 
                             vehicle_data.rpm, msb, lsb);
                    break;
                }
                case 0x11: {  // Throttle position (1 byte)
                    strncpy(hex, line, 2); hex[2]='\0';
                    uint8_t value = HEXBYTE_TO_INT(hex);
                    vehicle_data.throttle_position = (value * 100) / 255;
                    throttle_last_update = xTaskGetTickCount();
                    data_parsed = true;
                    LOG_DEBUG(TAG, "Throttle updated: %d%% (from %02X)", 
                             vehicle_data.throttle_position, value);
                    break;
                }
                case 0x0D: {  // Vehicle speed (1 byte)
                    strncpy(hex, line, 2); hex[2]='\0';
                    uint8_t value = HEXBYTE_TO_INT(hex);
                    vehicle_data.vehicle_speed = value;
                    speed_last_update = xTaskGetTickCount();
                    data_parsed = true;
                    LOG_DEBUG(TAG, "Speed updated: %d km/h (from %02X)", value, value);
                    break;
                }
                default:
                    LOG_DEBUG(TAG, "Skipping unknown PID %02X", pid);
                    break;
            }

            // Move past the data bytes
            line += data_len;
            received_bytes += data_len;
        }
    } else {
        // Process second frame (no mode byte)
        while (strlen(line) >= 2) {  // Need at least PID
            // Exit if we've received all expected bytes
            if (expected_bytes > 0 && received_bytes >= expected_bytes) break;

            // Get PID
            strncpy(hex, line, 2); hex[2]='\0';
            uint8_t pid = HEXBYTE_TO_INT(hex);
            line += 2;
            received_bytes += 2;

            // Get data length for this PID
            size_t data_len = get_pid_data_length(pid);
            if (strlen(line) < data_len) break;  // Not enough data

            switch (pid) {
                case 0x0D: {  // Vehicle speed (1 byte)
                    strncpy(hex, line, 2); hex[2]='\0';
                    uint8_t value = HEXBYTE_TO_INT(hex);
                    vehicle_data.vehicle_speed = value;
                    speed_last_update = xTaskGetTickCount();
                    data_parsed = true;
                    LOG_DEBUG(TAG, "Speed updated: %d km/h (from %02X)", 
                             vehicle_data.vehicle_speed, value);
                    break;
                }
                default:
                    LOG_DEBUG(TAG, "Skipping unknown PID %02X", pid);
                    break;
            }

            // Move past the data bytes
            line += data_len;
            received_bytes += data_len;
        }
    }

    // Reset ECU error counters if any data was successfully parsed
    if (data_parsed) {
        reset_ecu_error_counters();
    }
}

// Send OBD command with adaptive delay and response timeout
int send_obd_command_adaptive(const char *cmd, uint16_t *current_delay_ms, uint8_t *errors_at_max_delay) {
    LOG_DEBUG(TAG, "📤 Sending command '%s' with %dms delay", cmd, *current_delay_ms);
    
    // Track timestamps for response detection
    TickType_t cmd_start_time = xTaskGetTickCount();
    TickType_t last_data_time = 0;
    
    // Get current data timestamps before sending command
    TickType_t rpm_before = rpm_last_update;
    TickType_t throttle_before = throttle_last_update;
    TickType_t speed_before = speed_last_update;
    
    // Reset response flag before sending command
    response_received_flag = false;
    
    // Send the command
    esp_err_t ret = elm327_send_command(cmd);
    if (ret != ESP_OK) {
        LOG_WARN(TAG, "❌ Failed to send command '%s'", cmd);
        return 0;
    }
    
    // Wait for response with timeout
    TickType_t timeout = pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS);
    bool response_received = false;
    bool data_updated = false;
    
    while ((xTaskGetTickCount() - cmd_start_time) < timeout) {
        // Check if any OBD data was updated after command was sent
        if (rpm_last_update > rpm_before || 
            throttle_last_update > throttle_before || 
            speed_last_update > speed_before) {
            response_received = true;
            data_updated = true;
            last_data_time = xTaskGetTickCount();
            LOG_DEBUG(TAG, "✅ OBD data updated after command '%s'", cmd);
            break;
        }
        
        // Check if any response was received (good or bad)
        if (response_received_flag) {
            // Give a small delay to ensure data parsing completes
            vTaskDelay(pdMS_TO_TICKS(20));
            
            // Check again if data was updated after the delay
            if (rpm_last_update > rpm_before || 
                throttle_last_update > throttle_before || 
                speed_last_update > speed_before) {
                response_received = true;
                data_updated = true;
                last_data_time = xTaskGetTickCount();
                LOG_DEBUG(TAG, "✅ OBD data updated after delay for command '%s'", cmd);
            } else {
                response_received = true;
                last_data_time = xTaskGetTickCount();
                LOG_DEBUG(TAG, "✅ Response received for command '%s' (may be error)", cmd);
            }
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // Check every 10ms
    }
    
    if (response_received) {
        if (data_updated) {
            // True success - valid data received, decrease delay (speed up)
            if (*current_delay_ms > MIN_COMMAND_DELAY_MS) {
                *current_delay_ms -= DELAY_DECREASE_MS;
                if (*current_delay_ms < MIN_COMMAND_DELAY_MS) {
                    *current_delay_ms = MIN_COMMAND_DELAY_MS;
                }
                LOG_DEBUG(TAG, "⚡ Valid data received in %dms, decreasing delay to %dms", 
                        (int)pdTICKS_TO_MS(last_data_time - cmd_start_time), *current_delay_ms);
            }
            *errors_at_max_delay = 0;  // Reset error count on success
            return 1;
        } else {
            // Response received but no data (error response like NO DATA, CAN ERROR)
            LOG_WARN(TAG, "⚠️ Error response to '%s' in %dms - increasing delay", 
                    cmd, (int)pdTICKS_TO_MS(last_data_time - cmd_start_time));
            
            // Treat error responses as failures - increase delay
            if (*current_delay_ms < MAX_COMMAND_DELAY_MS) {
                *current_delay_ms += DELAY_INCREASE_MS;
                if (*current_delay_ms > MAX_COMMAND_DELAY_MS) {
                    *current_delay_ms = MAX_COMMAND_DELAY_MS;
                }
                LOG_DEBUG(TAG, "🐌 Increasing delay to %dms due to error response", *current_delay_ms);
            } else {
                // Already at max delay, count errors
                (*errors_at_max_delay)++;
                LOG_WARN(TAG, "📊 Error at max delay: %d/%d", *errors_at_max_delay, MAX_ERRORS_AT_MAX_DELAY);
                
                // Check if we should consider ECU disconnected
                if (*errors_at_max_delay >= MAX_ERRORS_AT_MAX_DELAY) {
                    LOG_ERROR(TAG, "🔴 Too many errors at maximum delay - ECU may be disconnected");
                    reset_ecu_connection();
                    return 0;
                }
            }
            return 0;  // Return failure for error responses
        }
    } else {
        // Timeout - increase delay (slow down)
        LOG_WARN(TAG, "⏰ No response to '%s' within %dms", cmd, RESPONSE_TIMEOUT_MS);
        
        if (*current_delay_ms < MAX_COMMAND_DELAY_MS) {
            *current_delay_ms += DELAY_INCREASE_MS;
            if (*current_delay_ms > MAX_COMMAND_DELAY_MS) {
                *current_delay_ms = MAX_COMMAND_DELAY_MS;
            }
            LOG_DEBUG(TAG, "🐌 Increasing delay to %dms", *current_delay_ms);
        } else {
            // Already at max delay, count errors
            (*errors_at_max_delay)++;
            LOG_WARN(TAG, "📊 Error at max delay: %d/%d", *errors_at_max_delay, MAX_ERRORS_AT_MAX_DELAY);
            
            // Check if we should consider ECU disconnected
            if (*errors_at_max_delay >= MAX_ERRORS_AT_MAX_DELAY) {
                LOG_ERROR(TAG, "🔴 Too many errors at maximum delay - ECU may be disconnected");
                reset_ecu_connection();
                return 0;
            }
        }
        return 0;
    }
}

// OBD data polling task
void obd_task(void *pv) {
    restart_obd_task:
    LOG_VERBOSE(TAG, "OBD Task started - waiting for Bluetooth connection...");
    
    while (1) {
        // Check if Bluetooth is connected
        if (!is_connected) {
            LOG_INFO(TAG, "⏳ Waiting for Bluetooth connection...");
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        
        // Wait for ELM327 to be connected and initialized
        if (connection_semaphore != NULL) {
            if (xSemaphoreTake(connection_semaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
                LOG_INFO(TAG, "ELM327 initialized, waiting for ECU connection...");
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Wait for ECU connection to be established
    while (!ecu_connected && is_connected) {
        LOG_INFO(TAG, "⏳ Waiting for ECU connection to be established...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    
    if (!is_connected) {
        LOG_WARN(TAG, "🔴 Bluetooth disconnected during ECU wait - restarting");
        vTaskDelay(pdMS_TO_TICKS(1000));
        goto restart_obd_task;
    }
    
    LOG_INFO(TAG, "🚗 ECU connected - Starting OBD data polling...");
    
    // Optimized OBD Data Polling - Single request for all PIDs
    LOG_INFO(TAG, "🚀 Starting optimized OBD polling system");
    LOG_INFO(TAG, "📊 Single request strategy: 010C110D (RPM + Throttle + Speed)");
    
    static TickType_t last_ecu_check = 0;

    // Adaptive polling delay management
    static uint16_t current_delay_ms = MIN_COMMAND_DELAY_MS;
    static uint8_t errors_at_max_delay = 0;
    
    while (1) {
        // Connection checks remain the same
        if (!is_connected) {
            LOG_WARN(TAG, "🔴 Bluetooth disconnected - stopping OBD polling");
            last_ecu_check = 0;
            current_delay_ms = MIN_COMMAND_DELAY_MS;
            errors_at_max_delay = 0;
            
            while (!is_connected) {
                LOG_INFO(TAG, "⏳ Waiting for Bluetooth reconnection...");
                vTaskDelay(pdMS_TO_TICKS(3000));
            }
            
            LOG_INFO(TAG, "✅ Bluetooth reconnected - waiting for ELM327 initialization");
            continue;
        }
        
        // ECU connection check remains the same
        if (!ecu_connected) {
            LOG_WARN(TAG, "🔴 ECU disconnected during polling - waiting for reconnection...");
            while (!ecu_connected && is_connected) {
                LOG_INFO(TAG, "⏳ Waiting for ECU reconnection...");
                vTaskDelay(pdMS_TO_TICKS(3000));
            }
            
            if (!is_connected) {
                continue;
            }
            
            LOG_INFO(TAG, "✅ ECU reconnected - resuming OBD data polling");
            last_ecu_check = xTaskGetTickCount();
            current_delay_ms = MIN_COMMAND_DELAY_MS;
            errors_at_max_delay = 0;
        }
        
        // Periodic ECU check remains the same
        TickType_t current_time = xTaskGetTickCount();
        if ((current_time - last_ecu_check) > pdMS_TO_TICKS(30000)) {
            LOG_DEBUG(TAG, "🔍 Performing periodic ECU connectivity check...");
            check_ecu_disconnection();
            last_ecu_check = current_time;
        }
        
        if (is_connected && elm327_initialized) {
            // Always send multi-PID request (RPM + Throttle + Speed)
            send_obd_command_adaptive("010C110D", &current_delay_ms, &errors_at_max_delay);
            
            // Use adaptive delay
            vTaskDelay(pdMS_TO_TICKS(current_delay_ms));
            
            // Check for stale data
            // check_and_reset_stale_data(use_individual_pids); // This function is no longer needed
            
            // Log vehicle status after each cycle
            log_vehicle_status();
        } else {
            LOG_INFO(TAG, "⏳ Waiting for ELM327 connection...");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

// Initialize OBD data system
void obd_data_init(void) {
    // Reset vehicle data to defaults
    vehicle_data.rpm = 0;
    vehicle_data.throttle_position = 0;
    vehicle_data.vehicle_speed = 0;
    
    // Initialize timestamps to current time
    TickType_t current_time = xTaskGetTickCount();
    rpm_last_update = current_time;
    throttle_last_update = current_time;
    speed_last_update = current_time;
    
    LOG_VERBOSE(TAG, "OBD data system initialized");
}


// Display current vehicle data
void display_vehicle_data(void) {
    ESP_LOGI(TAG, "Vehicle Data: RPM=%lu | Throttle=%d%% | Speed=%d km/h", 
             vehicle_data.rpm,
             vehicle_data.throttle_position,
             vehicle_data.vehicle_speed);
}

// Log vehicle status
void log_vehicle_status(void) {
    ESP_LOGI(TAG, "RPM: %lu | Throttle: %d%% | Speed: %d km/h",
             vehicle_data.rpm,
             vehicle_data.throttle_position,
             vehicle_data.vehicle_speed);
    
    // Update NOS system remains
    update_nos_system(vehicle_data.rpm, vehicle_data.throttle_position, vehicle_data.vehicle_speed);
}

 