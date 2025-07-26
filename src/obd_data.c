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
static TickType_t rpm_last_update = 0;
static TickType_t throttle_last_update = 0;
static TickType_t speed_last_update = 0;

#define DATA_TIMEOUT_MS 500
#define DATA_TIMEOUT_TICKS pdMS_TO_TICKS(DATA_TIMEOUT_MS)

// Check for stale data and reset values older than timeout
static void check_and_reset_stale_data(bool using_individual_pids) {
    TickType_t current_time = xTaskGetTickCount();
    
    // Adjust timeout based on polling strategy
    TickType_t timeout = using_individual_pids ? 
        pdMS_TO_TICKS(1000) :  // 1 second for individual PIDs (750ms cycle + margin)
        pdMS_TO_TICKS(600);    // 600ms for multi-PID (500ms cycle + margin)
    
    // Check RPM freshness
    if ((current_time - rpm_last_update) > timeout) {
        if (vehicle_data.rpm != 0) {
            vehicle_data.rpm = 0;
            LOG_WARN(TAG, "⚠️ RPM data stale, reset to 0");
        }
    }
    
    // Check throttle freshness  
    if ((current_time - throttle_last_update) > timeout) {
        if (vehicle_data.throttle_position != 0) {
            vehicle_data.throttle_position = 0;
            LOG_WARN(TAG, "⚠️ Throttle data stale, reset to 0");
        }
    }
    
    // Check speed freshness
    if ((current_time - speed_last_update) > timeout) {
        if (vehicle_data.vehicle_speed != 0) {
            vehicle_data.vehicle_speed = 0;
            LOG_WARN(TAG, "⚠️ Speed data stale, reset to 0");
        }
    }
}

// Parse multi-PID response line
void parse_multi_pid_line(char *line)
{
    /* Example after trimming CR/LF + prompt:
       "41 0C 1A F8 41 0D 3C 41 11 5A" */

    // Handle lines that start with "0: " or similar prefixes
    char *data_start = strstr(line, "41 ");
    if (data_start == NULL) {
        return;   // not a Mode-01 reply
    }
    
    // Use the data starting from "41" instead of the beginning of the line
    line = data_start;

    char *tok = strtok(line, " ");
    
    // First token should be "41" (Mode 1 response)
    if (!tok || strcmp(tok, "41") != 0) {
        return;
    }
    
    bool data_parsed = false;  // Track if any valid data was parsed
    
    // Now parse PID+data pairs
    while ((tok = strtok(NULL, " ")) != NULL) {
        // Stop at filler bytes (ELM pad)
        if (strcmp(tok, "55") == 0) {
            break;
        }
        
        // This token should be a PID
        uint8_t pid_val = HEXBYTE_TO_INT(tok);
        
        char *data1 = strtok(NULL, " ");   // first data byte
        if (!data1) {
            break;
        }
        
        char *data2 = NULL;
        // Check if this PID needs two data bytes
        if (pid_val == 0x0C) {  // RPM needs 2 bytes
            data2 = strtok(NULL, " ");
        }
        
        switch (pid_val) {
            case 0x0C: {                   // RPM (needs two bytes)
                if (!data2) {
                    break;
                }
                uint16_t raw = (HEXBYTE_TO_INT(data1) << 8) |
                               HEXBYTE_TO_INT(data2);
                vehicle_data.rpm = raw / 4;
                rpm_last_update = xTaskGetTickCount(); // Update timestamp
                data_parsed = true; // Mark data as parsed
                break;
            }
            case 0x0D:                      // Vehicle speed (1 byte)
                vehicle_data.vehicle_speed = HEXBYTE_TO_INT(data1);
                speed_last_update = xTaskGetTickCount(); // Update timestamp
                data_parsed = true; // Mark data as parsed
                break;
            case 0x11:                      // Throttle position (1 byte)
                vehicle_data.throttle_position =
                    (HEXBYTE_TO_INT(data1) * 100) / 255;
                throttle_last_update = xTaskGetTickCount(); // Update timestamp
                data_parsed = true; // Mark data as parsed
                break;
            default:
                break;
        }
    }

    // Reset ECU error counters if any data was successfully parsed
    if (data_parsed) {
        reset_ecu_error_counters(); // Reset ECU disconnection error counters
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
    
    static bool use_individual_pids = false;
    static uint8_t individual_pid_cycles = 0;
    static TickType_t last_success_time = 0;
    static TickType_t last_ecu_check = 0;
    
    // Adaptive polling delay management
    static uint16_t current_delay_ms = MIN_COMMAND_DELAY_MS;
    static uint8_t errors_at_max_delay = 0;
    
    while (1) {
        // Connection checks remain the same
        if (!is_connected) {
            LOG_WARN(TAG, "🔴 Bluetooth disconnected - stopping OBD polling");
            use_individual_pids = false;
            individual_pid_cycles = 0;
            last_success_time = 0;
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
            use_individual_pids = false;
            individual_pid_cycles = 0;
            last_success_time = xTaskGetTickCount();
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
            // Check if we should switch to individual PIDs due to errors
            if ((current_time - last_success_time) > pdMS_TO_TICKS(5000)) {
                if (!use_individual_pids) {
                    LOG_WARN(TAG, "⚠️ Switching to individual PID requests due to errors");
                    use_individual_pids = true;
                    individual_pid_cycles = 0;
                }
            }
            
            if (use_individual_pids) {
                // Fallback: Individual PID requests with adaptive delay
                static uint8_t phase = 0;
                const char *cmd = NULL;
                switch (phase % 3) {
                    case 0: cmd = "010C"; break;  // RPM
                    case 1: cmd = "0111"; break;  // Throttle
                    case 2: cmd = "010D"; break;  // Speed
                }
                
                if (cmd) {
                    bool success = send_obd_command_adaptive(cmd, &current_delay_ms, &errors_at_max_delay);
                    if (success) {
                        last_success_time = xTaskGetTickCount();
                    }
                }
                
                phase = (phase + 1) % 3;
                
                if (phase == 0) {
                    individual_pid_cycles++;
                    if (individual_pid_cycles >= 3) {
                        LOG_INFO(TAG, "🔄 3 individual PID cycles completed - switching back to multi-PID mode");
                        use_individual_pids = false;
                        individual_pid_cycles = 0;
                        last_success_time = xTaskGetTickCount();
                    }
                }
            } else {
                // Optimized: Single multi-PID request for all data
                bool success = send_obd_command_adaptive("010C110D", &current_delay_ms, &errors_at_max_delay);
                if (success) {
                    last_success_time = xTaskGetTickCount();
                }
            }
            
            // Use adaptive delay
            vTaskDelay(pdMS_TO_TICKS(current_delay_ms));
            
            // Check for stale data
            check_and_reset_stale_data(use_individual_pids);
            
            // Log vehicle status after each cycle
            log_vehicle_status();
            
            // Update success time if we have valid data
            if (vehicle_data.rpm > 0 || vehicle_data.throttle_position > 0 || vehicle_data.vehicle_speed > 0) {
                last_success_time = current_time;
            }
        } else {
            LOG_INFO(TAG, "⏳ Waiting for ELM327 connection...");
            use_individual_pids = false;
            individual_pid_cycles = 0;
            last_success_time = 0;
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

 