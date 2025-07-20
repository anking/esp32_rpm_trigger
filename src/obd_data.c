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
            ESP_LOGW(TAG, "⚠️ RPM data stale, reset to 0");
        }
    }
    
    // Check throttle freshness  
    if ((current_time - throttle_last_update) > timeout) {
        if (vehicle_data.throttle_position != 0) {
            vehicle_data.throttle_position = 0;
            ESP_LOGW(TAG, "⚠️ Throttle data stale, reset to 0");
        }
    }
    
    // Check speed freshness
    if ((current_time - speed_last_update) > timeout) {
        if (vehicle_data.vehicle_speed != 0) {
            vehicle_data.vehicle_speed = 0;
            ESP_LOGW(TAG, "⚠️ Speed data stale, reset to 0");
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

// Log vehicle status with GPIO state
void log_vehicle_status(void) {
    const char* gpio_state = gpio_status ? "ON" : "OFF";
    ESP_LOGI(TAG, "RPM: %lu | Throttle: %d%% | Speed: %d km/h | GPIO2: %s",
             vehicle_data.rpm,
             vehicle_data.throttle_position,
             vehicle_data.vehicle_speed,
             gpio_state);
}

// OBD data polling task
void obd_task(void *pv) {
    restart_obd_task:
    LOG_VERBOSE(TAG, "OBD Task started - waiting for Bluetooth connection...");
    
    while (1) {
        // Check if Bluetooth is connected
        if (!is_connected) {
            ESP_LOGI(TAG, "⏳ Waiting for Bluetooth connection...");
            vTaskDelay(pdMS_TO_TICKS(3000));  // Check every 3 seconds
            continue;
        }
        
        // Wait for ELM327 to be connected and initialized
        if (connection_semaphore != NULL) {
            if (xSemaphoreTake(connection_semaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
                LOG_INFO(TAG, "ELM327 initialized, waiting for ECU connection...");
                break;  // ELM327 ready, now wait for ECU
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100ms
    }
    
    // Wait for ECU connection to be established
    while (!ecu_connected && is_connected) {  // Also check Bluetooth is still connected
        ESP_LOGI(TAG, "⏳ Waiting for ECU connection to be established...");
        vTaskDelay(pdMS_TO_TICKS(5000));  // Check every 5 seconds instead of 1 second
    }
    
    if (!is_connected) {
        ESP_LOGW(TAG, "🔴 Bluetooth disconnected during ECU wait - restarting");
        // Don't start OBD polling, will restart from the beginning
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Continue will restart the function from the beginning
        goto restart_obd_task;
    }
    
    LOG_INFO(TAG, "🚗 ECU connected - Starting OBD data polling...");
    
    // Optimized OBD Data Polling - Production Ready
    ESP_LOGI(TAG, "🚀 Starting optimized OBD polling system");
    ESP_LOGI(TAG, "📊 Two-phase strategy: 010C11 → 010D");
    
    static uint8_t phase = 0;
    static bool use_individual_pids = false;
    static TickType_t last_success_time = 0;
    static TickType_t last_ecu_check = 0;
    
    while (1) {
        // Check if Bluetooth is still connected
        if (!is_connected) {
            ESP_LOGW(TAG, "🔴 Bluetooth disconnected - stopping OBD polling");
            
            // Reset state variables when Bluetooth disconnects
            phase = 0;
            use_individual_pids = false;
            last_success_time = 0;
            last_ecu_check = 0;
            
            // Wait for Bluetooth reconnection
            while (!is_connected) {
                ESP_LOGI(TAG, "⏳ Waiting for Bluetooth reconnection...");
                vTaskDelay(pdMS_TO_TICKS(3000));  // Check every 3 seconds
            }
            
            ESP_LOGI(TAG, "✅ Bluetooth reconnected - waiting for ELM327 initialization");
            continue;  // Restart the main loop
        }
        
        // Check if ECU is still connected
        if (!ecu_connected) {
            ESP_LOGW(TAG, "🔴 ECU disconnected during polling - waiting for reconnection...");
            
            // Wait for ECU connection to be re-established
            while (!ecu_connected && is_connected) {  // Also check Bluetooth is still connected
                ESP_LOGI(TAG, "⏳ Waiting for ECU reconnection...");
                vTaskDelay(pdMS_TO_TICKS(3000));  // Check every 3 seconds
            }
            
            if (!is_connected) {
                continue;  // Bluetooth disconnected, restart main loop
            }
            
            ESP_LOGI(TAG, "✅ ECU reconnected - resuming OBD data polling");
            
            // Reset state variables after reconnection
            phase = 0;
            use_individual_pids = false;
            last_success_time = xTaskGetTickCount();
            last_ecu_check = xTaskGetTickCount();
        }
        
        // Periodic ECU connectivity check (every 30 seconds)
        TickType_t current_time = xTaskGetTickCount();
        if ((current_time - last_ecu_check) > pdMS_TO_TICKS(30000)) {
            ESP_LOGD(TAG, "🔍 Performing periodic ECU connectivity check...");
            check_ecu_disconnection();
            last_ecu_check = current_time;
        }
        
        if (is_connected && elm327_initialized) {
            
            // Check if we should switch to individual PIDs due to CAN errors
            TickType_t current_time = xTaskGetTickCount();
            if ((current_time - last_success_time) > pdMS_TO_TICKS(5000)) {
                if (!use_individual_pids) {
                    ESP_LOGW(TAG, "⚠️ Switching to individual PID requests due to errors");
                    use_individual_pids = true;
                }
            }
            
            // Adaptive polling strategy
            if (use_individual_pids) {
                // Fallback: Individual PID requests
                switch (phase % 3) {
                    case 0:
                        elm327_send_command("010C");  // RPM only
                        break;
                    case 1:
                        elm327_send_command("0111");  // Throttle only
                        break;
                    case 2:
                        elm327_send_command("010D");  // Speed only
                        break;
                }
                phase = (phase + 1) % 3;
            } else {
                // Optimized: Multi-PID requests
                switch (phase) {
                    case 0:
                        // Phase 0: RPM + Throttle (critical engine data)
                        elm327_send_command("010C11");
                        break;
                    case 1:
                        // Phase 1: Vehicle Speed
                        elm327_send_command("010D");
                        break;
                }
                // Alternate phases every 250ms
                phase ^= 1;
            }
            
            // Check for stale data and reset if needed
            check_and_reset_stale_data(use_individual_pids);
            
            // Log status (adjust frequency based on mode)
            if (use_individual_pids) {
                // Log every cycle when using individual PIDs
                if (phase == 0) {
                    log_vehicle_status();
                }
            } else {
                // Log once per complete cycle (every 500ms) for multi-PID
                if (phase == 0) {
                    log_vehicle_status();
                }
            }
            
            // Update success time if we have valid data
            if (vehicle_data.rpm > 0 || vehicle_data.throttle_position > 0 || vehicle_data.vehicle_speed > 0) {
                last_success_time = current_time;
            }
            
            // Wait before next phase
            vTaskDelay(pdMS_TO_TICKS(150));
            
        } else {
            ESP_LOGI(TAG, "⏳ Waiting for ELM327 connection...");
            // Reset strategy when disconnected
            use_individual_pids = false;
            last_success_time = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
} 