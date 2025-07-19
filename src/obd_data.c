#include "esp_log.h"
#include <stdio.h>
#include <string.h>

#include "logging_config.h"
#include "obd_data.h"
#include "elm327.h"
#include "bluetooth.h"
#include "gpio_control.h"

static const char *TAG = "OBD_DATA";

// Global vehicle data instance
vehicle_data_t vehicle_data = {
    .rpm = 0,
    .throttle_position = 0,
    .vehicle_speed = 0,
    .current_gear = "N/A"
};

// Initialize OBD data system
void obd_data_init(void) {
    // Reset vehicle data to defaults
    vehicle_data.rpm = 0;
    vehicle_data.throttle_position = 0;
    vehicle_data.vehicle_speed = 0;
    strcpy(vehicle_data.current_gear, "N/A");
    
    LOG_VERBOSE(TAG, "OBD data system initialized");
}

// Parse RPM response from ELM327
uint32_t parse_rpm(const char *response) {
    // Example response: "41 0C 1A F8\r"
    int A, B;
    if (sscanf(response, "41 0C %x %x", &A, &B) == 2) {
        uint32_t rpm = ((A * 256) + B) / 4;
        ESP_LOGD(TAG, "Parsed RPM: %lu", rpm);
        return rpm;
    }
    return 0;
}

// Parse throttle position response from ELM327
uint8_t parse_throttle(const char *response) {
    // Example response: "41 11 5A\r" (90% throttle)
    int A;
    if (sscanf(response, "41 11 %x", &A) == 1) {
        uint8_t throttle = (A * 100) / 255;
        ESP_LOGD(TAG, "Parsed throttle: %d%%", throttle);
        return throttle;
    }
    return 0;
}

// Parse vehicle speed response from ELM327
uint8_t parse_speed(const char *response) {
    // Example response: "41 0D 3C\r" (60 km/h)
    int A;
    if (sscanf(response, "41 0D %x", &A) == 1) {
        uint8_t speed = A;  // Speed is directly in km/h
        ESP_LOGD(TAG, "Parsed speed: %d km/h", speed);
        return speed;
    }
    return 0;
}

// Parse gear position (simplified estimation based on RPM and speed)
void parse_gear(const char *response) {
    // This is a simplified gear estimation
    // Real gear detection would require more sophisticated OBD-II commands
    
    if (vehicle_data.vehicle_speed == 0) {
        strcpy(vehicle_data.current_gear, "N");  // Neutral/Park
    } else if (vehicle_data.vehicle_speed > 0 && vehicle_data.vehicle_speed < 20) {
        strcpy(vehicle_data.current_gear, "1");  // First gear
    } else if (vehicle_data.vehicle_speed >= 20 && vehicle_data.vehicle_speed < 40) {
        strcpy(vehicle_data.current_gear, "2");  // Second gear
    } else if (vehicle_data.vehicle_speed >= 40 && vehicle_data.vehicle_speed < 60) {
        strcpy(vehicle_data.current_gear, "3");  // Third gear
    } else if (vehicle_data.vehicle_speed >= 60 && vehicle_data.vehicle_speed < 80) {
        strcpy(vehicle_data.current_gear, "4");  // Fourth gear
    } else if (vehicle_data.vehicle_speed >= 80) {
        strcpy(vehicle_data.current_gear, "5+"); // Fifth+ gear
    } else {
        strcpy(vehicle_data.current_gear, "N/A");
    }
}

// Display current vehicle data
void display_vehicle_data(void) {
    ESP_LOGI(TAG, "Vehicle Data: RPM=%lu | Throttle=%d%% | Speed=%d km/h | Gear=%s", 
             vehicle_data.rpm, 
             vehicle_data.throttle_position, 
             vehicle_data.vehicle_speed, 
             vehicle_data.current_gear);
}

// Log vehicle status with GPIO state
void log_vehicle_status(void) {
    const char* gpio_state = gpio_status ? "ON" : "OFF";
    ESP_LOGI(TAG, "RPM: %lu | Throttle: %d%% | Speed: %d km/h | Gear: %s | GPIO2: %s",
             vehicle_data.rpm,
             vehicle_data.throttle_position,
             vehicle_data.vehicle_speed,
             vehicle_data.current_gear,
             gpio_state);
}

// OBD data polling task
void obd_task(void *pv) {
    LOG_VERBOSE(TAG, "OBD Task started - waiting for Bluetooth connection...");
    
    while (1) {
        // Wait for ELM327 to be connected and initialized
        if (connection_semaphore != NULL) {
            if (xSemaphoreTake(connection_semaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
                LOG_INFO(TAG, "Starting OBD data polling...");
                break;  // Connected and initialized, start polling
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100ms
    }
    
    // Main OBD polling loop
    while (1) {
        if (is_connected && elm327_initialized) {
            // Send OBD commands for vehicle data
            send_obd_command("010C");  // RPM
            vTaskDelay(pdMS_TO_TICKS(50));  // Small delay between commands
            
            send_obd_command("0111");  // Throttle position
            vTaskDelay(pdMS_TO_TICKS(50));
            
            send_obd_command("010D");  // Vehicle speed
            vTaskDelay(pdMS_TO_TICKS(50));
            
            // Update gear estimation based on current data
            parse_gear(NULL);  // Use current vehicle data for estimation
            
            // Log current status every ~500ms (close to user's request)
            static int log_counter = 0;
            if (++log_counter >= 1) {  // Log every 400ms (1 * 400ms ≈ 500ms)
                log_vehicle_status();
                log_counter = 0;
            }
            
            // Wait before next poll cycle
            vTaskDelay(pdMS_TO_TICKS(400));  // Poll every 400ms
        } else {
            // Not connected, wait longer
            LOG_DEBUG(TAG, "Waiting for connection...");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
} 