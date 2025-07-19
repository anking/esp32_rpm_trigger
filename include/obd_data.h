#ifndef OBD_DATA_H
#define OBD_DATA_H

#include <stdint.h>

// Vehicle data structure
typedef struct {
    uint32_t rpm;
    uint8_t throttle_position;  // 0-100%
    uint8_t vehicle_speed;      // km/h
} vehicle_data_t;

// Global vehicle data
extern vehicle_data_t vehicle_data;

// Function declarations
void obd_data_init(void);
void obd_task(void *pv);

// OBD response parsing functions
uint32_t parse_rpm(const char *response);
uint8_t parse_throttle(const char *response);
uint8_t parse_speed(const char *response);

// Data display
void display_vehicle_data(void);
void log_vehicle_status(void);

#endif // OBD_DATA_H 