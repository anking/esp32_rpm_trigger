#ifndef WEBSERVER_H
#define WEBSERVER_H

#include "esp_err.h"
#include "esp_http_server.h"

// WiFi Access Point Configuration
#define WIFI_AP_SSID        "ESP32-NOS-Controller"
#define WIFI_AP_PASSWORD    "nos123456"
#define WIFI_AP_CHANNEL     1
#define WIFI_AP_MAX_CONN    4

// Function declarations
esp_err_t webserver_init(void);
void webserver_stop(void);
esp_err_t wifi_init_ap(void);

// HTTP handlers
esp_err_t root_get_handler(httpd_req_t *req);
esp_err_t toggle_mode_post_handler(httpd_req_t *req);
esp_err_t status_get_handler(httpd_req_t *req);

#endif // WEBSERVER_H