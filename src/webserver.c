#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "logging_config.h"
#include "webserver.h"
#include "gpio_control.h"
#include "obd_data.h"
#include "bluetooth.h"
#include "elm327.h"

static const char *TAG = "WEBSERVER";
static httpd_handle_t server = NULL;

// HTML content for the web interface
static const char index_html[] = 
"<!DOCTYPE html>"
"<html>"
"<head>"
"    <meta charset='UTF-8'>"
"    <meta name='viewport' content='width=device-width, initial-scale=1.0'>"
"    <title>ESP32 NOS Controller</title>"
"    <style>"
"        body { font-family: Arial, sans-serif; margin: 20px; background-color: #1a1a1a; color: #ffffff; }"
"        .container { max-width: 400px; margin: 0 auto; text-align: center; }"
"        .status-box { background-color: #333; padding: 15px; margin: 20px 0; border-radius: 10px; }"
"        .mode-toggle { background-color: #4CAF50; color: white; padding: 15px 30px; border: none; border-radius: 8px; font-size: 18px; cursor: pointer; margin: 10px; }"
"        .mode-toggle.manual { background-color: #ff6b35; }"
"        .data-item { margin: 8px 0; font-size: 16px; }"
"        .ready-indicator { display: inline-block; width: 15px; height: 15px; border-radius: 50%; margin-left: 10px; }"
"        .ready-true { background-color: #4CAF50; }"
"        .ready-false { background-color: #f44336; }"
"    </style>"
"</head>"
"<body>"
"    <div class='container'>"
"        <h1>🚗 NOS Controller</h1>"
"        <div class='status-box'>"
"            <h3>Current Mode</h3>"
"            <p id='current-mode'></p>"
"            <button id='toggle-btn' class='mode-toggle' onclick='toggleMode()'>Toggle Mode</button>"
"        </div>"
"        <div class='status-box'>"
"            <h3>Vehicle Data</h3>"
"            <div class='data-item'>RPM: <span id='rpm'>--</span></div>"
"            <div class='data-item'>Throttle: <span id='throttle'>--</span>%</div>"
"            <div class='data-item'>Speed: <span id='speed'>--</span> km/h</div>"
"        </div>"
"        <div class='status-box'>"
"            <h3>NOS Status</h3>"
"            <div class='data-item'>NOS Ready: <span id='nos-ready'>--</span><span id='ready-dot' class='ready-indicator'></span></div>"
"            <div class='data-item'>Auto Injection: <span id='auto-injection'>--</span></div>"
"        </div>"
"    </div>"
"    <script>"
"        function updateStatus() {"
"            fetch('/status')"
"            .then(response => response.json())"
"            .then(data => {"
"                document.getElementById('current-mode').textContent = data.auto_injection ? 'AUTO INJECTION' : 'MANUAL MODE';"
"                document.getElementById('toggle-btn').textContent = data.auto_injection ? 'Switch to Manual' : 'Switch to Auto';"
"                document.getElementById('toggle-btn').className = data.auto_injection ? 'mode-toggle' : 'mode-toggle manual';"
"                document.getElementById('rpm').textContent = data.rpm;"
"                document.getElementById('throttle').textContent = data.throttle;"
"                document.getElementById('speed').textContent = data.speed;"
"                document.getElementById('nos-ready').textContent = data.nos_ready ? 'READY' : 'NOT READY';"
"                document.getElementById('ready-dot').className = 'ready-indicator ' + (data.nos_ready ? 'ready-true' : 'ready-false');"
"                document.getElementById('auto-injection').textContent = data.auto_injection_active ? 'ACTIVE' : 'INACTIVE';"
"            })"
"            .catch(error => console.error('Error:', error));"
"        }"
"        function toggleMode() {"
"            fetch('/toggle-mode', {method: 'POST'})"
"            .then(response => response.json())"
"            .then(data => {"
"                console.log('Mode toggled:', data);"
"                updateStatus();"
"            })"
"            .catch(error => console.error('Error:', error));"
"        }"
"        setInterval(updateStatus, 1000);"
"        updateStatus();"
"    </script>"
"</body>"
"</html>";

// Initialize WiFi in AP mode
esp_err_t wifi_init_ap(void) {
    ESP_LOGI(TAG, "🚀 Starting WiFi AP initialization...");
    
    // Initialize TCP/IP stack
    ESP_LOGI(TAG, "📡 Initializing TCP/IP stack...");
    esp_err_t ret = esp_netif_init();
    ESP_LOGI(TAG, "📡 TCP/IP init result: %s", esp_err_to_name(ret));
    ESP_ERROR_CHECK(ret);
    
    // Create default event loop
    ESP_LOGI(TAG, "🔄 Creating event loop...");
    ret = esp_event_loop_create_default();
    ESP_LOGI(TAG, "🔄 Event loop result: %s", esp_err_to_name(ret));
    ESP_ERROR_CHECK(ret);
    
    // Create default WiFi AP interface
    ESP_LOGI(TAG, "📶 Creating WiFi AP interface...");
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    ESP_LOGI(TAG, "📶 WiFi AP interface created: %p", ap_netif);
    
    // Initialize WiFi with default config
    ESP_LOGI(TAG, "⚙️ Initializing WiFi driver...");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    ESP_LOGI(TAG, "⚙️ WiFi driver init result: %s", esp_err_to_name(ret));
    ESP_ERROR_CHECK(ret);
    
    // Configure WiFi AP
    ESP_LOGI(TAG, "🔧 Configuring WiFi AP settings...");
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASSWORD,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    
    ESP_LOGI(TAG, "🔧 SSID: %s", WIFI_AP_SSID);
    ESP_LOGI(TAG, "🔧 Password: %s", WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, "🔧 Channel: %d", WIFI_AP_CHANNEL);
    ESP_LOGI(TAG, "🔧 Max connections: %d", WIFI_AP_MAX_CONN);
    
    if (strlen(WIFI_AP_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
        ESP_LOGI(TAG, "🔧 Using OPEN authentication (no password)");
    } else {
        ESP_LOGI(TAG, "🔧 Using WPA/WPA2 authentication");
    }
    
    ESP_LOGI(TAG, "📋 Setting WiFi mode to AP...");
    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    ESP_LOGI(TAG, "📋 Set mode result: %s", esp_err_to_name(ret));
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "📋 Setting WiFi configuration...");
    ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    ESP_LOGI(TAG, "📋 Set config result: %s", esp_err_to_name(ret));
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "🚀 Starting WiFi AP...");
    ret = esp_wifi_start();
    ESP_LOGI(TAG, "🚀 WiFi start result: %s", esp_err_to_name(ret));
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "✅ WiFi AP started successfully!");
    ESP_LOGI(TAG, "📱 SSID: %s", WIFI_AP_SSID);
    ESP_LOGI(TAG, "🔑 Password: %s", WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, "🌍 Connect and go to: http://192.168.4.1");
    
    return ESP_OK;
}

// Root page handler
esp_err_t root_get_handler(httpd_req_t *req) {
    LOG_DEBUG(TAG, "Serving root page");
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
}

// Status API handler
esp_err_t status_get_handler(httpd_req_t *req) {
    LOG_DEBUG(TAG, "Serving status API");
    
    // Get current status
    bool auto_mode = get_auto_injection_mode();
    
    // Determine if NOS conditions are met and auto injection is active
    bool nos_ready = (vehicle_data.vehicle_speed > 20) && (vehicle_data.rpm > 3000) && (vehicle_data.throttle_position > 40);
    bool auto_injection_active = auto_mode && nos_ready && is_connected && ecu_connected;
    
    // Create JSON response
    char json_resp[512];
    snprintf(json_resp, sizeof(json_resp),
        "{"
        "\"auto_injection\":%s,"
        "\"rpm\":%lu,"
        "\"throttle\":%d,"
        "\"speed\":%d,"
        "\"nos_ready\":%s,"
        "\"auto_injection_active\":%s,"
        "\"bluetooth_connected\":%s,"
        "\"ecu_connected\":%s"
        "}",
        auto_mode ? "true" : "false",
        vehicle_data.rpm,
        vehicle_data.throttle_position,
        vehicle_data.vehicle_speed,
        nos_ready ? "true" : "false",
        auto_injection_active ? "true" : "false",
        is_connected ? "true" : "false",
        ecu_connected ? "true" : "false"
    );
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_resp, HTTPD_RESP_USE_STRLEN);
}

// Toggle mode handler
esp_err_t toggle_mode_post_handler(httpd_req_t *req) {
    LOG_INFO(TAG, "Mode toggle requested via web interface");
    
    // Toggle the mode (same as button press)
    toggle_auto_injection_mode();
    
    // Create response
    bool new_mode = get_auto_injection_mode();
    char json_resp[128];
    snprintf(json_resp, sizeof(json_resp),
        "{\"success\":true,\"new_mode\":\"%s\"}",
        new_mode ? "auto" : "manual"
    );
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_resp, HTTPD_RESP_USE_STRLEN);
}

// Start HTTP server
static httpd_handle_t start_webserver(void) {
    ESP_LOGI(TAG, "🌐 Preparing HTTP server configuration...");
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    
    ESP_LOGI(TAG, "🌐 HTTP server port: %d", config.server_port);
    ESP_LOGI(TAG, "🌐 Max connections: %d", config.max_open_sockets);
    ESP_LOGI(TAG, "🌐 Starting HTTP server...");
    
    esp_err_t start_result = httpd_start(&server, &config);
    ESP_LOGI(TAG, "🌐 HTTP start result: %s", esp_err_to_name(start_result));
    
    if (start_result == ESP_OK) {
        ESP_LOGI(TAG, "🌐 ✅ HTTP server started successfully!");
        // Set URI handlers
        ESP_LOGI(TAG, "🌐 Registering URI handlers...");
        
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        esp_err_t reg_result = httpd_register_uri_handler(server, &root_uri);
        ESP_LOGI(TAG, "🌐 Root handler (/) registration: %s", esp_err_to_name(reg_result));
        
        httpd_uri_t status_uri = {
            .uri       = "/status",
            .method    = HTTP_GET,
            .handler   = status_get_handler,
            .user_ctx  = NULL
        };
        reg_result = httpd_register_uri_handler(server, &status_uri);
        ESP_LOGI(TAG, "🌐 Status handler (/status) registration: %s", esp_err_to_name(reg_result));
        
        httpd_uri_t toggle_uri = {
            .uri       = "/toggle-mode",
            .method    = HTTP_POST,
            .handler   = toggle_mode_post_handler,
            .user_ctx  = NULL
        };
        reg_result = httpd_register_uri_handler(server, &toggle_uri);
        ESP_LOGI(TAG, "🌐 Toggle handler (/toggle-mode) registration: %s", esp_err_to_name(reg_result));
        
        ESP_LOGI(TAG, "🌐 ✅ All URI handlers registered successfully!");
        ESP_LOGI(TAG, "🌐 🎉 HTTP SERVER FULLY OPERATIONAL!");
        return server;
    }
    
    LOG_ERROR(TAG, "Failed to start HTTP server");
    return NULL;
}

// Initialize webserver system
esp_err_t webserver_init(void) {
    ESP_LOGI(TAG, "🚀 =================================");
    ESP_LOGI(TAG, "🚀 WEBSERVER INITIALIZATION START");
    ESP_LOGI(TAG, "🚀 =================================");
    
    // Initialize WiFi AP
    ESP_LOGI(TAG, "📶 Step 1: Initializing WiFi Access Point...");
    esp_err_t ret = wifi_init_ap();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Step 1 FAILED: WiFi AP init error: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ Step 1 SUCCESS: WiFi AP initialized");
    
    // Add a small delay to let WiFi stabilize
    ESP_LOGI(TAG, "⏱️ Waiting 2 seconds for WiFi to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Start HTTP server
    ESP_LOGI(TAG, "🌐 Step 2: Starting HTTP server...");
    server = start_webserver();
    if (server == NULL) {
        ESP_LOGE(TAG, "❌ Step 2 FAILED: HTTP server start failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "✅ Step 2 SUCCESS: HTTP server started");
    
    ESP_LOGI(TAG, "🎉 =================================");
    ESP_LOGI(TAG, "🎉 WEBSERVER FULLY INITIALIZED!");
    ESP_LOGI(TAG, "🎉 =================================");
    ESP_LOGI(TAG, "📱 WiFi Network: %s", WIFI_AP_SSID);
    ESP_LOGI(TAG, "🔑 Password: %s", WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, "🌍 Web Interface: http://192.168.4.1");
    ESP_LOGI(TAG, "🎉 =================================");
    
    return ESP_OK;
}

// Stop webserver
void webserver_stop(void) {
    if (server) {
        httpd_stop(server);
        server = NULL;
        LOG_INFO(TAG, "HTTP server stopped");
    }
}