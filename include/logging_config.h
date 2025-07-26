#ifndef LOGGING_CONFIG_H
#define LOGGING_CONFIG_H

// Logging control flags - Set to 1 to enable, 0 to disable
#define ENABLE_VERBOSE_LOGGING    0  // Detailed status messages (DISABLED - STACK OVERFLOW FIX)
#define ENABLE_DEBUG_LOGGING      0  // Debug and trace messages (DISABLED - STACK OVERFLOW FIX)
#define ENABLE_EMOJI_LOGGING      0  // Emoji-decorated messages (DISABLED - REDUCING NOISE)
#define ENABLE_BLUETOOTH_LOGGING  0  // Detailed Bluetooth protocol logs (DISABLED - REDUCING NOISE)
#define ENABLE_ELM327_LOGGING     0  // Detailed ELM327 communication logs (DISABLED - REDUCING NOISE)
#define ENABLE_ESP_BT_LOGS        0  // ESP-IDF Bluetooth stack logs (DISABLED - STACK OVERFLOW FIX)

// Conditional logging macros
#if ENABLE_VERBOSE_LOGGING
    #define LOG_VERBOSE(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#else
    #define LOG_VERBOSE(tag, format, ...)
#endif

#if ENABLE_DEBUG_LOGGING
    #define LOG_DEBUG(tag, format, ...) ESP_LOGD(tag, format, ##__VA_ARGS__)
#else
    #define LOG_DEBUG(tag, format, ...)
#endif

#if ENABLE_EMOJI_LOGGING
    #define LOG_EMOJI(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#else
    #define LOG_EMOJI(tag, format, ...)
#endif

#if ENABLE_BLUETOOTH_LOGGING
    #define LOG_BT(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#else
    #define LOG_BT(tag, format, ...)
#endif

#if ENABLE_ELM327_LOGGING
    #define LOG_ELM(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#else
    #define LOG_ELM(tag, format, ...)
#endif

// Always-enabled essential logs
#define LOG_INFO(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#define LOG_WARN(tag, format, ...) ESP_LOGW(tag, format, ##__VA_ARGS__)
#define LOG_ERROR(tag, format, ...) ESP_LOGE(tag, format, ##__VA_ARGS__)

// ESP-IDF Bluetooth stack log control function
void configure_esp_bt_logging(void);

#endif // LOGGING_CONFIG_H 