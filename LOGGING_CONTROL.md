# 📊 Logging Control System

Your ESP32 OBD-II Controller now has **configurable logging levels** to reduce noise and improve performance.

## 🎛️ Logging Flags (`include/logging_config.h`)

| **Flag** | **Purpose** | **Default** | **When to Enable** |
|----------|-------------|-------------|-------------------|
| `ENABLE_VERBOSE_LOGGING` | Detailed status messages | **OFF** | Debugging system initialization |
| `ENABLE_DEBUG_LOGGING` | Debug/trace messages | **OFF** | Troubleshooting data flow |
| `ENABLE_EMOJI_LOGGING` | Emoji-decorated messages | **OFF** | Fun visual feedback |
| `ENABLE_BLUETOOTH_LOGGING` | Detailed Bluetooth protocol | **OFF** | Bluetooth connection issues |
| `ENABLE_ELM327_LOGGING` | ELM327 communication details | **OFF** | ELM327 initialization problems |
| `ENABLE_ESP_BT_LOGS` | ESP-IDF BT stack logs (BT_RFCOMM, BT_L2CAP) | **OFF** | Deep Bluetooth stack debugging |

## 🔧 How to Enable Logging

Edit `include/logging_config.h` and change the desired flag from `0` to `1`:

```c
// Example: Enable verbose logging for debugging
#define ENABLE_VERBOSE_LOGGING    1  // Enable detailed status messages
#define ENABLE_BLUETOOTH_LOGGING  1  // Enable Bluetooth debugging
```

## 📈 Current Output (All Flags OFF)

With all flags disabled, you'll see only **essential information**:

```
ESP32 OBD-II Controller Starting
NVS initialized
ESP-IDF Bluetooth stack logs suppressed (set to WARN level)
Starting Bluetooth initialization...
Bluetooth initialization complete!
Looking for ELM327 device: [01:23:45:67:89:BA]
System initialization complete. Searching for ELM327...
Found ELM327: 01:23:45:67:89:BA
SCN 2 connection initiated
RFCOMM connection established
Basic ELM327 initialization complete!
Starting OBD data polling...
[Vehicle data: RPM, Throttle, Speed logged every 400ms (close to requested 500ms)]
[NO MORE BT_RFCOMM/BT_L2CAP spam!]
```

## ⚡ Performance Benefits

- **Faster execution** - Less time spent on logging
- **Cleaner output** - Only essential information shown (RPM, Throttle, Speed)
- **Reduced flash usage** - Conditional compilation removes unused strings
- **Better debugging** - Enable only what you need
- **Configurable frequency** - Vehicle data logged every ~400ms (close to 500ms)
- **Accurate data only** - Removed unreliable gear estimation logic

## 🎯 Recommended Settings

**Normal Operation:**
```c
#define ENABLE_VERBOSE_LOGGING    0
#define ENABLE_DEBUG_LOGGING      0  
#define ENABLE_EMOJI_LOGGING      0
#define ENABLE_BLUETOOTH_LOGGING  0
#define ENABLE_ELM327_LOGGING     0
#define ENABLE_ESP_BT_LOGS        0  // ← This eliminates BT_RFCOMM/BT_L2CAP noise!
```

**Debugging Connection Issues:**
```c
#define ENABLE_VERBOSE_LOGGING    1
#define ENABLE_BLUETOOTH_LOGGING  1
#define ENABLE_ELM327_LOGGING     1
#define ENABLE_ESP_BT_LOGS        0  // Keep OFF unless debugging BT stack itself
```

**Deep Bluetooth Stack Debugging:**
```c
#define ENABLE_VERBOSE_LOGGING    1
#define ENABLE_BLUETOOTH_LOGGING  1
#define ENABLE_ELM327_LOGGING     1
#define ENABLE_ESP_BT_LOGS        1  // Enable for BT_RFCOMM/BT_L2CAP debugging
```

**Fun Mode:**
```c
#define ENABLE_EMOJI_LOGGING      1
```

Remember to **rebuild** after changing any flags! 🚀 