#ifndef WIFI_HELPER_H
#define WIFI_HELPER_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include "esp_err.h"
#include "esp_wifi_types.h"

// ########################## Configuration ##########################

#define WIFI_HELPER_MAX_RETRIES           30    // ~24hrs minutes total with exponential backoff
#define WIFI_HELPER_RSSI_UPDATE_THRESHOLD 5     // Update TELEMETRY on RSSI change > 5 dBm
#define WIFI_HELPER_RSSI_CHECK_INTERVAL   (1000 * 60 * 15) // Check RSSI every 15 min (ms)

// ########################## Data Structures ##########################

/**
 * @brief WiFi connection state
 */
typedef enum {
    WIFI_STATE_DISABLED,      // Power save: WiFi off (CRITICAL power state)
    WIFI_STATE_INIT,          // Initializing WiFi subsystem
    WIFI_STATE_CONNECTING,    // First connection attempt in progress
    WIFI_STATE_CONNECTED,     // Connected with IP address
    WIFI_STATE_RECONNECTING,  // Reconnecting with exponential backoff
    WIFI_STATE_FAILED         // Max retries exceeded, requires manual intervention
} wifi_state_t;

/**
 * @brief WiFi snapshot for TELEMETRY cache
 * Size: ~24 bytes (compact, event-driven updates only)
 */
typedef struct {
    wifi_state_t state;              // Current WiFi state
    int8_t rssi;                     // Signal strength (-100 to 0 dBm, 0 = not connected)
    uint16_t reconnect_count;        // Total reconnections this session
    uint32_t connected_time_sec;     // Total time connected since boot (seconds)
    uint32_t last_disconnect_reason; // Last esp_wifi_reason_t (0 = never disconnected)
    bool has_ip;                     // True if IP address assigned
    bool power_save_mode;            // True if modem power save enabled
    time_t snapshot_timestamp;
} wifi_snapshot_t;

// ########################## Public API ##########################

/**
 * @brief Initialize WiFi helper component
 * Sets up WiFi in station mode, registers event handlers, starts connection
 *
 * @param ssid WiFi network SSID (max 32 chars)
 * @param password WiFi password (max 64 chars)
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t wifi_helper_init(const char *ssid, const char *password);

/**
 * @brief Get current WiFi status snapshot
 * Thread-safe access to WiFi state for HMI/diagnostics
 *
 * @param snapshot Output buffer for WiFi snapshot
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if snapshot is NULL
 */
esp_err_t wifi_helper_get_snapshot(wifi_snapshot_t *snapshot);

/**
 * @brief Enable/disable WiFi modem power save mode
 * Called by FLUCTUS during load shedding transitions
 *
 * Power save modes:
 * - false (normal): WIFI_PS_NONE - Max performance, ~100mA continuous
 * - true (power save): WIFI_PS_MAX_MODEM - Reduced power, ~20mA average
 *
 * @param enable true to enable power save, false for normal operation
 * @return ESP_OK on success
 */
esp_err_t wifi_helper_set_power_save_mode(bool enable);

/**
 * @brief Shutdown/enable WiFi subsystem
 * Called by FLUCTUS at CRITICAL power state (SOC ~0%)
 *
 * @param shutdown true to disable WiFi, false to re-enable
 * @return ESP_OK on success
 */
esp_err_t wifi_helper_set_shutdown(bool shutdown);

/**
 * @brief Force WiFi reconnection
 * Resets retry counter and initiates immediate connection attempt
 * Can be called from HMI to recover from FAILED state
 *
 * @return ESP_OK if reconnection initiated, ESP_FAIL if WiFi disabled
 */
esp_err_t wifi_helper_force_reconnect(void);

/**
 * @brief Write WiFi snapshot to TELEMETRY cache (TELEMETRY callback)
 * Called by TELEMETRY subsystem via stream writer function pointer
 * Components should NOT call this directly - use wifi_helper_inject_to_telemetry() instead
 *
 * @param cache Pointer to TELEMETRY's wifi_snapshot_t buffer
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t wifi_helper_write_to_telemetry_cache(wifi_snapshot_t *cache);

#endif // WIFI_HELPER_H
