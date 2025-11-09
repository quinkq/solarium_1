#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <time.h>
#include <stdbool.h>
#include "esp_err.h"

#include "solar_calc.h"

#include "impluvium.h"
#include "fluctus.h"
#include "tempesta.h"
#include "stellaria.h"
#include "wifi_helper.h"

// ########################## Configuration ##########################

// Storage configuration
#define TELEMETRY_LITTLEFS_MOUNT_POINT    "/data"
#define TELEMETRY_POWER_DATA_FILE         "/data/power.dat"

// MQTT configuration (from menuconfig - see Kconfig.projbuild)
#define TELEMETRY_MQTT_BROKER_URI     CONFIG_TELEMETRY_MQTT_BROKER_URI
#define TELEMETRY_MQTT_USERNAME       CONFIG_TELEMETRY_MQTT_USERNAME
#define TELEMETRY_MQTT_PASSWORD       CONFIG_TELEMETRY_MQTT_PASSWORD
#define TELEMETRY_MQTT_CLIENT_ID      CONFIG_TELEMETRY_MQTT_CLIENT_ID
#define TELEMETRY_MQTT_TOPIC_HOURLY   "solarium/power/hourly"
#define TELEMETRY_MQTT_TOPIC_DAILY    "solarium/power/daily"

// ########################## Data Structures ##########################

// Telemetry source enumeration for unified cache API
typedef enum {
    TELEMETRY_SRC_FLUCTUS,       // Normal mode power data
    TELEMETRY_SRC_FLUCTUS_RT,    // Realtime power data
    TELEMETRY_SRC_TEMPESTA,      // Weather data
    TELEMETRY_SRC_IMPLUVIUM,     // Irrigation data
    TELEMETRY_SRC_IMPLUVIUM_RT,  // Realtime watering sensors
    TELEMETRY_SRC_STELLARIA,     // Lighting data
    TELEMETRY_SRC_WIFI,          // WiFi connection status
    TELEMETRY_SRC_COUNT
} telemetry_source_t;

// ########################## Public API Functions ##########################

/**
 * @brief Initialize telemetry subsystem
 * @return ESP_OK on success
 */
esp_err_t telemetry_init(void);

// ################ Unified Cache API ################

/**
 * @brief Lock cache for a specific source and get pointer
 * @param src Source component identifier
 * @param cache_ptr Output pointer to cache structure
 * @return ESP_OK on success, ESP_FAIL on timeout (100ms)
 */
esp_err_t telemetry_lock_cache(telemetry_source_t src, void **cache_ptr);

/**
 * @brief Unlock cache for a specific source
 * Automatically updates snapshot_timestamp to current time
 * @param src Source component identifier
 */
void telemetry_unlock_cache(telemetry_source_t src);

/**
 * @brief Fetch snapshot from component and write to cache
 * Calls component's write function, then notifies MQTT task
 * @param src Source component identifier
 * @return ESP_OK on success
 */
esp_err_t telemetry_fetch_snapshot(telemetry_source_t src);

// ################ Component Get Functions (HMI Retrieval) ################

/**
 * @brief Get cached STELLARIA data for HMI display
 * Thread-safe retrieval from TELEMETRY cache
 *
 * @param[out] data Pointer to stellaria_snapshot_t structure to fill (from stellaria.h)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t telemetry_get_stellaria_data(stellaria_snapshot_t *data);

/**
 * @brief Get cached FLUCTUS data for HMI display (unified snapshot)
 * Thread-safe retrieval from TELEMETRY cache
 *
 * UNIFIED STRUCTURE: Contains both instantaneous and averaged data
 * - RT fields (bus states, instantaneous power, tracking errors): 0-500ms old
 * - Normal fields (15min averages, energy stats, system state): 0-15min old
 * - Pages choose which fields to display based on their needs
 *
 * @param[out] data Pointer to fluctus_snapshot_t structure to fill (from fluctus.h)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t telemetry_get_fluctus_data(fluctus_snapshot_t *data);

/**
 * @brief Get cached TEMPESTA data for HMI display
 * Thread-safe retrieval from TELEMETRY cache
 *
 * @param[out] data Pointer to tempesta_snapshot_t structure to fill (from weather_station.h)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t telemetry_get_tempesta_data(tempesta_snapshot_t *data);

/**
 * @brief Get cached IMPLUVIUM data for HMI display (full snapshot)
 * Thread-safe retrieval from TELEMETRY cache
 *
 * @param[out] data Pointer to impluvium_snapshot_t structure to fill (from irrigation.h)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t telemetry_get_impluvium_data(impluvium_snapshot_t *data);

/**
 * @brief Get cached IMPLUVIUM realtime data for HMI display (lightweight)
 * Thread-safe retrieval from TELEMETRY cache
 * Updated every 500ms during active watering
 *
 * @param[out] data Pointer to impluvium_snapshot_rt_t structure to fill (from irrigation.h)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t telemetry_get_impluvium_realtime_data(impluvium_snapshot_rt_t *data);

/**
 * @brief Get cached WiFi status data for HMI display
 * Thread-safe retrieval from TELEMETRY cache
 * Updated on connection state changes and RSSI deltas >5dBm
 *
 * @param[out] data Pointer to wifi_snapshot_t structure to fill (from wifi_helper.h)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t telemetry_get_wifi_data(wifi_snapshot_t *data);

// ################ Realtime Mode & Power Control ################

/**
 * @brief Get current realtime mode state
 * Components should check this before calling telemetry_fetch_snapshot() for RT sources
 *
 * @return true if realtime mode enabled, false otherwise
 */
bool telemetry_is_realtime_enabled(void);

/**
 * @brief Get current normal telemetry publishing state
 *
 * @return true if publishing enabled, false if buffering only
 */
bool telemetry_is_telemetry_publishing_enabled(void);

/**
 * @brief Set realtime mode enabled/disabled
 * Controls whether realtime messages (FLUCTUS_RT, IMPLUVIUM_RT) are enqueued
 * Can be controlled by: HMI, MQTT server command
 * Stores user preference for restoration after power state recovery
 *
 * @param enabled true to enable realtime mode, false to disable
 */
void telemetry_set_realtime_mode(bool enabled);

/**
 * @brief Enable or disable normal telemetry publishing
 * When disabled, normal mode messages are enqueued but NOT published (buffering only)
 * Control messages (message_type == 4) are ALWAYS published regardless of this flag
 * Called by FLUCTUS at VERY_LOW state (SOC <15%) to conserve power
 *
 * Power state behavior:
 * - NORMAL/POWER_SAVING: Publishing enabled (mode=true)
 * - VERY_LOW: Buffering only (mode=false)
 * - CRITICAL: Buffering only (mode=false)
 *
 * @param mode true to enable MQTT publishing, false for buffering-only mode
 */
void telemetry_enable_telemetry_publishing(bool mode);

/**
 * @brief Force disable/enable realtime mode (power state override)
 * Called by FLUCTUS to override realtime mode based on battery SOC
 * Does NOT update user preference - allows restoration when SOC recovers
 *
 * Power state behavior:
 * - NORMAL/POWER_SAVING: Restore user preference (force=false)
 * - LOW_POWER: Force disable (force=true)
 * - VERY_LOW/CRITICAL: Force disable (force=true)
 *
 * @param force true to force disable realtime mode, false to restore user preference
 */
void telemetry_force_realtime_monitoring_disable(bool force);

/**
 * @brief Manually flush all buffered MQTT messages from PSRAM to FLASH
 * Useful before user-triggered MCU reset to preserve unsent messages
 * Called from HMI "Save & Reset" option
 *
 * @param[out] flushed_count Optional pointer to receive count of messages flushed
 * @return ESP_OK on success, ESP_FAIL if FLASH backup not available
 */
esp_err_t telemetry_manual_flush_to_flash(uint16_t *flushed_count);

/**
 * @brief Get current MQTT buffer status
 * @param[out] buffered_count Number of messages in PSRAM buffer
 * @param[out] buffer_capacity Total PSRAM buffer capacity
 * @return ESP_OK on success
 */
esp_err_t telemetry_get_buffer_status(uint16_t *buffered_count, uint16_t *buffer_capacity);

#endif // TELEMETRY_H
