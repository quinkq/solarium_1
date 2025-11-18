/**
 * @file interval_config.h
 * @brief Centralized interval configuration management for all components
 *
 * This module manages user-adjustable check intervals for FLUCTUS, TEMPESTA,
 * and IMPLUVIUM components. Configuration is persisted to LittleFS with CRC32
 * integrity checking.
 */

#ifndef INTERVAL_CONFIG_H
#define INTERVAL_CONFIG_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ########################## Interval Range Definitions ##########################

// FLUCTUS Power Monitoring
#define INTERVAL_FLUCTUS_POWER_DAY_MIN       5
#define INTERVAL_FLUCTUS_POWER_DAY_MAX       60
#define INTERVAL_FLUCTUS_POWER_NIGHT_MIN     15
#define INTERVAL_FLUCTUS_POWER_NIGHT_MAX     120

// FLUCTUS Solar Tracking
#define INTERVAL_FLUCTUS_SOLAR_MIN           5
#define INTERVAL_FLUCTUS_SOLAR_MAX           60

// TEMPESTA Weather Collection
#define INTERVAL_TEMPESTA_NORMAL_MIN         5
#define INTERVAL_TEMPESTA_NORMAL_MAX         60
#define INTERVAL_TEMPESTA_POWER_SAVE_MIN     15
#define INTERVAL_TEMPESTA_POWER_SAVE_MAX     120

// IMPLUVIUM Moisture Checks
#define INTERVAL_IMPLUVIUM_OPTIMAL_MIN       5
#define INTERVAL_IMPLUVIUM_OPTIMAL_MAX       60
#define INTERVAL_IMPLUVIUM_COOL_MIN          10
#define INTERVAL_IMPLUVIUM_COOL_MAX          90
#define INTERVAL_IMPLUVIUM_POWER_SAVE_MIN    30
#define INTERVAL_IMPLUVIUM_POWER_SAVE_MAX    120
#define INTERVAL_IMPLUVIUM_NIGHT_MIN         1
#define INTERVAL_IMPLUVIUM_NIGHT_MAX         6

// ########################## Data Structures ##########################

/**
 * @brief Interval configuration structure
 *
 * All intervals are stored in minutes (or hours for nighttime minimum) to
 * reduce user confusion. Components convert to milliseconds at runtime.
 */
typedef struct {
    // FLUCTUS Power Monitoring (2 intervals)
    uint32_t fluctus_power_day_min;          // Range: 5-60 min, default: 15
    uint32_t fluctus_power_night_min;        // Range: 15-120 min, default: 60

    // FLUCTUS Solar Tracking (1 interval)
    uint32_t fluctus_solar_correction_min;   // Range: 5-60 min, default: 15

    // TEMPESTA Weather Collection (2 intervals)
    uint32_t tempesta_normal_min;            // Range: 5-60 min, default: 15
    uint32_t tempesta_power_save_min;        // Range: 15-120 min, default: 60

    // IMPLUVIUM Moisture Checks (3 base intervals + nighttime minimum)
    uint32_t impluvium_optimal_min;          // Range: 5-60 min, default: 15 (temp ≥20°C)
    uint32_t impluvium_cool_min;             // Range: 10-90 min, default: 30 (10-20°C)
    uint32_t impluvium_power_save_min;       // Range: 30-120 min, default: 60
    uint32_t impluvium_night_min_hours;      // Range: 1-6 hours, default: 3

    uint32_t crc32;                          // Integrity check (calculated on save)
} interval_config_t;

/**
 * @brief Interval preset identifiers
 */
typedef enum {
    INTERVAL_PRESET_AGGRESSIVE = 0,   // Short intervals (high power consumption)
    INTERVAL_PRESET_BALANCED,         // Default intervals (recommended)
    INTERVAL_PRESET_CONSERVATIVE,     // Long intervals (battery saver)
    INTERVAL_PRESET_CUSTOM            // User-defined (not a preset)
} interval_preset_t;

/**
 * @brief Global configuration instance
 *
 * Components read from this structure after interval_config_init() is called.
 * Updates via API functions automatically save to LittleFS.
 */
extern interval_config_t g_interval_config;

/**
 * @brief Initialize interval configuration
 *
 * Loads configuration from LittleFS (/littlefs/intervals.dat). If file doesn't
 * exist or is corrupted, defaults are used and saved.
 *
 * @note Must be called before component initialization (components read g_interval_config)
 *
 * @return ESP_OK on success, ESP_FAIL on storage error (defaults used)
 */
esp_err_t interval_config_init(void);

/**
 * @brief Save current configuration to LittleFS
 *
 * Calculates CRC32 and writes to /littlefs/intervals.dat. Called automatically
 * by setter functions, but can be called manually if g_interval_config is
 * modified directly.
 *
 * @return ESP_OK on success, ESP_FAIL on write error
 */
esp_err_t interval_config_save(void);

/**
 * @brief Reset configuration to factory defaults
 *
 * Balanced preset values:
 * - FLUCTUS: 15m day, 60m night, 15m solar
 * - TEMPESTA: 15m normal, 60m power save
 * - IMPLUVIUM: 15m optimal, 30m cool, 60m power save, 3h night
 *
 * Automatically saves to LittleFS.
 */
void interval_config_reset_defaults(void);

/**
 * @brief Set FLUCTUS power monitoring intervals
 *
 * @param day_min Daytime interval (5-60 minutes)
 * @param night_min Nighttime interval (15-120 minutes)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out of range
 */
esp_err_t interval_config_set_fluctus_power(uint32_t day_min, uint32_t night_min);

/**
 * @brief Set FLUCTUS solar tracking correction interval
 *
 * @param correction_min Correction cycle interval (5-60 minutes)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out of range
 */
esp_err_t interval_config_set_fluctus_solar(uint32_t correction_min);

/**
 * @brief Set TEMPESTA weather collection intervals
 *
 * @param normal_min Normal mode interval (5-60 minutes)
 * @param power_save_min Power save mode interval (15-120 minutes)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out of range
 */
esp_err_t interval_config_set_tempesta(uint32_t normal_min, uint32_t power_save_min);

/**
 * @brief Set IMPLUVIUM moisture check intervals
 *
 * @param optimal_min Optimal temperature interval (5-60 minutes, temp ≥20°C)
 * @param cool_min Cool temperature interval (10-90 minutes, 10-20°C)
 * @param power_save_min Power save mode interval (30-120 minutes)
 * @param night_hours Nighttime minimum interval (1-6 hours)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out of range
 */
esp_err_t interval_config_set_impluvium(uint32_t optimal_min, uint32_t cool_min,
                                        uint32_t power_save_min, uint32_t night_hours);

/**
 * @brief Apply a predefined interval preset
 *
 * Preset configurations:
 * - AGGRESSIVE: 5min day, 30min night (high power, fast response)
 * - BALANCED: 15min day, 60min night (default, recommended)
 * - CONSERVATIVE: 30min day, 120min night (battery saver)
 *
 * @param preset Preset identifier
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if preset unknown
 */
esp_err_t interval_config_apply_preset(interval_preset_t preset);

/**
 * @brief Get current preset (or CUSTOM if modified)
 *
 * Compares current configuration against known presets. Returns CUSTOM if
 * values don't match any preset.
 *
 * @return Current preset identifier
 */
interval_preset_t interval_config_get_current_preset(void);

/**
 * @brief Get preset name string
 *
 * @param preset Preset identifier
 * @return Human-readable name ("Aggressive", "Balanced", "Conservative", "Custom")
 */
const char* interval_config_get_preset_name(interval_preset_t preset);

/**
 * @brief Validate configuration ranges
 *
 * Checks if all interval values are within valid ranges. Used internally
 * before saving, but can be called externally for validation.
 *
 * @param config Configuration to validate
 * @return true if valid, false if any value out of range
 */
bool interval_config_validate(const interval_config_t *config);

#ifdef __cplusplus
}
#endif

#endif // INTERVAL_CONFIG_H
