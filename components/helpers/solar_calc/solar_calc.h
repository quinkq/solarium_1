#ifndef SOLAR_CALC_H
#define SOLAR_CALC_H

#include <time.h>
#include <stdbool.h>
#include "esp_err.h"

// ########################## Solar Calculation Configuration ##########################

// Gdansk, Poland coordinates
#define SOLAR_CALC_LATITUDE     54.3521   // Degrees North
#define SOLAR_CALC_LONGITUDE    18.6464   // Degrees East
#define SOLAR_CALC_TIMEZONE_OFFSET  1     // UTC+1 (CET), adjust for DST if needed

// Daytime buffer for system operations (same as solar tracking buffers)
#define SOLAR_CALC_SUNRISE_BUFFER_MINUTES  30  // Start "daytime" 30 min before sunrise
#define SOLAR_CALC_SUNSET_BUFFER_MINUTES   30  // End "daytime" 30 min after sunset

// ########################## Data Structures ##########################

typedef struct {
    time_t sunrise_time;    // Sunrise time (UTC)
    time_t sunset_time;     // Sunset time (UTC)
    bool is_daytime;        // Current daytime status
    time_t last_update;     // Last calculation time
} solar_times_t;

// ########################## Public API Functions ##########################

/**
 * @brief Initialize solar calculation module
 * @return ESP_OK on success
 */
void solar_calc_init(void);

/**
 * @brief Calculate sunrise and sunset times for current day
 * @note Calculations are cached and updated at midnight
 * @return Pointer to solar_times_t structure with current times
 */
const solar_times_t* solar_calc_get_times(void);

/**
 * @brief Check if it's currently daytime (exact sunrise/sunset bounds)
 * @return true if between sunrise and sunset, false otherwise
 */
bool solar_calc_is_daytime(void);

/**
 * @brief Check if it's currently daytime with operational buffers applied
 *
 * Returns true if within buffered daytime window:
 * - Start: 30 minutes before sunrise
 * - End: 30 minutes after sunset
 *
 * Used by components for day/night operational mode switching (e.g., monitoring
 * intervals, irrigation scheduling) to align with solar tracking behavior.
 *
 * @return true if within buffered daytime hours, false during night
 */
bool solar_calc_is_daytime_buffered(void);

/**
 * @brief Force recalculation of solar times (useful after time sync)
 */
void solar_calc_update(void);

// ########################## Sunrise Notification System ##########################

/**
 * @brief Sunrise callback function type
 *
 * Components can register callbacks to be notified when sunrise time is reached.
 * Useful for waking up SLEEPING solar tracking or switching to daytime operations.
 *
 * Callbacks are triggered by solar_calc_update() when current time crosses the
 * buffered sunrise threshold (sunrise - 30 minutes).
 *
 * Called from midnight timer (daily check at 00:00).
 */
typedef void (*solar_calc_sunrise_callback_t)(void);

/**
 * @brief Register a callback to be notified when sunrise time is reached
 *
 * @param callback Function to be called at sunrise time (buffered)
 * @return ESP_OK on success, ESP_ERR_NO_MEM if callback list is full
 */
esp_err_t solar_calc_register_sunrise_callback(solar_calc_sunrise_callback_t callback);

/**
 * @brief Broadcast sunrise event to all registered subscribers
 *
 * Called internally by solar_calc_update() when sunrise detection logic triggers.
 * Not intended for external use.
 */
void solar_calc_broadcast_sunrise(void);

// ########################## Midnight Notification System ##########################

/**
 * @brief Midnight callback function type
 *
 * Components can register callbacks to be notified when midnight is reached (day change).
 * Useful for daily data backups, log rotation, or daily reset operations.
 *
 * Callbacks are triggered by an esp_timer at precisely 00:00:00 local time.
 * Solar times are automatically recalculated before broadcasting to midnight subscribers.
 *
 * Called from midnight timer (daily trigger at 00:00:00 local time).
 */
typedef void (*solar_calc_midnight_callback_t)(void);

/**
 * @brief Register a callback to be notified at midnight (day change)
 *
 * @param callback Function to be called at midnight
 * @return ESP_OK on success, ESP_ERR_NO_MEM if callback list is full
 */
esp_err_t solar_calc_register_midnight_callback(solar_calc_midnight_callback_t callback);

/**
 * @brief Broadcast midnight event to all registered subscribers
 *
 * Called internally by midnight timer callback at 00:00:00 local time.
 * Not intended for external use.
 */
void solar_calc_broadcast_midnight(void);

#endif // SOLAR_CALC_H
