#ifndef SOLAR_CALC_H
#define SOLAR_CALC_H

#include <time.h>
#include <stdbool.h>

// ########################## Solar Calculation Configuration ##########################

// Gdansk, Poland coordinates
#define SOLAR_CALC_LATITUDE     54.3521   // Degrees North
#define SOLAR_CALC_LONGITUDE    18.6464   // Degrees East
#define SOLAR_CALC_TIMEZONE_OFFSET  1     // UTC+1 (CET), adjust for DST if needed

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
 * @brief Check if it's currently daytime
 * @return true if between sunrise and sunset, false otherwise
 */
bool solar_calc_is_daytime(void);

/**
 * @brief Force recalculation of solar times (useful after time sync)
 */
void solar_calc_update(void);

#endif // SOLAR_CALC_H
