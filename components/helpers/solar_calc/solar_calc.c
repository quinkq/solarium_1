/**
 * @file solar_calc.c
 * @brief NOAA solar position algorithm implementation
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Original implementation of solar position calculations using NOAA algorithm.
 *
 * Key features:
 * - NOAA solar position algorithm (±5-10 minute accuracy)
 * - Configured for Gdansk, Poland (54.3521°N, 18.6464°E)
 * - Automatic sunrise/sunset detection with 30-minute safety buffers
 * - Event callbacks for midnight and sunrise
 * - Real-time solar position tracking for panel orientation
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "solar_calc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

// ########################## Constants ##########################

static const char *TAG = "SOLAR_CALC";

#define PI 3.14159265358979323846
#define DEG_TO_RAD(deg) ((deg) * PI / 180.0)
#define RAD_TO_DEG(rad) ((rad) * 180.0 / PI)

// Callback configuration
#define SOLAR_CALC_MAX_SUNRISE_CALLBACKS 4
#define SOLAR_CALC_MAX_MIDNIGHT_CALLBACKS 8

// ########################## Private Variables ##########################

static solar_times_t solar_times = {0};

// Sunrise notification system
static solar_calc_sunrise_callback_t sunrise_callbacks[SOLAR_CALC_MAX_SUNRISE_CALLBACKS] = {0};
static uint8_t sunrise_callback_count = 0;
static bool sunrise_triggered_today = false;  // Prevent multiple triggers per day

// Midnight notification system
static solar_calc_midnight_callback_t midnight_callbacks[SOLAR_CALC_MAX_MIDNIGHT_CALLBACKS] = {0};
static uint8_t midnight_callback_count = 0;
static esp_timer_handle_t xMidnightTimer = NULL;

// ########################## Private Function Declarations ##########################

static double solar_calc_get_julian_day(time_t t);
static void solar_calc_calculate_solar_times(double julian_day, double latitude, double longitude,
                                              double *sunrise_hour, double *sunset_hour);
static time_t solar_calc_decimal_hours_to_time(time_t base_time, double hours);
static void solar_calc_schedule_next_midnight(void);
static void solar_calc_midnight_timer_callback(void *arg);

// ########################## Private Functions ##########################

/**
 * @brief Calculate Julian Day from time_t
 */
static double solar_calc_get_julian_day(time_t t)
{
    return (double)t / 86400.0 + 2440587.5;
}

/**
 * @brief Simplified sunrise/sunset calculation using basic equations
 *
 * Based on NOAA Solar Calculator simplified algorithm
 * Accuracy: ±5-10 minutes (sufficient for power management)
 *
 * HEMISPHERE AGNOSTIC DESIGN:
 * This algorithm works correctly for any location on Earth using standard coordinate conventions:
 * - Latitude: Positive = North (0° to +90°), Negative = South (0° to -90°)
 * - Longitude: Positive = East (0° to +180°), Negative = West (0° to -180°)
 * The trigonometric calculations automatically handle hemisphere differences.
 *
 * @param julian_day Julian day number
 * @param latitude Latitude in degrees (positive = North, negative = South)
 * @param longitude Longitude in degrees (positive = East, negative = West)
 * @param[out] sunrise_hour Sunrise time in decimal hours (UTC)
 * @param[out] sunset_hour Sunset time in decimal hours (UTC)
 */
static void solar_calc_calculate_solar_times(double julian_day, double latitude, double longitude,
                                              double *sunrise_hour, double *sunset_hour)
{
    // Calculate Julian century (same for all locations)
    double julian_century = (julian_day - 2451545.0) / 36525.0;

    // Sun's mean anomaly
    double mean_anomaly = 357.52911 + julian_century * (35999.05029 - 0.0001537 * julian_century);
    mean_anomaly = fmod(mean_anomaly, 360.0);

    // Sun's equation of center
    double center = sin(DEG_TO_RAD(mean_anomaly)) * (1.914602 - julian_century * (0.004817 + 0.000014 * julian_century));
    center += sin(DEG_TO_RAD(2 * mean_anomaly)) * (0.019993 - 0.000101 * julian_century);
    center += sin(DEG_TO_RAD(3 * mean_anomaly)) * 0.000289;

    // Sun's true longitude
    double sun_longitude = fmod(mean_anomaly + center + 180.0 + 102.93735, 360.0);

    // Sun's declination
    double declination = asin(sin(DEG_TO_RAD(sun_longitude)) * sin(DEG_TO_RAD(23.439)));

    // Hour angle at sunrise/sunset (accounting for atmospheric refraction)
    // Latitude sign naturally handles hemisphere: negative latitude (South) produces correct angles
    double latitude_rad = DEG_TO_RAD(latitude);
    double cos_hour_angle = (sin(DEG_TO_RAD(-0.833)) - sin(latitude_rad) * sin(declination)) /
                            (cos(latitude_rad) * cos(declination));

    // Check for polar day/night
    if (cos_hour_angle > 1.0) {
        // Polar night - sun never rises
        *sunrise_hour = 0.0;
        *sunset_hour = 0.0;
        ESP_LOGW(TAG, "Polar night condition detected");
        return;
    } else if (cos_hour_angle < -1.0) {
        // Polar day - sun never sets
        *sunrise_hour = 0.0;
        *sunset_hour = 24.0;
        ESP_LOGW(TAG, "Polar day condition detected");
        return;
    }

    double hour_angle = RAD_TO_DEG(acos(cos_hour_angle));

    // Solar noon (in decimal hours UTC)
    // Longitude sign naturally handles hemisphere: negative longitude (West) shifts noon later
    double equation_of_time = 4.0 * (longitude - RAD_TO_DEG(atan2(sin(2 * DEG_TO_RAD(sun_longitude)),
                                                                    cos(2 * DEG_TO_RAD(sun_longitude))) * 2));
    double solar_noon = 12.0 - equation_of_time / 60.0;

    // Sunrise and sunset times (decimal hours UTC)
    *sunrise_hour = solar_noon - hour_angle / 15.0;
    *sunset_hour = solar_noon + hour_angle / 15.0;

    // Normalize to 0-24 range
    while (*sunrise_hour < 0.0) *sunrise_hour += 24.0;
    while (*sunrise_hour >= 24.0) *sunrise_hour -= 24.0;
    while (*sunset_hour < 0.0) *sunset_hour += 24.0;
    while (*sunset_hour >= 24.0) *sunset_hour -= 24.0;
}

/**
 * @brief Convert decimal hours to time_t for a given date
 *
 * Uses fast arithmetic to avoid timezone/DST issues with mktime().
 * Assumes POSIX time_t (seconds since epoch), which ESP-IDF guarantees.
 */
static time_t solar_calc_decimal_hours_to_time(time_t base_time, double hours)
{
    // Truncate to UTC day boundary (86400 = seconds per day)
    // This eliminates timezone/DST offset bugs from gmtime_r/mktime round-trip
    time_t day_start = (base_time / 86400) * 86400;

    // Add decimal hours as seconds
    return day_start + (time_t)(hours * 3600.0);
}

// ########################## Public API Functions ##########################

void solar_calc_init(void)
{
    ESP_LOGI(TAG, "Initializing solar calculator for (%.4f°N, %.4f°E)",
             SOLAR_CALC_LATITUDE, SOLAR_CALC_LONGITUDE);

    solar_times.last_update = 0;
    solar_calc_update();

    // Create midnight timer
    esp_timer_create_args_t timer_args = {
        .callback = solar_calc_midnight_timer_callback,
        .name = "midnight",
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .skip_unhandled_events = false
    };
    esp_err_t ret = esp_timer_create(&timer_args, &xMidnightTimer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create midnight timer: %s", esp_err_to_name(ret));
        return;
    }

    // Schedule first midnight
    solar_calc_schedule_next_midnight();
}

const solar_times_t* solar_calc_get_times(void)
{
    // Return cached solar times
    // Note: Updates are handled by midnight timer callback and solar_calc_init() at startup
    return &solar_times;
}

bool solar_calc_is_daytime(void)
{
    time_t now = time(NULL);
    const solar_times_t *times = solar_calc_get_times();

    // Pure calculation - no state modification
    // (is_daytime field is updated by solar_calc_update())
    return (now >= times->sunrise_time && now <= times->sunset_time);
}

bool solar_calc_is_daytime_buffered(void)
{
    time_t now = time(NULL);
    const solar_times_t *times = solar_calc_get_times();

    // Calculate buffered times (30 min before sunrise, 30 min after sunset)
    time_t buffered_start = times->sunrise_time - (SOLAR_CALC_SUNRISE_BUFFER_MINUTES * 60);
    time_t buffered_end = times->sunset_time + (SOLAR_CALC_SUNSET_BUFFER_MINUTES * 60);

    // Check if current time is within buffered daytime window
    return (now >= buffered_start && now < buffered_end);
}

void solar_calc_update(void)
{
    time_t now = time(NULL);

    // Calculate Julian day for today at noon UTC
    // Use fast arithmetic to avoid timezone/DST bugs with mktime
    time_t day_start = (now / 86400) * 86400;  // UTC midnight
    time_t noon_today = day_start + 12 * 3600;  // UTC noon
    double julian_day = solar_calc_get_julian_day(noon_today);

    // Calculate sunrise and sunset
    double sunrise_hour, sunset_hour;
    solar_calc_calculate_solar_times(julian_day, SOLAR_CALC_LATITUDE, SOLAR_CALC_LONGITUDE,
                                     &sunrise_hour, &sunset_hour);

    // Convert to time_t
    solar_times.sunrise_time = solar_calc_decimal_hours_to_time(now, sunrise_hour);
    solar_times.sunset_time = solar_calc_decimal_hours_to_time(now, sunset_hour);
    solar_times.last_update = now;

    // Update daytime status
    solar_times.is_daytime = (now >= solar_times.sunrise_time && now <= solar_times.sunset_time);

    // Log the calculated times
    struct tm sunrise_tm, sunset_tm;
    localtime_r(&solar_times.sunrise_time, &sunrise_tm);
    localtime_r(&solar_times.sunset_time, &sunset_tm);

    ESP_LOGI(TAG, "Solar times updated - Sunrise: %02d:%02d, Sunset: %02d:%02d (Local time)",
             sunrise_tm.tm_hour, sunrise_tm.tm_min,
             sunset_tm.tm_hour, sunset_tm.tm_min);

    // Sunrise detection: Check if we've crossed buffered sunrise time
    time_t buffered_sunrise = solar_times.sunrise_time - (SOLAR_CALC_SUNRISE_BUFFER_MINUTES * 60);

    // Trigger sunrise callbacks if:
    // 1. We haven't triggered today yet
    // 2. Current time is past buffered sunrise
    // 3. We're still before sunset (within the day)
    if (!sunrise_triggered_today && now >= buffered_sunrise && now < solar_times.sunset_time) {
        ESP_LOGI(TAG, "Sunrise threshold reached - broadcasting to subscribers");
        solar_calc_broadcast_sunrise();
        sunrise_triggered_today = true;
    }

    // Note: sunrise_triggered_today is reset by solar_calc_broadcast_midnight()
    // at precisely 00:00:00 local time via the midnight timer callback
}

// ########################## Sunrise Notification System ##########################

esp_err_t solar_calc_register_sunrise_callback(solar_calc_sunrise_callback_t callback)
{
    if (callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sunrise_callback_count >= SOLAR_CALC_MAX_SUNRISE_CALLBACKS) {
        ESP_LOGE(TAG, "Sunrise callback list full (max %d)", SOLAR_CALC_MAX_SUNRISE_CALLBACKS);
        return ESP_ERR_NO_MEM;
    }

    sunrise_callbacks[sunrise_callback_count++] = callback;
    ESP_LOGI(TAG, "Sunrise callback registered (total: %d)", sunrise_callback_count);

    return ESP_OK;
}

void solar_calc_broadcast_sunrise(void)
{
    ESP_LOGI(TAG, "Broadcasting sunrise to %d subscribers", sunrise_callback_count);

    for (uint8_t i = 0; i < sunrise_callback_count; i++) {
        if (sunrise_callbacks[i] != NULL) {
            sunrise_callbacks[i]();
        }
    }
}

// ########################## Midnight Notification System ##########################

/**
 * @brief Schedule next midnight timer
 * Calculates seconds until next local midnight and arms one-shot timer
 */
static void solar_calc_schedule_next_midnight(void)
{
    time_t now = time(NULL);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    // Calculate seconds until next midnight (local time)
    int seconds_until_midnight =
        (23 - timeinfo.tm_hour) * 3600 +
        (59 - timeinfo.tm_min) * 60 +
        (60 - timeinfo.tm_sec);

    // Handle edge case: if we're very close to midnight, add 24 hours
    if (seconds_until_midnight < 10) {
        seconds_until_midnight += 86400;  // Add 24 hours
        ESP_LOGW(TAG, "Too close to midnight, scheduling for next day");
    }

    // Convert to microseconds for esp_timer
    uint64_t timeout_us = (uint64_t)seconds_until_midnight * 1000000ULL;

    // Start one-shot timer
    esp_err_t ret = esp_timer_start_once(xMidnightTimer, timeout_us);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Next midnight scheduled in %d seconds (%02d:%02d:%02d remaining)",
                 seconds_until_midnight,
                 seconds_until_midnight / 3600,
                 (seconds_until_midnight % 3600) / 60,
                 seconds_until_midnight % 60);
    } else {
        ESP_LOGE(TAG, "Failed to schedule midnight timer: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief Midnight timer callback - triggers day change notifications
 * Called precisely at local midnight by esp_timer
 */
static void solar_calc_midnight_timer_callback(void *arg)
{
    time_t now = time(NULL);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    ESP_LOGI(TAG, "Midnight triggered (precise timer) - %04d-%02d-%02d 00:00:00",
             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);

    // Broadcast midnight event to all registered subscribers
    solar_calc_broadcast_midnight();

    // Schedule next midnight (24 hours from now)
    solar_calc_schedule_next_midnight();
}

esp_err_t solar_calc_register_midnight_callback(solar_calc_midnight_callback_t callback)
{
    if (callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (midnight_callback_count >= SOLAR_CALC_MAX_MIDNIGHT_CALLBACKS) {
        ESP_LOGE(TAG, "Midnight callback list full (max %d)", SOLAR_CALC_MAX_MIDNIGHT_CALLBACKS);
        return ESP_ERR_NO_MEM;
    }

    midnight_callbacks[midnight_callback_count++] = callback;
    ESP_LOGI(TAG, "Midnight callback registered (total: %d)", midnight_callback_count);

    return ESP_OK;
}

void solar_calc_broadcast_midnight(void)
{
    ESP_LOGI(TAG, "Broadcasting midnight to %d subscribers", midnight_callback_count);

    // Reset sunrise trigger for the new day (single source of truth for day change)
    sunrise_triggered_today = false;

    // Recalculate solar times for the new day
    // This also checks for sunrise threshold and broadcasts sunrise callbacks if needed
    solar_calc_update();

    // Broadcast midnight to all registered subscribers
    for (uint8_t i = 0; i < midnight_callback_count; i++) {
        if (midnight_callbacks[i] != NULL) {
            midnight_callbacks[i]();
        }
    }
}
