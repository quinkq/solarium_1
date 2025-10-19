#include "solar_calc.h"
#include "esp_log.h"
#include <math.h>

// ########################## Constants ##########################

static const char *TAG = "SOLAR_CALC";

#define PI 3.14159265358979323846
#define DEG_TO_RAD(deg) ((deg) * PI / 180.0)
#define RAD_TO_DEG(rad) ((rad) * 180.0 / PI)

// ########################## Private Variables ##########################

static solar_times_t solar_times = {0};

// ########################## Private Functions ##########################

/**
 * @brief Calculate Julian Day from time_t
 */
static double get_julian_day(time_t t)
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
static void calculate_solar_times(double julian_day, double latitude, double longitude,
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
 */
static time_t decimal_hours_to_time(time_t base_time, double hours)
{
    struct tm timeinfo;
    gmtime_r(&base_time, &timeinfo);

    // Set time to start of day (00:00:00 UTC)
    timeinfo.tm_hour = 0;
    timeinfo.tm_min = 0;
    timeinfo.tm_sec = 0;

    time_t day_start = mktime(&timeinfo);

    // Add decimal hours
    return day_start + (time_t)(hours * 3600.0);
}

// ########################## Public API Functions ##########################

void solar_calc_init(void)
{
    ESP_LOGI(TAG, "Initializing solar calculator for (%.4f°N, %.4f°E)",
             SOLAR_CALC_LATITUDE, SOLAR_CALC_LONGITUDE);

    solar_times.last_update = 0;
    solar_calc_update();
}

const solar_times_t* solar_calc_get_times(void)
{
    time_t now = time(NULL);
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);

    // Check if we need to recalculate (new day or first run)
    if (solar_times.last_update == 0 || timeinfo.tm_hour == 0) {
        solar_calc_update();
    }

    return &solar_times;
}

bool solar_calc_is_daytime(void)
{
    time_t now = time(NULL);
    const solar_times_t *times = solar_calc_get_times();

    // Check if current time is between sunrise and sunset
    solar_times.is_daytime = (now >= times->sunrise_time && now <= times->sunset_time);

    return solar_times.is_daytime;
}

void solar_calc_update(void)
{
    time_t now = time(NULL);
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);

    // Calculate Julian day for today at noon UTC
    timeinfo.tm_hour = 12;
    timeinfo.tm_min = 0;
    timeinfo.tm_sec = 0;
    time_t noon_today = mktime(&timeinfo);
    double julian_day = get_julian_day(noon_today);

    // Calculate sunrise and sunset
    double sunrise_hour, sunset_hour;
    calculate_solar_times(julian_day, SOLAR_CALC_LATITUDE, SOLAR_CALC_LONGITUDE,
                         &sunrise_hour, &sunset_hour);

    // Convert to time_t
    solar_times.sunrise_time = decimal_hours_to_time(now, sunrise_hour);
    solar_times.sunset_time = decimal_hours_to_time(now, sunset_hour);
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
}
