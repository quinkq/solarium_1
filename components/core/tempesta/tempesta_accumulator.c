/**
 * @file tempesta_accumulator.c
 * @brief TEMPESTA rainfall and tank intake accumulation tracking
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Handles time-based accumulation for tipbucket sensors:
 * - Hourly rollover tracking (completed hour + current hour in progress)
 * - Daily accumulation (midnight reset via solar_calc callback)
 * - Weekly accumulation (boot day reset to prevent overflow)
 * - Hardware counter management (32767 pulse limit protection)
 * - Pulse-to-physical unit conversions (mm rainfall depth, mL volume)
 * - Public reset APIs for manual counter management
 *
 * Uses monotonic time for hourly tracking and solar_calc midnight callback
 * for daily resets. Weekly reset on boot day clears hardware counters to
 * prevent overflow during heavy rain weeks.
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "tempesta.h"
#include "tempesta_private.h"
#include "mcp23008_helper.h"

#include "solar_calc.h"

static const char *TAG = "TEMPESTA_ACCUMULATOR";


/**
 * @brief Process rainfall accumulation with hourly/daily calculations and data update
 */
esp_err_t tempesta_read_and_process_rainfall(void)
{
    int pulse_count = 0;
    esp_err_t ret = tempesta_get_rainfall_pulse_count(&pulse_count);
    if (ret != ESP_OK) {
        // Update weather data with error state
        if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
            weather_data.rainfall_last_hour_mm = WEATHER_INVALID_VALUE;
            weather_data.rainfall_current_hour_mm = WEATHER_INVALID_VALUE;
            weather_data.rainfall_daily_mm = WEATHER_INVALID_VALUE;
            weather_data.rainfall_weekly_mm = WEATHER_INVALID_VALUE;
            weather_data.rain_gauge_status = WEATHER_SENSOR_ERROR;
            xSemaphoreGive(xTempestaDataMutex);
        }
        return ret;
    }

    // Acquire mutex to protect calculation_data and weather_data access
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for rainfall processing");
        return ESP_FAIL;
    }

    // Check if hour has completed
    int64_t current_time_ms = esp_timer_get_time() / 1000;
    int64_t elapsed_ms = current_time_ms - calculation_data.rainfall_hourly_start_time_ms;

    if (elapsed_ms >= 3600000) { // Hour completed
        // Store completed hour measurement
        int pulses_this_hour = pulse_count - calculation_data.rainfall_hourly_start_pulses;
        weather_data.rainfall_last_hour_mm = tempesta_convert_pulses_to_rainfall_mm(pulses_this_hour);

        // Reset tracking for next hour
        calculation_data.rainfall_hourly_start_time_ms = current_time_ms;
        calculation_data.rainfall_hourly_start_pulses = pulse_count;

        ESP_LOGI(TAG, "Rainfall: Completed hour = %.3f mm", weather_data.rainfall_last_hour_mm);
    }

    // Always calculate current hour accumulation (for debug/monitoring)
    int pulses_current_hour = pulse_count - calculation_data.rainfall_hourly_start_pulses;
    weather_data.rainfall_current_hour_mm = tempesta_convert_pulses_to_rainfall_mm(pulses_current_hour);

    // Calculate daily and weekly accumulation from base pulse counts
    int pulses_since_midnight = pulse_count - calculation_data.rainfall_daily_base_pulses;
    int pulses_since_weekly_reset = pulse_count - calculation_data.rainfall_weekly_base_pulses;
    weather_data.rainfall_daily_mm = tempesta_convert_pulses_to_rainfall_mm(pulses_since_midnight);
    weather_data.rainfall_weekly_mm = tempesta_convert_pulses_to_rainfall_mm(pulses_since_weekly_reset);
    weather_data.rain_gauge_status = WEATHER_SENSOR_OK;

    xSemaphoreGive(xTempestaDataMutex);

    ESP_LOGD(TAG, "Rain: Last=%.3f mm, Current=%.3f mm, Daily=%.3f mm, Weekly=%.3f mm",
             weather_data.rainfall_last_hour_mm, weather_data.rainfall_current_hour_mm,
             weather_data.rainfall_daily_mm, weather_data.rainfall_weekly_mm);
    return ESP_OK;
}

/**
 * @brief Process tank intake accumulation with hourly/daily calculations and data update
 */
esp_err_t tempesta_read_and_process_tank_intake(void)
{
    int pulse_count = 0;
    esp_err_t ret = tempesta_get_tank_intake_pulse_count(&pulse_count);
    if (ret != ESP_OK) {
        // Update weather data with error state
        if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
            weather_data.tank_intake_last_hour_ml = WEATHER_INVALID_VALUE;
            weather_data.tank_intake_current_hour_ml = WEATHER_INVALID_VALUE;
            weather_data.tank_intake_daily_ml = WEATHER_INVALID_VALUE;
            weather_data.tank_intake_weekly_ml = WEATHER_INVALID_VALUE;
            weather_data.tank_intake_status = WEATHER_SENSOR_ERROR;
            xSemaphoreGive(xTempestaDataMutex);
        }
        return ret;
    }

    // Acquire mutex to protect calculation_data and weather_data access
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for tank intake processing");
        return ESP_FAIL;
    }

    // Check if hour has completed
    int64_t current_time_ms = esp_timer_get_time() / 1000;
    int64_t elapsed_ms = current_time_ms - calculation_data.tank_intake_hourly_start_time_ms;

    if (elapsed_ms >= 3600000) { // Hour completed
        // Store completed hour measurement
        int pulses_this_hour = pulse_count - calculation_data.tank_intake_hourly_start_pulses;
        weather_data.tank_intake_last_hour_ml = tempesta_convert_pulses_to_tank_intake_ml(pulses_this_hour);

        // Reset tracking for next hour
        calculation_data.tank_intake_hourly_start_time_ms = current_time_ms;
        calculation_data.tank_intake_hourly_start_pulses = pulse_count;

        ESP_LOGI(TAG, "Tank intake: Completed hour = %.1f mL", weather_data.tank_intake_last_hour_ml);
    }

    // Always calculate current hour accumulation (for debug/monitoring)
    int pulses_current_hour = pulse_count - calculation_data.tank_intake_hourly_start_pulses;
    weather_data.tank_intake_current_hour_ml = tempesta_convert_pulses_to_tank_intake_ml(pulses_current_hour);

    // Calculate daily and weekly accumulation from base pulse counts
    int pulses_since_midnight = pulse_count - calculation_data.tank_intake_daily_base_pulses;
    int pulses_since_weekly_reset = pulse_count - calculation_data.tank_intake_weekly_base_pulses;
    weather_data.tank_intake_daily_ml = tempesta_convert_pulses_to_tank_intake_ml(pulses_since_midnight);
    weather_data.tank_intake_weekly_ml = tempesta_convert_pulses_to_tank_intake_ml(pulses_since_weekly_reset);
    weather_data.tank_intake_status = WEATHER_SENSOR_OK;

    xSemaphoreGive(xTempestaDataMutex);

    ESP_LOGD(TAG, "Tank: Last=%.1f mL, Current=%.1f mL, Daily=%.1f mL, Weekly=%.1f mL",
             weather_data.tank_intake_last_hour_ml, weather_data.tank_intake_current_hour_ml,
             weather_data.tank_intake_daily_ml, weather_data.tank_intake_weekly_ml);
    return ESP_OK;
}

/**
 * @brief Convert pulse count to rainfall depth in mm using physical tipbucket parameters
 */
float tempesta_convert_pulses_to_rainfall_mm(int pulse_count)
{
    if (pulse_count < 0) {
        return 0.0f;
    }

    // Calculate rainfall depth: volume / area
    // Depth(mm) = (pulses * volume_per_pulse_mm3) / collection_area_mm2
    float total_volume_mm3 = (float) pulse_count * (WEATHER_RAIN_ML_PER_PULSE * 1000); // Multiplying due to conversion from cm3 (mL) to mm3.
    float rainfall_depth_mm = total_volume_mm3 / WEATHER_RAIN_COLLECTION_AREA_MM2;

    return rainfall_depth_mm;
}

/**
 * @brief Convert pulse count to tank intake volume in mL
 */
float tempesta_convert_pulses_to_tank_intake_ml(int pulse_count)
{
    if (pulse_count < 0) {
        return 0.0f;
    }

    // Each pulse represents one tipbucket tip
    return (float) pulse_count * WEATHER_TANK_INTAKE_ML_PER_PULSE;
}

/**
 * @brief Daily reset callback triggered at midnight
 * Resets daily accumulation counters for rain gauge and tank intake
 * On weekly reset day (set at first boot): Also clears hardware counters to prevent overflow
 */
void tempesta_daily_reset_callback(void)
{
    static const char *day_names[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
    static int weekly_reset_day = -1;  // -1 = not initialized, will be set on first run

    // Get current day of week (0=Sunday, 1=Monday, ..., 6=Saturday)
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    int day_of_week = timeinfo.tm_wday;

    // Initialize weekly reset day on first run (boot day)
    if (weekly_reset_day == -1) {
        weekly_reset_day = day_of_week;
        ESP_LOGI(TAG, "Weekly reset day initialized to %s (boot day)", day_names[weekly_reset_day]);
    }

    bool is_weekly_reset = (day_of_week == weekly_reset_day);

    if (is_weekly_reset) {
        ESP_LOGI(TAG, "%s midnight - HARDWARE COUNTER RESET + daily/weekly reset", day_names[day_of_week]);

        // Clear MCP23008 helper counters to prevent overflow (max uint32_t)
        // This ensures counters don't overflow during heavy rain/usage
        mcp23008_helper_reset_rainfall_pulses();
        mcp23008_helper_reset_tank_intake_pulses();

        // Acquire mutex to protect calculation_data and weather_data access
        if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to acquire mutex in daily_reset_callback (Monday)");
            return;
        }

        // Reset all base pulse counts to 0 (hardware counters now at 0)
        calculation_data.rainfall_hourly_start_pulses = 0;
        calculation_data.rainfall_daily_base_pulses = 0;
        calculation_data.rainfall_weekly_base_pulses = 0;
        calculation_data.tank_intake_hourly_start_pulses = 0;
        calculation_data.tank_intake_daily_base_pulses = 0;
        calculation_data.tank_intake_weekly_base_pulses = 0;

        // Update weather data to reflect reset
        weather_data.rainfall_last_hour_mm = 0.0f;
        weather_data.rainfall_current_hour_mm = 0.0f;
        weather_data.rainfall_daily_mm = 0.0f;
        weather_data.rainfall_weekly_mm = 0.0f;
        weather_data.tank_intake_last_hour_ml = 0.0f;
        weather_data.tank_intake_current_hour_ml = 0.0f;
        weather_data.tank_intake_daily_ml = 0.0f;
        weather_data.tank_intake_weekly_ml = 0.0f;

        xSemaphoreGive(xTempestaDataMutex);

        ESP_LOGI(TAG, "Hardware counters cleared, all accumulators reset to 0");
    } else {
        ESP_LOGI(TAG, "Midnight reset - clearing daily accumulation counters");

        // Get current pulse counts to set as new daily base (hardware NOT cleared)
        int rain_pulses = 0, tank_pulses = 0;
        tempesta_get_rainfall_pulse_count(&rain_pulses);
        tempesta_get_tank_intake_pulse_count(&tank_pulses);

        // Acquire mutex to protect calculation_data and weather_data access
        if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to acquire mutex in daily_reset_callback");
            return;
        }

        // Update base pulse counts for daily tracking
        calculation_data.rainfall_daily_base_pulses = rain_pulses;
        calculation_data.tank_intake_daily_base_pulses = tank_pulses;

        // Update weather data to reflect daily reset
        weather_data.rainfall_daily_mm = 0.0f;
        weather_data.tank_intake_daily_ml = 0.0f;

        xSemaphoreGive(xTempestaDataMutex);

        ESP_LOGI(TAG, "Daily counters reset (rain=%d pulses, tank=%d pulses)", rain_pulses, tank_pulses);
    }
}

// ########################## Public APIs ##########################

/**
 * @brief Reset daily accumulation counters (rainfall and tank intake)
 */
esp_err_t tempesta_reset_daily_counters(void)
{
    ESP_LOGI(TAG, "Resetting daily accumulation counters");

    // Get current pulse counts to set as new base
    int rain_pulses = 0, tank_pulses = 0;
    tempesta_get_rainfall_pulse_count(&rain_pulses);
    tempesta_get_tank_intake_pulse_count(&tank_pulses);

    // Acquire mutex to protect calculation_data and weather_data access
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for daily counter reset");
        return ESP_FAIL;
    }

    // Update base pulse counts for daily tracking
    calculation_data.rainfall_daily_base_pulses = rain_pulses;
    calculation_data.tank_intake_daily_base_pulses = tank_pulses;

    // Update weather data to reflect reset
    weather_data.rainfall_daily_mm = 0.0f;
    weather_data.tank_intake_daily_ml = 0.0f;

    xSemaphoreGive(xTempestaDataMutex);

    ESP_LOGI(TAG, "Daily counters reset complete (rain=%d pulses, tank=%d pulses)",
             rain_pulses, tank_pulses);
    return ESP_OK;
}

/**
 * @brief Reset weekly accumulation counters (rainfall and tank intake)
 */
esp_err_t tempesta_reset_weekly_counters(void)
{
    ESP_LOGI(TAG, "Resetting weekly accumulation counters");

    // Get current pulse counts to set as new base
    int rain_pulses = 0, tank_pulses = 0;
    tempesta_get_rainfall_pulse_count(&rain_pulses);
    tempesta_get_tank_intake_pulse_count(&tank_pulses);

    // Acquire mutex to protect calculation_data and weather_data access
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for weekly counter reset");
        return ESP_FAIL;
    }

    // Update base pulse counts for weekly tracking
    calculation_data.rainfall_weekly_base_pulses = rain_pulses;
    calculation_data.tank_intake_weekly_base_pulses = tank_pulses;

    // Update weather data to reflect reset
    weather_data.rainfall_weekly_mm = 0.0f;
    weather_data.tank_intake_weekly_ml = 0.0f;

    xSemaphoreGive(xTempestaDataMutex);

    ESP_LOGI(TAG, "Weekly counters reset complete (rain=%d pulses, tank=%d pulses)",
             rain_pulses, tank_pulses);
    return ESP_OK;
}

/**
 * @brief Reset rain gauge accumulation (clears MCP23008 helper counter)
 * WARNING: This resets hourly/daily/weekly data. Consider using
 * tempesta_reset_daily_counters() or tempesta_reset_weekly_counters() instead.
 */
esp_err_t tempesta_reset_rain_gauge_total(void)
{
    ESP_LOGI(TAG, "Resetting rain gauge (MCP23008 counter clear)");

    // Clear MCP23008 helper counter
    mcp23008_helper_reset_rainfall_pulses();

    // Acquire mutex to protect calculation_data and weather_data access
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for rain gauge reset");
        return ESP_FAIL;
    }

    // Reset all base pulse counts to 0
    calculation_data.rainfall_hourly_start_pulses = 0;
    calculation_data.rainfall_daily_base_pulses = 0;
    calculation_data.rainfall_weekly_base_pulses = 0;

    // Reset all accumulation values
    weather_data.rainfall_last_hour_mm = 0.0f;
    weather_data.rainfall_current_hour_mm = 0.0f;
    weather_data.rainfall_daily_mm = 0.0f;
    weather_data.rainfall_weekly_mm = 0.0f;

    xSemaphoreGive(xTempestaDataMutex);

    ESP_LOGI(TAG, "Rain gauge reset complete");
    return ESP_OK;
}

/**
 * @brief Reset tank intake accumulation (clears MCP23008 helper counter)
 * WARNING: This resets hourly/daily/weekly data. Consider using
 * tempesta_reset_daily_counters() or tempesta_reset_weekly_counters() instead.
 */
esp_err_t tempesta_reset_tank_intake_total(void)
{
    ESP_LOGI(TAG, "Resetting tank intake (MCP23008 counter clear)");

    // Clear MCP23008 helper counter
    mcp23008_helper_reset_tank_intake_pulses();

    // Acquire mutex to protect calculation_data and weather_data access
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for tank intake reset");
        return ESP_FAIL;
    }

    // Reset all base pulse counts to 0
    calculation_data.tank_intake_hourly_start_pulses = 0;
    calculation_data.tank_intake_daily_base_pulses = 0;
    calculation_data.tank_intake_weekly_base_pulses = 0;

    // Reset all accumulation values
    weather_data.tank_intake_last_hour_ml = 0.0f;
    weather_data.tank_intake_current_hour_ml = 0.0f;
    weather_data.tank_intake_daily_ml = 0.0f;
    weather_data.tank_intake_weekly_ml = 0.0f;

    xSemaphoreGive(xTempestaDataMutex);

    ESP_LOGI(TAG, "Tank intake reset complete");
    return ESP_OK;
}

/**
 * @brief Reset all accumulation counters (total, weekly, daily for both sensors)
 */
esp_err_t tempesta_reset_all_counters(void)
{
    ESP_LOGI(TAG, "Resetting ALL accumulation counters");

    esp_err_t ret1 = tempesta_reset_rain_gauge_total();
    esp_err_t ret2 = tempesta_reset_tank_intake_total();

    if (ret1 == ESP_OK && ret2 == ESP_OK) {
        ESP_LOGI(TAG, "All counters reset successfully");
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Some counters failed to reset (rain=%s, tank=%s)",
             esp_err_to_name(ret1), esp_err_to_name(ret2));
    return ESP_FAIL;
}