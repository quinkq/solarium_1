/**
 * @file impluvium_learning.c
 * @brief Learning algorithm for IMPLUVIUM irrigation system
 *
 * Implements adaptive watering predictions based on historical data,
 * temperature correction, confidence tracking, and interval calculation.
 */

#include "impluvium.h"
#include "impluvium_private.h"
#include "tempesta.h"
#include "solar_calc.h"
#include "interval_config.h"
#include <math.h>
#include <string.h>

static const char *TAG = "IMPLUVIUM_LEARN";

/**
 * @brief Calculate dynamic moisture check interval based on temperature and power state
 *
 * Adjusts moisture checking frequency based on:
 * - Power save mode (configured interval)
 * - Temperature (optimal vs cool)
 * - Day/night cycle (nighttime minimum)
 * - Safety checks (skip below minimum temperature)
 *
 * @param current_temperature Current temperature in degrees Celsius
 * @return Interval in milliseconds, or UINT32_MAX to skip moisture checks
 */
uint32_t impluvium_calc_moisture_check_interval(float current_temperature)
{
    uint32_t interval;

    // Check for power save mode override (from config)
    if (irrigation_system.power_save_mode) {
        ESP_LOGD(TAG, "Power save mode active - using configured interval");
        interval = impluvium_power_save_interval_ms;
    }
    else if (current_temperature == WEATHER_INVALID_VALUE) {
        ESP_LOGW(TAG, "Invalid temperature - using optimal interval as fallback");
        interval = impluvium_optimal_interval_ms;
    }
    else if (current_temperature < MIN_TEMPERATURE_WATERING) {
        ESP_LOGI(TAG, "Temperature %.1f°C below watering threshold - skipping moisture checks", current_temperature);
        return UINT32_MAX; // Skip moisture checks completely (safety)
    }
    else if (current_temperature >= TEMPERATURE_OPTIMAL_THRESHOLD) {
        ESP_LOGD(TAG, "Optimal temperature %.1f°C - using optimal interval (from config)", current_temperature);
        interval = impluvium_optimal_interval_ms;
    }
    else {
        ESP_LOGD(TAG, "Cool temperature %.1f°C - using cool interval (from config)", current_temperature);
        interval = impluvium_cool_interval_ms;
    }

    // Apply nighttime minimum (from config) to reduce unnecessary checks during darkness
    if (!solar_calc_is_daytime_buffered()) {
        if (interval < impluvium_night_minimum_ms) {
            ESP_LOGD(TAG, "Nighttime - extending interval from %lums to %lums", interval, impluvium_night_minimum_ms);
            interval = impluvium_night_minimum_ms;
        }
    }

    return interval;
}

/**
 * @brief Calculate temperature correction factor for watering predictions
 *
 * @param[in] zone_learning Zone learning data structure
 * @return Temperature correction factor (typically 0.8 - 1.2)
 */
float impluvium_calculate_temperature_correction(zone_learning_t *learning)
{
    // Calculate temperature correction factor
    // Formula: 1.0 + (current_temperature - baseline_temp) * correction_factor
    // Example: At 30°C: 1.0 + (30-20) * 0.01 = 1.10 (10% more water)
    //          At 10°C: 1.0 + (10-20) * 0.01 = 0.90 (10% less water)
    float current_temperature = tempesta_get_temperature();

    if (current_temperature == WEATHER_INVALID_VALUE) {
        ESP_LOGW(TAG, "Temperature sensors failed - using baseline correction");
        learning->last_temperature_correction = 1.0f;
        return 1.0f;
    }

    float calculated_temp_correction = 1.0f + ((current_temperature - TEMPERATURE_BASELINE) * TEMP_CORRECTION_FACTOR);
    learning->last_temperature_correction = calculated_temp_correction;
    ESP_LOGD(TAG, "Temp %.1f°C, correction factor %.2f", current_temperature, calculated_temp_correction);
    return calculated_temp_correction;
}

/**
 * @brief Calculate weighted learning ratio from historical data
 *
 * @param[in] learning Zone learning data
 * @param[out] valid_cycles Number of valid historical cycles found
 * @return Calculated pulses per percent ratio, or 0.0 if insufficient data
 */
float impluvium_calculate_pulse_per_moisture_percent(zone_learning_t *learning, uint8_t *valid_cycles)
{
    float weighted_ratio = 0.0f;
    float total_weight = 0.0f;
    *valid_cycles = 0;

    for (uint8_t h = 0; h < learning->history_entry_count; h++) {
        // Skip anomalous cycles (rain, manual watering, etc.)
        if (!learning->anomaly_flags[h] && learning->moisture_increase_percent_history[h] > 0) {
            // Calculate recency: most recent entries get higher weight
            // Recent entries are at (history_index - 1), (history_index - 2), etc. with wraparound
            uint8_t relative_age = (learning->history_index - h - 1 + LEARNING_HISTORY_SIZE) % LEARNING_HISTORY_SIZE;
            float weight = (relative_age < 3) ? LEARNING_WEIGHT_RECENT : (1.0f - LEARNING_WEIGHT_RECENT);

            // Calculate pulses per moisture percent for this cycle
            float ratio = learning->pulses_used_history[h] / learning->moisture_increase_percent_history[h];

            weighted_ratio += ratio * weight;
            total_weight += weight;
            (*valid_cycles)++;

            ESP_LOGD(TAG,
                     "Cycle %d (age %d): %d pulses, %.2f%% increase, ratio %.1f, weight %.2f",
                     h,
                     relative_age,
                     (int) learning->pulses_used_history[h],
                     learning->moisture_increase_percent_history[h],
                     ratio,
                     weight);
        }
    }

    return (total_weight > 0 && *valid_cycles >= 2) ? (weighted_ratio / total_weight) : 0.0f;
}

/**
 * @brief Calculate target pulses for a single zone
 *
 * @param[in] zone_id Zone identifier
 * @param[in] queue_index Index in watering queue
 * @return ESP_OK on success
 */
esp_err_t impluvium_calculate_zone_target_pulses(uint8_t zone_id, uint8_t queue_index)
{
    irrigation_zone_t *zone = &irrigation_zones[zone_id];
    zone_learning_t *learning = &zone->learning;

    // Calculate temperature correction
    float calculated_temp_correction = impluvium_calculate_temperature_correction(learning);

    // Check if we have sufficient learning data
    if (learning->history_entry_count >= LEARNING_MIN_CYCLES) {
        uint8_t valid_cycles;
        float calculated_ppmp_ratio = impluvium_calculate_pulse_per_moisture_percent(learning, &valid_cycles);

        if (calculated_ppmp_ratio > 0.0f) {
            learning->calculated_ppmp_ratio = calculated_ppmp_ratio;

            // Calculate learned prediction
            float learned_target_pulses = irrigation_system.watering_queue[queue_index].moisture_deficit_percent *
                                         calculated_ppmp_ratio * calculated_temp_correction;

            // Calculate default prediction for blending
            float default_target_pulses = irrigation_system.watering_queue[queue_index].moisture_deficit_percent *
                                        DEFAULT_PULSES_PER_PERCENT * calculated_temp_correction;

            // Apply confidence-based blending
            float target_pulses;
            float confidence = learning->confidence_level;

            if (confidence >= 0.70f) {
                // High confidence: Use full learned prediction
                target_pulses = learned_target_pulses;
                ESP_LOGI(TAG, "Zone %d: High confidence (%.0f%%), using learned prediction",
                        zone_id, confidence * 100.0f);
            } else if (confidence >= 0.40f) {
                // Medium confidence: Blend learned + default
                float blend_factor = (confidence - 0.40f) / 0.30f; // 0.0 to 1.0 scale
                target_pulses = (learned_target_pulses * blend_factor) + (default_target_pulses * (1.0f - blend_factor));
                ESP_LOGI(TAG, "Zone %d: Medium confidence (%.0f%%), blending predictions (%.0f%% learned)",
                        zone_id, confidence * 100.0f, blend_factor * 100.0f);
            } else {
                // Low confidence: Use mostly default with slight learned influence
                target_pulses = (default_target_pulses * 0.8f) + (learned_target_pulses * 0.2f);
                ESP_LOGI(TAG, "Zone %d: Low confidence (%.0f%%), using mostly default prediction",
                        zone_id, confidence * 100.0f);
            }

            // Limit to reasonable range
            if (target_pulses < MINIMUM_TARGET_PULSES)
                target_pulses = MINIMUM_TARGET_PULSES;
            if (target_pulses > MAXIMUM_TARGET_PULSES)
                target_pulses = MAXIMUM_TARGET_PULSES;

            irrigation_system.watering_queue[queue_index].target_pulses = (uint16_t) target_pulses;

            ESP_LOGI(TAG,
                     "Zone %d: Final prediction %d pulses (learned: %.1f, default: %.1f pulses/%%, temp: %.2f°C)",
                     zone_id,
                     irrigation_system.watering_queue[queue_index].target_pulses,
                     calculated_ppmp_ratio,
                     DEFAULT_PULSES_PER_PERCENT,
                     calculated_temp_correction);
        } else {
            // Not enough valid learning data
            irrigation_system.watering_queue[queue_index].target_pulses = DEFAULT_TARGET_PULSES;
            ESP_LOGW(TAG,
                     "Zone %d: Insufficient valid learning data (%d cycles), using default %d pulses",
                     zone_id,
                     valid_cycles,
                     DEFAULT_TARGET_PULSES);
        }
    } else {
        // Not enough learning cycles - use default
        irrigation_system.watering_queue[queue_index].target_pulses = DEFAULT_TARGET_PULSES;
        ESP_LOGI(TAG,
                 "Zone %d: Learning phase (%d/%d cycles), using default %d pulses",
                 zone_id,
                 learning->history_entry_count,
                 LEARNING_MIN_CYCLES,
                 DEFAULT_TARGET_PULSES);
    }

    ESP_LOGI(TAG,
             "Queue[%d]: Zone %d, deficit %.2f%%, target %d pulses",
             queue_index,
             zone_id,
             irrigation_system.watering_queue[queue_index].moisture_deficit_percent,
             irrigation_system.watering_queue[queue_index].target_pulses);

    return ESP_OK;
}

/**
 * @brief Calculate watering predictions for all zones in queue
 *
 * Orchestrates the learning algorithm to predict optimal water amounts
 * for each zone in the watering queue.
 *
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t impluvium_calculate_zone_watering_predictions(void)
{
    ESP_LOGI(TAG, "Calculating watering predictions for %d zones", irrigation_system.watering_queue_size);

    for (uint8_t i = 0; i < irrigation_system.watering_queue_size; i++) {
        uint8_t zone_id = irrigation_system.watering_queue[i].zone_id;
        esp_err_t ret = impluvium_calculate_zone_target_pulses(zone_id, i);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to calculate target pulses for zone %d: %s", zone_id, esp_err_to_name(ret));
            return ret;
        }
    }

    return ESP_OK;
}

/**
 * @brief Update learning algorithm with post-watering data
 *
 * This function records the results of a watering cycle for future learning:
 * 1. Stores pulses used and moisture increase achieved
 * 2. Marks anomalous cycles (rain, manual watering, sensor errors)
 * 3. Maintains circular buffer of historical data
 * 4. Updates learning statistics
 *
 * Anomaly Detection Criteria:
 * - Excessive moisture increase (>0.3V indicates rain/manual watering)
 * - Extreme temperature conditions (<5°C or >45°C)
 * - Flow anomalies detected during watering
 * - Other system-detected anomalies
 *
 * @param zone_id Zone that was watered (0-4)
 * @param pulses_used Number of flow sensor pulses during watering
 * @param moisture_increase_percent Percentage increase observed in moisture sensor (final - initial)
 * @param learning_valid Whether this cycle should be used for learning
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid parameters
 */
esp_err_t impluvium_process_zone_watering_data(uint8_t zone_id,
                                                   uint32_t pulses_used,
                                                   float moisture_increase_percent,
                                                   bool learning_valid)
{
    if (zone_id >= IRRIGATION_ZONE_COUNT) {
        ESP_LOGE(TAG, "Invalid zone_id %d for learning update", zone_id);
        return ESP_ERR_INVALID_ARG;
    }

    irrigation_zone_t *zone = &irrigation_zones[zone_id];
    zone_learning_t *learning = &zone->learning;

    // Get current write position in circular buffer
    uint8_t index = learning->history_index;

    // Store the learning data
    learning->pulses_used_history[index] = pulses_used;
    learning->moisture_increase_percent_history[index] = moisture_increase_percent;
    learning->anomaly_flags[index] = !learning_valid; // Invert: true = anomaly (stored for future calculations)

    // Advance circular buffer index
    learning->history_index = (learning->history_index + 1) % LEARNING_HISTORY_SIZE;

    // Update count (saturates at LEARNING_HISTORY_SIZE)
    if (learning->history_entry_count < LEARNING_HISTORY_SIZE) {
        learning->history_entry_count++;
    }

    // Calculate basic statistics for this cycle (pulses per 1% moisture)
    float measured_ppmp_ratio = (moisture_increase_percent > 0) ? (pulses_used / moisture_increase_percent) : 0;

    ESP_LOGI(TAG,
             "Zone %d learning update: %lu pulses → %.2f%% increase (%.1f pulses/%%), %s",
             zone_id,
             pulses_used,
             moisture_increase_percent,
             measured_ppmp_ratio,
             learning_valid ? "valid" : "anomaly");

    // Real-time learning updates (only for valid cycles)
    if (learning_valid && measured_ppmp_ratio > 0.1f && measured_ppmp_ratio < 50.0f) { // Reasonable measured_ppmp_ratio range
        // Update learned measured_ppmp_ratio with exponential moving average (10% new, 90% old)
        learning->calculated_ppmp_ratio =
            (learning->calculated_ppmp_ratio * 0.9f) + (measured_ppmp_ratio * 0.1f);

        ESP_LOGD(TAG, "Zone %d: Real-time measured_ppmp_ratio update: %.1f pulses/%% (was %.1f)",
                 zone_id, learning->calculated_ppmp_ratio, measured_ppmp_ratio);
    }

    // ### Confidence tracking calculations (only for valid predictions)
    if (learning_valid) {
        learning->total_predictions++;

        // Check if prediction was reasonably accurate (within 20% of expected)
        if (learning->calculated_ppmp_ratio > 0) {
            float expected_increase = (pulses_used / learning->calculated_ppmp_ratio);
            float accuracy = fabs(moisture_increase_percent - expected_increase) / expected_increase;
            if (accuracy <= 0.20f) { // Within 20% tolerance
                learning->successful_predictions++;
            }
        }

        // Update confidence level based on success rate
        if (learning->total_predictions > 0) {
            learning->confidence_level = (float) learning->successful_predictions / learning->total_predictions;
        }
    }

    // Update learned parameters and log progress (only after sufficient learning cycles)
    if (learning->history_entry_count >= LEARNING_MIN_CYCLES) {
        // Update pump duty cycle (only for valid cycles)
        if (learning_valid) {
            // Exponential moving average for pump duty cycle (10% new, 90% old)
            learning->calculated_pump_duty_cycle =
                (uint32_t)((learning->calculated_pump_duty_cycle * 0.9f) + (irrigation_system.pump_pwm_duty * 0.1f));

            ESP_LOGI(TAG,
                     "Zone %d: Updated learned pump duty cycle to %" PRIu32 ", confidence %.2f",
                     zone_id,
                     learning->calculated_pump_duty_cycle,
                     learning->confidence_level);
        }

        // Count valid entries for logging (cached calculation instead of loop each time)
        uint8_t valid_entries = 0;
        for (uint8_t i = 0; i < learning->history_entry_count; i++) {
            if (!learning->anomaly_flags[i])
                valid_entries++;
        }

        ESP_LOGI(TAG,
                 "Zone %d learning: %d total cycles, %d valid, %lu/%lu predictions successful (%.1f%% confidence)",
                 zone_id,
                 learning->history_entry_count,
                 valid_entries,
                 learning->successful_predictions,
                 learning->total_predictions,
                 learning->confidence_level * 100.0f);
    }

    return ESP_OK;
}

/**
 * @brief Update runtime interval configuration
 *
 * Validates and applies new interval settings for moisture check timing.
 * Called by HMI or MQTT commands to adjust irrigation scheduling.
 *
 * @param optimal_min Optimal temperature interval (minutes)
 * @param cool_min Cool temperature interval (minutes)
 * @param power_save_min Power save mode interval (minutes)
 * @param night_hours Nighttime minimum interval (hours)
 * @return ESP_OK on success, error code if validation fails
 */
esp_err_t impluvium_set_check_intervals(uint32_t optimal_min, uint32_t cool_min,
                                        uint32_t power_save_min, uint32_t night_hours)
{
    // Validate using interval_config API (enforces ranges)
    esp_err_t ret = interval_config_set_impluvium(optimal_min, cool_min, power_save_min, night_hours);
    if (ret != ESP_OK) {
        return ret;  // Invalid ranges, error already logged
    }

    // Update runtime variables
    impluvium_optimal_interval_ms = optimal_min * 60 * 1000;
    impluvium_cool_interval_ms = cool_min * 60 * 1000;
    impluvium_power_save_interval_ms = power_save_min * 60 * 1000;
    impluvium_night_minimum_ms = night_hours * 60 * 60 * 1000;

    // Timer callback will automatically use new values on next cycle
    ESP_LOGI(TAG, "Check intervals updated: %lum (opt), %lum (cool), %lum (pwr), %luh (night)",
             optimal_min, cool_min, power_save_min, night_hours);

    return ESP_OK;
}

/**
 * @brief Reset learning data for specific zone to defaults
 */
esp_err_t impluvium_reset_zone_learning(uint8_t zone_id)
{
    if (zone_id >= IRRIGATION_ZONE_COUNT) {
        ESP_LOGE(TAG, "Invalid zone_id %d (must be 0-%d)", zone_id, IRRIGATION_ZONE_COUNT - 1);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Resetting learning data for zone %d to defaults", zone_id);

    // Clear history arrays
    memset(irrigation_zones[zone_id].learning.pulses_used_history, 0,
           sizeof(irrigation_zones[zone_id].learning.pulses_used_history));
    memset(irrigation_zones[zone_id].learning.moisture_increase_percent_history, 0,
           sizeof(irrigation_zones[zone_id].learning.moisture_increase_percent_history));
    memset(irrigation_zones[zone_id].learning.anomaly_flags, 0,
           sizeof(irrigation_zones[zone_id].learning.anomaly_flags));

    // Reset counters
    irrigation_zones[zone_id].learning.history_entry_count = 0;
    irrigation_zones[zone_id].learning.history_index = 0;
    irrigation_zones[zone_id].learning.successful_predictions = 0;
    irrigation_zones[zone_id].learning.total_predictions = 0;

    // Reset calculated values to defaults
    irrigation_zones[zone_id].learning.calculated_ppmp_ratio = DEFAULT_PULSES_PER_PERCENT;
    irrigation_zones[zone_id].learning.calculated_pump_duty_cycle = PUMP_DEFAULT_DUTY;
    irrigation_zones[zone_id].learning.target_moisture_gain_rate = TARGET_MOISTURE_GAIN_RATE;
    irrigation_zones[zone_id].learning.confidence_level = 0.0f;
    irrigation_zones[zone_id].learning.last_temperature_correction = 1.0f;

    ESP_LOGI(TAG, "Zone %d learning data reset complete", zone_id);

    xSemaphoreGive(xIrrigationMutex);
    return ESP_OK;
}

/**
 * @brief Reset learning data for all zones to defaults
 */
esp_err_t impluvium_reset_all_learning(void)
{
    ESP_LOGI(TAG, "Resetting learning data for all zones");

    esp_err_t ret = ESP_OK;
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        esp_err_t zone_ret = impluvium_reset_zone_learning(i);
        if (zone_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to reset learning for zone %d", i);
            ret = zone_ret;
        }
    }

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "All zone learning data reset successfully");
    }

    return ret;
}
