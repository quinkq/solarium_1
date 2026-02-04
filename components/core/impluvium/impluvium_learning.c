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

static const char *TAG = "IMPLUVIUM_LEARNING";

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
        ESP_LOGW(TAG, "Invalid temperature - using ""optimal interval"" as fallback");
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
            ESP_LOGD(TAG, "Nighttime - extending interval from %lus to %lus", (interval / 1000), (impluvium_night_minimum_ms/ 1000));
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
 * @brief Calculate weighted learning PPMP ratio from historical data
 *
 * Traverses circular buffer from newest to oldest entry, applying recency weighting.
 * Recent 3 cycles get 70% weight, older cycles get 30% weight.
 *
 * @param[in] learning Zone learning data
 * @param[out] valid_cycles Number of valid historical cycles found
 * @return Calculated pulses per percent ratio, or 0.0 if insufficient data
 */
float impluvium_calculate_pulse_per_moisture_percent(zone_learning_t *learning, uint8_t *valid_cycles)
{
    float weighted_ppmp_ratio = 0.0f;
    float total_weight = 0.0f;
    *valid_cycles = 0;

    // Traverse circular buffer from newest to oldest
    // age=0 is most recent, age=1 is second most recent, etc.
    for (uint8_t age = 0; age < learning->history_entry_count; age++) {

        // Calculate actual buffer index for this age
        // history_index points to next write slot, so most recent will be at (history_index - 1)
        uint8_t index = (learning->history_index - age - 1 + LEARNING_HISTORY_SIZE) % LEARNING_HISTORY_SIZE;

        // Skip anomalous cycles (rain, manual watering, etc.) and invalid ratios
        float ppmp_ratio = learning->ppmp_ratio_history[index];
        if (!learning->anomaly_flags[index] && ppmp_ratio > 0.0f) {

            // Recent 3 cycles get higher weight (70%), older ones get lower (30%)
            float weight = (age < 3) ? LEARNING_WEIGHT_RECENT : (1.0f - LEARNING_WEIGHT_RECENT);

            // Accumulate weighted sum
            weighted_ppmp_ratio += ppmp_ratio * weight;
            total_weight += weight;
            (*valid_cycles)++;

            ESP_LOGD(TAG, "History[%d] (age %d): PPMP ratio %.1f, weight %.2f",
                     index, age, ppmp_ratio, weight);
        }
    }

    // Need at least 2 valid cycles for meaningful average
    if (*valid_cycles < 2 || total_weight <= 0) {
        ESP_LOGD(TAG, "Insufficient data: %d valid cycles, %.2f total weight", *valid_cycles, total_weight);
        return 0.0f;
    }

    // Calculate final weighted average
    float calculated_ppmp = weighted_ppmp_ratio / total_weight;
    ESP_LOGD(TAG, "Weighted PPMP: %.1f (from %d cycles, weight %.2f)", calculated_ppmp, *valid_cycles, total_weight);

    return calculated_ppmp;
}

/**
 * @brief Calculate target pulses for a single zone BEFORE watering - called from measuring state.
 *
 * Uses learned data (PPMP, soil redistribution) + confidence-based blending to predict
 * water needs. Falls back to defaults for early learning phase or invalid data.
 *
 * Workflow (5 phases):
 * 1. Calculate temperature correction (±1%/°C from 20°C)
 * 2. Calculate & clamp PPMP ratio (learned pulses per moisture %)
 * 3. Apply confidence-based blending (high/medium/low confidence → learned/blend/default)
 * 4. Clamp final target pulses to safe limits
 * 5. Compute dynamic moisture cutoff (accounts for soil redistribution)
 *
 * @param[in] zone_id Zone identifier
 * @param[in] queue_index Index in watering queue
 * @return ESP_OK on success
 */
esp_err_t impluvium_precalculate_zone_watering_targets(uint8_t zone_id, uint8_t queue_index)
{
    irrigation_zone_t *zone = &irrigation_zones[zone_id];
    zone_learning_t *learning = &zone->learning;

    // ============================================================================
    // PHASE 1: Calculate Temperature Correction
    // ============================================================================
    // Calculate temperature correction
    float calculated_temp_correction = impluvium_calculate_temperature_correction(learning);

    // ============================================================================
    // PHASE 2: Calculate & Clamp PPMP Ratio
    // ============================================================================
    // Check if we have sufficient learning data
    if (learning->history_entry_count >= LEARNING_MIN_CYCLES) {
        uint8_t valid_cycles;
        float calculated_ppmp_ratio = impluvium_calculate_pulse_per_moisture_percent(learning, &valid_cycles);

        // Validate and clamp PPMP to reasonable bounds
        if (calculated_ppmp_ratio > 0.0f) {
            // Clamp to valid range and log if clamping occurred
            bool was_clamped = false;
            if (calculated_ppmp_ratio < MINIMUM_PULSES_PER_MOISTURE_PERCENT) {
                ESP_LOGW(TAG, "Zone %d: PPMP ratio %.1f below minimum, clamping to %.1f",
                         zone_id, calculated_ppmp_ratio, MINIMUM_PULSES_PER_MOISTURE_PERCENT);
                calculated_ppmp_ratio = MINIMUM_PULSES_PER_MOISTURE_PERCENT;
                was_clamped = true;
            } else if (calculated_ppmp_ratio > MAXIMUM_PULSES_PER_MOISTURE_PERCENT) {
                ESP_LOGW(TAG, "Zone %d: PPMP ratio %.1f above maximum, clamping to %.1f",
                         zone_id, calculated_ppmp_ratio, MAXIMUM_PULSES_PER_MOISTURE_PERCENT);
                calculated_ppmp_ratio = MAXIMUM_PULSES_PER_MOISTURE_PERCENT;
                was_clamped = true;
            }

            learning->calculated_ppmp_ratio = calculated_ppmp_ratio;

            // Effective deficit accounts for soil redistribution
            float effective_moisture_deficit = irrigation_system.watering_queue[queue_index].moisture_deficit_percent /
                                                learning->soil_redistribution_factor;

            // Calculate learned prediction (using effective deficit)
            float learning_adjusted_target_pulses = effective_moisture_deficit *
                                         calculated_ppmp_ratio * calculated_temp_correction;

            // Calculate default prediction for blending (using effective deficit)
            float default_target_pulses = effective_moisture_deficit *
                                        DEFAULT_PULSES_PER_MOISTURE_PERCENT * calculated_temp_correction;

            // ============================================================================
            // PHASE 3: Apply Confidence-Based Linear Blending
            // ============================================================================
            // Linear blend from 0% learned at conf=0 to 100% learned at conf=HIGH_CONFIDENCE_THRESHOLD
            float target_pulses;
            float confidence = learning->confidence_level;

            if (confidence >= HIGH_CONFIDENCE_THRESHOLD) {
                // High confidence: Use full learned prediction
                target_pulses = learning_adjusted_target_pulses;
                ESP_LOGI(TAG, "Zone %d: High confidence (%.0f%%), using 100%% learned",
                         zone_id, confidence * 100.0f);
            } else {
                // Linear interpolation: blend_factor scales from 0.0 to 1.0 as confidence rises
                float blend_factor = confidence / HIGH_CONFIDENCE_THRESHOLD;
                target_pulses = (learning_adjusted_target_pulses * blend_factor) +
                                (default_target_pulses * (1.0f - blend_factor));
                ESP_LOGI(TAG, "Zone %d: Confidence %.0f%%, blending %.0f%% learned + %.0f%% default",
                         zone_id, confidence * 100.0f, blend_factor * 100.0f, (1.0f - blend_factor) * 100.0f);
            }

            // ============================================================================
            // PHASE 4: Clamp Final Target Pulses
            // ============================================================================
            // Limit to reasonable range
            if (target_pulses < MINIMUM_TARGET_PULSES)
                target_pulses = MINIMUM_TARGET_PULSES;
            if (target_pulses > MAXIMUM_TARGET_PULSES)
                target_pulses = MAXIMUM_TARGET_PULSES;

            irrigation_system.watering_queue[queue_index].target_pulses = (uint16_t) target_pulses;

            ESP_LOGI(TAG,
                     "Zone %d: Final prediction %d pulses (PPMP: %.1f%s, temp corr: %.2f, RF: %.2f)",
                     zone_id,
                     irrigation_system.watering_queue[queue_index].target_pulses,
                     calculated_ppmp_ratio,
                     was_clamped ? " [clamped]" : "",
                     calculated_temp_correction,
                     learning->soil_redistribution_factor);
        } else {
            // PPMP is invalid (<=0) - use default
            irrigation_system.watering_queue[queue_index].target_pulses = DEFAULT_TARGET_PULSES;
            ESP_LOGW(TAG,
                     "Zone %d: Invalid PPMP ratio (%.1f), insufficient valid learning data (%d cycles), using default %d pulses",
                     zone_id,
                     calculated_ppmp_ratio,
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

    // ============================================================================
    // PHASE 5: Compute Dynamic Moisture Cutoff
    // ============================================================================
    // Compute dynamic moisture cutoff for this zone (accounts for soil redistribution)
    float start_moisture = irrigation_system.watering_queue[queue_index].measured_moisture_percent;
    float target_moisture = zone->target_moisture_percent;
    float rf = learning->soil_redistribution_factor;
    irrigation_system.watering_queue[queue_index].dynamic_moisture_cutoff =
        start_moisture + (target_moisture - start_moisture) / rf;

    ESP_LOGI(TAG,
             "Queue[%d]: Zone %d, deficit %.2f%%, target %d pulses, cutoff %.1f%% (RF=%.2f)",
             queue_index,
             zone_id,
             irrigation_system.watering_queue[queue_index].moisture_deficit_percent,
             irrigation_system.watering_queue[queue_index].target_pulses,
             irrigation_system.watering_queue[queue_index].dynamic_moisture_cutoff,
             rf);

    return ESP_OK;
}

/**
 * @brief Calculate watering predictions for all zones in queue BEFORE watering - called from measuring state.
 *
 * Orchestrates the learning algorithm to predict optimal water amounts
 * for each zone in the watering queue.
 *
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t impluvium_precalculate_zone_watering_predictions(void)
{
    ESP_LOGI(TAG, "Calculating watering predictions for %d zones", irrigation_system.watering_queue_size);

    for (uint8_t i = 0; i < irrigation_system.watering_queue_size; i++) {
        uint8_t zone_id = irrigation_system.watering_queue[i].zone_id;
        esp_err_t ret = impluvium_precalculate_zone_watering_targets(zone_id, i);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to calculate target pulses for zone %d: %s", zone_id, esp_err_to_name(ret));
            return ret;
        }
    }

    return ESP_OK;
}

/**
 * @brief Adjust pump duty cycle based on measured vs target gain rate
 *
 * Called once per zone after watering completes. Compares the measured moisture
 * gain rate against the configurable target and adjusts pump duty for the next event.
 * This provides between-event optimization without mid-watering control complexity.
 *
 * @param zone_id Zone that was watered (0-4)
 * @param measured_moisture_increase Moisture increase observed (%)
 * @param watering_duration_ms Duration of watering event (ms)
 * @param target_moisture_gain_rate Configurable target gain rate (%/sec)
 */
void impluvium_update_pump_duty_from_gain_rate(uint8_t zone_id,
                                                float measured_moisture_increase,
                                                uint32_t watering_duration_ms,
                                                float target_moisture_gain_rate)
{
    if (zone_id >= IRRIGATION_ZONE_COUNT) {
        return;
    }

    zone_learning_t *learning = &irrigation_zones[zone_id].learning;

    // Validate watering duration
    if (watering_duration_ms < 3000 || watering_duration_ms > MAX_WATERING_TIME_MS) {
        ESP_LOGD(TAG, "Zone %d: Invalid watering duration %lums for pump duty adjustment",
                 zone_id, watering_duration_ms);
        return;
    }

    // Calculate measured gain rate
    float measured_gain_rate = measured_moisture_increase / (watering_duration_ms / 1000.0f);

    // Store UNCLAMPED measured rate for telemetry/diagnostics (shows actual soil behavior)
    learning->measured_moisture_gain_rate = measured_gain_rate;

    // Clamp gain rate for calculations to prevent extreme adjustments
    float clamped_gain_rate = measured_gain_rate;
    bool was_clamped = false;

    if (clamped_gain_rate < MIN_TARGET_GAIN_RATE_PER_SEC) {
        ESP_LOGW(TAG, "Zone %d: Measured gain rate %.2f%%/sec below minimum, clamping to 0.1",
                 zone_id, measured_gain_rate);
        clamped_gain_rate = 0.1f;
        was_clamped = true;
    } else if (clamped_gain_rate > MAX_TARGET_GAIN_RATE_PER_SEC) {
        ESP_LOGW(TAG, "Zone %d: Measured gain rate %.2f%%/sec above maximum, clamping to 3.0",
                 zone_id, measured_gain_rate);
        clamped_gain_rate = 3.0f;
        was_clamped = true;
    }

    // Adjust pump duty toward target gain rate for NEXT event (using clamped value)
    float gain_ratio = target_moisture_gain_rate / clamped_gain_rate;

    // Clamp to max 15% change per event (prevents oscillation)
    if (gain_ratio > PUMP_DUTY_ADJUSTMENT_MAX_RATIO) {
        gain_ratio = PUMP_DUTY_ADJUSTMENT_MAX_RATIO;
    }
    if (gain_ratio < PUMP_DUTY_ADJUSTMENT_MIN_RATIO) {
        gain_ratio = PUMP_DUTY_ADJUSTMENT_MIN_RATIO;
    }

    uint32_t old_duty = learning->calculated_pump_duty_cycle;
    uint32_t new_duty = (uint32_t)(old_duty * gain_ratio);

    // Clamp to pump limits
    if (new_duty < PUMP_MIN_DUTY) {
        new_duty = PUMP_MIN_DUTY;
    }
    if (new_duty > PUMP_MAX_DUTY) {
        new_duty = PUMP_MAX_DUTY;
    }

    learning->calculated_pump_duty_cycle = new_duty;

    ESP_LOGI(TAG, "Zone %d pump duty adjusted: %lu → %lu (gain: %.2f%s vs target %.2f %%/sec, ratio %.2f)",
             zone_id, old_duty, new_duty, measured_gain_rate,
             was_clamped ? " [clamped]" : "", target_moisture_gain_rate, gain_ratio);
}

/**
 * @brief Update learning algorithm with POST-watering data. Called from stopping state.
 *
 * Records watering results in circular buffer, updates confidence via EMA (10% weight),
 * and optimizes pump duty for next cycle. Handles both valid and anomalous cycles.
 *
 * Workflow (4 phases):
 * 1. Store data in circular buffer (pulses, moisture increase, anomaly flag)
 * 2. Calculate basic statistics (pulses per moisture %)
 * 3. Update confidence & pump duty (valid cycles only, EMA-based)
 * 4. Log learning progress (after minimum cycles reached)
 *
 * Anomaly Detection Criteria (set via impluvium_set_anomaly):
 * - Excessive moisture increase rate (>5%/sec indicates rain/manual watering)
 * - Extreme temperature conditions (<5°C or >45°C)
 * - Flow rate below minimum threshold (<15 L/h)
 * - Invalid temperature reading from TEMPESTA
 *
 * @param zone_id Zone that was watered (0-4)
 * @param pulses_used Number of flow sensor pulses during watering
 * @param measured_moisture_increase Percentage increase observed in moisture sensor (final - initial)
 * @param watering_duration_ms Duration of watering (for gain rate calculation)
 * @param learning_valid Whether this cycle should be used for learning
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid parameters
 */
esp_err_t impluvium_postprocess_zone_watering_data(uint8_t zone_id,
                                                   uint32_t pulses_used,
                                                   float measured_moisture_increase,
                                                   uint32_t watering_duration_ms,
                                                   bool learning_valid)
{
    // ============================================================================
    // PHASE 1: Store Data in Circular Buffer
    // ============================================================================
    if (zone_id >= IRRIGATION_ZONE_COUNT) {
        ESP_LOGE(TAG, "Invalid zone_id %d for learning update", zone_id);
        return ESP_ERR_INVALID_ARG;
    }

    irrigation_zone_t *zone = &irrigation_zones[zone_id];
    zone_learning_t *learning = &zone->learning;

    // Calculate PPMP ratio for this cycle (pulses per 1% moisture increase)
    float measured_ppmp_ratio = (measured_moisture_increase > 0.0f)
                                    ? ((float)pulses_used / measured_moisture_increase)
                                    : 0.0f;

    // Get current write position in circular buffer
    uint8_t index = learning->history_index;

    // Store pre-computed PPMP ratio and anomaly flag
    learning->ppmp_ratio_history[index] = measured_ppmp_ratio;
    learning->anomaly_flags[index] = !learning_valid; // valid learning = NOT anomaly

    // Advance circular buffer index
    learning->history_index = (learning->history_index + 1) % LEARNING_HISTORY_SIZE;

    // Update count (saturates at LEARNING_HISTORY_SIZE)
    if (learning->history_entry_count < LEARNING_HISTORY_SIZE) {
        learning->history_entry_count++;
    }

    ESP_LOGI(TAG,
             "Zone %d learning update: %lu pulses → %.2f%% increase (%.1f pulses/%%), %s",
             zone_id,
             pulses_used,
             measured_moisture_increase,
             measured_ppmp_ratio,
             learning_valid ? "valid" : "anomaly");

    // ============================================================================
    // PHASE 3: Update Confidence & Pump Duty (Valid Cycles Only)
    // ============================================================================
    // ### Confidence tracking calculations (only for valid predictions)
    if (learning_valid) {
        float prediction_quality = 0.0f;

        // Calculate prediction quality with smooth gradient scoring
        if (learning->calculated_ppmp_ratio > 0) {
            float expected_moisture_increase = pulses_used / learning->calculated_ppmp_ratio;
            float relative_error = fabs(measured_moisture_increase - expected_moisture_increase) / expected_moisture_increase;

            // Smooth gradient: 1.0 at 0% error, linearly decreasing to 0.0 at 30% error
            if (relative_error <= 0.30f) {
                prediction_quality = 1.0f - (relative_error / 0.30f);
            }
            // Errors > 30% contribute 0.0 quality
        }

        // Update confidence using Exponential Moving Average
        float ema_retain = 1.0f - CONFIDENCE_EMA_WEIGHT;
        learning->confidence_level = (learning->confidence_level * ema_retain) + (prediction_quality * CONFIDENCE_EMA_WEIGHT);

        // Confidence boost: When prediction is accurate but confidence is still low, boost to floor
        // This accelerates convergence after PPMP is learned correctly
        if (learning->history_entry_count >= LEARNING_MIN_CYCLES &&
            prediction_quality >= 0.50f &&
            learning->confidence_level < CONFIDENCE_BOOST_FLOOR) {
            ESP_LOGI(TAG, "Zone %d: Accurate prediction with low confidence, boosting %.0f%% → %.0f%%",
                     zone_id, learning->confidence_level * 100.0f, CONFIDENCE_BOOST_FLOOR * 100.0f);
            learning->confidence_level = CONFIDENCE_BOOST_FLOOR;
        }

        // Update legacy counters for telemetry/logging
        learning->total_predictions++;
        if (prediction_quality >= 0.33f) { // Equivalent to ≤20% error threshold
            learning->successful_predictions++;
        }

        // Between-event pump duty optimization (only for valid cycles)
        impluvium_update_pump_duty_from_gain_rate(zone_id,
                                                   measured_moisture_increase,
                                                   watering_duration_ms,
                                                   irrigation_zones[zone_id].target_moisture_gain_rate);
    }

    // ============================================================================
    // PHASE 4: Log Learning Progress
    // ============================================================================
    // Update learned parameters and log progress (only after sufficient learning cycles)
    if (learning->history_entry_count >= LEARNING_MIN_CYCLES) {
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
 * @brief Update soil redistribution factors from delayed moisture readings
 *
 * Called 5 minutes after a watering session completes. Powers on sensors, reads
 * delayed moisture for each zone that was watered, and updates soil redistribution
 * factors based on immediate vs delayed moisture readings.
 *
 * Batched operation - checks all watered zones in one sensor power-on cycle to
 * minimize power consumption.
 *
 * Physics: Water initially concentrates at application depth, then redistributes
 * upward/downward over time. The redistribution factor (RF = delayed/immediate)
 * quantifies how much moisture movement occurs, allowing better target predictions.
 *
 * @return ESP_OK on success (cleanup always happens even if reads fail)
 */
esp_err_t impluvium_update_redistribution_from_delayed_readings(void)
{
    ESP_LOGI(TAG, "Starting delayed verification for %d zones", irrigation_system.verification_zone_count);

    // Power on sensor bus for moisture readings
    if (impluvium_request_power_buses(POWER_ONLY_SENSORS, "IMPLUVIUM_VERIFY") != ESP_OK) {
        ESP_LOGE(TAG, "Failed to power on sensors for verification - skipping");
        goto cleanup;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));  // Allow sensors to stabilize

    // Check each zone that was watered
    for (uint8_t i = 0; i < irrigation_system.verification_zone_count; i++) {
        uint8_t zone_id = irrigation_system.verification_zones[i].zone_id;
        float moisture_start = irrigation_system.verification_zones[i].moisture_at_start_percent;
        float moisture_immediate = irrigation_system.verification_zones[i].moisture_immediate_percent;

        // Read delayed moisture level
        float delayed_moisture;
        if (impluvium_read_moisture_sensor(zone_id, &delayed_moisture) != ESP_OK) {
            ESP_LOGW(TAG, "Zone %d: Failed to read delayed moisture - skipping verification", zone_id);
            continue;
        }

        // Calculate immediate and delayed increases
        float immediate_increase = moisture_immediate - moisture_start;
        float delayed_increase = delayed_moisture - moisture_start;

        // Validate: delayed should be >= immediate, and immediate should be meaningful
        if (delayed_increase > immediate_increase && immediate_increase > 0.5f) {
            float measured_factor = delayed_increase / immediate_increase;

            // Clamp to valid range
            if (measured_factor < MIN_SOIL_REDISTRIBUTION_FACTOR) {
                measured_factor = MIN_SOIL_REDISTRIBUTION_FACTOR;
            }
            if (measured_factor > MAX_SOIL_REDISTRIBUTION_FACTOR) {
                measured_factor = MAX_SOIL_REDISTRIBUTION_FACTOR;
            }

            // Update redistribution factor using EMA (15% weight to new measurement)
            zone_learning_t *learning = &irrigation_zones[zone_id].learning;
            float old_factor = learning->soil_redistribution_factor;
            learning->soil_redistribution_factor =
                (learning->soil_redistribution_factor * (1.0f - SOIL_REDISTRIBUTION_EMA_WEIGHT)) +
                (measured_factor * SOIL_REDISTRIBUTION_EMA_WEIGHT);

            ESP_LOGI(TAG, "Zone %d RF updated: %.2f → %.2f (measured: %.2f, immediate: +%.1f%%, delayed: +%.1f%%)",
                     zone_id, old_factor, learning->soil_redistribution_factor, measured_factor,
                     immediate_increase, delayed_increase);
        } else {
            ESP_LOGW(TAG, "Zone %d: Invalid redistribution data - immediate: +%.1f%%, delayed: +%.1f%% (skipping)",
                     zone_id, immediate_increase, delayed_increase);
        }
    }

    // Power off sensor bus
    impluvium_release_power_buses(POWER_ONLY_SENSORS, "IMPLUVIUM_VERIFY");

cleanup:
    // Clear verification state
    irrigation_system.verification_pending = false;
    irrigation_system.verification_zone_count = 0;
    memset(irrigation_system.verification_zones, 0, sizeof(irrigation_system.verification_zones));

    ESP_LOGI(TAG, "Delayed verification complete");
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
    memset(irrigation_zones[zone_id].learning.ppmp_ratio_history, 0,
           sizeof(irrigation_zones[zone_id].learning.ppmp_ratio_history));
    memset(irrigation_zones[zone_id].learning.anomaly_flags, 0,
           sizeof(irrigation_zones[zone_id].learning.anomaly_flags));

    // Reset counters
    irrigation_zones[zone_id].learning.history_entry_count = 0;
    irrigation_zones[zone_id].learning.history_index = 0;
    irrigation_zones[zone_id].learning.successful_predictions = 0;
    irrigation_zones[zone_id].learning.total_predictions = 0;

    // Reset calculated values to defaults
    irrigation_zones[zone_id].learning.calculated_ppmp_ratio = DEFAULT_PULSES_PER_MOISTURE_PERCENT;
    irrigation_zones[zone_id].learning.calculated_pump_duty_cycle = PUMP_DEFAULT_DUTY;
    irrigation_zones[zone_id].learning.soil_redistribution_factor = DEFAULT_SOIL_REDISTRIBUTION_FACTOR;
    irrigation_zones[zone_id].learning.measured_moisture_gain_rate = 0.0f; // No measurement yet
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
