/**
 * @file impluvium_safety.c
 * @brief Safety monitoring and emergency diagnostics for IMPLUVIUM irrigation system
 *
 * Implements pre-check validation, real-time safety monitoring during watering,
 * emergency stop handling, and automated diagnostic testing for fault isolation.
 */

#include "impluvium.h"
#include "impluvium_private.h"
#include "fluctus.h"
#include "tempesta.h"
#include "telemetry.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include <string.h>

static const char *TAG = "IMPLUVIUM_SAFETY";

/**
 * @brief Perform one-time safety checks before watering
 *
 * Validates:
 * - Temperature within operating range
 * - Water level sufficient
 * - System pressure within limits
 *
 * @return ESP_OK if all checks pass, ESP_FAIL otherwise
 */
esp_err_t impluvium_pre_check(void)
{
    // Check temperature
    float current_temperature = tempesta_get_temperature();

    if (current_temperature == WEATHER_INVALID_VALUE) {
        ESP_LOGE(TAG, "Pre-check failed: Temperature sensors unavailable");
        return ESP_FAIL;
    }

    // Global temperature safety limits (wider range for system protection)
    if (current_temperature < MIN_TEMPERATURE_GLOBAL || current_temperature > MAX_TEMPERATURE_GLOBAL) {
        ESP_LOGE(TAG,
                 "Pre-check failed: Temperature %.1f°C outside global safety range (%.1f°C to %.1f°C)",
                 current_temperature,
                 MIN_TEMPERATURE_GLOBAL,
                 MAX_TEMPERATURE_GLOBAL);
        impluvium_emergency_stop("Temperature outside global safety range");
        return ESP_FAIL;
    }

    // Watering-specific temperature limits (narrower range for optimal operation)
    if (current_temperature < MIN_TEMPERATURE_WATERING) {
        ESP_LOGW(TAG,
                 "Pre-check failed: Temperature %.1f°C is below watering minimum %.1f°C",
                 current_temperature,
                 MIN_TEMPERATURE_WATERING);
        return ESP_FAIL;
    }

    // Read current water level in tank
#ifdef CONFIG_IMPLUVIUM_DRY_TEST_MODE
    // Dry test mode: Return fixed safe water level (50%)
    irrigation_system.water_level = 50.0f;
    ESP_LOGI(TAG, "DRY_TEST_MODE: Using fixed water level 50.0%%");
#else
    impluvium_read_water_level(&irrigation_system.water_level);
    if (irrigation_system.water_level < MIN_WATER_LEVEL_PERCENT) {
        ESP_LOGW(TAG,
                 "Pre-check failed: Water level %.1f%% is below minimum %.1f%%",
                 irrigation_system.water_level,
                 MIN_WATER_LEVEL_PERCENT);
        return ESP_FAIL;
    }
#endif

    // Read current pressure
#ifdef CONFIG_IMPLUVIUM_DRY_TEST_MODE
    // Dry test mode: Return fixed safe pressure (0.5 bar)
    irrigation_system.outlet_pressure = 0.5f;
    ESP_LOGI(TAG, "DRY_TEST_MODE: Using fixed outlet pressure 0.5 bar");
#else
    if (impluvium_read_pressure(&irrigation_system.outlet_pressure) != ESP_OK) {
        ESP_LOGW(TAG, "Pre-check failed: Could not read system's outlet pressure.");
        return ESP_FAIL;
    }

    if (irrigation_system.outlet_pressure > MAX_PRESSURE_BAR) {
        ESP_LOGE(TAG,
                 "Pre-check failed: Outlet pressure %.2f bar is above maximum %.2f bar",
                 irrigation_system.outlet_pressure,
                 MAX_PRESSURE_BAR);
        impluvium_emergency_stop("System's outlet pressure too high");
        return ESP_FAIL;
    }
#endif

    ESP_LOGI(TAG,
             "Safety pre-check complete: temp %.1f°C, water level %.1f %%, outlet pressure %.2f bar, allowed YES",
             current_temperature,
             irrigation_system.water_level,
             irrigation_system.outlet_pressure);

    return ESP_OK; // All checks passed
}

/**
 * @brief Update flow rate calculation
 */
void impluvium_calc_flow_rate(const char *task_tag,
                              uint32_t *last_pulse_count,
                              uint32_t *last_flow_time,
                              uint32_t current_time)
{
    int current_pulse_count;
    if (pcnt_unit_get_count(flow_pcnt_unit, &current_pulse_count) == ESP_OK) {
        if (*last_flow_time > 0 && current_time - *last_flow_time >= FLOW_RATE_CALC_PERIOD_MS) {
            uint32_t pulse_diff = current_pulse_count - *last_pulse_count;
            uint32_t time_diff_ms = current_time - *last_flow_time;

            // Calculate Liters per Second, then convert to Liters per Hour
            float flow_rate_lps = (pulse_diff / FLOW_CALIBRATION_PULSES_PER_LITER) / (time_diff_ms / 1000.0f);
            irrigation_system.current_flow_rate = flow_rate_lps * 3600.0f; // Convert L/s to L/h

            ESP_LOGD(task_tag,
                     "Flow rate updated: %lu pulses in %lums = %.1f L/h",
                     pulse_diff,
                     time_diff_ms,
                     irrigation_system.current_flow_rate);

            *last_pulse_count = current_pulse_count;
            *last_flow_time = current_time;
        }
    }
}

/**
 * @brief Perform smart watering cutoffs based on real-time conditions
 * @return ESP_OK if cutoff triggered, ESP_FAIL to continue watering
 */
esp_err_t impluvium_watering_cutoffs_check(const char *task_tag, uint32_t time_since_start_ms)
{
    if (irrigation_system.active_zone >= IRRIGATION_ZONE_COUNT ||
        irrigation_system.queue_index >= irrigation_system.watering_queue_size) {
        return ESP_FAIL;
    }

    irrigation_zone_t *active_zone = &irrigation_zones[irrigation_system.active_zone];
    watering_queue_item_t *queue_item = &irrigation_system.watering_queue[irrigation_system.queue_index];

    // **MANUAL WATERING MODE**: Use time-based cutoff only, skip moisture checks
    if (irrigation_system.manual_watering_active) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Check if manual watering duration has elapsed
        if (current_time >= irrigation_system.manual_water_end_time) {
            ESP_LOGI(task_tag,
                     "Manual watering time completed for zone %d (%d seconds)",
                     irrigation_system.active_zone,
                     irrigation_system.manual_water_duration_sec);
            xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_WATERING_CUTOFF, eSetBits);
            return ESP_OK;
        }

        // Continue manual watering (pressure safety checks still performed in periodic_safety_check)
        return ESP_FAIL;
    }

    // Check maximum watering time hasn't been exceeded
    if (time_since_start_ms > MAX_WATERING_TIME_MS) {
        ESP_LOGW(TAG, "Zone %d max watering time reached - stopping", irrigation_system.active_zone);
        xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_WATERING_CUTOFF, eSetBits);
        return ESP_OK;
    }

    // Check if target pulses reached
    int pulse_count;
    if (pcnt_unit_get_count(flow_pcnt_unit, &pulse_count) == ESP_OK) {
        uint32_t pulses_used = pulse_count - irrigation_system.watering_start_pulses;

        if (pulses_used >= queue_item->target_pulses) {
            ESP_LOGI(task_tag,
                     "Zone %d target pulses reached: %lu >= %d",
                     irrigation_system.active_zone,
                     pulses_used,
                     queue_item->target_pulses);
            xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_WATERING_CUTOFF, eSetBits);
            return ESP_OK;
        }
    }

    // Check moisture level and safety margin
    float measured_moisture_percent;
    if (impluvium_read_moisture_sensor(irrigation_system.active_zone, &measured_moisture_percent) == ESP_OK) {
        irrigation_zones[irrigation_system.active_zone].last_moisture_percent =
            measured_moisture_percent; // Store last reading
        // Calculate, store, and check the current moisture gain rate
        float moisture_increase = measured_moisture_percent - queue_item->moisture_at_start_percent;

        if (time_since_start_ms > 1000) { // Calculate after 1 second to get a meaningful rate
            irrigation_system.current_moisture_gain_rate = (moisture_increase / (time_since_start_ms / 1000.0f));

            // Check for anomalous spike after 2 seconds
            if (time_since_start_ms > 2000 &&
                irrigation_system.current_moisture_gain_rate > MOISTURE_SPIKE_RATE_THRESHOLD_PER_SEC) {
                ESP_LOGW(task_tag,
                         "Zone %d moisture spike detected: %.2f%%/sec increase - possible rain/manual watering",
                         irrigation_system.active_zone,
                         irrigation_system.current_moisture_gain_rate);
                irrigation_system.current_anomaly.type = ANOMALY_MOISTURE_SPIKE;
                irrigation_system.current_anomaly.anomaly_timestamp = time(NULL);
                irrigation_system.current_anomaly.expected_vs_actual = irrigation_system.current_moisture_gain_rate;
                xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_WATERING_CUTOFF, eSetBits);
                return ESP_OK; // Stop watering
            }
        } else {
            irrigation_system.current_moisture_gain_rate = 0.0f;
        }

        // Check if moisture reached target - with safety margin (target - 2%)
        float safety_target = active_zone->target_moisture_percent - 2.0f;
        if (measured_moisture_percent >= safety_target) {
            ESP_LOGI(task_tag,
                     "Zone %d safety margin reached: %.1f%% >= %.1f%%",
                     irrigation_system.active_zone,
                     measured_moisture_percent,
                     safety_target);
            xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_WATERING_CUTOFF, eSetBits);
            return ESP_OK;
        }

        // Store for anomaly detection
        irrigation_system.last_moisture_reading_percent = measured_moisture_percent;
    }

    return ESP_FAIL; // Continue watering
}

/**
 * @brief Perform comprehensive safety monitoring while watering
 * @return true if safe to continue, false if emergency stop was triggered
 */
bool impluvium_periodic_safety_check(uint32_t *error_count, uint32_t time_since_start_ms)
{
    // Allow 4 consecutive errors before emergency stop (immediate if current exceeds limits set in FLUCTUS)
    const uint32_t MAX_ERROR_COUNT = 4;
    const char *failure_reason = NULL;

    // Read outlet pressure and level sensors
#ifdef CONFIG_IMPLUVIUM_DRY_TEST_MODE
    // Dry test mode: Use fixed safe values instead of reading sensors
    irrigation_system.outlet_pressure = 0.5f;
    irrigation_system.water_level = 50.0f;
    ESP_LOGD(TAG, "DRY_TEST_MODE: Using fixed values (pressure: 0.5 bar, water level: 50%%)");
#else
    esp_err_t ret = impluvium_read_pressure(&irrigation_system.outlet_pressure);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Safety check: Failed to read outlet pressure sensor: %s", esp_err_to_name(ret));
        failure_reason = "Outlet pressure sensor read failed";
    }
    ret = impluvium_read_water_level(&irrigation_system.water_level);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Safety check: Failed to read water level: %s", esp_err_to_name(ret));
        failure_reason = "Water level sensor read failed";
    }
#endif

    // Overcurrent monitoring delegated to Fluctus (power management) system.

    // Check safety parameters
    if (failure_reason == NULL) {
#ifdef CONFIG_IMPLUVIUM_DRY_TEST_MODE
        // Dry test mode: Bypass pressure and water level checks (use fixed safe values)
        // Only check flow rate if we want to allow it to fail (for testing), but user wants to bypass it
        // So we skip flow rate check entirely in dry test mode
        ESP_LOGD(TAG, "DRY_TEST_MODE: Bypassing pressure, water level, and flow rate safety checks");
#else
        if (irrigation_system.outlet_pressure > MAX_PRESSURE_BAR) {
            ESP_LOGW(TAG,
                     "Outlet pressure too high: %.2f > %.2f bar",
                     irrigation_system.outlet_pressure,
                     MAX_PRESSURE_BAR);
            failure_reason = "Outlet pressure too high";
        }
        if (time_since_start_ms > 1000 && irrigation_system.current_flow_rate < MIN_FLOW_RATE_LH) {
            ESP_LOGW(TAG, "Flow rate too low: %.1f < %.1f L/h", irrigation_system.current_flow_rate, MIN_FLOW_RATE_LH);
            failure_reason = "Flow rate too low.";
        }
        if (irrigation_system.water_level < MIN_WATER_LEVEL_PERCENT - 3.0f) { // Stop watering below 2%
            ESP_LOGW(TAG, "Water level too low, currently at: %.1f %%", irrigation_system.water_level);
            failure_reason = "Water level critically low";
        }
#endif
    }

    // Handle safety failures
    if (failure_reason != NULL) {
        // Increment error count
        (*error_count)++;

        ESP_LOGW(TAG,
                 "Safety failure %" PRIu32 "/%" PRIu32 ": %s (Outlet pressure: %.2f, Flow: %.1f)",
                 *error_count,
                 (uint32_t) MAX_ERROR_COUNT,
                 failure_reason,
                 irrigation_system.outlet_pressure,
                 irrigation_system.current_flow_rate);

        if (*error_count >= MAX_ERROR_COUNT) {
            ESP_LOGE(TAG, "Maximum safety failures reached for: %s", failure_reason);
            impluvium_emergency_stop(failure_reason);
            return false; // Emergency triggered
        } else if (*error_count == 1) {
            ESP_LOGW(TAG, "Reducing pump speed due to safety issue");
            impluvium_set_pump_speed(PUMP_MIN_DUTY);
        }
    }

    return true; // Safe to continue
}

// ########################## Irrigation System ##########################
// ------------------- Emergency Diagnostics Functions -------------------

/**
 * @brief Emergency stop - immediate shutdown
 */
esp_err_t impluvium_emergency_stop(const char *reason)
{
    ESP_LOGE(TAG, "EMERGENCY STOP TRIGGERED: %s - Immediate irrigation shutdown", reason);

    // Only trigger if not already in an emergency shutdown
    if (!irrigation_system.emergency_stop) {
        irrigation_system.emergency_stop = true;
        // Store the reason for the diagnostics phase
        irrigation_system.emergency.failure_reason = reason;
        // Signal the main task to handle the emergency
        xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_EMERGENCY_SHUTDOWN, eSetBits);
    }

    return ESP_OK;
}

/**
 * @brief Initialize emergency diagnostics system
 */
esp_err_t emergency_diagnostics_init(void)
{
    // memset is redundant - static variables are zero-initialized
    irrigation_system.emergency.state = EMERGENCY_NONE;

    ESP_LOGI(TAG, "Emergency diagnostics system initialized");
    return ESP_OK;
}

/**
 * @brief Start emergency diagnostics with given reason
 */
esp_err_t emergency_diagnostics_start(const char *reason)
{
    ESP_LOGW(TAG, "=== EMERGENCY DIAGNOSTICS INITIATED ===");
    ESP_LOGW(TAG, "Reason: %s", reason);

    // Power on sensors for the duration of the diagnostics
    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_3V3, "IMPLUVIUM_DIAG", return ESP_FAIL);

    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_5V, "IMPLUVIUM_DIAG", {
        fluctus_release_bus_power(POWER_BUS_3V3, "IMPLUVIUM_DIAG");
        return ESP_FAIL;
    });

    // Reset diagnostics state
    memset(&irrigation_system.emergency, 0, sizeof(emergency_diagnostics_t));
    irrigation_system.emergency.state = EMERGENCY_TRIGGERED;
    irrigation_system.emergency.diagnostic_start_time_ms = esp_timer_get_time() / 1000; // Monotonic time
    irrigation_system.emergency.failure_reason = reason;

    // Transition to maintenance state to handle diagnostics
    impluvium_change_state(IMPLUVIUM_MAINTENANCE);

    ESP_LOGI(TAG, "Emergency diagnostics scheduled - switching to MAINTENANCE state");
    return ESP_OK;
}

/**
 * @brief Check moisture levels before starting diagnostics
 */
esp_err_t emergency_diagnostics_check_moisture_levels(void)
{
    ESP_LOGI(TAG, "Checking moisture levels before diagnostics...");

    irrigation_system.emergency.eligible_zones_count = 0;
    irrigation_system.emergency.eligible_zones_mask = 0;

    for (uint8_t zone_id = 0; zone_id < IRRIGATION_ZONE_COUNT; zone_id++) {
        if (!irrigation_zones[zone_id].watering_enabled)
            continue;

        float moisture_percent;
        if (impluvium_read_moisture_sensor(zone_id, &moisture_percent) == ESP_OK) {
            irrigation_zones[zone_id].last_moisture_percent = moisture_percent; // Store last reading
            ESP_LOGI(TAG, "Zone %d moisture: %.1f%%", zone_id, moisture_percent);

            if (moisture_percent < EMERGENCY_MOISTURE_THRESHOLD) {
                ESP_LOGI(TAG,
                         "Zone %d is eligible for testing (%.1f%% < %.1f%%)",
                         zone_id,
                         moisture_percent,
                         EMERGENCY_MOISTURE_THRESHOLD);
                irrigation_system.emergency.eligible_zones_count++;
                irrigation_system.emergency.eligible_zones_mask |= (1U << zone_id);
            }
        } else {
            ESP_LOGE(TAG, "Failed to read moisture for zone %d", zone_id);
        }
    }

    if (irrigation_system.emergency.eligible_zones_count >= 2) {
        irrigation_system.emergency.initial_moisture_check_passed = true;

        ESP_LOGI(TAG,
                 "Moisture levels sufficient for diagnostics (%d eligible zones) - proceeding",
                 irrigation_system.emergency.eligible_zones_count);
        irrigation_system.emergency.state = EMERGENCY_DIAGNOSING;
        irrigation_system.emergency.test_zone = 0; // Start search from zone 0
        irrigation_system.emergency.test_cycle_count = 0;
    } else {
        ESP_LOGE(TAG,
                 "Not enough eligible zones (%d < 2) for diagnostics - manual intervention required",
                 irrigation_system.emergency.eligible_zones_count);
        irrigation_system.emergency.state = EMERGENCY_USER_REQUIRED;
        irrigation_system.emergency.failure_reason = "Moisture levels too high for diagnostic testing";
    }

    return ESP_OK;
}

/**
 * @brief Test a specific zone with short watering cycle
 */
esp_err_t emergency_diagnostics_test_zone(uint8_t zone_id)
{
    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_12V, "IMPLUVIUM_DIAG", return ESP_FAIL);

    if (zone_id >= IRRIGATION_ZONE_COUNT) {
        ESP_LOGE(TAG, "Invalid zone for diagnostics: %d", zone_id);
        return ESP_ERR_INVALID_ARG;
    }

    irrigation_zone_t *zone = &irrigation_zones[zone_id];

    ESP_LOGI(TAG,
             "Testing zone %d (diagnostic cycle %d)...",
             zone_id,
             irrigation_system.emergency.test_cycle_count + 1);

    // Open valve for this zone
    gpio_set_level(zone->valve_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(VALVE_OPEN_DELAY_MS)); // Wait for valve to open

    // Start pump at reduced speed for safety
    esp_err_t ret = impluvium_set_pump_speed(PUMP_MIN_DUTY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start pump for zone %d test", zone_id);
        gpio_set_level(zone->valve_gpio, 0);
        irrigation_system.emergency.failed_zones_mask |= (1U << zone_id);
        irrigation_system.emergency.test_flow_rates[zone_id] = 0.0f;
        irrigation_system.emergency.test_pressures[zone_id] = 0.0f;
        return ESP_FAIL;
    }

    // Record test start time
    irrigation_system.emergency.test_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Reset flow counter
    pcnt_unit_clear_count(flow_pcnt_unit);

    // Wait for test duration, checking for overcurrent midway
    vTaskDelay(pdMS_TO_TICKS(EMERGENCY_TEST_DURATION_MS / 5));

    // Overcurrent monitoring delegated to Fluctus (power management) system.

    vTaskDelay(pdMS_TO_TICKS(EMERGENCY_TEST_DURATION_MS));

    // Measure results
    int test_pulses;
    pcnt_unit_get_count(flow_pcnt_unit, &test_pulses);
    float test_volume_ml = (test_pulses / FLOW_CALIBRATION_PULSES_PER_LITER) * 1000.0f;
    float test_flow_rate = (test_volume_ml / 1000.0f) * (3600.0f / (EMERGENCY_TEST_DURATION_MS / 1000.0f)); // L/h

    // Read outlet pressure
    float test_pressure;
    if (impluvium_read_pressure(&test_pressure) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read outlet pressure during diagnostic test for zone %d", zone_id);
        test_pressure = -1.0f; // Indicate failure
    }

    // Stop pump and close valve
    impluvium_set_pump_speed(0);
    vTaskDelay(pdMS_TO_TICKS(PRESSURE_EQUALIZE_DELAY_MS)); // Wait for outlet pressure to drop
    gpio_set_level(zone->valve_gpio, 0);
    fluctus_release_bus_power(POWER_BUS_12V, "IMPLUVIUM_DIAG");

    // Store test results
    irrigation_system.emergency.test_flow_rates[zone_id] = test_flow_rate;
    irrigation_system.emergency.test_pressures[zone_id] = test_pressure;

    // Analyze test results
    bool test_passed = true;
    const char *failure_reason = "";

    if (test_flow_rate < EMERGENCY_TEST_MIN_FLOW_RATE) {
        test_passed = false;
        failure_reason = "Low flow rate";
        ESP_LOGW(TAG,
                 "Zone %d FAILED: Low flow rate %.1f L/h < %.1f L/h",
                 zone_id,
                 test_flow_rate,
                 EMERGENCY_TEST_MIN_FLOW_RATE);
    }

    if (test_pressure > EMERGENCY_TEST_MAX_PRESSURE) {
        test_passed = false;
        failure_reason = "High outlet pressure";
        ESP_LOGW(TAG,
                 "Zone %d FAILED: High outlet pressure %.2f bar > %.2f bar",
                 zone_id,
                 test_pressure,
                 EMERGENCY_TEST_MAX_PRESSURE);
    }

    if (test_passed) {
        ESP_LOGI(TAG,
                 "Zone %d PASSED: Flow %.1f L/h, Outlet pressure %.2f bar, Volume %.1f mL",
                 zone_id,
                 test_flow_rate,
                 test_pressure,
                 test_volume_ml);
    } else {
        ESP_LOGE(TAG,
                 "Zone %d FAILED: %s (Flow %.1f L/h, Outlet pressure %.2f bar)",
                 zone_id,
                 failure_reason,
                 test_flow_rate,
                 test_pressure);
        irrigation_system.emergency.failed_zones_mask |= (1U << zone_id);
    }

    // Move to next zone
    irrigation_system.emergency.test_cycle_count++;

    // Small delay between tests
    vTaskDelay(pdMS_TO_TICKS(2000));

    return test_passed ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Analyze diagnostic test results and determine next action
 */
esp_err_t emergency_diagnostics_analyze_results(void)
{
    ESP_LOGI(TAG, "Analyzing emergency diagnostic results...");

    uint8_t failed_zones = 0;

    // Count failures among eligible zones
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        // Only consider zones that were eligible for testing
        if (irrigation_system.emergency.eligible_zones_mask & (1U << i)) {
            if (irrigation_system.emergency.failed_zones_mask & (1U << i)) {
                failed_zones++;
                ESP_LOGW(TAG,
                         "Eligible Zone %d FAILED: Flow %.1f L/h, Outlet pressure %.2f bar",
                         i,
                         irrigation_system.emergency.test_flow_rates[i],
                         irrigation_system.emergency.test_pressures[i]);
            } else {
                ESP_LOGI(TAG,
                         "Eligible Zone %d PASSED: Flow %.1f L/h, Outlet pressure %.2f bar",
                         i,
                         irrigation_system.emergency.test_flow_rates[i],
                         irrigation_system.emergency.test_pressures[i]);
            }
        }
    }

    ESP_LOGI(TAG,
             "Diagnostic summary: %d/%d eligible zones failed",
             failed_zones,
             irrigation_system.emergency.eligible_zones_count);

    // Determine course of action
    if (failed_zones == 0) {
        // All eligible zones passed - system appears healthy
        ESP_LOGI(TAG, "All eligible zones passed diagnostics - system appears healthy");
        irrigation_system.emergency.state = EMERGENCY_RESOLVED;
        irrigation_system.emergency.failure_reason = "Auto-recovered: All diagnostic tests passed";

    } else if (failed_zones == irrigation_system.emergency.eligible_zones_count) {
        // All eligible zones failed - system-wide issue
        ESP_LOGE(TAG, "All eligible zones failed - system-wide issue detected");
        irrigation_system.emergency.consecutive_failures++;

        if (irrigation_system.emergency.consecutive_failures >= EMERGENCY_MAX_CONSECUTIVE_FAILS) {
            irrigation_system.emergency.state = EMERGENCY_USER_REQUIRED;
            irrigation_system.emergency.failure_reason = "System-wide failure: pump, filter, or supply issue";
        } else {
            ESP_LOGW(TAG,
                     "Retrying diagnostics (attempt %d/%d)",
                     irrigation_system.emergency.consecutive_failures,
                     EMERGENCY_MAX_CONSECUTIVE_FAILS);
            // Reset for another diagnostic cycle
            irrigation_system.emergency.state = EMERGENCY_TRIGGERED;
            irrigation_system.emergency.test_zone = 0;
            irrigation_system.emergency.failed_zones_mask = 0;
        }

    } else {
        // Some zones failed - zone-specific issues
        ESP_LOGW(TAG, "Partial failure - %d zones have issues", failed_zones);

        if (failed_zones <= (irrigation_system.emergency.eligible_zones_count / 2)) {
            // Less than half failed - disable failed zones and continue
            ESP_LOGI(TAG, "Disabling failed zones and continuing operation");
            for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
                if (irrigation_system.emergency.failed_zones_mask & (1U << i)) {
                    irrigation_zones[i].watering_enabled = false;
                    ESP_LOGW(TAG, "Zone %d disabled due to diagnostic failure", i);
                }
            }
            irrigation_system.emergency.state = EMERGENCY_RESOLVED;
            irrigation_system.emergency.failure_reason = "Partial recovery: some zones disabled";
        } else {
            // More than half failed - requires manual intervention
            irrigation_system.emergency.state = EMERGENCY_USER_REQUIRED;
            irrigation_system.emergency.failure_reason = "Too many zone failures for automatic recovery";
        }
    }

    return ESP_OK;
}

/**
 * @brief Resolve emergency diagnostics and return to normal operation
 */
esp_err_t emergency_diagnostics_resolve(void)
{
    int64_t diagnostic_duration_ms =
        (esp_timer_get_time() / 1000) - irrigation_system.emergency.diagnostic_start_time_ms;
    int64_t diagnostic_duration_seconds = diagnostic_duration_ms / 1000;

    ESP_LOGW(TAG, "=== EMERGENCY DIAGNOSTICS COMPLETED ===");
    ESP_LOGI(TAG, "Resolution: %s", irrigation_system.emergency.failure_reason);
    ESP_LOGI(TAG, "Duration: %lld seconds", (long long) diagnostic_duration_seconds);
    ESP_LOGI(TAG, "Failed zones mask: 0x%02X", irrigation_system.emergency.failed_zones_mask);

    // Log detailed results for maintenance records
    ESP_LOGI(TAG, "Diagnostic test results:");
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        if (irrigation_zones[i].watering_enabled || (irrigation_system.emergency.failed_zones_mask & (1U << i))) {
            ESP_LOGI(TAG,
                     "  Zone %d: Flow %.1f L/h, Outlet pressure %.2f bar, %s",
                     i,
                     irrigation_system.emergency.test_flow_rates[i],
                     irrigation_system.emergency.test_pressures[i],
                     (irrigation_system.emergency.failed_zones_mask & (1U << i)) ? "FAILED" : "PASSED");
        }
    }

    // Power off sensors now that diagnostics are complete
    fluctus_release_bus_power(POWER_BUS_3V3, "IMPLUVIUM_DIAG");
    fluctus_release_bus_power(POWER_BUS_5V, "IMPLUVIUM_DIAG");

    // Clear emergency state
    irrigation_system.emergency.state = EMERGENCY_NONE;

    // Return to normal operation
    ESP_LOGI(TAG, "Returning to normal irrigation operation");
    xTimerStart(xMoistureCheckTimer, 0); // Restart periodic checks
    impluvium_change_state(IMPLUVIUM_STANDBY);

    return ESP_OK;
}

/**
 * @brief Monitoring task for continuous safety checks during watering
 *
 * Runs in parallel with main irrigation task, performing:
 * - Flow rate monitoring
 * - Pressure/water level checks
 * - Smart moisture-based cutoffs
 * - Adaptive pump speed control
 * - Real-time telemetry updates
 */
void impluvium_monitoring_task(void *pvParameters)
{
    const char *task_tag = "IMPLVM_MONITOR_TASK";
    ESP_LOGI(task_tag, "Irrigation monitoring task started");

    // Monitoring state variables
    static bool continuous_monitoring = false;
    static uint32_t error_count = 0;
    static uint32_t last_pulse_count = 0;
    static uint32_t last_flow_measurement_time = 0;
    uint32_t notification_value = 0;

    while (1) {
        notification_value = 0;
        // Wait for a notification to trigger action
        xTaskNotifyWait(0x00,
                        ULONG_MAX,
                        &notification_value,
                        continuous_monitoring ? pdMS_TO_TICKS(MONITORING_INTERVAL_MS) : portMAX_DELAY);

        // Handle specific notifications
        if (notification_value & MONITORING_TASK_NOTIFY_START_MONITORING) {
            ESP_LOGI(task_tag, "Starting continuous monitoring for watering");
            continuous_monitoring = true;
            error_count = 0;
            last_flow_measurement_time = 0; // Reset flow measurement
            pcnt_unit_get_count(flow_pcnt_unit, (int *) &last_pulse_count);
        }

        if (notification_value & MONITORING_TASK_NOTIFY_STOP_MONITORING) {
            ESP_LOGI(task_tag, "Stopping continuous monitoring");
            continuous_monitoring = false;
        }

        // Continuous monitoring only during watering
        if (continuous_monitoring) {
            // Take mutex before accessing any shared data
            if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                // Double-check state inside mutex to handle race conditions
                if (irrigation_system.state != IMPLUVIUM_WATERING) {
                    xSemaphoreGive(xIrrigationMutex);
                    continuous_monitoring = false; // State changed, stop monitoring
                    continue;
                }

                uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                uint32_t time_since_start_ms = current_time - irrigation_system.watering_start_time;

                // Update flow rate
                impluvium_calc_flow_rate(task_tag, &last_pulse_count, &last_flow_measurement_time, current_time);

                // Safety monitoring
                if (!impluvium_periodic_safety_check(&error_count, time_since_start_ms)) {
                    // Emergency condition detected. The stop function notifies the main task.
                    continuous_monitoring = false;
                }

                // Real-time moisture monitoring and smart cutoffs
                impluvium_watering_cutoffs_check(task_tag, time_since_start_ms);

                // Use the gain rate calculated in the check function for adaptive control
                uint8_t zone_id = irrigation_system.active_zone;
                if (zone_id < IRRIGATION_ZONE_COUNT) {
                    impluvium_pump_adaptive_control(irrigation_system.current_moisture_gain_rate,
                                                    irrigation_zones[zone_id].learning.target_moisture_gain_rate);
                }

                // Release mutex before calling telemetry (avoid deadlock)
                xSemaphoreGive(xIrrigationMutex);

                // Update TELEMETRY realtime cache OUTSIDE mutex to avoid deadlock
                // (telemetry will take its own mutex, then call back to irrigation)
                if (telemetry_is_realtime_enabled()) {
                    esp_err_t ret = telemetry_fetch_snapshot(TELEMETRY_SRC_IMPLUVIUM_RT);
                    if (ret != ESP_OK) {
                        ESP_LOGW(task_tag, "Failed to update TELEMETRY realtime cache: %s", esp_err_to_name(ret));
                    }
                }

            } else {
                ESP_LOGW(task_tag, "Failed to get mutex for monitoring cycle");
            }
        }
    }
}
