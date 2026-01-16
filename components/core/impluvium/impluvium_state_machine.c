/**
 * @file impluvium_state_machine.c
 * @brief State machine implementation for IMPLUVIUM irrigation system
 *
 * Manages irrigation workflow states: STANDBY -> MEASURING -> WATERING ->
 * STOPPING -> MAINTENANCE. Handles zone queueing, moisture checking,
 * valve/pump control, and learning data updates.
 */

#include "impluvium.h"
#include "impluvium_private.h"
#include "fluctus.h"
#include "stellaria.h"
#include "telemetry.h"

// Should keep them here?:
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "IMPLUVIUM_STATE";

/**
 * @brief Change irrigation system state with logging
 *
 * Updates state and records transition timestamp. All state changes
 * should use this function for consistent logging.
 *
 * @param new_state Target state
 * @return ESP_OK on success
 */
esp_err_t impluvium_change_state(impluvium_state_t new_state)
{
    if (irrigation_system.state != new_state) {
        // Convert states to strings for logging
        const char *old_state_str = "UNKNOWN";
        const char *new_state_str = "UNKNOWN";

        switch (irrigation_system.state) {
            case IMPLUVIUM_STANDBY: old_state_str = "STANDBY"; break;
            case IMPLUVIUM_MEASURING: old_state_str = "MEASURING"; break;
            case IMPLUVIUM_WATERING: old_state_str = "WATERING"; break;
            case IMPLUVIUM_STOPPING: old_state_str = "STOPPING"; break;
            case IMPLUVIUM_MAINTENANCE: old_state_str = "MAINTENANCE"; break;
            case IMPLUVIUM_DISABLED: old_state_str = "DISABLED"; break;
        }

        switch (new_state) {
            case IMPLUVIUM_STANDBY: new_state_str = "STANDBY"; break;
            case IMPLUVIUM_MEASURING: new_state_str = "MEASURING"; break;
            case IMPLUVIUM_WATERING: new_state_str = "WATERING"; break;
            case IMPLUVIUM_STOPPING: new_state_str = "STOPPING"; break;
            case IMPLUVIUM_MAINTENANCE: new_state_str = "MAINTENANCE"; break;
            case IMPLUVIUM_DISABLED: new_state_str = "DISABLED"; break;
        }

        ESP_LOGI(TAG, "State change: %s -> %s", old_state_str, new_state_str);

        irrigation_system.state = new_state;
        irrigation_system.state_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
    return ESP_OK;
}

/**
 * @brief Execute MEASURING state operations
 *
 * Powers on sensor bus, performs safety checks, scans all zones
 * for watering needs, builds prioritized watering queue.
 *
 * @return ESP_OK on success, ESP_FAIL on safety check failure
 */
esp_err_t impluvium_state_measuring(void)
{
    // Check FLUCTUS power state before requesting power
    fluctus_power_state_t power_state = fluctus_get_power_state();
    if (power_state == FLUCTUS_POWER_STATE_CRITICAL) {
        ESP_LOGW(TAG, "Critical power state - skipping irrigation cycle");
        impluvium_change_state(IMPLUVIUM_STANDBY);
        return ESP_OK;
    }

    if (power_state >= FLUCTUS_POWER_STATE_VERY_LOW) {
        ESP_LOGW(TAG, "Very low power state - irrigation disabled for battery conservation");
        impluvium_change_state(IMPLUVIUM_STANDBY);
        return ESP_OK;
    }

    // Power on sensors (3V3 for sensors, 5V for pressure transmitter)
    if (impluvium_request_power_buses(POWER_ONLY_SENSORS, "IMPLUVIUM") != ESP_OK) {
        impluvium_change_state(IMPLUVIUM_MAINTENANCE);
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Perform pre-start safety checks directly
    if (impluvium_pre_check() != ESP_OK) {
        impluvium_release_power_buses(POWER_ONLY_SENSORS, "IMPLUVIUM");

        // If emergency_stop was triggered, go to MAINTENANCE for diagnostics
        // Otherwise, it's just "not ready" (sensors unavailable) - return to STANDBY
        if (irrigation_system.emergency_stop) {
            ESP_LOGW(TAG, "Pre-check failed with safety violation - entering maintenance");
            impluvium_change_state(IMPLUVIUM_MAINTENANCE);
        } else {
            ESP_LOGW(TAG, "Pre-check failed - system not ready, returning to standby");
            impluvium_change_state(IMPLUVIUM_STANDBY);
        }
        return ESP_FAIL;
    }

    // **MANUAL WATERING MODE**: Bypass normal moisture checks and directly queue the manual zone
    if (irrigation_system.manual_watering_active) {
        uint8_t zone_id = irrigation_system.manual_water_zone;

        ESP_LOGI(TAG, "Manual watering mode active for zone %d (duration: %d seconds)",
                 zone_id, irrigation_system.manual_water_duration_sec);

        // Add manual zone directly to queue (bypass moisture check and interval check)
        irrigation_system.watering_queue_size = 0;
        irrigation_system.watering_queue[0].zone_id = zone_id;
        irrigation_system.watering_queue[0].measured_moisture_percent = 0.0f; // Not applicable for manual watering
        irrigation_system.watering_queue[0].moisture_deficit_percent = 0.0f;  // Not applicable
        irrigation_system.watering_queue[0].target_pulses = 0;                // Time-based cutoff instead
        irrigation_system.watering_queue[0].watering_completed = false;
        irrigation_system.watering_queue_size = 1;

        // Start watering immediately
        irrigation_system.queue_index = 0;
        ESP_LOGI(TAG, "Starting manual watering for zone %d", zone_id);
        impluvium_change_state(IMPLUVIUM_WATERING);

        return ESP_OK;
    }

    // Check each zone for watering needs
    for (uint8_t zone_id = 0; zone_id < IRRIGATION_ZONE_COUNT; zone_id++) {
        if (!irrigation_zones[zone_id].watering_enabled) {
            ESP_LOGI(TAG, "Zone %d: SKIPPED - disabled in configuration", zone_id);
            continue;
        }

        float moisture_percent;
        esp_err_t ret = impluvium_read_moisture_sensor(zone_id, &moisture_percent);
        if (ret == ESP_OK) {
            irrigation_zone_t *zone = &irrigation_zones[zone_id];
            zone->last_moisture_percent = moisture_percent; // Store last reading
            float moisture_deficit_percent = zone->target_moisture_percent - moisture_percent;

            // Check if watering is needed
            if (moisture_deficit_percent > zone->moisture_deadband_percent) {
                // Check minimum interval since last watering (using monotonic time)
                int64_t current_time_ms = esp_timer_get_time() / 1000;
                if (zone->last_watered_time_ms == 0 ||
                    (current_time_ms - zone->last_watered_time_ms) >= MIN_WATERING_INTERVAL_MS) {
                    ESP_LOGI(TAG,
                             "Zone %d needs water: %.1f%% < %.1f%% (deficit: %.1f%%)",
                             zone_id,
                             moisture_percent,
                             zone->target_moisture_percent,
                             moisture_deficit_percent);

                    // Add to watering queue
                    if (irrigation_system.watering_queue_size < IRRIGATION_ZONE_COUNT) {
                        irrigation_system.watering_queue[irrigation_system.watering_queue_size].zone_id = zone_id;
                        irrigation_system.watering_queue[irrigation_system.watering_queue_size]
                            .measured_moisture_percent = moisture_percent;
                        irrigation_system.watering_queue[irrigation_system.watering_queue_size]
                            .moisture_deficit_percent = moisture_deficit_percent;
                        irrigation_system.watering_queue[irrigation_system.watering_queue_size].watering_completed = false;
                        irrigation_system.watering_queue_size++;
                    }
                } else {
                    // Too soon since last watering
                    int64_t time_since_watering_s = (current_time_ms - zone->last_watered_time_ms) / 1000;
                    int64_t time_remaining_s = (MIN_WATERING_INTERVAL_MS - (current_time_ms - zone->last_watered_time_ms)) / 1000;
                    ESP_LOGI(TAG,
                             "Zone %d: SKIPPED - watered %lld s ago, need %lld s more (15 min minimum)",
                             zone_id,
                             (long long)time_since_watering_s,
                             (long long)time_remaining_s);
                }
            } else {
                // Moisture within acceptable range
                ESP_LOGI(TAG,
                         "Zone %d: SKIPPED - moisture %.1f%% within target %.1f%% ±%.1f%% (deficit %.1f%%)",
                         zone_id,
                         moisture_percent,
                         zone->target_moisture_percent,
                         zone->moisture_deadband_percent,
                         moisture_deficit_percent);
            }
        }
    }

    // Process the watering queue
    if (irrigation_system.watering_queue_size > 0) {
        // Sort queue by moisture deficit (highest priority first) - Insertion Sort
        for (uint8_t i = 1; i < irrigation_system.watering_queue_size; i++) {
            watering_queue_item_t key = irrigation_system.watering_queue[i];
            int8_t j = i - 1;

            // Move elements with lower deficit one position ahead
            while (j >= 0 &&
                   irrigation_system.watering_queue[j].moisture_deficit_percent < key.moisture_deficit_percent) {
                irrigation_system.watering_queue[j + 1] = irrigation_system.watering_queue[j];
                j--;
            }
            irrigation_system.watering_queue[j + 1] = key;
        }

        // Calculate predicted pulses for each zone using learning algorithm
        impluvium_calculate_zone_watering_predictions();

        // Start watering queue - keep sensors powered
        irrigation_system.queue_index = 0;
        ESP_LOGI(TAG, "Starting watering queue with %d zones", irrigation_system.watering_queue_size);
        impluvium_change_state(IMPLUVIUM_WATERING);
    } else {
        // No zones need watering - power off sensors and return to idle
        impluvium_release_power_buses(POWER_ONLY_SENSORS, "IMPLUVIUM");
        irrigation_system.last_moisture_check = xTaskGetTickCount() * portTICK_PERIOD_MS;
        impluvium_change_state(IMPLUVIUM_STANDBY);

        // Moisture check workflow complete - publish to MQTT
        telemetry_fetch_snapshot(TELEMETRY_SRC_IMPLUVIUM);
    }

    return ESP_OK;
}

/**
 * @brief Execute WATERING state initialization
 *
 * Sets up watering for current zone: opens valve, ramps up pump,
 * initializes flow counting, notifies monitoring task.
 *
 * @return ESP_OK on success, ESP_FAIL on pump/valve failure
 */
esp_err_t impluvium_state_watering(void)
{
    // Now called only once when transitioning into the WATERING state.
    // It sets up the watering for the current zone in the queue.

    // Get current zone from queue
    if (irrigation_system.queue_index >= irrigation_system.watering_queue_size) {
        ESP_LOGE(TAG, "Invalid queue index in WATERING state");
        impluvium_change_state(IMPLUVIUM_STOPPING);
        return ESP_FAIL;
    }

    uint8_t zone_id = irrigation_system.watering_queue[irrigation_system.queue_index].zone_id;
    irrigation_zone_t *zone = &irrigation_zones[zone_id];
    ESP_LOGI(TAG, "Watering sequence started for zone %d", zone_id);

    // Request 12V power for valve & pump operation (sensors already powered from measuring state)
    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_12V, "IMPLUVIUM", {
        impluvium_close_valve(zone_id);
        impluvium_release_power_buses(POWER_ONLY_SENSORS, "IMPLUVIUM");
        impluvium_change_state(IMPLUVIUM_MAINTENANCE);
        return ESP_FAIL;
    });

    // Request level shifter enable for valve/pump control signals (SN74AHCT125)
    if (fluctus_request_level_shifter("IMPLUVIUM") != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable level shifter for zone %d", zone_id);
        impluvium_close_valve(zone_id);
        impluvium_release_power_buses(POWER_ALL_DEVICES, "IMPLUVIUM");
        impluvium_change_state(IMPLUVIUM_MAINTENANCE);
        return ESP_FAIL;
    }

    // Request STELLARIA to dim lights for power management during irrigation
    esp_err_t ret = stellaria_request_irrigation_dim(true);
    if(ret != ESP_OK) {
        ESP_LOGI(TAG, "Skiping STELLARIA dimming...");
    } else {
        ESP_LOGI(TAG, "STELLARIA dimming active - delaying for 1s.");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Open valve and wait for actuation
    impluvium_open_valve(zone_id);
    vTaskDelay(pdMS_TO_TICKS(VALVE_OPEN_DELAY_MS));

    // Ramp up pump speed
    if (impluvium_pump_ramp(zone_id, PUMP_RAMP_UP) != ESP_OK) {
        ESP_LOGE(TAG, "Pump ramp-up failed for zone %d, turning off power buses", zone_id);
        impluvium_close_valve(zone_id);
        fluctus_release_level_shifter("IMPLUVIUM");
        impluvium_release_power_buses(POWER_ALL_DEVICES, "IMPLUVIUM");
        impluvium_change_state(IMPLUVIUM_MAINTENANCE);
        return ESP_FAIL;
    }

    // Reset flow sensor counter and update system state
    pcnt_unit_clear_count(flow_pcnt_unit);
    pcnt_unit_get_count(flow_pcnt_unit, (int *) &irrigation_system.watering_start_pulses);
    irrigation_system.active_zone = zone_id;
    zone->watering_in_progress = true;
    zone->last_watered_time_ms = esp_timer_get_time() / 1000;  // Monotonic time
    irrigation_system.watering_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Store starting moisture for learning algorithm
    irrigation_system.watering_queue[irrigation_system.queue_index].moisture_at_start_percent =
        irrigation_system.watering_queue[irrigation_system.queue_index].measured_moisture_percent;

    // Notify monitoring task to start continuous monitoring
    xTaskNotify(xIrrigationMonitoringTaskHandle, MONITORING_TASK_NOTIFY_START_MONITORING, eSetBits);

    // Now, the main task will simply wait in the while loop for a notification
    // to stop watering. The monitoring task is in full control.
    return ESP_OK;
}

/**
 * @brief STOPPING state - stop pump, update learning, process next zone in queue
 */
esp_err_t impluvium_state_stopping(void)
{
    uint8_t active_zone = irrigation_system.active_zone;

    // Notify monitoring task to stop
    xTaskNotify(xIrrigationMonitoringTaskHandle, MONITORING_TASK_NOTIFY_STOP_MONITORING, eSetBits);

    // Ramp down pump (3 seconds for gentle shutdown)
    if (impluvium_pump_ramp(active_zone, PUMP_RAMP_DOWN) != ESP_OK) {
        ESP_LOGW(TAG, "Pump ramp-down failed, forcing immediate stop");
        impluvium_set_pump_speed(0);
    }
    ESP_LOGI(TAG, "Pump stopped - delaying %dms for pressure equalization", PRESSURE_EQUALIZE_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(PRESSURE_EQUALIZE_DELAY_MS));

    // Restore STELLARIA to previous intensity (irrigation complete)
    stellaria_request_irrigation_dim(false);

    // Close current zone valve
    if (active_zone < IRRIGATION_ZONE_COUNT) {
        impluvium_close_valve(active_zone);
        irrigation_zones[active_zone].watering_in_progress = false;
    } else {
        ESP_LOGE(TAG, "Invalid active zone in STOPPING state: %d", active_zone);
    }

    // Release level shifter after valves closed and pump stopped
    fluctus_release_level_shifter("IMPLUVIUM");

    // Calculate volume used and update learning algorithm
    if (active_zone < IRRIGATION_ZONE_COUNT && irrigation_system.queue_index < irrigation_system.watering_queue_size) {
        int total_pulses;
        pcnt_unit_get_count(flow_pcnt_unit, &total_pulses);
        uint32_t pulses_used = total_pulses - irrigation_system.watering_start_pulses;
        float volume_ml = (pulses_used / FLOW_CALIBRATION_PULSES_PER_LITER) * 1000.0f;

        // Update RTC accumulator with water usage (single source of truth - persistent!)
        impluvium_update_accumulator(active_zone, volume_ml);
        impluvium_check_hourly_rollover();

        // **MANUAL WATERING MODE**: Skip learning algorithm, just log volume used
        if (irrigation_system.manual_watering_active) {
            ESP_LOGI(TAG, "Manual watering completed for zone %d: %lu pulses, %.1f mL (%d seconds)",
                     active_zone, pulses_used, volume_ml, irrigation_system.manual_water_duration_sec);

            // Clear manual watering flags
            irrigation_system.manual_watering_active = false;
            irrigation_system.manual_water_zone = 0;
            irrigation_system.manual_water_duration_sec = 0;
            irrigation_system.manual_water_end_time = 0;

            ESP_LOGI(TAG, "Manual watering mode deactivated");
        }
        // Normal watering: Read final moisture level for learning
        else {
            float final_moisture_percent;
            bool learning_valid = true;
            if (impluvium_read_moisture_sensor(active_zone, &final_moisture_percent) == ESP_OK) {
            irrigation_zones[active_zone].last_moisture_percent = final_moisture_percent; // Store last reading
            watering_queue_item_t *queue_item = &irrigation_system.watering_queue[irrigation_system.queue_index];
            float moisture_increase_percent = final_moisture_percent - queue_item->moisture_at_start_percent;

            // Check for anomalies that would invalidate learning data
            // Note: Temperature check is for learning validity only - safety was already verified before watering
            float current_temperature = tempesta_get_temperature();

            if (irrigation_system.current_anomaly.type != ANOMALY_NONE ||
                current_temperature == WEATHER_INVALID_VALUE ||
                current_temperature < TEMP_EXTREME_LOW || current_temperature > TEMP_EXTREME_HIGH) {
                learning_valid = false;
                ESP_LOGD(TAG,
                         "Zone %d: Learning data invalidated (anomaly=%d, temp=%.1f°C) - extreme conditions affect soil behavior",
                         active_zone,
                         irrigation_system.current_anomaly.type,
                         current_temperature);
            }

            // Calculate watering duration for gain rate learning
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            uint32_t watering_duration_ms = current_time - irrigation_system.watering_start_time;

            // Update learning algorithm with results
            impluvium_process_zone_watering_data(active_zone, pulses_used, moisture_increase_percent, learning_valid);

            // Learn optimal target moisture gain rate (only for valid, reasonable cycles)
            if (learning_valid && watering_duration_ms > 5000 && watering_duration_ms < 60000) { // 5s to 60s range
                float actual_gain_rate = moisture_increase_percent / (watering_duration_ms / 1000.0f);
                if (actual_gain_rate > 0.1f && actual_gain_rate < 2.0f) { // Reasonable gain rate range
                    zone_learning_t *learning = &irrigation_zones[active_zone].learning;
                    // Update target gain rate with exponential moving average (5% new, 95% old for slower adaptation)
                    learning->target_moisture_gain_rate =
                        (learning->target_moisture_gain_rate * 0.95f) + (actual_gain_rate * 0.05f);

                    ESP_LOGI(TAG, "Zone %d: Updated target gain rate: %.2f %%/sec (measured %.2f %%/sec over %lu s)",
                             active_zone, learning->target_moisture_gain_rate, actual_gain_rate, watering_duration_ms / 1000);
                }
            }

            ESP_LOGI(TAG,
                     "Zone %d: Used %lu pulses, %.1fmL, moisture %.1f%%->%.1f%% (+%.1f%%)",
                     active_zone,
                     pulses_used,
                     volume_ml,
                     queue_item->moisture_at_start_percent,
                     final_moisture_percent,
                     moisture_increase_percent);
            }
        } // End of normal watering (else block)

        // Mark current queue item as completed
        irrigation_system.watering_queue[irrigation_system.queue_index].watering_completed = true;
    }

    // Reset anomaly detection for next zone
    memset(&irrigation_system.current_anomaly, 0, sizeof(watering_anomaly_t));

    // Move to next zone in queue
    irrigation_system.queue_index++;
    irrigation_system.active_zone = NO_ACTIVE_ZONE_ID; // Reset active zone

    if (irrigation_system.queue_index < irrigation_system.watering_queue_size) {
        // More zones to water - go back to WATERING state for the next zone
        ESP_LOGI(TAG,
                 "Moving to next zone in queue: %d/%d",
                 irrigation_system.queue_index + 1,
                 irrigation_system.watering_queue_size);
        impluvium_change_state(IMPLUVIUM_WATERING);
    } else {
        // All zones completed - finish session
        ESP_LOGI(TAG, "All %d zones in queue completed", irrigation_system.watering_queue_size);

        // Power off all buses now that all zones are done
        fluctus_release_bus_power(POWER_BUS_12V, "IMPLUVIUM");
        fluctus_release_bus_power(POWER_BUS_5V, "IMPLUVIUM");
        fluctus_release_bus_power(POWER_BUS_3V3, "IMPLUVIUM");

        // Reset queue and state
        irrigation_system.queue_index = 0;
        irrigation_system.watering_queue_size = 0;
        irrigation_system.last_moisture_check = xTaskGetTickCount() * portTICK_PERIOD_MS;

        impluvium_change_state(IMPLUVIUM_MAINTENANCE);
    }

    return ESP_OK;
}

/**
 * @brief MAINTENANCE state - perform system maintenance tasks and emergency diagnostics
 *
 * Handles:
 * - Emergency diagnostics execution and analysis
 * - Daily volume resets
 * - NVS configuration saves
 */
esp_err_t impluvium_state_maintenance(void)
{
    // If an emergency was triggered, start the diagnostic process.
    if (irrigation_system.emergency_stop) {
        ESP_LOGI(TAG, "Emergency detected, beginning diagnostics...");
        emergency_diagnostics_start(irrigation_system.emergency.failure_reason);
        irrigation_system.emergency_stop = false; // Reset trigger
        // emergency_diagnostics_start() sets the emergency state to TRIGGERED.
        // The state machine will loop and re-evaluate this function, entering the logic below.
        return ESP_OK;
    }

    // If we are in an emergency, the diagnostic logic will handle the flow.
    // If not, we perform normal maintenance and return to idle.
    if (irrigation_system.emergency.state != EMERGENCY_NONE) {
        // Convert emergency state to string for logging
        const char *emergency_state_str = "UNKNOWN";
        switch (irrigation_system.emergency.state) {
            case EMERGENCY_NONE: emergency_state_str = "NONE"; break;
            case EMERGENCY_TRIGGERED: emergency_state_str = "TRIGGERED"; break;
            case EMERGENCY_DIAGNOSING: emergency_state_str = "DIAGNOSING"; break;
            case EMERGENCY_USER_REQUIRED: emergency_state_str = "USER_REQUIRED"; break;
            case EMERGENCY_RESOLVED: emergency_state_str = "RESOLVED"; break;
        }

        ESP_LOGI(TAG, "Processing emergency diagnostics (state: %s)", emergency_state_str);

        switch (irrigation_system.emergency.state) {
            case EMERGENCY_TRIGGERED:
                // Start diagnostics by checking moisture levels
                ESP_LOGI(TAG, "Starting emergency diagnostics - checking moisture levels");
                emergency_diagnostics_check_moisture_levels();
                break;

            case EMERGENCY_DIAGNOSING:
                // Find next eligible zone to test
                uint8_t next_zone_to_test = IRRIGATION_ZONE_COUNT;
                for (uint8_t i = irrigation_system.emergency.test_zone; i < IRRIGATION_ZONE_COUNT; i++) {
                    if ((irrigation_system.emergency.eligible_zones_mask & (1 << i))) {
                        next_zone_to_test = i;
                        break;
                    }
                }

                if (next_zone_to_test < IRRIGATION_ZONE_COUNT) {
                    irrigation_system.emergency.test_zone = next_zone_to_test;
                    emergency_diagnostics_test_zone(irrigation_system.emergency.test_zone);
                    // Increment here to ensure we test the next eligible zone in the following cycle
                    irrigation_system.emergency.test_zone++;
                } else {
                    // All eligible zones tested - analyze results
                    emergency_diagnostics_analyze_results();
                }
                break;

            case EMERGENCY_USER_REQUIRED:
                ESP_LOGE(TAG, "Emergency diagnostics failed - USER INTERVENTION REQUIRED");
                ESP_LOGE(TAG, "Failure: %s", irrigation_system.emergency.failure_reason);
                ESP_LOGE(TAG, "Failed zones mask: 0x%02X", irrigation_system.emergency.failed_zones_mask);
                // Power off all buses to conserve power while waiting for user
                fluctus_release_bus_power(POWER_BUS_12V, "IMPLUVIUM");
                fluctus_release_bus_power(POWER_BUS_5V, "IMPLUVIUM");
                fluctus_release_bus_power(POWER_BUS_3V3, "IMPLUVIUM");
                // Stay in this state until user manually resets system. The task will block waiting for a notification.
                return ESP_OK;

            case EMERGENCY_RESOLVED:
                emergency_diagnostics_resolve();
                break;

            default:
                break;
        }
    } else {
        // Normal maintenance after a watering cycle
        ESP_LOGI(TAG, "Maintenance tasks completed, returning to IDLE");
        impluvium_change_state(IMPLUVIUM_STANDBY);

        // Watering workflow complete - publish to MQTT
        telemetry_fetch_snapshot(TELEMETRY_SRC_IMPLUVIUM);
    }

    return ESP_OK;
}
