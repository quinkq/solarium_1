/**
 * @file fluctus_accumulator.c
 * @brief FLUCTUS energy tracking module
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Manages energy accumulation and statistics for the FLUCTUS power system:
 * - RTC RAM accumulator (survives resets, not power loss)
 * - 15-minute power averaging for telemetry
 * - Hourly energy totals (PV generation + battery consumption)
 * - Daily energy statistics with peak power tracking
 * - Automatic hourly rollover and midnight reset
 *
 * The accumulator is updated every 500ms from the power monitoring task
 * and provides stable averaged values for telemetry snapshots.
 *
 * THREAD SAFETY:
 * - Protected by xEnergyMutex (100ms timeout)
 * - Called from power_monitor task (Med-5 priority)
 * - Midnight callback from TELEMETRY (event-driven)
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "fluctus.h"
#include "fluctus_private.h"
#include "telemetry.h"

static const char *TAG = "FLUCTUS_ACCU";

// ########################## Module State ##########################

SemaphoreHandle_t xEnergyMutex = NULL;
RTC_DATA_ATTR power_accumulator_rtc_t rtc_fluctus_accumulator = {0};

// ########################## Energy Accumulation ##########################

/**
 * @brief Update power energy accumulator (called every 500ms from monitoring task)
 *
 * Accumulates energy in Wh using trapezoidal integration:
 * - Energy = Power × Time
 * - Updates hourly/daily totals and peak power tracking
 * - Maintains 15-minute rolling averages for telemetry
 *
 * @param pv_power_w Current PV power in watts (instantaneous)
 * @param battery_power_w Current battery power in watts (positive = consumption)
 */
void fluctus_update_energy_accumulator(float pv_power_w, float battery_power_w)
{
    if (xSemaphoreTake(xEnergyMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire energy mutex (100ms timeout)");
        return;
    }

    time_t now = time(NULL);

    // Initialize accumulator on first call or after power loss
    if (!rtc_fluctus_accumulator.initialized || rtc_fluctus_accumulator.current_hour_start == 0) {
        ESP_LOGI(TAG, "Initializing RTC power accumulator (first boot or power loss)");
        rtc_fluctus_accumulator.current_hour_start = now;
        rtc_fluctus_accumulator.current_day_start = now;
        rtc_fluctus_accumulator.interval_start_15min = now;
        rtc_fluctus_accumulator.last_sample_time = now;
        rtc_fluctus_accumulator.initialized = true;
        xSemaphoreGive(xEnergyMutex);  // Don't forget to release!
        return;
    }

    // Calculate time delta for energy accumulation
    time_t delta_sec = now - rtc_fluctus_accumulator.last_sample_time;

    // Sanity check: skip if time delta is invalid (>10s indicates clock issue or missed samples)
    if (delta_sec <= 0 || delta_sec > 10) {
        ESP_LOGW(TAG, "Invalid time delta: %ld seconds, skipping energy accumulation", (long)delta_sec);
        rtc_fluctus_accumulator.last_sample_time = now;
        xSemaphoreGive(xEnergyMutex);  // Don't forget to release!
        return;
    }

    // Accumulate energy: Wh = W × hours
    float delta_hour = delta_sec / 3600.0f;
    rtc_fluctus_accumulator.pv_energy_wh_accumulator += pv_power_w * delta_hour;
    rtc_fluctus_accumulator.battery_energy_wh_accumulator += battery_power_w * delta_hour;

    // Update hourly peaks
    if (pv_power_w > rtc_fluctus_accumulator.pv_peak_w_hour) {
        rtc_fluctus_accumulator.pv_peak_w_hour = pv_power_w;
    }
    if (battery_power_w > rtc_fluctus_accumulator.battery_peak_w_hour) {
        rtc_fluctus_accumulator.battery_peak_w_hour = battery_power_w;
    }

    // Update daily peaks
    if (pv_power_w > rtc_fluctus_accumulator.pv_peak_w_day) {
        rtc_fluctus_accumulator.pv_peak_w_day = pv_power_w;
    }
    if (battery_power_w > rtc_fluctus_accumulator.battery_peak_w_day) {
        rtc_fluctus_accumulator.battery_peak_w_day = battery_power_w;
    }

    // Update 15-minute averaging sums
    rtc_fluctus_accumulator.pv_power_sum_15min += pv_power_w;
    rtc_fluctus_accumulator.pv_voltage_sum_15min += monitoring_data.solar_pv.voltage;
    rtc_fluctus_accumulator.pv_current_sum_15min += monitoring_data.solar_pv.current;
    rtc_fluctus_accumulator.battery_power_sum_15min += battery_power_w;
    rtc_fluctus_accumulator.battery_voltage_sum_15min += monitoring_data.battery_voltage;
    rtc_fluctus_accumulator.battery_current_sum_15min += monitoring_data.battery_out.current;
    rtc_fluctus_accumulator.sample_count_15min++;

    rtc_fluctus_accumulator.last_sample_time = now;

    xSemaphoreGive(xEnergyMutex);
}

// ########################## Hourly Rollover ##########################

/**
 * @brief Check for hourly rollover and update daily totals
 *
 * Called from fluctus_monitoring_task every loop iteration (500ms).
 * Detects hour changes and:
 * - Adds completed hour energy to daily totals
 * - Counts active hours (hours with >0.1 Wh PV generation)
 * - Resets hourly accumulators
 * - Logs hourly summary
 * @note Return value currently unused. Intended for future MQTT hourly
 *
 * @return true if hour changed (hourly rollover occurred), false otherwise
 */
bool fluctus_check_hourly_rollover(void)
{
    if (!rtc_fluctus_accumulator.initialized) {
        return false;
    }

    time_t now = time(NULL);
    struct tm tm_now, tm_hour_start;
    bool rollover_occurred = false;

    if (xSemaphoreTake(xEnergyMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire energy mutex for hourly check");
        return false;
    }

    gmtime_r(&now, &tm_now);
    gmtime_r(&rtc_fluctus_accumulator.current_hour_start, &tm_hour_start);

    // Check if hour changed
    if (tm_now.tm_hour != tm_hour_start.tm_hour ||
        tm_now.tm_yday != tm_hour_start.tm_yday ||
        tm_now.tm_year != tm_hour_start.tm_year) {

        // Update daily totals with completed hour data
        rtc_fluctus_accumulator.pv_energy_wh_day += rtc_fluctus_accumulator.pv_energy_wh_accumulator;
        rtc_fluctus_accumulator.battery_consumed_wh_day += rtc_fluctus_accumulator.battery_energy_wh_accumulator;

        // Count active hours (hour with significant PV generation)
        if (rtc_fluctus_accumulator.pv_energy_wh_accumulator > 0.1f) {
            rtc_fluctus_accumulator.hours_active_day++;
        }

        ESP_LOGI(TAG, "Hourly rollover: PV %.1f Wh, Battery %.1f Wh consumed",
                 rtc_fluctus_accumulator.pv_energy_wh_accumulator,
                 rtc_fluctus_accumulator.battery_energy_wh_accumulator);

        // Reset hourly counters
        rtc_fluctus_accumulator.current_hour_start = now;
        rtc_fluctus_accumulator.pv_energy_wh_accumulator = 0.0f;
        rtc_fluctus_accumulator.battery_energy_wh_accumulator = 0.0f;
        rtc_fluctus_accumulator.pv_peak_w_hour = 0.0f;
        rtc_fluctus_accumulator.battery_peak_w_hour = 0.0f;

        rollover_occurred = true;
    }

    xSemaphoreGive(xEnergyMutex);
    return rollover_occurred;
}

// ########################## Midnight Reset ##########################

/**
 * @brief Midnight callback - daily energy summary and reset
 *
 * Registered with solar_calc during fluctus_init().
 * Called automatically at midnight (00:00:00 local time).
 *
 * Actions performed:
 * - Logs daily energy summary (PV total, battery consumed, net energy)
 * - Triggers MQTT telemetry snapshot publication
 * - Resets daily counters for new day
 *
 * IMPORTANT: Snapshot is captured BEFORE reset to preserve full day's data.
 */
void fluctus_midnight_callback(void)
{
    if (xSemaphoreTake(xEnergyMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire energy mutex for midnight callback!");
        return;
    }

    ESP_LOGI(TAG, "=== Midnight Callback: Daily Energy Summary ===");
    ESP_LOGI(TAG, "  PV total:         %.1f Wh", rtc_fluctus_accumulator.pv_energy_wh_day);
    ESP_LOGI(TAG, "  Battery consumed: %.1f Wh", rtc_fluctus_accumulator.battery_consumed_wh_day);
    ESP_LOGI(TAG, "  Net energy:       %.1f Wh",
             rtc_fluctus_accumulator.pv_energy_wh_day - rtc_fluctus_accumulator.battery_consumed_wh_day);
    ESP_LOGI(TAG, "  Hours active:     %d", rtc_fluctus_accumulator.hours_active_day);
    ESP_LOGI(TAG, "  PV peak (day):    %.1f W", rtc_fluctus_accumulator.pv_peak_w_day);
    ESP_LOGI(TAG, "  Battery peak:     %.1f W", rtc_fluctus_accumulator.battery_peak_w_day);
    ESP_LOGI(TAG, "===============================================");

    // Trigger MQTT publish with current snapshot (BEFORE reset)
    telemetry_fetch_snapshot(TELEMETRY_SRC_FLUCTUS);

    // Reset daily counters AFTER snapshot captured
    rtc_fluctus_accumulator.current_day_start = time(NULL);
    rtc_fluctus_accumulator.pv_energy_wh_day = 0.0f;
    rtc_fluctus_accumulator.battery_consumed_wh_day = 0.0f;
    rtc_fluctus_accumulator.pv_peak_w_day = 0.0f;
    rtc_fluctus_accumulator.battery_peak_w_day = 0.0f;
    rtc_fluctus_accumulator.hours_active_day = 0;

    ESP_LOGI(TAG, "Daily energy counters reset");

    xSemaphoreGive(xEnergyMutex);
}
