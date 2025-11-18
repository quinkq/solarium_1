/**
 * @file fluctus.c
 * @brief Power management and solar tracking system
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Original implementation of FLUCTUS - the central power management system.
 *
 * Key features:
 * - Four-bus power distribution (3.3V/5V/6.6V/12V) with reference counting
 * - Five-stage load shedding coordinated across all components
 * - Dual INA219 power monitoring (solar PV and battery)
 * - Dual-axis solar tracking with NOAA position calculations
 * - Adaptive power metering (500ms to 15min intervals)
 * - Sensor-level power gating for maximum efficiency
 * - Thermal management with PWM fan control
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "fluctus.h"
#include "fluctus_private.h"
#include "stellaria.h"
#include "impluvium.h"
#include "tempesta.h"
#include "telemetry.h"
#include "solar_calc.h"
#include "interval_config.h"

#include "esp_timer.h"
#include <string.h>

// ########################## Constants and Variables ##########################

static const char *TAG = "FLUCTUS";

// Task handles
TaskHandle_t xFluctusMonitoringTaskHandle = NULL;
TaskHandle_t xFluctusSolarTrackingTaskHandle = NULL;
TaskHandle_t xFluctusCoreOrchestrationTaskHandle = NULL;

// System initialization flag
bool fluctus_initialized = false;

// Interval configuration (loaded from interval_config at init)
uint32_t fluctus_power_day_interval_ms = 0;         // Power check interval during daytime
uint32_t fluctus_power_night_interval_ms = 0;       // Power check interval during nighttime
uint32_t fluctus_solar_correction_interval_ms = 0;  // Solar tracking correction cycle interval

// ########################## Private Function Declarations ##########################

static float fluctus_calculate_battery_soc(float voltage);
static void fluctus_core_orchestration_task(void *parameters);  // Core module task

// ########################## Private Functions ##########################

/**
 * @brief Handle power state changes and load shedding
 */
void fluctus_handle_power_state_change(fluctus_power_state_t new_state)
{
    if (xSemaphoreTake(xPowerBusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        fluctus_power_state_t old_state = system_status.power_state;
        system_status.power_state = new_state;
        
        ESP_LOGI(TAG, "Power state change: %s -> %s", 
                fluctus_power_state_to_string(old_state),
                fluctus_power_state_to_string(new_state));
        
        fluctus_log_power_event(FLUCTUS_EVENT_POWER_STATE_CHANGE, POWER_BUS_COUNT, NULL);
        
        // Implement load shedding logic based on power state transitions
        switch (new_state) {
            case FLUCTUS_POWER_STATE_NORMAL:
                // Full operation - all systems enabled
                ESP_LOGI(TAG, "Power state: NORMAL - all systems operational");
                stellaria_set_power_save_mode(false);  // Disable power save mode
                stellaria_set_shutdown(false);         // Restore from shutdown
                impluvium_set_power_save_mode(false);  // Normal moisture check intervals
                impluvium_set_shutdown(false);         // Restore irrigation operations
                tempesta_set_power_save_mode(false);   // Normal 15min weather collection
                tempesta_set_shutdown(false);          // Restore weather monitoring
                tempesta_set_pms5003_enabled(true);    // Enable air quality sensor
                wifi_helper_set_power_save_mode(false);              // WiFi normal mode (~100mA)
                wifi_helper_set_shutdown(false);                     // Ensure WiFi enabled
                telemetry_enable_telemetry_publishing(true);         // Enable normal MQTT publishing
                telemetry_force_realtime_monitoring_disable(false);  // Restore realtime mode user preference
                break;

            case FLUCTUS_POWER_STATE_POWER_SAVING:
                // Limited operation - STELLARIA power save mode (max 20% intensity)
                ESP_LOGI(TAG, "Power state: POWER_SAVING - STELLARIA limited to 20%% intensity");
                stellaria_set_power_save_mode(true);   // Enable power save mode (max 12% intensity)
                stellaria_set_shutdown(false);         // Ensure not in shutdown
                impluvium_set_power_save_mode(true);   // 60min moisture check interval
                impluvium_set_shutdown(false);         // Keep irrigation operational
                tempesta_set_power_save_mode(false);   // Keep normal weather collection
                tempesta_set_shutdown(false);          // Keep weather monitoring
                tempesta_set_pms5003_enabled(true);    // Keep air quality sensor enabled
                wifi_helper_set_power_save_mode(false);              // Keep WiFi normal mode
                wifi_helper_set_shutdown(false);                     // Keep WiFi enabled
                telemetry_enable_telemetry_publishing(true);         // Keep normal MQTT publishing enabled
                telemetry_force_realtime_monitoring_disable(false);  // Keep realtime mode at user preference
                break;

            case FLUCTUS_POWER_STATE_LOW_POWER:
                // Reduced operation - STELLARIA shutdown, TEMPESTA power save, realtime telemetry disabled
                ESP_LOGI(TAG, "Power state: LOW_POWER - STELLARIA disabled, TEMPESTA power save, realtime telemetry off, WiFi power save");
                stellaria_set_shutdown(true);          // Shutdown STELLARIA
                impluvium_set_power_save_mode(true);   // Keep 60min moisture check interval
                impluvium_set_shutdown(false);         // Keep irrigation operational
                tempesta_set_power_save_mode(true);    // 60min weather collection interval
                tempesta_set_shutdown(false);          // Keep weather monitoring
                tempesta_set_pms5003_enabled(true);    // Keep air quality sensor enabled
                wifi_helper_set_power_save_mode(true);               // WiFi modem power save (~20mA avg)
                wifi_helper_set_shutdown(false);                     // Keep WiFi enabled for MQTT
                telemetry_enable_telemetry_publishing(true);        // Keep normal MQTT publishing
                telemetry_force_realtime_monitoring_disable(true);  // Force disable realtime mode (SOC <25%)
                break;

            case FLUCTUS_POWER_STATE_VERY_LOW:
                // Minimal operation - STELLARIA + IMPLUVIUM shutdown, TEMPESTA skip PMS5003, MQTT buffering only
                ESP_LOGI(TAG, "Power state: VERY_LOW - STELLARIA and IMPLUVIUM disabled, MQTT buffering only, WiFi power save");
                stellaria_set_shutdown(true);          // Shutdown STELLARIA
                impluvium_set_shutdown(true);          // Shutdown all irrigation operations
                tempesta_set_power_save_mode(true);    // 60min weather collection interval
                tempesta_set_shutdown(false);          // Keep weather monitoring
                tempesta_set_pms5003_enabled(false);   // Skip air quality sensor to save power
                wifi_helper_set_power_save_mode(true);               // WiFi modem power save (~20mA avg)
                wifi_helper_set_shutdown(false);                     // Keep WiFi for MQTT buffering
                telemetry_enable_telemetry_publishing(false);       // Buffering only (SOC <15%)
                telemetry_force_realtime_monitoring_disable(true);  // Force disable realtime mode
                break;

            case FLUCTUS_POWER_STATE_CRITICAL:
                // Emergency operation - only essential systems (TEMPESTA + solar tracking + WiFi off), MQTT buffering only
                ESP_LOGI(TAG, "Power state: CRITICAL - TEMPESTA, solar tracking, and WiFi disabled, MQTT buffering only");
                stellaria_set_shutdown(true);          // Shutdown STELLARIA
                impluvium_set_shutdown(true);          // Shutdown all irrigation operations
                tempesta_set_shutdown(true);           // Shutdown weather monitoring
                wifi_helper_set_shutdown(true);                      // Shutdown WiFi (save ~20mA)
                telemetry_enable_telemetry_publishing(false);       // Buffering only (critical power)
                telemetry_force_realtime_monitoring_disable(true);  // Force disable realtime mode
                if (system_status.solar_tracking_state != SOLAR_TRACKING_DISABLED) {
                    current_parking_reason = PARKING_REASON_CRITICAL_POWER;
                    fluctus_disable_solar_tracking();
                }
                break;
            case FLUCTUS_POWER_STATE_SHUTDOWN:
                // Emergency shutdown - disable all buses
                ESP_LOGI(TAG, "Power state: SHUTDOWN - disabling all power buses");
                for (int i = 0; i < POWER_BUS_COUNT; i++) {
                    system_status.bus_enabled[i] = false;
                    fluctus_update_bus_hardware(i);
                }
                system_status.solar_tracking_state = SOLAR_TRACKING_DISABLED;
                break;
            default:
                break;
        } 
        xSemaphoreGive(xPowerBusMutex);
    } else {
        ESP_LOGE(TAG, "Failed to take xPowerBusMutex - fluctus_handle_power_state_change");
    }
}

// ########################## Core Orchestration Task ##########################

/**
 * @brief Core orchestration task - event-driven load shedding coordinator
 *
 * Runs at Low-3 priority (same as monitoring and solar tasks).
 * Waits for notifications from other modules and orchestrates power state changes:
 *
 * Handled Notifications:
 * - FLUCTUS_NOTIFY_POWER_STATE_CHANGE: Battery voltage crossed threshold
 *   → Extract new power state from notification bits [8:15]
 *   → Call fluctus_handle_power_state_change() to coordinate load shedding
 *
 * Design Pattern: Event-driven, non-blocking
 * - No polling or unnecessary CPU usage
 * - Wakes only when power state changes (rare events)
 * - Coordinates component shutdowns in priority order
 * - Prevents safety-critical overcurrent detection from being blocked
 *
 * @param parameters Unused
 */
void fluctus_core_orchestration_task(void *parameters)
{
    ESP_LOGI(TAG, "Core orchestration task started");

    while (true) {
        // Wait indefinitely for power state change notifications
        uint32_t notification_value = 0;
        BaseType_t notified = xTaskNotifyWait(
            0x00,           // Don't clear bits on entry
            ULONG_MAX,      // Clear all bits on exit
            &notification_value,
            portMAX_DELAY   // Wait indefinitely for notification
        );

        if (notified == pdTRUE && (notification_value & FLUCTUS_NOTIFY_POWER_STATE_CHANGE)) {
            // Extract new power state from bits [8:15]
            fluctus_power_state_t new_state = (fluctus_power_state_t)((notification_value >> 8) & 0xFF);

            ESP_LOGI(TAG, "Power state change notification received: %s",
                     fluctus_power_state_to_string(new_state));

            // Call load shedding coordinator
            // NOTE: This is thread-safe because:
            // - system_status is protected by xPowerBusMutex (held briefly in handler)
            // - Component calls (stellaria, impluvium, etc.) are async and thread-safe
            // - Only one power state change at a time (from single monitor task)
            fluctus_handle_power_state_change(new_state);
        }
    }
}

// ########################## Public API Functions ##########################

esp_err_t fluctus_init(void)
{
    ESP_LOGI(TAG, "Initializing FLUCTUS power management and solar tracking system...");

    if (fluctus_initialized) {
        ESP_LOGW(TAG, "FLUCTUS already initialized");
        return ESP_OK;
    }

    xPowerBusMutex = xSemaphoreCreateMutex();
    if (xPowerBusMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create power bus mutex");
        return ESP_FAIL;
    }

    xMonitoringMutex = xSemaphoreCreateMutex();
    if (xMonitoringMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create monitoring mutex");
        vSemaphoreDelete(xPowerBusMutex);
        xPowerBusMutex = NULL;
        return ESP_FAIL;
    }

    xSolarMutex = xSemaphoreCreateMutex();
    if (xSolarMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create solar tracking mutex");
        vSemaphoreDelete(xPowerBusMutex);
        vSemaphoreDelete(xMonitoringMutex);
        xPowerBusMutex = NULL;
        xMonitoringMutex = NULL;
        return ESP_FAIL;
    }

    xEnergyMutex = xSemaphoreCreateMutex();
    if (xEnergyMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create energy tracking mutex");
        vSemaphoreDelete(xPowerBusMutex);
        vSemaphoreDelete(xMonitoringMutex);
        vSemaphoreDelete(xSolarMutex);
        xPowerBusMutex = NULL;
        xMonitoringMutex = NULL;
        xSolarMutex = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Per-module mutexes created (power_bus, monitoring, solar, energy)");

    // Initialize subsystems
    esp_err_t ret = fluctus_gpio_init();
    if (ret != ESP_OK) {
        goto cleanup;
    }

    // INA219 sensors use software power-down mode (INA219_MODE_POWER_DOWN) for power management
    ret = fluctus_ina219_init();
    if (ret != ESP_OK) {
        goto cleanup;
    }
    
    ret = fluctus_servo_pwm_init();
    if (ret != ESP_OK) {
        goto cleanup;
    }
    
    ret = fluctus_fan_pwm_init();
    if (ret != ESP_OK) {
        goto cleanup;
    }

    // Initialize DS18B20 temperature sensor (non-critical - warn but continue on failure)
    ret = fluctus_ds18b20_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "DS18B20 initialization failed - temperature monitoring disabled");
        // Don't goto cleanup - continue without temperature monitoring
    }

    // Initialize system status
    memset(&system_status, 0, sizeof(fluctus_power_status_t));
    system_status.power_state = FLUCTUS_POWER_STATE_NORMAL;
    system_status.current_yaw_duty = FLUCTUS_SERVO_CENTER_DUTY;
    system_status.current_pitch_duty = FLUCTUS_SERVO_CENTER_DUTY;

    // Register sunrise callback for automatic SLEEPING → STANDBY transitions
    ret = solar_calc_register_sunrise_callback(fluctus_on_sunrise_callback);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register sunrise callback - auto-resume at sunrise disabled");
        // Non-critical - continue initialization
    }

    // Register midnight callback for daily energy summary and reset (v3.3)
    ret = solar_calc_register_midnight_callback(fluctus_midnight_callback);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register midnight callback - daily energy reset disabled");
        // Non-critical - continue initialization
    } else {
        ESP_LOGI(TAG, "Midnight callback registered for daily energy tracking");
    }

    // Load interval configuration from global config
    fluctus_power_day_interval_ms = g_interval_config.fluctus_power_day_min * 60 * 1000;
    fluctus_power_night_interval_ms = g_interval_config.fluctus_power_night_min * 60 * 1000;
    fluctus_solar_correction_interval_ms = g_interval_config.fluctus_solar_correction_min * 60 * 1000;
    ESP_LOGI(TAG, "Power monitoring intervals: %lums (day), %lums (night)",
             fluctus_power_day_interval_ms, fluctus_power_night_interval_ms);
    ESP_LOGI(TAG, "Solar tracking correction interval: %lums",
             fluctus_solar_correction_interval_ms);

    // Create monitoring task
    BaseType_t task_ret = xTaskCreate(
        fluctus_monitoring_task,
        "fluctus_monitor",
        4096,
        NULL,
        5,
        &xFluctusMonitoringTaskHandle
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitoring task");
        ret = ESP_FAIL;
        goto cleanup;
    }
    
    // Create solar tracking task
    task_ret = xTaskCreate(
        fluctus_solar_tracking_task,
        "fluctus_solar",
        3072,
        NULL,
        5,
        &xFluctusSolarTrackingTaskHandle
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create solar tracking task");
        ret = ESP_FAIL;
        goto cleanup;
    }

    // Create core orchestration task (event-driven load shedding coordinator)
    task_ret = xTaskCreate(
        fluctus_core_orchestration_task,
        "fluctus_core_orq",
        2048,
        NULL,
        3,
        &xFluctusCoreOrchestrationTaskHandle
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create core orchestration task");
        ret = ESP_FAIL;
        goto cleanup;
    }

    fluctus_initialized = true;
    ESP_LOGI(TAG, "FLUCTUS initialization complete");
    return ESP_OK;
    
cleanup:
    if (xFluctusMonitoringTaskHandle) {
        vTaskDelete(xFluctusMonitoringTaskHandle);
        xFluctusMonitoringTaskHandle = NULL;
    }
    if (xFluctusSolarTrackingTaskHandle) {
        vTaskDelete(xFluctusSolarTrackingTaskHandle);
        xFluctusSolarTrackingTaskHandle = NULL;
    }
    if (xFluctusCoreOrchestrationTaskHandle) {
        vTaskDelete(xFluctusCoreOrchestrationTaskHandle);
        xFluctusCoreOrchestrationTaskHandle = NULL;
    }
    // Delete module mutexes in reverse order of creation
    if (xEnergyMutex) {
        vSemaphoreDelete(xEnergyMutex);
        xEnergyMutex = NULL;
    }
    if (xSolarMutex) {
        vSemaphoreDelete(xSolarMutex);
        xSolarMutex = NULL;
    }
    if (xMonitoringMutex) {
        vSemaphoreDelete(xMonitoringMutex);
        xMonitoringMutex = NULL;
    }
    if (xPowerBusMutex) {
        vSemaphoreDelete(xPowerBusMutex);
        xPowerBusMutex = NULL;
    }
    return ret;
}

fluctus_power_state_t fluctus_get_power_state(void)
{
    if (!fluctus_initialized) {
        return FLUCTUS_POWER_STATE_SHUTDOWN;
    }

    fluctus_power_state_t state = FLUCTUS_POWER_STATE_SHUTDOWN;
    if (xSemaphoreTake(xPowerBusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        state = system_status.power_state;
        xSemaphoreGive(xPowerBusMutex);
    }

    return state;
}


/**
 * @brief Calculate battery SOC percentage from voltage (12V AGM battery)
 */
static float fluctus_calculate_battery_soc(float voltage)
{
    // SOC mapping for 12V AGM battery (50% SOC = 0% displayed)
    // 12.08V = 0% (critical), 12.58V = 100% (power saving threshold)
    if (voltage < FLUCTUS_BATTERY_LEVEL_CRITICAL) {
        return 0.0f;
    } else if (voltage >= FLUCTUS_BATTERY_LEVEL_POWER_SAVING_ON) {
        return 100.0f;
    }

    // Linear interpolation between critical and power saving
    float voltage_range = FLUCTUS_BATTERY_LEVEL_POWER_SAVING_ON - FLUCTUS_BATTERY_LEVEL_CRITICAL;
    float voltage_offset = voltage - FLUCTUS_BATTERY_LEVEL_CRITICAL;
    return (voltage_offset / voltage_range) * 100.0f;
}

/**
 * @brief Write FLUCTUS data directly to TELEMETRY cache (full snapshot)
 *
 * Comprehensive snapshot for HMI/MQTT distribution with all fields.
 * Writes directly to TELEMETRY's cache buffer to eliminate intermediate copy.
 * Only FLUCTUS mutexes needed - TELEMETRY lock/unlock handled by caller.
 *
 * @param cache Pointer to TELEMETRY's fluctus_snapshot_t buffer
 * @return ESP_OK on success, ESP_FAIL if mutex timeout
 */
esp_err_t fluctus_write_to_telemetry_cache(fluctus_snapshot_t *cache)
{
    if (cache == NULL || !fluctus_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    // Acquire module mutexes sequentially (consistent order prevents deadlocks)
    // Order: power_bus → monitoring → solar → energy

    // 1. Lock power bus data (system_status)
    if (xSemaphoreTake(xPowerBusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Power buses - current state snapshot
    cache->bus_3v3_enabled = system_status.bus_enabled[POWER_BUS_3V3];
    cache->bus_5v_enabled = system_status.bus_enabled[POWER_BUS_5V];
    cache->bus_6v6_enabled = system_status.bus_enabled[POWER_BUS_6V6];
    cache->bus_12v_enabled = system_status.bus_enabled[POWER_BUS_12V];
    cache->bus_3v3_consumers = system_status.bus_ref_count[POWER_BUS_3V3];
    cache->bus_5v_consumers = system_status.bus_ref_count[POWER_BUS_5V];
    cache->bus_6v6_consumers = system_status.bus_ref_count[POWER_BUS_6V6];
    cache->bus_12v_consumers = system_status.bus_ref_count[POWER_BUS_12V];

    // System state
    cache->power_state = system_status.power_state;
    cache->safety_shutdown = system_status.safety_shutdown;
    cache->manual_reset_required = system_status.manual_reset_required;
    cache->last_activity_time = system_status.last_activity_time;

    xSemaphoreGive(xPowerBusMutex);

    // 2. Lock monitoring data (power monitoring)
    if (xSemaphoreTake(xMonitoringMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Battery - instantaneous readings
    cache->battery_voltage_inst = monitoring_data.battery_voltage;
    cache->battery_current_inst = monitoring_data.battery_out.current;
    cache->battery_power_inst = monitoring_data.battery_out.power;
    cache->battery_soc_inst = fluctus_calculate_battery_soc(monitoring_data.battery_voltage);

    // Solar - instantaneous readings
    cache->solar_voltage_inst = monitoring_data.solar_pv.voltage;
    cache->solar_current_inst = monitoring_data.solar_pv.current;
    cache->solar_power_inst = monitoring_data.solar_pv.power;
    cache->solar_pv_active = monitoring_data.pv_ina_active;

    // Thermal management - current state
    cache->case_temperature = monitoring_data.case_temperature;
    cache->temperature_valid = monitoring_data.temperature_valid;
    if (monitoring_data.fan_active) {
        if (monitoring_data.case_temperature >= FLUCTUS_FAN_MAX_TEMP_THRESHOLD) {
            cache->fan_speed_percent = 100;
        } else if (monitoring_data.case_temperature <= FLUCTUS_FAN_TURN_ON_TEMP_THRESHOLD) {
            cache->fan_speed_percent = FLUCTUS_FAN_MIN_DUTY;
        } else {
            float temp_range = FLUCTUS_FAN_MAX_TEMP_THRESHOLD - FLUCTUS_FAN_TURN_ON_TEMP_THRESHOLD;
            float temp_offset = monitoring_data.case_temperature - FLUCTUS_FAN_TURN_ON_TEMP_THRESHOLD;
            float duty_range = FLUCTUS_FAN_MAX_DUTY - FLUCTUS_FAN_MIN_DUTY;
            cache->fan_speed_percent = FLUCTUS_FAN_MIN_DUTY +
                (uint8_t)((temp_offset / temp_range) * duty_range);
        }
    } else {
        cache->fan_speed_percent = 0;
    }

    // Validity flags
    cache->battery_data_valid = monitoring_data.battery_out.valid;
    cache->solar_data_valid = monitoring_data.solar_pv.valid;

    xSemaphoreGive(xMonitoringMutex);

    // 3. Lock solar tracking data (cached_solar_data and solar state)
    if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Solar tracking - current state (minimal, no debug data)
    cache->tracking_state = system_status.solar_tracking_state;
    cache->yaw_position_percent = (uint8_t)fluctus_duty_to_percent(system_status.current_yaw_duty);
    cache->pitch_position_percent = (uint8_t)fluctus_duty_to_percent(system_status.current_pitch_duty);

    xSemaphoreGive(xSolarMutex);

    // 4. Lock energy data (rtc_fluctus_accumulator)
    if (xSemaphoreTake(xEnergyMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Battery - calculate 15-minute averages from accumulator sums
    if (rtc_fluctus_accumulator.sample_count_15min > 0) {
        cache->battery_voltage_avg_15min = rtc_fluctus_accumulator.battery_voltage_sum_15min / rtc_fluctus_accumulator.sample_count_15min;
        cache->battery_current_avg_15min = rtc_fluctus_accumulator.battery_current_sum_15min / rtc_fluctus_accumulator.sample_count_15min;
        cache->battery_power_avg_15min = rtc_fluctus_accumulator.battery_power_sum_15min / rtc_fluctus_accumulator.sample_count_15min;
        cache->battery_soc_avg_15min = fluctus_calculate_battery_soc(cache->battery_voltage_avg_15min);
    } else {
        // No samples yet - use current instantaneous values from monitoring
        cache->battery_voltage_avg_15min = monitoring_data.battery_voltage;
        cache->battery_current_avg_15min = monitoring_data.battery_out.current;
        cache->battery_power_avg_15min = monitoring_data.battery_out.power;
        cache->battery_soc_avg_15min = fluctus_calculate_battery_soc(monitoring_data.battery_voltage);
    }

    // Solar - calculate 15-minute averages from accumulator sums
    if (rtc_fluctus_accumulator.sample_count_15min > 0) {
        cache->solar_voltage_avg_15min = rtc_fluctus_accumulator.pv_voltage_sum_15min / rtc_fluctus_accumulator.sample_count_15min;
        cache->solar_current_avg_15min = rtc_fluctus_accumulator.pv_current_sum_15min / rtc_fluctus_accumulator.sample_count_15min;
        cache->solar_power_avg_15min = rtc_fluctus_accumulator.pv_power_sum_15min / rtc_fluctus_accumulator.sample_count_15min;
    } else {
        // No samples yet - use current instantaneous values from monitoring
        cache->solar_voltage_avg_15min = monitoring_data.solar_pv.voltage;
        cache->solar_current_avg_15min = monitoring_data.solar_pv.current;
        cache->solar_power_avg_15min = monitoring_data.solar_pv.power;
    }

    // Energy statistics - hourly totals and peaks
    cache->current_hour_start = rtc_fluctus_accumulator.current_hour_start;
    cache->pv_energy_wh_hour = rtc_fluctus_accumulator.pv_energy_wh_accumulator;
    cache->battery_energy_wh_hour = rtc_fluctus_accumulator.battery_energy_wh_accumulator;
    cache->pv_peak_w_hour = rtc_fluctus_accumulator.pv_peak_w_hour;
    cache->battery_peak_w_hour = rtc_fluctus_accumulator.battery_peak_w_hour;

    // Energy statistics - daily totals and peaks
    cache->current_day_start = rtc_fluctus_accumulator.current_day_start;
    cache->pv_energy_wh_day = rtc_fluctus_accumulator.pv_energy_wh_day;
    cache->battery_consumed_wh_day = rtc_fluctus_accumulator.battery_consumed_wh_day;
    cache->pv_peak_w_day = rtc_fluctus_accumulator.pv_peak_w_day;
    cache->battery_peak_w_day = rtc_fluctus_accumulator.battery_peak_w_day;
    cache->hours_active_day = rtc_fluctus_accumulator.hours_active_day;

    xSemaphoreGive(xEnergyMutex);

    // Note: snapshot_timestamp set by TELEMETRY unlock function

    return ESP_OK;
}

/**
 * @brief Write FLUCTUS realtime data to TELEMETRY cache (rich snapshot)
 *
 * High-frequency power monitoring snapshot for realtime MQTT streaming (QoS 0).
 * Contains instantaneous sensor readings and full debug data (photoresistors, duty cycles).
 * Optimized for 500ms update rate during active monitoring.
 * "Rich" version includes debug fields for removal after debug period.
 *
 * @param cache Pointer to TELEMETRY's fluctus_snapshot_rt_t buffer
 * @return ESP_OK on success, ESP_FAIL if mutex timeout
 */
esp_err_t fluctus_write_realtime_to_telemetry_cache(fluctus_snapshot_t *cache)
{
    if (cache == NULL || !fluctus_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    // Acquire module mutexes sequentially (consistent order prevents deadlocks)
    // Order: power_bus → monitoring → solar (for debug data)

    // 1. Lock power bus data (system_status)
    if (xSemaphoreTake(xPowerBusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Power buses - dynamic state (catches transient activity at 500ms)
    cache->bus_3v3_enabled = system_status.bus_enabled[POWER_BUS_3V3];
    cache->bus_5v_enabled = system_status.bus_enabled[POWER_BUS_5V];
    cache->bus_6v6_enabled = system_status.bus_enabled[POWER_BUS_6V6];
    cache->bus_12v_enabled = system_status.bus_enabled[POWER_BUS_12V];
    cache->bus_3v3_consumers = system_status.bus_ref_count[POWER_BUS_3V3];
    cache->bus_5v_consumers = system_status.bus_ref_count[POWER_BUS_5V];
    cache->bus_6v6_consumers = system_status.bus_ref_count[POWER_BUS_6V6];
    cache->bus_12v_consumers = system_status.bus_ref_count[POWER_BUS_12V];

    // Solar tracking - dynamic state
    cache->tracking_state = system_status.solar_tracking_state;
    cache->yaw_position_percent = (uint8_t)fluctus_duty_to_percent(system_status.current_yaw_duty);
    cache->pitch_position_percent = (uint8_t)fluctus_duty_to_percent(system_status.current_pitch_duty);
    cache->current_yaw_duty = system_status.current_yaw_duty;       // DEBUG: raw PWM duty
    cache->current_pitch_duty = system_status.current_pitch_duty;   // DEBUG: raw PWM duty

    xSemaphoreGive(xPowerBusMutex);

    // 2. Lock monitoring data (power monitoring)
    if (xSemaphoreTake(xMonitoringMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Battery - instantaneous readings
    cache->battery_voltage_inst = monitoring_data.battery_voltage;
    cache->battery_current_inst = monitoring_data.battery_out.current;
    cache->battery_power_inst = monitoring_data.battery_out.power;
    cache->battery_soc_inst = fluctus_calculate_battery_soc(monitoring_data.battery_voltage);  // DEBUG: for removal

    // Solar - instantaneous readings
    cache->solar_voltage_inst = monitoring_data.solar_pv.voltage;
    cache->solar_current_inst = monitoring_data.solar_pv.current;
    cache->solar_power_inst = monitoring_data.solar_pv.power;
    cache->solar_pv_active = monitoring_data.pv_ina_active;

    // Thermal management - instantaneous
    cache->case_temperature = monitoring_data.case_temperature;
    cache->temperature_valid = monitoring_data.temperature_valid;
    if (monitoring_data.fan_active) {
        if (monitoring_data.case_temperature >= FLUCTUS_FAN_MAX_TEMP_THRESHOLD) {
            cache->fan_speed_percent = 100;
        } else if (monitoring_data.case_temperature <= FLUCTUS_FAN_TURN_ON_TEMP_THRESHOLD) {
            cache->fan_speed_percent = FLUCTUS_FAN_MIN_DUTY;
        } else {
            float temp_range = FLUCTUS_FAN_MAX_TEMP_THRESHOLD - FLUCTUS_FAN_TURN_ON_TEMP_THRESHOLD;
            float temp_offset = monitoring_data.case_temperature - FLUCTUS_FAN_TURN_ON_TEMP_THRESHOLD;
            float duty_range = FLUCTUS_FAN_MAX_DUTY - FLUCTUS_FAN_MIN_DUTY;
            cache->fan_speed_percent = FLUCTUS_FAN_MIN_DUTY +
                (uint8_t)((temp_offset / temp_range) * duty_range);
        }
    } else {
        cache->fan_speed_percent = 0;
    }

    // Validity flags
    cache->battery_data_valid = monitoring_data.battery_out.valid;
    cache->solar_data_valid = monitoring_data.solar_pv.valid;

    xSemaphoreGive(xMonitoringMutex);

    // 3. Lock solar tracking data (cached_solar_data - debug info only)
    if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Solar tracking - full debug data (photoresistors, errors)
    cache->yaw_error = cached_solar_data.yaw_error;
    cache->pitch_error = cached_solar_data.pitch_error;
    memcpy(cache->photoresistor_readings, cached_solar_data.photoresistor_readings, sizeof(cache->photoresistor_readings));  // DEBUG

    xSemaphoreGive(xSolarMutex);

    // Note: snapshot_timestamp set by TELEMETRY unlock function

    return ESP_OK;
}


// ########################## End Energy Tracking Functions ##########################


/**
 * @brief Set FLUCTUS power monitoring intervals (runtime adjustment)
 *
 * Updates both configuration file and runtime variables. Sends notification to
 * monitoring task for immediate effect (task wakes up and recalculates timeout).
 *
 * @param day_min Daytime interval in minutes (5-60)
 * @param night_min Nighttime interval in minutes (15-120)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out of range
 */
esp_err_t fluctus_set_power_intervals(uint32_t day_min, uint32_t night_min)
{
    // Validate using interval_config API (enforces ranges)
    esp_err_t ret = interval_config_set_fluctus_power(day_min, night_min);
    if (ret != ESP_OK) {
        return ret;  // Invalid ranges, error already logged
    }

    // Update runtime variables
    fluctus_power_day_interval_ms = day_min * 60 * 1000;
    fluctus_power_night_interval_ms = night_min * 60 * 1000;

    // Wake monitoring task to recalculate timeout immediately
    if (xFluctusMonitoringTaskHandle != NULL) {
        xTaskNotify(xFluctusMonitoringTaskHandle, FLUCTUS_NOTIFY_CONFIG_UPDATE, eSetBits);
    }

    ESP_LOGI(TAG, "Power monitoring intervals updated: %lum (day), %lum (night)",
             day_min, night_min);

    return ESP_OK;
}

/**
 * @brief Set FLUCTUS solar tracking correction interval (runtime adjustment)
 *
 * Updates both configuration file and runtime variable. Solar tracking task
 * will use new interval on next STANDBY cycle.
 *
 * @param correction_min Correction cycle interval in minutes (5-60)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out of range
 */
esp_err_t fluctus_set_solar_interval(uint32_t correction_min)
{
    // Validate using interval_config API (enforces ranges)
    esp_err_t ret = interval_config_set_fluctus_solar(correction_min);
    if (ret != ESP_OK) {
        return ret;  // Invalid range, error already logged
    }

    // Update runtime variable
    fluctus_solar_correction_interval_ms = correction_min * 60 * 1000;

    ESP_LOGI(TAG, "Solar tracking correction interval updated: %lum", correction_min);

    return ESP_OK;
}

const char* fluctus_power_state_to_string(fluctus_power_state_t state)
{
    switch (state) {
        case FLUCTUS_POWER_STATE_NORMAL:        return "NORMAL";
        case FLUCTUS_POWER_STATE_POWER_SAVING:  return "POWER_SAVING";
        case FLUCTUS_POWER_STATE_LOW_POWER:     return "LOW_POWER";
        case FLUCTUS_POWER_STATE_VERY_LOW:      return "VERY_LOW";
        case FLUCTUS_POWER_STATE_CRITICAL:      return "CRITICAL";
        case FLUCTUS_POWER_STATE_SHUTDOWN:      return "SHUTDOWN";
        default:                                return "UNKNOWN";
    }
}