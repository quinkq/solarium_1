/**
 * @file fluctus_power_monitor.c
 * @brief FLUCTUS power monitoring module
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Handles real-time power monitoring and battery management:
 * - Dual INA219 sensors (Solar PV @ 0x40, Battery @ 0x41)
 * - Adaptive power metering (500ms active / 15min steady / nighttime shutdown)
 * - Overcurrent protection (3A/3s warning, 4A immediate shutdown)
 * - 3.3V bus voltage monitoring
 * - Battery state detection (voltage thresholds for load shedding)
 * - Event-driven notification dispatch to core orchestration
 *
 * ADAPTIVE METERING STRATEGY:
 * - Active: Constant monitoring (500ms) when buses powered
 * - Steady: 15s probes every 15min (Stellaria-only or idle)
 * - Shutdown: INA power-down at night (saves ~2mA)
 *
 * THREAD SAFETY:
 * - Protected by xMonitoringMutex (100ms timeout)
 * - Runs in monitoring task (Med-5 priority)
 *
 * EVENT-DRIVEN LOAD SHEDDING (v2.0):
 * - Detects battery state changes (voltage thresholds)
 * - Notifies core orchestration task (non-blocking)
 * - Core coordinates component power modes
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "fluctus.h"
#include "fluctus_private.h"
#include "ads1115_helper.h"
#include "telemetry.h"
#include "solar_calc.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

static const char *TAG = "FLUCTUS_P_MON";

// ########################## Module State ##########################
// Note: NO 'static' keyword - these are accessed via extern in fluctus_private.h

SemaphoreHandle_t xMonitoringMutex = NULL;  // Monitoring data mutex (initialized in fluctus.c)

// Power monitoring data (INA219 readings, temperature, fan state)
fluctus_monitoring_data_t monitoring_data = {0};

// INA219 device configurations (0 = Solar PV @ 0x40, 1 = Battery @ 0x41)
ina219_t ina219_dev[FLUCTUS_INA219_DEVICE_COUNT];

// Overcurrent protection state machine
int64_t overcurrent_start_time = 0;
bool overcurrent_timer_active = false;

// Power metering state
int64_t steady_state_probe_start = 0;  // Steady state probe timer

// ########################## INA219 Initialization ##########################

/**
 * @brief Initialize both INA219 power monitoring sensors
 *
 * Configures dual INA219 sensors for power monitoring:
 * - Solar PV (0x40): 32V range, 0.5A gain, 0.1Ω shunt
 * - Battery (0x41): 32V range, 0.125A gain, 0.01Ω shunt (higher current)
 *
 * Both configured for continuous shunt+bus measurement at 12-bit, 8 samples avg.
 *
 * @return ESP_OK on success, ESP_FAIL if any sensor init fails
 */
esp_err_t fluctus_ina219_init(void)
{
    ESP_LOGI(TAG, "Initializing INA219 power monitoring sensors...");

    // Initialize "Solar PV" sensor (0x40) descriptor
    esp_err_t ret = ina219_init_desc(&ina219_dev[0], FLUCTUS_INA219_SOLAR_PV_ADDR,
                                     I2C_NUM_0, CONFIG_I2CDEV_DEFAULT_SDA_PIN, CONFIG_I2CDEV_DEFAULT_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize description Solar PV INA219: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = ina219_init(&ina219_dev[0]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Solar PV INA219: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure Solar PV INA219
    ret = ina219_configure(&ina219_dev[0], INA219_BUS_RANGE_32V, INA219_GAIN_0_5,
                          INA219_RES_12BIT_8S, INA219_RES_12BIT_8S, INA219_MODE_CONT_SHUNT_BUS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Solar PV INA219: %s", esp_err_to_name(ret));
        return ret;
    }

    // Calibrate with 0.1 ohm shunt for Solar PV
    ret = ina219_calibrate(&ina219_dev[0], 0.1f);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate Solar PV INA219: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize "Battery Output" sensor (0x41) descriptor
    ret = ina219_init_desc(&ina219_dev[1], FLUCTUS_INA219_BATTERY_OUT_ADDR,
                          I2C_NUM_0, CONFIG_I2CDEV_DEFAULT_SDA_PIN, CONFIG_I2CDEV_DEFAULT_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Battery Output INA219: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = ina219_init(&ina219_dev[1]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Battery Output INA219: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure Battery Output INA219 (higher current range)
    ret = ina219_configure(&ina219_dev[1], INA219_BUS_RANGE_32V, INA219_GAIN_0_125,
                          INA219_RES_12BIT_8S, INA219_RES_12BIT_8S, INA219_MODE_CONT_SHUNT_BUS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Battery Output INA219: %s", esp_err_to_name(ret));
        return ret;
    }

    // Calibrate with 0.01 ohm shunt for Battery Output (higher current range)
    ret = ina219_calibrate(&ina219_dev[1], 0.01f);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate Battery Output INA219: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "INA219 sensors initialized successfully");
    return ESP_OK;
}

// ########################## Power Monitoring ##########################

/**
 * @brief Read all INA219 sensors and update monitoring data
 *
 * Reads voltage, current, and power from both INA219 sensors.
 * Updates monitoring_data structure with timestamp and validity flags.
 *
 * @return ESP_OK if both sensors read successfully, ESP_FAIL if any fails
 */
esp_err_t fluctus_read_ina219_sensors(void)
{
    esp_err_t ret = ESP_OK;

    // Read Solar PV sensor
    float voltage, current, power;
    if (ina219_get_bus_voltage(&ina219_dev[0], &voltage) == ESP_OK &&
        ina219_get_current(&ina219_dev[0], &current) == ESP_OK &&
        ina219_get_power(&ina219_dev[0], &power) == ESP_OK) {

        monitoring_data.solar_pv.voltage = voltage;
        monitoring_data.solar_pv.current = current;
        monitoring_data.solar_pv.power = power;
        monitoring_data.solar_pv.valid = true;
        monitoring_data.solar_pv.timestamp = time(NULL);
    } else {
        monitoring_data.solar_pv.valid = false;
        ret = ESP_FAIL;
    }

    // Read Battery Output sensor
    if (ina219_get_bus_voltage(&ina219_dev[1], &voltage) == ESP_OK &&
        ina219_get_current(&ina219_dev[1], &current) == ESP_OK &&
        ina219_get_power(&ina219_dev[1], &power) == ESP_OK) {

        monitoring_data.battery_out.voltage = voltage;
        monitoring_data.battery_out.current = current;
        monitoring_data.battery_out.power = power;
        monitoring_data.battery_out.valid = true;
        monitoring_data.battery_out.timestamp = time(NULL);

        // Use battery output voltage as system battery voltage
        monitoring_data.battery_voltage = voltage;
        monitoring_data.total_current = current;
        monitoring_data.total_power = power;
    } else {
        monitoring_data.battery_out.valid = false;
        ret = ESP_FAIL;
    }

    return ret;
}

// ########################## Adaptive Power Metering ##########################

/**
 * @brief Set INA219 power mode (active or power-down)
 *
 * Controls INA219 operating mode to save power when not needed.
 * Power-down mode saves ~1mA per sensor.
 *
 * @param device_index INA219 device index (0 = PV, 1 = Battery)
 * @param active true to activate, false to power down
 * @return ESP_OK on success
 */
esp_err_t fluctus_ina219_set_power_mode(uint8_t device_index, bool active)
{
    if (device_index >= FLUCTUS_INA219_DEVICE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    // INA219 operating mode bits (bits 0-2 of config register)
    // 0b000 = Power-down, 0b111 = Continuous shunt+bus
    ina219_mode_t mode = active ? INA219_MODE_CONT_SHUNT_BUS : INA219_MODE_POWER_DOWN;

    // Reconfigure with new mode (preserve other settings)
    esp_err_t ret = ina219_configure(&ina219_dev[device_index],
                                    INA219_BUS_RANGE_32V,
                                    device_index == 0 ? INA219_GAIN_0_5 : INA219_GAIN_0_125,
                                    INA219_RES_12BIT_8S,
                                    INA219_RES_12BIT_8S,
                                    mode);

    if (ret == ESP_OK) {
        const char *ina_name = (device_index == 0) ? "PV" : "Battery";
        ESP_LOGD(TAG, "INA219 %s %s", ina_name, active ? "activated" : "powered down");
    }

    return ret;
}

/**
 * @brief Update power metering state based on system activity
 *
 * Manages INA219 power modes with 3-tier strategy:
 * 1. PV INA: Active during daytime, powered down at night (saves ~1mA)
 * 2. Battery INA Active: Continuous monitoring when buses active
 * 3. Battery INA Steady: 15s probes every 15min (Stellaria-only or idle)
 *
 * @param monitoring_active Is power monitoring currently active?
 */
void fluctus_set_ina_metering_mode(bool monitoring_active)
{
    // ====== PV INA219 Power Management (Daytime Only) ======

    // Use centralized daytime/light detection (independent of tracking state)
    bool should_pv_be_active = fluctus_is_daytime_with_sufficient_light();

    // Update PV INA power state
    if (should_pv_be_active != monitoring_data.pv_ina_active) {
        if (fluctus_ina219_set_power_mode(0, should_pv_be_active) == ESP_OK) {
            monitoring_data.pv_ina_active = should_pv_be_active;
            ESP_LOGI(TAG, "PV INA219 %s (%s)", should_pv_be_active ? "activated" : "powered down",
                     should_pv_be_active ? "daytime" : "nighttime");
        }
    }

    // ====== Battery INA219 Power Management (Active/Steady State) ======

    // Determine system activity level
    bool any_bus_active = false;
    bool stellaria_only = false;

    for (int i = 0; i < POWER_BUS_COUNT; i++) {
        if (system_status.bus_enabled[i]) {
            any_bus_active = true;
            break;
        }
    }

    // Check for Stellaria-only scenario (12V bus only with 1 consumer)
    if (system_status.bus_enabled[POWER_BUS_12V] &&
        system_status.bus_ref_count[POWER_BUS_12V] == 1 &&
        !system_status.bus_enabled[POWER_BUS_3V3] &&
        !system_status.bus_enabled[POWER_BUS_5V] &&
        !system_status.bus_enabled[POWER_BUS_6V6]) {
            stellaria_snapshot_t stellaria_data;
            if (telemetry_get_stellaria_data(&stellaria_data) == ESP_OK &&
                stellaria_data.state == STELLARIA_STATE_ENABLED) {
                stellaria_only = true;
            }
    }

    // Determine metering state (inlined logic from fluctus_get_recommended_meter_state)
    fluctus_power_meter_state_t recommended_state;
    if (!any_bus_active) {
        // No buses active = steady state
        recommended_state = FLUCTUS_POWER_METER_STATE_STEADY;
    } else if (stellaria_only) {
        // Only Stellaria active = steady state (extrapolate power from recent history)
        recommended_state = FLUCTUS_POWER_METER_STATE_STEADY;
    } else {
        // Any other bus active = active monitoring
        recommended_state = FLUCTUS_POWER_METER_STATE_ACTIVE;
    }

    // Handle Battery INA power state based on recommended state
    if (recommended_state == FLUCTUS_POWER_METER_STATE_STEADY) {
        // Steady state: Probe for 15 seconds every 15 minutes.
        int64_t current_time_ms = esp_timer_get_time() / 1000;

        if (steady_state_probe_start == 0) {
            // Start new probe cycle
            steady_state_probe_start = current_time_ms;
            if (!monitoring_data.battery_ina_active) {
                fluctus_ina219_set_power_mode(1, true);
                monitoring_data.battery_ina_active = true;
                ESP_LOGD(TAG, "Battery INA: Starting steady state probe");
            }
        } else if ((current_time_ms - steady_state_probe_start) < FLUCTUS_POWER_STEADY_PROBE_DURATION_MS) {
            // Within probe duration - keep active
            if (!monitoring_data.battery_ina_active) {
                fluctus_ina219_set_power_mode(1, true);
                monitoring_data.battery_ina_active = true;
            }
        } else if ((current_time_ms - steady_state_probe_start) < FLUCTUS_POWER_STEADY_MONITOR_INTERVAL_MS) {
            // Between probes - power down
            if (monitoring_data.battery_ina_active) {
                fluctus_ina219_set_power_mode(1, false);
                monitoring_data.battery_ina_active = false;
                ESP_LOGD(TAG, "Battery INA: Ending steady state probe");
            }
        } else {
            // Cycle complete - reset for next probe
            steady_state_probe_start = 0;
        }
    } else {
        // Active state: Keep Battery INA powered
        if (!monitoring_data.battery_ina_active) {
            fluctus_ina219_set_power_mode(1, true);
            monitoring_data.battery_ina_active = true;
            ESP_LOGD(TAG, "Battery INA: Activated for active monitoring");
        }
        steady_state_probe_start = 0;  // Reset steady state timer
    }
}

// ########################## Safety & Protection ##########################

/**
 * @brief Check for overcurrent conditions and handle safety shutdown
 *
 * Two-tier overcurrent protection:
 * - Immediate: ≥4A → instant shutdown
 * - Delayed: ≥3A for 3 seconds → delayed shutdown
 *
 * Hysteresis prevents oscillation during brief current spikes.
 */
void fluctus_check_overcurrent(void)
{
    if (!monitoring_data.battery_out.valid) {
        return;
    }

    float current = monitoring_data.total_current;
    int64_t current_time_ms = esp_timer_get_time() / 1000;

    // Check immediate shutdown threshold (4A)
    if (current >= FLUCTUS_OVERCURRENT_THRESHOLD_2) {
        ESP_LOGE(TAG, "IMMEDIATE OVERCURRENT SHUTDOWN: %.2fA >= %.2fA",
                 current, FLUCTUS_OVERCURRENT_THRESHOLD_2);

        fluctus_log_active_consumers("IMMEDIATE OVERCURRENT SHUTDOWN");

        system_status.safety_shutdown = true;
        system_status.manual_reset_required = true;
        fluctus_log_power_event(FLUCTUS_EVENT_OVERCURRENT_SHUTDOWN, POWER_BUS_COUNT, NULL);

        // Directly call shutdown handler (overcurrent is critical, bypass notification)
        fluctus_handle_power_state_change(FLUCTUS_POWER_STATE_SHUTDOWN);
        return;
    }

    // Check delayed shutdown threshold (3A for 3 seconds)
    if (current >= FLUCTUS_OVERCURRENT_THRESHOLD_1) {
        if (!overcurrent_timer_active) {
            overcurrent_start_time = current_time_ms;
            overcurrent_timer_active = true;
            ESP_LOGW(TAG, "Overcurrent warning: %.2fA >= %.2fA (timer started)",
                     current, FLUCTUS_OVERCURRENT_THRESHOLD_1);

            fluctus_log_active_consumers("OVERCURRENT WARNING");
            fluctus_log_power_event(FLUCTUS_EVENT_OVERCURRENT_WARNING, POWER_BUS_COUNT, NULL);
        } else if ((current_time_ms - overcurrent_start_time) >= FLUCTUS_OVERCURRENT_DELAY_MS) {
            ESP_LOGE(TAG, "DELAYED OVERCURRENT SHUTDOWN: %.2fA for %dms",
                     current, FLUCTUS_OVERCURRENT_DELAY_MS);

            fluctus_log_active_consumers("DELAYED OVERCURRENT SHUTDOWN");

            system_status.safety_shutdown = true;
            system_status.manual_reset_required = true;
            overcurrent_timer_active = false;
            fluctus_log_power_event(FLUCTUS_EVENT_OVERCURRENT_SHUTDOWN, POWER_BUS_COUNT, NULL);

            // Directly call shutdown handler (overcurrent is critical, bypass notification)
            fluctus_handle_power_state_change(FLUCTUS_POWER_STATE_SHUTDOWN);
        }
    } else {
        // Current below threshold, reset timer
        if (overcurrent_timer_active) {
            ESP_LOGI(TAG, "Overcurrent condition cleared");
            overcurrent_timer_active = false;
        }
    }
}

/**
 * @brief Check 3.3V bus voltage level for drift
 *
 * Reads ADS1115 device #1, channel 3 and warns if voltage is outside ±0.1V of 3.3V.
 * Used for early detection of buck converter issues.
 */
void fluctus_check_3v3_bus_voltage(void)
{
    int16_t raw_value;
    float voltage;

    // Read 3V3 bus voltage from ADS1115 device #1, Channel 3
    esp_err_t ret = ads1115_helper_read_channel(1, ADS111X_MUX_3_GND, &raw_value, &voltage);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Failed to read 3V3 bus voltage: %s", esp_err_to_name(ret));
        return;
    }

    // Check if voltage is within acceptable range (3.3V ± 0.1V)
    const float target_voltage = 3.3f;
    const float tolerance = 0.1f;
    float voltage_error = fabsf(voltage - target_voltage);

    if (voltage_error > tolerance) {
        ESP_LOGW(TAG, "3V3 bus voltage drift detected: %.3fV (target: %.1fV ±%.1fV)",
                 voltage, target_voltage, tolerance);
    } else {
        ESP_LOGD(TAG, "3V3 bus voltage OK: %.3fV", voltage);
    }
}

// ########################## Load Shedding Detection ##########################

/**
 * @brief Check battery voltage and detect power state changes
 *
 * Monitors battery voltage against power state thresholds with hysteresis:
 * - Falling thresholds: Different values when battery discharging
 * - Rising thresholds: Different values when battery recovering (prevents oscillation)
 *
 * Power States (by battery voltage):
 * - NORMAL:      > 12.4V (rising: > 12.5V)
 * - POWER_SAVING: ≤ 12.4V (rising: > 12.4V)
 * - LOW_POWER:    ≤ 12.0V (rising: > 12.1V)
 * - VERY_LOW:     ≤ 11.7V (rising: > 11.8V)
 * - CRITICAL:     ≤ 11.4V (rising: > 11.5V)
 * - SHUTDOWN:    Safety shutdown override
 *
 * When a power state change is detected, notifies the core orchestration task
 * via xTaskNotify() with the new state value. The core task handles component
 * load shedding coordination.
 *
 * Called from monitoring task every cycle (500ms active / 15min idle).
 *
 * THREAD SAFETY: Assumes caller holds xMonitoringMutex (monitoring_data access).
 */
void fluctus_monitor_check_power_state_change(void)
{
    float battery_voltage = monitoring_data.battery_voltage;
    fluctus_power_state_t current_state = system_status.power_state;
    fluctus_power_state_t new_state = current_state;

    // Safety shutdown overrides everything
    if (system_status.safety_shutdown) {
        new_state = FLUCTUS_POWER_STATE_SHUTDOWN;
    } else {
        // Check voltage thresholds with hysteresis
        if (current_state <= FLUCTUS_POWER_STATE_NORMAL) {
            // Falling thresholds (battery discharging - turn off systems)
            if (battery_voltage <= FLUCTUS_BATTERY_LEVEL_CRITICAL) {
                new_state = FLUCTUS_POWER_STATE_CRITICAL;
            } else if (battery_voltage <= FLUCTUS_BATTERY_LEVEL_VERY_LOW) {
                new_state = FLUCTUS_POWER_STATE_VERY_LOW;
            } else if (battery_voltage <= FLUCTUS_BATTERY_LEVEL_LOW_POWER) {
                new_state = FLUCTUS_POWER_STATE_LOW_POWER;
            } else if (battery_voltage <= FLUCTUS_BATTERY_LEVEL_POWER_SAVING) {
                new_state = FLUCTUS_POWER_STATE_POWER_SAVING;
            }
        } else {
            // Rising thresholds (battery recovering - turn on systems with hysteresis)
            if (battery_voltage >= FLUCTUS_BATTERY_LEVEL_POWER_SAVING_ON) {
                new_state = FLUCTUS_POWER_STATE_NORMAL;
            } else if (battery_voltage >= FLUCTUS_BATTERY_LEVEL_LOW_POWER_ON &&
                      current_state >= FLUCTUS_POWER_STATE_LOW_POWER) {
                new_state = FLUCTUS_POWER_STATE_POWER_SAVING;
            } else if (battery_voltage >= FLUCTUS_BATTERY_LEVEL_VERY_LOW_ON &&
                      current_state >= FLUCTUS_POWER_STATE_VERY_LOW) {
                new_state = FLUCTUS_POWER_STATE_LOW_POWER;
            } else if (battery_voltage >= FLUCTUS_BATTERY_LEVEL_CRITICAL_ON &&
                      current_state >= FLUCTUS_POWER_STATE_CRITICAL) {
                new_state = FLUCTUS_POWER_STATE_VERY_LOW;
            }
        }
    }

    // If state changed, notify core orchestration task
    if (new_state != current_state) {
        ESP_LOGI(TAG, "Power state change detected: %s -> %s (battery: %.2fV)",
                 fluctus_power_state_to_string(current_state),
                 fluctus_power_state_to_string(new_state),
                 battery_voltage);

        // Update state immediately (prevents duplicate notifications)
        system_status.power_state = new_state;

        // Notify core orchestration task with new power state
        if (xFluctusCoreOrchestrationTaskHandle != NULL) {
            // Encode power state in notification value (bits 8-15)
            uint32_t notification_value = FLUCTUS_NOTIFY_POWER_STATE_CHANGE |
                                         ((uint32_t)new_state << 8);
            xTaskNotify(xFluctusCoreOrchestrationTaskHandle,
                       notification_value,
                       eSetValueWithOverwrite);
            ESP_LOGD(TAG, "Power state change notification sent to core task");
        } else {
            ESP_LOGW(TAG, "Core orchestration task not initialized - cannot notify power state change!");
        }
    }
}

// ########################## FreeRTOS Task ##########################

/**
 * @brief Power monitoring FreeRTOS task (notification-based with dynamic timeout)
 *
 * Main monitoring loop with adaptive intervals:
 * - Active: 500ms (when buses powered)
 * - Idle day: User-configurable (default 15min)
 * - Idle night: User-configurable (default 60min)
 *
 * Responsibilities:
 * - INA219 sensor reading
 * - Overcurrent protection
 * - Battery state detection (load shedding triggers)
 * - Temperature monitoring
 * - Energy accumulation
 * - Telemetry injection
 *
 * Runs at Med-5 priority.
 */
void fluctus_monitoring_task(void *parameters)
{
    ESP_LOGI(TAG, "Power monitoring task started");

    bool monitoring_active = false;

    while (true) {
        uint32_t notification_value = 0;
        TickType_t timeout;

        // Determine timeout based on monitoring state and time of day
        if (monitoring_active) {
            // Active monitoring - short timeout for continuous monitoring
            timeout = pdMS_TO_TICKS(FLUCTUS_POWER_ACTIVE_MONITOR_INTERVAL_MS);
        } else {
            // Idle monitoring - use day/night aware interval (from config)
            uint32_t idle_interval_ms = solar_calc_is_daytime_buffered() ?
                                       fluctus_power_day_interval_ms :
                                       fluctus_power_night_interval_ms;
            timeout = pdMS_TO_TICKS(idle_interval_ms);
        }

        // Wait for notification or timeout
        BaseType_t notified = xTaskNotifyWait(0x00,              // Don't clear on entry
                                              ULONG_MAX,          // Clear all on exit
                                              &notification_value, // Store notification
                                              timeout);            // Timeout

        // Handle power activity notification
        if (notified == pdTRUE && (notification_value & FLUCTUS_NOTIFY_POWER_ACTIVITY)) {
            ESP_LOGI(TAG, "Power activity notification received - starting active monitoring");
            monitoring_active = true;
        }

        // Handle config update notification (causes immediate recalculation of timeout)
        if (notified == pdTRUE && (notification_value & FLUCTUS_NOTIFY_CONFIG_UPDATE)) {
            ESP_LOGI(TAG, "Configuration update notification - recalculating timeout");
            // Loop immediately to apply new timeout
            continue;
        }

        // Update power metering state (INA power-down management)
        fluctus_set_ina_metering_mode(monitoring_active);

        // Read power sensors
        if (xSemaphoreTake(xMonitoringMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
            fluctus_read_ina219_sensors();
            fluctus_check_3v3_bus_voltage();  // Check 3.3V bus voltage
            fluctus_handle_temperature_monitoring(monitoring_active);  // Temperature monitoring

            // Update energy accumulator with current power readings
            fluctus_update_energy_accumulator(
                monitoring_data.solar_pv.power,
                monitoring_data.battery_out.power
            );

            // Check for hourly rollover (updates daily totals, resets hourly counters)
            fluctus_check_hourly_rollover();

            xSemaphoreGive(xMonitoringMutex);
        }

        // Check power management conditions
        if (xSemaphoreTake(xMonitoringMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
            fluctus_check_overcurrent();
            fluctus_monitor_check_power_state_change();  // Check battery state and notify core on changes

            // Check if any buses are still active
            bool system_active = false;
            for (int i = 0; i < POWER_BUS_COUNT; i++) {
                if (system_status.bus_enabled[i]) {
                    system_active = true;
                    break;
                }
            }

            // If no buses active, switch to idle monitoring
            if (!system_active && monitoring_active) {
                ESP_LOGD(TAG, "No active buses - switching to idle monitoring");
                monitoring_active = false;
            }

            xSemaphoreGive(xMonitoringMutex);
        }

        // Update telemetry cache after every monitoring cycle
        if (telemetry_is_realtime_enabled()) {
            telemetry_fetch_snapshot(TELEMETRY_SRC_FLUCTUS_RT);
        }
        telemetry_fetch_snapshot(TELEMETRY_SRC_FLUCTUS);
    }
}

// ########################## Public API ##########################

/**
 * @brief Manually reset safety shutdown state
 *
 * Clears overcurrent shutdown flag and resets system to NORMAL state.
 * Only works if manual reset is required (manual_reset_required flag set).
 *
 * Thread-safe with xMonitoringMutex (1000ms timeout).
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized or reset not required
 * @return ESP_FAIL if mutex timeout
 */
esp_err_t fluctus_manual_safety_reset(void)
{
    if (!fluctus_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xMonitoringMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
        if (!system_status.manual_reset_required) {
            xSemaphoreGive(xMonitoringMutex);
            return ESP_ERR_INVALID_STATE;
        }

        system_status.safety_shutdown = false;
        system_status.manual_reset_required = false;
        overcurrent_timer_active = false;

        // Reset to normal power state
        system_status.power_state = FLUCTUS_POWER_STATE_NORMAL;

        ESP_LOGI(TAG, "Manual safety reset performed - system ready");

        xSemaphoreGive(xMonitoringMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}
