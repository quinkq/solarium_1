/**
 * @file fluctus_private.h
 * @brief Shared internal declarations for FLUCTUS power system modules
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * This private header centralizes shared data structures, global variables, and
 * internal function declarations that are used across multiple fluctus modules.
 *
 * ARCHITECTURE NOTES:
 * - Per-module mutexes prevent high-frequency task blocking (v2.0 design)
 * - Each module owns its state (declared in module .c file, extern here)
 * - Only included by fluctus_*.c files, NOT part of public API
 *
 * MODULE OWNERSHIP (state + mutex):
 * - fluctus_power_bus.c:    system_status, xPowerBusMutex
 * - fluctus_power_monitor.c: monitoring_data, xMonitoringMutex, ina219_dev[]
 * - fluctus_solar_tracking.c: cached_solar_data, xSolarMutex
 * - fluctus_energy.c:        rtc_fluctus_accumulator, xEnergyMutex
 * - fluctus_thermal.c:       ds18b20_*, temp_state (shared with monitoring_data)
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#ifndef FLUCTUS_PRIVATE_H
#define FLUCTUS_PRIVATE_H

#include "fluctus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "ina219.h"
#include "ds18x20.h"

// ########################## Per-Module Mutexes ##########################
// Each module owns its mutex to prevent contention between high/low frequency tasks

extern SemaphoreHandle_t xPowerBusMutex;       // Protects system_status (power_bus.c)
extern SemaphoreHandle_t xMonitoringMutex;     // Protects monitoring_data (power_monitor.c)
extern SemaphoreHandle_t xSolarMutex;          // Protects cached_solar_data (solar_tracking.c)
extern SemaphoreHandle_t xEnergyMutex;         // Protects rtc_fluctus_accumulator (energy.c)

// ########################## Module State Variables ##########################

// Power Bus Control State (owned by fluctus_power_bus.c)
extern fluctus_power_status_t system_status;
extern bool hardware_bus_state[POWER_BUS_COUNT];

// Power Monitoring State (owned by fluctus_power_monitor.c)
extern fluctus_monitoring_data_t monitoring_data;
extern ina219_t ina219_dev[FLUCTUS_INA219_DEVICE_COUNT];
extern int64_t overcurrent_start_time;
extern bool overcurrent_timer_active;

// Solar Tracking State (owned by fluctus_solar_tracking.c)
// solar_data is the SOURCE OF TRUTH for all solar/servo data (protected by xSolarMutex)
extern fluctus_solar_data_t solar_data;
extern int64_t last_correction_time;
extern int64_t correction_start_time;
extern uint8_t consecutive_tracking_errors;

// Energy Tracking State (owned by fluctus_energy.c)
// Note: RTC_DATA_ATTR only on definition in energy.c, not on extern declaration
extern power_accumulator_rtc_t rtc_fluctus_accumulator;

// Thermal Management State (shared between thermal.c and power_monitor.c)
extern onewire_addr_t ds18b20_addr;
extern bool ds18b20_found;
extern int64_t last_temp_read_time;

// DS18B20 async conversion state machine
typedef enum {
    TEMP_STATE_IDLE,        // No conversion in progress
    TEMP_STATE_CONVERTING   // Conversion started, waiting for completion
} temp_conversion_state_t;

extern temp_conversion_state_t temp_state;
extern int64_t temp_conversion_start_time;

// Power Metering State (owned by fluctus_power_monitor.c)
extern int64_t steady_state_probe_start;  // Steady state probe timer

// ########################## Solar Tracking Enums ##########################

// Parking reason tracker (determines SLEEPING vs DISABLED after parking)
typedef enum {
    PARKING_REASON_SUNSET,          // Natural sunset - transition to SLEEPING
    PARKING_REASON_USER_DISABLE,    // Manual disable - transition to DISABLED
    PARKING_REASON_CRITICAL_POWER   // Critical power - transition to DISABLED
} parking_reason_t;

extern parking_reason_t current_parking_reason;

// ########################## Task Handles ##########################

extern TaskHandle_t xFluctusCoreOrchestrationTaskHandle;  // Core orchestration task (Low-3)
extern TaskHandle_t xFluctusMonitoringTaskHandle;         // Power monitoring task (Med-5)
extern TaskHandle_t xFluctusSolarTrackingTaskHandle;      // Solar tracking task (Med-5)
extern TaskHandle_t xFluctusSolarServoControlTaskHandle;  // Servo control task (Med-5)

// ########################## System Flags & Configuration ##########################

extern bool fluctus_initialized;  // System initialization flag

// Interval configuration (loaded from interval_config at init)
extern uint32_t fluctus_power_day_interval_ms;         // Power check interval during daytime
extern uint32_t fluctus_power_night_interval_ms;       // Power check interval during nighttime
extern uint32_t fluctus_solar_correction_interval_ms;  // Solar tracking correction cycle interval

// ########################## Constants ##########################

#define FLUCTUS_MAX_CONSECUTIVE_ERRORS 5  // Solar tracking error threshold

// ########################## Internal Function Declarations ##########################

// ==================== Initialization Functions ====================
// Each module provides its own init function, called from fluctus_init()

esp_err_t fluctus_gpio_init(void);           // Power bus GPIO init (power_bus.c)
esp_err_t fluctus_ina219_init(void);         // INA219 sensor init (power_monitor.c)
esp_err_t fluctus_servo_pwm_init(void);      // Servo PWM init (solar_tracking.c)
esp_err_t fluctus_fan_pwm_init(void);        // Fan PWM init (thermal.c)
esp_err_t fluctus_ds18b20_init(void);        // DS18B20 sensor init (thermal.c)

// ==================== Power Bus Control Functions ====================
// Module: fluctus_power_bus.c

esp_err_t fluctus_update_bus_hardware(power_bus_t bus);
esp_err_t fluctus_add_consumer(power_bus_t bus, const char* consumer_id);
esp_err_t fluctus_remove_consumer(power_bus_t bus, const char* consumer_id);
void fluctus_log_active_consumers(const char* event_context);
void fluctus_log_power_event(fluctus_event_type_t event_type, power_bus_t bus,
                             const char* consumer_id);

// ==================== Power Monitoring Functions ====================
// Module: fluctus_power_monitor.c

esp_err_t fluctus_read_ina219_sensors(void);
void fluctus_check_overcurrent(void);
void fluctus_check_3v3_bus_voltage(void);
esp_err_t fluctus_ina219_set_power_mode(uint8_t device_index, bool active);
void fluctus_set_ina_metering_mode(bool monitoring_active);
void fluctus_monitoring_task(void *parameters);  // FreeRTOS task

// ==================== Solar Tracking Functions ====================
// Module: fluctus_solar_tracking.c

esp_err_t fluctus_read_photoresistors(fluctus_solar_data_t *data);
esp_err_t fluctus_servo_set_position(ledc_channel_t channel, uint32_t duty_cycle);
uint32_t fluctus_percent_to_duty(float percent);
float fluctus_duty_to_percent(uint32_t duty);
esp_err_t fluctus_park_servos_night(void);
esp_err_t fluctus_park_servos_error(void);
bool fluctus_tracking_error_margin_check(float yaw_error, float pitch_error);
void fluctus_set_tracking_state(solar_tracking_state_t new_state, const char *log_msg);
bool fluctus_is_daytime_with_sufficient_light(void);

// Solar tracking state handlers
void fluctus_solar_state_standby(int64_t current_time_ms);
void fluctus_solar_state_correcting(void);  // Blocking entry function, no parameters
void fluctus_solar_state_parking(void);
void fluctus_solar_state_error(void);

// Solar tracking sunrise callback
void fluctus_on_sunrise_callback(void);

// FreeRTOS tasks
void fluctus_solar_tracking_task(void *parameters);
void fluctus_solar_servo_correction_task(void *parameters);

// Refactored smooth tracking helper functions
bool fluctus_apply_servo_corrections_smooth(fluctus_solar_data_t *tracking_data);
void fluctus_update_solar_cache(fluctus_solar_data_t *reading);

// ==================== Thermal Management Functions ====================
// Module: fluctus_thermal.c

void fluctus_update_fan_speed(float temperature);
bool fluctus_should_read_temperature(bool monitoring_active);
void fluctus_handle_temperature_monitoring(bool monitoring_active);

// ==================== Energy Tracking Functions ====================
// Module: fluctus_accumulator.c

void fluctus_update_energy_accumulator(void);
bool fluctus_check_hourly_rollover(void);
void fluctus_midnight_callback(void);

// ==================== Core Orchestration Functions ====================
// Module: fluctus.c

void fluctus_handle_power_state_change(fluctus_power_state_t new_state);

#endif // FLUCTUS_PRIVATE_H
