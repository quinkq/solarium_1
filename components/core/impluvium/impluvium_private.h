/**
 * @file impluvium_private.h
 * @brief Shared internal declarations for IMPLUVIUM irrigation system modules
 *
 * This private header centralizes shared data structures, global variables, and
 * internal function declarations that are used across multiple impluvium modules.
 *
 * Only included by impluvium_*.c files, not part of public API.
 */

#ifndef IMPLUVIUM_PRIVATE_H
#define IMPLUVIUM_PRIVATE_H

#include "impluvium.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_log.h"

#include "abp.h"

// ########################## Global Variables ##########################

/**
 * Per-zone configuration and state.
 * Array of IRRIGATION_ZONE_COUNT zones (5 zones total).
 * Shared across all modules via this global.
 */
extern irrigation_zone_t irrigation_zones[IRRIGATION_ZONE_COUNT];

/**
 * Master system state structure.
 * Contains global state machine, queues, power state, emergency flags, etc.
 */
extern irrigation_system_t irrigation_system;

/**
 * Mutex protecting access to irrigation_zones and irrigation_system.
 * Used by all modules accessing shared state.
 */
extern SemaphoreHandle_t xIrrigationMutex;

/**
 * Main irrigation task handle.
 * Used for task notifications from timer and monitoring task.
 */
extern TaskHandle_t xIrrigationTaskHandle;

/**
 * Irrigation monitoring task handle.
 * Used for task notifications from main task (start/stop monitoring).
 */
extern TaskHandle_t xIrrigationMonitoringTaskHandle;

/**
 * Moisture check timer handle.
 * Periodic timer for moisture scanning cycle.
 */
extern TimerHandle_t xMoistureCheckTimer;

/**
 * Verification timer handle (one-shot).
 * Fires 5 minutes after watering to update redistribution factors.
 */
extern TimerHandle_t xVerificationTimer;

/**
 * Hardware device handles
 */
extern abp_t abp_dev;              // ABP pressure sensor (SPI)
extern pcnt_unit_handle_t flow_pcnt_unit; // Flow sensor pulse counter

// ########################## Interval Variables ##########################

/**
 * Runtime interval settings (updated via impluvium_set_check_intervals).
 * These values change the moisture check timer period dynamically.
 */
extern uint32_t impluvium_optimal_interval_ms;    // Normal temp (≥20°C)
extern uint32_t impluvium_cool_interval_ms;       // Cool temp (10-20°C)
extern uint32_t impluvium_power_save_interval_ms; // Power save mode
extern uint32_t impluvium_night_minimum_ms;       // Nighttime minimum

// ########################## GPIO Mapping ##########################

/**
 * GPIO mapping for zone valve control
 */
extern const gpio_num_t zone_valve_gpios[IRRIGATION_ZONE_COUNT];

// ########################## LittleFS Storage Structures ##########################

#define LEARNING_STORED_HISTORY 5  // Store 5 most recent valid events per zone

/**
 * @brief Zone configuration file structure (user-editable settings)
 *
 * Stored in /irrigation/config.dat (~70 bytes)
 * Write trigger: When user updates via MQTT or HMI
 * Version 2: Added target_moisture_gain_rate per zone
 */
typedef struct {
    uint32_t magic;                    // 0x494D5043 ("IMPC")
    uint16_t version;                  // Format version (2)
    uint16_t crc16;                    // CRC16-CCITT of zones[] data

    struct {
        bool enabled;                          // Zone enabled/disabled
        float target_moisture_percent;         // Desired moisture level
        float moisture_deadband_percent;       // Tolerance around target
        float target_moisture_gain_rate;       // Configurable target gain rate (%/sec)
    } zones[IRRIGATION_ZONE_COUNT];            // 13 bytes × 5 = 65 bytes

    uint32_t last_saved;               // Timestamp (epoch seconds)
} __attribute__((packed)) irrigation_config_file_t;

/**
 * @brief Learning data file structure (all zones)
 *
 * Stored in /irrigation/learning.dat (~270 bytes)
 * Write trigger: Daily at midnight via impluvium_daily_reset_callback()
 * Version 3: Simplified history to store pre-computed ppmp_ratio instead of raw pulses/moisture
 */
typedef struct {
    uint32_t magic;                    // 0x494D504C ("IMPL")
    uint16_t version;                  // Format version (3)
    uint16_t crc16;                    // CRC16-CCITT of zones[] data

    struct {
        // Learned parameters (28 bytes)
        float calculated_ppmp_ratio;              // Weighted average pulses per moisture percent
        uint32_t calculated_pump_duty_cycle;      // Optimal pump speed
        float soil_redistribution_factor;         // Delayed/immediate moisture ratio
        float measured_moisture_gain_rate;        // Raw gain rate from last event (%/sec) - for telemetry
        float confidence_level;                   // 0.0-1.0
        uint32_t successful_predictions;          // Success count
        uint32_t total_predictions;               // Total count

        // Recent history (5 entries, saved newest-first)
        uint8_t history_count;                              // Valid entries (0-5)
        float ppmp_ratio[LEARNING_STORED_HISTORY];          // Pre-computed ratios (20 bytes)
        bool anomaly_flags[LEARNING_STORED_HISTORY];        // 5 bytes

    } zones[IRRIGATION_ZONE_COUNT];    // 54 bytes × 5 = 270 bytes

    uint32_t last_saved;               // Timestamp (epoch seconds)
} __attribute__((packed)) irrigation_learning_file_t;

// ########################## RTC Accumulator ##########################

/**
 * @brief Irrigation accumulator structure (stored in RTC RAM for persistence)
 * Survives ESP32 resets but not power loss
 * Tracks hourly and daily water usage statistics per zone
 *
 * Design: Store per-zone granular data, derive system totals at cache write time
 * Benefits: Single source of truth, no redundancy, full persistence
 */
typedef struct {
    // Per-zone hourly tracking (reset at top of hour)
    time_t current_hour_start;                               // Start time of current hour
    float zone_water_used_hour_ml[IRRIGATION_ZONE_COUNT];    // Hourly per-zone (mL)
    uint8_t zone_events_hour[IRRIGATION_ZONE_COUNT];         // Events per zone this hour

    // Per-zone daily tracking (reset at midnight)
    time_t current_day_start;                                // Day start timestamp
    float zone_water_used_day_ml[IRRIGATION_ZONE_COUNT];     // Daily per-zone (mL)
    uint8_t zone_events_day[IRRIGATION_ZONE_COUNT];          // Events per zone today

    // Metadata
    bool initialized;                                        // Accumulator initialization flag
} irrigation_accumulator_rtc_t;

// RTC RAM accumulator (survives ESP32 resets, persists across deep sleep)
// Note: RTC_DATA_ATTR is only on the definition in impluvium.c, not on extern declaration
extern irrigation_accumulator_rtc_t rtc_impluvium_accumulator;

// ########################## Internal Function Declarations ##########################

// --- Sensor Functions (impluvium_sensors.c) ---
esp_err_t impluvium_read_moisture_sensor(uint8_t zone_id, float *moisture_percent);
esp_err_t impluvium_read_pressure(float *outlet_pressure_bar);
esp_err_t impluvium_read_water_level(float *water_level_percent);

// --- Actuator Functions (impluvium_actuators.c) ---
esp_err_t impluvium_gpio_init(void);
esp_err_t impluvium_pump_init(void);
esp_err_t impluvium_flow_sensor_init(void);
esp_err_t impluvium_set_pump_speed(uint32_t pwm_duty);

// Pump ramp direction
typedef enum {
    PUMP_RAMP_UP = 0,   // Ramp from min to target duty
    PUMP_RAMP_DOWN = 1  // Ramp from current to zero
} pump_ramp_direction_t;

esp_err_t impluvium_pump_speed_ramping(uint8_t zone_id, pump_ramp_direction_t direction);

// Between-event pump duty optimization
void impluvium_update_pump_duty_from_gain_rate(uint8_t zone_id, float measured_moisture_increase,
                                                uint32_t watering_duration_ms, float target_moisture_gain_rate);

// Valve control with logging and validation
esp_err_t impluvium_open_valve(uint8_t zone_id);
esp_err_t impluvium_close_valve(uint8_t zone_id);
esp_err_t impluvium_close_all_valves(void);

// --- Power Management Functions (impluvium.c) ---
// Note: These are called from multiple modules
typedef enum {
    POWER_ONLY_SENSORS = 0,   // 3.3V sensor bus
    POWER_ONLY_PUMP = 1,      // 12V pump bus
    POWER_ALL_DEVICES = 2,    // Both buses
    POWER_PUMP_AND_SOLENOID = 3 // 12V for pump and solenoids
} power_level_t;

esp_err_t impluvium_request_power_buses(power_level_t level, const char *tag);
esp_err_t impluvium_release_power_buses(power_level_t level, const char *tag);

// --- Storage Functions (impluvium_storage.c) ---
esp_err_t impluvium_save_zone_config(void);
esp_err_t impluvium_load_zone_config(void);
esp_err_t impluvium_save_learning_data_all_zones(void);
esp_err_t impluvium_load_learning_data_all_zones(void);
void impluvium_update_accumulator(uint8_t zone_id, float water_used_ml);
bool impluvium_check_hourly_rollover(void);
void impluvium_handle_midnight_reset(void);

// --- Learning Functions (impluvium_learning.c) ---
uint32_t impluvium_calc_moisture_check_interval(float current_temperature);
float impluvium_calculate_temperature_correction(zone_learning_t *learning);
float impluvium_calculate_pulse_per_moisture_percent(zone_learning_t *learning, uint8_t *valid_cycles);
esp_err_t impluvium_precalculate_zone_watering_targets(uint8_t zone_id, uint8_t queue_index);
esp_err_t impluvium_precalculate_zone_watering_predictions(void);
esp_err_t impluvium_postprocess_zone_watering_data(uint8_t zone_id, uint32_t pulses_used,
                                                float moisture_increase_percent,
                                                uint32_t watering_duration_ms, bool learning_valid);
esp_err_t impluvium_update_redistribution_from_delayed_readings(void);

// --- State Machine Functions (impluvium_state_machine.c) ---
esp_err_t impluvium_change_state(impluvium_state_t new_state);
esp_err_t impluvium_state_measuring(void);
esp_err_t impluvium_state_watering(void);
esp_err_t impluvium_state_stopping(void);
esp_err_t impluvium_state_maintenance(void);

// --- Emergency Stop Function (impluvium.c) ---
// Immediate hardware kill - stops pump, closes valves, releases power
// Pass reason=NULL for graceful shutdown, non-NULL for safety emergency (triggers diagnostics)
void impluvium_perform_emergency_stop(const char *reason);

// --- Safety Functions (impluvium_safety.c) ---
void impluvium_set_anomaly(anomaly_type_t type, float value);
esp_err_t impluvium_pre_check(void);
void impluvium_calc_flow_rate(const char *task_tag, uint32_t *last_pulse_count,
                                     uint32_t *last_flow_measurement_time, uint32_t current_time);
void impluvium_check_moisture_gain_rate(const char *task_tag, float current_moisture, uint32_t time_since_start_ms);
bool impluvium_should_stop_watering(const char *task_tag, float current_moisture, uint32_t time_since_start_ms);
bool impluvium_handle_safety_failure(uint32_t *error_count, const char *failure_reason, const char *task_tag);
void impluvium_periodic_safety_check(const char **failure_reason, uint32_t time_since_start_ms);
esp_err_t emergency_diagnostics_init(void);
esp_err_t emergency_diagnostics_start(const char *reason);
esp_err_t emergency_diagnostics_check_moisture_levels(void);
esp_err_t emergency_diagnostics_test_zone(uint8_t zone_id);
esp_err_t emergency_diagnostics_analyze_results(void);
esp_err_t emergency_diagnostics_resolve(void);
void impluvium_monitoring_task(void *pvParameters);

#endif // IMPLUVIUM_PRIVATE_H
