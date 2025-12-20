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
 * Stored in /irrigation/config.dat (~50 bytes)
 * Write trigger: When user updates via MQTT (future feature)
 */
typedef struct {
    uint32_t magic;                    // 0x494D5043 ("IMPC")
    uint16_t version;                  // Format version (1)
    uint16_t crc16;                    // CRC16-CCITT of zones[] data

    struct {
        bool enabled;                       // Zone enabled/disabled
        float target_moisture_percent;      // Desired moisture level
        float moisture_deadband_percent;    // Tolerance around target
        uint8_t padding;                    // Alignment
    } zones[IRRIGATION_ZONE_COUNT];         // 10 bytes × 5 = 50 bytes

    uint32_t last_saved;               // Timestamp (epoch seconds)
} __attribute__((packed)) irrigation_config_file_t;

/**
 * @brief Learning data file structure (all zones)
 *
 * Stored in /irrigation/learning.dat (~350 bytes)
 * Write trigger: Daily at midnight via impluvium_daily_reset_callback()
 */
typedef struct {
    uint32_t magic;                    // 0x494D504C ("IMPL")
    uint16_t version;                  // Format version (1)
    uint16_t crc16;                    // CRC16-CCITT of zones[] data

    struct {
        // Learned parameters (20 bytes)
        float calculated_ppmp_ratio;              // Pulses per moisture percent
        uint32_t calculated_pump_duty_cycle;      // Optimal pump speed
        float target_moisture_gain_rate;          // Adaptive gain rate (%/sec)
        float confidence_level;                   // 0.0-1.0
        uint32_t successful_predictions;          // Success count
        uint32_t total_predictions;               // Total count

        // Recent history (5 entries = 47 bytes)
        uint8_t history_count;                    // Valid entries (0-5)
        uint8_t history_write_index;              // Circular buffer position
        uint16_t padding;                         // Alignment
        float pulses_used[LEARNING_STORED_HISTORY];                // 20 bytes
        float moisture_increase_percent[LEARNING_STORED_HISTORY];  // 20 bytes
        bool anomaly_flags[LEARNING_STORED_HISTORY];               // 5 bytes
        uint8_t padding2[2];                      // Alignment

    } zones[IRRIGATION_ZONE_COUNT];    // 69 bytes × 5 = 345 bytes

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
esp_err_t impluvium_pump_ramp_up(uint8_t zone_id);
void impluvium_pump_adaptive_control(float current_gain_rate, float target_gain_rate);

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
esp_err_t impluvium_calculate_zone_target_pulses(uint8_t zone_id, uint8_t queue_index);
esp_err_t impluvium_calculate_zone_watering_predictions(void);
esp_err_t impluvium_process_zone_watering_data(uint8_t zone_id, uint32_t pulses_used,
                                                float moisture_increase_percent, bool learning_valid);

// --- State Machine Functions (impluvium_state_machine.c) ---
esp_err_t impluvium_change_state(impluvium_state_t new_state);
esp_err_t impluvium_state_measuring(void);
esp_err_t impluvium_state_watering(void);
esp_err_t impluvium_state_stopping(void);
esp_err_t impluvium_state_maintenance(void);

// --- Safety Functions (impluvium_safety.c) ---
esp_err_t impluvium_pre_check(void);
void impluvium_calc_flow_rate(const char *task_tag, uint32_t *last_pulse_count,
                                     uint32_t *last_flow_measurement_time, uint32_t current_time);
esp_err_t impluvium_watering_cutoffs_check(const char *task_tag, uint32_t time_since_start_ms);
bool impluvium_periodic_safety_check(uint32_t *error_count, uint32_t time_since_start_ms);
esp_err_t impluvium_emergency_stop(const char *reason);
esp_err_t emergency_diagnostics_init(void);
esp_err_t emergency_diagnostics_start(const char *reason);
esp_err_t emergency_diagnostics_check_moisture_levels(void);
esp_err_t emergency_diagnostics_test_zone(uint8_t zone_id);
esp_err_t emergency_diagnostics_analyze_results(void);
esp_err_t emergency_diagnostics_resolve(void);
void impluvium_monitoring_task(void *pvParameters);

#endif // IMPLUVIUM_PRIVATE_H
