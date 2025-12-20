#ifndef IMPLUVIUM_H
#define IMPLUVIUM_H

#include "ads1115_helper.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include <time.h>


// Forward declaration for telemetry cache lock (uses impluvium_snapshot_t)

// ########################## Global settings and variables ##########################

// Safety Limits
#define MAX_WATERING_TIME_MS (1 * 60 * 1000)      // 1 minute max
#define MIN_WATERING_INTERVAL_MS (15 * 60 * 1000) // 15 minutes between
#define MAX_PRESSURE_BAR 2.5f                     // 3 bar pressure limit
#define MIN_FLOW_RATE_LH 15.0f                    // 15 L/h minimum flow
#define MIN_TEMPERATURE_WATERING 10.0f            // 10°C minimum temp
#define MIN_TEMPERATURE_GLOBAL 0.0f               // 0°C global system safety limit
#define MAX_TEMPERATURE_GLOBAL 50.0f              // 50°C global system safety limit
#define MIN_WATER_LEVEL_MBAR 15.0f                // 15 mbar minimum level // Use percentage??
#define MIN_WATER_LEVEL_PERCENT 3.0f              // % minimum level

#define IRRIGATION_ZONE_COUNT 5

// GPIO Pin Assignments
#define FLOW_SENSOR_GPIO GPIO_NUM_48
#define VALVE_GPIO_ZONE_0 GPIO_NUM_38
#define VALVE_GPIO_ZONE_1 GPIO_NUM_39
#define VALVE_GPIO_ZONE_2 GPIO_NUM_40
#define VALVE_GPIO_ZONE_3 GPIO_NUM_41
#define VALVE_GPIO_ZONE_4 GPIO_NUM_42
#define PUMP_PWM_GPIO GPIO_NUM_46

// ABP Pin definitions
#define ABP_SPI_HOST SPI2_HOST
#define ABP_CS_PIN   GPIO_NUM_12

// Power Management
#define MAX_OUTPUT_CURRENT_AMPS 1.25f // Total system current limit (immediate emergency stop)
#define SENSOR_POWERUP_DELAY_MS 500   // Time for 3.3V sensors to stabilize
#define SERVO_POWERUP_DELAY_MS 100    // Time for 6.2V servo bus to stabilize


// Sensor Retry Settings
#define SENSOR_READ_MAX_RETRIES 3     // Maximum retry attempts for sensor readings
#define SENSOR_READ_RETRY_DELAY_MS 50 // Delay between retry attempts

// Pump PWM Settings
#define PUMP_PWM_FREQUENCY 1000 // 1kHz for pump speed control
#define PUMP_PWM_RESOLUTION LEDC_TIMER_10_BIT
#define PUMP_PWM_TIMER LEDC_TIMER_0  // Dedicated timer for pump (avoid sharing with Stellaria)
#define PUMP_PWM_CHANNEL LEDC_CHANNEL_4  // Changed from 2 to avoid conflict with pitch servo
#define PUMP_MIN_DUTY 434 // 42% minimum duty cycle (pump stall protection)
#define PUMP_MAX_DUTY 1023
#define PUMP_DEFAULT_DUTY 512 // 50% default startup duty cycle

// Flow and Monitoring
#define MONITORING_INTERVAL_MS 500               // 500ms monitoring during watering
#define FLOW_RATE_CALC_PERIOD_MS 500             // Calculate flow rate every 500ms
#define VALVE_OPEN_DELAY_MS 1500                 // Wait for valve to open + power stabilization (ms)
#define PUMP_FLOW_ESTABLISH_DELAY_MS 3000        // Wait for flow to establish after pump start before measuring
#define PRESSURE_EQUALIZE_DELAY_MS 1000          // Wait for pressure equalization after pump stop
#define FLOW_CALIBRATION_PULSES_PER_LITER 400.0f // Global flow sensor calibration
#define NO_ACTIVE_ZONE_ID 255                    // No active zone

// Learning Algorithm Settings
#define MOISTURE_CHECK_INTERVAL_MS (15 * 60 * 1000)                               // 15 minutes (default)
#define MOISTURE_CHECK_INTERVAL_COLD_MS (30 * 60 * 1000)                          // 30 minutes below 20°C
#define MOISTURE_CHECK_INTERVAL_OPTIMAL_MS (15 * 60 * 1000)                       // 15 minutes above 20°C
#define MOISTURE_CHECK_INTERVAL_POWER_SAVE_MS (60 * 60 * 1000)                    // 60 minutes (power save mode)
#define TEMPERATURE_OPTIMAL_THRESHOLD 20.0f                                       // Optimal irrigation temp
#define POST_WATER_CHECK_INTERVALS 3                                              // Check 3 times after watering
#define POST_WATER_CHECK_TIMES_MS {5 * 60 * 1000, 10 * 60 * 1000, 15 * 60 * 1000} // 5, 10, 15 min
#define LEARNING_HISTORY_SIZE 15                                                  // Keep last 15 cycles
#define LEARNING_MIN_CYCLES 3                                                     // Start predictions after 3 cycles
#define LEARNING_WEIGHT_RECENT 0.7f                                               // Weight recent cycles more
#define DEFAULT_TARGET_PULSES 80                                                  // Default starting target (200mL)
#define DEFAULT_PULSES_PER_PERCENT 8.0f                                           // Default pulses per 1% moisture gain
#define MINIMUM_TARGET_PULSES 20                                                  // Minimum target pulses
#define MAXIMUM_TARGET_PULSES 300                                                 // Maximum target pulses

// Anomaly Detection Settings
#define MOISTURE_SPIKE_RATE_THRESHOLD_PER_SEC 5.0f // % per second increase indicates anomaly
#define FLOW_RATE_VARIANCE_MAX 20.0f               // ±20 L/h from expected flow rate
#define TEMP_EXTREME_LOW 5.0f                      // Below 5°C unusual
#define TEMP_EXTREME_HIGH 45.0f                    // Above 45°C unusual
#define TEMPERATURE_BASELINE 20.0f                 // Baseline temperature for correction
#define TEMP_CORRECTION_FACTOR 0.01f               // 1% correction per degree C

// Dynamic Pump Control Settings
#define PUMP_RAMP_UP_TIME_MS 5000      // 5-second ramp-up time
#define TARGET_MOISTURE_GAIN_RATE 0.5f // Target 0.5% moisture gain per second
#define PUMP_ADJUSTMENT_STEP 20        // Pump duty adjustment step
#define PUMP_GAIN_RATE_TOLERANCE 0.1f  // Acceptable gain rate deviation

// Sensor Calibration Settings
#define MOISTURE_SENSOR_DRY_V 2.2f // Voltage for 0% moisture
#define MOISTURE_SENSOR_WET_V 0.85f // Voltage for 100% moisture
#define WATER_LEVEL_MIN_MBAR 15.0f // mbar for 0% water level
#define WATER_LEVEL_MAX_MBAR 60.0f // mbar for 100% water level

// Emergency Diagnostics Settings
#define EMERGENCY_MOISTURE_THRESHOLD 50.0f // Minimum 50% moisture for diagnostics
#define EMERGENCY_TEST_DURATION_MS 5000    // 5 second test cycles
#define EMERGENCY_MAX_CONSECUTIVE_FAILS 3  // Max failures before user intervention
#define EMERGENCY_TEST_MIN_FLOW_RATE 15.0f // Minimum flow rate for test to pass (L/h)
#define EMERGENCY_TEST_MAX_PRESSURE 3.0f   // Maximum pressure during test (bar)

// Delta P Level measurement - ABP Pressure Sensor Settings
#define ABP_SENSOR_MIN_RANGE_MBAR -68.94f // ABPDJJT001PDSA3 range
#define ABP_SENSOR_MAX_RANGE_MBAR 68.94f
#define ABP_SENSOR_MIN_MBAR 15.0f // Expected water level range
#define ABP_SENSOR_MAX_MBAR 60.0f

// Irrigation State Machine
typedef enum {
    IMPLUVIUM_STANDBY,    // System standby, waiting for moisture check timer (between cycles)
    IMPLUVIUM_MEASURING,  // Powering sensors, reading moisture levels
    IMPLUVIUM_WATERING,   // Active watering with event-driven monitoring
    IMPLUVIUM_STOPPING,   // Pump off, pressure equalizing, valve closing
    IMPLUVIUM_MAINTENANCE,// System maintenance tasks (NVS saves, resets, etc.)
    IMPLUVIUM_DISABLED    // System disabled (manual disable or load shedding shutdown)
} impluvium_state_t;

// Learning Algorithm Data Structures
typedef struct {
    float pulses_used_history[LEARNING_HISTORY_SIZE];               // Last 15 cycles of pulses used
    float moisture_increase_percent_history[LEARNING_HISTORY_SIZE]; // Corresponding moisture increases in percent
    bool anomaly_flags[LEARNING_HISTORY_SIZE];                      // Mark anomalous cycles
    uint8_t history_entry_count;                                    // Valid entries (0-15)
    uint8_t history_index;                                          // Current write position
    float last_temperature_correction;                              // Last used temperature factor
    float calculated_ppmp_ratio;                                    // Most recent learned pulses per moisture percent ratio
    uint32_t calculated_pump_duty_cycle;                            // Optimal pump speed for this zone
    float target_moisture_gain_rate;                                // Learned optimal moisture gain rate (%/sec)
    float confidence_level;                                         // Learning confidence (0.0-1.0)
    uint32_t successful_predictions;                                // Count of accurate predictions
    uint32_t total_predictions;                                     // Total prediction attempts
} zone_learning_t;

// Watering Queue Item
typedef struct {
    uint8_t zone_id;                 // Zone to water
    float measured_moisture_percent; // Current moisture level (%)
    float moisture_deficit_percent;  // How much below target (%)
    uint16_t target_pulses;          // Predicted pulses needed
    bool watering_completed;         // Whether this zone has been watered
    float moisture_at_start_percent; // Moisture when watering started (%)
} watering_queue_item_t;

// Anomaly Detection Types
typedef enum {
    ANOMALY_NONE = 0,
    ANOMALY_MOISTURE_SPIKE,     // Sudden moisture increase (rain/manual)
    ANOMALY_FLOW_ISSUE,         // Flow rate outside normal range
    ANOMALY_TEMPERATURE_EXTREME // Unusual temperature during watering
} anomaly_type_t;

// Anomaly Detection during watering
typedef struct {
    anomaly_type_t type;      // Type of anomaly detected
    time_t anomaly_timestamp; // When anomaly was detected
    float expected_vs_actual; // For flow anomalies: expected vs actual rate
} watering_anomaly_t;

// Irrigation Zone Configuration
typedef struct {
    uint8_t zone_id;                 // Zone number (0-4)
    gpio_num_t valve_gpio;           // Solenoid valve control pin
    uint8_t moisture_ads_device;     // ADS1115 device ID (0 or 1)
    ads111x_mux_t moisture_channel;  // Channel on ADS device
    float target_moisture_percent;   // Desired moisture level (0-100%)
    float moisture_deadband_percent; // Tolerance around target (±%)
    float last_moisture_percent;     // Last measured moisture reading (0-100%)
    bool watering_enabled;           // Zone active/disabled
    int64_t last_watered_time_ms;    // Last watering time (monotonic milliseconds)
    // NOTE: volume_used_today removed - now stored in RTC accumulator (persistent!)
    bool watering_in_progress;       // Watering in progress flag
    zone_learning_t learning;        // Learning algorithm data
} irrigation_zone_t;

// TODO: implement event logging - or maybe not?
// Storing Data Structure
typedef struct {
    float initial_moisture_percent; // Before watering (%)
    float final_moisture_percent;   // After stabilization (%)
    float volume_applied;           // Water used (mL)
    float temperature;              // Ambient temp during watering (°C)
    uint32_t stabilization_time;    // Time to reach final moisture (ms)
    time_t timestamp;               // When this occurred. TODO: Change to time_t
    uint8_t zone_id;                // Which zone this applies to
} watering_event_t;

// Emergency Diagnostics System
typedef enum {
    EMERGENCY_NONE,          // Normal operation
    EMERGENCY_TRIGGERED,     // Emergency detected, need diagnostics
    EMERGENCY_DIAGNOSING,    // Running diagnostic test cycles
    EMERGENCY_USER_REQUIRED, // Manual intervention needed
    EMERGENCY_RESOLVED       // Auto-recovered, logging event
} emergency_state_t;

typedef struct {
    emergency_state_t state;                      // Current emergency state
    uint8_t test_zone;                            // Current zone being tested (0-4)
    uint8_t eligible_zones_count;                 // Number of zones with moisture below threshold
    uint8_t eligible_zones_mask;                  // Bitmask of zones eligible for testing
    uint8_t test_cycle_count;                     // Number of test cycles completed
    uint8_t failed_zones_mask;                    // Bitmask of zones that failed tests
    int64_t diagnostic_start_time_ms;             // When diagnostics started (monotonic milliseconds)
    uint32_t test_start_time;                     // When current test cycle started
    bool initial_moisture_check_passed;           // Did initial moisture levels pass?
    float test_flow_rates[IRRIGATION_ZONE_COUNT]; // Flow rates measured per zone
    float test_pressures[IRRIGATION_ZONE_COUNT];  // Pressures measured per zone
    uint8_t consecutive_failures;                 // Count of consecutive diagnostic failures
    const char *failure_reason;                   // Human readable failure description
} emergency_diagnostics_t;

// System State Structure
typedef struct {
    impluvium_state_t state;                                     // Current system state
    uint8_t active_zone;                                         // Currently watering zone (255 = none)
    float outlet_pressure;                                       // System pressure (bar)
    float water_level;                                           // Tank level (%)
    uint32_t pump_pwm_duty;                                      // Current pump speed (0-1023)
    uint32_t watering_start_pulses;                              // Pulse count when watering started
    float current_flow_rate;                                     // Current flow rate (L/h)
    float current_moisture_gain_rate;                            // Current moisture gain rate (%/sec) during watering
    bool emergency_stop;                                         // Emergency shutdown flag
    bool sensors_powered;                                        // 3.3V sensor bus state
    bool power_save_mode;                                        // Power save mode (60min moisture check interval)
    bool load_shed_shutdown;                                     // Load shedding shutdown flag
    bool manual_watering_active;                                 // Manual watering mode active (safety override)
    uint8_t manual_water_zone;                                   // Zone being manually watered (0-4)
    uint16_t manual_water_duration_sec;                          // Manual watering duration in seconds
    uint32_t manual_water_end_time;                              // When manual watering should end (ms since boot)
    uint32_t state_start_time;                                   // When current state started
    uint32_t system_start_time;                                  // System startup timestamp
    uint32_t watering_start_time;                                // When current watering started (global, only one zone waters at a time)
    uint32_t last_moisture_check;                                // Last time all zones were checked
    float last_moisture_reading_percent;                         // For anomaly detection
    watering_anomaly_t current_anomaly;                          // Current watering anomaly status
    emergency_diagnostics_t emergency;                           // Emergency diagnostics system
    watering_queue_item_t watering_queue[IRRIGATION_ZONE_COUNT]; // Zone watering queue
    uint8_t watering_queue_size;                                 // Number of zones in queue
    uint8_t queue_index;                                         // Current position in watering queue
} irrigation_system_t;

// Task Notification Bits for Irrigation Task
#define IRRIGATION_TASK_NOTIFY_REQUEST_MOISTURE_CHECK (1 << 0) // Timer fired to check moisture
#define IRRIGATION_TASK_NOTIFY_WATERING_CUTOFF (1 << 1)        // Monitoring task requests watering cutoff
#define IRRIGATION_TASK_NOTIFY_EMERGENCY_SHUTDOWN (1 << 2)     // Monitoring task requests emergency shutdown
#define IRRIGATION_TASK_NOTIFY_MIDNIGHT_RESET (1 << 3)         // Daily reset at midnight
#define IRRIGATION_TASK_NOTIFY_MANUAL_WATER (1 << 4)           // Manual watering triggered

// Task Notification Bits for Monitoring Task
#define MONITORING_TASK_NOTIFY_START_MONITORING (1 << 0) // Start continuous monitoring during watering
#define MONITORING_TASK_NOTIFY_STOP_MONITORING (1 << 1)  // Stop monitoring when watering complete

// Global Variables
extern irrigation_zone_t irrigation_zones[IRRIGATION_ZONE_COUNT];
extern irrigation_system_t irrigation_system;

// ########################## Snapshot Structure ##########################

/**
 * @brief Comprehensive irrigation system snapshot (single mutex operation)
 *
 * Provides complete system state for HMI display and MQTT telemetry.
 * All zone data, learning algorithm state, sensors, and diagnostics in one call.
 */
typedef struct {
    // System state
    impluvium_state_t state;
    uint8_t active_zone;                 // NO_ACTIVE_ZONE_ID if none
    bool emergency_stop;
    bool power_save_mode;
    bool load_shed_shutdown;

    // Physical sensors
    float water_level_percent;

    // ADD: Daily/hourly statistics (like FLUCTUS)
    time_t current_hour_start;
    float total_water_used_hour_ml;
    float total_water_used_day_ml;
    uint8_t watering_events_hour;
    uint8_t watering_events_day;

    // Per-zone data (5 zones)
    struct {
        bool watering_enabled;
        float current_moisture_percent;
        float target_moisture_percent;
        float moisture_deadband_percent;

        // Per-zone hourly/daily statistics (from RTC accumulator)
        float volume_used_hour_ml;       // Water used this hour (mL)
        float volume_used_today_ml;      // Water used today (mL)
        uint8_t events_hour;             // Watering events this hour
        uint8_t events_day;              // Watering events today
        float avg_hourly_consumption_ml; // Average hourly consumption since midnight (mL/hr)

        time_t last_watered_time;

        // Learning algorithm data
        float calculated_ppmp_ratio;         // Pulses per moisture percent
        uint32_t calculated_pump_duty;       // Learned optimal pump speed
        float target_moisture_gain_rate;     // Learned gain rate (%/sec)
        float confidence_level;              // 0.0-1.0
        float last_temperature_correction;   // Last temperature correction factor (0.8-1.2)

        // Prediction vars - to be decided if moved somewhere else later
        uint32_t successful_predictions;
        uint32_t total_predictions;
        uint8_t history_entry_count;         // 0-15
    } zones[IRRIGATION_ZONE_COUNT];

    // Emergency diagnostics
    emergency_state_t emergency_state;
    uint8_t emergency_test_zone;
    uint8_t emergency_failed_zones_mask;
    uint8_t consecutive_failures;
    const char *emergency_failure_reason;

    // Anomaly tracking
    anomaly_type_t current_anomaly_type;
    time_t anomaly_timestamp;

    time_t snapshot_timestamp;
} impluvium_snapshot_t;

/**
 * @brief Lightweight realtime snapshot for high-frequency updates (500ms)
 *
 * Contains only fast-changing sensor values read during active watering.
 * Updated from irrigation_monitoring_task every 500ms when state == WATERING.
 * Much smaller than impluvium_snapshot_t (6 fields vs 100+ fields).
 */
typedef struct {
    // Fast-changing sensor readings (updated every 500ms)
    float water_level_percent;
    float outlet_pressure_bar;          // Current system pressure (bar)
    float current_flow_rate_lh;         // Current flow rate (L/h)
    float current_moisture_gain_rate;   // Moisture gain rate (%/sec) during watering

    // Current operation status
    uint8_t pump_duty_percent;          // 0-100%
    uint32_t pump_pwm_duty;             // Current pump PWM duty (0-1023) TODO: remove after initial diagnostics
    uint8_t active_zone;                // Which zone is being watered (255 = none)

    // System status flags
    bool sensors_powered;
    bool sensor_data_valid;
    bool pressure_alarm;          // pressure out of range
    bool flow_alarm;              // flow out of range

    // Watering queue
    uint8_t watering_queue_size;
    uint8_t queue_index;
    struct {
        uint8_t zone_id;
        float measured_moisture_percent;
        float moisture_deficit_percent;
        uint16_t target_pulses;
        bool watering_completed;
    } queue[1];

    // Metadata
    time_t snapshot_timestamp;                   // When this snapshot was taken
} impluvium_snapshot_rt_t;

// ########################## FUNCTION DECLARATIONS ################################

/**
 * @brief Write irrigation data directly to TELEMETRY cache (full snapshot)
 *
 * Called by telemetry_update_impluvium() to populate cache for HMI/MQTT.
 * Writes comprehensive snapshot with all fields (learning data, diagnostics, queue info).
 *
 * @param[out] cache Pointer to TELEMETRY's impluvium_snapshot_t buffer
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer, ESP_FAIL on mutex timeout
 */
esp_err_t impluvium_write_to_telemetry_cache(impluvium_snapshot_t *cache);

/**
 * @brief Write realtime sensor data to TELEMETRY cache (lightweight, 500ms updates)
 *
 * Called by irrigation_monitoring_task during active watering.
 * Writes only 6 fast-changing sensor fields for real-time HMI display.
 * Much faster than full snapshot (~5 fields vs 100+ fields).
 *
 * @param[out] cache Pointer to TELEMETRY's impluvium_snapshot_rt_t buffer
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer, ESP_FAIL on mutex timeout
 */
esp_err_t impluvium_write_realtime_to_telemetry_cache(impluvium_snapshot_rt_t *cache);

/**
 * @brief Initialize the IMPLUVIUM irrigation system
 *
 * Initializes the irrigation system hardware, creates required tasks,
 * and loads configuration from NVS. Must be called before any irrigation operations.
 *
 * @return ESP_OK on successful initialization
 * @return ESP_FAIL on initialization failure
 */
esp_err_t impluvium_init(void);

/**
 * @brief Enable/disable IMPLUVIUM system master switch
 *
 * When disabled, allows current watering to finish, stops moisture check timer,
 * and transitions to IMPLUVIUM_DISABLED state.
 *
 * @param enable True to enable system (resumes normal operation), false to disable
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t impluvium_set_system_enabled(bool enable);

/**
 * @brief Get current IMPLUVIUM operational state (lightweight, no snapshot fetch)
 * @return Current impluvium_state_t value
 * @note Thread-safe, uses quick mutex with 100ms timeout. Returns DISABLED on mutex timeout.
 */
impluvium_state_t impluvium_get_state(void);

/**
 * @brief Force immediate moisture check (bypasses scheduled interval)
 *
 * Triggers moisture measurement cycle immediately if system is in STANDBY state.
 *
 * @return ESP_OK if check scheduled successfully
 * @return ESP_ERR_INVALID_STATE if system not in STANDBY state (busy/disabled)
 */
esp_err_t impluvium_force_moisture_check(void);

/**
 * @brief Clear emergency stop flag (manual reset after emergency shutdown)
 *
 * Clears emergency_stop flag to allow system to resume normal operation.
 * System will wait for next moisture check cycle to resume watering.
 *
 * @return ESP_OK on success
 */
esp_err_t impluvium_clear_emergency_stop(void);

/**
 * @brief Clear diagnostic state and error history
 *
 * Resets emergency diagnostic system: clears active state, error flags,
 * consecutive failure counters, and failed zones mask.
 *
 * @return ESP_OK on success
 */
esp_err_t impluvium_clear_diagnostics(void);

/**
 * @brief Reset learning data for specific zone to defaults
 *
 * Clears history arrays, resets calculated ratios and counters to defaults.
 * Does not delete LittleFS saved data (will be overwritten on next save).
 *
 * @param zone_id Zone to reset (0-4)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if zone_id out of range
 */
esp_err_t impluvium_reset_zone_learning(uint8_t zone_id);

/**
 * @brief Reset learning data for all zones to defaults
 *
 * Clears history arrays, resets calculated ratios and counters for all zones.
 *
 * @return ESP_OK on success
 */
esp_err_t impluvium_reset_all_learning(void);

/**
 * @brief Force manual watering for specific zone (safety override)
 *
 * Forces watering for specified duration regardless of moisture levels.
 * Only respects over-pressure safety limit (hydraulic protection).
 * Duration enforced to 5-second increments.
 *
 * @param zone_id Zone to water (0-4)
 * @param duration_sec Duration in seconds (5-300, enforced to 5s increments)
 * @return ESP_OK if manual watering started successfully
 * @return ESP_ERR_INVALID_ARG if zone_id invalid or duration out of range
 * @return ESP_ERR_INVALID_STATE if system busy (already watering/measuring)
 */
esp_err_t impluvium_force_water_zone(uint8_t zone_id, uint16_t duration_sec);

/**
 * @brief Set power saving mode for IMPLUVIUM irrigation system
 * @param enable True to enable power save mode (60min moisture check interval), false for normal operation
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t impluvium_set_power_save_mode(bool enable);

/**
 * @brief Set IMPLUVIUM moisture check intervals (runtime adjustment)
 * @param optimal_min Optimal temperature interval in minutes (5-60, for temp ≥20°C)
 * @param cool_min Cool temperature interval in minutes (10-90, for 10-20°C)
 * @param power_save_min Power save mode interval in minutes (30-120)
 * @param night_hours Nighttime minimum interval in hours (1-6)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out of range
 * @note Updates configuration file, takes effect automatically on next moisture check cycle
 */
esp_err_t impluvium_set_check_intervals(uint32_t optimal_min, uint32_t cool_min,
                                        uint32_t power_save_min, uint32_t night_hours);

/**
 * @brief Set shutdown state for IMPLUVIUM irrigation system (for load shedding)
 * @param shutdown True to shutdown all irrigation operations, false to restore
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t impluvium_set_shutdown(bool shutdown);

/**
 * @brief Update zone configuration and save to LittleFS
 *
 * Public API for updating zone settings via MQTT or other external interfaces.
 * Updates zone configuration in RAM and immediately saves to LittleFS for persistence.
 * Intended to be called by TELEMETRY when MQTT config updates are received.
 *
 * @param[in] zone_id Zone identifier (0-4)
 * @param[in] target_moisture_percent Desired moisture level (0-100%)
 * @param[in] moisture_deadband_percent Tolerance around target (±%)
 * @param[in] enabled Zone enabled/disabled state
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if zone_id out of range or invalid parameters
 * @return ESP_FAIL on save error
 */
esp_err_t impluvium_update_zone_config(uint8_t zone_id,
                                       float target_moisture_percent,
                                       float moisture_deadband_percent,
                                       bool enabled);


#endif // IMPLUVIUM_H