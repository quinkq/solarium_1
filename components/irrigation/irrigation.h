#ifndef IRRIGATION_H
#define IRRIGATION_H

#include <inttypes.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_err.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

#include "ads111x.h"
#include "abp.h"

// ########################## Global settings and variables ##########################

// Safety Limits
#define MAX_WATERING_TIME_MS (1 * 60 * 1000)      // 1 minute max
#define MIN_WATERING_INTERVAL_MS (15 * 60 * 1000) // 15 minutes between
#define MAX_PRESSURE_BAR 2.5f                     // 3 bar pressure limit
#define MIN_FLOW_RATE_LH 15.0f                    // 15 L/h minimum flow
#define MIN_TEMPERATURE_WATERING 10.0f            // 10°C minimum temp
#define MIN_WATER_LEVEL_MBAR 15.0f                // 15 mbar minimum level // Use percentage??
#define MIN_WATER_LEVEL_PERCENT 5.0f              // % minimum level

#define IRRIGATION_ZONE_COUNT 5

// GPIO Pin Assignments
#define FLOW_SENSOR_GPIO GPIO_NUM_6
#define VALVE_GPIO_ZONE_1 GPIO_NUM_37
#define VALVE_GPIO_ZONE_2 GPIO_NUM_38
#define VALVE_GPIO_ZONE_3 GPIO_NUM_39
#define VALVE_GPIO_ZONE_4 GPIO_NUM_40
#define VALVE_GPIO_ZONE_5 GPIO_NUM_41
#define PUMP_PWM_GPIO GPIO_NUM_42

// ABP Pin definitions
#define ABP_SPI_HOST SPI2_HOST
#define ABP_MOSI_PIN GPIO_NUM_11 // Not used but required for SPI bus
#define ABP_SCLK_PIN GPIO_NUM_12
#define ABP_MISO_PIN GPIO_NUM_13
#define ABP_CS_PIN GPIO_NUM_14

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
#define PUMP_PWM_TIMER LEDC_TIMER_1
#define PUMP_PWM_CHANNEL LEDC_CHANNEL_2
#define PUMP_MIN_DUTY 434 // 42% minimum duty cycle (pump stall protection)
#define PUMP_MAX_DUTY 1023
#define PUMP_DEFAULT_DUTY 512 // 50% default startup duty cycle

// Flow and Monitoring
#define MONITORING_INTERVAL_MS 500               // 500ms monitoring during watering
#define FLOW_RATE_CALC_PERIOD_MS 500             // Calculate flow rate every 500ms
#define VALVE_OPEN_DELAY_MS 100                  // Wait for valve to open (ms)
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
#define MOISTURE_SENSOR_DRY_V 0.3f // Voltage for 0% moisture
#define MOISTURE_SENSOR_WET_V 3.3f // Voltage for 100% moisture
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
    IRRIGATION_IDLE,       // System idle, all sensors/pump off, checking schedule
    IRRIGATION_MEASURING,  // Powering sensors, reading moisture levels
    IRRIGATION_WATERING,   // Active watering with event-driven monitoring
    IRRIGATION_STOPPING,   // Pump off, pressure equalizing, valve closing
    IRRIGATION_MAINTENANCE // System maintenance tasks (NVS saves, resets, etc.)
} irrigation_state_t;

// Learning Algorithm Data Structures
typedef struct {
    float pulse_amount_history[LEARNING_HISTORY_SIZE];              // Last 15 cycles of pulses used
    float moisture_increase_percent_history[LEARNING_HISTORY_SIZE]; // Corresponding moisture increases in percent
    bool anomaly_flags[LEARNING_HISTORY_SIZE];                      // Mark anomalous cycles
    uint8_t history_entry_count;                                    // Valid entries (0-15)
    uint8_t history_index;                                          // Current write position
    float last_temperature_correction;                              // Last used temperature factor
    float current_pulses_per_percent;                               // Current learned ratio
    uint32_t learned_pump_duty_cycle;                               // Optimal pump speed for this zone
    float target_moisture_gain_rate;                                // Learned optimal moisture gain rate (%/sec)
    float confidence_level;                                         // Learning confidence (0.0-1.0)
    uint32_t successful_predictions;                                // Count of accurate predictions
    uint32_t total_predictions;                                     // Total prediction attempts
} zone_learning_t;

// Watering Queue Item
typedef struct {
    uint8_t zone_id;                 // Zone to water
    float current_moisture_percent;  // Current moisture level (%)
    float moisture_deficit_percent;  // How much below target (%)
    uint16_t target_pulses;          // Predicted pulses needed
    bool completed;                  // Whether this zone has been watered
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
    uint8_t moisture_ads_device;     // ADS1115 device ID (1 or 2)
    ads111x_mux_t moisture_channel;  // Channel on ADS device
    float target_moisture_percent;   // Desired moisture level (0-100%)
    float moisture_deadband_percent; // Tolerance around target (±%)
    bool enabled;                    // Zone active/disabled
    time_t last_watered_timestamp;   // Last watering timestamp
    float volume_used_today;         // Daily water usage (mL)
    bool currently_watering;         // Watering in progress flag
    uint32_t watering_start_time;    // When current watering started
    zone_learning_t learning;        // Learning algorithm data
} irrigation_zone_t;

// TODO: implement event logging
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
    time_t diagnostic_start_timestamp;            // When diagnostics started
    uint32_t test_start_time;                     // When current test cycle started
    bool initial_moisture_check_passed;           // Did initial moisture levels pass?
    float test_flow_rates[IRRIGATION_ZONE_COUNT]; // Flow rates measured per zone
    float test_pressures[IRRIGATION_ZONE_COUNT];  // Pressures measured per zone
    uint8_t consecutive_failures;                 // Count of consecutive diagnostic failures
    const char *failure_reason;                   // Human readable failure description
} emergency_diagnostics_t;

// System State Structure
typedef struct {
    irrigation_state_t state;                                    // Current system state
    uint8_t active_zone;                                         // Currently watering zone (255 = none)
    float pressure;                                              // System pressure (bar)
    float water_level;                                           // Tank level (%)
    uint32_t pump_pwm_duty;                                      // Current pump speed (0-1023)
    uint32_t watering_start_pulses;                              // Pulse count when watering started
    float current_flow_rate;                                     // Current flow rate (L/h)
    float current_moisture_gain_rate;                            // Current moisture gain rate (%/sec) during watering
    bool emergency_stop;                                         // Emergency shutdown flag
    bool sensors_powered;                                        // 3.3V sensor bus state
    bool power_save_mode;                                        // Power save mode (60min moisture check interval)
    bool load_shed_shutdown;                                     // Load shedding shutdown flag
    uint32_t state_start_time;                                   // When current state started
    uint32_t system_start_time;                                  // System startup timestamp
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

// Task Notification Bits for Monitoring Task
#define MONITORING_TASK_NOTIFY_START_MONITORING (1 << 0) // Start continuous monitoring during watering
#define MONITORING_TASK_NOTIFY_STOP_MONITORING (1 << 1)  // Stop monitoring when watering complete

// Global Variables
extern irrigation_zone_t irrigation_zones[IRRIGATION_ZONE_COUNT];
extern irrigation_system_t irrigation_system;


// ########################## FUNCTION DECLARATIONS ################################

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
 * @brief Set power saving mode for IMPLUVIUM irrigation system
 * @param enable True to enable power save mode (60min moisture check interval), false for normal operation
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t impluvium_set_power_save_mode(bool enable);

/**
 * @brief Set shutdown state for IMPLUVIUM irrigation system (for load shedding)
 * @param shutdown True to shutdown all irrigation operations, false to restore
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t impluvium_set_shutdown(bool shutdown);

// Main irrigation task to be created by app_main
void irrigation_task(void *pvParameters);


#endif // IRRIGATION_H