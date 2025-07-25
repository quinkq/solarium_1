#include <stdio.h>
#include <string.h>
#include <math.h>
// #include <inttypes.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "bmp280.h"
#include "sht4x.h"
#include "ads111x.h"
#include "as5600.h"
#include "ina219.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

#include "esp_log.h"
#include "esp_err.h"

// ########################## Global settings and variables ##########################
#define TAG "ESP_SOLARIUM_1"

// ########################## I2C Settings ##########################
#define I2C_PORT_NUM I2C_NUM_0

// ########################## Debug display settings ##########################
// Debuging display variables
#define SERIAL_DISPLAY_INTERVAL_MS 2000 // Update display every 2 seconds

SemaphoreHandle_t xDisplayDataMutex = NULL;

float latest_sht_temp = -999.9;
float latest_sht_hum = -999.9;
float latest_bmp_temp = -999.9;
float latest_bmp_press = -999.9;
float latest_bmp_hum = -999.9; // If BME280
float latest_as5600_angle = -999.9;
uint16_t latest_as5600_raw = 0;

typedef enum { SENSOR_NONE = 0, SENSOR_BME280, SENSOR_SHT4X } SensorType;

typedef struct {
    SensorType tag;
    float temperature;
    float humidity;
    float pressure;
} TaggedSensorData;

// ########################## ADS1115 Settings ##########################
#define ADS1115_DEVICE_COUNT 3  // Define the number of ADS1115 modules used
#define GAIN ADS111X_GAIN_4V096 // +-4.096V

// ADS1115 addresses (Updated mapping)
static const uint8_t ads1115_addresses[ADS1115_DEVICE_COUNT] = {
    ADS111X_ADDR_GND, // 0x48 - Photoresistor array (Dev#0)
    ADS111X_ADDR_VCC, // 0x49 - Moisture sensors (Dev#1)
    ADS111X_ADDR_SDA  // 0x4A - Zone5 moisture + pressure (Dev#2)
};

// Consolidated device structure (replaces parallel arrays)
typedef struct {
    i2c_dev_t device;             // I2C device handle
    bool initialized;             // Device initialization status
    uint32_t last_retry_time;     // Last retry attempt timestamp
    uint8_t retry_count;          // Number of retry attempts
    uint32_t next_retry_delay_ms; // Next retry delay (exponential backoff)
    const char *name;             // Device name for logging
} ads1115_device_t;

// Single array containing all device data
static ads1115_device_t ads1115_devices[ADS1115_DEVICE_COUNT] = {
    {.name = "Photoresistors", .initialized = false, .retry_count = 0, .next_retry_delay_ms = 60000},   // Dev#0
    {.name = "Moisture_Sensors", .initialized = false, .retry_count = 0, .next_retry_delay_ms = 60000}, // Dev#1
    {.name = "Mixed_Sensors", .initialized = false, .retry_count = 0, .next_retry_delay_ms = 60000}};   // Dev#2

static float gain_val; // Gain value

// ADS1115 display arrays - organized by device and channel (declared after ADS1115_DEVICE_COUNT)
static float latest_ads_voltages[ADS1115_DEVICE_COUNT][4] = {
    {-999.9, -999.9, -999.9, -999.9}, // Dev#0: Photoresistors
    {-999.9, -999.9, -999.9, -999.9}, // Dev#1: Moisture sensors
    {-999.9, -999.9, -999.9, -999.9}  // Dev#2: Zone5 moisture + pressure + spare
};

// ########################## Solar Tracking System Settings ##########################
// Photoresistor layout:
// [Ch0: Left-Top]  [Ch1: Right-Top]
// [Ch2: Left-Bot]  [Ch3: Right-Bot]

typedef struct {
    float left_top;     // Ch0
    float right_top;    // Ch1
    float left_bottom;  // Ch2
    float right_bottom; // Ch3
    float yaw_error;    // (left_avg - right_avg)
    float pitch_error;  // (top_avg - bottom_avg)
} photoresistor_readings_t;

// PWM/Servo Settings
#define SERVO_YAW_GPIO_PIN 25   // GPIO for yaw servo
#define SERVO_PITCH_GPIO_PIN 26 // GPIO for pitch servo

#define SERVO_PWM_FREQUENCY 50 // 50Hz for standard servos
#define SERVO_PWM_RESOLUTION LEDC_TIMER_14_BIT
#define SERVO_PWM_TIMER LEDC_TIMER_0
#define SERVO_YAW_CHANNEL LEDC_CHANNEL_0
#define SERVO_PITCH_CHANNEL LEDC_CHANNEL_1

// Servo position limits (in PWM duty cycle)
#define SERVO_MIN_DUTY 410     // ~1ms pulse width (0 degrees)
#define SERVO_MAX_DUTY 2048    // ~2ms pulse width (180 degrees)
#define SERVO_CENTER_DUTY 1229 // ~1.5ms pulse width (90 degrees)

// Solar tracking parameters
#define PHOTORESISTOR_THRESHOLD 0.050f // 50mV difference threshold
#define SERVO_STEP_SIZE 13             // PWM steps per adjustment
#define MAX_SERVO_ADJUSTMENT 250       // Maximum PWM adjustment per cycle
#define TRACKING_INTERVAL_MS 5000      // Check every 5 seconds

// Global servo positions
static uint32_t current_yaw_duty = SERVO_CENTER_DUTY;
static uint32_t current_pitch_duty = SERVO_CENTER_DUTY;

// ########################## Irrigation System Settings ##########################
#define IRRIGATION_ZONE_COUNT 5

// GPIO Pin Assignments (from your pinout)
#define FLOW_SENSOR_GPIO GPIO_NUM_6
#define VALVE_GPIO_ZONE_1 GPIO_NUM_37
#define VALVE_GPIO_ZONE_2 GPIO_NUM_38
#define VALVE_GPIO_ZONE_3 GPIO_NUM_39
#define VALVE_GPIO_ZONE_4 GPIO_NUM_40
#define VALVE_GPIO_ZONE_5 GPIO_NUM_41
#define PUMP_PWM_GPIO GPIO_NUM_42

// Power Management
#define MOSFET_3V3_SENSOR_BUS_CUTOFF_GPIO GPIO_NUM_16 // 3.3V sensor bus control
#define MOSFET_5V_BUS_CUTOFF_GPIO GPIO_NUM_17         // 5V bus control (placeholder)
#define MOSFET_6V2_SERVO_BUS_CUTOFF_GPIO GPIO_NUM_18  // 6.2V servo bus control
#define MOSFET_12V_FAN_PWM_GPIO GPIO_NUM_19           // 12V fan PWM (placeholder)

#define MAX_OUTPUT_CURRENT_AMPS 1.25f // Total system current limit (immediate emergency stop)
#define SENSOR_POWERUP_DELAY_MS 500   // Time for 3.3V sensors to stabilize
#define SERVO_POWERUP_DELAY_MS 100    // Time for 6.2V servo bus to stabilize

// Pump PWM Settings
#define PUMP_PWM_FREQUENCY 1000 // 1kHz for pump speed control
#define PUMP_PWM_RESOLUTION LEDC_TIMER_10_BIT
#define PUMP_PWM_TIMER LEDC_TIMER_1
#define PUMP_PWM_CHANNEL LEDC_CHANNEL_2
#define MIN_PUMP_DUTY 434     // 42% minimum duty cycle (pump stall protection)
#define DEFAULT_PUMP_DUTY 512 // 50% default startup duty cycle

// Flow and Monitoring
#define MONITORING_INTERVAL_MS 500               // 500ms monitoring during watering
#define FLOW_RATE_CALC_PERIOD_MS 500             // Calculate flow rate every 500ms
#define PUMP_FLOW_ESTABLISH_DELAY_MS 3000        // Wait for flow to establish after pump start
#define PRESSURE_EQUALIZE_DELAY_MS 1000          // Wait for pressure equalization after pump stop
#define FLOW_CALIBRATION_PULSES_PER_LITER 400.0f // Global flow sensor calibration

// Safety Limits
#define MAX_WATERING_TIME_MS (1 * 60 * 1000)      // 1 minute max
#define MIN_WATERING_INTERVAL_MS (15 * 60 * 1000) // 15 minutes between
#define MAX_PRESSURE_BAR 2.5f                     // 3 bar pressure limit
#define MIN_FLOW_RATE_LH 15.0f                    // 15 L/h minimum flow
#define MIN_TEMPERATURE_WATERING 10.0f            // 10°C minimum temp
#define MIN_WATER_LEVEL_MBAR 15.0f                // 15 mbar minimum level // Use percentage??
#define MIN_WATER_LEVEL_PERCENT 5.0f              // % minimum level

// Learning Algorithm Settings
#define MOISTURE_CHECK_INTERVAL_MS (15 * 60 * 1000)                               // 15 minutes
#define POST_WATER_CHECK_INTERVALS 3                                              // Check 3 times after watering
#define POST_WATER_CHECK_TIMES_MS {5 * 60 * 1000, 10 * 60 * 1000, 15 * 60 * 1000} // 5, 10, 15 min
#define LEARNING_HISTORY_SIZE 15                                                  // Keep last 15 cycles
#define LEARNING_MIN_CYCLES 3                                                     // Start predictions after 3 cycles
#define LEARNING_WEIGHT_RECENT 0.7f                                               // Weight recent cycles more
#define DEFAULT_TARGET_PULSES 80                                                  // Default starting target (200mL)

// Anomaly Detection Settings
#define MOISTURE_SPIKE_RATE_THRESHOLD_PER_SEC 0.5f // % per second increase indicates anomaly
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
#define MOISTURE_SENSOR_DRY_V 1.0f // Voltage for 0% moisture
#define MOISTURE_SENSOR_WET_V 3.0f // Voltage for 100% moisture
#define WATER_LEVEL_MIN_MBAR 15.0f // mbar for 0% water level
#define WATER_LEVEL_MAX_MBAR 60.0f // mbar for 100% water level

// Emergency Diagnostics Settings
#define EMERGENCY_MOISTURE_THRESHOLD 50.0f // Minimum 50% moisture for diagnostics
#define EMERGENCY_TEST_DURATION_MS 5000    // 5 second test cycles
#define EMERGENCY_MAX_CONSECUTIVE_FAILS 3  // Max failures before user intervention
#define EMERGENCY_TEST_MIN_FLOW_RATE 10.0f // Minimum flow rate for test to pass (L/h)
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
    IRRIGATION_STARTING,   // Opening valve, starting pump, waiting for flow
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

// Anomaly Detection during watering
typedef struct {
    bool moisture_spike_detected; // Sudden moisture increase (rain/manual)
    bool flow_anomaly_detected;   // Flow rate outside normal range
    bool temperature_extreme;     // Unusual temperature during watering
    uint32_t anomaly_timestamp;   // When anomaly was detected
    float expected_flow_rate;     // What we expected vs actual
} watering_anomaly_t;

// Irrigation Zone Configuration
typedef struct {
    uint8_t zone_id;                 // Zone number (0-4)
    gpio_num_t valve_gpio;           // Solenoid valve control pin
    uint8_t moisture_ads_device;     // ADS1115 device ID (0, 1 or 2)
    ads111x_mux_t moisture_channel;  // Channel on ADS device
    float target_moisture_percent;   // Desired moisture level (0-100%)
    float moisture_deadband_percent; // Tolerance around target (±%)
    bool enabled;                    // Zone active/disabled
    uint32_t last_watered_time;      // Last watering timestamp
    float volume_used_today;         // Daily water usage (mL)
    bool currently_watering;         // Watering in progress flag
    uint32_t watering_start_time;    // When current watering started
    zone_learning_t learning;        // Learning algorithm data
} irrigation_zone_t;

// Learning Algorithm Data Structure
typedef struct {
    float initial_moisture_percent; // Before watering (%)
    float final_moisture_percent;   // After stabilization (%)
    float volume_applied;           // Water used (mL)
    float temperature;              // Ambient temp during watering (°C)
    uint32_t stabilization_time;    // Time to reach final moisture (ms)
    uint32_t timestamp;             // When this occurred
    uint8_t zone_id;                // Which zone this applies to
} watering_event_t;

// System State Structure
typedef struct {
    irrigation_state_t state;            // Current system state
    bool watering_allowed;               // Global enable (temperature check)
    uint8_t active_zone;                 // Currently watering zone (255 = none)
    uint8_t queue_index;                 // Current position in watering queue
    uint32_t pump_pwm_duty;              // Current pump speed (0-1023)
    float pressure;                      // System pressure (bar)
    float water_level;                   // Tank level (%)
    uint32_t flow_pulse_count;           // Flow meter pulse count
    uint32_t watering_start_pulses;      // Pulse count when watering started
    float current_flow_rate;             // Current flow rate (L/h)
    float current_moisture_gain_rate;    // Current moisture gain rate (%/sec) during watering
    bool pump_enabled;                   // Pump safety interlock
    bool emergency_stop;                 // Emergency shutdown flag
    bool sensors_powered;                // 3.3V sensor bus state
    uint32_t state_start_time;           // When current state started
    uint32_t system_start_time;          // System startup timestamp
    uint32_t last_moisture_check;        // Last time all zones were checked
    float last_moisture_reading_percent; // For anomaly detection
    watering_anomaly_t current_anomaly;  // Current watering anomaly status
    emergency_diagnostics_t emergency;   // Emergency diagnostics system
} irrigation_system_t;

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
    uint32_t diagnostic_start_time;               // When diagnostics started
    uint32_t test_start_time;                     // When current test cycle started
    bool initial_moisture_check_passed;           // Did initial moisture levels pass?
    float test_flow_rates[IRRIGATION_ZONE_COUNT]; // Flow rates measured per zone
    float test_pressures[IRRIGATION_ZONE_COUNT];  // Pressures measured per zone
    uint8_t consecutive_failures;                 // Count of consecutive diagnostic failures
    const char *failure_reason;                   // Human readable failure description
} emergency_diagnostics_t;

// Monitoring Event Group Bits
#define MONITORING_START_BIT BIT0 // Start continuous monitoring during watering
#define MONITORING_STOP_BIT BIT1  // Stop monitoring when watering complete
#define SAFETY_CHECK_BIT BIT2     // Perform one-time safety check

// Global Variables
static irrigation_zone_t irrigation_zones[IRRIGATION_ZONE_COUNT];
static irrigation_system_t irrigation_system;
static watering_queue_item_t watering_queue[IRRIGATION_ZONE_COUNT];
static uint8_t watering_queue_size = 0; // Current number of items in the watering queue
static SemaphoreHandle_t xIrrigationMutex = NULL;
static TaskHandle_t xIrrigationMonitoringTaskHandle = NULL;
static EventGroupHandle_t xMonitoringEventGroup = NULL;

// GPIO mapping for zones
static const gpio_num_t zone_valve_gpios[IRRIGATION_ZONE_COUNT] = {VALVE_GPIO_ZONE_1,
                                                                   VALVE_GPIO_ZONE_2,
                                                                   VALVE_GPIO_ZONE_3,
                                                                   VALVE_GPIO_ZONE_4,
                                                                   VALVE_GPIO_ZONE_5};

// NVS Storage Keys
static const char *NVS_NAMESPACE = "irrigation";
static const char *NVS_ZONE_HISTORY_KEY = "zone_%d_hist";
static const char *NVS_ZONE_CONFIG_KEY = "zone_%d_cfg";

// ########################## AS5600 Settings ##########################

#define AS5600_I2C_ADDR AS5600_DEFAULT_ADDRESS // 0x36
// Global AS5600 device descriptor
static as5600_dev_t as5600_dev;

// ########################## INA219 Settings ##########################
// Define INA219 sensor types
typedef enum {
    INA219_TYPE_GENERIC, // Generic INA219 with 0.1Ω shunt resistor (3.2A max)
    INA219_TYPE_DFROBOT  // DFRobot SEN0291 with 0.01Ω shunt resistor (8A max)
} ina219_sensor_type_t;

// INA219 sensor configuration structure
typedef struct {
    ina219_t dev;              // Device handle
    ina219_sensor_type_t type; // Sensor type
    uint8_t addr;              // I2C address
    float shunt_ohms;          // Shunt resistor value in ohms
    ina219_gain_t gain;        // Gain setting
    const char *name;          // Human-readable name
    bool initialized;          // Whether initialization succeeded
} ina219_sensor_t;

// INA219 global variables
#define INA219_DEVICE_COUNT 1 // Define number of INA219 modules (planning for 2, 1 for now)
float latest_ina219_voltage[INA219_DEVICE_COUNT] = {-999.9};
float latest_ina219_current[INA219_DEVICE_COUNT] = {-999.9};
float latest_ina219_power[INA219_DEVICE_COUNT] = {-999.9};

// INA219 sensor configurations
static ina219_sensor_t ina219_devices[INA219_DEVICE_COUNT] = {
    {.type = INA219_TYPE_GENERIC,
     .addr = INA219_ADDR_GND_GND, // 0x40
     .shunt_ohms = 0.1f,          // R100
     .gain = INA219_GAIN_0_5,     // ±2A range
     .name = "GEN_3.2A",
     .initialized = false},
    // TESTING WITHOUT 2ND
    /*{
        .type = INA219_TYPE_DFROBOT,
        .addr = INA219_ADDR_GND_VS,   // 0x41
        .shunt_ohms = 0.01f,          // R010
        .gain = INA219_GAIN_0_125,    // ±8A range
        .name = "DFR_8A",
        .initialized = false
    }*/
};

// ########################## FUNCTION DECLARATIONS ################################

// ADS1115 Functions
static esp_err_t ads1115_init_all(void);
static esp_err_t ads1115_init_device(uint8_t device_id);
static esp_err_t ads1115_read_channel(uint8_t device_id, ads111x_mux_t channel, int16_t *raw, float *voltage);

// Solar Tracking Functions
static esp_err_t solar_t_read_photoresistors(photoresistor_readings_t *readings);
static esp_err_t solar_t_servo_pwm_init(void);
static esp_err_t solar_t_servo_set_position(ledc_channel_t channel, uint32_t duty_cycle);
static uint32_t solar_t_calc_servo_adjust(float error, uint32_t current_duty);
static esp_err_t solar_tracker_loop(void);

// Power Management Functions
static esp_err_t power_on_3V3_sensor_bus(void);
static esp_err_t power_off_3V3_sensor_bus(void);
static esp_err_t power_on_servo_bus(void);
static esp_err_t power_off_servo_bus(void);

// Irrigation System Functions
static esp_err_t irrigation_system_init(void);
static esp_err_t irrigation_gpio_init(void);
static esp_err_t irrigation_power_management_init(void);
static esp_err_t irrigation_pump_init(void);
static esp_err_t irrigation_flow_sensor_init(void);
static esp_err_t irrigation_load_zone_config(void);
static esp_err_t irrigation_save_learning_data(uint8_t zone_id, const watering_event_t *event);

// State Machine Functions
static esp_err_t irrigation_change_state(irrigation_state_t new_state);
static const char *irrigation_state_name(irrigation_state_t state);
static esp_err_t irrigation_state_idle(void);
static esp_err_t irrigation_state_measuring(void);
static esp_err_t irrigation_state_starting(void);
static esp_err_t irrigation_state_watering(void);
static esp_err_t irrigation_state_stopping(void);
static esp_err_t irrigation_state_maintenance(void);

static esp_err_t irrigation_read_moisture_sensor(uint8_t zone_id, float *moisture_percent);
static esp_err_t irrigation_read_pressure(float *pressure_bar);
static esp_err_t irrigation_read_level(float *water_level_percent);
static esp_err_t irrigation_emergency_stop(const char *reason);

// Pump Control Functions
static esp_err_t irrigation_set_pump_speed(uint32_t pwm_duty);
static esp_err_t irrigation_pump_ramp_up(uint8_t zone_id);
static void irrigation_pump_adaptive_control(float current_gain_rate, float target_gain_rate);

// Learning Algorithm Functions
static esp_err_t irrigation_calc_zone_watering_predictions(void);
static esp_err_t irrigation_log_zone_watering_data(uint8_t zone_id,
                                                   uint32_t pulses_used,
                                                   float moisture_increase_percent,
                                                   bool learning_valid);

// Monitoring Helper Functions
static esp_err_t irrigation_pre_check(void);
static void irrigation_calc_flow_rate(const char *task_tag,
                                      uint32_t *last_pulse_count,
                                      uint32_t *last_flow_time,
                                      uint32_t current_time);
static esp_err_t irrigation_watering_cutoffs_check(const char *task_tag, uint32_t current_time);
static bool irrigation_safety_check(uint32_t *error_count);

// Emergency Diagnostics Functions
static esp_err_t emergency_diagnostics_init(void);
static esp_err_t emergency_diagnostics_start(const char *reason);
static esp_err_t emergency_diagnostics_check_moisture_levels(void);
static esp_err_t emergency_diagnostics_test_zone(uint8_t zone_id);
static esp_err_t emergency_diagnostics_analyze_results(void);
static esp_err_t emergency_diagnostics_resolve(void);
static const char *emergency_state_name(emergency_state_t state);


// Monitoring Task Functions
static void irrigation_monitoring_task(void *pvParameters);


// ########################## FUNCTIONS ################################

/**
 * @brief Initialize all ADS1115 devices
 *
 * Attempts to initialize all ADS1115 devices defined in the system.
 * Continues operation even if some devices fail to initialize.
 *
 * @return ESP_OK if at least one device initialized successfully
 * @return ESP_FAIL if no devices could be initialized
 */
static esp_err_t ads1115_init_all(void)
{
    ESP_LOGI(TAG, "Initializing %d ADS1115 devices...", ADS1115_DEVICE_COUNT);

    gain_val = ads111x_gain_values[GAIN];

    // Initialize status tracking
    for (uint8_t device = 0; device < ADS1115_DEVICE_COUNT; device++) {
        ads1115_devices[device].initialized = false;
        ads1115_devices[device].last_retry_time = 0;
        ads1115_devices[device].retry_count = 0;
        ads1115_devices[device].next_retry_delay_ms = 60000; // Start with 1 minute
    }

    uint8_t successful_devices = 0;

    // Try to initialize each device
    for (uint8_t device = 0; device < ADS1115_DEVICE_COUNT; device++) {
        ESP_LOGI(TAG,
                 "Initializing ADS1115 device %d (%s) at 0x%02x",
                 device,
                 ads1115_devices[device].name,
                 ads1115_addresses[device]);

        esp_err_t result = ads1115_init_device(device);
        if (result == ESP_OK) {
            ads1115_devices[device].initialized = true;
            successful_devices++;
            ESP_LOGI(TAG, "ADS1115 # %d (%s) initialized successfully", device, ads1115_devices[device].name);
        } else {
            ESP_LOGW(TAG,
                     "ADS1115 # %d (%s) failed to initialize: %s",
                     device,
                     ads1115_devices[device].name,
                     esp_err_to_name(result));
            ESP_LOGW(TAG, "Will retry in %" PRIu32 " seconds", ads1115_devices[device].next_retry_delay_ms / 1000);
        }
    }

    ESP_LOGI(TAG, "ADS1115 initialization complete: %d/%d devices working", successful_devices, ADS1115_DEVICE_COUNT);

    if (successful_devices == 0) {
        ESP_LOGE(TAG, "No ADS1115 devices initialized! System functionality will be limited.");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Initialize a single ADS1115 device
 *
 * Initializes a specific ADS1115 device with default settings:
 * - Single-shot mode
 * - 32 SPS data rate
 * - Configured gain setting
 * - Performs test read to verify communication
 *
 * @param device_id Device index (0 to ADS1115_DEVICE_COUNT-1)
 * @return ESP_OK on successful initialization
 * @return ESP_ERR_INVALID_ARG if device_id is invalid
 * @return ESP_ERR_* on communication or configuration errors
 */
static esp_err_t ads1115_init_device(uint8_t device_id)
{
    if (device_id >= ADS1115_DEVICE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_dev_t *dev = &ads1115_devices[device_id].device;
    esp_err_t ret;

    // 1. Initialize descriptor
    ret = ads111x_init_desc(dev,
                            ads1115_addresses[device_id],
                            I2C_PORT_NUM,
                            CONFIG_I2CDEV_DEFAULT_SDA_PIN,
                            CONFIG_I2CDEV_DEFAULT_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed ads111x_init_desc: %s", esp_err_to_name(ret));
        return ret;
    }

    // 2. Set single-shot mode
    ret = ads111x_set_mode(dev, ADS111X_MODE_SINGLE_SHOT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed ads111x_set_mode: %s", esp_err_to_name(ret));
        ads111x_free_desc(dev); // Cleanup on failure
        return ret;
    }

    // 3. Set data rate
    ret = ads111x_set_data_rate(dev, ADS111X_DATA_RATE_32);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed ads111x_set_data_rate: %s", esp_err_to_name(ret));
        ads111x_free_desc(dev); // Cleanup on failure
        return ret;
    }

    // 4. Set gain
    ret = ads111x_set_gain(dev, GAIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed ads111x_set_gain: %s", esp_err_to_name(ret));
        ads111x_free_desc(dev); // Cleanup on failure
        return ret;
    }

    // 5. Test read to verify device is responsive
    int16_t test_raw;
    ret = ads111x_set_input_mux(dev, ADS111X_MUX_0_GND);
    if (ret == ESP_OK) {
        ret = ads111x_start_conversion(dev);
        if (ret == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(50)); // Wait for conversion
            ret = ads111x_get_value(dev, &test_raw);
        }
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed test read: %s", esp_err_to_name(ret));
        ads111x_free_desc(dev); // Cleanup on failure
        return ret;
    }

    ESP_LOGD(TAG, "Test read successful (raw: %d)", test_raw);
    return ESP_OK;
}

/**
 * @brief Read a single channel from an ADS1115 device
 *
 * Performs a single-shot conversion on the specified channel.
 * Automatically marks device as failed if communication errors occur.
 *
 * @param device_id Device index (0 to ADS1115_DEVICE_COUNT-1)
 * @param channel Input channel/MUX setting (ADS111X_MUX_*)
 * @param raw Pointer to store raw ADC value (required)
 * @param voltage Pointer to store calculated voltage (optional, can be NULL)
 * @return ESP_OK on successful read
 * @return ESP_ERR_INVALID_ARG if device_id is invalid or raw is NULL
 * @return ESP_ERR_INVALID_STATE if device is not initialized
 * @return ESP_ERR_TIMEOUT if conversion times out
 * @return ESP_ERR_* on communication errors
 */
static esp_err_t ads1115_read_channel(uint8_t device_id, ads111x_mux_t channel, int16_t *raw, float *voltage)
{
    if (!raw) {
        return ESP_ERR_INVALID_ARG;
    }

    if (device_id >= ADS1115_DEVICE_COUNT) {
        ESP_LOGE(TAG, "[Dev %d] Invalid ADS1115 device index!", device_id);
        return ESP_ERR_INVALID_ARG;
    }

    // Check if device is initialized
    if (!ads1115_devices[device_id].initialized) {
        ESP_LOGD(TAG, "[Dev %d] Device not initialized, skipping read", device_id);
        return ESP_ERR_INVALID_STATE;
    }

    i2c_dev_t *dev = &ads1115_devices[device_id].device;
    esp_err_t ret;

    // 1. Set MUX (channel selection)
    ret = ads111x_set_input_mux(dev, channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed to set MUX: %s (%d)", device_id, esp_err_to_name(ret), ret);
        // Mark device as failed for retry
        ads1115_devices[device_id].initialized = false;
        return ret;
    }

    // 2. Start Conversion
    ret = ads111x_start_conversion(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed to start conversion: %s (%d)", device_id, esp_err_to_name(ret), ret);
        ads1115_devices[device_id].initialized = false;
        return ret;
    }

    // ADD A SMALL DELAY HERE for stability
    vTaskDelay(pdMS_TO_TICKS(1));

    // 3. Wait for conversion to complete (with timeout)
    // Max conversion time for 32 SPS is ~32ms. Add some margin. Timeout after ~100ms.
    int conversion_timeout_ms = 100; // Increased timeout
    int delay_ms = 5;                // Check every 5ms
    int elapsed_ms = 0;
    bool busy = true;

    do {
        ret = ads111x_is_busy(dev, &busy);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[Dev %d] Failed to check busy status: %s (%d)", device_id, esp_err_to_name(ret), ret);
            ads1115_devices[device_id].initialized = false;
            return ret;
        }

        vTaskDelay(1);

        if (busy) {
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
            elapsed_ms += delay_ms;
            if (elapsed_ms > conversion_timeout_ms) {
                ESP_LOGE(TAG, "[Dev %d] Conversion timeout after %d ms", device_id, conversion_timeout_ms);
                ads1115_devices[device_id].initialized = false;
                return ESP_ERR_TIMEOUT;
            }
        }
    } while (busy);

    // 4. Read Value
    ret = ads111x_get_value(dev, raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed to get value: %s (%d)", device_id, esp_err_to_name(ret), ret);
        ads1115_devices[device_id].initialized = false;
        return ret;
    }
    // ESP_LOGI(TAG, "[Dev %d] Read value OK (Raw: %d).", device_index, *raw_value);

    // 5. Calculate Voltage (if requested)
    if (voltage) {
        *voltage = (*raw / (float) ADS111X_MAX_VALUE) * gain_val;
        // ESP_LOGI(TAG, "[Dev %d] Calculated voltage: %.4fV", device_index, *voltage);
    }

    return ESP_OK;
}

// ########################## PWM/Servo/Solar Functions ##########################

/**
 * @brief Initialize PWM channels for servo control
 *
 * Sets up LEDC PWM channels for yaw and pitch servo control.
 * Configures 50Hz frequency with 16-bit resolution.
 *
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
static esp_err_t solar_t_servo_pwm_init(void)
{
    // Config timer
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = SERVO_PWM_TIMER,
        .duty_resolution = SERVO_PWM_RESOLUTION,
        .freq_hz = SERVO_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Config yaw servo channel
    ledc_channel_config_t yaw_channel_config = {
        .gpio_num = SERVO_YAW_GPIO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = SERVO_YAW_CHANNEL,
        .timer_sel = SERVO_PWM_TIMER,
        .duty = SERVO_CENTER_DUTY,
        .hpoint = 0,
    };

    ret = ledc_channel_config(&yaw_channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure yaw servo channel: %s", esp_err_to_name(ret));
    }

    // Config pitch servo channel
    ledc_channel_config_t pitch_channel_config = {
        .gpio_num = SERVO_PITCH_GPIO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = SERVO_PITCH_CHANNEL,
        .timer_sel = SERVO_PWM_TIMER,
        .duty = SERVO_CENTER_DUTY,
        .hpoint = 0,
    };

    ret = ledc_channel_config(&pitch_channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure yaw servo channel: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Servo PWM initialized - Yaw: GPIO%d, Pitch: GPIO%d", SERVO_YAW_GPIO_PIN, SERVO_PITCH_GPIO_PIN);
    return ESP_OK;
}

/**
 * @brief Set servo position via PWM duty cycle
 *
 * @param channel LEDC channel (SERVO_YAW_CHANNEL or SERVO_PITCH_CHANNEL)
 * @param duty_cycle PWM duty cycle (SERVO_MIN_DUTY to SERVO_MAX_DUTY)
 * @return ESP_OK on success
 */
static esp_err_t solar_t_servo_set_position(ledc_channel_t channel, uint32_t duty_cycle)
{
    // Clamp duty cycle to valid range
    if (duty_cycle < SERVO_MIN_DUTY)
        duty_cycle = SERVO_MIN_DUTY;
    if (duty_cycle > SERVO_MAX_DUTY)
        duty_cycle = SERVO_MAX_DUTY;

    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty_cycle);
    if (ret == ESP_OK) {
        ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    }
    return ret;
}

/**
 * @brief Read all photoresistor channels and calculate tracking errors
 *
 * @param readings Pointer to structure to store readings and errors
 * @return ESP_OK if all readings successful, ESP_ERR_* otherwise
 */
static esp_err_t solar_t_read_photoresistors(photoresistor_readings_t *readings)
{
    if (!readings) {
        return ESP_ERR_INVALID_ARG;
    }
    // Check if photoresistor ADS1115 #0 is available
    if (!ads1115_devices[0].initialized) {
        ESP_LOGW(TAG, "Photoresistor ADS1115 #0 is not available, aborting photoresistor read");
        return ESP_ERR_INVALID_STATE;
    }

    int16_t raw[4];
    float voltages[4];
    esp_err_t ret;

    // Read all 4 channels
    const ads111x_mux_t channels[4] = {
        ADS111X_MUX_0_GND, // Ch0: Left-Top
        ADS111X_MUX_1_GND, // Ch1: Right-Top
        ADS111X_MUX_2_GND, // Ch2: Left-Bottom
        ADS111X_MUX_3_GND  // Ch3: Right-Bottom
    };

    for (int i = 0; i < 4; i++) {
        ret = ads1115_read_channel(0, channels[i], &raw[i], &voltages[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read photoresistor channel %d: %s", i, esp_err_to_name(ret));
            return ret;
        }

        // Small delay between readings for stability
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Store individual readings
    readings->left_top = voltages[0];
    readings->right_top = voltages[1];
    readings->left_bottom = voltages[2];
    readings->right_bottom = voltages[3];

    // Calculate average readings
    float left_avg = (readings->left_top + readings->left_bottom) / 2.0f;
    float right_avg = (readings->right_top + readings->right_bottom) / 2.0f;
    float top_avg = (readings->left_top + readings->right_top) / 2.0f;
    float bottom_avg = (readings->left_bottom + readings->right_bottom) / 2.0f;

    // Calculate tracking errors
    readings->yaw_error = left_avg - right_avg;
    readings->pitch_error = top_avg - bottom_avg;

    ESP_LOGI(TAG,
             "Photoresistors: LT=%.3f RT=%.3f LB=%.3f RB=%.3f",
             readings->left_top,
             readings->right_top,
             readings->left_bottom,
             readings->right_bottom);
    ESP_LOGI(TAG, "Tracking errors: Yaw=%.3f Pitch=%.3f", readings->yaw_error, readings->pitch_error);

    return ESP_OK;
}

/**
 * @brief Convert tracking error to servo adjustment
 *
 * @param error Tracking error in volts
 * @param current_position Current servo PWM duty cycle
 * @return New servo PWM duty cycle
 */
static uint32_t solar_t_calc_servo_adjust(float error, uint32_t current_position)
{
    // Skip adjustment if error is below threshold (deadband)
    if (fabs(error) < PHOTORESISTOR_THRESHOLD) {
        return current_position;
    }

    // Calculate proportional adjustment
    // Larger errors = larger adjustments, capped at MAX_SERVO_ADJUSTMENT
    uint32_t adjustment = (uint32_t) (error * 500.0f); // Scale factor: 500 PWM units per volt (14-bit resolution)

    // Limit adjustment magnitude
    if (adjustment > MAX_SERVO_ADJUSTMENT)
        adjustment = MAX_SERVO_ADJUSTMENT;
    if (adjustment < -MAX_SERVO_ADJUSTMENT)
        adjustment = -MAX_SERVO_ADJUSTMENT;

    // Apply adjustment to current position
    uint32_t new_position = (int32_t) current_position + adjustment;

    // Clamp to valid servo range
    if (new_position < SERVO_MIN_DUTY)
        new_position = SERVO_MIN_DUTY;
    if (new_position > SERVO_MAX_DUTY)
        new_position = SERVO_MAX_DUTY;

    return (uint32_t) new_position;
}

/**
 * @brief Perform closed-loop solar tracking with verification
 *
 * @return ESP_OK if tracking successful, ESP_ERR_* otherwise
 */
static esp_err_t solar_tracker_loop(void)
{
    const uint8_t MAX_ADJUSTMENT_ATTEMPTS = 3;
    const uint32_t SERVO_SETTLE_TIME_MS = 1500; // Wait for servos to settle
    const float FINE_THRESHOLD = 0.020f;        // 20mV should be good enough

    photoresistor_readings_t readings;

    // Initial reading
    esp_err_t ret = solar_t_read_photoresistors(&readings);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read photoresistors BEFORE adjustment: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG,
             "Starting tracking cycle - initial errors: Yaw=%.3fV, Pitch=%.3fV",
             readings.yaw_error,
             readings.pitch_error);

    // Check if we're already well-aligned
    if (fabs(readings.yaw_error) < FINE_THRESHOLD && fabs(readings.pitch_error) < FINE_THRESHOLD) {
        ESP_LOGI(TAG, "Already well-aligned, skipping tracking cycle");
        return ESP_OK;
    }

    // Adjustment loop with verification
    for (uint8_t attempt = 0; attempt < MAX_ADJUSTMENT_ATTEMPTS; attempt++) {
        bool adjustment_made = false;

        // Calculate new servo positions
        uint32_t new_yaw_duty = solar_t_calc_servo_adjust(readings.yaw_error, current_yaw_duty);
        uint32_t new_pitch_duty = solar_t_calc_servo_adjust(readings.pitch_error, current_pitch_duty);

        // Power on servos only if adjustment is needed
        if (new_yaw_duty != current_yaw_duty || new_pitch_duty != current_pitch_duty) {
            power_on_servo_bus();
        }

        // Apply yaw adjustment if needed
        if (new_yaw_duty != current_yaw_duty) {
            esp_err_t yaw_result = solar_t_servo_set_position(SERVO_YAW_CHANNEL, new_yaw_duty);
            if (yaw_result == ESP_OK) {
                ESP_LOGI(TAG,
                         "Yaw adjusted: %" PRIu32 " -> %" PRIu32 " (error: %.3fV)",
                         current_yaw_duty,
                         new_yaw_duty,
                         readings.yaw_error);
                current_yaw_duty = new_yaw_duty;
                adjustment_made = true;
            } else {
                ESP_LOGE(TAG, "Failed to set yaw servo: %s", esp_err_to_name(yaw_result));
            }
        }

        if (new_pitch_duty != current_pitch_duty) {
            esp_err_t pitch_result = solar_t_servo_set_position(SERVO_PITCH_CHANNEL, new_pitch_duty);
            if (pitch_result == ESP_OK) {
                ESP_LOGI(TAG,
                         "Pitch adjusted: %" PRIu32 " -> %" PRIu32 " (error: %.3fV)",
                         current_pitch_duty,
                         new_pitch_duty,
                         readings.pitch_error);
                current_pitch_duty = new_pitch_duty;
                adjustment_made = true;
            } else {
                ESP_LOGE(TAG, "Failed to set pitch servo: %s", esp_err_to_name(pitch_result));
            }
        }

        // Power off servos after adjustment
        if (adjustment_made) {
            // Wait for servos to settle, then power off
            vTaskDelay(pdMS_TO_TICKS(SERVO_SETTLE_TIME_MS));
            power_off_servo_bus();
        }

        // If no adjustmet was made (errors below deadband/threshold), we're done
        if (!adjustment_made) {
            ESP_LOGI(TAG, "No adjustments needed, tracking cycle completed");
            break;
        }

        // Wait for servos to settle
        ESP_LOGD(TAG, "Waiting %" PRIu32 " ms for servos to settle", SERVO_SETTLE_TIME_MS);
        vTaskDelay(pdMS_TO_TICKS(SERVO_SETTLE_TIME_MS));

        // Verify the adjustment by re-reading
        ret = solar_t_read_photoresistors(&readings);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read photoresistors after adjustment: %s", esp_err_to_name(ret));
            break;
        }

        ESP_LOGI(TAG, "Post adjustment errors: Yaw=%.3fV, Pitch=%.3fV", readings.yaw_error, readings.pitch_error);

        // Check if we're within the fine threshold
        if (fabs(readings.yaw_error) < FINE_THRESHOLD && fabs(readings.pitch_error) < FINE_THRESHOLD) {
            ESP_LOGI(TAG, "Tracking cycle completed successfully after %d attempts", attempt + 1);
            return ESP_OK;
        }

        // Log improvement (or lack thereof)
        ESP_LOGD(TAG, "Errors still above threshold, will attempt further adjustment");
    }

    // If we got here, we hit max attempts
    ESP_LOGW(TAG,
             "Reached max adjustment attempts (%d). Final errors: Yaw=%.3fV, Pitch=%.3fV",
             MAX_ADJUSTMENT_ATTEMPTS,
             readings.yaw_error,
             readings.pitch_error);

    return ESP_OK; // Not an error condition, just suboptimal alignment
}

/**
 * @brief Power on 3.3V sensor bus
 */
static esp_err_t power_on_3V3_sensor_bus(void)
{
    if (!irrigation_system.sensors_powered) {
        gpio_set_level(MOSFET_3V3_SENSOR_BUS_CUTOFF_GPIO, 1);
        irrigation_system.sensors_powered = true;
        ESP_LOGI(TAG, "3.3V sensor bus powered ON - waiting %dms for stabilization", SENSOR_POWERUP_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(SENSOR_POWERUP_DELAY_MS));
    }
    return ESP_OK;
}

/**
 * @brief Power off 3.3V sensor bus
 */
static esp_err_t power_off_3V3_sensor_bus(void)
{
    if (irrigation_system.sensors_powered) {
        gpio_set_level(MOSFET_3V3_SENSOR_BUS_CUTOFF_GPIO, 0);
        irrigation_system.sensors_powered = false;
        ESP_LOGI(TAG, "3.3V sensor bus powered OFF");
    }
    return ESP_OK;
}

/**
 * @brief Power on 6.2V servo bus
 *//*
static esp_err_t power_on_servo_bus(void)
{
    if (!irrigation_system.servos_powered) {
        gpio_set_level(MOSFET_6V2_SERVO_BUS_CUTOFF_GPIO, 1);
        irrigation_system.servos_powered = true;
        ESP_LOGI(TAG, "6.2V servo bus powered ON - waiting %dms for stabilization", SERVO_POWERUP_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(SERVO_POWERUP_DELAY_MS));
    }
    return ESP_OK;
}

/**
 * @brief Power off 6.2V servo bus
 *//*
static esp_err_t power_off_servo_bus(void)
{
    if (irrigation_system.servos_powered) {
        gpio_set_level(MOSFET_6V2_SERVO_BUS_CUTOFF_GPIO, 0);
        irrigation_system.servos_powered = false;
        ESP_LOGI(TAG, "6.2V servo bus powered OFF");
    }
    return ESP_OK;
}
*/

// ########################## Irrigation System ##########################

/**
 * @brief Initialize the complete irrigation system
 */
static esp_err_t irrigation_system_init(void)
{
    ESP_LOGI(TAG, "Initializing irrigation system...");

    // Initialize mutex and semaphores
    xIrrigationMutex = xSemaphoreCreateMutex();
    if (xIrrigationMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create irrigation mutex");
        return ESP_FAIL;
    }

    xMonitoringEventGroup = xEventGroupCreate();
    if (xMonitoringEventGroup == NULL) {
        ESP_LOGE(TAG, "Failed to create monitoring event group");
        return ESP_FAIL;
    }

    // Initialize system state (static variables are zeroinitialized)
    irrigation_system.state = IRRIGATION_IDLE;
    irrigation_system.active_zone = 255; // No active zone
    irrigation_system.queue_index = 0;   // Start at beginning of queue
    irrigation_system.sensors_powered = false;
    irrigation_system.state_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    irrigation_system.system_start_time = irrigation_system.state_start_time;

    // Initialize zone configurations
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        irrigation_zones[i].zone_id = i;
        irrigation_zones[i].valve_gpio = zone_valve_gpios[i];
        irrigation_zones[i].target_moisture_percent = 40.0f;  // 40% default target
        irrigation_zones[i].moisture_deadband_percent = 5.0f; // +/- 5% deadband
        irrigation_zones[i].enabled = true;

        // Set ADS1115 device and channel mapping
        // Dev#1: Moisture sensors Zone 1-4, Dev#2: Moisture sensor Zone 5
        if (i < 4) {
            irrigation_zones[i].moisture_ads_device = 1; // ADS1115 #1 (Moisture sensors)
            irrigation_zones[i].moisture_channel = (ads111x_mux_t) (ADS111X_MUX_0_GND + i);
        } else {
            irrigation_zones[i].moisture_ads_device = 2; // ADS1115 #2 (Zone 5 on Ch0)
            irrigation_zones[i].moisture_channel = ADS111X_MUX_0_GND;
        }

        // Initialize learning algorithm data
        // no memset - static variables are zero-initialized
        irrigation_zones[i].learning.current_pulses_per_percent = 8.0f; // Default: 8 pulses per 1% moisture
        irrigation_zones[i].learning.learned_pump_duty_cycle = DEFAULT_PUMP_DUTY;
        irrigation_zones[i].learning.target_moisture_gain_rate = TARGET_MOISTURE_GAIN_RATE;
    }
    // Initialize hardware
    esp_err_t ret = irrigation_gpio_init();
    if (ret != ESP_OK)
        return ret;

    ret = irrigation_power_management_init();
    if (ret != ESP_OK)
        return ret;

    ret = irrigation_pump_init();
    if (ret != ESP_OK)
        return ret;

    ret = irrigation_flow_sensor_init();
    if (ret != ESP_OK)
        return ret;

    // Load zone configurations from NVS
    // TODO: Implement remote zone configuration
    ret = irrigation_load_zone_config();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Could not load zone config from NVS, using defaults");
    }

    // Initialize emergency diagnostics
    emergency_diagnostics_init();

    // Create monitoring task
    BaseType_t result = xTaskCreate(irrigation_monitoring_task,
                                    "IrrigationMonitor",
                                    configMINIMAL_STACK_SIZE * 4,
                                    NULL,
                                    6, // Higher priority than main irrigation task
                                    &xIrrigationMonitoringTaskHandle);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create irrigation monitoring task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Irrigation system initialized successfully with monitoring task");
    return ESP_OK;
}

/**
 * @brief Initialize the GPIOs for valve control
 */
static esp_err_t irrigation_gpio_init(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    // Configure valve control pins
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        io_conf.pin_bit_mask = (1ULL << zone_valve_gpios[i]);
        esp_err_t ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure valve GPIO %d: %s", zone_valve_gpios[i], esp_err_to_name(ret));
            return ret;
        }

        // Ensure valve start closed
        gpio_set_level(zone_valve_gpios[i], 0);
    }

    ESP_LOGI(TAG,
             "Irrigation valve GPIOs initialized - Valve GPIO%d - GPIO%d",
             irrigation_zones[0].valve_gpio,
             irrigation_zones[IRRIGATION_ZONE_COUNT - 1].valve_gpio);
    return ESP_OK;
}

/**
 * @brief Initialize PWM for pump speed control
 */
static esp_err_t irrigation_pump_init(void)
{
    // Config timer
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = PUMP_PWM_TIMER,
        .duty_resolution = PUMP_PWM_RESOLUTION,
        .freq_hz = PUMP_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure pump PWM timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure pump PWM channel
    ledc_channel_config_t channel_config = {
        .gpio_num = PUMP_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = PUMP_PWM_CHANNEL,
        .timer_sel = PUMP_PWM_TIMER,
        .duty = 0, // Start with pump off
        .hpoint = 0,
    };

    ret = ledc_channel_config(&channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure pump PWM channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Pump PWM initialized - GPIO42, 1kHz, 10bit");
    return ESP_OK;
}

// Global pulse counter handle
static pcnt_unit_handle_t flow_pcnt_unit = NULL;

/**
 * @brief Initialize flow sensor pulse counting
 */
static esp_err_t irrigation_flow_sensor_init(void)
{
    // Create pulse counter unit configuration
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = 0,
    };

    esp_err_t ret = pcnt_new_unit(&unit_config, &flow_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create pulse counter unit: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create pulse counter channel configuration
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = FLOW_SENSOR_GPIO,
        .level_gpio_num = -1, // Not used
    };

    pcnt_channel_handle_t flow_pcnt_chan = NULL;
    ret = pcnt_new_channel(flow_pcnt_unit, &chan_config, &flow_pcnt_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create pulse counter channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set edge action (count on positive edge only)
    ret =
        pcnt_channel_set_edge_action(flow_pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set edge action: %s", esp_err_to_name(ret));
        return ret;
    }

    // Enable and start counting
    ret = pcnt_unit_enable(flow_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable pulse counter: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_clear_count(flow_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear pulse counter: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_start(flow_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start pulse counter: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Flow sensor initialized - GPIO19, pulse counting");
    return ESP_OK;
}

/**
 * @brief Initialize power management GPIOs
 */
static esp_err_t irrigation_power_management_init(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    // Configure power control pins
    const gpio_num_t power_gpios[] = {MOSFET_3V3_SENSOR_BUS_CUTOFF_GPIO,
                                      MOSFET_5V_BUS_CUTOFF_GPIO,
                                      MOSFET_6V2_SERVO_BUS_CUTOFF_GPIO,
                                      MOSFET_12V_FAN_PWM_GPIO};

    for (size_t i = 0; i < sizeof(power_gpios) / sizeof(power_gpios[0]); i++) {
        io_conf.pin_bit_mask = (1ULL << power_gpios[i]);
        esp_err_t ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure power management GPIO %d: %s", power_gpios[i], esp_err_to_name(ret));
            return ret;
        }

        // Start with all power buses OFF (LOW = OFF for MOSFET gates)
        gpio_set_level(power_gpios[i], 0);
    }

    ESP_LOGI(TAG, "Power management GPIOs initialized - All buses OFF");
    return ESP_OK;
}

/**
 * @brief Load zone configuration from NVS (placeholder)
 */
static esp_err_t irrigation_load_zone_config(void)
{
    ESP_LOGI(TAG, "Loading zone configuration from NVS (placeholder)");
    // TODO: Implement NVS loading
    return ESP_OK;
}

/**
 * @brief Save learning data to NVS (placeholder)
 */
static esp_err_t irrigation_save_learning_data(uint8_t zone_id, const watering_event_t *event)
{
    ESP_LOGI(TAG, "Saving learning data for zone %d (placeholder)", zone_id);
    // TODO: Implement NVS saving
    return ESP_OK;
}

// ########################## Irrigation System ##########################
// ---------------------- Sensor Reading Functions -----------------------

/**
 * @brief Read moisture sensor for specific zone, convert to percentage, and update debug display
 *
 * @param zone_id Zone ID (0-4)
 * @param moisture_percent Pointer to store calculated moisture percentage
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
static esp_err_t irrigation_read_moisture_sensor(uint8_t zone_id, float *moisture_percent)
{
    if (zone_id >= IRRIGATION_ZONE_COUNT || !moisture_percent) {
        return ESP_ERR_INVALID_ARG;
    }

    irrigation_zone_t *zone = &irrigation_zones[zone_id];

    // Check if ADS1115 device is available
    if (!ads1115_devices[zone->moisture_ads_device].initialized) {
        ESP_LOGW(TAG, "Zone %d moisture sensor ADS1115 #%d not available", zone_id, zone->moisture_ads_device);
        return ESP_ERR_INVALID_STATE;
    }

    // Read raw voltage from ADS1115
    int16_t raw;
    float voltage;
    esp_err_t ret = ads1115_read_channel(zone->moisture_ads_device, zone->moisture_channel, &raw, &voltage);

    if (ret == ESP_OK) {
        // Convert voltage to percentage internally
        // Clamp voltage to calibrated range
        float clamped_voltage = voltage;
        if (clamped_voltage < MOISTURE_SENSOR_DRY_V)
            clamped_voltage = MOISTURE_SENSOR_DRY_V;
        if (clamped_voltage > MOISTURE_SENSOR_WET_V)
            clamped_voltage = MOISTURE_SENSOR_WET_V;

        // Linear mapping to percentage
        *moisture_percent =
            ((clamped_voltage - MOISTURE_SENSOR_DRY_V) / (MOISTURE_SENSOR_WET_V - MOISTURE_SENSOR_DRY_V)) * 100.0f;

        // Update debug display variables with raw voltage
        if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            uint8_t dev_id = zone->moisture_ads_device;
            uint8_t ch_id = zone->moisture_channel - ADS111X_MUX_0_GND;
            latest_ads_voltages[dev_id][ch_id] = voltage;
            xSemaphoreGive(xDisplayDataMutex);
        }

        ESP_LOGD(TAG, "Zone %d moisture sensor read: %.1f%% (%.3fV)", zone_id, *moisture_percent, voltage);
    }

    return ret;
}

/**
 * @brief Read system pressure sensor, convert to bar, and update debug display
 *
 * @param pressure_bar Pointer to store calculated pressure in bar
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
static esp_err_t irrigation_read_pressure(float *pressure_bar)
{
    if (!pressure_bar) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read pump pressure from ADS1115 #2 Ch1
    if (!ads1115_devices[2].initialized) {
        ESP_LOGD(TAG, "Pressure sensor ADS1115 #2 not available");
        return ESP_ERR_INVALID_STATE;
    }

    int16_t raw;
    float voltage;
    esp_err_t ret = ads1115_read_channel(2, ADS111X_MUX_1_GND, &raw, &voltage);

    if (ret == ESP_OK) {
        // Convert voltage to pressure
        // TODO: calibration needed
        // Assuming linear conversion for now
        *pressure_bar = voltage * 3.0f; // For implementation time/tests: 3 bar per volt

        // Update debug display with raw voltage
        if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            latest_ads_voltages[2][1] = voltage; // Dev#2, Ch1
            xSemaphoreGive(xDisplayDataMutex);
        }

        ESP_LOGD(TAG, "System pressure: %.2f bar (%.3fV)", *pressure_bar, voltage);
    }

    return ret;
}

/**
 * @brief Read water level sensor and convert to percentage
 *
 * @param water_level_percent Pointer to store calculated water level percentage
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
static esp_err_t irrigation_read_level(float *water_level_percent)
{
    if (!water_level_percent) {
        return ESP_ERR_INVALID_ARG;
    }

    float water_level_pressure;

    // TODO: Add ABP SPI sensor reading here when implemented
    // irrigation_system.water_level_pressure = read_abp_sensor();
    // For now, use placeholder logic

    // This will be: esp_err_t ret = read_abp_sensor(&water_level_pressure);

    esp_err_t ret = ESP_OK; // Placeholder

    if (ret == ESP_OK) {
        // Convert pressure to percentage internally
        float mbar = water_level_pressure;

        // Clamp pressure to calibrated range
        if (mbar < WATER_LEVEL_MIN_MBAR)
            mbar = WATER_LEVEL_MIN_MBAR;
        if (mbar > WATER_LEVEL_MAX_MBAR)
            mbar = WATER_LEVEL_MAX_MBAR;

        // Linear mapping to percentage
        *water_level_percent = ((mbar - WATER_LEVEL_MIN_MBAR) / (WATER_LEVEL_MAX_MBAR - WATER_LEVEL_MIN_MBAR)) * 100.0f;

        ESP_LOGI(TAG, "Water level: %.1f%% (%.1f mbar)", *water_level_percent, mbar);
    }

    return ret;
}

// ########################## Irrigation System ##########################
// ------------------------ Pump Control Functions -----------------------

/**
 * @brief Set pump PWM duty cycle with safety limits
 */
static esp_err_t irrigation_set_pump_speed(uint32_t pwm_duty)
{
    // Enforce pump speed limits
    if (pwm_duty > 1023) {
        pwm_duty = 1023;
    }
    if (pwm_duty > 0 && pwm_duty < MIN_PUMP_DUTY) {
        ESP_LOGW(TAG, "Pump duty %" PRIu32 " below minimum %" PRIu32 ", adjusting", pwm_duty, MIN_PUMP_DUTY);
        pwm_duty = MIN_PUMP_DUTY;
    }

    irrigation_system.pump_pwm_duty = pwm_duty;

    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, PUMP_PWM_CHANNEL, pwm_duty);
    if (ret == ESP_OK) {
        ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, PUMP_PWM_CHANNEL);
    }

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Pump speed set to %" PRIu32 " /1023 (%.1f%%)", pwm_duty, (pwm_duty / 1023.0f) * 100.0f);
    }

    return ret;
}

/**
 * @brief Ramps up the pump speed over a defined period.
 *
 * Linearly increases the pump's PWM duty cycle from a minimum value
 * to the zone's learned optimal duty cycle. This gradual start-up
 * reduces mechanical stress on the pump and pipes.
 *
 * @param zone_id The ID of the zone being watered.
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
static esp_err_t irrigation_pump_ramp_up(uint8_t zone_id)
{
    ESP_LOGI(TAG, "Ramping up pump for zone %d...", zone_id);

    uint32_t start_duty = MIN_PUMP_DUTY;
    uint32_t target_duty = irrigation_zones[zone_id].learning.learned_pump_duty_cycle;
    uint32_t ramp_duration_ms = PUMP_RAMP_UP_TIME_MS;
    int steps = 50; // 50 steps for a smooth ramp
    uint32_t step_delay = ramp_duration_ms / steps;

    for (int i = 0; i <= steps; i++) {
        uint32_t duty = start_duty + ((target_duty - start_duty) * i) / steps;
        if (irrigation_set_pump_speed(duty) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set pump speed during ramp-up");
            return ESP_FAIL;
        }
        vTaskDelay(pdMS_TO_TICKS(step_delay));
    }

    ESP_LOGI(TAG, "Pump ramp-up complete for zone %d at %" PRIu32 " duty.", zone_id, target_duty);
    return ESP_OK;
}

/**
 * @brief Simple adaptive pump control based on moisture gain rate
 *
 * Adjusts pump speed up or down based on whether we're meeting the target
 * moisture gain rate. Much simpler and more appropriate than PID for soil moisture.
 *
 * @param current_gain_rate Current moisture gain rate (%/sec)
 * @param target_gain_rate Target moisture gain rate (%/sec)
 */
static void irrigation_pump_adaptive_control(float current_gain_rate, float target_gain_rate)
{
    float rate_error = target_gain_rate - current_gain_rate;

    // Only adjust if error is significant
    if (fabs(rate_error) < PUMP_GAIN_RATE_TOLERANCE) {
        return; // Close enough, no adjustment needed
    }

    uint32_t current_duty = irrigation_system.pump_pwm_duty;
    uint32_t new_duty = current_duty;

    if (rate_error > 0) {
        // Need more moisture gain - increase pump speed
        new_duty = current_duty + PUMP_ADJUSTMENT_STEP;
        ESP_LOGD(TAG, "Increasing pump speed: %.2f < %.2f %%/sec", current_gain_rate, target_gain_rate);
    } else {
        // Too much moisture gain - decrease pump speed
        new_duty = current_duty - (PUMP_ADJUSTMENT_STEP / 2); // Decrease more slowly
        ESP_LOGD(TAG, "Decreasing pump speed: %.2f > %.2f %%/sec", current_gain_rate, target_gain_rate);
    }

    irrigation_set_pump_speed(new_duty);
}

// ########################## Irrigation System ##########################
// ------------------- Learning Algorithm Functions -----------------------

/**
 * @brief Calculate watering predictions for all zones in queue using learning algorithm
 *
 * This function implements an intelligent watering prediction system that:
 * 1. Applies temperature correction based on ambient conditions
 * 2. Uses weighted historical data (recent cycles weighted more heavily)
 * 3. Calculates pulses needed based on moisture deficit and learned ratios
 * 4. Falls back to default values when insufficient learning data exists
 *
 * Algorithm Details:
 * - Temperature Correction: Linear adjustment of ±1% per degree from 20°C baseline
 * - Weighted Learning: Recent 3 cycles get 70% weight, older cycles get 30%
 * - Pulse Calculation: (moisture_deficit * 100) * learned_ratio * temp_correction
 * - Anomaly Filtering: Cycles marked as anomalous are excluded from calculations
 *
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
static esp_err_t irrigation_calc_zone_watering_predictions(void)
{
    ESP_LOGI(TAG, "Calculating watering predictions for %d zones", watering_queue_size);

    for (uint8_t i = 0; i < watering_queue_size; i++) {
        uint8_t zone_id = watering_queue[i].zone_id;
        irrigation_zone_t *zone = &irrigation_zones[zone_id];
        zone_learning_t *learning = &zone->learning;

        // Step 1: Calculate temperature correction factor
        // Formula: 1.0 + (current_temp - baseline_temp) * correction_factor
        // Example: At 30°C: 1.0 + (30-20) * 0.01 = 1.10 (10% more water)
        //          At 10°C: 1.0 + (10-20) * 0.01 = 0.90 (10% less water)
        float temp_correction = 1.0f + ((latest_sht_temp - TEMPERATURE_BASELINE) * TEMP_CORRECTION_FACTOR);
        learning->last_temperature_correction = temp_correction;

        ESP_LOGD(TAG, "Zone %d: Temp %.1f°C, correction factor %.2f", zone_id, latest_sht_temp, temp_correction);

        // Step 2: Check if we have sufficient learning data
        if (learning->history_entry_count >= LEARNING_MIN_CYCLES) {
            // Step 3: Calculate weighted average of historical pulse/moisture ratios
            float weighted_ratio = 0.0f;
            float total_weight = 0.0f;
            uint8_t valid_cycles = 0;

            for (uint8_t h = 0; h < learning->history_entry_count; h++) {
                // Skip anomalous cycles (rain, manual watering, etc.)
                if (!learning->anomaly_flags[h] && learning->moisture_increase_percent_history[h] > 0) {
                    // Recent cycles (last 3) get higher weight
                    float weight = (h >= learning->history_entry_count - 3) ? LEARNING_WEIGHT_RECENT
                                                                            : (1.0f - LEARNING_WEIGHT_RECENT);

                    // Calculate pulses per moisture percent for this cycle
                    float ratio = learning->pulse_amount_history[h] / learning->moisture_increase_percent_history[h];

                    weighted_ratio += ratio * weight;
                    total_weight += weight;
                    valid_cycles++;

                    ESP_LOGD(TAG,
                             "Zone %d cycle %d: %d pulses, %.2f%% increase, ratio %.1f, weight %.2f",
                             zone_id,
                             h,
                             (int) learning->pulse_amount_history[h],
                             learning->moisture_increase_percent_history[h],
                             ratio,
                             weight);
                }
            }

            // Step 4: Apply learned ratio if we have valid data
            if (total_weight > 0 && valid_cycles >= 2) {
                learning->current_pulses_per_percent = weighted_ratio / total_weight;

                // Calculate predicted pulses needed for current moisture deficit
                // Formula: deficit_percent * pulses_per_percent * temperature_correction
                float target_pulses =
                    watering_queue[i].moisture_deficit_percent * learning->current_pulses_per_percent * temp_correction;

                // Sanity check: limit to reasonable range
                if (target_pulses < 20)
                    target_pulses = 20; // Minimum 20 pulses
                if (target_pulses > 300)
                    target_pulses = 300; // Maximum 300 pulses (0.75 l)

                watering_queue[i].target_pulses = (uint16_t) target_pulses;

                ESP_LOGI(TAG,
                         "Zone %d: Learned %.1f pulses/%, predicted %d pulses (%.1f°C correction)",
                         zone_id,
                         learning->current_pulses_per_percent,
                         watering_queue[i].target_pulses,
                         temp_correction);
            } else {
                // Not enough valid learning data
                watering_queue[i].target_pulses = DEFAULT_TARGET_PULSES;
                ESP_LOGW(TAG,
                         "Zone %d: Insufficient valid learning data (%d cycles), using default %d pulses",
                         zone_id,
                         valid_cycles,
                         DEFAULT_TARGET_PULSES);
            }
        } else {
            // Step 5: Not enough learning cycles - use default
            watering_queue[i].target_pulses = DEFAULT_TARGET_PULSES;
            ESP_LOGI(TAG,
                     "Zone %d: Learning phase (%d/%d cycles), using default %d pulses",
                     zone_id,
                     learning->history_entry_count,
                     LEARNING_MIN_CYCLES,
                     DEFAULT_TARGET_PULSES);
        }

        ESP_LOGI(TAG,
                 "Queue[%d]: Zone %d, deficit %.2f%%, target %d pulses",
                 i,
                 zone_id,
                 watering_queue[i].moisture_deficit_percent,
                 watering_queue[i].target_pulses);
    }

    return ESP_OK;
}

/**
 * @brief Update learning algorithm with post-watering data
 *
 * This function records the results of a watering cycle for future learning:
 * 1. Stores pulses used and moisture increase achieved
 * 2. Marks anomalous cycles (rain, manual watering, sensor errors)
 * 3. Maintains circular buffer of historical data
 * 4. Updates learning statistics
 *
 * Anomaly Detection Criteria:
 * - Excessive moisture increase (>0.3V indicates rain/manual watering)
 * - Extreme temperature conditions (<5°C or >45°C)
 * - Flow anomalies detected during watering
 * - Other system-detected anomalies
 *
 * @param zone_id Zone that was watered (0-4)
 * @param pulses_used Number of flow sensor pulses during watering
 * @param moisture_increase_percent Percentage increase observed in moisture sensor (final - initial)
 * @param learning_valid Whether this cycle should be used for learning
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid parameters
 */
static esp_err_t irrigation_log_zone_watering_data(uint8_t zone_id,
                                                   uint32_t pulses_used,
                                                   float moisture_increase_percent,
                                                   bool learning_valid)
{
    if (zone_id >= IRRIGATION_ZONE_COUNT) {
        ESP_LOGE(TAG, "Invalid zone_id %d for learning update", zone_id);
        return ESP_ERR_INVALID_ARG;
    }

    irrigation_zone_t *zone = &irrigation_zones[zone_id];
    zone_learning_t *learning = &zone->learning;

    // Get current write position in circular buffer
    uint8_t index = learning->history_index;

    // Store the learning data
    learning->pulse_amount_history[index] = pulses_used;
    learning->moisture_increase_percent_history[index] = moisture_increase_percent;
    learning->anomaly_flags[index] = !learning_valid; // Invert: true = anomaly

    // Advance circular buffer index
    learning->history_index = (learning->history_index + 1) % LEARNING_HISTORY_SIZE;

    // Update count (saturates at LEARNING_HISTORY_SIZE)
    if (learning->history_entry_count < LEARNING_HISTORY_SIZE) {
        learning->history_entry_count++;
    }

    // Calculate basic statistics for this cycle
    float efficiency = (moisture_increase_percent > 0) ? (pulses_used / moisture_increase_percent) : 0;

    ESP_LOGI(TAG,
             "Zone %d learning update: %lu pulses → %.2f%% increase (%.1f pulses/%%), %s",
             zone_id,
             pulses_used,
             moisture_increase_percent,
             efficiency,
             learning_valid ? "valid" : "anomaly");

    // Log learning progress
    if (learning_valid && learning->history_entry_count >= LEARNING_MIN_CYCLES) {
        // Simple moving average to update the learned pump duty cycle for the next run.
        // We use the final duty cycle from the completed session as the basis for the next.
        learning->learned_pump_duty_cycle =
            (learning->learned_pump_duty_cycle * (LEARNING_HISTORY_SIZE - 1) + irrigation_system.pump_pwm_duty) /
            LEARNING_HISTORY_SIZE;

        ESP_LOGI(TAG,
                 "Zone %d: Updated learned pump duty cycle to %" PRIu32,
                 zone_id,
                 learning->learned_pump_duty_cycle);
    }

    if (learning->history_entry_count >= LEARNING_MIN_CYCLES) {
        uint8_t valid_entries = 0;
        for (uint8_t i = 0; i < learning->history_entry_count; i++) {
            if (!learning->anomaly_flags[i])
                valid_entries++;
        }
        ESP_LOGI(TAG,
                 "Zone %d learning: %d total cycles, %d valid for predictions",
                 zone_id,
                 learning->history_entry_count,
                 valid_entries);
    }

    return ESP_OK;
}

// ########################## Irrigation System ##########################
// ---------------------- State Machine Functions ------------------------

/**
 * @brief Change irrigation system state with logging
 */
static esp_err_t irrigation_change_state(irrigation_state_t new_state)
{
    if (irrigation_system.state != new_state) {
        ESP_LOGI(TAG,
                 "State change: %s -> %s",
                 irrigation_state_name(irrigation_system.state),
                 irrigation_state_name(new_state));
        irrigation_system.state = new_state;
        irrigation_system.state_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
    return ESP_OK;
}

/**
 * @brief Get human-readable state name
 */
static const char *irrigation_state_name(irrigation_state_t state)
{
    switch (state) {
        case IRRIGATION_IDLE:
            return "IDLE";
        case IRRIGATION_MEASURING:
            return "MEASURING";
        case IRRIGATION_STARTING:
            return "STARTING";
        case IRRIGATION_WATERING:
            return "WATERING";
        case IRRIGATION_STOPPING:
            return "STOPPING";
        case IRRIGATION_MAINTENANCE:
            return "MAINTENANCE";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief Get human-readable emergency state name
 */
static const char *emergency_state_name(emergency_state_t state)
{
    switch (state) {
        case EMERGENCY_NONE:
            return "NONE";
        case EMERGENCY_TRIGGERED:
            return "TRIGGERED";
        case EMERGENCY_DIAGNOSING:
            return "DIAGNOSING";
        case EMERGENCY_USER_REQUIRED:
            return "USER_REQUIRED";
        case EMERGENCY_RESOLVED:
            return "RESOLVED";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief IDLE state - check schedule and look for zones needing water
 */
static esp_err_t irrigation_state_idle(void)
{
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Check if it's time for moisture monitoring
    if (current_time - irrigation_system.last_moisture_check >= MOISTURE_CHECK_INTERVAL_MS) {
        irrigation_change_state(IRRIGATION_MEASURING);
        return ESP_OK;
    }

    return ESP_OK;
}

/**
 * @brief MEASURING state - power sensors and check moisture levels
 */
static esp_err_t irrigation_state_measuring(void)
{
    // Power on sensors
    power_on_3V3_sensor_bus();

    // Trigger monitoring task for sensor reading assistance
    xEventGroupSetBits(xMonitoringEventGroup, SAFETY_CHECK_BIT);

    // Check each zone for watering needs
    for (uint8_t zone_id = 0; zone_id < IRRIGATION_ZONE_COUNT; zone_id++) {
        if (!irrigation_zones[zone_id].enabled)
            continue;

        float moisture_percent;
        if (irrigation_read_moisture_sensor(zone_id, &moisture_percent) == ESP_OK) {
            irrigation_zone_t *zone = &irrigation_zones[zone_id];
            float moisture_deficit_percent = zone->target_moisture_percent - moisture_percent;

            // Check if watering is needed
            if (moisture_deficit_percent > zone->moisture_deadband_percent) {
                // Check minimum interval since last watering
                uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if (current_time - zone->last_watered_time >= MIN_WATERING_INTERVAL_MS) {
                    ESP_LOGI(TAG,
                             "Zone %d needs water: %.1f%% < %.1f%% (deficit: %.1f%%)",
                             zone_id,
                             moisture_percent,
                             zone->target_moisture_percent,
                             moisture_deficit_percent);

                    // Add to watering queue
                    if (watering_queue_size < IRRIGATION_ZONE_COUNT) {
                        watering_queue[watering_queue_size].zone_id = zone_id;
                        watering_queue[watering_queue_size].current_moisture_percent = moisture_percent;
                        watering_queue[watering_queue_size].moisture_deficit_percent = moisture_deficit_percent;
                        watering_queue[watering_queue_size].completed = false;
                        watering_queue_size++;
                    }
                }
            }
        }
    }

    // Process the watering queue
    if (watering_queue_size > 0) {
        // Sort queue by moisture deficit (highest priority first) - Insertion Sort
        for (uint8_t i = 1; i < watering_queue_size; i++) {
            watering_queue_item_t key = watering_queue[i];
            int8_t j = i - 1;

            // Move elements with lower deficit one position ahead
            while (j >= 0 && watering_queue[j].moisture_deficit_percent < key.moisture_deficit_percent) {
                watering_queue[j + 1] = watering_queue[j];
                j--;
            }
            watering_queue[j + 1] = key;
        }

        // Calculate predicted pulses for each zone using learning algorithm
        irrigation_calc_zone_watering_predictions();

        // Start watering queue - keep sensors powered
        irrigation_system.queue_index = 0;
        ESP_LOGI(TAG, "Starting watering queue with %d zones", watering_queue_size);
        irrigation_change_state(IRRIGATION_STARTING);
    } else {
        // No zones need watering - power off sensors and return to idle
        power_off_3V3_sensor_bus();
        irrigation_system.last_moisture_check = xTaskGetTickCount() * portTICK_PERIOD_MS;
        irrigation_change_state(IRRIGATION_IDLE);
    }

    return ESP_OK;
}

/**
 * @brief STARTING state - perform safety checks, open valve, start pump
 */
static esp_err_t irrigation_state_starting(void)
{
    // Get current zone from queue
    if (irrigation_system.queue_index >= watering_queue_size) {
        ESP_LOGE(TAG, "Invalid queue index in STARTING state");
        irrigation_change_state(IRRIGATION_STOPPING);
        return ESP_FAIL;
    }

    uint8_t zone_id = watering_queue[irrigation_system.queue_index].zone_id;

    // Trigger monitoring task for pre-start safety checks
    xEventGroupSetBits(xMonitoringEventGroup, SAFETY_CHECK_BIT);
    vTaskDelay(pdMS_TO_TICKS(50)); // Give monitoring task time to perform checks

    // Check if safety checks passed
    if (!irrigation_system.watering_allowed) {
        ESP_LOGE(TAG, "Pre-start safety check failed for zone %d", zone_id);
        power_off_3V3_sensor_bus();
        irrigation_change_state(IRRIGATION_MAINTENANCE);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Starting watering zone %d", zone_id);
    irrigation_zone_t *zone = &irrigation_zones[zone_id];

    // Open valve
    gpio_set_level(zone->valve_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for valve to open

    // Ramp up pump speed
    if (irrigation_pump_ramp_up(zone_id) != ESP_OK) {
        ESP_LOGE(TAG, "Pump ramp-up failed for zone %d", zone_id);
        gpio_set_level(zone->valve_gpio, 0);
        power_off_3V3_sensor_bus();
        irrigation_change_state(IRRIGATION_MAINTENANCE);
        return ESP_FAIL;
    }

    // Reset flow sensor counter and update system state
    pcnt_unit_clear_count(flow_pcnt_unit);
    pcnt_unit_get_count(flow_pcnt_unit, (int *) &irrigation_system.watering_start_pulses);
    irrigation_system.active_zone = zone_id;
    zone->currently_watering = true;
    zone->last_watered_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Store starting moisture for learning algorithm
    watering_queue[irrigation_system.queue_index].moisture_at_start_percent =
        watering_queue[irrigation_system.queue_index].current_moisture_percent;

    // Wait for flow to establish before checking flow rate
    ESP_LOGI(TAG, "Waiting %dms for flow to establish...", PUMP_FLOW_ESTABLISH_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(PUMP_FLOW_ESTABLISH_DELAY_MS));

    // Trigger monitoring task to check flow rate
    xEventGroupSetBits(xMonitoringEventGroup, SAFETY_CHECK_BIT);
    vTaskDelay(pdMS_TO_TICKS(50)); // Give monitoring task time to update flow rate

    // Check if flow is established
    if (irrigation_system.current_flow_rate < MIN_FLOW_RATE_LH) {
        ESP_LOGE(TAG,
                 "Flow rate too low after pump start: %.1f L/h < %.1f L/h",
                 irrigation_system.current_flow_rate,
                 MIN_FLOW_RATE_LH);
        irrigation_change_state(IRRIGATION_STOPPING);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Zone %d watering started successfully", zone_id);

    // Start continuous monitoring for real-time watering control
    xEventGroupSetBits(xMonitoringEventGroup, MONITORING_START_BIT);

    irrigation_change_state(IRRIGATION_WATERING);
    return ESP_OK;
}

/**
 * @brief WATERING state - simplified state management with event-driven monitoring
 *
 * This state now focuses on high-level logic and time-based cutoffs only.
 * All real-time monitoring (moisture, flow, safety) is handled by the monitoring task.
 */
static esp_err_t irrigation_state_watering(void)
{
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    irrigation_zone_t *active_zone = &irrigation_zones[irrigation_system.active_zone];

    // Check maximum watering time (safety backup)
    if (current_time - active_zone->watering_start_time > MAX_WATERING_TIME_MS) {
        ESP_LOGW(TAG, "Zone %d max watering time reached - stopping", irrigation_system.active_zone);
        xEventGroupSetBits(xMonitoringEventGroup, MONITORING_STOP_BIT);
        irrigation_change_state(IRRIGATION_STOPPING);
        return ESP_OK;
    }

    // The monitoring task handles all real-time cutoffs:
    // - Target pulse count reached
    // - Safety margin (moisture level) reached
    // - Anomaly detection (rain, manual watering)
    // - Safety monitoring (pressure, flow, temperature)

    // This state just ensures monitoring continues and handles emergencies
    ESP_LOGD(TAG,
             "Zone %d watering in progress - monitoring task handling real-time control",
             irrigation_system.active_zone);

    return ESP_OK;
}

/**
 * @brief STOPPING state - stop pump, update learning, process next zone in queue
 */
static esp_err_t irrigation_state_stopping(void)
{
    uint8_t active_zone = irrigation_system.active_zone;

    // Stop continuous monitoring
    xEventGroupSetBits(xMonitoringEventGroup, MONITORING_STOP_BIT);

    // Stop pump
    irrigation_set_pump_speed(0);
    ESP_LOGI(TAG, "Pump stopped - waiting %dms for pressure equalization", PRESSURE_EQUALIZE_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(PRESSURE_EQUALIZE_DELAY_MS));

    // Close current zone valve
    if (active_zone < IRRIGATION_ZONE_COUNT) {
        gpio_set_level(irrigation_zones[active_zone].valve_gpio, 0);
        irrigation_zones[active_zone].currently_watering = false;
    } else {
        ESP_LOGE(TAG, "Invalid active zone in STOPPING state: %d", active_zone);
    }

    // Calculate volume used and update learning algorithm
    if (active_zone < IRRIGATION_ZONE_COUNT && irrigation_system.queue_index < watering_queue_size) {
        int total_pulses;
        pcnt_unit_get_count(flow_pcnt_unit, &total_pulses);
        uint32_t pulses_used = total_pulses - irrigation_system.watering_start_pulses;
        float volume_ml = (pulses_used / FLOW_CALIBRATION_PULSES_PER_LITER) * 1000.0f;
        irrigation_zones[active_zone].volume_used_today += volume_ml;

        // Read final moisture level for learning
        float final_moisture_percent;
        bool learning_valid = true;
        if (irrigation_read_moisture_sensor(active_zone, &final_moisture_percent) == ESP_OK) {
            watering_queue_item_t *queue_item = &watering_queue[irrigation_system.queue_index];
            float moisture_increase_percent = final_moisture_percent - queue_item->moisture_at_start_percent;

            // Check for anomalies that would invalidate learning
            if (irrigation_system.current_anomaly.moisture_spike_detected ||
                irrigation_system.current_anomaly.flow_anomaly_detected ||
                irrigation_system.current_anomaly.temperature_extreme || latest_sht_temp < TEMP_EXTREME_LOW ||
                latest_sht_temp > TEMP_EXTREME_HIGH) {
                learning_valid = false;
                ESP_LOGW(TAG, "Zone %d: Anomaly detected, not using for learning", active_zone);
            }

            // Update learning algorithm with results
            irrigation_log_zone_watering_data(active_zone, pulses_used, moisture_increase_percent, learning_valid);

            ESP_LOGI(TAG,
                     "Zone %d: Used %lu pulses, %.1fmL, moisture %.1f%%->%.1f%% (+%.1f%%)",
                     active_zone,
                     pulses_used,
                     volume_ml,
                     queue_item->moisture_at_start_percent,
                     final_moisture_percent,
                     moisture_increase_percent);
        }

        // Mark current queue item as completed
        watering_queue[irrigation_system.queue_index].completed = true;
    }

    // Reset anomaly detection for next zone
    memset(&irrigation_system.current_anomaly, 0, sizeof(watering_anomaly_t));

    // Move to next zone in queue
    irrigation_system.queue_index++;
    irrigation_system.active_zone = 255; // Reset active zone

    if (irrigation_system.queue_index < watering_queue_size) {
        // More zones to water - continue with next zone
        ESP_LOGI(TAG, "Moving to next zone in queue: %d/%d", irrigation_system.queue_index + 1, watering_queue_size);
        irrigation_change_state(IRRIGATION_STARTING);
    } else {
        // All zones completed - finish session
        ESP_LOGI(TAG, "All %d zones in queue completed", watering_queue_size);

        // Power off sensors now that all zones are done
        power_off_3V3_sensor_bus();

        // Reset queue and state
        irrigation_system.queue_index = 0;
        watering_queue_size = 0;
        irrigation_system.last_moisture_check = xTaskGetTickCount() * portTICK_PERIOD_MS;

        irrigation_change_state(IRRIGATION_MAINTENANCE);
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
static esp_err_t irrigation_state_maintenance(void)
{
    // Check if we entered MAINTENANCE because of an emergency
    if (irrigation_system.emergency_stop) {
        // This is the first time we're handling the emergency after shutdown
        ESP_LOGI(TAG, "MAINTENANCE state entered due to emergency, starting diagnostics.");
        emergency_diagnostics_start(irrigation_system.emergency.failure_reason);
        // Reset flag, diagnostics system is now in control.
        // This prevents re-triggering on the next main loop cycle.
        irrigation_system.emergency_stop = false;
    }

    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Handle emergency diagnostics if active
    if (irrigation_system.emergency.state != EMERGENCY_NONE) {
        ESP_LOGI(TAG,
                 "Processing emergency diagnostics (state: %s)",
                 emergency_state_name(irrigation_system.emergency.state));

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
                    // After test, increment to ensure we check the next zone in the following cycle
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
                // Power off sensors to conserve power while waiting for user
                power_off_3V3_sensor_bus();
                // Stay in this state until user manually resets system
                return ESP_OK;

            case EMERGENCY_RESOLVED:
                emergency_diagnostics_resolve();
                break;

            default:
                break;
        }

        // Don't proceed to normal maintenance while handling emergency
        return ESP_OK;
    }

    // Normal maintenance tasks
    // TODO: Add maintenance tasks here
    // - Reset daily volume counters if new day
    // - Save learning data to NVS periodically
    // - Perform system health diagnostics?
    // - Optimize learning algorithm parameters?

    // Brief pause before returning to idle (5 seconds)
    if (current_time - irrigation_system.state_start_time >= 5000) {
        ESP_LOGI(TAG, "Maintenance tasks completed, returning to IDLE");
        irrigation_change_state(IRRIGATION_IDLE);
    }

    return ESP_OK;
}

// ########################## Irrigation System ##########################
// -------------------- Safety Monitoring Functions ----------------------

/**
 * @brief Perform one-time safety checks
 */
static esp_err_t irrigation_pre_check(void)
{
    // Check temperature
    if (latest_sht_temp < MIN_TEMPERATURE_WATERING) {
        ESP_LOGW(TAG,
                 "Pre-check failed: Temperature %.1f°C is below minimum %.1f°C",
                 latest_sht_temp,
                 MIN_TEMPERATURE_WATERING);
        return ESP_FAIL;
    }

    // Read current water level in tank
    float water_level_percent;
    irrigation_read_level(&water_level_percent);
    irrigation_system.water_level = water_level_percent;
    if (water_level_percent < MIN_WATER_LEVEL_PERCENT) {
        ESP_LOGW(TAG,
                 "Pre-check failed: Water level %.1f%% is below minimum %.1f%%",
                 water_level_percent,
                 MIN_WATER_LEVEL_PERCENT);
        return ESP_FAIL;
    }

    // Read current pressure
    if (irrigation_read_pressure(&irrigation_system.pressure) != ESP_OK) {
        ESP_LOGW(TAG, "Pre-check failed: Could not read system pressure.");
        return ESP_FAIL;
    }

    if (irrigation_system.pressure > MAX_PRESSURE_BAR) {
        ESP_LOGE(TAG,
                 "Pre-check failed: Pressure %.2f bar is above maximum %.2f bar",
                 irrigation_system.pressure,
                 MAX_PRESSURE_BAR);
        irrigation_emergency_stop("System pressure too high");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG,
             "Safety pre-check complete: temp %.1f°C, water level %.1f %%, pressure %.2f bar, allowed YES",
             latest_sht_temp,
             water_level_percent,
             irrigation_system.pressure);

    return ESP_OK; // All checks passed
}

/**
 * @brief Update flow rate calculation
 */
static void irrigation_calc_flow_rate(const char *task_tag,
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
static esp_err_t irrigation_watering_cutoffs_check(const char *task_tag, uint32_t current_time)
{
    if (irrigation_system.active_zone >= IRRIGATION_ZONE_COUNT ||
        irrigation_system.queue_index >= watering_queue_size) {
        return ESP_FAIL;
    }

    irrigation_zone_t *active_zone = &irrigation_zones[irrigation_system.active_zone];
    watering_queue_item_t *queue_item = &watering_queue[irrigation_system.queue_index];

    // Check if target pulses reached
    int current_pulses;
    if (pcnt_unit_get_count(flow_pcnt_unit, &current_pulses) == ESP_OK) {
        uint32_t pulses_used = current_pulses - irrigation_system.watering_start_pulses;

        if (pulses_used >= queue_item->target_pulses) {
            ESP_LOGI(task_tag,
                     "Zone %d target pulses reached: %lu >= %d",
                     irrigation_system.active_zone,
                     pulses_used,
                     queue_item->target_pulses);
            irrigation_change_state(IRRIGATION_STOPPING);
            return ESP_OK;
        }
    }

    // Check moisture level and safety margin
    float current_moisture_percent;
    if (irrigation_read_moisture_sensor(irrigation_system.active_zone, &current_moisture_percent) == ESP_OK) {
        // Calculate, store, and check the current moisture gain rate
        float moisture_increase = current_moisture_percent - queue_item->moisture_at_start_percent;
        uint32_t time_since_start_ms = current_time - active_zone->watering_start_time;

        if (time_since_start_ms > 1000) { // Calculate after 1 second to get a meaningful rate
            irrigation_system.current_moisture_gain_rate = (moisture_increase / (time_since_start_ms / 1000.0f));

            // Check for anomalous spike after 2 seconds
            if (time_since_start_ms > 2000 &&
                irrigation_system.current_moisture_gain_rate > MOISTURE_SPIKE_RATE_THRESHOLD_PER_SEC) {
                ESP_LOGW(task_tag,
                         "Zone %d moisture spike detected: %.2f%%/sec increase - possible rain/manual watering",
                         irrigation_system.active_zone,
                         irrigation_system.current_moisture_gain_rate);
                irrigation_system.current_anomaly.moisture_spike_detected = true;
                irrigation_system.current_anomaly.anomaly_timestamp = current_time;
                irrigation_change_state(IRRIGATION_STOPPING);
                return ESP_OK; // Stop watering
            }
        } else {
            irrigation_system.current_moisture_gain_rate = 0.0f;
        }

        // Check safety margin (target - 2%)
        float safety_target = active_zone->target_moisture_percent - 2.0f;
        if (current_moisture_percent >= safety_target) {
            ESP_LOGI(task_tag,
                     "Zone %d safety margin reached: %.1f%% >= %.1f%%",
                     irrigation_system.active_zone,
                     current_moisture_percent,
                     safety_target);
            irrigation_change_state(IRRIGATION_STOPPING);
            return ESP_OK;
        }

        // Store for anomaly detection
        irrigation_system.last_moisture_reading_percent = current_moisture_percent;
    }

    return ESP_FAIL; // Continue watering
}

/**
 * @brief Perform comprehensive safety monitoring while watering
 * @return true if safe to continue, false if emergency stop was triggered
 */
static bool irrigation_safety_check(uint32_t *error_count)
{
    // Allow 4 consecutive errors before emergency stop (immediate if current exceeds MAX_OUTPUT_CURRENT_AMPS)
    const uint32_t MAX_ERROR_COUNT = 4;
    const char *failure_reason = NULL;

    // Read pressure sensors
    if (irrigation_read_pressure(&irrigation_system.pressure) != ESP_OK) {
        ESP_LOGW(TAG, "Safety check: Failed to read pressure sensor.");
        failure_reason = "Pressure sensor read failed";
    }

    // Check output current (emergency stop)
    if (ina219_devices[0].initialized) {
        float current_amps;
        if (ina219_get_current(&ina219_devices[0].dev, &current_amps) == ESP_OK) {
            if (current_amps > MAX_OUTPUT_CURRENT_AMPS) {
                ESP_LOGE(TAG,
                         "CRITICAL: Current %.2fA > %.2fA - EMERGENCY STOP",
                         current_amps,
                         MAX_OUTPUT_CURRENT_AMPS);
                irrigation_emergency_stop("Overcurrent protection triggered.");
                return false;
            }
        }
    }

    // Check safety parameters
    if (failure_reason == NULL) {
        if (irrigation_system.pressure > MAX_PRESSURE_BAR) {
            ESP_LOGW(TAG, "Pressure too high: %.2f > %.2f bar", irrigation_system.pressure, MAX_PRESSURE_BAR);
            failure_reason = "Pressure too high";
        } else if (irrigation_system.current_flow_rate < MIN_FLOW_RATE_LH) {
            ESP_LOGW(TAG, "Flow rate too low: %.1f < %.1f L/h", irrigation_system.current_flow_rate, MIN_FLOW_RATE_LH);
            failure_reason = "Flow rate too low.";
        } else {
            float water_level_percent;
            if (irrigation_read_level(&water_level_percent) == ESP_OK) {
                irrigation_system.water_level = water_level_percent;
                if (water_level_percent < MIN_WATER_LEVEL_PERCENT - 3.0f) { // Stop watering below 2%
                    ESP_LOGW(TAG, "Water level too low, currently at: %.1f %%", water_level_percent);
                    failure_reason = "Water level critically low";
                }
            } else {
                ESP_LOGW(TAG, "Safety check: Failed to read water level sensor.");
                failure_reason = "Water level sensor read failed";
            }
        }
    }

    // Handle safety failures
    if (failure_reason != NULL) {
        // Increment error count
        (*error_count)++;

        ESP_LOGW(TAG,
                 "Safety failure %" PRIu32 "/%d: %s (Pressure: %.2f, Flow: %.1f)",
                 *error_count,
                 MAX_ERROR_COUNT,
                 failure_reason,
                 irrigation_system.pressure,
                 irrigation_system.current_flow_rate);

        if (*error_count >= MAX_ERROR_COUNT) {
            ESP_LOGE(TAG, "Maximum safety failures reached for: %s", failure_reason);
            irrigation_emergency_stop(failure_reason);
            return false; // Emergency triggered
        } else if (*error_count == 1) {
            ESP_LOGW(TAG, "Reducing pump speed due to safety issue");
            irrigation_set_pump_speed(MIN_PUMP_DUTY);
        }
    }

    return true; // Safe to continue
}

// ########################## Irrigation System ##########################
// ------------------- Emergency Diagnostics Functions -------------------

/**
 * @brief Emergency stop - immediate shutdown
 */
static esp_err_t irrigation_emergency_stop(const char *reason)
{
    ESP_LOGE(TAG, "EMERGENCY STOP TRIGGERED: %s - Immediate irrigation shutdown", reason);

    // Only trigger if not already in an emergency shutdown
    if (!irrigation_system.emergency_stop) {
        irrigation_system.emergency_stop = true;
        // Store the reason for the diagnostics phase
        irrigation_system.emergency.failure_reason = reason;
        irrigation_change_state(IRRIGATION_STOPPING);
    }

    return ESP_OK;
}

/**
 * @brief Initialize emergency diagnostics system
 */
static esp_err_t emergency_diagnostics_init(void)
{
    // memset is redundant - static variables are zero-initialized
    irrigation_system.emergency.state = EMERGENCY_NONE;

    ESP_LOGI(TAG, "Emergency diagnostics system initialized");
    return ESP_OK;
}

/**
 * @brief Start emergency diagnostics with given reason
 */
static esp_err_t emergency_diagnostics_start(const char *reason)
{
    ESP_LOGW(TAG, "=== EMERGENCY DIAGNOSTICS INITIATED ===");
    ESP_LOGW(TAG, "Reason: %s", reason);

    // Power on sensors for the duration of the diagnostics
    power_on_3V3_sensor_bus();

    // Reset diagnostics state
    memset(&irrigation_system.emergency, 0, sizeof(emergency_diagnostics_t));
    irrigation_system.emergency.state = EMERGENCY_TRIGGERED;
    irrigation_system.emergency.diagnostic_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    irrigation_system.emergency.failure_reason = reason;

    // Transition to maintenance state to handle diagnostics
    irrigation_change_state(IRRIGATION_MAINTENANCE);

    ESP_LOGI(TAG, "Emergency diagnostics scheduled - switching to MAINTENANCE state");
    return ESP_OK;
}

/**
 * @brief Check moisture levels before starting diagnostics
 */
static esp_err_t emergency_diagnostics_check_moisture_levels(void)
{
    ESP_LOGI(TAG, "Checking moisture levels before diagnostics...");

    irrigation_system.emergency.eligible_zones_count = 0;
    irrigation_system.emergency.eligible_zones_mask = 0;

    for (uint8_t zone_id = 0; zone_id < IRRIGATION_ZONE_COUNT; zone_id++) {
        if (!irrigation_zones[zone_id].enabled)
            continue;

        float moisture_percent;
        if (irrigation_read_moisture_sensor(zone_id, &moisture_percent) == ESP_OK) {
            ESP_LOGI(TAG, "Zone %d moisture: %.1f%%", zone_id, moisture_percent);

            if (moisture_percent < EMERGENCY_MOISTURE_THRESHOLD) {
                ESP_LOGI(TAG,
                         "Zone %d is eligible for testing (%.1f%% < %.1f%%)",
                         zone_id,
                         moisture_percent,
                         EMERGENCY_MOISTURE_THRESHOLD);
                irrigation_system.emergency.eligible_zones_count++;
                irrigation_system.emergency.eligible_zones_mask |= (1 << zone_id);
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
static esp_err_t emergency_diagnostics_test_zone(uint8_t zone_id)
{
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
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for valve to open

    // Start pump at reduced speed for safety
    esp_err_t ret = irrigation_set_pump_speed(DEFAULT_PUMP_DUTY / 2); // 25% speed for testing
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start pump for zone %d test", zone_id);
        gpio_set_level(zone->valve_gpio, 0);
        irrigation_system.emergency.failed_zones_mask |= (1 << zone_id);
        irrigation_system.emergency.test_flow_rates[zone_id] = 0.0f;
        irrigation_system.emergency.test_pressures[zone_id] = 0.0f;
        irrigation_system.emergency.test_zone++;
        return ESP_FAIL;
    }

    // Record test start time
    irrigation_system.emergency.test_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Reset flow counter
    pcnt_unit_clear_count(flow_pcnt_unit);

    // Wait for test duration
    ESP_LOGI(TAG, "Running %d second test on zone %d...", EMERGENCY_TEST_DURATION_MS / 1000, zone_id);
    vTaskDelay(pdMS_TO_TICKS(EMERGENCY_TEST_DURATION_MS));

    // Measure results
    int test_pulses;
    pcnt_unit_get_count(flow_pcnt_unit, &test_pulses);
    float test_volume_ml = (test_pulses / FLOW_CALIBRATION_PULSES_PER_LITER) * 1000.0f;
    float test_flow_rate = (test_volume_ml / 1000.0f) * (3600.0f / (EMERGENCY_TEST_DURATION_MS / 1000.0f)); // L/h

    // Read pressure
    float test_pressure;
    if (irrigation_read_pressure(&test_pressure) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read pressure during diagnostic test for zone %d", zone_id);
        test_pressure = -1.0f; // Indicate failure
    }

    // Stop pump and close valve
    irrigation_set_pump_speed(0);
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait for pressure to drop
    gpio_set_level(zone->valve_gpio, 0);

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
        failure_reason = "High pressure";
        ESP_LOGW(TAG,
                 "Zone %d FAILED: High pressure %.2f bar > %.2f bar",
                 zone_id,
                 test_pressure,
                 EMERGENCY_TEST_MAX_PRESSURE);
    }

    if (test_passed) {
        ESP_LOGI(TAG,
                 "Zone %d PASSED: Flow %.1f L/h, Pressure %.2f bar, Volume %.1f mL",
                 zone_id,
                 test_flow_rate,
                 test_pressure,
                 test_volume_ml);
    } else {
        ESP_LOGE(TAG,
                 "Zone %d FAILED: %s (Flow %.1f L/h, Pressure %.2f bar)",
                 zone_id,
                 failure_reason,
                 test_flow_rate,
                 test_pressure);
        irrigation_system.emergency.failed_zones_mask |= (1 << zone_id);
    }

    // Move to next zone
    irrigation_system.emergency.test_zone++;
    irrigation_system.emergency.test_cycle_count++;

    // Small delay between tests
    vTaskDelay(pdMS_TO_TICKS(2000));

    return test_passed ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Analyze diagnostic test results and determine next action
 */
static esp_err_t emergency_diagnostics_analyze_results(void)
{
    ESP_LOGI(TAG, "Analyzing emergency diagnostic results...");

    uint8_t failed_zones = 0;

    // Count failures among eligible zones
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        // Only consider zones that were eligible for testing
        if (irrigation_system.emergency.eligible_zones_mask & (1 << i)) {
            if (irrigation_system.emergency.failed_zones_mask & (1 << i)) {
                failed_zones++;
                ESP_LOGW(TAG,
                         "Eligible Zone %d FAILED: Flow %.1f L/h, Pressure %.2f bar",
                         i,
                         irrigation_system.emergency.test_flow_rates[i],
                         irrigation_system.emergency.test_pressures[i]);
            } else {
                ESP_LOGI(TAG,
                         "Eligible Zone %d PASSED: Flow %.1f L/h, Pressure %.2f bar",
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
                if (irrigation_system.emergency.failed_zones_mask & (1 << i)) {
                    irrigation_zones[i].enabled = false;
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
static esp_err_t emergency_diagnostics_resolve(void)
{
    uint32_t diagnostic_duration =
        (xTaskGetTickCount() * portTICK_PERIOD_MS) - irrigation_system.emergency.diagnostic_start_time;

    ESP_LOGW(TAG, "=== EMERGENCY DIAGNOSTICS COMPLETED ===");
    ESP_LOGI(TAG, "Resolution: %s", irrigation_system.emergency.failure_reason);
    ESP_LOGI(TAG, "Duration: %" PRIu32 " seconds", diagnostic_duration / 1000);
    ESP_LOGI(TAG, "Failed zones mask: 0x%02X", irrigation_system.emergency.failed_zones_mask);

    // Log detailed results for maintenance records
    ESP_LOGI(TAG, "Diagnostic test results:");
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        if (irrigation_zones[i].enabled || (irrigation_system.emergency.failed_zones_mask & (1 << i))) {
            ESP_LOGI(TAG,
                     "  Zone %d: Flow %.1f L/h, Pressure %.2f bar, %s",
                     i,
                     irrigation_system.emergency.test_flow_rates[i],
                     irrigation_system.emergency.test_pressures[i],
                     (irrigation_system.emergency.failed_zones_mask & (1 << i)) ? "FAILED" : "PASSED");
        }
    }

    // Power off sensors now that diagnostics are complete
    power_off_3V3_sensor_bus();

    // Clear emergency state
    irrigation_system.emergency.state = EMERGENCY_NONE;

    // Return to normal operation
    ESP_LOGI(TAG, "Returning to normal irrigation operation");
    irrigation_change_state(IRRIGATION_IDLE);

    return ESP_OK;
}

// -----------------#################################-----------------
// -----------------############# TASKS #############-----------------
// -----------------#################################-----------------

/**
 * @brief Main irrigation control task (State Machine)
 */
void irrigation_task(void *pvParameters)
{
    const char *task_tag = "irrigation";
    ESP_LOGI(task_tag, "Irrigation Task started with state machine");

    esp_err_t ret = irrigation_system_init();
    if (ret != ESP_OK) {
        ESP_LOGE(task_tag, "Failed to initialize irrigation system: %s", esp_err_to_name(ret));
        ESP_LOGE(task_tag, "Irrigation task terminating");
        vTaskDelete(NULL);
        return;
    }

    // Wait for other systems to initialize
    vTaskDelay(pdMS_TO_TICKS(3000));

    uint32_t cycle_count = 0;

    ESP_LOGI(task_tag, "Starting irrigation state machine in IDLE state");

    while (1) {
        cycle_count++;

        // Take mutex for state machine execution
        if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Execute current state
            switch (irrigation_system.state) {
                case IRRIGATION_IDLE:
                    irrigation_state_idle();
                    break;

                case IRRIGATION_MEASURING:
                    irrigation_state_measuring();
                    break;

                case IRRIGATION_STARTING:
                    irrigation_state_starting();
                    break;

                case IRRIGATION_WATERING:
                    irrigation_state_watering();
                    break;

                case IRRIGATION_STOPPING:
                    irrigation_state_stopping();
                    break;

                case IRRIGATION_MAINTENANCE:
                    irrigation_state_maintenance();
                    break;

                default:
                    ESP_LOGE(task_tag, "Unknown irrigation state: %d", irrigation_system.state);
                    irrigation_change_state(IRRIGATION_IDLE);
                    break;
            }

            xSemaphoreGive(xIrrigationMutex);
        }

        // Handle emergency stop override - start diagnostics
        if (irrigation_system.emergency_stop) {
            ESP_LOGE(task_tag, "Emergency stop triggered - initiating diagnostics");
            emergency_diagnostics_start("Emergency stop triggered during operation");
            irrigation_system.emergency_stop = false; // Reset trigger flag (diagnostics handle the rest)
        }

        // Periodic status report
        if (cycle_count % 60 == 0) { // Every 5 minutes (assuming 5s cycle)
            ESP_LOGI(task_tag,
                     "State: %s | Active zone: %d | Time in current state: %" PRIu32
                     " s | Pressure: %.1f bar | Flow: %.1f L/h",
                     irrigation_state_name(irrigation_system.state),
                     irrigation_system.state_start_time / 1000,
                     irrigation_system.active_zone,
                     irrigation_system.pressure,
                     irrigation_system.current_flow_rate);
        }

        // State machine runs at 5-second intervals (except monitoring task which runs at 250ms)
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/**
 * @brief Event-driven monitoring task with real-time watering control
 *
 * This task operates in different modes based on event triggers:
 * - SAFETY_CHECK_BIT: One-time safety verification
 * - MONITORING_START_BIT: Begin continuous monitoring during watering
 * - MONITORING_STOP_BIT: End continuous monitoring
 *
 * During continuous monitoring, performs real-time:
 * - Flow rate calculation and validation
 * - Moisture level monitoring with smart cutoffs
 * - Safety checks and emergency stop triggers
 * - Anomaly detection (rain, manual watering, etc.)
 */
static void irrigation_monitoring_task(void *pvParameters)
{
    const char *task_tag = "irrigation_monitor";
    ESP_LOGI(task_tag, "Irrigation monitoring task started");

    // Monitoring state variables
    static bool continuous_monitoring = false;
    static uint32_t error_count = 0;
    static uint32_t last_pulse_count = 0;
    static uint32_t last_flow_measurement_time = 0;
    static uint32_t last_moisture_check_time = 0;

    while (1) {
        // Handle event-driven operations
        EventBits_t event_bits =
            xEventGroupWaitBits(xMonitoringEventGroup,
                                MONITORING_START_BIT | MONITORING_STOP_BIT | SAFETY_CHECK_BIT,
                                pdTRUE,  // Clear bits on exit
                                pdFALSE, // Wait for any bit (OR)
                                continuous_monitoring ? pdMS_TO_TICKS(MONITORING_INTERVAL_MS)
                                                      : portMAX_DELAY); // 500ms monitoring during watering

        // Handle specific events
        if (event_bits & MONITORING_START_BIT) {
            ESP_LOGI(task_tag, "Starting continuous monitoring for watering");
            continuous_monitoring = true;
            error_count = 0;
            last_flow_measurement_time = 0; // Reset flow measurement
        }

        if (event_bits & MONITORING_STOP_BIT) {
            ESP_LOGI(task_tag, "Stopping continuous monitoring");
            continuous_monitoring = false;
        }

        if (event_bits & SAFETY_CHECK_BIT) {
            ESP_LOGD(task_tag, "Performing one-time safety check");
            if (irrigation_pre_check() != ESP_OK) {
                // Failure is handled inside irrigation_pre_check by calling emergency_stop
                irrigation_system.watering_allowed = false;
            } else {
                irrigation_system.watering_allowed = true;
            }
        }

        // Continuous monitoring only during watering
        if (continuous_monitoring && irrigation_system.state == IRRIGATION_WATERING) {
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

            // Update flow rate (every 1 second)
            irrigation_calc_flow_rate(task_tag, &last_pulse_count, &last_flow_measurement_time, current_time);

            // Real-time moisture monitoring and smart cutoffs (every 500ms)
            if (current_time - last_moisture_check_time >= MONITORING_INTERVAL_MS) {
                esp_err_t cutoff_result = irrigation_watering_cutoffs_check(task_tag, current_time);

                if (cutoff_result == ESP_OK) {
                    // Cutoff triggered - watering should stop
                    continuous_monitoring = false;
                } else if (cutoff_result == ESP_FAIL) {
                    // Use the gain rate calculated in the check function for adaptive control
                    uint8_t zone_id = irrigation_system.active_zone;
                    irrigation_pump_adaptive_control(irrigation_system.current_moisture_gain_rate,
                                                     irrigation_zones[zone_id].learning.target_moisture_gain_rate);
                }
                last_moisture_check_time = current_time;
            }

            // Safety monitoring
            if (!irrigation_safety_check(&error_count)) {
                // Emergency condition detected and handled by calling irrigation_emergency_stop().
                continuous_monitoring = false;
            }
        }

        // Reset for next cycle
        if (irrigation_system.emergency_stop) {
            ESP_LOGW(task_tag, "Emergency stop flag detected, monitoring will cease.");
            continuous_monitoring = false;
            error_count = 0;
        }
    }
}

// Task for periodically checking PV panel alignment via photoresistors (ADS1115 Dev#2)
void solar_tracking_task(void *pvParameters)
{
    const char *task_tag = "solar_tracking";
    ESP_LOGI(task_tag, "Solar Tracking Task started. Tracking interval: %d minutes.", TRACKING_INTERVAL_MS / 60000);

    // Initialize servo PWM
    esp_err_t ret = solar_t_servo_pwm_init();
    if (ret != ESP_OK) {
        ESP_LOGE(task_tag, "Failed to initialize servo PWM: %s", esp_err_to_name(ret));
        ESP_LOGE(task_tag, "Solar tracking task terminating");
        vTaskDelete(NULL);
        return;
    }

    // Power on servo bus and move servos to center position initially
    power_on_servo_bus();
    solar_t_servo_set_position(SERVO_YAW_CHANNEL, SERVO_CENTER_DUTY);
    solar_t_servo_set_position(SERVO_PITCH_CHANNEL, SERVO_CENTER_DUTY);
    ESP_LOGI(task_tag, "Servos powered on and moved to center position");

    // Power off servos when not tracking
    power_off_servo_bus();

    // waiting for other systems to initialize
    vTaskDelay(pdMS_TO_TICKS(2000));

    uint32_t tracking_cycles = 0;
    uint32_t successful_reads = 0;
    uint32_t servo_adjustments = 0;

    while (1) {
        tracking_cycles++;

        ESP_LOGD(task_tag, "Starting tracking cycle %" PRIu32 "", tracking_cycles);

        esp_err_t cycle_result = solar_tracker_loop();
        if (cycle_result == ESP_OK) {
            successful_reads++;

            // Update global debug display variables with final readings
            photoresistor_readings_t final_readings;
            if (solar_t_read_photoresistors(&final_readings) == ESP_OK) {
                if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    latest_ads_voltages[0][0] = final_readings.left_top;
                    latest_ads_voltages[0][1] = final_readings.right_top;
                    latest_ads_voltages[0][2] = final_readings.left_bottom;
                    latest_ads_voltages[0][3] = final_readings.right_bottom;
                    xSemaphoreGive(xDisplayDataMutex);
                }
            }
        } else {
            ESP_LOGE(task_tag, "Tracking cycle failed: %s", esp_err_to_name(cycle_result));
        }

        // Periodic status report
        if (tracking_cycles % 12 == 0) // !During tests every 1 minute
        {
            ESP_LOGI(task_tag,
                     "Tracking stats: %" PRIu32 "/%" PRIu32 " successful reads, %" PRIu32 " servo adjustments",
                     successful_reads,
                     tracking_cycles,
                     servo_adjustments);
        }

        // Wait until next tracking cycle
        vTaskDelay(pdMS_TO_TICKS(TRACKING_INTERVAL_MS));
    }
}


// Task for periodically reading AS5600 angle
void as5600_read_task(void *pvParameters)
{
    ESP_LOGI(TAG, "AS5600 Read Task started. Initializing sensor...");

    // Initialize AS5600 Sensor within the task
    esp_err_t as5600_init_result = as5600_init_desc(&as5600_dev,
                                                    I2C_PORT_NUM,
                                                    AS5600_I2C_ADDR,
                                                    CONFIG_I2CDEV_DEFAULT_SDA_PIN,
                                                    CONFIG_I2CDEV_DEFAULT_SCL_PIN);
    if (as5600_init_result != ESP_OK) {
        ESP_LOGE(TAG, "AS5600: Failed to initialize descriptor: %s", esp_err_to_name(as5600_init_result));
        ESP_LOGE(TAG, "AS5600: Task terminating due to initialization failure");
        vTaskDelete(NULL);
        return;
    }

    // Then initialize the device itself
    as5600_init_result = as5600_init(&as5600_dev);
    if (as5600_init_result != ESP_OK) {
        ESP_LOGE(TAG, "AS5600: Failed to initialize device: %s", esp_err_to_name(as5600_init_result));
        ESP_LOGE(TAG, "AS5600: Task terminating due to initialization failure");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "AS5600: Sensor initialized successfully");

    // Wait a moment for the sensor's internal update task to start
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        float angle_deg = 0;
        uint16_t raw_counts = 0;
        float accumulated_counts = 0;

        esp_err_t ret_angle = as5600_read_angle_degrees(&as5600_dev, &angle_deg);
        esp_err_t ret_raw = as5600_read_raw_counts(&as5600_dev, &raw_counts);
        esp_err_t ret_acc = as5600_read_accumulated_counts(&as5600_dev, &accumulated_counts);

        if (ret_angle == ESP_OK && ret_raw == ESP_OK && ret_acc == ESP_OK) {
            // ESP_LOGW("AS5600", "Angle: %.2f deg, Raw: %u, Accumulated: %.1f", angle_deg,
            // raw_counts, accumulated_counts);

            // Debug display variables
            if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) { // Wait max 100ms
                latest_as5600_angle = angle_deg;
                latest_as5600_raw = raw_counts;
                xSemaphoreGive(xDisplayDataMutex);
            } else {
                ESP_LOGW("AS5600", "Could not get display mutex to update globals");
            }
        } else {
            ESP_LOGE("AS5600", "Failed to read - Angle: %d, Raw: %d, Acc: %d", ret_angle, ret_raw, ret_acc);
        }

        // Wait for 1 second
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// Re-add SHT4x Task definition
void SHT4xReadTask(void *pvParameters)
{
    static sht4x_t sht4x_dev;

    TaggedSensorData sht4xdata;
    sht4xdata.tag = SENSOR_SHT4X;
    sht4xdata.pressure = 0;

    // Note: Static variables are automatically zero-initialized, memset not needed
    ESP_ERROR_CHECK(
        sht4x_init_desc(&sht4x_dev, I2C_PORT_NUM, CONFIG_I2CDEV_DEFAULT_SDA_PIN, CONFIG_I2CDEV_DEFAULT_SCL_PIN));
    ESP_LOGI("sht4x", ">> Entering sht4x_init_desc()");
    ESP_ERROR_CHECK(sht4x_init(&sht4x_dev));

    // Get the measurement duration for high repeatability;
    uint8_t duration = sht4x_get_measurement_duration(&sht4x_dev);
    while (1) {
        // Trigger one measurement in single shot mode with high repeatability.
        ESP_ERROR_CHECK(sht4x_start_measurement(&sht4x_dev));
        // Wait until measurement is ready (duration returned from
        // *sht4x_get_measurement_duration*).
        vTaskDelay(duration); // duration is in ticks

        // retrieve the values and send it to the queue
        if (sht4x_get_results(&sht4x_dev, &sht4xdata.temperature, &sht4xdata.humidity) == ESP_OK) {
            // ESP_LOGI("SHT40", "Timestamp: %lu, SHT40  - Temperature: %.2f °C, Humidity: %.2f %%",
            // (unsigned long)xTaskGetTickCount(), sht4xdata.temperature, sht4xdata.humidity);
            if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) { // Wait max 100ms
                latest_sht_temp = sht4xdata.temperature;
                latest_sht_hum = sht4xdata.humidity;
                xSemaphoreGive(xDisplayDataMutex);
            } else {
                ESP_LOGW("SHT40", "Could not get display mutex to update globals");
            }
        } else {
            ESP_LOGI("SHT40", "SHT40 Failed to read sensor data.");
        }
        // 10 s delay
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Re-add BME280 Task definition
void BME280ReadTask(void *pvParameters)
{
    static bmp280_t bme280_dev;
    bmp280_params_t params;

    TaggedSensorData bme280data;
    bme280data.tag = SENSOR_BME280;

    // Initialize the sensor
    bmp280_init_default_params(&params);
    // Note: Static variables are automatically zero-initialized, memset not needed
    ESP_ERROR_CHECK(bmp280_init_desc(&bme280_dev,
                                     BMP280_I2C_ADDRESS_0,
                                     I2C_PORT_NUM,
                                     CONFIG_I2CDEV_DEFAULT_SDA_PIN,
                                     CONFIG_I2CDEV_DEFAULT_SCL_PIN));
    ESP_ERROR_CHECK(bmp280_init(&bme280_dev, &params));
    bool bme280p = bme280_dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", (bme280p ? "BME280" : "BMP280"));

    while (1) {
        // Set the sensor to forced mode to initiate a measurement
        ESP_ERROR_CHECK(bmp280_force_measurement(&bme280_dev));

        // Wait for the measurement to complete
        bool busy;
        do {
            ESP_ERROR_CHECK(bmp280_is_measuring(&bme280_dev, &busy));
            if (busy)
                vTaskDelay(pdMS_TO_TICKS(5)); // Wait for 5ms before checking again
        } while (busy);

        // Read the measurement results
        if (bmp280_read_float(&bme280_dev, &bme280data.temperature, &bme280data.pressure, &bme280data.humidity) ==
            ESP_OK) {
            // ESP_LOGI("BME280", "Timestamp: %lu, BME280 - Temperature: %.2f °C, Humidity: %.2f %%,
            // Pressure: %.2f hPa", (unsigned long)xTaskGetTickCount(),
            //  bme280data.temperature, bme280data.humidity, bme280data.pressure/100);

            // Debug display variables - Wait max 100ms
            if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                latest_bmp_temp = bme280data.temperature;
                latest_bmp_hum = bme280data.humidity;
                latest_bmp_press = bme280data.pressure;
                xSemaphoreGive(xDisplayDataMutex);
            } else {
                ESP_LOGW("BME280", "Could not get display mutex to update globals");
            }
        } else {
            ESP_LOGI("BME280", "BME280 Failed to read sensor data.");
        }
        // 10 s delay
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Task for reading INA219 voltage and current sensors
void ina219_read_task(void *pvParameters)
{
    ESP_LOGI(TAG, "INA219 Read Task started.");

    // Initialize each INA219 sensor
    for (int i = 0; i < INA219_DEVICE_COUNT; i++) {
        ina219_sensor_t *ina219_sensor = &ina219_devices[i];

        ESP_LOGI(TAG,
                 "Initializing INA219 %s (type: %d, addr: 0x%02x, shunt: %.3f ohm)...",
                 ina219_sensor->name,
                 ina219_sensor->type,
                 ina219_sensor->addr,
                 ina219_sensor->shunt_ohms);

        // Initialize descriptor
        esp_err_t init_desc_res = ina219_init_desc(&ina219_sensor->dev,
                                                   ina219_sensor->addr,
                                                   I2C_PORT_NUM,
                                                   CONFIG_I2CDEV_DEFAULT_SDA_PIN,
                                                   CONFIG_I2CDEV_DEFAULT_SCL_PIN);
        if (init_desc_res != ESP_OK) {
            ESP_LOGE(TAG,
                     "  Failed to initialize INA219 %s descriptor: %s",
                     ina219_sensor->name,
                     esp_err_to_name(init_desc_res));
            continue;
        }

        // Initialize device
        esp_err_t init_res = ina219_init(&ina219_sensor->dev);
        if (init_res != ESP_OK) {
            ESP_LOGE(TAG, "  Failed to initialize INA219 %s: %s", ina219_sensor->name, esp_err_to_name(init_res));
            continue;
        }

        // Configure device
        esp_err_t cfg_res = ina219_configure(&ina219_sensor->dev,
                                             INA219_BUS_RANGE_32V,        // 32V range for all sensors
                                             ina219_sensor->gain,         // Gain from sensor config
                                             INA219_RES_12BIT_8S,         // 12-bit resolution with 8 samples
                                             INA219_RES_12BIT_8S,         // 12-bit resolution with 8 samples
                                             INA219_MODE_CONT_SHUNT_BUS); // Continuous mode
        if (cfg_res != ESP_OK) {
            ESP_LOGE(TAG, "  Failed to configure INA219 %s: %s", ina219_sensor->name, esp_err_to_name(cfg_res));
            continue;
        }

        // Calibrate with the appropriate shunt resistor value
        ESP_LOGI(TAG, "  Calibrating INA219 %s with %.3f ohm shunt", ina219_sensor->name, ina219_sensor->shunt_ohms);
        esp_err_t cal_res = ina219_calibrate(&ina219_sensor->dev, ina219_sensor->shunt_ohms);
        if (cal_res != ESP_OK) {
            ESP_LOGE(TAG, "  Failed to calibrate INA219 %s: %s", ina219_sensor->name, esp_err_to_name(cal_res));
            continue;
        }

        // Mark as initialized if all steps succeeded
        ina219_sensor->initialized = true;
        ESP_LOGI(TAG, "  INA219 %s initialized successfully", ina219_sensor->name);
    }

    // Main task loop
    while (1) {
        for (int i = 0; i < INA219_DEVICE_COUNT; i++) {
            ina219_sensor_t *ina219_sensor = &ina219_devices[i];

            // Skip sensors that failed initialization
            if (!ina219_sensor->initialized) {
                continue;
            }

            float voltage = -999.9;
            float current = -999.9;
            float power = -999.9;

            esp_err_t v_res = ina219_get_bus_voltage(&ina219_sensor->dev, &voltage);
            esp_err_t c_res = ina219_get_current(&ina219_sensor->dev, &current);
            esp_err_t p_res = ina219_get_power(&ina219_sensor->dev, &power);

            if (v_res == ESP_OK && c_res == ESP_OK && p_res == ESP_OK) {
                // Update global values with mutex protection
                if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    latest_ina219_voltage[i] = voltage;
                    latest_ina219_current[i] = current;
                    latest_ina219_power[i] = power;
                    xSemaphoreGive(xDisplayDataMutex);
                } else {
                    ESP_LOGW(TAG, "INA219 %s: Could not get display mutex to update globals", ina219_sensor->name);
                }
            } else {
                ESP_LOGE(TAG,
                         "INA219 %s: Read error - Voltage: %s, Current: %s, Power: %s",
                         ina219_sensor->name,
                         esp_err_to_name(v_res),
                         esp_err_to_name(c_res),
                         esp_err_to_name(p_res));
            }
        }

        // Read every 2 seconds
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void local_power_monitoring_task(void *pvParameters)
{
    while (1) {
        // TODO: Implement power monitoring functionality
        /*
        // Read battery voltage/current
        // read_battery_status();

        // Read solar panel power generation
        // read_solar_generation();

        // Read system consumption
        // read_system_consumption();

        // Trigger power management decisions
        // update_power_management_strategy();
        */

        // Placeholder delay
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void serial_display_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Serial Display Task started.");
    vTaskDelay(pdMS_TO_TICKS(1000));
    // ANSI escape codes
    const char *ANSI_CLEAR_SCREEN = "\033[2J";
    const char *ANSI_CURSOR_HOME = "\033[H";

    char buffer[100]; // Buffer for formatting lines

    // Declare local variables for sensor readings outside the loop
    float sht_t, sht_h, bmp_t, bmp_p, bmp_h, as_a;
    uint16_t as_r;
    float ads_voltages[ADS1115_DEVICE_COUNT][4];
    float ina219_v[INA219_DEVICE_COUNT], ina219_c[INA219_DEVICE_COUNT], ina219_p[INA219_DEVICE_COUNT];

    while (1) {
        // --- Read Global Variables Safely ---
        if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // SHT4x
            sht_t = latest_sht_temp;
            sht_h = latest_sht_hum;
            // BMP280
            bmp_t = latest_bmp_temp;
            bmp_p = latest_bmp_press;
            bmp_h = latest_bmp_hum;
            // Copy ADS1115 arrays
            for (int dev = 0; dev < ADS1115_DEVICE_COUNT; dev++) {
                for (int ch = 0; ch < 4; ch++) {
                    ads_voltages[dev][ch] = latest_ads_voltages[dev][ch];
                }
            }
            // AS5600
            as_a = latest_as5600_angle;
            as_r = latest_as5600_raw;
            // Read INA219 values
            for (int i = 0; i < INA219_DEVICE_COUNT; i++) {
                ina219_v[i] = latest_ina219_voltage[i];
                ina219_c[i] = latest_ina219_current[i];
                ina219_p[i] = latest_ina219_power[i];
            }

            xSemaphoreGive(xDisplayDataMutex);
        } else {
            ESP_LOGW(TAG, "Serial display task could not get mutex");
            // Optionally print an error message to serial?
            printf("%s%s!!! Failed to get mutex !!!\n", ANSI_CLEAR_SCREEN, ANSI_CURSOR_HOME);
            vTaskDelay(pdMS_TO_TICKS(SERIAL_DISPLAY_INTERVAL_MS));
            continue;
        }
        // --- End Read ---

        // --- Format and Print to Serial Monitor ---
        // Clear screen and move cursor to top-left

        // printf("%s%s", ANSI_CLEAR_SCREEN, ANSI_CURSOR_HOME);

        // Print header
        printf("--- Sensor Readings ---\n");

        // Print sensor data (chat says using snprintf for safety is good practice)
        snprintf(buffer, sizeof(buffer), "SHT4x: Temp=%.1f C, Hum=%.1f %%\n", sht_t, sht_h);
        printf("%s", buffer);

        snprintf(buffer,
                 sizeof(buffer),
                 "BMP280: Temp=%.1f C, Press=%.0f hPa, Hum=%.1f %%\n",
                 bmp_t,
                 bmp_p / 100,
                 bmp_h); // Assuming BME & Pa -> hPa
        printf("%s", buffer);

        // Display ADS1115 readings organized by function
        snprintf(buffer,
                 sizeof(buffer),
                 "ADS0 (Photo): LT=%.3f RT=%.3f LB=%.3f RB=%.3f\n",
                 ads_voltages[0][0],
                 ads_voltages[0][1],
                 ads_voltages[0][2],
                 ads_voltages[0][3]);
        printf("%s", buffer);

        snprintf(buffer,
                 sizeof(buffer),
                 "ADS1 (Moist): Z1=%.3f Z2=%.3f Z3=%.3f Z4=%.3f\n",
                 ads_voltages[1][0],
                 ads_voltages[1][1],
                 ads_voltages[1][2],
                 ads_voltages[1][3]);
        printf("%s", buffer);

        snprintf(buffer,
                 sizeof(buffer),
                 "ADS2 (Mixed): Z5=%.3f Press=%.3f Ch2=%.3f Ch3=%.3f\n",
                 ads_voltages[2][0],
                 ads_voltages[2][1],
                 ads_voltages[2][2],
                 ads_voltages[2][3]);
        printf("%s", buffer);

        snprintf(buffer, sizeof(buffer), "AS5600: Angle=%.1f deg, Raw=%u\n", as_a, as_r);
        printf("%s", buffer);

        // Add INA219 display
        for (int i = 0; i < INA219_DEVICE_COUNT; i++) {
            snprintf(buffer,
                     sizeof(buffer),
                     "INA219_%s: V=%.2f V, I=%.2f mA, P=%.2f mW\n",
                     ina219_devices[i].name,
                     ina219_v[i],
                     ina219_c[i] * 1000,
                     ina219_p[i] * 1000);
            printf("%s", buffer);
        }

        printf("-----------------------\n");
        // Flush stdout buffer to ensure data is sent immediately
        fflush(stdout);

        // --- End Format and Print ---

        vTaskDelay(pdMS_TO_TICKS(SERIAL_DISPLAY_INTERVAL_MS));
    }
}

/**
 * @brief Background task for retrying failed ADS1115 devices
 *
 * Continuously monitors for failed devices and attempts to recover them
 * using exponential backoff strategy (1min -> 3min -> 10min max).
 * Runs in background to provide automatic recovery without user intervention.
 *
 * @param pvParameters Unused task parameter
 */
static void ads1115_retry_task(void *pvParameters)
{
    ESP_LOGI(TAG, "ADS1115 retry task started");

    const uint32_t CHECK_INTERVAL_MS = 5000;    // Check every 5 seconds
    const uint32_t MAX_RETRY_DELAY_MS = 600000; // Max 10 minutes between retries

    while (1) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        bool retry_attempted = false;

        // Check each device for retry
        for (uint8_t device = 0; device < ADS1115_DEVICE_COUNT; device++) {
            ads1115_device_t *status = &ads1115_devices[device];

            // Skip if device is already working
            if (status->initialized) {
                continue;
            }

            // Check if it's time to retry
            uint32_t time_since_last_retry = current_time - status->last_retry_time;
            if (time_since_last_retry >= status->next_retry_delay_ms) {
                ESP_LOGI(TAG,
                         "Retrying ADS1115 device %d (%s) - attempt %d",
                         device,
                         ads1115_devices[device].name,
                         status->retry_count + 1);

                // Attempt to reinitialize
                esp_err_t result = ads1115_init_device(device);
                status->last_retry_time = current_time;
                status->retry_count++;
                retry_attempted = true;

                if (result == ESP_OK) {
                    status->initialized = true;
                    status->retry_count = 0;             // Reset retry count on success
                    status->next_retry_delay_ms = 60000; // Reset delay to 1 minute
                    ESP_LOGI(TAG,
                             "✓ ADS1115 device %d (%s) successfully recovered!",
                             device,
                             ads1115_devices[device].name);
                } else {
                    ESP_LOGW(TAG,
                             "✗ ADS1115 device %d (%s) retry failed: %s",
                             device,
                             ads1115_devices[device].name,
                             esp_err_to_name(result));

                    // Exponential backoff: 1min -> 3min -> 10min -> 10min (max)
                    if (status->retry_count == 1) {
                        status->next_retry_delay_ms = 180000; // 3 minutes
                    } else if (status->retry_count >= 2) {
                        status->next_retry_delay_ms = MAX_RETRY_DELAY_MS; // 10 minutes
                    }

                    ESP_LOGW(TAG, "    Next retry in %" PRIu32 " minutes", status->next_retry_delay_ms / 60000);
                }
            }
        }

        // Log status periodically
        if (retry_attempted) {
            uint8_t working_count = 0;
            for (uint8_t i = 0; i < ADS1115_DEVICE_COUNT; i++) {
                if (ads1115_devices[i].initialized) {
                    working_count++;
                }
            }
            ESP_LOGI(TAG, "ADS1115 status: %d/%d devices working", working_count, ADS1115_DEVICE_COUNT);
        }

        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_MS));
    }
}

// -----------------##############################-----------------
// -----------------############# MAIN ###########-----------------
// -----------------##############################-----------------

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Starting ESP...");

    /* Display Mutex creation */
    xDisplayDataMutex = xSemaphoreCreateMutex();
    if (xDisplayDataMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create display data mutex!");
        // Handle error appropriately, maybe halt or return an error code
    } else {
        ESP_LOGI(TAG, "Display data mutex created successfully.");
    }

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }
    printf("%" PRIu32 "MB %s flash\n",
           flash_size / (uint32_t) (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // Initialize I2C subsystem
    ESP_LOGI(TAG, "Initializing I2C...");
    ESP_ERROR_CHECK(i2cdev_init());

    // Initialize all ADS1115 devices
    esp_err_t ads_init_result = ads1115_init_all();
    if (ads_init_result == ESP_FAIL) {
        ESP_LOGW(TAG,
                 "No ADS1115 devices available at startup - system will continue with limited "
                 "functionality");
        ESP_LOGW(TAG, "The retry task will attempt to recover failed devices automatically");
    } else {
        ESP_LOGI(TAG, "ADS1115 initialization completed successfully");
    }

    //---------- TASKS ----------
    ESP_LOGI(TAG, "Creating tasks...");
    // Core sensor tasks
    xTaskCreate(solar_tracking_task, "SolarTrackingTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
    xTaskCreate(irrigation_task, "IrrigationTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
    // xTaskCreate(weather_task, "WeatherTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

    xTaskCreate(SHT4xReadTask, "SHT4xReadTask", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(BME280ReadTask, "BME280ReadTask", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(as5600_read_task, "AS5600ReadTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

    // Local power monitoring task
    // xTaskCreate(local_power_monitoring_task, "LocalPowerMonitoringTask", configMINIMAL_STACK_SIZE
    // * 4, NULL, 5, NULL);
    xTaskCreate(ina219_read_task, "INA219ReadTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

    // ADS1115 retry task for automatic recovery
    xTaskCreate(ads1115_retry_task, "Ads1115RetryTask", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

    // Communication tasks
    // xTaskCreate(telemetry_task, "TelemetryTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
    // xTaskCreate(mppt_communication_task, "mppt_communication_task", configMINIMAL_STACK_SIZE * 4,
    // NULL, 5, NULL);

    // Misc tasks
    xTaskCreate(serial_display_task, "serial_display_task", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

    ESP_LOGI(TAG, "Tasks created.");
}
