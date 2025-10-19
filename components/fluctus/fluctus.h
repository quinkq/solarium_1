#ifndef FLUCTUS_H
#define FLUCTUS_H

#include <stdbool.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "ads111x.h"
#include "ina219.h"

// ########################## FLUCTUS Configuration ##########################

// Power Bus Control GPIOs - Hardware Configuration
// 3.3V: Push-pull output, HIGH=ON (controls N-MOSFET via SN74AHCT125N buffer)
// 5V:   Open-drain output, FLOAT=ON, LOW=OFF (100kΩ series to buck EN pin)
// 6.6V: Open-drain output, LOW=ON (inverted logic buck, 100kΩ pullup to 3.3V)
// 12V:  Open-drain output, FLOAT=ON, LOW=OFF (100kΩ series to buck EN pin)
#define FLUCTUS_3V3_BUS_ENABLE_GPIO     GPIO_NUM_0   // 3.3V non-essential sensor bus
#define FLUCTUS_5V_BUS_ENABLE_GPIO      GPIO_NUM_4   // 5V bus (MOSFET drivers)
#define FLUCTUS_6V6_BUS_ENABLE_GPIO     GPIO_NUM_5   // 6.6V servo bus (inverted logic)
#define FLUCTUS_12V_BUS_ENABLE_GPIO     GPIO_NUM_6   // 12V bus enable
#define FLUCTUS_12V_FAN_PWM_GPIO        GPIO_NUM_45  // 12V cooling fan PWM (Intel cooler)

// Solar Tracking Servo GPIOs
#define FLUCTUS_SERVO_YAW_GPIO          GPIO_NUM_47  // Yaw servo control
#define FLUCTUS_SERVO_PITCH_GPIO        GPIO_NUM_48  // Pitch servo control

// Temperature Monitoring GPIO
#define FLUCTUS_DS18B20_GPIO            GPIO_NUM_15  // DS18B20 OneWire temperature sensor

// INA219 Configuration
#define FLUCTUS_INA219_DEVICE_COUNT     2
#define FLUCTUS_INA219_SOLAR_PV_ADDR    0x40         // Solar PV input monitoring
#define FLUCTUS_INA219_BATTERY_OUT_ADDR 0x41         // Battery output monitoring

// Solar Tracking PWM Configuration
#define FLUCTUS_SERVO_PWM_FREQUENCY     50           // 50Hz for servos
#define FLUCTUS_SERVO_PWM_RESOLUTION    LEDC_TIMER_14_BIT  // 14-bit resolution
#define FLUCTUS_SERVO_PWM_TIMER         LEDC_TIMER_2
#define FLUCTUS_SERVO_YAW_CHANNEL       LEDC_CHANNEL_1
#define FLUCTUS_SERVO_PITCH_CHANNEL     LEDC_CHANNEL_2
#define FLUCTUS_SERVO_PWM_SPEED_MODE    LEDC_LOW_SPEED_MODE

// Solar Tracking Servo Limits
#define FLUCTUS_SERVO_MIN_DUTY          410          // Minimum servo duty cycle
#define FLUCTUS_SERVO_MAX_DUTY          2048         // Maximum servo duty cycle  
#define FLUCTUS_SERVO_CENTER_DUTY       1229         // Center/park position
#define FLUCTUS_MAX_SERVO_ADJUSTMENT    250          // Maximum adjustment per cycle

// Solar Tracking Configuration
#define FLUCTUS_PHOTORESISTOR_THRESHOLD 0.050f       // Minimum error threshold (V)
#define FLUCTUS_TRACKING_INTERVAL_MS    5000         // Tracking update interval (when correcting)
#define FLUCTUS_SERVO_POWERUP_DELAY_MS  100          // Servo power stabilization

// Solar Tracking Automation Configuration
#define FLUCTUS_TRACKING_CORRECTION_INTERVAL_MS     (15 * 60 * 1000)  // 15 minutes between correction cycles
#define FLUCTUS_TRACKING_CORRECTION_TIMEOUT_MS      30000    // 30 seconds max correction time
#define FLUCTUS_TRACKING_SUNRISE_BUFFER_MINUTES     30       // Start tracking 30 min before sunrise
#define FLUCTUS_TRACKING_SUNSET_BUFFER_MINUTES      30       // Allow disable 30 min before sunset

// Solar Tracking Parking Positions (duty cycle percentages)
#define FLUCTUS_SERVO_NIGHT_PARK_YAW_PERCENT        10.0f    // 10% from left (east-facing)
#define FLUCTUS_SERVO_NIGHT_PARK_PITCH_PERCENT      60.0f    // 60% upward from bottom
#define FLUCTUS_SERVO_ERROR_PARK_YAW_PERCENT        50.0f    // 50% center position (yaw)
#define FLUCTUS_SERVO_ERROR_PARK_PITCH_PERCENT      50.0f    // 50% center position (pitch)

// Fan PWM Configuration (Intel cooler with PWM input)
#define FLUCTUS_FAN_PWM_FREQUENCY       25000        // 25kHz for 4-pin fan control
#define FLUCTUS_FAN_PWM_RESOLUTION      LEDC_TIMER_8_BIT   // 8-bit resolution (0-255)
#define FLUCTUS_FAN_PWM_TIMER           LEDC_TIMER_3
#define FLUCTUS_FAN_PWM_CHANNEL         LEDC_CHANNEL_3

// Temperature Monitoring Configuration
#define FLUCTUS_TEMP_TURN_ON_THRESHOLD      30.0f    // Turn on fan at 30°C
#define FLUCTUS_TEMP_TURN_OFF_THRESHOLD     28.0f    // Turn off fan at 28°C (hysteresis)
#define FLUCTUS_TEMP_MAX_FAN_THRESHOLD      40.0f    // 100% fan speed at 40°C
#define FLUCTUS_TEMP_MIN_FAN_DUTY           20       // Minimum fan speed (20% at 30°C)
#define FLUCTUS_TEMP_MAX_FAN_DUTY           100      // Maximum fan speed (100% at 40°C)
#define FLUCTUS_TEMP_ACTIVE_INTERVAL_MS     5000     // Read temp every 5s during active monitoring
#define FLUCTUS_TEMP_THERMAL_INTERVAL_MS    60000    // Read temp every 1min during thermal event
#define FLUCTUS_TEMP_CONVERSION_DELAY_MS    750      // DS18B20 12-bit conversion time

// Power Monitoring Configuration
#define FLUCTUS_POWER_ACTIVE_MONITOR_INTERVAL_MS    500     // Power reading interval when active
#define FLUCTUS_POWER_STEADY_MONITOR_INTERVAL_MS    60000   // 60 seconds when in steady state
#define FLUCTUS_POWER_STEADY_PROBE_DURATION_MS      3000    // 3 second probe duration in steady state
#define FLUCTUS_POWER_PERIODIC_CHECK_INTERVAL_MS (15 * 60 * 1000)  // 15 minutes when idle

// Power Metering Configuration
#define FLUCTUS_PV_LIGHT_THRESHOLD      0.5f    // Photoresistor threshold for nighttime (V)

// Task Notification Bits for fluctus_monitoring_task
#define FLUCTUS_NOTIFY_POWER_ACTIVITY        (1UL << 0)  // Power bus activity detected

// Battery Voltage Thresholds (12V AGM, 50% SOC = 0%)
#define FLUCTUS_BATTERY_LEVEL_POWER_SAVING    12.48f  // 40% SOC - Limit STELLARIA
#define FLUCTUS_BATTERY_LEVEL_LOW_POWER       12.33f  // 25% SOC - Turn off STELLARIA
#define FLUCTUS_BATTERY_LEVEL_VERY_LOW        12.23f  // 15% SOC - Turn off IMPLUVIUM  
#define FLUCTUS_BATTERY_LEVEL_CRITICAL        12.08f  // 0% SOC  - Turn off TEMPESTA + Solar

// Battery Voltage Hysteresis (+0.1V for turn-on)
#define FLUCTUS_BATTERY_HYSTERESIS            0.10f
#define FLUCTUS_BATTERY_LEVEL_POWER_SAVING_ON (FLUCTUS_BATTERY_LEVEL_POWER_SAVING + FLUCTUS_BATTERY_HYSTERESIS)
#define FLUCTUS_BATTERY_LEVEL_LOW_POWER_ON    (FLUCTUS_BATTERY_LEVEL_LOW_POWER + FLUCTUS_BATTERY_HYSTERESIS)
#define FLUCTUS_BATTERY_LEVEL_VERY_LOW_ON     (FLUCTUS_BATTERY_LEVEL_VERY_LOW + FLUCTUS_BATTERY_HYSTERESIS)
#define FLUCTUS_BATTERY_LEVEL_CRITICAL_ON     (FLUCTUS_BATTERY_LEVEL_CRITICAL + FLUCTUS_BATTERY_HYSTERESIS)

// Overcurrent Protection
#define FLUCTUS_OVERCURRENT_THRESHOLD_1       3.0f    // Amps, 3-second delay
#define FLUCTUS_OVERCURRENT_THRESHOLD_2       4.0f    // Amps, immediate shutdown
#define FLUCTUS_OVERCURRENT_DELAY_MS          3000    // Delay for threshold 1

// Mutex timeout constants
#define FLUCTUS_MUTEX_TIMEOUT_QUICK_MS        100     // Quick operations
#define FLUCTUS_MUTEX_TIMEOUT_LONG_MS         1000    // Longer operations

// ########################## Data Structures ##########################

typedef enum {
    POWER_BUS_3V3 = 0,          // 3.3V sensor bus
    POWER_BUS_5V,               // 5V bus (MOSFET drivers)  
    POWER_BUS_6V6,              // 6.2V servo bus
    POWER_BUS_12V,              // 12V bus
    POWER_BUS_COUNT             // Total number of buses
} power_bus_t;

typedef enum {
    FLUCTUS_POWER_STATE_NORMAL = 0,     // Normal operation
    FLUCTUS_POWER_STATE_POWER_SAVING,   // Power saving mode (40% Battery)
    FLUCTUS_POWER_STATE_LOW_POWER,      // Low power mode (25% Battery)
    FLUCTUS_POWER_STATE_VERY_LOW,       // Very low power mode (15% Battery)
    FLUCTUS_POWER_STATE_CRITICAL,       // Critical power mode (0% Battery)
    FLUCTUS_POWER_STATE_SHUTDOWN        // Overcurrent safety shutdown
} fluctus_power_state_t;

typedef enum {
    SOLAR_TRACKING_DISABLED = 0,        // Solar tracking completely disabled
    SOLAR_TRACKING_PAUSED,              // Tracking enabled but paused between corrections
    SOLAR_TRACKING_CORRECTING,          // Actively correcting position
    SOLAR_TRACKING_PARKING,             // Moving to park position
    SOLAR_TRACKING_ERROR                // Error state (timeout or failure)
} solar_tracking_state_t;

typedef struct {
    bool bus_enabled[POWER_BUS_COUNT];          // Individual bus states
    uint8_t bus_ref_count[POWER_BUS_COUNT];     // Reference counting for consumers
    char bus_consumers[POWER_BUS_COUNT][8][16]; // Consumer names per bus (max 8 per bus)
    fluctus_power_state_t power_state;          // Current power management state
    solar_tracking_state_t solar_tracking_state; // Solar tracking state machine
    uint32_t current_yaw_duty;                  // Current yaw servo position
    uint32_t current_pitch_duty;                // Current pitch servo position
    bool safety_shutdown;                       // Overcurrent safety shutdown flag
    bool manual_reset_required;                 // Manual reset flag for safety
    time_t last_activity_time;                  // Last consumer activity timestamp
} fluctus_power_status_t;

typedef struct {
    float voltage;                      // Bus voltage (V)
    float current;                      // Bus current (A)
    float power;                        // Bus power (W)
    bool valid;                         // Reading validity
    time_t timestamp;                   // Reading timestamp
} fluctus_power_reading_t;

typedef struct {
    fluctus_power_reading_t solar_pv;   // Solar PV input readings
    fluctus_power_reading_t battery_out; // Battery output readings
    float battery_voltage;              // Calculated battery voltage
    float total_current;                // Total system current
    float total_power;                  // Total system power
    float case_temperature;             // Case temperature from DS18B20 (°C)
    bool temperature_valid;             // Temperature reading validity
    time_t temperature_timestamp;       // Temperature reading timestamp
    bool pv_ina_active;                 // PV INA219 power state (true = active, false = shutdown)
    bool battery_ina_active;            // Battery INA219 power state (true = active, false = shutdown)
} fluctus_monitoring_data_t;

typedef struct {
    float yaw_error;                    // Yaw tracking error (V)
    float pitch_error;                  // Pitch tracking error (V)
    float photoresistor_readings[4];    // Individual photoresistor voltages
    bool valid;                         // Reading validity
} fluctus_solar_tracking_data_t;

typedef enum {
    FLUCTUS_EVENT_BUS_ON = 0,
    FLUCTUS_EVENT_BUS_OFF,
    FLUCTUS_EVENT_CONSUMER_REQUEST,
    FLUCTUS_EVENT_CONSUMER_RELEASE,
    FLUCTUS_EVENT_OVERCURRENT_WARNING,
    FLUCTUS_EVENT_OVERCURRENT_SHUTDOWN,
    FLUCTUS_EVENT_BATTERY_LOW,
    FLUCTUS_EVENT_SOLAR_TRACKING_PARK,
    FLUCTUS_EVENT_POWER_STATE_CHANGE
} fluctus_event_type_t;

typedef struct {
    fluctus_event_type_t event_type;
    power_bus_t bus;
    char consumer_id[16];
    float current_value;
    float voltage_value;
    fluctus_power_state_t power_state;
    time_t timestamp;
} fluctus_power_event_t;

// ########################## Public API Functions ##########################

/**
 * @brief Initialize FLUCTUS power management and solar tracking system
 * @return ESP_OK on success, ESP_FAIL on initialization failure
 */
esp_err_t fluctus_init(void);

// ################ Power Bus Control Functions ################

/**
 * @brief Request power bus activation
 * @param bus Power bus to activate
 * @param consumer_id String identifier for the requesting consumer (max 15 chars)
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if safety shutdown active
 */
esp_err_t fluctus_request_bus_power(power_bus_t bus, const char* consumer_id);

/**
 * @brief Release power bus (decrements reference count)
 * @param bus Power bus to release
 * @param consumer_id String identifier for the releasing consumer
 * @return ESP_OK on success
 */
esp_err_t fluctus_release_bus_power(power_bus_t bus, const char* consumer_id);

/**
 * @brief Check if power bus is currently enabled
 * @param bus Power bus to check
 * @return true if bus is powered, false otherwise
 */
bool fluctus_is_bus_powered(power_bus_t bus);

/**
 * @brief Get current power management state
 * @return Current fluctus_power_state_t
 */
fluctus_power_state_t fluctus_get_power_state(void);

/**
 * @brief Check if system is in safety shutdown
 * @return true if safety shutdown is active
 */
bool fluctus_is_safety_shutdown(void);

/**
 * @brief Manual reset from safety shutdown (requires external confirmation)
 * @return ESP_OK if reset successful, ESP_ERR_INVALID_STATE if not in shutdown
 */
esp_err_t fluctus_manual_safety_reset(void);

// ################ Solar Tracking Functions ################

/**
 * @brief Enable solar tracking system
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if power insufficient
 */
esp_err_t fluctus_enable_solar_tracking(void);

/**
 * @brief Disable solar tracking and park servos
 * @return ESP_OK on success
 */
esp_err_t fluctus_disable_solar_tracking(void);

/**
 * @brief Check if solar tracking is currently active
 * @return true if solar tracking is enabled and operational
 */
bool fluctus_is_solar_tracking_enabled(void);

/**
 * @brief Get current servo positions
 * @param[out] yaw_duty Current yaw servo duty cycle
 * @param[out] pitch_duty Current pitch servo duty cycle
 * @return ESP_OK on success
 */
esp_err_t fluctus_get_servo_positions(uint32_t *yaw_duty, uint32_t *pitch_duty);

// ################ Power Monitoring Functions ################

/**
 * @brief Get current power monitoring data (returns pointer to internal data)
 * @return Pointer to fluctus_monitoring_data_t structure, or NULL on error
 */
const fluctus_monitoring_data_t* fluctus_get_monitoring_data_ptr(void);

/**
 * @brief Get current power monitoring data (legacy function with copy)
 * @param[out] data Pointer to fluctus_monitoring_data_t structure to fill
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t fluctus_get_monitoring_data(fluctus_monitoring_data_t *data);

/**
 * @brief Get current battery voltage
 * @return Battery voltage in volts, or -1.0 on error
 */
float fluctus_get_battery_voltage(void);

/**
 * @brief Get total system current consumption
 * @return Total current in amps, or -1.0 on error  
 */
float fluctus_get_total_current(void);

// ################ System Status Functions ################

/**
 * @brief Get comprehensive system status (returns pointer to internal data)
 * @return Pointer to fluctus_power_status_t structure, or NULL on error
 */
const fluctus_power_status_t* fluctus_get_system_status_ptr(void);

/**
 * @brief Get comprehensive system status (legacy function with copy)
 * @param[out] status Pointer to fluctus_power_status_t structure to fill
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t fluctus_get_system_status(fluctus_power_status_t *status);

/**
 * @brief Set 12V cooling fan speed
 * @param duty_percent Fan speed percentage (0-100)
 * @return ESP_OK on success
 */
esp_err_t fluctus_set_cooling_fan_speed(uint8_t duty_percent);

/**
 * @brief Get current case temperature from DS18B20
 * @return Temperature in °C, or -127.0 on error or invalid reading
 */
float fluctus_get_case_temperature(void);

/**
 * @brief Get human-readable power state string
 * @param state Power state to convert
 * @return Constant string describing the power state
 */
const char* fluctus_power_state_to_string(fluctus_power_state_t state);

// ################ Power Metering Functions ################

/**
 * @brief Set PV INA219 power state
 * @param active true to activate, false to power down
 * @return ESP_OK on success
 */
esp_err_t fluctus_set_pv_ina_power(bool active);

/**
 * @brief Set Battery INA219 power state
 * @param active true to activate, false to power down
 * @return ESP_OK on success
 */
esp_err_t fluctus_set_battery_ina_power(bool active);

// ################ Snapshot Functions ################

/**
 * @brief Comprehensive snapshot structure for HMI/MQTT (single mutex operation)
 */
typedef struct {
    // Power buses
    bool bus_3v3_enabled;
    bool bus_5v_enabled;
    bool bus_6v6_enabled;
    bool bus_12v_enabled;
    uint8_t bus_3v3_consumers;
    uint8_t bus_5v_consumers;
    uint8_t bus_6v6_consumers;
    uint8_t bus_12v_consumers;

    // Battery monitoring
    float battery_voltage;
    float battery_current;
    float battery_soc_percent;    // Calculated SOC based on voltage
    float battery_power_w;

    // Solar monitoring
    float solar_voltage;
    float solar_current;
    float solar_power_w;
    bool solar_pv_active;

    // Solar tracking
    solar_tracking_state_t tracking_state;
    uint8_t yaw_position_percent;      // 0-100%
    uint8_t pitch_position_percent;     // 0-100%
    float yaw_error;                    // Tracking error in volts
    float pitch_error;

    // Thermal management
    float case_temperature;
    bool temperature_valid;
    uint8_t fan_speed_percent;

    // System state
    fluctus_power_state_t power_state;
    bool safety_shutdown;
    bool manual_reset_required;
    time_t last_activity_time;

    // Readings validity
    bool battery_data_valid;
    bool solar_data_valid;
    time_t snapshot_timestamp;
} fluctus_snapshot_t;

/**
 * @brief Get comprehensive system data snapshot (single mutex operation)
 * @param[out] snapshot Pointer to fluctus_snapshot_t structure to fill
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t fluctus_get_data_snapshot(fluctus_snapshot_t *snapshot);

// ################ Utility Macros ################

/**
 * @brief Macro to simplify power bus requests with error handling
 * @param bus The power bus to request
 * @param consumer Consumer ID string
 * @param fail_action Action to take on failure (e.g., "return ESP_FAIL" or "goto cleanup")
 */
#define FLUCTUS_REQUEST_BUS_OR_FAIL(bus, consumer, fail_action) \
    do { \
        esp_err_t _ret = fluctus_request_bus_power(bus, consumer); \
        if (_ret != ESP_OK) { \
            ESP_LOGE(TAG, "Failed to power on %s bus: %s", \
                     ((bus) == POWER_BUS_3V3) ? "3V3" : \
                     ((bus) == POWER_BUS_5V) ? "5V" : \
                     ((bus) == POWER_BUS_6V6) ? "6V6" : \
                     ((bus) == POWER_BUS_12V) ? "12V" : "UNKNOWN", \
                     esp_err_to_name(_ret)); \
            fail_action; \
        } \
    } while(0)

#endif // FLUCTUS_H