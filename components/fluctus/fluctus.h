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

// Power Bus Control GPIOs (Open-drain with external 10kΩ pullups)
// Note: LOW = Buck enabled, FLOAT = Buck disabled
#define FLUCTUS_BUCK_3V3_ENABLE_GPIO    GPIO_NUM_7   // 3.3V sensor bus
#define FLUCTUS_BUCK_5V_ENABLE_GPIO     GPIO_NUM_8   // 5V bus (MOSFET drivers)
#define FLUCTUS_BUCK_6V2_ENABLE_GPIO    GPIO_NUM_9   // 6.2V servo bus
#define FLUCTUS_BUCK_12V_ENABLE_GPIO    GPIO_NUM_10  // 12V bus enable
#define FLUCTUS_12V_FAN_PWM_GPIO        GPIO_NUM_48  // 12V cooling fan PWM

// Solar Tracking Servo GPIOs
#define FLUCTUS_SERVO_YAW_GPIO          GPIO_NUM_45  // Yaw servo control
#define FLUCTUS_SERVO_PITCH_GPIO        GPIO_NUM_46  // Pitch servo control

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
#define FLUCTUS_TRACKING_INTERVAL_MS    5000         // Tracking update interval
#define FLUCTUS_SERVO_POWERUP_DELAY_MS  100          // Servo power stabilization

// Fan PWM Configuration  
#define FLUCTUS_FAN_PWM_FREQUENCY       25000        // 25kHz for fan control
#define FLUCTUS_FAN_PWM_RESOLUTION      LEDC_TIMER_8_BIT   // 8-bit resolution
#define FLUCTUS_FAN_PWM_TIMER           LEDC_TIMER_3
#define FLUCTUS_FAN_PWM_CHANNEL         LEDC_CHANNEL_3

// Power Monitoring Configuration
#define FLUCTUS_POWER_MONITOR_INTERVAL_MS    500     // Power reading interval when active
#define FLUCTUS_POWER_MONITOR_IDLE_INTERVAL_MS (15 * 60 * 1000)  // 15 minutes when idle

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
#define FLUCTUS_OVERCURRENT_THRESHOLD_1       1.5f    // Amps, 3-second delay
#define FLUCTUS_OVERCURRENT_THRESHOLD_2       3.0f    // Amps, immediate shutdown
#define FLUCTUS_OVERCURRENT_DELAY_MS          3000    // Delay for threshold 1

// Mutex timeout constants
#define FLUCTUS_MUTEX_TIMEOUT_QUICK_MS        100     // Quick operations
#define FLUCTUS_MUTEX_TIMEOUT_LONG_MS         1000    // Longer operations

// ########################## Data Structures ##########################

typedef enum {
    POWER_BUS_3V3 = 0,          // 3.3V sensor bus
    POWER_BUS_5V,               // 5V bus (MOSFET drivers)  
    POWER_BUS_6V2,              // 6.2V servo bus
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

typedef struct {
    bool bus_enabled[POWER_BUS_COUNT];          // Individual bus states
    uint8_t bus_ref_count[POWER_BUS_COUNT];     // Reference counting for consumers
    char bus_consumers[POWER_BUS_COUNT][8][16]; // Consumer names per bus (max 8 per bus)
    fluctus_power_state_t power_state;          // Current power management state
    bool solar_tracking_enabled;                // Solar tracking system state
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
 * @brief Get human-readable power state string
 * @param state Power state to convert
 * @return Constant string describing the power state
 */
const char* fluctus_power_state_to_string(fluctus_power_state_t state);

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
                     ((bus) == POWER_BUS_6V2) ? "6V2" : \
                     ((bus) == POWER_BUS_12V) ? "12V" : "UNKNOWN", \
                     esp_err_to_name(_ret)); \
            fail_action; \
        } \
    } while(0)

#endif // FLUCTUS_H