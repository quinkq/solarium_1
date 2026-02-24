#ifndef FLUCTUS_H
#define FLUCTUS_H

#include <stdbool.h>
#include <time.h>
#include "esp_err.h"
#include "driver/ledc.h"


// ########################## FLUCTUS Configuration ##########################

// Power Bus Control GPIOs - Hardware Configuration
// 3.3V: Open-drain output, LOW=ON (P-MOSFET with external pullup on gate)
// 5V:   Push-pull output, HIGH=ON (pulls converter EN to 3.3V via Schottky diode, 100kΩ pulldown=OFF)
// 6.6V: Open-drain output, LOW=ON (inverted logic buck, 100kΩ pullup to 3.3V)
// 12V:  Push-pull output, HIGH=ON (pulls converter EN to 3.3V via Schottky diode, 100kΩ pulldown=OFF)

#define FLUCTUS_3V3_BUS_ENABLE_GPIO         GPIO_NUM_4   // 3.3V non-essential sensor bus
#define FLUCTUS_5V_BUS_ENABLE_GPIO          GPIO_NUM_5   // 5V bus (MOSFET drivers + some sensors)
#define FLUCTUS_6V6_BUS_ENABLE_GPIO         GPIO_NUM_6   // 6.6V servo bus (inverted logic)
#define FLUCTUS_12V_BUS_ENABLE_GPIO         GPIO_NUM_7   // 12V bus enable (Valves + pump)

//#define FLUCTUS_3V3_HALL_ARRAY_ENABLE_GPIO  GPIO_NUM_7   // Legacy define - now MCP23008 GP5 (mcp23008_helper.h)
#define FLUCTUS_12V_FAN_ENABLE_GPIO         GPIO_NUM_3   // 12V Fan enable

// Level Shifter Configuration
// SN74AHCT125 Level Shifter Control GPIO (for 12V actuators: valves, pump, fan)
// Open-drain: LOW (pull to GND) = OE enabled (buffers active, 3.3V→5V translation)
//             FLOAT (input mode) = OE disabled via external pullup (buffers tri-state)
#define FLUCTUS_LEVEL_SHIFTER_OE_GPIO       GPIO_NUM_0   // SN74AHCT125 Output Enable (active-low)
#define FLUCTUS_LEVEL_SHIFTER_STABILIZATION_MS  10       // Stabilization delay after enabling buffers

// Solar Tracking Servo GPIOs
#define FLUCTUS_SERVO_PWM_YAW_GPIO          GPIO_NUM_47  // Yaw servo control
#define FLUCTUS_SERVO_PWM_PITCH_GPIO        GPIO_NUM_21  // Pitch servo control

// Temperature Monitoring GPIO
#define FLUCTUS_DS18B20_GPIO            GPIO_NUM_17  // DS18B20 OneWire temperature sensor

// Power bus consumer limits
#define FLUCTUS_POWER_BUS_MAX_CONSUMERS         16
#define FLUCTUS_LEVEL_SHIFTER_MAX_CONSUMERS     8
#define FLUCTUS_POWER_BUS_OFF_DELAY_MS          500  // Debounce delay, wait if another consumer requests the bus, prevents rapid power cycling

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
#define FLUCTUS_PHOTORESISTOR_THRESHOLD 0.010f       // Minimum error threshold (V)
#define FLUCTUS_SERVO_POWERUP_DELAY_MS  100          // Servo power stabilization

// Servo Control Task Parameters (Refactored smooth tracking)
#define FLUCTUS_SERVO_CONTROL_LOOP_MS           250     // Control loop period (250ms for smooth motion)
#define FLUCTUS_SERVO_MAX_STEP_PER_ITERATION    15      // Max duty change per iteration (slow, deliberate movement)
#define FLUCTUS_SERVO_SETTLING_TIME_MS          3000    // Time below threshold to declare convergence
#define FLUCTUS_SERVO_SETTLING_COUNT            (FLUCTUS_SERVO_SETTLING_TIME_MS / FLUCTUS_SERVO_CONTROL_LOOP_MS)  // 12 iterations

// Solar Tracking Automation Configuration
#define FLUCTUS_TRACKING_CORRECTION_INTERVAL_MS         (15 * 60 * 1000)  // 15 minutes between correction cycles
#define FLUCTUS_TRACKING_ADJUSTMENT_CYCLE_TIMEOUT_MS    30000             // 30 seconds max correction time (enforced by inner xTaskNotifyWait)
#define FLUCTUS_TRACKING_SUNRISE_BUFFER_MINUTES         30                // Start tracking 30 min before sunrise
#define FLUCTUS_TRACKING_SUNSET_BUFFER_MINUTES          30                // Allow disable 30 min before sunset

// Solar Tracking Parking Positions (duty cycle percentages)
#define FLUCTUS_SERVO_NIGHT_PARK_YAW_PERCENT        10.0f    // 10% from left (east-facing)
#define FLUCTUS_SERVO_NIGHT_PARK_PITCH_PERCENT      60.0f    // 60% upward from bottom
#define FLUCTUS_SERVO_ERROR_PARK_YAW_PERCENT        50.0f    // 50% center position (yaw)
#define FLUCTUS_SERVO_ERROR_PARK_PITCH_PERCENT      50.0f    // 50% center position (pitch)


// Fan PWM Configuration (Intel cooler with PWM input)
#define FLUCTUS_FAN_PWM_GPIO            GPIO_NUM_8  // 12V cooling fan PWM (Intel cooler)
#define FLUCTUS_FAN_PWM_FREQUENCY       25000        // 25kHz for 4-pin fan control
#define FLUCTUS_FAN_PWM_RESOLUTION      LEDC_TIMER_8_BIT   // 8-bit resolution (0-255)
#define FLUCTUS_FAN_PWM_TIMER           LEDC_TIMER_3
#define FLUCTUS_FAN_PWM_CHANNEL         LEDC_CHANNEL_3

// Temperature Monitoring Configuration
#define FLUCTUS_FAN_TURN_ON_TEMP_THRESHOLD      32.0f    // Turn on fan at 32°C
#define FLUCTUS_FAN_TURN_OFF_TEMP_THRESHOLD     28.0f    // Turn off fan at 28°C (hysteresis)
#define FLUCTUS_FAN_MAX_TEMP_THRESHOLD          42.0f    // 100% fan speed at 42°C
#define FLUCTUS_FAN_MIN_DUTY                    10       // Minimum fan speed (10% at 32°C)
#define FLUCTUS_FAN_MAX_DUTY                    100      // Maximum fan speed (100% at 42°C)
#define FLUCTUS_TEMP_ACTIVE_INTERVAL_MS         30000    // Read temp every 30s during active monitoring
#define FLUCTUS_TEMP_THERMAL_INTERVAL_MS        60000    // Read temp every 1min during thermal event
#define FLUCTUS_TEMP_CONVERSION_DELAY_MS        750      // DS18B20 12-bit conversion time

// Power Monitoring Configuration
#define FLUCTUS_POWER_ACTIVE_MONITOR_INTERVAL_MS    500     // Power reading interval when active
#define FLUCTUS_POWER_STEADY_MONITOR_INTERVAL_MS    (15 * 60 * 1000)   // 15 min when in steady state
#define FLUCTUS_POWER_STEADY_PROBE_DURATION_MS      15000    // 15 second probe duration in steady state
#define FLUCTUS_POWER_PERIODIC_CHECK_INTERVAL_MS (15 * 60 * 1000)  // 15 minutes when idle (daytime)
#define FLUCTUS_POWER_PERIODIC_CHECK_INTERVAL_NIGHT_MS (60 * 60 * 1000)  // 1 hour when idle (nighttime)

// Power Metering Configuration
#define FLUCTUS_SOLAR_LIGHT_THRESHOLD      0.4f    // Photoresistor threshold for nighttime (V)

// Tasks Notification Bits
#define FLUCTUS_NOTIFY_POWER_ACTIVITY        (1UL << 0)  // Power bus activity detected (monitoring task)
#define FLUCTUS_NOTIFY_SOLAR_ENABLE          (1UL << 1)  // Enable solar tracking (solar task)
#define FLUCTUS_NOTIFY_SOLAR_DISABLE         (1UL << 2)  // Disable solar tracking (solar task)
#define FLUCTUS_NOTIFY_SUNRISE               (1UL << 3)  // Sunrise detected - wake SLEEPING tracking (solar task)
#define FLUCTUS_NOTIFY_POWER_STATE_CHANGE    (1UL << 4)  // Power state change detected (core orchestration task)
#define FLUCTUS_NOTIFY_CONFIG_UPDATE         (1UL << 5)  // Configuration updated - recalculate timeouts (monitoring task)
#define FLUCTUS_NOTIFY_SERVO_CONVERGED       (1UL << 6)  // Servo control → state machine: tracking converged

// Battery Voltage Thresholds (12V AGM, 50% SOC = 0%)
#define FLUCTUS_BATTERY_LEVEL_POWER_SAVING    12.40f  // 40% SOC - Impose limit on STELLARIA
#define FLUCTUS_BATTERY_LEVEL_LOW_POWER       12.25f  // 25% SOC - Turn off STELLARIA
#define FLUCTUS_BATTERY_LEVEL_VERY_LOW        12.15f  // 15% SOC - Turn off IMPLUVIUM  
#define FLUCTUS_BATTERY_LEVEL_CRITICAL        12.00f  // 0% SOC  - Turn off TEMPESTA + Solar

#define FLUCTUS_STATE_DEBOUNCE_REQUIRED_COUNT 3  // Require 3 consecutive readings to change state

// Battery Voltage Hysteresis (+0.1V for turn-on)
#define FLUCTUS_BATTERY_HYSTERESIS            0.10f
#define FLUCTUS_BATTERY_LEVEL_POWER_SAVING_ON (FLUCTUS_BATTERY_LEVEL_POWER_SAVING + FLUCTUS_BATTERY_HYSTERESIS)
#define FLUCTUS_BATTERY_LEVEL_LOW_POWER_ON    (FLUCTUS_BATTERY_LEVEL_LOW_POWER + FLUCTUS_BATTERY_HYSTERESIS)
#define FLUCTUS_BATTERY_LEVEL_VERY_LOW_ON     (FLUCTUS_BATTERY_LEVEL_VERY_LOW + FLUCTUS_BATTERY_HYSTERESIS)
#define FLUCTUS_BATTERY_LEVEL_CRITICAL_ON     (FLUCTUS_BATTERY_LEVEL_CRITICAL + FLUCTUS_BATTERY_HYSTERESIS)

// Overcurrent Protection
#define FLUCTUS_OVERCURRENT_THRESHOLD_1       3.5f    // Amps, 5-second delay
#define FLUCTUS_OVERCURRENT_THRESHOLD_2       4.5f    // Amps, immediate shutdown
#define FLUCTUS_OVERCURRENT_DELAY_MS          5000    // Delay for threshold 1

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
    FLUCTUS_POWER_METER_STATE_ACTIVE = 0,      // Active monitoring (500ms sampling)
    FLUCTUS_POWER_METER_STATE_STEADY,          // Steady state (60s sampling with 3s probes)
    FLUCTUS_POWER_METER_STATE_PV_SHUTDOWN,     // PV INA in power-down mode (nighttime)
    FLUCTUS_POWER_METER_STATE_BATTERY_SHUTDOWN // Battery INA in power-down mode
} fluctus_power_meter_state_t;

typedef enum {
    SOLAR_TRACKING_DISABLED = 0,        // Solar tracking completely disabled (requires manual enable)
    SOLAR_TRACKING_SLEEPING,            // Parked for night, will auto-resume at sunrise
    SOLAR_TRACKING_STANDBY,             // Operational, awaiting next correction cycle (15-min intervals)
    SOLAR_TRACKING_CORRECTING,          // Actively correcting position
    SOLAR_TRACKING_PARKING,             // Moving to park position (transient state)
    SOLAR_TRACKING_ERROR                // Error state (timeout or failure)
} solar_tracking_state_t;

typedef struct {
    bool bus_enabled[POWER_BUS_COUNT];          // Individual bus states
    uint8_t bus_ref_count[POWER_BUS_COUNT];     // Reference counting for consumers
    char bus_consumers[POWER_BUS_COUNT][FLUCTUS_POWER_BUS_MAX_CONSUMERS][16]; // Consumer names per bus (DEFINE sets max per bus)
    fluctus_power_state_t power_state;          // Current power management state
    bool safety_shutdown;                       // Overcurrent safety shutdown flag
    bool manual_reset_required;                 // Manual reset flag for safety
    time_t last_activity_time;                  // Last consumer activity timestamp

    // SN74AHCT125 level shifter control (for 12V actuators: pump, valves, fan)
    bool level_shifter_enabled;                 // SN74AHCT125 output enable state
    uint8_t level_shifter_ref_count;            // Reference counting for level shifter
    char level_shifter_consumers[8][16];        // Consumer names for level shifter (max 8)
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
    bool fan_active;                    // Cooling fan active state
    bool thermal_monitoring_active;     // Thermal monitoring mode active (high frequency temp reads)
} fluctus_monitoring_data_t;


/**
 * @brief Solar tracking state structure - SOURCE OF TRUTH for all solar/servo data
 *
 * This is the single authoritative source for:
 * - Current servo positions (hardware state)
 * - Solar tracking state machine state
 * - Photoresistor readings and tracking errors
 *
 * Protected by xSolarMutex. Used for both:
 * - Internal decision making (servo control, state machine)
 * - Telemetry snapshot generation (MQTT, HMI)
 *
 * THREAD SAFETY: All reads/writes must hold xSolarMutex
 */
typedef struct {
    // Tracking state machine
    solar_tracking_state_t tracking_state;

    // Servo positions - SOURCE OF TRUTH for hardware state
    uint32_t current_yaw_duty;          // Raw servo PWM duty cycle (410-2048)
    uint32_t current_pitch_duty;        // Raw servo PWM duty cycle (410-2048)
    uint8_t yaw_position_percent;       // 0-100% (derived from duty for telemetry)
    uint8_t pitch_position_percent;     // 0-100% (derived from duty for telemetry)

    // Tracking errors (updated during correction cycles)
    float yaw_error;                    // Tracking error in volts (left-right differential)
    float pitch_error;                  // Tracking error in volts (top-bottom differential)

    // Photoresistor raw readings (TL, TR, BL, BR)
    float photoresistor_readings[4];    // Individual photoresistor voltages

    // Validity and timestamp
    bool valid;                         // Sensor reading validity flag
    time_t timestamp;                   // Last update timestamp

} fluctus_solar_data_t;

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
 * @brief Get current solar tracking state (lightweight, no snapshot fetch)
 * @return Current solar_tracking_state_t value
 * @note Thread-safe, uses quick mutex with 100ms timeout. Returns DISABLED on mutex timeout.
 */
solar_tracking_state_t fluctus_get_solar_tracking_state(void);

/**
 * @brief Manual reset from safety shutdown (requires external confirmation)
 * @return ESP_OK if reset successful, ESP_ERR_INVALID_STATE if not in shutdown
 */
esp_err_t fluctus_manual_safety_reset(void);

/**
 * @brief Set FLUCTUS power monitoring intervals (runtime adjustment)
 * @param day_min Daytime interval in minutes (5-60)
 * @param night_min Nighttime interval in minutes (15-120)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out of range
 * @note Updates configuration file and wakes monitoring task for immediate effect
 */
esp_err_t fluctus_set_power_intervals(uint32_t day_min, uint32_t night_min);

/**
 * @brief Set FLUCTUS solar tracking correction interval (runtime adjustment)
 * @param correction_min Correction cycle interval in minutes (5-60)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out of range
 * @note Updates configuration file, takes effect on next STANDBY cycle
 */
esp_err_t fluctus_set_solar_interval(uint32_t correction_min);

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
 * @brief Enable solar tracking debug mode (continuous operation for 90 seconds)
 *
 * Forces solar tracking to run continuously regardless of daytime conditions.
 * Useful for testing and debugging servo positioning and photoresistor readings.
 * Automatically times out after 90 seconds to prevent excessive servo wear.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if solar tracking is disabled
 */
esp_err_t fluctus_enable_solar_debug_mode(void);

/**
 * @brief Disable solar tracking debug mode (return to normal operation)
 * @return ESP_OK on success
 */
esp_err_t fluctus_disable_solar_debug_mode(void);

/**
 * @brief Check if solar debug mode is currently active
 * @return true if debug mode active, false otherwise
 */
bool fluctus_is_solar_debug_mode_active(void);

/**
 * @brief Set servo position directly for debug/testing purposes
 *
 * This function bypasses the solar tracking state machine and directly
 * sets the servo position. It also updates the internal state so the
 * position is correctly reflected in telemetry and HMI.
 *
 * Thread-safe with xSolarMutex (100ms timeout).
 *
 * @param channel LEDC channel (FLUCTUS_SERVO_YAW_CHANNEL or FLUCTUS_SERVO_PITCH_CHANNEL)
 * @param duty_cycle Desired duty cycle value (automatically clamped to FLUCTUS_SERVO_MIN_DUTY..FLUCTUS_SERVO_MAX_DUTY)
 * @return ESP_OK on success, ESP_FAIL on mutex timeout or hardware error
 */
esp_err_t fluctus_servo_debug_set_position(ledc_channel_t channel, uint32_t duty_cycle);

/**
 * @brief Set 12V cooling fan speed
 * @param duty_percent Fan speed percentage (0-100)
 * @return ESP_OK on success
 */
esp_err_t fluctus_set_cooling_fan_speed(uint8_t duty_percent);

/**
 * @brief Enable or disable hall array power (N-MOSFET control)
 * @param enable true to power on hall array, false to power off
 */
void fluctus_hall_array_enable(bool enable);

// ################ SN74AHCT125 Level Shifter Control ################

/**
 * @brief Request SN74AHCT125 level shifter enable (reference counted)
 *
 * Enables the SN74AHCT125 level shifter buffers that translate 3.3V logic to
 * 5V for driving 12V actuator MOSFETs (pump, valves, fan). Uses reference
 * counting to safely handle multiple concurrent actuator operations.
 * Includes 10ms stabilization delay on first enable.
 *
 * MUST be called before operating any 12V actuator to ensure proper signal levels.
 *
 * Thread-safe with xPowerBusMutex (1000ms timeout).
 *
 * @param consumer_id String identifier for the requesting consumer (max 15 chars)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if consumer_id is NULL
 * @return ESP_FAIL on mutex timeout
 */
esp_err_t fluctus_request_level_shifter(const char* consumer_id);

/**
 * @brief Release SN74AHCT125 level shifter (decrements reference count)
 *
 * Decrements the reference count for the level shifter. When count reaches
 * zero, the SN74AHCT125 is disabled (all outputs tri-state, saving power).
 *
 * Thread-safe with xPowerBusMutex (1000ms timeout).
 *
 * @param consumer_id String identifier for the releasing consumer
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if consumer_id is NULL
 * @return ESP_FAIL on mutex timeout
 */
esp_err_t fluctus_release_level_shifter(const char* consumer_id);

/**
 * @brief Get human-readable power state string
 * @param state Power state to convert
 * @return Constant string describing the power state
 */
const char* fluctus_power_state_to_string(fluctus_power_state_t state);

// ################ Energy Accumulator Structure ################

/**
 * @brief Power accumulator structure (stored in RTC RAM for persistence)
 * Survives ESP32 resets but not power loss
 */
typedef struct {
    // Hourly tracking (reset at top of hour)
    time_t current_hour_start;              // Start time of current hour
    float pv_energy_wh_accumulator;         // Hourly PV energy (Wh)
    float battery_energy_wh_accumulator;    // Hourly battery consumption (Wh)
    float pv_peak_w_hour;                   // Hourly peak power
    float battery_peak_w_hour;              // Hourly peak consumption

    // Daily tracking (reset at midnight)
    time_t current_day_start;               // Day start timestamp
    float pv_produced_wh_day;                 // Daily PV energy generation total
    float battery_consumed_wh_day;          // Daily battery consumption
    float pv_peak_w_day;                    // Daily peak generation
    float battery_peak_w_day;               // Daily peak consumption
    uint8_t hours_active_day;               // Hours with PV generation

    // 15-minute averaging (for normal mode publishing)
    time_t interval_start_15min;            // 15-min interval start time
    float pv_power_sum_15min;               // Sum of power readings
    float pv_voltage_sum_15min;             // Sum of voltage readings
    float pv_current_sum_15min;             // Sum of current readings
    float battery_power_sum_15min;          // Sum of battery power
    float battery_voltage_sum_15min;        // Sum of battery voltage
    float battery_current_sum_15min;        // Sum of battery current
    uint16_t sample_count_15min;            // Number of samples taken

    // Metadata
    time_t last_sample_time;                // Last sample timestamp
    bool initialized;                       // Accumulator initialization flag
} power_accumulator_rtc_t;

// ################ Snapshot Functions ################

/**
 * @brief Unified FLUCTUS snapshot structure for HMI display and MQTT distribution
 *
 * LAYERED WRITE ARCHITECTURE:
 * - Normal write (15min/60min): Updates ALL fields as baseline
 * - Realtime write (500ms): Overwrites dynamic subset only (26 fields marked "RT")
 *
 * MQTT ENCODING:
 * - TELEMETRY_SRC_FLUCTUS → msgpack_encode_fluctus() encodes all fields (QoS1, retained)
 * - TELEMETRY_SRC_FLUCTUS_RT → msgpack_encode_fluctus_rt() encodes RT subset (QoS0, streaming)
 *
 * HMI ACCESS:
 * - All pages call single API: telemetry_get_fluctus_data()
 * - Always returns freshest data (RT fields auto-refreshed at 500ms)
 * - Pages choose which fields to display (instantaneous vs averages)
 */
typedef struct {
    // ============ Power Buses (8 fields) ============
    // Written by: Normal + RT (RT overwrites for fresher data)
    bool bus_3v3_enabled;              // RT: Current bus state
    bool bus_5v_enabled;               // RT: Current bus state
    bool bus_6v6_enabled;              // RT: Current bus state
    bool bus_12v_enabled;              // RT: Current bus state
    uint8_t bus_3v3_consumers;         // RT: Active consumer count
    uint8_t bus_5v_consumers;          // RT: Active consumer count
    uint8_t bus_6v6_consumers;         // RT: Active consumer count
    uint8_t bus_12v_consumers;         // RT: Active consumer count

    // ============ Battery - Instantaneous (4 fields) ============
    // Written by: RT only (500ms updates)
    float battery_voltage_inst;        // RT: Instantaneous voltage (V)
    float battery_current_inst;        // RT: Instantaneous current (A)
    float battery_power_inst;          // RT: Instantaneous power (W)
    float battery_soc_inst;            // RT: Instantaneous SOC (%) - DEBUG field

    // ============ Battery - 15-Minute Averages (4 fields) ============
    // Written by: Normal only (15min/60min updates)
    float battery_voltage_avg_15min;   // Normal: 15-minute rolling average (V)
    float battery_current_avg_15min;   // Normal: 15-minute rolling average (A)
    float battery_power_avg_15min;     // Normal: 15-minute rolling average (W)
    float battery_soc_avg_15min;       // Normal: 15-minute rolling average (%)

    // ============ Solar - Instantaneous (4 fields) ============
    // Written by: RT only (500ms updates)
    float solar_voltage_inst;          // RT: Instantaneous PV voltage (V)
    float solar_current_inst;          // RT: Instantaneous PV current (A)
    float solar_power_inst;            // RT: Instantaneous PV power (W)
    bool solar_pv_active;              // RT: INA219 power state

    // ============ Solar - 15-Minute Averages (3 fields) ============
    // Written by: Normal only (15min/60min updates)
    float solar_voltage_avg_15min;     // Normal: 15-minute rolling average (V)
    float solar_current_avg_15min;     // Normal: 15-minute rolling average (A)
    float solar_power_avg_15min;       // Normal: 15-minute rolling average (W)

    // ============ Solar Tracking - Position (3 fields) ============
    // Written by: Normal + RT (RT overwrites for fresher position)
    solar_tracking_state_t tracking_state;  // RT: Current state machine state
    uint8_t yaw_position_percent;           // RT: Azimuth position 0-100%
    uint8_t pitch_position_percent;         // RT: Elevation position 0-100%

    // ============ Solar Tracking - Debug (5 fields) ============
    // Written by: RT only (500ms updates)
    float yaw_error;                   // RT: Tracking error in volts
    float pitch_error;                 // RT: Tracking error in volts
    uint32_t current_yaw_duty;         // RT: DEBUG - Raw servo PWM duty cycle
    uint32_t current_pitch_duty;       // RT: DEBUG - Raw servo PWM duty cycle
    float photoresistor_readings[4];   // RT: DEBUG - [TL, TR, BL, BR] voltages

    // ============ Thermal Management (3 fields) ============
    // Written by: Normal + RT (RT overwrites for fresher temp)
    float case_temperature;            // RT: Current case temperature (°C)
    bool temperature_valid;            // RT: DS18B20 reading validity
    uint8_t fan_speed_percent;         // RT: Current fan PWM duty (0-100%)

    // ============ Energy Statistics - Hourly (5 fields) ============
    // Written by: Normal only (updated at 15min intervals, reset hourly)
    time_t current_hour_start;         // Normal: Start timestamp of current hour
    float pv_produced_wh_hour;           // Normal: Hourly PV energy total (Wh)
    float battery_consumed_wh_hour;      // Normal: Hourly battery consumption (Wh)
    float pv_peak_w_hour;              // Normal: Hourly peak PV power (W)
    float battery_peak_w_hour;         // Normal: Hourly peak battery power (W)

    // ============ Energy Statistics - Daily (6 fields) ============
    // Written by: Normal only (updated at 15min intervals, reset daily)
    time_t current_day_start;          // Normal: Start timestamp of current day
    float pv_produced_wh_day;            // Normal: Daily PV energy total (Wh)
    float battery_consumed_wh_day;     // Normal: Daily battery consumption (Wh)
    float pv_peak_w_day;               // Normal: Daily peak PV power (W)
    float battery_peak_w_day;          // Normal: Daily peak battery power (W)
    uint8_t hours_active_day;          // Normal: Hours with PV generation today

    // ============ System State (4 fields) ============
    // Written by: Normal only (changes infrequently)
    fluctus_power_state_t power_state; // Normal: Load shedding state
    bool safety_shutdown;              // Normal: Overcurrent shutdown flag
    bool manual_reset_required;        // Normal: User intervention needed
    time_t last_activity_time;         // Normal: Last significant state change

    // ============ Validity Flags (2 fields) ============
    // Written by: Normal + RT (RT overwrites for current validity)
    bool battery_data_valid;           // RT: INA219 battery reading valid
    bool solar_data_valid;             // RT: INA219 solar reading valid

    // ============ Metadata (1 field) ============
    // Written by: Normal + RT (RT overwrites with latest timestamp)
    time_t snapshot_timestamp;         // RT: Last update time

} fluctus_snapshot_t;

/**
 * @brief Write FLUCTUS data directly to TELEMETRY cache (full snapshot)
 * Comprehensive snapshot for HMI/MQTT distribution with all fields.
 * Includes nested solar tracking data with photoresistor readings from global cache.
 * Writes directly to TELEMETRY's cache buffer to eliminate intermediate copy.
 * Only FLUCTUS mutex needed - TELEMETRY lock/unlock handled by caller.
 *
 * @param cache Pointer to TELEMETRY's fluctus_snapshot_t buffer
 * @return ESP_OK on success, ESP_FAIL if mutex timeout
 */
esp_err_t fluctus_write_to_telemetry_cache(fluctus_snapshot_t *cache);

/**
 * @brief Write FLUCTUS realtime data to TELEMETRY cache (rich snapshot)
 * High-frequency power monitoring snapshot for realtime MQTT streaming (QoS 0).
 * Contains instantaneous sensor readings and full debug data (photoresistors, duty cycles).
 * Optimized for 500ms update rate during active monitoring.
 *
 * @param cache Pointer to TELEMETRY's fluctus_snapshot_rt_t buffer
 * @return ESP_OK on success, ESP_FAIL if mutex timeout
 */
esp_err_t fluctus_write_realtime_to_telemetry_cache(fluctus_snapshot_t *cache);

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

#endif  // FLUCTUS_H