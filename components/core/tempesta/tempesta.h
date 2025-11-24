#ifndef TEMPESTA_H
#define TEMPESTA_H

#include <stdbool.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/uart.h"

// ########################## Weather Station Configuration ##########################

// Timing Configuration
// Probing intervals in interval_config
#define WEATHER_AS5600_SAMPLE_DURATION_MS (5 * 1000)               // 5 seconds for wind speed
#define WEATHER_AS5600_SAMPLE_INTERVAL_MS (100)                    // 10Hz sampling
#define WEATHER_PMS5003_WARMUP_TIME_MS (35 * 1000)                 // 35s warmup (30s + safety)

// Sensor Configuration
#define WEATHER_AVERAGING_SAMPLES 10  // Historical samples for temperature averaging
#define WEATHER_INVALID_VALUE -999.9f // Invalid sensor reading marker

// Mutex timeout constants
#define WEATHER_MUTEX_TIMEOUT_QUICK_MS 100   // For simple reads
#define WEATHER_MUTEX_TIMEOUT_UPDATE_MS 1000 // For data updates/init

// GPIO Configuration
#define WEATHER_RAINFALL_GPIO GPIO_NUM_3    // Tipbucket pulse counter for rainfall measurement
#define WEATHER_TANK_INTAKE_GPIO GPIO_NUM_8 // Tipbucket pulse counter for water tank ingress
#define WEATHER_PMS5003_TX_GPIO GPIO_NUM_43 // PMS5003 UART TX (GPIO43)
#define WEATHER_PMS5003_RX_GPIO GPIO_NUM_44 // PMS5003 UART RX (GPIO44)
#define WEATHER_PMS5003_UART_NUM UART_NUM_2 // UART2 (UART0 reserved for console)
#define WEATHER_PMS5003_RETRY_COUNT 3       // Count of retries for PMS5003

// Rain Sensor Configuration (Tipbucket #1)
#define WEATHER_RAIN_ML_PER_PULSE 100.0f        // Volume per tip in mm³ (50mL bucket, 2 tips)
#define WEATHER_RAIN_COLLECTION_AREA_MM2 78.54f // Collection area in mm² (to be measured)
// Tank Intake Sensor Configuration (Tipbucket #2)
#define WEATHER_TANK_INTAKE_ML_PER_PULSE 50.0f // Volume per tip in mL (50mL bucket)

// Wind Sensor Configuration
#define WEATHER_WIND_RPM_TO_MS_FACTOR 1.0f // Conversion factor RPM to m/s (to be calibrated with triple cup anemometer)

// Wind Direction Configuration (Hall Sensor Array - ADS1115 Device #3)
#define WEATHER_WIND_DIR_ADS1115_DEVICE 3                // ADS1115 device ID (0x4B - Hall_array)
#define WEATHER_WIND_DIR_HALL_NORTH_CH ADS111X_MUX_0_GND // ADS1115 channel for North sensor
#define WEATHER_WIND_DIR_HALL_EAST_CH ADS111X_MUX_1_GND  // ADS1115 channel for East sensor
#define WEATHER_WIND_DIR_HALL_SOUTH_CH ADS111X_MUX_2_GND // ADS1115 channel for South sensor
#define WEATHER_WIND_DIR_HALL_WEST_CH ADS111X_MUX_3_GND  // ADS1115 channel for West sensor
#define WEATHER_WIND_DIR_MAX_VOLTAGE 2.5f                // Maximum voltage when magnet directly overhead (V)
#define WEATHER_WIND_DIR_THRESHOLD_VOLTAGE 0.2f          // Minimum voltage to consider sensor active (V)

// ########################## Data Structures ##########################

typedef enum {
    WEATHER_SENSOR_OK = 0,
    WEATHER_SENSOR_UNAVAILABLE,
    WEATHER_SENSOR_ERROR,
    WEATHER_SENSOR_WARMING_UP
} weather_sensor_status_t;

/**
 * @brief TEMPESTA weather station operational states
 *
 * State machine for weather station control and monitoring.
 * Provides uniform interface matching IMPLUVIUM and STELLARIA components.
 */
typedef enum {
    TEMPESTA_STATE_DISABLED = 0, // System disabled (manual or init)
    TEMPESTA_STATE_IDLE,         // Normal operation between readings
    TEMPESTA_STATE_READING,      // Actively polling sensors (includes PMS5003 warmup)
    TEMPESTA_STATE_SHUTDOWN,     // Load shedding forced shutdown (FLUCTUS VERY_LOW power)
    TEMPESTA_STATE_ERROR         // Sensor errors requiring user attention
} tempesta_state_t;

typedef struct {
    float temperature;      // Averaged temperature in °C
    float humidity;         // Current humidity in %
    float pressure;         // Current pressure in hPa
    float air_quality_pm25; // PM2.5 in ug/m3
    float air_quality_pm10; // PM10 in ug/m3

    // Wind measurements
    float wind_speed_rpm;                // Wind speed in RPM
    float wind_speed_ms;                 // Wind speed in m/s
    float wind_direction_deg;            // Wind direction in degrees (0-360, 0=North, clockwise)
    const char *wind_direction_cardinal; // Wind direction (N/NE/E/SE/S/SW/W/NW)

    // Rainfall measurements (rain gauge) - weekly reset based on boot day
    float rainfall_last_hour_mm;    // Last completed hour measurement (mm, stable value)
    float rainfall_current_hour_mm; // Current hour accumulation so far (mm, for debug/monitoring)
    float rainfall_daily_mm;        // Rainfall accumulated today (mm, reset at midnight)
    float rainfall_weekly_mm;       // Rainfall accumulated this week (mm, reset on boot day)

    // Tank intake measurements - weekly reset based on boot day
    float tank_intake_last_hour_ml;    // Last completed hour measurement (mL, stable value)
    float tank_intake_current_hour_ml; // Current hour accumulation so far (mL, for debug/monitoring)
    float tank_intake_daily_ml;        // Tank intake accumulated today (mL, reset at midnight)
    float tank_intake_weekly_ml;       // Tank intake accumulated this week (mL, reset on boot day)

    // System state
    tempesta_state_t state; // Current operational state
    bool power_save_mode;   // Power saving mode active (60min cycle vs 15min)

    // Sensor status tracking
    weather_sensor_status_t temp_sensor_status;
    weather_sensor_status_t humidity_sensor_status;
    weather_sensor_status_t pressure_sensor_status;
    weather_sensor_status_t air_quality_status;
    weather_sensor_status_t wind_sensor_status;
    weather_sensor_status_t wind_direction_status;
    weather_sensor_status_t rain_gauge_status;
    weather_sensor_status_t tank_intake_status;

    time_t snapshot_timestamp;
} tempesta_snapshot_t;

typedef struct {
    // Temperature averaging
    float temperature_history[WEATHER_AVERAGING_SAMPLES];
    uint8_t temp_history_index;
    uint8_t temp_history_count;

    // Humidity averaging
    float humidity_history[WEATHER_AVERAGING_SAMPLES];
    uint8_t humidity_history_index;
    uint8_t humidity_history_count;

    // Rainfall tracking
    int64_t rainfall_hourly_start_time_ms; // Monotonic time when hourly tracking started
    int rainfall_hourly_start_pulses;      // Pulse count at start of hour
    int rainfall_daily_base_pulses;        // Pulse count at last midnight (for daily calculation)
    int rainfall_weekly_base_pulses;       // Pulse count at last Monday midnight (for weekly calculation)

    // Tank intake tracking
    int64_t tank_intake_hourly_start_time_ms; // Monotonic time when hourly tracking started
    int tank_intake_hourly_start_pulses;      // Pulse count at start of hour
    int tank_intake_daily_base_pulses;        // Pulse count at last midnight (for daily calculation)
    int tank_intake_weekly_base_pulses;       // Pulse count at last Monday midnight (for weekly calculation)
} weather_calculation_data_t;

// ########################## Public API Functions ##########################

/**
 * @brief Initialize weather station component
 * @return ESP_OK on success, ESP_FAIL on initialization failure
 */
esp_err_t tempesta_station_init(void);

/**
 * @brief Get comprehensive weather data snapshot (single mutex operation)
 * @param[out] data Pointer to tempesta_snapshot_t structure to fill
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t tempesta_get_data_snapshot(tempesta_snapshot_t *data);

/**
 * @brief Write TEMPESTA data directly to TELEMETRY cache
 * Comprehensive snapshot for HMI/MQTT distribution with all fields.
 * Writes directly to TELEMETRY's cache buffer to eliminate intermediate copy.
 * Only TEMPESTA mutex needed - TELEMETRY lock/unlock handled by caller.
 *
 * @param cache Pointer to TELEMETRY's tempesta_snapshot_t buffer
 * @return ESP_OK on success, ESP_FAIL if mutex timeout
 */
esp_err_t tempesta_write_to_telemetry_cache(tempesta_snapshot_t *cache);

/**
 * @brief Get current temperature for IMPLUVIUM integration (thread-safe)
 * @note This is a legacy function for irrigation temperature correction
 * @return Current averaged temperature in °C, or WEATHER_INVALID_VALUE on error
 */
float tempesta_get_temperature(void);

/**
 * @brief Set power saving mode for TEMPESTA weather station
 * @param enable True to enable power save mode (60min polling interval), false for normal (15min)
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 * @note This is a flag modifier, independent of the state machine. Works when state is IDLE.
 */
esp_err_t tempesta_set_power_save_mode(bool enable);

/**
 * @brief Set TEMPESTA collection intervals (runtime adjustment)
 * @param normal_min Normal mode interval in minutes (5-60)
 * @param power_save_min Power save mode interval in minutes (15-120)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out of range
 * @note Updates configuration file and timer period immediately if system is operational
 */
esp_err_t tempesta_set_collection_intervals(uint32_t normal_min, uint32_t power_save_min);

/**
 * @brief Set shutdown state for TEMPESTA weather station (for load shedding)
 * @param shutdown True to shutdown weather monitoring, false to restore
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t tempesta_set_shutdown(bool shutdown);

/**
 * @brief Enable/disable PMS5003 air quality sensor (for load shedding)
 * @param enable True to enable PMS5003 readings, false to skip them
 * @return ESP_OK on success
 */
esp_err_t tempesta_set_pms5003_enabled(bool enable);

/**
 * @brief Enable/disable TEMPESTA weather station system
 * @param enable True to enable weather monitoring, false to disable
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t tempesta_set_system_enabled(bool enable);

/**
 * @brief Get current TEMPESTA operational state (lightweight, no snapshot fetch)
 * @return Current tempesta_state_t value
 * @note Thread-safe, uses quick mutex with 100ms timeout. Returns DISABLED on mutex timeout.
 */
tempesta_state_t tempesta_get_state(void);

/**
 * @brief Force immediate data collection cycle (outside normal 15-min schedule)
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized or disabled
 */
esp_err_t tempesta_force_collection(void);

/**
 * @brief Reset daily accumulation counters (rainfall and tank intake)
 * @return ESP_OK on success
 */
esp_err_t tempesta_reset_daily_counters(void);

/**
 * @brief Reset weekly accumulation counters (rainfall and tank intake)
 * @return ESP_OK on success
 */
esp_err_t tempesta_reset_weekly_counters(void);

/**
 * @brief Reset rain gauge (clears hardware counter)
 * WARNING: Also resets hourly/daily/weekly. Use daily/weekly reset instead unless calibrating.
 * @return ESP_OK on success
 */
esp_err_t tempesta_reset_rain_gauge_total(void);

/**
 * @brief Reset tank intake (clears hardware counter)
 * WARNING: Also resets hourly/daily/weekly. Use daily/weekly reset instead unless calibrating.
 * @return ESP_OK on success
 */
esp_err_t tempesta_reset_tank_intake_total(void);

/**
 * @brief Reset all accumulation counters (clears both hardware counters)
 * WARNING: Also resets hourly/daily/weekly for both sensors. Use daily/weekly reset instead.
 * @return ESP_OK on success
 */
esp_err_t tempesta_reset_all_counters(void);

#endif // TEMPESTA_H