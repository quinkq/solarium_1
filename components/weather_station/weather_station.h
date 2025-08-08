#ifndef WEATHER_STATION_H
#define WEATHER_STATION_H

#include <stdbool.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/uart.h"

// ########################## Weather Station Configuration ##########################

// Timing Configuration
#define WEATHER_COLLECTION_INTERVAL_MS (15 * 60 * 1000) // 15 minutes
#define WEATHER_COLLECTION_INTERVAL_POWER_SAVE_MS (60 * 60 * 1000) // 60 minutes (power save mode)
#define WEATHER_AS5600_SAMPLE_DURATION_MS (5 * 1000)    // 5 seconds for wind speed
#define WEATHER_AS5600_SAMPLE_INTERVAL_MS (100)         // 10Hz sampling
#define WEATHER_PMS5003_WARMUP_TIME_MS (35 * 1000)      // 35s warmup (30s + safety)

// Sensor Configuration
#define WEATHER_AVERAGING_SAMPLES 10  // Historical samples for temperature averaging
#define WEATHER_INVALID_VALUE -999.9f // Invalid sensor reading marker

// Mutex timeout constants
#define WEATHER_MUTEX_TIMEOUT_QUICK_MS 100   // For simple reads
#define WEATHER_MUTEX_TIMEOUT_UPDATE_MS 1000 // For data updates/init

// GPIO Configuration
#define WEATHER_RAINFALL_GPIO 4             // Hall sensor for rainfall (GPIO4)
#define WEATHER_PMS5003_TX_GPIO 43          // PMS5003 UART TX (GPIO43)
#define WEATHER_PMS5003_RX_GPIO 44          // PMS5003 UART RX (GPIO44)
#define WEATHER_PMS5003_UART_NUM UART_NUM_1 // Use UART1
#define WEATHER_PMS5003_RETRY_COUNT 3       // Count of retries for PMS5003


// Rain Sensor Configuration
#define WEATHER_RAIN_MM3_PER_PULSE 0.2f         // Volume per tip in cubic mm (to be calibrated)
#define WEATHER_RAIN_COLLECTION_AREA_MM2 78.54f // Collection area in mm² (to be measured)

// Wind Sensor Configuration
#define WEATHER_WIND_RPM_TO_MS_FACTOR 1.0f      // Conversion factor RPM to m/s (to be calibrated with triple cup anemometer)

// ########################## Data Structures ##########################

typedef enum {
    WEATHER_SENSOR_OK = 0,
    WEATHER_SENSOR_UNAVAILABLE,
    WEATHER_SENSOR_ERROR,
    WEATHER_SENSOR_WARMING_UP
} weather_sensor_status_t;

typedef struct {
    float temperature;      // Averaged temperature in °C
    float humidity;         // Current humidity in %
    float pressure;         // Current pressure in hPa
    float air_quality_pm25; // PM2.5 in ug/m3
    float air_quality_pm10; // PM10 in ug/m3
    float wind_speed_rpm;   // Wind speed in RPM
    float wind_speed_ms;    // Wind speed in m/s
    float rainfall_mm;      // Accumulated rainfall in mm since last reset
    time_t timestamp;       // Timestamp of last complete measurement cycle

    // Sensor status tracking
    weather_sensor_status_t temp_sensor_status;
    weather_sensor_status_t humidity_sensor_status;
    weather_sensor_status_t pressure_sensor_status;
    weather_sensor_status_t air_quality_status;
    weather_sensor_status_t wind_sensor_status;
    weather_sensor_status_t rain_sensor_status;
} WeatherData;

typedef struct {
    // Temperature averaging
    float temperature_history[WEATHER_AVERAGING_SAMPLES];
    uint8_t temp_history_index;
    uint8_t temp_history_count;

    // Humidity averaging
    float humidity_history[WEATHER_AVERAGING_SAMPLES];
    uint8_t humidity_history_index;
    uint8_t humidity_history_count;

    // Rainfall tracking for hourly calculations
    time_t last_rainfall_reset_time;
    float rainfall_last_hour;
} weather_calculation_data_t;

// ########################## Public API Functions ##########################

/**
 * @brief Initialize weather station component
 * @return ESP_OK on success, ESP_FAIL on initialization failure
 */
esp_err_t weather_station_init(void);

/**
 * @brief Get current weather data (thread-safe)
 * @param[out] data Pointer to WeatherData structure to fill
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t weather_get_data(WeatherData *data);

/**
 * @brief Get current temperature (thread-safe)
 * @return Current averaged temperature in °C, or WEATHER_INVALID_VALUE on error
 */
float weather_get_temperature(void);

/**
 * @brief Get current humidity (thread-safe)
 * @return Current humidity in %, or WEATHER_INVALID_VALUE on error
 */
float weather_get_humidity(void);

/**
 * @brief Get current pressure (thread-safe)
 * @return Current pressure in hPa, or WEATHER_INVALID_VALUE on error
 */
float weather_get_pressure(void);

/**
 * @brief Get current air quality readings (thread-safe)
 * @param[out] pm25 PM2.5 in ug/m3
 * @param[out] pm10 PM10 in ug/m3
 * @return ESP_OK on success, ESP_FAIL if air quality sensor unavailable
 */
esp_err_t weather_get_air_quality(float *pm25, float *pm10);

/**
 * @brief Get current wind speed in RPM (thread-safe)
 * @return Wind speed in RPM, or WEATHER_INVALID_VALUE on error
 */
float weather_get_wind_speed_rpm(void);

/**
 * @brief Get current wind speed in m/s (thread-safe)
 * @return Wind speed in m/s, or WEATHER_INVALID_VALUE on error
 */
float weather_get_wind_speed_ms(void);

/**
 * @brief Get accumulated rainfall and optionally reset counter (thread-safe)
 * @param reset_counter If true, reset the rainfall accumulator after reading
 * @return Accumulated rainfall in mm, or WEATHER_INVALID_VALUE on error
 */
float weather_get_rainfall_mm(bool reset_counter);

/**
 * @brief Convert pulse count to rainfall depth in mm
 * Uses tipbucket volume and collection area to calculate accurate rainfall
 * @param pulse_count Number of tipbucket pulses recorded
 * @return Rainfall depth in mm
 */
float weather_convert_pulses_to_rainfall_mm(int pulse_count);

/**
 * @brief Get human-readable sensor status
 * @param[out] status_buffer Buffer to write status string
 * @param buffer_size Size of status buffer
 * @return ESP_OK on success
 */
esp_err_t weather_get_status_string(char *status_buffer, size_t buffer_size);

/**
 * @brief Get current pulse count from rainfall sensor (for diagnostics)
 * @param[out] pulse_count Current pulse counter value
 * @return ESP_OK on success, ESP_FAIL on sensor error
 */
esp_err_t weather_get_rainfall_pulse_count(int *pulse_count);

/**
 * @brief Set power saving mode for TEMPESTA weather station
 * @param enable True to enable power save mode (60min polling interval), false for normal operation
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t tempesta_set_power_save_mode(bool enable);

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

#endif // WEATHER_STATION_H