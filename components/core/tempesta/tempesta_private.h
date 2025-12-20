/**
 * @file tempesta_private.h
 * @brief TEMPESTA internal shared declarations for modularized implementation
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Central header for internal TEMPESTA module communication:
 * - Shared global variable extern declarations (mutexes, data structures)
 * - Internal function declarations across modules (sensors, processing, etc.)
 * - Private type definitions (pms5003_data_t, consolidated_sensor_data_t)
 * - Internal constants and helper macros
 *
 * This header provides a single source of truth for cross-module declarations
 * while avoiding circular dependencies. Included by all TEMPESTA .c modules.
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#ifndef TEMPESTA_PRIVATE_H
#define TEMPESTA_PRIVATE_H

#include "tempesta.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "sht4x.h"
#include "bmp280.h"
#include "as5600.h"

// Macro for updating weather data with mutex protection (use only in error paths where mutex is NOT already held)
#define WEATHER_UPDATE_DATA_WITH_STATUS_ERROR(field, value, status_field, status_value) do { \
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) { \
        weather_data.field = (value); \
        weather_data.status_field = (status_value); \
        xSemaphoreGive(xTempestaDataMutex); \
    } else { \
        ESP_LOGW(TAG, "Failed to acquire mutex for " #field " data update"); \
        return ESP_FAIL; \
    } \
} while(0)


// ########################## Global Variables ##########################

// Thread-safe data protection
extern SemaphoreHandle_t xTempestaDataMutex;

extern tempesta_snapshot_t weather_data;
extern weather_calculation_data_t calculation_data;

// State machine
extern tempesta_state_t tempesta_state;  // Current operational state
extern tempesta_state_t tempesta_previous_state;  // Previous state (for SHUTDOWN recovery)

// Power and hardware configuration
extern bool power_save_mode;                 // Power save mode flag (60min vs 15min cycle)
extern bool previous_power_save_mode;        // Previous power save mode (for SHUTDOWN recovery)
extern bool weather_pms5003_enabled;          // PMS5003 air quality sensor hardware enabled

// Interval configuration (loaded from interval_config at init)
extern uint32_t tempesta_normal_interval_ms;         // Normal mode collection interval (from config)
extern uint32_t tempesta_power_save_interval_ms;     // Power save mode collection interval (from config)

// Task and timer handles
extern TaskHandle_t xTempestaMainTaskHandle;
extern TaskHandle_t xTempestaAS5600TaskHandle;
extern TimerHandle_t xTempestaCollectionTimer;

// Sensor device handles
extern sht4x_t sht4x_dev;
extern bmp280_t bmp280_dev;
extern as5600_t as5600_dev;

// PMS5003 data structure
typedef struct {
    uint16_t pm1_0_cf1; // PM1.0 concentration (CF=1)
    uint16_t pm2_5_cf1; // PM2.5 concentration (CF=1)
    uint16_t pm10_cf1;  // PM10 concentration (CF=1)
    uint16_t pm1_0_atm; // PM1.0 concentration (atmospheric)
    uint16_t pm2_5_atm; // PM2.5 concentration (atmospheric)
    uint16_t pm10_atm;  // PM10 concentration (atmospheric)
    bool valid;
} pms5003_data_t;

// Consolidated sensor SHT40/BME280 readings
typedef struct {
    struct {
        float temperature;
        float humidity;
        bool valid;
    } sht4x;
    struct {
        float temperature;
        float pressure;
        float humidity;    // Only valid for BME280
        bool has_humidity; // True if BME280, false if BMP280
        bool valid;
    } bmp280;
} consolidated_sensor_data_t;

// ########################## Private Declarations ##########################

// Initialization functions
esp_err_t tempesta_hardware_init(void);
esp_err_t tempesta_i2c_sensors_init(void);
esp_err_t tempesta_pulse_sensors_init(void);
esp_err_t tempesta_pms5003_init(void);
esp_err_t tempesta_pms5003_send_sleep_command(void);
esp_err_t tempesta_pms5003_send_wake_command(void);

// Low level sensor reading functions
esp_err_t tempesta_read_env_sensors(consolidated_sensor_data_t *sensor_data);
esp_err_t tempesta_read_sht4x_all(consolidated_sensor_data_t *sensor_data);
esp_err_t tempesta_read_bmp280_all(consolidated_sensor_data_t *sensor_data);
esp_err_t tempesta_read_pms5003(pms5003_data_t *data, weather_sensor_status_t *status);
esp_err_t tempesta_read_and_process_rainfall(void);
esp_err_t tempesta_read_and_process_tank_intake(void);
esp_err_t tempesta_read_and_process_wind_direction(void);

// Data processing functions
esp_err_t tempesta_process_temperature(const consolidated_sensor_data_t *sensor_data);
esp_err_t tempesta_process_humidity(const consolidated_sensor_data_t *sensor_data);
esp_err_t tempesta_process_pressure(const consolidated_sensor_data_t *sensor_data);
esp_err_t tempesta_process_air_quality(void);
float tempesta_process_sensor_averaging(float new_value,
                                              float *history_array,
                                              uint8_t array_size,
                                              uint8_t *history_index,
                                              uint8_t *history_count);

// Task helper functions
void tempesta_handle_pms5003_reading(TickType_t warmup_start_time, bool should_read_pms5003);

// Utility functions
float tempesta_convert_pulses_to_rainfall_mm(int pulse_count);
esp_err_t tempesta_get_rainfall_pulse_count(int *pulse_count);
float tempesta_convert_pulses_to_tank_intake_ml(int pulse_count);
esp_err_t tempesta_get_tank_intake_pulse_count(int *pulse_count);
const char* tempesta_degrees_to_cardinal(float degrees);
void tempesta_daily_reset_callback(void);

// Task functions
void tempesta_as5600_sampling_task(void *pvParameters);


#endif // TEMPESTA_PRIVATE_H