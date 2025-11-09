/**
 * @file weather_station.c
 * @brief TEMPESTA - Multi-sensor weather station with power optimization
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Original implementation of TEMPESTA - comprehensive environmental monitoring.
 *
 * Key features:
 * - Eight-sensor array: temperature, humidity, pressure, wind, air quality, rainfall
 * - Power-aware polling (15min normal / 60min power-save)
 * - Sensor-level gating (hall array 10ms per cycle, PMS5003 UART sleep)
 * - Per-sensor status tracking (OK/UNAVAILABLE/ERROR/WARMING_UP)
 * - Temperature data sharing with IMPLUVIUM for watering correction
 * - AS5600 wind speed with low-power mode (5mA savings)
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "tempesta.h"
#include "main.h"
#include "fluctus.h"
#include "telemetry.h"
#include "solar_calc.h"

#include <string.h>
#include <math.h>
#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "hal/gpio_types.h"

// Components
#include "sht4x.h"
#include "bmp280.h"
#include "as5600.h"
#include "i2cdev.h"
#include "ads1115_helper.h"

#define TAG "TEMPESTA"

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
static SemaphoreHandle_t xTempestaDataMutex = NULL;
static tempesta_snapshot_t weather_data = {0};
static weather_calculation_data_t calculation_data = {0};

// State machine
static tempesta_state_t tempesta_state = TEMPESTA_STATE_DISABLED;  // Current operational state
static tempesta_state_t tempesta_previous_state = TEMPESTA_STATE_DISABLED;  // Previous state (for SHUTDOWN recovery)

// Hardware configuration
static bool weather_pms5003_enabled = true;          // PMS5003 air quality sensor hardware enabled

// Task and timer handles
static TaskHandle_t xTempestaMainTaskHandle = NULL;
static TaskHandle_t xTempestaAS5600TaskHandle = NULL;
static TimerHandle_t xTempestaCollectionTimer = NULL;

// Sensor device handles
static sht4x_t sht4x_dev = {0};
static bmp280_t bmp280_dev = {0};
static as5600_dev_t as5600_dev = {0};

// Hardware handles
static pcnt_unit_handle_t rain_pcnt_unit_handle = NULL;
static pcnt_unit_handle_t tank_intake_pcnt_unit_handle = NULL;

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

// Consolidated sensor readings
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

// ########################## Function Prototypes ##########################

// Initialization functions
static esp_err_t tempesta_hardware_init(void);
static esp_err_t tempesta_i2c_sensors_init(void);
static esp_err_t tempesta_pulse_sensors_init(void);
static esp_err_t tempesta_pms5003_init(void);
static esp_err_t tempesta_pms5003_send_sleep_command(void);
static esp_err_t tempesta_pms5003_send_wake_command(void);

// Low level sensor reading functions
static esp_err_t tempesta_read_env_sensors(consolidated_sensor_data_t *sensor_data);
static esp_err_t tempesta_read_sht4x_all(consolidated_sensor_data_t *sensor_data);
static esp_err_t tempesta_read_bmp280_all(consolidated_sensor_data_t *sensor_data);
static esp_err_t tempesta_read_pms5003(pms5003_data_t *data, weather_sensor_status_t *status);
static esp_err_t tempesta_read_and_process_rainfall(void);
static esp_err_t tempesta_read_and_process_tank_intake(void);
static esp_err_t tempesta_read_and_process_wind_direction(void);

// Data processing functions
static esp_err_t tempesta_process_temperature(const consolidated_sensor_data_t *sensor_data);
static esp_err_t tempesta_process_humidity(const consolidated_sensor_data_t *sensor_data);
static esp_err_t tempesta_process_pressure(const consolidated_sensor_data_t *sensor_data);
static esp_err_t tempesta_process_air_quality(void);
static float tempesta_process_sensor_averaging(float new_value,
                                              float *history_array,
                                              uint8_t array_size,
                                              uint8_t *history_index,
                                              uint8_t *history_count);

// State machine functions
static void tempesta_change_state(tempesta_state_t new_state);

// Power management functions
static esp_err_t tempesta_request_power_buses(bool requires_5v);
static void tempesta_release_power_buses(bool had_5v);
static void tempesta_update_timer_period(void);

// Task helper functions
//static void tempesta_log_summary(void);
static void tempesta_handle_pms5003_reading(TickType_t warmup_start_time, bool should_read_pms5003);

// Utility functions
static void tempesta_timer_callback(TimerHandle_t xTimer);
static uint32_t tempesta_calculate_time_to_next_quarter_hour(void);
static float tempesta_convert_pulses_to_rainfall_mm(int pulse_count);
static esp_err_t tempesta_get_rainfall_pulse_count(int *pulse_count);
static float tempesta_convert_pulses_to_tank_intake_ml(int pulse_count);
static esp_err_t tempesta_get_tank_intake_pulse_count(int *pulse_count);
static const char* tempesta_degrees_to_cardinal(float degrees);
static void tempesta_daily_reset_callback(void);

// Task functions
static void tempesta_main_task(void *pvParameters);
static void tempesta_as5600_sampling_task(void *pvParameters);

// ########################## State Machine Functions ##########################

/**
 * @brief Change TEMPESTA operational state with logging
 * @param new_state Target state to transition to
 * @note This function is not thread-safe - caller must hold xTempestaDataMutex if needed
 */
static void tempesta_change_state(tempesta_state_t new_state)
{
    if (tempesta_state == new_state) {
        return;  // No change needed
    }

    // Log state transition
    const char *old_state_str = "UNKNOWN";
    const char *new_state_str = "UNKNOWN";

    switch (tempesta_state) {
        case TEMPESTA_STATE_DISABLED:   old_state_str = "DISABLED"; break;
        case TEMPESTA_STATE_IDLE:       old_state_str = "IDLE"; break;
        case TEMPESTA_STATE_READING:    old_state_str = "READING"; break;
        case TEMPESTA_STATE_POWER_SAVE: old_state_str = "POWER_SAVE"; break;
        case TEMPESTA_STATE_SHUTDOWN:   old_state_str = "SHUTDOWN"; break;
        case TEMPESTA_STATE_ERROR:      old_state_str = "ERROR"; break;
    }

    switch (new_state) {
        case TEMPESTA_STATE_DISABLED:   new_state_str = "DISABLED"; break;
        case TEMPESTA_STATE_IDLE:       new_state_str = "IDLE"; break;
        case TEMPESTA_STATE_READING:    new_state_str = "READING"; break;
        case TEMPESTA_STATE_POWER_SAVE: new_state_str = "POWER_SAVE"; break;
        case TEMPESTA_STATE_SHUTDOWN:   new_state_str = "SHUTDOWN"; break;
        case TEMPESTA_STATE_ERROR:      new_state_str = "ERROR"; break;
    }

    ESP_LOGI(TAG, "State transition: %s -> %s", old_state_str, new_state_str);

    // Save previous state (for SHUTDOWN recovery)
    if (new_state == TEMPESTA_STATE_SHUTDOWN) {
        tempesta_previous_state = tempesta_state;
    }

    tempesta_state = new_state;

    // Update state in snapshot (for telemetry/HMI)
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        weather_data.state = new_state;
        xSemaphoreGive(xTempestaDataMutex);
    }
}

// ============================== ERROR STATE TODO ==============================
/*
 * TODO: Implement TEMPESTA_STATE_ERROR handling
 *
 * REQUIREMENTS:
 * 1. Transition to ERROR state when sensor failures exceed threshold
 *    - Count consecutive read failures per sensor
 *    - Trigger ERROR state when multiple critical sensors fail (e.g., 3+ sensors)
 *    - Continue reading remaining functional sensors in ERROR state
 *
 * 2. ERROR state behavior:
 *    - Follow normal reading cycles (15min/60min) with operational sensors
 *    - Mark failed sensors as WEATHER_SENSOR_ERROR in snapshot
 *    - Log detailed error information for diagnostics
 *    - Display ERROR state prominently in HMI
 *
 * 3. Recovery mechanism:
 *    - Require user confirmation via HMI or MQTT to exit ERROR state
 *    - Add tempesta_clear_error() function (HMI button handler)
 *    - Transition ERROR → IDLE on user confirmation
 *    - Reset sensor failure counters on recovery
 *
 * 4. TELEMETRY integration:
 *    - ERROR state transitions trigger immediate injection
 *    - Include failure masks/counts in ERROR state snapshot
 *
 * IMPLEMENTATION LOCATIONS:
 * - Sensor reading functions: Add failure counting logic
 * - Main task: Check failure threshold, transition to ERROR
 * - HMI: Add "Clear Error" button in TEMPESTA controls
 * - Add tempesta_clear_error() public function in header
 *
 * SAFETY: System remains partially operational in ERROR state - does not disable
 * all monitoring. User must explicitly acknowledge sensor issues.
 */
// ==============================================================================

// ########################## Initialization Functions ##########################

/**
 * @brief Initialize weather station component
 */
esp_err_t tempesta_station_init(void)
{
    ESP_LOGI(TAG, "Initializing weather station component...");

    // Create mutex for thread-safe data access
    xTempestaDataMutex = xSemaphoreCreateMutex();
    if (xTempestaDataMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create weather data mutex");
        return ESP_FAIL;
    }

    // Initialize weather data with invalid values
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
        weather_data.temperature = WEATHER_INVALID_VALUE;
        weather_data.humidity = WEATHER_INVALID_VALUE;
        weather_data.pressure = WEATHER_INVALID_VALUE;
        weather_data.air_quality_pm25 = WEATHER_INVALID_VALUE;
        weather_data.air_quality_pm10 = WEATHER_INVALID_VALUE;
        weather_data.wind_speed_rpm = WEATHER_INVALID_VALUE;
        weather_data.wind_speed_ms = WEATHER_INVALID_VALUE;
        weather_data.wind_direction_deg = WEATHER_INVALID_VALUE;
        weather_data.wind_direction_cardinal = "---";
        weather_data.rainfall_last_hour_mm = 0.0f;
        weather_data.rainfall_current_hour_mm = 0.0f;
        weather_data.rainfall_daily_mm = 0.0f;
        weather_data.rainfall_weekly_mm = 0.0f;
        weather_data.tank_intake_last_hour_ml = 0.0f;
        weather_data.tank_intake_current_hour_ml = 0.0f;
        weather_data.tank_intake_daily_ml = 0.0f;
        weather_data.tank_intake_weekly_ml = 0.0f;
        weather_data.snapshot_timestamp = 0;

        // Initialize state machine
        weather_data.state = TEMPESTA_STATE_DISABLED;

        // Initialize all sensor statuses as unavailable
        weather_data.temp_sensor_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.humidity_sensor_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.pressure_sensor_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.air_quality_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.wind_sensor_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.wind_direction_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.rain_gauge_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.tank_intake_status = WEATHER_SENSOR_UNAVAILABLE;

        xSemaphoreGive(xTempestaDataMutex);
    } else {
        ESP_LOGE(TAG, "Failed to initialize weather data - mutex timeout");
        return ESP_FAIL;
    }

    // Initialize humidity averaging data
    for (int i = 0; i < WEATHER_AVERAGING_SAMPLES; i++) {
        calculation_data.humidity_history[i] = WEATHER_INVALID_VALUE;
    }
    calculation_data.humidity_history_index = 0;
    calculation_data.humidity_history_count = 0;

    // Initialize temperature averaging data
    for (int i = 0; i < WEATHER_AVERAGING_SAMPLES; i++) {
        calculation_data.temperature_history[i] = WEATHER_INVALID_VALUE;
    }
    calculation_data.temp_history_index = 0;
    calculation_data.temp_history_count = 0;

    // Initialize rainfall tracking (using monotonic time)
    int64_t current_time_ms = esp_timer_get_time() / 1000;
    calculation_data.rainfall_hourly_start_time_ms = current_time_ms;
    calculation_data.rainfall_hourly_start_pulses = 0;
    calculation_data.rainfall_daily_base_pulses = 0;
    calculation_data.rainfall_weekly_base_pulses = 0;

    // Initialize tank intake tracking (using monotonic time)
    calculation_data.tank_intake_hourly_start_time_ms = current_time_ms;
    calculation_data.tank_intake_hourly_start_pulses = 0;
    calculation_data.tank_intake_daily_base_pulses = 0;
    calculation_data.tank_intake_weekly_base_pulses = 0;

    // Initialize hardware and sensors
    esp_err_t ret = tempesta_hardware_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Hardware initialization failed: %s", esp_err_to_name(ret));
        goto cleanup_mutex;
    }

    ret = tempesta_i2c_sensors_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Some sensors failed to initialize: %s", esp_err_to_name(ret));
        // Continue anyway - individual sensors will be marked as unavailable
    }

    // Register midnight callback for daily reset
    ret = solar_calc_register_midnight_callback(tempesta_daily_reset_callback);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register midnight callback: %s", esp_err_to_name(ret));
        // Continue anyway - daily reset won't work but hourly tracking will
    }

    // Create collection timer - auto-reload repeating timer
    xTempestaCollectionTimer = xTimerCreate("WeatherTimer",
                                           pdMS_TO_TICKS(WEATHER_COLLECTION_INTERVAL_MS),
                                           pdTRUE, // Auto-reload
                                           (void *) 0,
                                           tempesta_timer_callback);
    if (xTempestaCollectionTimer == NULL) {
        ESP_LOGE(TAG, "Failed to create collection timer");
        goto cleanup_mutex;
    }

    // Create main weather task
    BaseType_t xResult = xTaskCreate(tempesta_main_task,
                                     "WeatherMain",
                                     2048, // Reduced stack - mostly I2C operations
                                     NULL,
                                     5,
                                     &xTempestaMainTaskHandle);
    if (xResult != pdPASS) {
        ESP_LOGE(TAG, "Failed to create main weather task");
        goto cleanup_timer;
    }

    // Create AS5600 sampling task
    xResult = xTaskCreate(tempesta_as5600_sampling_task,
                          "WeatherAS5600",
                          2048, // Reduced stack - simple calculations
                          NULL,
                          6, // Higher priority for time-sensitive wind measurements
                          &xTempestaAS5600TaskHandle);
    if (xResult != pdPASS) {
        ESP_LOGE(TAG, "Failed to create AS5600 sampling task");
        goto cleanup_main_task;
    }

    // Start timer aligned to next quarter-hour boundary
    uint32_t delay_ms = tempesta_calculate_time_to_next_quarter_hour();
    if (delay_ms > 0 && delay_ms <= (16 * 60 * 1000)) {
        // Start with calculated delay, timer will auto-repeat every 15 minutes
        ESP_LOGI(TAG, "Starting timer in %" PRIu32 "ms to align with quarter-hour", delay_ms);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    
    if (xTimerStart(xTempestaCollectionTimer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start collection timer");
        goto cleanup_as5600_task;
    }

    ESP_LOGI(TAG, "Weather station initialized successfully");
    return ESP_OK;

    // Cleanup on failure
cleanup_as5600_task:
    if (xTempestaAS5600TaskHandle) {
        vTaskDelete(xTempestaAS5600TaskHandle);
        xTempestaAS5600TaskHandle = NULL;
    }
cleanup_main_task:
    if (xTempestaMainTaskHandle) {
        vTaskDelete(xTempestaMainTaskHandle);
        xTempestaMainTaskHandle = NULL;
    }
cleanup_timer:
    if (xTempestaCollectionTimer) {
        xTimerDelete(xTempestaCollectionTimer, portMAX_DELAY);
        xTempestaCollectionTimer = NULL;
    }
cleanup_mutex:
    if (xTempestaDataMutex) {
        vSemaphoreDelete(xTempestaDataMutex);
        xTempestaDataMutex = NULL;
    }
    return ESP_FAIL;
}

/**
 * @brief Initialize hardware components (GPIO, UART, pulse counters)
 */
static esp_err_t tempesta_hardware_init(void)
{
    esp_err_t ret;

    // Initialize pulse counters (rainfall and tank intake)
    ret = tempesta_pulse_sensors_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize pulse sensors: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize PMS5003 UART
    ret = tempesta_pms5003_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize PMS5003: %s", esp_err_to_name(ret));
        // Continue without PMS5003 - will be marked as unavailable
    }

    ESP_LOGI(TAG, "Weather station hardware initialized");
    return ESP_OK;
}

/**
 * @brief Initialize I2C sensors (SHT4x, BME280, AS5600)
 */
static esp_err_t tempesta_i2c_sensors_init(void)
{
    esp_err_t ret;
    bool any_sensor_ok = false;

    // Initialize SHT4x
    ret = sht4x_init_desc(&sht4x_dev, I2C_PORT_NUM, CONFIG_I2CDEV_DEFAULT_SDA_PIN, CONFIG_I2CDEV_DEFAULT_SCL_PIN);
    if (ret == ESP_OK) {
        ret = sht4x_init(&sht4x_dev);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "SHT4x sensor initialized successfully");
            any_sensor_ok = true;
        } else {
            ESP_LOGW(TAG, "SHT4x init failed: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGW(TAG, "SHT4x descriptor init failed: %s", esp_err_to_name(ret));
    }

    // Initialize BME280
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    ret = bmp280_init_desc(&bmp280_dev,
                           BMP280_I2C_ADDRESS_0,
                           I2C_PORT_NUM,
                           CONFIG_I2CDEV_DEFAULT_SDA_PIN,
                           CONFIG_I2CDEV_DEFAULT_SCL_PIN);
    if (ret == ESP_OK) {
        ret = bmp280_init(&bmp280_dev, &params);
        if (ret == ESP_OK) {
            bool is_bme280 = (bmp280_dev.id == BME280_CHIP_ID);
            ESP_LOGI(TAG, "%s sensor initialized successfully", is_bme280 ? "BME280" : "BMP280");
            any_sensor_ok = true;
        } else {
            ESP_LOGW(TAG, "BMP280/BME280 init failed: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGW(TAG, "BMP280/BME280 descriptor init failed: %s", esp_err_to_name(ret));
    }

    // Initialize AS5600
    ret = as5600_init_desc(&as5600_dev,
                           I2C_PORT_NUM,
                           AS5600_DEFAULT_ADDRESS,
                           CONFIG_I2CDEV_DEFAULT_SDA_PIN,
                           CONFIG_I2CDEV_DEFAULT_SCL_PIN);
    if (ret == ESP_OK) {
        ret = as5600_init(&as5600_dev);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "AS5600 sensor initialized successfully");
            any_sensor_ok = true;
        } else {
            ESP_LOGW(TAG, "AS5600 init failed: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGW(TAG, "AS5600 descriptor init failed: %s", esp_err_to_name(ret));
    }

    return any_sensor_ok ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Initialize pulse counter sensors (rainfall and tank intake)
 */
static esp_err_t tempesta_pulse_sensors_init(void)
{
    esp_err_t ret;
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = 0,
    };

    // ===== Initialize Rainfall Pulse Counter (GPIO 19) =====
    ret = pcnt_new_unit(&unit_config, &rain_pcnt_unit_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create rainfall pulse counter unit: %s", esp_err_to_name(ret));
        return ret;
    }

    pcnt_chan_config_t rain_chan_config = {
        .edge_gpio_num = WEATHER_RAINFALL_GPIO,
        .level_gpio_num = -1, // Not used
    };

    pcnt_channel_handle_t rain_pcnt_chan = NULL;
    ret = pcnt_new_channel(rain_pcnt_unit_handle, &rain_chan_config, &rain_pcnt_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create rainfall pulse counter channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_channel_set_edge_action(rain_pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set rainfall pulse counter edge action: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_enable(rain_pcnt_unit_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable rainfall pulse counter: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_clear_count(rain_pcnt_unit_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear rainfall pulse counter: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_start(rain_pcnt_unit_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start rainfall pulse counter: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Rainfall sensor initialized on GPIO%d", WEATHER_RAINFALL_GPIO);

    // ===== Initialize Tank Intake Pulse Counter (GPIO 20) =====
    ret = pcnt_new_unit(&unit_config, &tank_intake_pcnt_unit_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create tank intake pulse counter unit: %s", esp_err_to_name(ret));
        return ret;
    }

    pcnt_chan_config_t tank_chan_config = {
        .edge_gpio_num = WEATHER_TANK_INTAKE_GPIO,
        .level_gpio_num = -1, // Not used
    };

    pcnt_channel_handle_t tank_intake_pcnt_chan = NULL;
    ret = pcnt_new_channel(tank_intake_pcnt_unit_handle, &tank_chan_config, &tank_intake_pcnt_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create tank intake pulse counter channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_channel_set_edge_action(tank_intake_pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set tank intake pulse counter edge action: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_enable(tank_intake_pcnt_unit_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable tank intake pulse counter: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_clear_count(tank_intake_pcnt_unit_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear tank intake pulse counter: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_start(tank_intake_pcnt_unit_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start tank intake pulse counter: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Tank intake sensor initialized on GPIO%d", WEATHER_TANK_INTAKE_GPIO);

    return ESP_OK;
}

/**
 * @brief Initialize PMS5003 UART interface
 */
static esp_err_t tempesta_pms5003_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(WEATHER_PMS5003_UART_NUM, 256, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(WEATHER_PMS5003_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(WEATHER_PMS5003_UART_NUM,
                       WEATHER_PMS5003_TX_GPIO,
                       WEATHER_PMS5003_RX_GPIO,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG,
             "PMS5003 UART initialized on UART%d (TX: GPIO%d, RX: GPIO%d)",
             WEATHER_PMS5003_UART_NUM,
             WEATHER_PMS5003_TX_GPIO,
             WEATHER_PMS5003_RX_GPIO);
    return ESP_OK;
}

/**
 * @brief Send sleep command to PMS5003 sensor
 *
 * Puts the sensor into sleep mode (~200µA power consumption).
 * Command format: 0x42 0x4D 0xE4 0x00 0x00 + checksum
 *
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t tempesta_pms5003_send_sleep_command(void)
{
    // PMS5003 sleep command: 42 4D E4 00 00 01 73
    uint8_t sleep_cmd[] = {0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73};

    int written = uart_write_bytes(WEATHER_PMS5003_UART_NUM, sleep_cmd, sizeof(sleep_cmd));
    if (written != sizeof(sleep_cmd)) {
        ESP_LOGW(TAG, "PMS5003: Failed to send sleep command");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "PMS5003: Sleep command sent");
    return ESP_OK;
}

/**
 * @brief Send wake command to PMS5003 sensor
 *
 * Wakes the sensor from sleep mode. Sensor requires ~30 seconds warmup
 * time after waking before stable readings are available.
 * Command format: 0x42 0x4D 0xE4 0x00 0x01 + checksum
 *
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t tempesta_pms5003_send_wake_command(void)
{
    // PMS5003 wake command: 42 4D E4 00 01 01 74
    uint8_t wake_cmd[] = {0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74};

    int written = uart_write_bytes(WEATHER_PMS5003_UART_NUM, wake_cmd, sizeof(wake_cmd));
    if (written != sizeof(wake_cmd)) {
        ESP_LOGW(TAG, "PMS5003: Failed to send wake command");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "PMS5003: Wake command sent");
    return ESP_OK;
}

// ########################## Low Level Sensor Reading Functions ##########################

/**
 * @brief Read all sensors once
 */
static esp_err_t tempesta_read_env_sensors(consolidated_sensor_data_t *sensor_data)
{
    // Initialize all readings as invalid
    sensor_data->sht4x.valid = false;
    sensor_data->bmp280.valid = false;

    // Read SHT4x (temperature and humidity)
    tempesta_read_sht4x_all(sensor_data);

    // Read BMP280/BME280 (temperature, pressure, and humidity if BME280)
    tempesta_read_bmp280_all(sensor_data);

    // Return success if at least one sensor read successfully
    return (sensor_data->sht4x.valid || sensor_data->bmp280.valid) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Read SHT4x sensor once for temperature and humidity
 */
static esp_err_t tempesta_read_sht4x_all(consolidated_sensor_data_t *sensor_data)
{
    float sht_temp, sht_hum;
    if (sht4x_get_results(&sht4x_dev, &sht_temp, &sht_hum) == ESP_OK) {
        sensor_data->sht4x.temperature = sht_temp;
        sensor_data->sht4x.humidity = sht_hum;
        sensor_data->sht4x.valid = true;
        ESP_LOGI(TAG, "SHT4x: T=%.1f°C, H=%.1f%%", sht_temp, sht_hum);
        return ESP_OK;
    } else {
        sensor_data->sht4x.temperature = WEATHER_INVALID_VALUE;
        sensor_data->sht4x.humidity = WEATHER_INVALID_VALUE;
        sensor_data->sht4x.valid = false;
        ESP_LOGE(TAG, "SHT4x read failed");
        return ESP_FAIL;
    }
}

/**
 * @brief Read BMP280/BME280 sensor once for temperature, pressure, and humidity (if BME280)
 */
static esp_err_t tempesta_read_bmp280_all(consolidated_sensor_data_t *sensor_data)
{
    float bmp_temp, bmp_press, bmp_hum;
    if (bmp280_read_float(&bmp280_dev, &bmp_temp, &bmp_press, &bmp_hum) == ESP_OK) {
        sensor_data->bmp280.temperature = bmp_temp;
        sensor_data->bmp280.pressure = bmp_press / 100.0f; // Convert Pa to hPa
        sensor_data->bmp280.has_humidity = (bmp280_dev.id == BME280_CHIP_ID);
        sensor_data->bmp280.humidity = sensor_data->bmp280.has_humidity ? bmp_hum : WEATHER_INVALID_VALUE;
        sensor_data->bmp280.valid = true;

        if (sensor_data->bmp280.has_humidity) {
            ESP_LOGI(TAG, "BME280: T=%.1f°C, P=%.1f hPa, H=%.1f%%", bmp_temp, sensor_data->bmp280.pressure, bmp_hum);
        } else {
            ESP_LOGI(TAG, "BMP280: T=%.1f°C, P=%.1f hPa", bmp_temp, sensor_data->bmp280.pressure);
        }
        return ESP_OK;
    } else {
        sensor_data->bmp280.temperature = WEATHER_INVALID_VALUE;
        sensor_data->bmp280.pressure = WEATHER_INVALID_VALUE;
        sensor_data->bmp280.humidity = WEATHER_INVALID_VALUE;
        sensor_data->bmp280.has_humidity = false;
        sensor_data->bmp280.valid = false;
        ESP_LOGE(TAG, "BMP280/BME280 read failed");
        return ESP_FAIL;
    }
}

/**
 * @brief Read and parse PMS5003 data frame
 */
static esp_err_t tempesta_read_pms5003(pms5003_data_t *data, weather_sensor_status_t *status)
{
    // Simple retry logic for UART communication issues
    for (int retry = 0; retry < WEATHER_PMS5003_RETRY_COUNT; retry++) {
        uint8_t buffer[32];

        // Clear any stale data from UART buffer before reading
        uart_flush_input(WEATHER_PMS5003_UART_NUM);

        // Read data with timeout
        int len = uart_read_bytes(WEATHER_PMS5003_UART_NUM, buffer, sizeof(buffer), pdMS_TO_TICKS(1000));
        if (len <= 0) {
            ESP_LOGW(TAG, "PMS5003: No data received (attempt %d/3)", retry + 1);
            if (retry < 2)
                vTaskDelay(pdMS_TO_TICKS(100)); // 100ms between retries
            continue;
        }

        // Look for start header (0x42 0x4D)
        int start_idx = -1;
        for (int i = 0; i < len - 1; i++) {
            if (buffer[i] == 0x42 && buffer[i + 1] == 0x4D) {
                start_idx = i;
                break;
            }
        }

        if (start_idx == -1 || (len - start_idx) < 32) {
            ESP_LOGW(TAG, "PMS5003: Invalid data frame (attempt %d/3)", retry + 1);
            if (retry < 2)
                vTaskDelay(pdMS_TO_TICKS(100)); // 100ms between retries
            continue;
        }

        // Parse data frame
        uint8_t *frame = &buffer[start_idx];
        uint16_t frame_len = (frame[2] << 8) | frame[3];

        if (frame_len != 28) { // Should be 2*13+2
            ESP_LOGW(TAG, "PMS5003: Invalid frame length %d (attempt %d/3)", frame_len, retry + 1);
            if (retry < 2)
                vTaskDelay(pdMS_TO_TICKS(100)); // 100ms between retries
            continue;
        }

        // Calculate checksum
        uint16_t checksum = 0;
        for (int i = 0; i < 30; i++) {
            checksum += frame[i];
        }
        uint16_t received_checksum = (frame[30] << 8) | frame[31];

        if (checksum != received_checksum) {
            ESP_LOGW(TAG,
                     "PMS5003: Checksum mismatch (calc: %d, recv: %d, attempt %d/3)",
                     checksum,
                     received_checksum,
                     retry + 1);
            if (retry < 2)
                vTaskDelay(pdMS_TO_TICKS(100)); // 100ms between retries
            continue;
        }

        // Success! Extract PM data (use atmospheric environment values)
        data->pm1_0_atm = (frame[10] << 8) | frame[11];
        data->pm2_5_atm = (frame[12] << 8) | frame[13];
        data->pm10_atm = (frame[14] << 8) | frame[15];
        data->valid = true;

        *status = WEATHER_SENSOR_OK;
        ESP_LOGD(TAG,
                 "PMS5003: PM1.0=%d µg/m³, PM2.5=%d µg/m³, PM10=%d µg/m³",
                 data->pm1_0_atm,
                 data->pm2_5_atm,
                 data->pm10_atm);

        return ESP_OK;
    }

    // All retries exhausted
    *status = WEATHER_SENSOR_ERROR;
    ESP_LOGE(TAG, "PMS5003: Failed after 3 attempts");
    return ESP_FAIL;
}

/**
 * @brief Process rainfall accumulation with hourly/daily calculations and data update
 */
static esp_err_t tempesta_read_and_process_rainfall(void)
{
    int pulse_count = 0;
    esp_err_t ret = tempesta_get_rainfall_pulse_count(&pulse_count);
    if (ret != ESP_OK) {
        // Update weather data with error state
        if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
            weather_data.rainfall_last_hour_mm = WEATHER_INVALID_VALUE;
            weather_data.rainfall_current_hour_mm = WEATHER_INVALID_VALUE;
            weather_data.rainfall_daily_mm = WEATHER_INVALID_VALUE;
            weather_data.rainfall_weekly_mm = WEATHER_INVALID_VALUE;
            weather_data.rain_gauge_status = WEATHER_SENSOR_ERROR;
            xSemaphoreGive(xTempestaDataMutex);
        }
        return ret;
    }

    // Acquire mutex to protect calculation_data and weather_data access
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for rainfall processing");
        return ESP_FAIL;
    }

    // Check if hour has completed
    int64_t current_time_ms = esp_timer_get_time() / 1000;
    int64_t elapsed_ms = current_time_ms - calculation_data.rainfall_hourly_start_time_ms;

    if (elapsed_ms >= 3600000) { // Hour completed
        // Store completed hour measurement
        int pulses_this_hour = pulse_count - calculation_data.rainfall_hourly_start_pulses;
        weather_data.rainfall_last_hour_mm = tempesta_convert_pulses_to_rainfall_mm(pulses_this_hour);

        // Reset tracking for next hour
        calculation_data.rainfall_hourly_start_time_ms = current_time_ms;
        calculation_data.rainfall_hourly_start_pulses = pulse_count;

        ESP_LOGI(TAG, "Rainfall: Completed hour = %.3f mm", weather_data.rainfall_last_hour_mm);
    }

    // Always calculate current hour accumulation (for debug/monitoring)
    int pulses_current_hour = pulse_count - calculation_data.rainfall_hourly_start_pulses;
    weather_data.rainfall_current_hour_mm = tempesta_convert_pulses_to_rainfall_mm(pulses_current_hour);

    // Calculate daily and weekly accumulation from base pulse counts
    int pulses_since_midnight = pulse_count - calculation_data.rainfall_daily_base_pulses;
    int pulses_since_weekly_reset = pulse_count - calculation_data.rainfall_weekly_base_pulses;
    weather_data.rainfall_daily_mm = tempesta_convert_pulses_to_rainfall_mm(pulses_since_midnight);
    weather_data.rainfall_weekly_mm = tempesta_convert_pulses_to_rainfall_mm(pulses_since_weekly_reset);
    weather_data.rain_gauge_status = WEATHER_SENSOR_OK;

    xSemaphoreGive(xTempestaDataMutex);

    ESP_LOGD(TAG, "Rain: Last=%.3f mm, Current=%.3f mm, Daily=%.3f mm, Weekly=%.3f mm",
             weather_data.rainfall_last_hour_mm, weather_data.rainfall_current_hour_mm,
             weather_data.rainfall_daily_mm, weather_data.rainfall_weekly_mm);
    return ESP_OK;
}

/**
 * @brief Process tank intake accumulation with hourly/daily calculations and data update
 */
static esp_err_t tempesta_read_and_process_tank_intake(void)
{
    int pulse_count = 0;
    esp_err_t ret = tempesta_get_tank_intake_pulse_count(&pulse_count);
    if (ret != ESP_OK) {
        // Update weather data with error state
        if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
            weather_data.tank_intake_last_hour_ml = WEATHER_INVALID_VALUE;
            weather_data.tank_intake_current_hour_ml = WEATHER_INVALID_VALUE;
            weather_data.tank_intake_daily_ml = WEATHER_INVALID_VALUE;
            weather_data.tank_intake_weekly_ml = WEATHER_INVALID_VALUE;
            weather_data.tank_intake_status = WEATHER_SENSOR_ERROR;
            xSemaphoreGive(xTempestaDataMutex);
        }
        return ret;
    }

    // Acquire mutex to protect calculation_data and weather_data access
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for tank intake processing");
        return ESP_FAIL;
    }

    // Check if hour has completed
    int64_t current_time_ms = esp_timer_get_time() / 1000;
    int64_t elapsed_ms = current_time_ms - calculation_data.tank_intake_hourly_start_time_ms;

    if (elapsed_ms >= 3600000) { // Hour completed
        // Store completed hour measurement
        int pulses_this_hour = pulse_count - calculation_data.tank_intake_hourly_start_pulses;
        weather_data.tank_intake_last_hour_ml = tempesta_convert_pulses_to_tank_intake_ml(pulses_this_hour);

        // Reset tracking for next hour
        calculation_data.tank_intake_hourly_start_time_ms = current_time_ms;
        calculation_data.tank_intake_hourly_start_pulses = pulse_count;

        ESP_LOGI(TAG, "Tank intake: Completed hour = %.1f mL", weather_data.tank_intake_last_hour_ml);
    }

    // Always calculate current hour accumulation (for debug/monitoring)
    int pulses_current_hour = pulse_count - calculation_data.tank_intake_hourly_start_pulses;
    weather_data.tank_intake_current_hour_ml = tempesta_convert_pulses_to_tank_intake_ml(pulses_current_hour);

    // Calculate daily and weekly accumulation from base pulse counts
    int pulses_since_midnight = pulse_count - calculation_data.tank_intake_daily_base_pulses;
    int pulses_since_weekly_reset = pulse_count - calculation_data.tank_intake_weekly_base_pulses;
    weather_data.tank_intake_daily_ml = tempesta_convert_pulses_to_tank_intake_ml(pulses_since_midnight);
    weather_data.tank_intake_weekly_ml = tempesta_convert_pulses_to_tank_intake_ml(pulses_since_weekly_reset);
    weather_data.tank_intake_status = WEATHER_SENSOR_OK;

    xSemaphoreGive(xTempestaDataMutex);

    ESP_LOGD(TAG, "Tank: Last=%.1f mL, Current=%.1f mL, Daily=%.1f mL, Weekly=%.1f mL",
             weather_data.tank_intake_last_hour_ml, weather_data.tank_intake_current_hour_ml,
             weather_data.tank_intake_daily_ml, weather_data.tank_intake_weekly_ml);
    return ESP_OK;
}

/**
 * @brief Read Hall sensors and calculate wind direction using weighted averaging
 */
static esp_err_t tempesta_read_and_process_wind_direction(void)
{
    // Check if ADS1115 device is ready
    if (!ads1115_helper_is_device_ready(WEATHER_WIND_DIR_ADS1115_DEVICE)) {
        ESP_LOGW(TAG, "ADS1115 Hall sensor array not ready");
        if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
            weather_data.wind_direction_deg = WEATHER_INVALID_VALUE;
            weather_data.wind_direction_cardinal = "---";
            weather_data.wind_direction_status = WEATHER_SENSOR_UNAVAILABLE;
            xSemaphoreGive(xTempestaDataMutex);
        }
        return ESP_FAIL;
    }

    // Enable hall array power and allow settling time
    fluctus_hall_array_enable(true);
    vTaskDelay(pdMS_TO_TICKS(10));  // 10ms for sensor + ADC settling

    // Read all 4 Hall sensors (N, E, S, W)
    float voltage_north = 0.0f, voltage_east = 0.0f, voltage_south = 0.0f, voltage_west = 0.0f;
    int16_t raw_value;
    esp_err_t ret;

    ret = ads1115_helper_read_channel(WEATHER_WIND_DIR_ADS1115_DEVICE,
                                      WEATHER_WIND_DIR_HALL_NORTH_CH,
                                      &raw_value, &voltage_north);
    if (ret != ESP_OK) voltage_north = 0.0f;

    ret = ads1115_helper_read_channel(WEATHER_WIND_DIR_ADS1115_DEVICE,
                                      WEATHER_WIND_DIR_HALL_EAST_CH,
                                      &raw_value, &voltage_east);
    if (ret != ESP_OK) voltage_east = 0.0f;

    ret = ads1115_helper_read_channel(WEATHER_WIND_DIR_ADS1115_DEVICE,
                                      WEATHER_WIND_DIR_HALL_SOUTH_CH,
                                      &raw_value, &voltage_south);
    if (ret != ESP_OK) voltage_south = 0.0f;

    ret = ads1115_helper_read_channel(WEATHER_WIND_DIR_ADS1115_DEVICE,
                                      WEATHER_WIND_DIR_HALL_WEST_CH,
                                      &raw_value, &voltage_west);
    if (ret != ESP_OK) voltage_west = 0.0f;

    ESP_LOGD(TAG, "Hall sensors: N=%.2fV, E=%.2fV, S=%.2fV, W=%.2fV",
             voltage_north, voltage_east, voltage_south, voltage_west);

    // Normalize voltages to 0-1 range based on max voltage
    float norm_north = voltage_north / WEATHER_WIND_DIR_MAX_VOLTAGE;
    float norm_east = voltage_east / WEATHER_WIND_DIR_MAX_VOLTAGE;
    float norm_south = voltage_south / WEATHER_WIND_DIR_MAX_VOLTAGE;
    float norm_west = voltage_west / WEATHER_WIND_DIR_MAX_VOLTAGE;

    // Clamp to 0-1
    if (norm_north > 1.0f) norm_north = 1.0f;
    if (norm_east > 1.0f) norm_east = 1.0f;
    if (norm_south > 1.0f) norm_south = 1.0f;
    if (norm_west > 1.0f) norm_west = 1.0f;

    // Check if any sensors are active (above threshold)
    float threshold_norm = WEATHER_WIND_DIR_THRESHOLD_VOLTAGE / WEATHER_WIND_DIR_MAX_VOLTAGE;
    if (norm_north < threshold_norm && norm_east < threshold_norm &&
        norm_south < threshold_norm && norm_west < threshold_norm) {
        ESP_LOGW(TAG, "No active Hall sensors detected (all below threshold)");
        if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
            weather_data.wind_direction_deg = WEATHER_INVALID_VALUE;
            weather_data.wind_direction_cardinal = "---";
            weather_data.wind_direction_status = WEATHER_SENSOR_ERROR;
            xSemaphoreGive(xTempestaDataMutex);
        }
        fluctus_hall_array_enable(false);  // Always disable on exit
        return ESP_FAIL;
    }

    // Calculate weighted direction using vector addition (to handle circular averaging correctly)
    // Each sensor contributes a unit vector in its direction, weighted by its normalized reading
    // N=0°, E=90°, S=180°, W=270°

    // Convert to radians and calculate weighted unit vectors
    float x_component = 0.0f, y_component = 0.0f;

    // North (0°) - positive Y axis
    x_component += norm_north * sin(0.0f);      // sin(0) = 0
    y_component += norm_north * cos(0.0f);      // cos(0) = 1

    // East (90°) - positive X axis
    x_component += norm_east * sin(M_PI / 2.0f);  // sin(90) = 1
    y_component += norm_east * cos(M_PI / 2.0f);  // cos(90) = 0

    // South (180°) - negative Y axis
    x_component += norm_south * sin(M_PI);        // sin(180) = 0
    y_component += norm_south * cos(M_PI);        // cos(180) = -1

    // West (270°) - negative X axis
    x_component += norm_west * sin(3.0f * M_PI / 2.0f);  // sin(270) = -1
    y_component += norm_west * cos(3.0f * M_PI / 2.0f);  // cos(270) = 0

    // Calculate angle from vector components
    float angle_rad = atan2(x_component, y_component);
    float angle_deg = angle_rad * (180.0f / M_PI);

    // Normalize to 0-360°
    if (angle_deg < 0) angle_deg += 360.0f;

    // Convert to cardinal direction
    const char* cardinal = tempesta_degrees_to_cardinal(angle_deg);

    // Update weather data with successful reading
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
        weather_data.wind_direction_deg = angle_deg;
        weather_data.wind_direction_cardinal = cardinal;
        weather_data.wind_direction_status = WEATHER_SENSOR_OK;
        xSemaphoreGive(xTempestaDataMutex);
    }

    ESP_LOGD(TAG, "Wind direction: %.1f° (%s)", angle_deg, cardinal);

    // Disable hall array after reading
    fluctus_hall_array_enable(false);
    return ESP_OK;
}

// ########################## Data Processing Functions ##########################

/**
 * @brief Process temperature from sensors with integrated historical averaging and data update
 */
static esp_err_t tempesta_process_temperature(const consolidated_sensor_data_t *sensor_data)
{
    bool sht4x_ok = sensor_data->sht4x.valid;
    bool bmp280_ok = sensor_data->bmp280.valid;
    float raw_temperature;
    float averaged_temperature;
    weather_sensor_status_t status;

    // Use best available reading strategy
    if (sht4x_ok && bmp280_ok) {
        // Both sensors working - use averaged value
        raw_temperature = (sensor_data->sht4x.temperature + sensor_data->bmp280.temperature) / 2.0f;
        status = WEATHER_SENSOR_OK;
        ESP_LOGD(TAG,
                 "Temperature: SHT4x=%.1f°C, BME280=%.1f°C, Raw Average=%.1f°C",
                 sensor_data->sht4x.temperature,
                 sensor_data->bmp280.temperature,
                 raw_temperature);
    } else if (sht4x_ok) {
        raw_temperature = sensor_data->sht4x.temperature;
        status = WEATHER_SENSOR_OK;
        ESP_LOGD(TAG, "Temperature: SHT4x=%.1f°C (BME280 unavailable)", sensor_data->sht4x.temperature);
    } else if (bmp280_ok) {
        raw_temperature = sensor_data->bmp280.temperature;
        status = WEATHER_SENSOR_OK;
        ESP_LOGD(TAG, "Temperature: BME280=%.1f°C (SHT4x unavailable)", sensor_data->bmp280.temperature);
    } else {
        averaged_temperature = WEATHER_INVALID_VALUE;
        status = WEATHER_SENSOR_ERROR;
        ESP_LOGW(TAG, "No temperature sensors available");

        // Update weather data with error state
        WEATHER_UPDATE_DATA_WITH_STATUS_ERROR(temperature, WEATHER_INVALID_VALUE, temp_sensor_status, WEATHER_SENSOR_ERROR);
        return ESP_FAIL;
    }

    // Acquire mutex to protect calculation_data and weather_data access
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for temperature processing");
        return ESP_FAIL;
    }

    // Apply historical averaging (modifies calculation_data)
    averaged_temperature = tempesta_process_sensor_averaging(raw_temperature,
                                                            calculation_data.temperature_history,
                                                            WEATHER_AVERAGING_SAMPLES,
                                                            &calculation_data.temp_history_index,
                                                            &calculation_data.temp_history_count);

    ESP_LOGD(TAG,
             "Temperature final: raw=%.1f°C, averaged=%.1f°C (%d samples)",
             raw_temperature,
             averaged_temperature,
             calculation_data.temp_history_count);

    // Update weather data with successful reading
    weather_data.temperature = averaged_temperature;
    weather_data.temp_sensor_status = status;

    xSemaphoreGive(xTempestaDataMutex);

    return ESP_OK;
}

/**
 * @brief Process humidity from sensors with integrated historical averaging and data update
 * Combines readings from both sensors when available for improved accuracy
 */
static esp_err_t tempesta_process_humidity(const consolidated_sensor_data_t *sensor_data)
{
    bool sht4x_ok = sensor_data->sht4x.valid;
    bool bme280_ok = sensor_data->bmp280.valid && sensor_data->bmp280.has_humidity;
    float raw_humidity;
    float averaged_humidity;
    weather_sensor_status_t status;

    // Use best available reading strategy
    if (sht4x_ok && bme280_ok) {
        // Both sensors working - use averaged value (SHT4x tends to read slightly higher)
        raw_humidity = (sensor_data->sht4x.humidity + sensor_data->bmp280.humidity) / 2.0f;
        status = WEATHER_SENSOR_OK;
        ESP_LOGD(TAG,
                 "Humidity: SHT4x=%.1f%%, BME280=%.1f%%, Raw Average=%.1f%%",
                 sensor_data->sht4x.humidity,
                 sensor_data->bmp280.humidity,
                 raw_humidity);
    } else if (sht4x_ok) {
        raw_humidity = sensor_data->sht4x.humidity;
        status = WEATHER_SENSOR_OK;
        ESP_LOGD(TAG, "Humidity: SHT4x=%.1f%% (BME280 unavailable)", sensor_data->sht4x.humidity);
    } else if (bme280_ok) {
        raw_humidity = sensor_data->bmp280.humidity;
        status = WEATHER_SENSOR_OK;
        ESP_LOGD(TAG, "Humidity: BME280=%.1f%% (SHT4x unavailable)", sensor_data->bmp280.humidity);
    } else {
        averaged_humidity = WEATHER_INVALID_VALUE;
        status = WEATHER_SENSOR_ERROR;
        ESP_LOGW(TAG, "No humidity sensors available");

        // Update weather data with error state
        WEATHER_UPDATE_DATA_WITH_STATUS_ERROR(humidity, WEATHER_INVALID_VALUE, humidity_sensor_status, WEATHER_SENSOR_ERROR);
        return ESP_FAIL;
    }

    // Acquire mutex to protect calculation_data and weather_data access
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for humidity processing");
        return ESP_FAIL;
    }

    // Apply historical averaging (modifies calculation_data)
    averaged_humidity = tempesta_process_sensor_averaging(raw_humidity,
                                                         calculation_data.humidity_history,
                                                         WEATHER_AVERAGING_SAMPLES,
                                                         &calculation_data.humidity_history_index,
                                                         &calculation_data.humidity_history_count);

    ESP_LOGD(TAG,
             "Humidity final: raw=%.1f%%, averaged=%.1f%% (%d samples)",
             raw_humidity,
             averaged_humidity,
             calculation_data.humidity_history_count);

    // Update weather data with successful reading
    weather_data.humidity = averaged_humidity;
    weather_data.humidity_sensor_status = status;

    xSemaphoreGive(xTempestaDataMutex);

    return ESP_OK;
}

/**
 * @brief Process pressure from sensor data and update weather data
 */
static esp_err_t tempesta_process_pressure(const consolidated_sensor_data_t *sensor_data)
{
    float pressure;
    weather_sensor_status_t status;

    if (sensor_data->bmp280.valid) {
        pressure = sensor_data->bmp280.pressure;
        status = WEATHER_SENSOR_OK;
        ESP_LOGD(TAG, "Pressure: %.1f hPa", pressure);

        // Update weather data with successful reading
        WEATHER_UPDATE_DATA_WITH_STATUS_ERROR(pressure, pressure, pressure_sensor_status, status);
        return ESP_OK;
    }

    // Handle error case
    pressure = WEATHER_INVALID_VALUE;
    status = WEATHER_SENSOR_ERROR;
    ESP_LOGW(TAG, "Pressure sensor unavailable");

    // Update weather data with error state
    WEATHER_UPDATE_DATA_WITH_STATUS_ERROR(pressure, WEATHER_INVALID_VALUE, pressure_sensor_status, WEATHER_SENSOR_ERROR);

    return ESP_FAIL;
}

/**
 * @brief Process air quality with PMS5003 warmup handling and data update
 */
static esp_err_t tempesta_process_air_quality(void)
{
    // Note: This function assumes the warmup time has already been handled by the caller
    // and that the decision to read PMS5003 has been made at task level

    pms5003_data_t air_quality;
    weather_sensor_status_t air_quality_status;

    esp_err_t ret = tempesta_read_pms5003(&air_quality, &air_quality_status);
    if (ret == ESP_OK) {
        // Update weather data with successful reading
        if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
            weather_data.air_quality_pm25 = (float) air_quality.pm2_5_atm;
            weather_data.air_quality_pm10 = (float) air_quality.pm10_atm;
            weather_data.air_quality_status = air_quality_status;
            xSemaphoreGive(xTempestaDataMutex);
        } else {
            ESP_LOGW(TAG, "Failed to acquire mutex for air quality data update");
            return ESP_FAIL;
        }
    } else {
        // Update weather data with error state
        if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
            weather_data.air_quality_pm25 = WEATHER_INVALID_VALUE;
            weather_data.air_quality_pm10 = WEATHER_INVALID_VALUE;
            weather_data.air_quality_status = WEATHER_SENSOR_ERROR;
            xSemaphoreGive(xTempestaDataMutex);
        }
    }

    return ret;
}

/**
 * @brief Unified sensor averaging algorithm for temperature and humidity
 * Uses weighted averaging with more recent samples getting higher weight
 */
static float tempesta_process_sensor_averaging(float new_value,
                                              float *history_array,
                                              uint8_t array_size,
                                              uint8_t *history_index,
                                              uint8_t *history_count)
{
    if (new_value == WEATHER_INVALID_VALUE) {
        return WEATHER_INVALID_VALUE;
    }

    history_array[*history_index] = new_value;          // Add to circular buffer, write at current position
    *history_index = (*history_index + 1) % array_size; // Move to next slot

    if (*history_count < array_size) {
        (*history_count)++;
    }

    // Calculate weighted average (more recent samples get higher weight)
    float weighted_sum = 0.0f;
    float total_weight = 0.0f;

    for (uint8_t i = 0; i < *history_count; i++) {
        // Most recent sample gets highest weight, sample_idx calculation walks backwards from the most recent write position
        // so it's possible to assign higher weight to newest reading in history_array.
        uint8_t sample_idx = (*history_index - 1 - i + array_size) % array_size;
        float weight = 1.0f + (0.1f * (*history_count - 1 - i)); // Recent samples weighted higher

        weighted_sum += history_array[sample_idx] * weight;
        total_weight += weight;
    }

    return weighted_sum / total_weight;
}

// ########################## Power Management Functions ##########################

/**
 * @brief Unified power management for weather station based on sensor requirements
 * @param requires_5v Whether 5V bus is needed (for PMS5003)
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t tempesta_request_power_buses(bool requires_5v)
{
    // Always request 3V3 for basic sensors
    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_3V3, "TEMPESTA", return ESP_FAIL);

    // Request 5V if needed (decision made at task level)
    if (requires_5v) {
        FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_5V, "TEMPESTA", {
            fluctus_release_bus_power(POWER_BUS_3V3, "TEMPESTA");
            return ESP_FAIL;
        });
    }

    return ESP_OK;
}

/**
 * @brief Release power buses based on what was requested
 * @param had_5v Whether 5V bus was previously requested
 */
static void tempesta_release_power_buses(bool had_5v)
{
    if (had_5v) {
        fluctus_release_bus_power(POWER_BUS_5V, "TEMPESTA");
    }
    fluctus_release_bus_power(POWER_BUS_3V3, "TEMPESTA");
}

/**
 * @brief Helper function to update timer period based on power save mode
 */
static void tempesta_update_timer_period(void)
{
    if (xTempestaCollectionTimer != NULL) {
        TickType_t new_period = (tempesta_state == TEMPESTA_STATE_POWER_SAVE) ?
            pdMS_TO_TICKS(WEATHER_COLLECTION_INTERVAL_POWER_SAVE_MS) :
            pdMS_TO_TICKS(WEATHER_COLLECTION_INTERVAL_MS);

        xTimerChangePeriod(xTempestaCollectionTimer, new_period, portMAX_DELAY);
    }
}

// ########################## Task Helper Functions ##########################

/**
 * @brief Log comprehensive weather summary with sensor status
 */
/*
static void tempesta_log_summary(void)
{
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        ESP_LOGI(TAG,
                 "Weather: T=%.1f°C, H=%.1f%%, P=%.1f hPa, PM2.5=%.0f µg/m³",
                 weather_data.temperature,
                 weather_data.humidity,
                 weather_data.pressure,
                 weather_data.air_quality_pm25);

        ESP_LOGI(TAG,
                 "Wind: Speed=%.1f RPM (%.1f m/s), Direction=%.1f° (%s)",
                 weather_data.wind_speed_rpm,
                 weather_data.wind_speed_ms,
                 weather_data.wind_direction_deg,
                 weather_data.wind_direction_cardinal);

        ESP_LOGI(TAG,
                 "Rain: Last=%.2f mm, Current=%.2f mm, Daily=%.2f mm, Weekly=%.2f mm",
                 weather_data.rainfall_last_hour_mm,
                 weather_data.rainfall_current_hour_mm,
                 weather_data.rainfall_daily_mm,
                 weather_data.rainfall_weekly_mm);

        ESP_LOGI(TAG,
                 "Tank: Last=%.1f mL, Current=%.1f mL, Daily=%.1f mL, Weekly=%.1f mL",
                 weather_data.tank_intake_last_hour_ml,
                 weather_data.tank_intake_current_hour_ml,
                 weather_data.tank_intake_daily_ml,
                 weather_data.tank_intake_weekly_ml);

        if (weather_data.temp_sensor_status != WEATHER_SENSOR_OK ||
            weather_data.humidity_sensor_status != WEATHER_SENSOR_OK ||
            weather_data.pressure_sensor_status != WEATHER_SENSOR_OK ||
            weather_data.air_quality_status != WEATHER_SENSOR_OK ||
            weather_data.wind_sensor_status != WEATHER_SENSOR_OK ||
            weather_data.wind_direction_status != WEATHER_SENSOR_OK ||
            weather_data.rain_gauge_status != WEATHER_SENSOR_OK ||
            weather_data.tank_intake_status != WEATHER_SENSOR_OK) {
            ESP_LOGW(TAG, "Weather station is not fully operational, check sensor status");
        }
        xSemaphoreGive(xTempestaDataMutex);
    }
}
*/

/**
 * @brief Handle PMS5003 warmup and reading
 * @param warmup_start_time Tick count when power was applied
 * @param should_read_pms5003 Whether PMS5003 should be read this cycle
 */
static void tempesta_handle_pms5003_reading(TickType_t warmup_start_time, bool should_read_pms5003)
{
    if (!should_read_pms5003) {
        ESP_LOGI(TAG, "Skipping air quality measurement (PMS5003 disabled)");
        return;
    }

    // Calculate remaining warmup time
    TickType_t elapsed_time = xTaskGetTickCount() - warmup_start_time;
    TickType_t elapsed_ms = pdTICKS_TO_MS(elapsed_time);

    if (elapsed_ms < WEATHER_PMS5003_WARMUP_TIME_MS) {
        uint32_t remaining_warmup_ms = WEATHER_PMS5003_WARMUP_TIME_MS - elapsed_ms;
        ESP_LOGI(TAG, "Waiting additional %" PRIu32 "ms for PMS5003 warmup", remaining_warmup_ms);
        vTaskDelay(pdMS_TO_TICKS(remaining_warmup_ms));
    }

    // Read air quality sensor
    tempesta_process_air_quality();

    // Put PMS5003 back to sleep to save power
    tempesta_pms5003_send_sleep_command();
    ESP_LOGI(TAG, "PMS5003 sleep command sent (power saving)");
}

// ########################## Utility Functions ##########################

/**
 * @brief Timer callback to trigger data collection cycle
 */
static void tempesta_timer_callback(TimerHandle_t xTimer)
{
    // Notify main task to start collection cycle
    if (xTempestaMainTaskHandle != NULL) {
        xTaskNotify(xTempestaMainTaskHandle, 1, eSetBits);
    }
}

/**
 * @brief Calculate milliseconds until next quarter-hour boundary
 */
static uint32_t tempesta_calculate_time_to_next_quarter_hour(void)
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    // Calculate minutes until next quarter-hour (00, 15, 30, 45)
    int current_min = timeinfo.tm_min;
    int current_sec = timeinfo.tm_sec;
    
    int next_quarter = ((current_min / 15) + 1) * 15;
    int minutes_to_wait = (next_quarter <= 60) ? (next_quarter - current_min) : (60 - current_min);
    
    // Convert to milliseconds, subtract current seconds
    uint32_t delay_ms = (minutes_to_wait * 60 - current_sec) * 1000;
    
    ESP_LOGI(TAG, "Current: %02d:%02d:%02d, next quarter-hour in %" PRIu32 "ms", 
             timeinfo.tm_hour, current_min, current_sec, delay_ms);
    
    return delay_ms;
}

/**
 * @brief Convert pulse count to rainfall depth in mm using physical tipbucket parameters
 */
static float tempesta_convert_pulses_to_rainfall_mm(int pulse_count)
{
    if (pulse_count < 0) {
        return 0.0f;
    }

    // Calculate rainfall depth: volume / area
    // Depth(mm) = (pulses * volume_per_pulse_mm3) / collection_area_mm2
    float total_volume_mm3 = (float) pulse_count * (WEATHER_RAIN_ML_PER_PULSE * 1000); // Multiplying due to conversion from cm3 (mL) to mm3.
    float rainfall_depth_mm = total_volume_mm3 / WEATHER_RAIN_COLLECTION_AREA_MM2;

    return rainfall_depth_mm;
}

/**
 * @brief Get current pulse count from rainfall sensor
 */
static esp_err_t tempesta_get_rainfall_pulse_count(int *pulse_count)
{
    if (!pulse_count) {
        return ESP_ERR_INVALID_ARG;
    }

    if (rain_pcnt_unit_handle == NULL) {
        *pulse_count = 0;
        return ESP_FAIL;
    }

    esp_err_t ret = pcnt_unit_get_count(rain_pcnt_unit_handle, pulse_count);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read rainfall pulse count: %s", esp_err_to_name(ret));
        *pulse_count = 0;
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Convert pulse count to tank intake volume in mL
 */
static float tempesta_convert_pulses_to_tank_intake_ml(int pulse_count)
{
    if (pulse_count < 0) {
        return 0.0f;
    }

    // Each pulse represents one tipbucket tip
    return (float) pulse_count * WEATHER_TANK_INTAKE_ML_PER_PULSE;
}

/**
 * @brief Get current pulse count from tank intake sensor
 */
static esp_err_t tempesta_get_tank_intake_pulse_count(int *pulse_count)
{
    if (!pulse_count) {
        return ESP_ERR_INVALID_ARG;
    }

    if (tank_intake_pcnt_unit_handle == NULL) {
        *pulse_count = 0;
        return ESP_FAIL;
    }

    esp_err_t ret = pcnt_unit_get_count(tank_intake_pcnt_unit_handle, pulse_count);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read tank intake pulse count: %s", esp_err_to_name(ret));
        *pulse_count = 0;
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Convert wind direction angle to cardinal/intercardinal direction
 */
static const char* tempesta_degrees_to_cardinal(float degrees)
{
    // Normalize to 0-360
    while (degrees < 0) degrees += 360.0f;
    while (degrees >= 360) degrees -= 360.0f;

    // 8-direction compass rose with 45° sectors
    // N: 337.5-22.5, NE: 22.5-67.5, E: 67.5-112.5, SE: 112.5-157.5
    // S: 157.5-202.5, SW: 202.5-247.5, W: 247.5-292.5, NW: 292.5-337.5

    if (degrees >= 337.5f || degrees < 22.5f) return "N";
    else if (degrees >= 22.5f && degrees < 67.5f) return "NE";
    else if (degrees >= 67.5f && degrees < 112.5f) return "E";
    else if (degrees >= 112.5f && degrees < 157.5f) return "SE";
    else if (degrees >= 157.5f && degrees < 202.5f) return "S";
    else if (degrees >= 202.5f && degrees < 247.5f) return "SW";
    else if (degrees >= 247.5f && degrees < 292.5f) return "W";
    else return "NW"; // 292.5 - 337.5
}

/**
 * @brief Daily reset callback triggered at midnight
 * Resets daily accumulation counters for rain gauge and tank intake
 * On weekly reset day (set at first boot): Also clears hardware counters to prevent overflow
 */
static void tempesta_daily_reset_callback(void)
{
    static const char *day_names[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
    static int weekly_reset_day = -1;  // -1 = not initialized, will be set on first run

    // Get current day of week (0=Sunday, 1=Monday, ..., 6=Saturday)
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    int day_of_week = timeinfo.tm_wday;

    // Initialize weekly reset day on first run (boot day)
    if (weekly_reset_day == -1) {
        weekly_reset_day = day_of_week;
        ESP_LOGI(TAG, "Weekly reset day initialized to %s (boot day)", day_names[weekly_reset_day]);
    }

    bool is_weekly_reset = (day_of_week == weekly_reset_day);

    if (is_weekly_reset) {
        ESP_LOGI(TAG, "%s midnight - HARDWARE COUNTER RESET + daily/weekly reset", day_names[day_of_week]);

        // Clear hardware counters to prevent overflow (max 32,767 pulses)
        // This ensures tank intake can't overflow during heavy rain weeks
        if (rain_pcnt_unit_handle != NULL) {
            pcnt_unit_clear_count(rain_pcnt_unit_handle);
        }
        if (tank_intake_pcnt_unit_handle != NULL) {
            pcnt_unit_clear_count(tank_intake_pcnt_unit_handle);
        }

        // Acquire mutex to protect calculation_data and weather_data access
        if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to acquire mutex in daily_reset_callback (Monday)");
            return;
        }

        // Reset all base pulse counts to 0 (hardware counters now at 0)
        calculation_data.rainfall_hourly_start_pulses = 0;
        calculation_data.rainfall_daily_base_pulses = 0;
        calculation_data.rainfall_weekly_base_pulses = 0;
        calculation_data.tank_intake_hourly_start_pulses = 0;
        calculation_data.tank_intake_daily_base_pulses = 0;
        calculation_data.tank_intake_weekly_base_pulses = 0;

        // Update weather data to reflect reset
        weather_data.rainfall_last_hour_mm = 0.0f;
        weather_data.rainfall_current_hour_mm = 0.0f;
        weather_data.rainfall_daily_mm = 0.0f;
        weather_data.rainfall_weekly_mm = 0.0f;
        weather_data.tank_intake_last_hour_ml = 0.0f;
        weather_data.tank_intake_current_hour_ml = 0.0f;
        weather_data.tank_intake_daily_ml = 0.0f;
        weather_data.tank_intake_weekly_ml = 0.0f;

        xSemaphoreGive(xTempestaDataMutex);

        ESP_LOGI(TAG, "Hardware counters cleared, all accumulators reset to 0");
    } else {
        ESP_LOGI(TAG, "Midnight reset - clearing daily accumulation counters");

        // Get current pulse counts to set as new daily base (hardware NOT cleared)
        int rain_pulses = 0, tank_pulses = 0;
        tempesta_get_rainfall_pulse_count(&rain_pulses);
        tempesta_get_tank_intake_pulse_count(&tank_pulses);

        // Acquire mutex to protect calculation_data and weather_data access
        if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to acquire mutex in daily_reset_callback");
            return;
        }

        // Update base pulse counts for daily tracking
        calculation_data.rainfall_daily_base_pulses = rain_pulses;
        calculation_data.tank_intake_daily_base_pulses = tank_pulses;

        // Update weather data to reflect daily reset
        weather_data.rainfall_daily_mm = 0.0f;
        weather_data.tank_intake_daily_ml = 0.0f;

        xSemaphoreGive(xTempestaDataMutex);

        ESP_LOGI(TAG, "Daily counters reset (rain=%d pulses, tank=%d pulses)", rain_pulses, tank_pulses);
    }
}

// ########################## Task Functions ##########################

/**
 * @brief Main weather station coordination task
 */
static void tempesta_main_task(void *pvParameters)
{
    const char *task_tag = "WeatherMain";
    ESP_LOGI(task_tag, "Weather station main task started");

    // Wait for system initialization
    vTaskDelay(pdMS_TO_TICKS(3000));

    while (1) {
        // Wait for collection cycle notification
        uint32_t notification_value = 0;
        xTaskNotifyWait(0x00, ULONG_MAX, &notification_value, portMAX_DELAY);

        // Check if collection should proceed based on state
        if (tempesta_state == TEMPESTA_STATE_DISABLED ||
            tempesta_state == TEMPESTA_STATE_SHUTDOWN ||
            tempesta_state == TEMPESTA_STATE_ERROR) {
            ESP_LOGD(task_tag, "System not operational (state %d) - skipping weather collection", tempesta_state);
            continue;
        }
        // Safety check: Don't collect if FLUCTUS is in CRITICAL (last line of defense)
        fluctus_power_state_t power_state = fluctus_get_power_state();
        if (power_state == FLUCTUS_POWER_STATE_CRITICAL) {
            ESP_LOGW(task_tag, "Critical power state - skipping weather collection cycle");
            continue;
        }

        // Save current state to restore after reading
        tempesta_state_t return_state = tempesta_state;

        // Transition to READING state
        tempesta_change_state(TEMPESTA_STATE_READING);

        ESP_LOGI(task_tag, "Starting weather data collection cycle");

        // Determine if PMS5003 should be read this cycle (FLUCTUS controls via enable flag)
        bool should_read_pms5003 = weather_pms5003_enabled;
        esp_err_t ret = tempesta_request_power_buses(should_read_pms5003);
        if (ret != ESP_OK) {
            ESP_LOGE(task_tag, "Failed to request power buses");
            tempesta_change_state(return_state);  // Restore previous state on error
            continue;
        }

        ESP_LOGI(task_tag, "Sensor buses powered (%s), starting measurements",
                 should_read_pms5003 ? "3V3+5V" : "3V3 only");

        // Record start time for PMS5003 warmup tracking and send wake command
        TickType_t warmup_start_time = xTaskGetTickCount();
        if (should_read_pms5003) {
            tempesta_pms5003_send_wake_command();
            ESP_LOGI(task_tag, "PMS5003 wake command sent, warmup starting");
        }

        // Allow I2C sensors brief stabilization and trigger wind sampling
        vTaskDelay(pdMS_TO_TICKS(1000));
        xTaskNotify(xTempestaAS5600TaskHandle, 1, eSetBits);

        // Read and process essential I2C sensors
        consolidated_sensor_data_t sensor_data;
        tempesta_read_env_sensors(&sensor_data);
        tempesta_process_temperature(&sensor_data);
        tempesta_process_humidity(&sensor_data);
        tempesta_process_pressure(&sensor_data);

        // Read pulse counter sensors (rainfall and tank intake)
        tempesta_read_and_process_rainfall();
        tempesta_read_and_process_tank_intake();

        // Read wind direction from Hall sensor array (ADS1115)
        tempesta_read_and_process_wind_direction();

        // Handle PMS5003 air quality sensor (with warmup management)
        tempesta_handle_pms5003_reading(warmup_start_time, should_read_pms5003);

        // Update timestamp and power down
        if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
            weather_data.snapshot_timestamp = time(NULL);
            xSemaphoreGive(xTempestaDataMutex);
        }

        // Inject data to TELEMETRY central hub
        telemetry_fetch_snapshot(TELEMETRY_SRC_TEMPESTA);

        tempesta_release_power_buses(should_read_pms5003);

        // Return to previous state (IDLE or POWER_SAVE)
        tempesta_change_state(return_state);

        ESP_LOGI(task_tag, "Weather data collection cycle completed");
        //tempesta_log_summary();
    }
}

/**
 * @brief AS5600 sampling task for wind speed measurement
 * Samples at 10Hz for 5 seconds when triggered, then calculates RPM
 */
static void tempesta_as5600_sampling_task(void *pvParameters)
{
    const char *task_tag = "WeatherAS5600";
    ESP_LOGI(task_tag, "AS5600 wind speed sampling task started");

    while (1) {
        // Wait for sampling trigger
        uint32_t notification_value = 0;
        xTaskNotifyWait(0x00, ULONG_MAX, &notification_value, portMAX_DELAY);

        ESP_LOGI(task_tag, "Starting AS5600 sampling for wind speed");

        // Wake AS5600 to normal mode for active sampling
        if (as5600_set_power_mode(&as5600_dev, AS5600_POWER_MODE_NORMAL) != ESP_OK) {
            ESP_LOGW(task_tag, "Failed to set AS5600 to normal mode");
        } else {
            ESP_LOGD(task_tag, "AS5600 set to NORMAL mode for sampling");
        }

        float total_rotations = 0.0f;
        uint16_t last_raw_angle = 0;
        bool first_reading = true;
        uint32_t valid_samples = 0;

        TickType_t start_time = xTaskGetTickCount();
        TickType_t sample_interval_ticks = pdMS_TO_TICKS(WEATHER_AS5600_SAMPLE_INTERVAL_MS);

        // Sample for the specified duration
        while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(WEATHER_AS5600_SAMPLE_DURATION_MS)) {
            uint16_t current_raw_angle;
            esp_err_t ret = as5600_read_raw_counts(&as5600_dev, &current_raw_angle);

            if (ret == ESP_OK) {
                if (!first_reading) {
                    // Calculate rotation difference (handling wrap-around)
                    int16_t angle_diff = current_raw_angle - last_raw_angle;

                    // Handle wrap-around at 4096 counts
                    if (angle_diff > 2048) {
                        angle_diff -= 4096;
                    } else if (angle_diff < -2048) {
                        angle_diff += 4096;
                    }

                    total_rotations += (float) angle_diff / 4096.0f;
                    valid_samples++;
                }

                last_raw_angle = current_raw_angle;
                first_reading = false;
            } else {
                ESP_LOGW(task_tag, "Failed to read AS5600: %s", esp_err_to_name(ret));
            }

            vTaskDelay(sample_interval_ticks);
        }

        // Calculate RPM
        float wind_speed_rpm = 0.0f;
        if (valid_samples > 0) {
            float sampling_duration_minutes = WEATHER_AS5600_SAMPLE_DURATION_MS / 60000.0f;
            wind_speed_rpm = fabs(total_rotations) / sampling_duration_minutes;

            ESP_LOGI(task_tag,
                     "Wind speed calculated: %.2f RPM (%" PRIu32 " samples, %.1f rotations)",
                     wind_speed_rpm,
                     valid_samples,
                     total_rotations);
        } else {
            ESP_LOGW(task_tag, "No valid AS5600 samples - wind sensor unavailable");
        }

        // Convert RPM to m/s using calibration factor
        float wind_speed_ms = WEATHER_INVALID_VALUE;
        if (valid_samples > 0) {
            // TODO: Calibrate WEATHER_WIND_RPM_TO_MS_FACTOR with triple cup anemometer measurements
            // Formula will depend on cup diameter, number of cups, and aerodynamic characteristics
            wind_speed_ms = wind_speed_rpm * WEATHER_WIND_RPM_TO_MS_FACTOR;
        }

        // Check AS5600 magnet status for hardware diagnostics
        as5600_status_t as5600_status;
        if (as5600_read_status(&as5600_dev, &as5600_status) == ESP_OK) {
            if (!as5600_status.magnet_detected) {
                ESP_LOGW(task_tag, "AS5600 magnet NOT detected - check wind sensor hardware");
            } else if (as5600_status.magnet_too_weak) {
                ESP_LOGW(task_tag, "AS5600 magnet too weak - increase proximity or use stronger magnet");
            } else if (as5600_status.magnet_too_strong) {
                ESP_LOGW(task_tag, "AS5600 magnet too strong - increase distance or use weaker magnet");
            }
        }

        // Update weather data
        if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
            weather_data.wind_speed_rpm = wind_speed_rpm;
            weather_data.wind_speed_ms = wind_speed_ms;
            weather_data.wind_sensor_status = (valid_samples > 0) ? WEATHER_SENSOR_OK : WEATHER_SENSOR_ERROR;
            xSemaphoreGive(xTempestaDataMutex);
        }

        // Put AS5600 back into LPM3 mode to save power during idle
        if (as5600_set_power_mode(&as5600_dev, AS5600_POWER_MODE_LPM3) != ESP_OK) {
            ESP_LOGW(task_tag, "Failed to set AS5600 to LPM3 mode");
        } else {
            ESP_LOGD(task_tag, "AS5600 set to LPM3 mode (idle power saving)");
        }

        ESP_LOGI(task_tag, "Wind speed sampling completed");
    }
}

// ########################## Public API Functions ##########################

/**
 * @brief Get comprehensive weather data snapshot (single mutex operation)
 */
esp_err_t tempesta_get_data_snapshot(tempesta_snapshot_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        // Single efficient copy of entire structure
        memcpy(data, &weather_data, sizeof(tempesta_snapshot_t));
        xSemaphoreGive(xTempestaDataMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief Write TEMPESTA data directly to TELEMETRY cache
 *
 * Comprehensive snapshot for HMI/MQTT distribution with all fields.
 * Writes directly to TELEMETRY's cache buffer to eliminate intermediate copy.
 * Only TEMPESTA mutex needed - TELEMETRY lock/unlock handled by caller.
 *
 * @param cache Pointer to TELEMETRY's tempesta_snapshot_t buffer
 * @return ESP_OK on success, ESP_FAIL if mutex timeout
 */
esp_err_t tempesta_write_to_telemetry_cache(tempesta_snapshot_t *cache)
{
    if (cache == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        // Single efficient copy of entire structure to TELEMETRY cache
        memcpy(cache, &weather_data, sizeof(tempesta_snapshot_t));
        xSemaphoreGive(xTempestaDataMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief Get current temperature for IMPLUVIUM integration (thread-safe)
 * @note This is a legacy function for irrigation temperature correction
 */
float tempesta_get_temperature(void)
{
    float temperature = WEATHER_INVALID_VALUE;

    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        temperature = weather_data.temperature;
        xSemaphoreGive(xTempestaDataMutex);
    }

    return temperature;
}

// ########################## Load Shedding Functions ##########################

/**
 * @brief Set power saving mode for TEMPESTA weather station
 */
esp_err_t tempesta_set_power_save_mode(bool enable)
{
    if (xTempestaDataMutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (enable) {
        // Enable power save: IDLE → POWER_SAVE
        if (tempesta_state == TEMPESTA_STATE_IDLE) {
            tempesta_change_state(TEMPESTA_STATE_POWER_SAVE);
            tempesta_update_timer_period();
        } else if (tempesta_state != TEMPESTA_STATE_POWER_SAVE) {
            ESP_LOGW(TAG, "Cannot enter power save mode from state %d", tempesta_state);
        }
    } else {
        // Disable power save: POWER_SAVE → IDLE
        if (tempesta_state == TEMPESTA_STATE_POWER_SAVE) {
            tempesta_change_state(TEMPESTA_STATE_IDLE);
            tempesta_update_timer_period();
        }
    }

    return ESP_OK;
}

/**
 * @brief Set shutdown state for TEMPESTA weather station (for load shedding)
 */
esp_err_t tempesta_set_shutdown(bool shutdown)
{
    if (xTempestaDataMutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (shutdown) {
        // Any state → SHUTDOWN
        if (tempesta_state != TEMPESTA_STATE_SHUTDOWN && tempesta_state != TEMPESTA_STATE_DISABLED) {
            tempesta_change_state(TEMPESTA_STATE_SHUTDOWN);
            // Stop the collection timer
            if (xTempestaCollectionTimer != NULL) {
                xTimerStop(xTempestaCollectionTimer, portMAX_DELAY);
            }
            // Ensure hall array is powered off during shutdown
            fluctus_hall_array_enable(false);
        }
    } else {
        // SHUTDOWN → previous state (or IDLE if previous was DISABLED)
        if (tempesta_state == TEMPESTA_STATE_SHUTDOWN) {
            tempesta_state_t target_state = tempesta_previous_state;
            if (target_state == TEMPESTA_STATE_DISABLED || target_state == TEMPESTA_STATE_SHUTDOWN) {
                target_state = TEMPESTA_STATE_IDLE;  // Default to IDLE if previous state was invalid
            }
            tempesta_change_state(target_state);
            // Restart the collection timer
            tempesta_update_timer_period(); // Set correct period first
            if (xTempestaCollectionTimer != NULL) {
                xTimerStart(xTempestaCollectionTimer, portMAX_DELAY);
            }
        }
    }

    return ESP_OK;
}

/**
 * @brief Enable/disable PMS5003 air quality sensor (for load shedding)
 */
esp_err_t tempesta_set_pms5003_enabled(bool enable)
{
    weather_pms5003_enabled = enable;

    if (enable) {
        ESP_LOGI(TAG, "PMS5003 air quality sensor enabled");
    } else {
        ESP_LOGI(TAG, "PMS5003 air quality sensor disabled for load shedding");
    }

    return ESP_OK;
}

/**
 * @brief Enable/disable TEMPESTA weather station system
 */
esp_err_t tempesta_set_system_enabled(bool enable)
{
    if (xTempestaDataMutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (enable) {
        // Enable: DISABLED → IDLE (or stay in current state if already enabled)
        if (tempesta_state == TEMPESTA_STATE_DISABLED) {
            tempesta_change_state(TEMPESTA_STATE_IDLE);
            // Start the collection timer
            if (xTempestaCollectionTimer != NULL) {
                tempesta_update_timer_period();
                xTimerStart(xTempestaCollectionTimer, portMAX_DELAY);
            }
            // Trigger immediate data collection
            if (xTempestaMainTaskHandle != NULL) {
                xTaskNotify(xTempestaMainTaskHandle, 1, eSetBits);
                ESP_LOGI(TAG, "Triggered immediate data collection on enable");
            }
        } else {
            ESP_LOGD(TAG, "System already enabled (current state: %d)", tempesta_state);
        }
    } else {
        // Disable: Any state → DISABLED
        if (tempesta_state != TEMPESTA_STATE_DISABLED) {
            tempesta_change_state(TEMPESTA_STATE_DISABLED);
            // Stop the collection timer
            if (xTempestaCollectionTimer != NULL) {
                xTimerStop(xTempestaCollectionTimer, portMAX_DELAY);
            }
        }
    }

    return ESP_OK;
}

/**
 * @brief Force immediate data collection cycle (outside normal schedule)
 */
esp_err_t tempesta_force_collection(void)
{
    if (xTempestaDataMutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Only allow forced collection if in IDLE or POWER_SAVE states
    if (tempesta_state != TEMPESTA_STATE_IDLE && tempesta_state != TEMPESTA_STATE_POWER_SAVE) {
        ESP_LOGW(TAG, "Cannot force collection - system not in operational state (current: %d)", tempesta_state);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Forcing immediate data collection cycle");

    // Trigger main task notification
    if (xTempestaMainTaskHandle != NULL) {
        xTaskNotify(xTempestaMainTaskHandle, 1, eSetBits);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief Reset daily accumulation counters (rainfall and tank intake)
 */
esp_err_t tempesta_reset_daily_counters(void)
{
    ESP_LOGI(TAG, "Resetting daily accumulation counters");

    // Get current pulse counts to set as new base
    int rain_pulses = 0, tank_pulses = 0;
    tempesta_get_rainfall_pulse_count(&rain_pulses);
    tempesta_get_tank_intake_pulse_count(&tank_pulses);

    // Acquire mutex to protect calculation_data and weather_data access
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for daily counter reset");
        return ESP_FAIL;
    }

    // Update base pulse counts for daily tracking
    calculation_data.rainfall_daily_base_pulses = rain_pulses;
    calculation_data.tank_intake_daily_base_pulses = tank_pulses;

    // Update weather data to reflect reset
    weather_data.rainfall_daily_mm = 0.0f;
    weather_data.tank_intake_daily_ml = 0.0f;

    xSemaphoreGive(xTempestaDataMutex);

    ESP_LOGI(TAG, "Daily counters reset complete (rain=%d pulses, tank=%d pulses)",
             rain_pulses, tank_pulses);
    return ESP_OK;
}

/**
 * @brief Reset weekly accumulation counters (rainfall and tank intake)
 */
esp_err_t tempesta_reset_weekly_counters(void)
{
    ESP_LOGI(TAG, "Resetting weekly accumulation counters");

    // Get current pulse counts to set as new base
    int rain_pulses = 0, tank_pulses = 0;
    tempesta_get_rainfall_pulse_count(&rain_pulses);
    tempesta_get_tank_intake_pulse_count(&tank_pulses);

    // Acquire mutex to protect calculation_data and weather_data access
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for weekly counter reset");
        return ESP_FAIL;
    }

    // Update base pulse counts for weekly tracking
    calculation_data.rainfall_weekly_base_pulses = rain_pulses;
    calculation_data.tank_intake_weekly_base_pulses = tank_pulses;

    // Update weather data to reflect reset
    weather_data.rainfall_weekly_mm = 0.0f;
    weather_data.tank_intake_weekly_ml = 0.0f;

    xSemaphoreGive(xTempestaDataMutex);

    ESP_LOGI(TAG, "Weekly counters reset complete (rain=%d pulses, tank=%d pulses)",
             rain_pulses, tank_pulses);
    return ESP_OK;
}

/**
 * @brief Reset rain gauge accumulation (clears hardware counter)
 * WARNING: This resets hourly/daily/weekly data. Consider using
 * tempesta_reset_daily_counters() or tempesta_reset_weekly_counters() instead.
 */
esp_err_t tempesta_reset_rain_gauge_total(void)
{
    ESP_LOGI(TAG, "Resetting rain gauge (hardware counter clear)");

    if (rain_pcnt_unit_handle == NULL) {
        return ESP_FAIL;
    }

    // Clear hardware counter
    esp_err_t ret = pcnt_unit_clear_count(rain_pcnt_unit_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear rain gauge pulse counter: %s", esp_err_to_name(ret));
        return ret;
    }

    // Acquire mutex to protect calculation_data and weather_data access
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for rain gauge reset");
        return ESP_FAIL;
    }

    // Reset all base pulse counts to 0
    calculation_data.rainfall_hourly_start_pulses = 0;
    calculation_data.rainfall_daily_base_pulses = 0;
    calculation_data.rainfall_weekly_base_pulses = 0;

    // Reset all accumulation values
    weather_data.rainfall_last_hour_mm = 0.0f;
    weather_data.rainfall_current_hour_mm = 0.0f;
    weather_data.rainfall_daily_mm = 0.0f;
    weather_data.rainfall_weekly_mm = 0.0f;

    xSemaphoreGive(xTempestaDataMutex);

    ESP_LOGI(TAG, "Rain gauge reset complete");
    return ESP_OK;
}

/**
 * @brief Reset tank intake accumulation (clears hardware counter)
 * WARNING: This resets hourly/daily/weekly data. Consider using
 * tempesta_reset_daily_counters() or tempesta_reset_weekly_counters() instead.
 */
esp_err_t tempesta_reset_tank_intake_total(void)
{
    ESP_LOGI(TAG, "Resetting tank intake (hardware counter clear)");

    if (tank_intake_pcnt_unit_handle == NULL) {
        return ESP_FAIL;
    }

    // Clear hardware counter
    esp_err_t ret = pcnt_unit_clear_count(tank_intake_pcnt_unit_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear tank intake pulse counter: %s", esp_err_to_name(ret));
        return ret;
    }

    // Acquire mutex to protect calculation_data and weather_data access
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for tank intake reset");
        return ESP_FAIL;
    }

    // Reset all base pulse counts to 0
    calculation_data.tank_intake_hourly_start_pulses = 0;
    calculation_data.tank_intake_daily_base_pulses = 0;
    calculation_data.tank_intake_weekly_base_pulses = 0;

    // Reset all accumulation values
    weather_data.tank_intake_last_hour_ml = 0.0f;
    weather_data.tank_intake_current_hour_ml = 0.0f;
    weather_data.tank_intake_daily_ml = 0.0f;
    weather_data.tank_intake_weekly_ml = 0.0f;

    xSemaphoreGive(xTempestaDataMutex);

    ESP_LOGI(TAG, "Tank intake reset complete");
    return ESP_OK;
}

/**
 * @brief Reset all accumulation counters (total, weekly, daily for both sensors)
 */
esp_err_t tempesta_reset_all_counters(void)
{
    ESP_LOGI(TAG, "Resetting ALL accumulation counters");

    esp_err_t ret1 = tempesta_reset_rain_gauge_total();
    esp_err_t ret2 = tempesta_reset_tank_intake_total();

    if (ret1 == ESP_OK && ret2 == ESP_OK) {
        ESP_LOGI(TAG, "All counters reset successfully");
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Some counters failed to reset (rain=%s, tank=%s)",
             esp_err_to_name(ret1), esp_err_to_name(ret2));
    return ESP_FAIL;
}