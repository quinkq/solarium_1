#include "weather_station.h"
#include "main.h"
#include "fluctus.h"

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

#define TAG "TEMPESTA"

// Macro for updating weather data with mutex protection
#define WEATHER_UPDATE_DATA_WITH_STATUS(field, value, status_field, status_value) do { \
    if (xSemaphoreTake(xWeatherDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) { \
        weather_data.field = (value); \
        weather_data.status_field = (status_value); \
        xSemaphoreGive(xWeatherDataMutex); \
    } else { \
        ESP_LOGW(TAG, "Failed to acquire mutex for " #field " data update"); \
        return ESP_FAIL; \
    } \
} while(0)

// ########################## Global Variables ##########################

// Thread-safe data protection
static SemaphoreHandle_t xWeatherDataMutex = NULL;
static WeatherData weather_data = {0};
static weather_calculation_data_t calculation_data = {0};

// Load shedding state variables
static bool weather_power_save_mode = false;          // Power save mode (60min interval)
static bool weather_load_shed_shutdown = false;      // Load shedding shutdown flag  
static bool weather_pms5003_enabled = true;          // PMS5003 air quality sensor enabled

// Task and timer handles
static TaskHandle_t xWeatherMainTaskHandle = NULL;
static TaskHandle_t xWeatherAS5600TaskHandle = NULL;
static TimerHandle_t xWeatherCollectionTimer = NULL;

// Sensor device handles
static sht4x_t sht4x_dev = {0};
static bmp280_t bmp280_dev = {0};
static as5600_dev_t as5600_dev = {0};

// Hardware handles
static pcnt_unit_handle_t rain_pcnt_unit_handle = NULL;

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
static esp_err_t weather_hardware_init(void);
static esp_err_t weather_sensors_init(void);
static esp_err_t weather_rainfall_sensor_init(void);
static esp_err_t weather_pms5003_init(void);

// Low level sensor reading functions
static esp_err_t weather_read_env_sensors(consolidated_sensor_data_t *sensor_data);
static esp_err_t weather_read_sht4x_all(consolidated_sensor_data_t *sensor_data);
static esp_err_t weather_read_bmp280_all(consolidated_sensor_data_t *sensor_data);
static esp_err_t weather_read_pms5003(pms5003_data_t *data, weather_sensor_status_t *status);
static esp_err_t weather_read_and_process_rainfall(void);

// Data processing functions
static esp_err_t weather_process_temperature(const consolidated_sensor_data_t *sensor_data);
static esp_err_t weather_process_humidity(const consolidated_sensor_data_t *sensor_data);
static esp_err_t weather_process_pressure(const consolidated_sensor_data_t *sensor_data);
static esp_err_t weather_process_air_quality(void);
static float weather_process_sensor_averaging(float new_value,
                                              float *history_array,
                                              uint8_t array_size,
                                              uint8_t *history_index,
                                              uint8_t *history_count);

// Power management functions
static esp_err_t weather_request_power_buses(bool requires_5v);
static void weather_release_power_buses(bool had_5v);
static bool weather_should_collect_data(bool *reduced_functionality);
static void weather_update_timer_period(void);

// Task helper functions
static void weather_log_summary(void);
static void weather_handle_pms5003_reading(TickType_t warmup_start_time, bool should_read_pms5003);

// Utility functions
static void weather_timer_callback(TimerHandle_t xTimer);
static uint32_t weather_calculate_time_to_next_quarter_hour(void);
static float weather_convert_pulses_to_rainfall_mm(int pulse_count);
static esp_err_t weather_get_rainfall_pulse_count(int *pulse_count);

// Task functions
static void weather_main_task(void *pvParameters);
static void weather_as5600_sampling_task(void *pvParameters);

// ########################## Initialization Functions ##########################

/**
 * @brief Initialize weather station component
 */
esp_err_t weather_station_init(void)
{
    ESP_LOGI(TAG, "Initializing weather station component...");

    // Create mutex for thread-safe data access
    xWeatherDataMutex = xSemaphoreCreateMutex();
    if (xWeatherDataMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create weather data mutex");
        return ESP_FAIL;
    }

    // Initialize weather data with invalid values
    if (xSemaphoreTake(xWeatherDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
        weather_data.temperature = WEATHER_INVALID_VALUE;
        weather_data.humidity = WEATHER_INVALID_VALUE;
        weather_data.pressure = WEATHER_INVALID_VALUE;
        weather_data.air_quality_pm25 = WEATHER_INVALID_VALUE;
        weather_data.air_quality_pm10 = WEATHER_INVALID_VALUE;
        weather_data.wind_speed_rpm = WEATHER_INVALID_VALUE;
        weather_data.wind_speed_ms = WEATHER_INVALID_VALUE;
        weather_data.rainfall_mm = 0.0f;
        weather_data.timestamp = 0;

        // Initialize all sensor statuses as unavailable
        weather_data.temp_sensor_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.humidity_sensor_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.pressure_sensor_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.air_quality_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.wind_sensor_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.rain_sensor_status = WEATHER_SENSOR_UNAVAILABLE;

        xSemaphoreGive(xWeatherDataMutex);
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

    // Initialize rainfall tracking for hourly calculations
    calculation_data.last_rainfall_reset_time = time(NULL);
    calculation_data.rainfall_last_hour = 0.0f;

    // Initialize hardware and sensors
    esp_err_t ret = weather_hardware_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Hardware initialization failed: %s", esp_err_to_name(ret));
        goto cleanup_mutex;
    }

    ret = weather_sensors_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Some sensors failed to initialize: %s", esp_err_to_name(ret));
        // Continue anyway - individual sensors will be marked as unavailable
    }

    // Create collection timer - auto-reload repeating timer
    xWeatherCollectionTimer = xTimerCreate("WeatherTimer",
                                           pdMS_TO_TICKS(WEATHER_COLLECTION_INTERVAL_MS),
                                           pdTRUE, // Auto-reload
                                           (void *) 0,
                                           weather_timer_callback);
    if (xWeatherCollectionTimer == NULL) {
        ESP_LOGE(TAG, "Failed to create collection timer");
        goto cleanup_mutex;
    }

    // Create main weather task
    BaseType_t xResult = xTaskCreate(weather_main_task,
                                     "WeatherMain",
                                     2048, // Reduced stack - mostly I2C operations
                                     NULL,
                                     5,
                                     &xWeatherMainTaskHandle);
    if (xResult != pdPASS) {
        ESP_LOGE(TAG, "Failed to create main weather task");
        goto cleanup_timer;
    }

    // Create AS5600 sampling task
    xResult = xTaskCreate(weather_as5600_sampling_task,
                          "WeatherAS5600",
                          2048, // Reduced stack - simple calculations
                          NULL,
                          6, // Higher priority for time-sensitive wind measurements
                          &xWeatherAS5600TaskHandle);
    if (xResult != pdPASS) {
        ESP_LOGE(TAG, "Failed to create AS5600 sampling task");
        goto cleanup_main_task;
    }

    // Start timer aligned to next quarter-hour boundary
    uint32_t delay_ms = weather_calculate_time_to_next_quarter_hour();
    if (delay_ms > 0 && delay_ms <= (16 * 60 * 1000)) {
        // Start with calculated delay, timer will auto-repeat every 15 minutes
        ESP_LOGI(TAG, "Starting timer in %" PRIu32 "ms to align with quarter-hour", delay_ms);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    
    if (xTimerStart(xWeatherCollectionTimer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start collection timer");
        goto cleanup_as5600_task;
    }

    ESP_LOGI(TAG, "Weather station initialized successfully");
    return ESP_OK;

    // Cleanup on failure
cleanup_as5600_task:
    if (xWeatherAS5600TaskHandle) {
        vTaskDelete(xWeatherAS5600TaskHandle);
        xWeatherAS5600TaskHandle = NULL;
    }
cleanup_main_task:
    if (xWeatherMainTaskHandle) {
        vTaskDelete(xWeatherMainTaskHandle);
        xWeatherMainTaskHandle = NULL;
    }
cleanup_timer:
    if (xWeatherCollectionTimer) {
        xTimerDelete(xWeatherCollectionTimer, portMAX_DELAY);
        xWeatherCollectionTimer = NULL;
    }
cleanup_mutex:
    if (xWeatherDataMutex) {
        vSemaphoreDelete(xWeatherDataMutex);
        xWeatherDataMutex = NULL;
    }
    return ESP_FAIL;
}

/**
 * @brief Initialize hardware components (GPIO, UART, pulse counters)
 */
static esp_err_t weather_hardware_init(void)
{
    esp_err_t ret;

    // Initialize rainfall pulse counter
    ret = weather_rainfall_sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize rainfall sensor: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize PMS5003 UART
    ret = weather_pms5003_init();
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
static esp_err_t weather_sensors_init(void)
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
 * @brief Initialize HAL rainfall sensor pulse counter
 */
static esp_err_t weather_rainfall_sensor_init(void)
{
    // Create pulse counter unit configuration
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = 0,
    };

    esp_err_t ret = pcnt_new_unit(&unit_config, &rain_pcnt_unit_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create rainfall pulse counter unit: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create pulse counter channel configuration
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = WEATHER_RAINFALL_GPIO,
        .level_gpio_num = -1, // Not used
    };

    pcnt_channel_handle_t rain_pcnt_chan = NULL;
    ret = pcnt_new_channel(rain_pcnt_unit_handle, &chan_config, &rain_pcnt_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create rainfall pulse counter channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set edge action (count on positive edge only)
    ret =
        pcnt_channel_set_edge_action(rain_pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set rainfall pulse counter edge action: %s", esp_err_to_name(ret));
        return ret;
    }

    // Enable and start counting
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
    return ESP_OK;
}

/**
 * @brief Initialize PMS5003 UART interface
 */
static esp_err_t weather_pms5003_init(void)
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

// ########################## Low Level Sensor Reading Functions ##########################

/**
 * @brief Read all sensors once
 */
static esp_err_t weather_read_env_sensors(consolidated_sensor_data_t *sensor_data)
{
    // Initialize all readings as invalid
    sensor_data->sht4x.valid = false;
    sensor_data->bmp280.valid = false;

    // Read SHT4x (temperature and humidity)
    weather_read_sht4x_all(sensor_data);

    // Read BMP280/BME280 (temperature, pressure, and humidity if BME280)
    weather_read_bmp280_all(sensor_data);

    // Return success if at least one sensor read successfully
    return (sensor_data->sht4x.valid || sensor_data->bmp280.valid) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Read SHT4x sensor once for temperature and humidity
 */
static esp_err_t weather_read_sht4x_all(consolidated_sensor_data_t *sensor_data)
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
static esp_err_t weather_read_bmp280_all(consolidated_sensor_data_t *sensor_data)
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
static esp_err_t weather_read_pms5003(pms5003_data_t *data, weather_sensor_status_t *status)
{
    // Simple retry logic for UART communication issues
    for (int retry = 0; retry < WEATHER_PMS5003_RETRY_COUNT; retry++) {
        uint8_t buffer[32];

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
 * @brief Process rainfall accumulation with hourly calculations and data update
 */
static esp_err_t weather_read_and_process_rainfall(void)
{
    int pulse_count = 0;
    esp_err_t ret = weather_get_rainfall_pulse_count(&pulse_count);
    if (ret != ESP_OK) {
        // Update weather data with error state
        WEATHER_UPDATE_DATA_WITH_STATUS(rainfall_mm, WEATHER_INVALID_VALUE, rain_sensor_status, WEATHER_SENSOR_ERROR);
        return ret;
    }

    float rainfall_mm = weather_convert_pulses_to_rainfall_mm(pulse_count);
    ESP_LOGD(TAG,
             "Rainfall: %d pulses = %.3f mm (%.1f mm³/pulse, %.1f mm² area)",
             pulse_count,
             rainfall_mm,
             WEATHER_RAIN_MM3_PER_PULSE,
             WEATHER_RAIN_COLLECTION_AREA_MM2);

    // Calculate hourly rainfall rate
    time_t current_time = time(NULL);
    float hourly_rate = 0.0f;

    if (calculation_data.last_rainfall_reset_time > 0) {
        uint32_t time_diff_seconds = current_time - calculation_data.last_rainfall_reset_time;
        if (time_diff_seconds >= 3600) { // If an hour has passed
            hourly_rate = calculation_data.rainfall_last_hour;
            calculation_data.rainfall_last_hour = 0.0f; // Reset for next hour
            calculation_data.last_rainfall_reset_time = current_time;
        } else {
            // Calculate current hourly rate based on accumulation
            if (time_diff_seconds > 0) {
                hourly_rate = (rainfall_mm * 3600.0f) / time_diff_seconds;
            }
        }
    } else {
        calculation_data.last_rainfall_reset_time = current_time;
    }

    // Update weather data with successful reading
    WEATHER_UPDATE_DATA_WITH_STATUS(rainfall_mm, rainfall_mm, rain_sensor_status, WEATHER_SENSOR_OK);

    ESP_LOGD(TAG, "Rainfall: Total=%.3f mm, Hourly rate=%.3f mm/hr", rainfall_mm, hourly_rate);
    return ESP_OK;
}

// ########################## Data Processing Functions ##########################

/**
 * @brief Process temperature from sensors with integrated historical averaging and data update
 */
static esp_err_t weather_process_temperature(const consolidated_sensor_data_t *sensor_data)
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
        WEATHER_UPDATE_DATA_WITH_STATUS(temperature, WEATHER_INVALID_VALUE, temp_sensor_status, WEATHER_SENSOR_ERROR);
        return ESP_FAIL;
    }

    // Apply historical averaging
    averaged_temperature = weather_process_sensor_averaging(raw_temperature,
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
    WEATHER_UPDATE_DATA_WITH_STATUS(temperature, averaged_temperature, temp_sensor_status, status);

    return ESP_OK;
}

/**
 * @brief Process humidity from sensors with integrated historical averaging and data update
 * Combines readings from both sensors when available for improved accuracy
 */
static esp_err_t weather_process_humidity(const consolidated_sensor_data_t *sensor_data)
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
        WEATHER_UPDATE_DATA_WITH_STATUS(humidity, WEATHER_INVALID_VALUE, humidity_sensor_status, WEATHER_SENSOR_ERROR);
        return ESP_FAIL;
    }

    // Apply historical averaging
    averaged_humidity = weather_process_sensor_averaging(raw_humidity,
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
    WEATHER_UPDATE_DATA_WITH_STATUS(humidity, averaged_humidity, humidity_sensor_status, status);

    return ESP_OK;
}

/**
 * @brief Process pressure from sensor data and update weather data
 */
static esp_err_t weather_process_pressure(const consolidated_sensor_data_t *sensor_data)
{
    float pressure;
    weather_sensor_status_t status;

    if (sensor_data->bmp280.valid) {
        pressure = sensor_data->bmp280.pressure;
        status = WEATHER_SENSOR_OK;
        ESP_LOGD(TAG, "Pressure: %.1f hPa", pressure);

        // Update weather data with successful reading
        WEATHER_UPDATE_DATA_WITH_STATUS(pressure, pressure, pressure_sensor_status, status);
        return ESP_OK;
    }

    // Handle error case
    pressure = WEATHER_INVALID_VALUE;
    status = WEATHER_SENSOR_ERROR;
    ESP_LOGW(TAG, "Pressure sensor unavailable");

    // Update weather data with error state
    WEATHER_UPDATE_DATA_WITH_STATUS(pressure, WEATHER_INVALID_VALUE, pressure_sensor_status, WEATHER_SENSOR_ERROR);

    return ESP_FAIL;
}

/**
 * @brief Process air quality with PMS5003 warmup handling and data update
 */
static esp_err_t weather_process_air_quality(void)
{
    // Check if PMS5003 is disabled for load shedding
    if (!weather_pms5003_enabled) {
        ESP_LOGD(TAG, "PMS5003 sensor disabled for load shedding - skipping air quality reading");
        if (xSemaphoreTake(xWeatherDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
            weather_data.air_quality_status = WEATHER_SENSOR_UNAVAILABLE;
            xSemaphoreGive(xWeatherDataMutex);
        }
        return ESP_OK; // Not an error, just skipped
    }

    // Note: This function assumes the warmup time has already been handled by the caller
    // or that the sensor has been powered on for sufficient time

    pms5003_data_t air_quality;
    weather_sensor_status_t air_quality_status;

    esp_err_t ret = weather_read_pms5003(&air_quality, &air_quality_status);
    if (ret == ESP_OK) {
        // Update weather data with successful reading
        if (xSemaphoreTake(xWeatherDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
            weather_data.air_quality_pm25 = (float) air_quality.pm2_5_atm;
            weather_data.air_quality_pm10 = (float) air_quality.pm10_atm;
            weather_data.air_quality_status = air_quality_status;
            xSemaphoreGive(xWeatherDataMutex);
        } else {
            ESP_LOGW(TAG, "Failed to acquire mutex for air quality data update");
            return ESP_FAIL;
        }
    } else {
        // Update weather data with error state
        if (xSemaphoreTake(xWeatherDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
            weather_data.air_quality_pm25 = WEATHER_INVALID_VALUE;
            weather_data.air_quality_pm10 = WEATHER_INVALID_VALUE;
            weather_data.air_quality_status = WEATHER_SENSOR_ERROR;
            xSemaphoreGive(xWeatherDataMutex);
        }
    }

    return ret;
}

/**
 * @brief Unified sensor averaging algorithm for temperature and humidity
 * Uses weighted averaging with more recent samples getting higher weight
 */
static float weather_process_sensor_averaging(float new_value,
                                              float *history_array,
                                              uint8_t array_size,
                                              uint8_t *history_index,
                                              uint8_t *history_count)
{
    if (new_value == WEATHER_INVALID_VALUE) {
        return WEATHER_INVALID_VALUE;
    }

    // Add to circular buffer
    history_array[*history_index] = new_value;
    *history_index = (*history_index + 1) % array_size;

    if (*history_count < array_size) {
        (*history_count)++;
    }

    // Calculate weighted average (more recent samples get higher weight)
    float weighted_sum = 0.0f;
    float total_weight = 0.0f;

    for (uint8_t i = 0; i < *history_count; i++) {
        // Most recent sample gets highest weight
        uint8_t sample_idx = (*history_index - 1 - i + array_size) % array_size;
        float weight = 1.0f + (0.1f * i); // Recent samples weighted higher

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
static esp_err_t weather_request_power_buses(bool requires_5v)
{
    // Always request 3V3 for basic sensors
    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_3V3, "TEMPESTA", return ESP_FAIL);
    
    // Request 5V only if needed (PMS5003 enabled)
    if (requires_5v && weather_pms5003_enabled) {
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
static void weather_release_power_buses(bool had_5v)
{
    if (had_5v && weather_pms5003_enabled) {
        fluctus_release_bus_power(POWER_BUS_5V, "TEMPESTA");
    }
    fluctus_release_bus_power(POWER_BUS_3V3, "TEMPESTA");
}

/**
 * @brief Check if weather collection should proceed based on power state
 * @param[out] reduced_functionality Set to true if only essential sensors should be read
 * @return true if collection should proceed, false if should be skipped
 */
static bool weather_should_collect_data(bool *reduced_functionality)
{
    // Check for load shedding shutdown
    if (weather_load_shed_shutdown) {
        ESP_LOGD(TAG, "Load shedding shutdown active - skipping weather collection");
        return false;
    }

    // Check FLUCTUS power state
    fluctus_power_state_t power_state = fluctus_get_power_state();
    if (power_state == FLUCTUS_POWER_STATE_CRITICAL) {
        ESP_LOGW(TAG, "Critical power state - skipping weather collection cycle");
        return false;
    }

    // Determine functionality level
    *reduced_functionality = (power_state >= FLUCTUS_POWER_STATE_LOW_POWER);
    if (*reduced_functionality) {
        ESP_LOGI(TAG, "Low power mode - collecting essential sensors only");
    }

    return true;
}

/**
 * @brief Helper function to update timer period based on power save mode
 */
static void weather_update_timer_period(void)
{
    if (xWeatherCollectionTimer != NULL) {
        TickType_t new_period = weather_power_save_mode ?
            pdMS_TO_TICKS(WEATHER_COLLECTION_INTERVAL_POWER_SAVE_MS) :
            pdMS_TO_TICKS(WEATHER_COLLECTION_INTERVAL_MS);
        
        xTimerChangePeriod(xWeatherCollectionTimer, new_period, portMAX_DELAY);
    }
}

// ########################## Task Helper Functions ##########################

/**
 * @brief Log comprehensive weather summary with sensor status
 */
static void weather_log_summary(void)
{
    if (xSemaphoreTake(xWeatherDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        ESP_LOGI(TAG,
                 "Weather: T=%.1f°C, H=%.1f%%, P=%.1f hPa, PM2.5=%.0f µg/m³, Wind=%.1f RPM (%.1f m/s), Rain=%.2f mm",
                 weather_data.temperature,
                 weather_data.humidity,
                 weather_data.pressure,
                 weather_data.air_quality_pm25,
                 weather_data.wind_speed_rpm,
                 weather_data.wind_speed_ms,
                 weather_data.rainfall_mm);

        if (weather_data.temp_sensor_status != WEATHER_SENSOR_OK ||
            weather_data.humidity_sensor_status != WEATHER_SENSOR_OK ||
            weather_data.pressure_sensor_status != WEATHER_SENSOR_OK ||
            weather_data.air_quality_status != WEATHER_SENSOR_OK ||
            weather_data.wind_sensor_status != WEATHER_SENSOR_OK ||
            weather_data.rain_sensor_status != WEATHER_SENSOR_OK) {
            ESP_LOGW(TAG, "Weather station is not fully operational, check sensor status");
        }
        xSemaphoreGive(xWeatherDataMutex);
    }
}

/**
 * @brief Handle PMS5003 warmup and reading
 * @param warmup_start_time Tick count when power was applied
 * @param should_read_pms5003 Whether PMS5003 should be read this cycle
 */
static void weather_handle_pms5003_reading(TickType_t warmup_start_time, bool should_read_pms5003)
{
    if (!should_read_pms5003) {
        ESP_LOGI(TAG, "Skipping air quality measurement (PMS5003 disabled or low power mode)");
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
    weather_process_air_quality();
}

// ########################## Utility Functions ##########################

/**
 * @brief Timer callback to trigger data collection cycle
 */
static void weather_timer_callback(TimerHandle_t xTimer)
{
    // Notify main task to start collection cycle
    if (xWeatherMainTaskHandle != NULL) {
        xTaskNotify(xWeatherMainTaskHandle, 1, eSetBits);
    }
}

/**
 * @brief Calculate milliseconds until next quarter-hour boundary
 */
static uint32_t weather_calculate_time_to_next_quarter_hour(void)
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
static float weather_convert_pulses_to_rainfall_mm(int pulse_count)
{
    if (pulse_count < 0) {
        return 0.0f;
    }

    // Calculate rainfall depth: volume / area
    // Depth(mm) = (pulses * volume_per_pulse_mm3) / collection_area_mm2
    float total_volume_mm3 = (float) pulse_count * WEATHER_RAIN_MM3_PER_PULSE;
    float rainfall_depth_mm = total_volume_mm3 / WEATHER_RAIN_COLLECTION_AREA_MM2;

    return rainfall_depth_mm;
}

/**
 * @brief Get current pulse count from rainfall sensor
 */
static esp_err_t weather_get_rainfall_pulse_count(int *pulse_count)
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

// ########################## Task Functions ##########################

/**
 * @brief Main weather station coordination task
 */
static void weather_main_task(void *pvParameters)
{
    const char *task_tag = "WeatherMain";
    ESP_LOGI(task_tag, "Weather station main task started");

    // Wait for system initialization
    vTaskDelay(pdMS_TO_TICKS(3000));

    while (1) {
        // Wait for collection cycle notification
        uint32_t notification_value = 0;
        xTaskNotifyWait(0x00, ULONG_MAX, &notification_value, portMAX_DELAY);

        // Check if collection should proceed and determine functionality level
        bool reduced_functionality = false;
        if (!weather_should_collect_data(&reduced_functionality)) {
            continue; // Skip this cycle
        }

        ESP_LOGI(task_tag, "Starting weather data collection cycle");

        // Determine if PMS5003 should be read this cycle
        bool should_read_pms5003 = weather_pms5003_enabled && !reduced_functionality;
        esp_err_t ret = weather_request_power_buses(should_read_pms5003);
        if (ret != ESP_OK) {
            ESP_LOGE(task_tag, "Failed to request power buses");
            continue;
        }

        ESP_LOGI(task_tag, "Sensor buses powered (%s), starting measurements", 
                 should_read_pms5003 ? "3V3+5V" : "3V3 only");

        // Record start time for PMS5003 warmup tracking
        TickType_t warmup_start_time = xTaskGetTickCount();

        // Allow I2C sensors brief stabilization and trigger wind sampling
        vTaskDelay(pdMS_TO_TICKS(1000));
        xTaskNotify(xWeatherAS5600TaskHandle, 1, eSetBits);

        // Read and process essential I2C sensors
        consolidated_sensor_data_t sensor_data;
        weather_read_env_sensors(&sensor_data);
        weather_process_temperature(&sensor_data);
        weather_process_humidity(&sensor_data);
        weather_process_pressure(&sensor_data);
        weather_read_and_process_rainfall();

        // Handle PMS5003 air quality sensor (with warmup management)
        weather_handle_pms5003_reading(warmup_start_time, should_read_pms5003);

        // Update timestamp and power down
        if (xSemaphoreTake(xWeatherDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
            weather_data.timestamp = time(NULL);
            xSemaphoreGive(xWeatherDataMutex);
        }

        weather_release_power_buses(should_read_pms5003);
        
        ESP_LOGI(task_tag, "Weather data collection cycle completed");
        weather_log_summary();
    }
}

/**
 * @brief AS5600 sampling task for wind speed measurement
 * Samples at 10Hz for 5 seconds when triggered, then calculates RPM
 */
static void weather_as5600_sampling_task(void *pvParameters)
{
    const char *task_tag = "WeatherAS5600";
    ESP_LOGI(task_tag, "AS5600 wind speed sampling task started");

    while (1) {
        // Wait for sampling trigger
        uint32_t notification_value = 0;
        xTaskNotifyWait(0x00, ULONG_MAX, &notification_value, portMAX_DELAY);

        ESP_LOGI(task_tag, "Starting AS5600 sampling for wind speed");

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

        // Update weather data
        if (xSemaphoreTake(xWeatherDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
            weather_data.wind_speed_rpm = wind_speed_rpm;
            weather_data.wind_speed_ms = wind_speed_ms;
            weather_data.wind_sensor_status = (valid_samples > 0) ? WEATHER_SENSOR_OK : WEATHER_SENSOR_ERROR;
            xSemaphoreGive(xWeatherDataMutex);
        }

        ESP_LOGI(task_tag, "Wind speed sampling completed");
    }
}

// ########################## Public API Functions ##########################

/**
 * @brief Get current temperature (thread-safe)
 * This function replaces latest_sht_temp for irrigation system
 */
float weather_get_temperature(void)
{
    float temperature = WEATHER_INVALID_VALUE;
    
    if (xSemaphoreTake(xWeatherDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        temperature = weather_data.temperature;
        xSemaphoreGive(xWeatherDataMutex);
    }
    
    return temperature;
}

/**
 * @brief Get current humidity (thread-safe)
 */
float weather_get_humidity(void)
{
    float humidity = WEATHER_INVALID_VALUE;

    if (xSemaphoreTake(xWeatherDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        humidity = weather_data.humidity;
        xSemaphoreGive(xWeatherDataMutex);
    }

    return humidity;
}

/**
 * @brief Get current pressure (thread-safe)
 */
float weather_get_pressure(void)
{
    float pressure = WEATHER_INVALID_VALUE;

    if (xSemaphoreTake(xWeatherDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        pressure = weather_data.pressure;
        xSemaphoreGive(xWeatherDataMutex);
    }

    return pressure;
}

/**
 * @brief Get current air quality readings (thread-safe)
 */
esp_err_t weather_get_air_quality(float *pm25, float *pm10)
{
    if (!pm25 || !pm10) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xWeatherDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        *pm25 = weather_data.air_quality_pm25;
        *pm10 = weather_data.air_quality_pm10;
        
        // Check sensor status while still holding mutex
        weather_sensor_status_t status = weather_data.air_quality_status;
        xSemaphoreGive(xWeatherDataMutex);

        // Return success only if air quality sensor is working
        return (status == WEATHER_SENSOR_OK) ? ESP_OK : ESP_FAIL;
    }

    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Get current wind speed in RPM (thread-safe)
 */
float weather_get_wind_speed_rpm(void)
{
    float wind_speed = WEATHER_INVALID_VALUE;

    if (xSemaphoreTake(xWeatherDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        wind_speed = weather_data.wind_speed_rpm;
        xSemaphoreGive(xWeatherDataMutex);
    }

    return wind_speed;
}

/**
 * @brief Get current wind speed in m/s (thread-safe)
 */
float weather_get_wind_speed_ms(void)
{
    float wind_speed = WEATHER_INVALID_VALUE;

    if (xSemaphoreTake(xWeatherDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        wind_speed = weather_data.wind_speed_ms;
        xSemaphoreGive(xWeatherDataMutex);
    }

    return wind_speed;
}

/**
 * @brief Get accumulated rainfall and optionally reset counter (thread-safe)
 */
float weather_get_rainfall_mm(bool reset_counter)
{
    float rainfall = WEATHER_INVALID_VALUE;

    if (xSemaphoreTake(xWeatherDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        rainfall = weather_data.rainfall_mm;

        if (reset_counter && rain_pcnt_unit_handle != NULL) {
            // Reset the pulse counter
            esp_err_t ret = pcnt_unit_clear_count(rain_pcnt_unit_handle);
            if (ret == ESP_OK) {
                weather_data.rainfall_mm = 0.0f;
                ESP_LOGI(TAG, "Rainfall counter reset (was %.3f mm)", rainfall);
            } else {
                ESP_LOGW(TAG, "Failed to reset rainfall counter: %s", esp_err_to_name(ret));
            }
        }

        xSemaphoreGive(xWeatherDataMutex);
    }

    return rainfall;
}

/**
 * @brief Get human-readable sensor status
 */
esp_err_t weather_get_status_string(char *status_buffer, size_t buffer_size)
{
    if (!status_buffer || buffer_size < 200) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xWeatherDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        snprintf(status_buffer,
                 buffer_size,
                 "Weather Status: T:%s H:%s P:%s AQ:%s Wind:%s Rain:%s",
                 (weather_data.temp_sensor_status == WEATHER_SENSOR_OK) ? "OK" : "ERR",
                 (weather_data.humidity_sensor_status == WEATHER_SENSOR_OK) ? "OK" : "ERR",
                 (weather_data.pressure_sensor_status == WEATHER_SENSOR_OK) ? "OK" : "ERR",
                 (weather_data.air_quality_status == WEATHER_SENSOR_OK) ? "OK" : "ERR",
                 (weather_data.wind_sensor_status == WEATHER_SENSOR_OK) ? "OK" : "ERR",
                 (weather_data.rain_sensor_status == WEATHER_SENSOR_OK) ? "OK" : "ERR");

        xSemaphoreGive(xWeatherDataMutex);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

// ########################## Load Shedding Functions ##########################

/**
 * @brief Set power saving mode for TEMPESTA weather station
 */
esp_err_t tempesta_set_power_save_mode(bool enable)
{
    if (xWeatherDataMutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    weather_power_save_mode = enable;
    weather_update_timer_period();
    
    if (enable) {
        ESP_LOGI(TAG, "Power save mode enabled - weather collection interval extended to 60min");
    } else {
        ESP_LOGI(TAG, "Power save mode disabled - normal 15min collection interval restored");
    }
    
    return ESP_OK;
}

/**
 * @brief Set shutdown state for TEMPESTA weather station (for load shedding)
 */
esp_err_t tempesta_set_shutdown(bool shutdown)
{
    if (xWeatherDataMutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    weather_load_shed_shutdown = shutdown;
    
    if (shutdown) {
        ESP_LOGI(TAG, "Load shedding shutdown - weather monitoring disabled");
        // Stop the collection timer
        if (xWeatherCollectionTimer != NULL) {
            xTimerStop(xWeatherCollectionTimer, portMAX_DELAY);
        }
    } else {
        ESP_LOGI(TAG, "Load shedding shutdown lifted - weather monitoring restored");
        // Restart the collection timer
        weather_update_timer_period(); // Set correct period first
        if (xWeatherCollectionTimer != NULL) {
            xTimerStart(xWeatherCollectionTimer, portMAX_DELAY);
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