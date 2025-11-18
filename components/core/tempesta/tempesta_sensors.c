#include "tempesta.h"
#include "tempesta_private.h"
#include "fluctus.h"
#include "main.h"

#include "sht4x.h"
#include "bmp280.h"
#include "as5600.h"
#include "ads1115_helper.h"

#include <math.h>


static const char *TAG = "TEMPESTA_SENSOR";


/**
 * @brief Initialize hardware components (GPIO, UART, pulse counters)
 */
esp_err_t tempesta_hardware_init(void)
{
    esp_err_t ret;

    // Initialize pulse counters (rainfall and tank intake)
    ret = tempesta_pulse_sensors_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize pulse sensors: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize PMS5003 UART
    ESP_LOGE(TAG, "CHECKPOINT: Pulse sensors OK. Initializing PMS5003..."); // <-- ADD THIS
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
esp_err_t tempesta_i2c_sensors_init(void)
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
esp_err_t tempesta_pulse_sensors_init(void)
{
    esp_err_t ret;
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = -32768,
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
esp_err_t tempesta_pms5003_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_LOGE(TAG, "CHECKPOINT: Calling uart_driver_install on UART %d", WEATHER_PMS5003_UART_NUM);
    esp_err_t ret = uart_driver_install(WEATHER_PMS5003_UART_NUM, 256, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGE(TAG, "CHECKPOINT: Calling uart_param_config");
    ret = uart_param_config(WEATHER_PMS5003_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGE(TAG, "CHECKPOINT: Calling uart_set_pin (TX: %d, RX: %d)", WEATHER_PMS5003_TX_GPIO, WEATHER_PMS5003_RX_GPIO);
    ret = uart_set_pin(WEATHER_PMS5003_UART_NUM,
                       WEATHER_PMS5003_TX_GPIO,
                       WEATHER_PMS5003_RX_GPIO,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
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
esp_err_t tempesta_pms5003_send_sleep_command(void)
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
esp_err_t tempesta_pms5003_send_wake_command(void)
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
esp_err_t tempesta_read_env_sensors(consolidated_sensor_data_t *sensor_data)
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
esp_err_t tempesta_read_sht4x_all(consolidated_sensor_data_t *sensor_data)
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
esp_err_t tempesta_read_bmp280_all(consolidated_sensor_data_t *sensor_data)
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
esp_err_t tempesta_read_pms5003(pms5003_data_t *data, weather_sensor_status_t *status)
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
 * @brief Handle PMS5003 warmup and reading
 * @param warmup_start_time Tick count when power was applied
 * @param should_read_pms5003 Whether PMS5003 should be read this cycle
 */
void tempesta_handle_pms5003_reading(TickType_t warmup_start_time, bool should_read_pms5003)
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

/**
 * @brief Read Hall sensors and calculate wind direction using weighted averaging
 */
esp_err_t tempesta_read_and_process_wind_direction(void)
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

/**
 * @brief Convert wind direction angle to cardinal/intercardinal direction
 */
const char* tempesta_degrees_to_cardinal(float degrees)
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
 * @brief Get current pulse count from rainfall sensor
 */
esp_err_t tempesta_get_rainfall_pulse_count(int *pulse_count)
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
 * @brief Get current pulse count from tank intake sensor
 */
esp_err_t tempesta_get_tank_intake_pulse_count(int *pulse_count)
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
