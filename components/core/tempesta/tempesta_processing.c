#include "tempesta.h"
#include "tempesta_private.h"


static const char *TAG = "TEMPESTA_PROCESSING";

// ########################## Data Processing Functions ##########################

/**
 * @brief Process temperature from sensors with integrated historical averaging and data update
 */
esp_err_t tempesta_process_temperature(const consolidated_sensor_data_t *sensor_data)
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
esp_err_t tempesta_process_humidity(const consolidated_sensor_data_t *sensor_data)
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
esp_err_t tempesta_process_pressure(const consolidated_sensor_data_t *sensor_data)
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
esp_err_t tempesta_process_air_quality(void)
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
float tempesta_process_sensor_averaging(float new_value,
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
