/**
 * @file impluvium_sensors.c
 * @brief Moisture, pressure, and water level sensor reading functions
 *
 * Provides sensor abstraction layer for reading:
 * - Moisture levels (ADS1115)
 * - Outlet pressure (ABP via SPI2)
 * - Water tank level (ABP)
 */

#include "impluvium.h"
#include "impluvium_private.h"

static const char *TAG = "IMPLUVIUM_SENS";

// Sensor retry helper macro - to reduce duplication in retry logic
#define SENSOR_READ_WITH_RETRY(read_operation, sensor_name, context_id) do { \
    esp_err_t _ret = ESP_FAIL; \
    for (uint8_t _attempt = 0; _attempt < SENSOR_READ_MAX_RETRIES; _attempt++) { \
        _ret = (read_operation); \
        if (_ret == ESP_OK) { \
            if (_attempt > 0) { \
                ESP_LOGD(TAG, "%s %d succeeded on attempt %d", sensor_name, context_id, _attempt + 1); \
            } \
            break; \
        } \
        if (_attempt < SENSOR_READ_MAX_RETRIES - 1) { \
            ESP_LOGD(TAG, "%s %d attempt %d failed: %s, retrying...", \
                     sensor_name, context_id, _attempt + 1, esp_err_to_name(_ret)); \
            vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_RETRY_DELAY_MS)); \
        } \
    } \
    if (_ret != ESP_OK) { \
        ESP_LOGW(TAG, "Failed to read %s %d after %d attempts: %s", \
                 sensor_name, context_id, SENSOR_READ_MAX_RETRIES, esp_err_to_name(_ret)); \
    } \
    ret = _ret; \
} while(0)

/**
 * @brief Read moisture sensor for specific zone
 *
 * Reads ADS1115 voltage, converts to percentage using calibration constants,
 * and updates debug display mutex-protected variables.
 *
 * @param[in]  zone_id          Zone ID (0-4)
 * @param[out] moisture_percent Calculated moisture percentage (0-100%)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid parameters,
 *         ESP_ERR_INVALID_STATE if ADS1115 not available
 */
esp_err_t impluvium_read_moisture_sensor(uint8_t zone_id, float *moisture_percent)
{
    if (zone_id >= IRRIGATION_ZONE_COUNT || !moisture_percent) {
        return ESP_ERR_INVALID_ARG;
    }

    irrigation_zone_t *zone = &irrigation_zones[zone_id];

    // Check if ADS1115 device is available
    if (!ads1115_devices[zone->moisture_ads_device].initialized) {
        ESP_LOGW(TAG, "Zone %d moisture sensor ADS1115 #%d not available", zone_id, zone->moisture_ads_device);
        return ESP_ERR_INVALID_STATE;
    }

    // Read raw voltage from ADS1115 with retry logic
    int16_t raw;
    float voltage;
    esp_err_t ret = ESP_FAIL;

    SENSOR_READ_WITH_RETRY(
        ads1115_helper_read_channel(zone->moisture_ads_device, zone->moisture_channel, &raw, &voltage),
        "moisture sensor zone", zone_id
    );

    if (ret == ESP_OK) {
        // Success - convert voltage to percentage
        // Clamp voltage to calibrated range
        float clamped_voltage = voltage;
        if (clamped_voltage < MOISTURE_SENSOR_DRY_V)
            clamped_voltage = MOISTURE_SENSOR_DRY_V;
        if (clamped_voltage > MOISTURE_SENSOR_WET_V)
            clamped_voltage = MOISTURE_SENSOR_WET_V;

        // Linear mapping to percentage
        *moisture_percent =
            ((clamped_voltage - MOISTURE_SENSOR_DRY_V) / (MOISTURE_SENSOR_WET_V - MOISTURE_SENSOR_DRY_V)) * 100.0f;

        ESP_LOGD(TAG, "Zone %d moisture sensor read: %.1f%% (%.3fV)", zone_id, *moisture_percent, voltage);
    }

    return ret;
}

/**
 * @brief Read system outlet pressure sensor
 *
 * Reads outlet pressure from ADS1115 #2 Ch2, converts to bar using calibration,
 * and updates debug display variables.
 *
 * @param[out] outlet_pressure_bar System pressure in bar (0-3.0)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer,
 *         ESP_ERR_INVALID_STATE if ADS1115 not available
 */
esp_err_t impluvium_read_pressure(float *outlet_pressure_bar)
{
    if (!outlet_pressure_bar) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read pump pressure from ADS1115 #2 Ch2
    if (!ads1115_devices[1].initialized) {
        ESP_LOGD(TAG, "Pressure sensor ADS1115 #2 not available");
        return ESP_ERR_INVALID_STATE;
    }

    // Read pressure from ADS1115 with retry logic
    int16_t raw;
    float voltage;
    esp_err_t ret = ESP_FAIL;

    SENSOR_READ_WITH_RETRY(
        ads1115_helper_read_channel(1, ADS111X_MUX_1_GND, &raw, &voltage), // ADS111X_MUX_1_GND is 2nd channel on ADS1115
        "pressure sensor", 0
    );

    if (ret == ESP_OK) {
        // Success - convert voltage to pressure
        // TODO: calibration needed
        // Assuming linear conversion for now
        *outlet_pressure_bar = voltage * 3.0f; // For development time/tests: 3 bar per volt

        ESP_LOGD(TAG, "System pressure: %.2f bar (%.3fV)", *outlet_pressure_bar, voltage);
    }

    return ret;
}

/**
 * @brief Read water level sensor from ABP pressure sensor
 *
 * Reads water level using differential pressure measurement,
 * converts to percentage using calibration constants.
 *
 * @param[out] water_level_percent Water level percentage (0-100%)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 * @note Currently only a placeholder implementation
 */
esp_err_t impluvium_read_water_level(float *water_level_percent)
{
    if (!water_level_percent) {
        return ESP_ERR_INVALID_ARG;
    }

    float water_level_pressure_mbar = 0.0f;
    esp_err_t ret = ESP_FAIL;

    SENSOR_READ_WITH_RETRY(
        abp_read_pressure_mbar(&abp_dev, &water_level_pressure_mbar),
        "water level sensor", 0
    );

    if (ret == ESP_OK) {
        // Clamp pressure to calibrated range
        if (water_level_pressure_mbar < WATER_LEVEL_MIN_MBAR)
            water_level_pressure_mbar = WATER_LEVEL_MIN_MBAR;
        if (water_level_pressure_mbar > WATER_LEVEL_MAX_MBAR)
            water_level_pressure_mbar = WATER_LEVEL_MAX_MBAR;

        // Linear mapping to percentage
        *water_level_percent =
            ((water_level_pressure_mbar - WATER_LEVEL_MIN_MBAR) / (WATER_LEVEL_MAX_MBAR - WATER_LEVEL_MIN_MBAR)) *
            100.0f;

        ESP_LOGD(TAG, "Water level: %.1f%% (%.1f mbar)", *water_level_percent, water_level_pressure_mbar);
    }

    return ret;
}
