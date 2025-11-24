/**
 * @file tempesta_wind.c
 * @brief TEMPESTA wind speed monitoring with AS5600 magnetic encoder
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Dedicated task for time-sensitive wind speed measurements:
 * - AS5600 magnetic encoder sampling (10Hz for 5 seconds)
 * - RPM calculation from angle changes with wrap-around handling
 * - Conversion to m/s using calibration factor
 * - Low-power mode management (LPM3 during idle, 5mA savings)
 * - Independent task timing (5s sampling, separate from main cycle)
 * - Higher priority (6) for accurate time-critical measurements
 *
 * Runs autonomously, triggered by main task notifications. Isolated from
 * main collection cycle for precise timing control.
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "tempesta.h"
#include "tempesta_private.h"

#include "as5600.h"
#include <inttypes.h>
#include <math.h>

static const char *TAG = "TEMPESTA_WIND";


/**
 * @brief AS5600 sampling task for wind speed measurement
 * Samples at 10Hz for 5 seconds when triggered, then calculates RPM
 */
void tempesta_as5600_sampling_task(void *pvParameters)
{
    ESP_LOGI(TAG, "AS5600 wind speed sampling task started");

    while (1) {
        // Wait for sampling trigger
        uint32_t notification_value = 0;
        xTaskNotifyWait(0x00, ULONG_MAX, &notification_value, portMAX_DELAY);

        ESP_LOGI(TAG, "Starting AS5600 sampling for wind speed");

        // Wake AS5600 to normal mode for active sampling
        if (as5600_set_power_mode(&as5600_dev, AS5600_POWER_MODE_NORMAL) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set AS5600 to normal mode");
        } else {
            ESP_LOGD(TAG, "AS5600 set to NORMAL mode for sampling");
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
                ESP_LOGW(TAG, "Failed to read AS5600: %s", esp_err_to_name(ret));
            }

            vTaskDelay(sample_interval_ticks);
        }

        // Calculate RPM
        float wind_speed_rpm = 0.0f;
        if (valid_samples > 0) {
            float sampling_duration_minutes = WEATHER_AS5600_SAMPLE_DURATION_MS / 60000.0f;
            wind_speed_rpm = fabs(total_rotations) / sampling_duration_minutes;

            ESP_LOGI(TAG,
                     "Wind speed calculated: %.2f RPM (%" PRIu32 " samples, %.1f rotations)",
                     wind_speed_rpm,
                     valid_samples,
                     total_rotations);
        } else {
            ESP_LOGW(TAG, "No valid AS5600 samples - wind sensor unavailable");
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
                ESP_LOGW(TAG, "AS5600 magnet NOT detected - check wind sensor hardware");
            } else if (as5600_status.magnet_too_weak) {
                ESP_LOGW(TAG, "AS5600 magnet too weak - increase proximity or use stronger magnet");
            } else if (as5600_status.magnet_too_strong) {
                ESP_LOGW(TAG, "AS5600 magnet too strong - increase distance or use weaker magnet");
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
            ESP_LOGW(TAG, "Failed to set AS5600 to LPM3 mode");
        } else {
            ESP_LOGD(TAG, "AS5600 set to LPM3 mode (idle power saving)");
        }

        ESP_LOGI(TAG, "Wind speed sampling completed");
    }
}