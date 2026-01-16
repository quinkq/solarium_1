/*
 * Example usage of ABP pressure sensor driver
 *
 * This file demonstrates how to integrate the ABP driver into your project
 */

#include "abp.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"

static const char *TAG = "abp_example";

// ABP Pin definitions - adjust for your hardware
#define ABP_CS_PIN GPIO_NUM_5
#define ABP_SPI_HOST SPI2_HOST
#define ABP_MISO_PIN GPIO_NUM_19
#define ABP_SCLK_PIN GPIO_NUM_18

// Example function: Initialize and read pressure sensor
esp_err_t abp_example_init_and_read(void)
{
    esp_err_t ret;
    abp_t sensor = {0};
    abp_data_t data;

    // Initialize SPI bus (call this once in your main application)
    ret = abp_spi_bus_init(ABP_SPI_HOST, ABP_MISO_PIN, ABP_SCLK_PIN);
    if (ret != ESP_OK) {
        return ret;
    }

    // Initialize ABP sensor for ±1 psi differential pressure
    ret = abp_init(&sensor, ABP_SPI_HOST, ABP_CS_PIN, ABP_RANGE_001PD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ABP sensor: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for sensor to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));


    // Read pressure data
    ret = abp_read_pressure(&sensor, &data);
    /*
    // Read pressure data (just get mbar value)
    float pressure_mbar;
    ret = abp_read_pressure_mbar(&sensor, &pressure_mbar);
    */
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read pressure: %s", esp_err_to_name(ret));
        abp_deinit(&sensor);
        return ret;
    }

    if (data.valid) {
        ESP_LOGI(TAG, "Pressure reading:");
        ESP_LOGI(TAG, "  Raw value: %" PRIu32, data.raw_pressure);
        ESP_LOGI(TAG, "  Pressure: %.3f psi", data.pressure_psi);
        ESP_LOGI(TAG, "  Pressure: %.1f mbar", data.pressure_mbar);
        ESP_LOGI(TAG, "  Pressure: %.0f Pa", data.pressure_pa);
        ESP_LOGI(TAG, "  Status: %d", data.status);
    } else {
        ESP_LOGW(TAG, "Invalid pressure reading");
    }
    // ESP_LOGI(TAG, "Pressure reading: %.1f mbar", pressure_mbar);

    // Clean up
    ret = abp_deinit(&sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deinitialize sensor: %s", esp_err_to_name(ret));
    }

    // Clean up SPI bus if no other devices are using it
    abp_spi_bus_deinit(ABP_SPI_HOST);

    return ESP_OK;
}

// Example task for continuous pressure monitoring
void abp_monitor_task(void *pvParameters)
{
    abp_t *sensor = (abp_t *) pvParameters;
    abp_data_t data;
    // float pressure_mbar;
    TickType_t last_wake_time = xTaskGetTickCount();

    ESP_LOGI(TAG, "Starting ABP pressure monitoring task");

    while (1) {
        esp_err_t ret = abp_read_pressure(sensor, &data);
        // esp_err_t ret = abp_read_pressure_mbar(sensor, &pressure_mbar);

        if (ret == ESP_OK && data.valid) {
            // if (ret == ESP_OK) {
            //  Log pressure reading every 10 readings to avoid spam
            static int log_counter = 0;
            if (++log_counter >= 10) {
                ESP_LOGI(TAG, "Pressure: %.3f psi (%.1f mbar)", data.pressure_psi, data.pressure_mbar);
                // ESP_LOGI(TAG, "Pressure: %.1f mbar", pressure_mbar);
                log_counter = 0;
            }

            // You can add your application logic here:
            // - Convert to water level percentage
            // - Send data over WiFi/MQTT
            // - Store in NVS
            // - Trigger alarms based on thresholds
            // - etc.

        } else {
            ESP_LOGW(TAG, "Failed to read pressure data");
        }

        // Read every 1 second
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000));
    }
}

// Example function to start monitoring task
esp_err_t abp_start_monitoring(abp_t *sensor)
{
    if (!sensor || !sensor->initialized) {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_ARG;
    }

    BaseType_t result = xTaskCreate(abp_monitor_task,
                                    "abp_monitor",
                                    4096,   // Stack size
                                    sensor, // Task parameter
                                    5,      // Priority
                                    NULL    // Task handle
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitoring task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ABP monitoring task started");
    return ESP_OK;
}