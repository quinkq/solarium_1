/*
 * Copyright (c) 2025 quinkq
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * @file abp.c
 * @brief Honeywell ABP pressure sensor driver (SPI interface)
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Original driver implementation for Honeywell ABP (Basic) series pressure sensors.
 * Based on Honeywell ABP Series datasheet and successor ABP2 examples.
 *
 * Key features:
 * - SPI interface for differential pressure sensing (max 2.5bar)
 * - 14-bit resolution with temperature compensation
 * - Status byte monitoring (normal/command mode/stale/diagnostic)
 * - Configurable pressure range and units
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <rom/ets_sys.h>
#include "abp.h"

static const char *TAG = "abp";

// SPI Configuration
#define ABP_SPI_CLOCK_SPEED_HZ 800000 // 800 kHz (datasheet max: 800 kHz)
#define ABP_SPI_MODE 0                // SPI mode 0 (CPOL=0, CPHA=0)
#define ABP_SPI_QUEUE_SIZE 1          // SPI transaction queue size

// Timing constants (from datasheet)
#define ABP_WAKEUP_TIME_US 100 // 100μs typical wake-up time after power-on

// Data format constants (from datasheet)
#define ABP_STATUS_MASK 0xC0     // Status bits [15:14]
#define ABP_STATUS_SHIFT 14      // Status position in 16-bit word
#define ABP_PRESSURE_MASK 0x3FFF // Pressure data bits [13:0]

// Output range constants for different pressure ranges
// These are 10% and 90% (Compensated Pressure Range) of full scale for 14-bit output (16384 counts)
#define ABP_OUTPUT_MIN_PERCENT 0.10f // 10% of full scale
#define ABP_OUTPUT_MAX_PERCENT 0.90f // 90% of full scale
#define ABP_OUTPUT_FULL_SCALE 16384  // 2^14 for 14-bit output

/**
 * @brief Predefined pressure range configurations
 *
 * Based on ABP series datasheet Table 7
 */
const abp_config_t ABP_RANGE_CONFIGS[] = {
    [ABP_RANGE_001PD] =
        {
            .range = ABP_RANGE_001PD,
            .pressure_min = -1.0f,
            .pressure_max = 1.0f,
            .output_min = (uint32_t) (ABP_OUTPUT_FULL_SCALE * ABP_OUTPUT_MIN_PERCENT), // 1638
            .output_max = (uint32_t) (ABP_OUTPUT_FULL_SCALE * ABP_OUTPUT_MAX_PERCENT)  // 14745
        },
    [ABP_RANGE_005PD] = {.range = ABP_RANGE_005PD,
                         .pressure_min = -5.0f,
                         .pressure_max = 5.0f,
                         .output_min = (uint32_t) (ABP_OUTPUT_FULL_SCALE * ABP_OUTPUT_MIN_PERCENT),
                         .output_max = (uint32_t) (ABP_OUTPUT_FULL_SCALE * ABP_OUTPUT_MAX_PERCENT)},
    [ABP_RANGE_015PD] = {.range = ABP_RANGE_015PD,
                         .pressure_min = -15.0f,
                         .pressure_max = 15.0f,
                         .output_min = (uint32_t) (ABP_OUTPUT_FULL_SCALE * ABP_OUTPUT_MIN_PERCENT),
                         .output_max = (uint32_t) (ABP_OUTPUT_FULL_SCALE * ABP_OUTPUT_MAX_PERCENT)},
    [ABP_RANGE_030PD] = {.range = ABP_RANGE_030PD,
                         .pressure_min = -30.0f,
                         .pressure_max = 30.0f,
                         .output_min = (uint32_t) (ABP_OUTPUT_FULL_SCALE * ABP_OUTPUT_MIN_PERCENT),
                         .output_max = (uint32_t) (ABP_OUTPUT_FULL_SCALE * ABP_OUTPUT_MAX_PERCENT)},
    [ABP_RANGE_060PD] = {.range = ABP_RANGE_060PD,
                         .pressure_min = -60.0f,
                         .pressure_max = 60.0f,
                         .output_min = (uint32_t) (ABP_OUTPUT_FULL_SCALE * ABP_OUTPUT_MIN_PERCENT),
                         .output_max = (uint32_t) (ABP_OUTPUT_FULL_SCALE * ABP_OUTPUT_MAX_PERCENT)}};

#define CHECK_ARG(VAL)                                                                                                 \
    do {                                                                                                               \
        if (!(VAL))                                                                                                    \
            return ESP_ERR_INVALID_ARG;                                                                                \
    } while (0)

/**
 * @brief Perform SPI transaction to read data from ABP sensor
 */
static esp_err_t abp_spi_read(abp_t *dev, uint8_t *rx_data, size_t length)
{
    CHECK_ARG(dev && rx_data && length > 0);

    if (!dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t trans = {
        .length = length * 8, // Length in bits
        .rx_buffer = rx_data,
        .tx_buffer = NULL // No data to send
    };

    esp_err_t ret = spi_device_transmit(dev->config.spi_dev, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t abp_init(abp_t *dev, spi_host_device_t spi_host, gpio_num_t cs_pin, abp_pressure_range_t range)
{
    CHECK_ARG(dev);
    CHECK_ARG(range < sizeof(ABP_RANGE_CONFIGS) / sizeof(ABP_RANGE_CONFIGS[0]));

    if (dev->initialized) {
        ESP_LOGW(TAG, "ABP sensor already initialized");
        return ESP_OK;
    }

    // Copy configuration from predefined range
    memcpy(&dev->config, &ABP_RANGE_CONFIGS[range], sizeof(abp_config_t));
    dev->config.cs_pin = cs_pin;

    // Configure SPI device
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = ABP_SPI_CLOCK_SPEED_HZ,
        .mode = ABP_SPI_MODE,
        .spics_io_num = cs_pin,
        .queue_size = ABP_SPI_QUEUE_SIZE,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .duty_cycle_pos = 128, // 50% duty cycle
    };

    esp_err_t ret = spi_bus_add_device(spi_host, &dev_config, &dev->config.spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    dev->initialized = true;

    // Allow sensor to wake up before first reading
    ets_delay_us(ABP_WAKEUP_TIME_US);

    ESP_LOGI(TAG, "ABP sensor initialized: range=±%.1f psi, CS pin=%d", dev->config.pressure_max, cs_pin);

    return ESP_OK;
}

esp_err_t abp_deinit(abp_t *dev)
{
    CHECK_ARG(dev);

    if (!dev->initialized) {
        return ESP_OK;
    }

    esp_err_t ret = spi_bus_remove_device(dev->config.spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    dev->initialized = false;
    ESP_LOGI(TAG, "ABP sensor deinitialized");

    return ESP_OK;
}

esp_err_t abp_read_raw(abp_t *dev, uint32_t *raw_pressure, abp_status_t *status)
{
    CHECK_ARG(dev && raw_pressure && status);

    if (!dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Read 2 bytes from sensor
    uint8_t rx_data[2] = {0};
    esp_err_t ret = abp_spi_read(dev, rx_data, sizeof(rx_data));
    if (ret != ESP_OK) {
        return ret;
    }

    // Combine bytes into 16-bit value (MSB first)
    uint16_t raw_data = (rx_data[0] << 8) | rx_data[1];

    // Extract status bits (bits 15:14)
    *status = (abp_status_t) ((raw_data & ABP_STATUS_MASK) >> ABP_STATUS_SHIFT);

    // Extract pressure data (bits 13:0)
    *raw_pressure = raw_data & ABP_PRESSURE_MASK;

    ESP_LOGD(TAG, "Raw data: 0x%04X, Status: %d, Pressure: %" PRIu32, raw_data, *status, *raw_pressure);

    return ESP_OK;
}

esp_err_t abp_convert_pressure(const abp_t *dev, uint32_t raw_pressure, float *pressure_psi)
{
    CHECK_ARG(dev && pressure_psi);

    // Apply transfer function from datasheet:
    // P_applied = ((output - output_min) * (P_max - P_min)) / (output_max - output_min) + P_min

    float output_range = (float) (dev->config.output_max - dev->config.output_min);
    float pressure_range = dev->config.pressure_max - dev->config.pressure_min;

    if (output_range == 0.0f) {
        ESP_LOGE(TAG, "Invalid output range configuration");
        return ESP_ERR_INVALID_ARG;
    }

    *pressure_psi = ((float) raw_pressure - (float) dev->config.output_min) * pressure_range / output_range +
                    dev->config.pressure_min;

    return ESP_OK;
}

esp_err_t abp_read_pressure(abp_t *dev, abp_data_t *data)
{
    CHECK_ARG(dev && data);

    // Initialize data structure
    memset(data, 0, sizeof(abp_data_t));

    // Read raw data
    uint32_t raw_pressure;
    abp_status_t status;

    esp_err_t ret = abp_read_raw(dev, &raw_pressure, &status);
    if (ret != ESP_OK) {
        data->valid = false;
        return ret;
    }

    // Store raw data and status
    data->raw_pressure = raw_pressure;
    data->status = status;

    // Check sensor status - only accept normal operation
    if (status != ABP_STATUS_NORMAL) {
        data->valid = false;

        if (status == ABP_STATUS_DIAGNOSTIC_FAULT) {
            ESP_LOGW(TAG, "Sensor diagnostic fault detected");
            return ESP_ERR_INVALID_RESPONSE;
        }

        if (status == ABP_STATUS_STALE_DATA) {
            ESP_LOGW(TAG, "Stale data detected");
            return ESP_ERR_TIMEOUT;
        }

        if (status == ABP_STATUS_COMMAND_MODE) {
            ESP_LOGW(TAG, "Sensor is in command mode");
            return ESP_ERR_INVALID_STATE;
        }

        // Unknown status code
        ESP_LOGW(TAG, "Unknown sensor status: %d", status);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Convert to engineering units
    ret = abp_convert_pressure(dev, raw_pressure, &data->pressure_psi);
    if (ret != ESP_OK) {
        data->valid = false;
        return ret;
    }

    // Convert to other units
    data->pressure_mbar = abp_psi_to_mbar(data->pressure_psi);
    data->pressure_pa = abp_psi_to_pascal(data->pressure_psi);
    data->valid = true;

    ESP_LOGD(TAG, "Pressure: %.3f psi, %.1f mbar, %.0f Pa", data->pressure_psi, data->pressure_mbar, data->pressure_pa);

    return ESP_OK;
}

esp_err_t abp_read_pressure_mbar(abp_t *dev, float *pressure_mbar)
{
    CHECK_ARG(dev && pressure_mbar);

    abp_data_t data;
    esp_err_t ret = abp_read_pressure(dev, &data);

    if (ret == ESP_OK) {
        *pressure_mbar = data.pressure_mbar;
    }

    return ret;
}

esp_err_t abp_read_pressure_psi(abp_t *dev, float *pressure_psi)
{
    CHECK_ARG(dev && pressure_psi);

    abp_data_t data;
    esp_err_t ret = abp_read_pressure(dev, &data);

    if (ret == ESP_OK) {
        *pressure_psi = data.pressure_psi;
    }

    return ret;
}

esp_err_t abp_spi_bus_init(spi_host_device_t spi_host, gpio_num_t miso_pin, gpio_num_t sclk_pin)
{
    spi_bus_config_t bus_config = {
        .miso_io_num = miso_pin,
        .mosi_io_num = -1, // Not used for read-only sensor
        .sclk_io_num = sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64, // Small transfers for pressure readings
    };

    esp_err_t ret = spi_bus_initialize(spi_host, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ABP SPI bus initialized successfully");
    return ESP_OK;
}

esp_err_t abp_spi_bus_deinit(spi_host_device_t spi_host)
{
    esp_err_t ret = spi_bus_free(spi_host);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deinitialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ABP SPI bus deinitialized");
    return ESP_OK;
}