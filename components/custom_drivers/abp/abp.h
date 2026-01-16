/*
 * Copyright (c) 2025 quinkq <quinkq@gmail.com>
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
 * @file abp.h
 * @defgroup abp abp
 * @{
 *
 * ESP-IDF driver for Honeywell ABP (Basic) series pressure sensors with SPI interface
 *
 * Supports differential pressure measurement with 14-bit resolution
 * Used Part number: ABPDJJT001PDSA3 (±1 psi differential, SPI, 3.3V)
 *
 * Based on the Honeywell ABP Series datasheet and ABP2 successor examples
 */

#ifndef __ABP_H__
#define __ABP_H__

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ABP sensor status bits (from datasheet)
 */
typedef enum {
    ABP_STATUS_NORMAL = 0x00,          ///< Normal operation
    ABP_STATUS_COMMAND_MODE = 0x01,    ///< Command mode (not used in basic sensors)
    ABP_STATUS_STALE_DATA = 0x02,      ///< Stale data
    ABP_STATUS_DIAGNOSTIC_FAULT = 0x03 ///< Diagnostic fault
} abp_status_t;

/**
 * @brief ABP sensor pressure ranges (ABPDJJT001PDSA3)
 */
typedef enum {
    ABP_RANGE_001PD = 0, ///< ±1 psi differential (ABPDJJT001PDSA3)
    ABP_RANGE_005PD,     ///< ±5 psi differential
    ABP_RANGE_015PD,     ///< ±15 psi differential
    ABP_RANGE_030PD,     ///< ±30 psi differential
    ABP_RANGE_060PD      ///< ±60 psi differential
} abp_pressure_range_t;

/**
 * @brief ABP sensor configuration structure
 */
typedef struct {
    spi_device_handle_t spi_dev; ///< SPI device handle
    gpio_num_t cs_pin;           ///< Chip select pin
    abp_pressure_range_t range;  ///< Pressure range
    float pressure_min;          ///< Minimum pressure (psi)
    float pressure_max;          ///< Maximum pressure (psi)
    uint32_t output_min;         ///< Minimum output count (10% of full scale)
    uint32_t output_max;         ///< Maximum output count (90% of full scale)
} abp_config_t;

/**
 * @brief ABP sensor device structure
 */
typedef struct {
    abp_config_t config; ///< Configuration
    bool initialized;    ///< Initialization status
} abp_t;

/**
 * @brief ABP sensor measurement data
 */
typedef struct {
    float pressure_psi;    ///< Pressure in PSI
    float pressure_mbar;   ///< Pressure in mbar
    float pressure_pa;     ///< Pressure in Pascal
    abp_status_t status;   ///< Sensor status
    uint32_t raw_pressure; ///< Raw pressure count (14-bit)
    bool valid;            ///< Data validity flag
} abp_data_t;

/**
 * @brief Predefined pressure range configurations
 */
extern const abp_config_t ABP_RANGE_CONFIGS[];

/**
 * @brief Initialize ABP sensor
 *
 * @param dev Pointer to ABP device structure
 * @param spi_host SPI host (SPI2_HOST or SPI3_HOST)
 * @param cs_pin Chip select GPIO pin
 * @param range Pressure range configuration
 * @return ESP_OK on success
 */
esp_err_t abp_init(abp_t *dev, spi_host_device_t spi_host, gpio_num_t cs_pin, abp_pressure_range_t range);

/**
 * @brief Deinitialize ABP sensor
 *
 * @param dev Pointer to ABP device structure
 * @return ESP_OK on success
 */
esp_err_t abp_deinit(abp_t *dev);

/**
 * @brief Read pressure data from ABP sensor
 *
 * @param dev Pointer to ABP device structure
 * @param data Pointer to data structure to store results
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t abp_read_pressure(abp_t *dev, abp_data_t *data);

/**
 * @brief Read pressure in mbar (optimized, single value)
 *
 * @param dev Pointer to ABP device structure
 * @param pressure_mbar Pointer to store pressure in mbar
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t abp_read_pressure_mbar(abp_t *dev, float *pressure_mbar);

/**
 * @brief Read pressure in PSI (optimized, single value)
 *
 * @param dev Pointer to ABP device structure
 * @param pressure_psi Pointer to store pressure in PSI
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t abp_read_pressure_psi(abp_t *dev, float *pressure_psi);

/**
 * @brief Read raw pressure data from ABP sensor
 *
 * @param dev Pointer to ABP device structure
 * @param raw_pressure Pointer to store raw 14-bit pressure value
 * @param status Pointer to store sensor status
 * @return ESP_OK on success
 */
esp_err_t abp_read_raw(abp_t *dev, uint32_t *raw_pressure, abp_status_t *status);

/**
 * @brief Convert raw pressure count to engineering units
 *
 * @param dev Pointer to ABP device structure
 * @param raw_pressure Raw pressure count
 * @param pressure_psi Pointer to store pressure in PSI
 * @return ESP_OK on success
 */
esp_err_t abp_convert_pressure(const abp_t *dev, uint32_t raw_pressure, float *pressure_psi);

/**
 * @brief Initialize SPI bus for ABP sensors
 *
 * This is a convenience function for SPI bus initialization.
 * You can also initialize the SPI bus manually if needed.
 *
 * Note: MOSI pin is not needed as ABP sensors are read-only (set to -1 internally)
 *
 * @param spi_host SPI host (SPI2_HOST or SPI3_HOST)
 * @param miso_pin MISO pin number
 * @param sclk_pin SCLK pin number
 * @return ESP_OK on success
 */
esp_err_t abp_spi_bus_init(spi_host_device_t spi_host, gpio_num_t miso_pin, gpio_num_t sclk_pin);

/**
 * @brief Deinitialize SPI bus
 *
 * @param spi_host SPI host to deinitialize
 * @return ESP_OK on success
 */
esp_err_t abp_spi_bus_deinit(spi_host_device_t spi_host);

/**
 * @brief Utility function to convert PSI to mbar
 *
 * @param psi Pressure in PSI
 * @return Pressure in mbar
 */
static inline float abp_psi_to_mbar(float psi)
{
    return psi * 68.9476f;
}

/**
 * @brief Utility function to convert PSI to Pascal
 *
 * @param psi Pressure in PSI
 * @return Pressure in Pascal
 */
static inline float abp_psi_to_pascal(float psi)
{
    return psi * 6894.76f;
}


#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __ABP_H__ */