#ifndef __AS5600_H__
#define __AS5600_H__

#include <esp_err.h>
#include <stdint.h>
#include <stdbool.h>
#include "i2cdev.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AS5600_DEFAULT_ADDRESS 0x36 // Default I2C address

// Define the AS5600 device structure
typedef struct {
    i2c_dev_t i2c_dev;         // I2C device descriptor from i2cdev library
    float accumulated_counts;  // Accumulated counts including wraps
    float previous_raw_counts; // Last raw count reading (0-4095)
    uint16_t zero_offset;      // Offset subtracted from raw counts for relative readings
    TaskHandle_t update_task_handle; // Handle for the background update task
    SemaphoreHandle_t data_mutex; // Mutex to protect access to shared data (accumulated, previous)
    bool initialized;          // Flag indicating if the device is initialized
} as5600_dev_t;

/**
 * @brief Initialize the AS5600 descriptor.
 *
 * Sets up the I2C communication parameters for the device.
 *
 * @param dev Pointer to the as5600_dev_t structure to initialize.
 * @param port I2C port number (e.g., I2C_NUM_0).
 * @param addr The 7-bit I2C address of the AS5600 sensor (should be AS5600_DEFAULT_ADDRESS).
 * @param sda_gpio GPIO pin number for SDA.
 * @param scl_gpio GPIO pin number for SCL.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t as5600_init_desc(as5600_dev_t *dev, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free resources associated with the AS5600 descriptor.
 *
 * Stops the background task (if running), deletes the mutex, and frees the I2C device descriptor.
 *
 * @param dev Pointer to the initialized as5600_dev_t structure.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t as5600_free_desc(as5600_dev_t *dev);

/**
 * @brief Initialize the AS5600 sensor.
 *
 * Starts a background task to continuously read the sensor and 
 * track accumulated counts. Must be called after as5600_init_desc.
 *
 * @param dev Pointer to the as5600_dev_t structure with initialized descriptor.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t as5600_init(as5600_dev_t *dev);

/**
 * @brief Read the current angle in degrees (0 to 360).
 *
 * This reads the latest angle calculated based on the raw counts minus the zero offset.
 * The result wraps around from 360 to 0.
 *
 * @param dev Pointer to the initialized as5600_dev_t structure.
 * @param[out] angle Pointer to store the angle in degrees.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized, or I2C error code.
 */
esp_err_t as5600_read_angle_degrees(as5600_dev_t *dev, float *angle);

/**
 * @brief Read the current angle in radians (0 to 2*PI).
 *
 * This reads the latest angle calculated based on the raw counts minus the zero offset.
 * The result wraps around from 2*PI to 0.
 *
 * @param dev Pointer to the initialized as5600_dev_t structure.
 * @param[out] angle Pointer to store the angle in radians.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized, or I2C error code.
 */
esp_err_t as5600_read_angle_radians(as5600_dev_t *dev, float *angle);

/**
 * @brief Read the raw counts (0 to 4095) without applying the zero offset.
 *
 * This returns the most recent raw value read from the sensor by the background task.
 *
 * @param dev Pointer to the initialized as5600_dev_t structure.
 * @param[out] counts Pointer to store the raw counts.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized.
 */
esp_err_t as5600_read_raw_counts(as5600_dev_t *dev, uint16_t *counts);

/**
 * @brief Read the counts relative to the zero offset (0 to 4095).
 *
 * This reads the latest raw value and subtracts the zero offset, handling wrap-around.
 *
 * @param dev Pointer to the initialized as5600_dev_t structure.
 * @param[out] counts Pointer to store the relative counts.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized.
 */
esp_err_t as5600_read_relative_counts(as5600_dev_t *dev, uint16_t *counts);


/**
 * @brief Read the accumulated counts, including revolutions.
 *
 * This returns the total counts tracked by the background task, which accounts
 * for wrap-arounds (revolutions). Useful for tracking total rotation.
 *
 * @param dev Pointer to the initialized as5600_dev_t structure.
 * @param[out] counts Pointer to store the accumulated counts.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized.
 */
esp_err_t as5600_read_accumulated_counts(as5600_dev_t *dev, float *counts);

/**
 * @brief Set the current position as the zero offset.
 *
 * Future readings from `as5600_read_angle_*` and `as5600_read_relative_counts`
 * will be relative to the position at the time this function is called.
 *
 * @param dev Pointer to the initialized as5600_dev_t structure.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized.
 */
esp_err_t as5600_set_zero_offset(as5600_dev_t *dev);

/**
 * @brief Get the number of counts per full revolution (4096).
 *
 * @param dev Pointer to the initialized as5600_dev_t structure (can be NULL).
 * @return The number of counts per revolution (4096.0f).
 */
float as5600_get_counts_per_revolution(as5600_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif // __AS5600_H__ 