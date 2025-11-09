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
 * @brief Read absolute accumulated counts since initialization.
 *
 * Returns total counts including all revolutions, IGNORING zero_offset.
 * This tracks absolute rotation from device initialization.
 *
 * Example use case - Odometer / Total Distance:
 * - Wind turbine: Track total lifetime rotations for maintenance scheduling
 * - Robot wheels: Calculate total distance traveled over robot's lifetime
 * - Motor: Track wear by counting total shaft revolutions
 *
 * The zero_offset does NOT affect this value - setting a new zero point
 * for display purposes won't reset the lifetime accumulator.
 *
 * @param dev Pointer to the initialized as5600_dev_t structure.
 * @param[out] counts Pointer to store the absolute accumulated counts.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized.
 */
esp_err_t as5600_read_accumulated_counts(as5600_dev_t *dev, float *counts);

/**
 * @brief Read accumulated counts relative to zero offset.
 *
 * Returns total counts including all revolutions, APPLYING zero_offset.
 * This tracks rotation from the last set zero point (home position).
 *
 * Example use case - Multi-turn Position from Home:
 * - Robot arm joint: Track absolute position from calibrated home (e.g., 3.5 turns from home)
 * - Valve actuator: Know exact position relative to "fully closed" reference
 * - Telescope mount: Track position from calibration point through multiple rotations
 *
 * When you call as5600_set_zero_offset(), the accumulated relative count
 * resets to track position from that new reference point.
 *
 * @param dev Pointer to the initialized as5600_dev_t structure.
 * @param[out] counts Pointer to store the relative accumulated counts.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized.
 */
esp_err_t as5600_read_accumulated_counts_relative(as5600_dev_t *dev, float *counts);

/**
 * @brief Set the current position as the zero offset.
 *
 * Future readings from these functions will be relative to this position:
 * - as5600_read_angle_degrees()
 * - as5600_read_angle_radians()
 * - as5600_read_relative_counts()
 * - as5600_read_accumulated_counts_relative()
 *
 * This does NOT affect as5600_read_accumulated_counts(), which continues
 * tracking absolute total rotation from initialization.
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

/**
 * @brief Set AS5600 power mode for power consumption optimization.
 *
 * Configures the sensor's power mode by writing to the CONF register.
 * Different modes trade polling rate for power consumption:
 * - Normal (0): Always on, 6.5mA
 * - LPM1 (1): 5ms polling, 3.4mA
 * - LPM2 (2): 20ms polling, 1.8mA
 * - LPM3 (3): 100ms polling, 1.5mA
 *
 * @param dev Pointer to the initialized as5600_dev_t structure.
 * @param power_mode Power mode (0=Normal, 1=LPM1, 2=LPM2, 3=LPM3).
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t as5600_set_power_mode(as5600_dev_t *dev, uint8_t power_mode);

// Power mode constants for as5600_set_power_mode()
#define AS5600_POWER_MODE_NORMAL 0  // Normal mode: Always on, 6.5mA
#define AS5600_POWER_MODE_LPM1   1  // Low power 1: 5ms polling, 3.4mA
#define AS5600_POWER_MODE_LPM2   2  // Low power 2: 20ms polling, 1.8mA
#define AS5600_POWER_MODE_LPM3   3  // Low power 3: 100ms polling, 1.5mA

/**
 * @brief AS5600 status register information
 *
 * Contains magnet detection and strength status for hardware diagnostics.
 */
typedef struct {
    bool magnet_detected;   // True if magnet is detected
    bool magnet_too_weak;   // True if magnetic field is too weak (AGC min)
    bool magnet_too_strong; // True if magnetic field is too strong (AGC max)
} as5600_status_t;

/**
 * @brief Read AS5600 status register for hardware diagnostics.
 *
 * Provides information about magnet detection and field strength.
 * Useful for detecting hardware failures like magnet misalignment.
 *
 * Status conditions:
 * - magnet_detected: Magnet is within detection range
 * - magnet_too_weak: Increase magnet proximity or use stronger magnet
 * - magnet_too_strong: Increase distance or use weaker magnet
 *
 * @param dev Pointer to the initialized as5600_dev_t structure.
 * @param[out] status Pointer to store status information.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t as5600_read_status(as5600_dev_t *dev, as5600_status_t *status);

#ifdef __cplusplus
}
#endif

#endif // __AS5600_H__ 