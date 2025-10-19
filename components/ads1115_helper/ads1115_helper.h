#ifndef ADS1115_HELPER_H
#define ADS1115_HELPER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "ads111x.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

// ########################## Configuration Constants ##########################

#define ADS1115_DEVICE_COUNT 4
#define ADS1115_GAIN ADS111X_GAIN_4V096  // ±4.096V
#define ADS1115_I2C_PORT I2C_NUM_0

// Retry configuration
#define ADS1115_RETRY_DELAY_START_MS 2000
#define ADS1115_RETRY_DELAY_MAX_MS 60000

// ########################## Type Definitions ##########################

typedef struct {
    i2c_dev_t device;             // I2C device handle
    bool initialized;             // Device initialization status
    uint32_t last_retry_time;     // Last retry attempt timestamp
    uint8_t retry_count;          // Number of retry attempts
    uint32_t next_retry_delay_ms; // Next retry delay (exponential backoff)
    const char *name;             // Device name for logging
} ads1115_device_t;

extern ads1115_device_t ads1115_devices[ADS1115_DEVICE_COUNT];

// ########################## Public Function Declarations ##########################

/**
 * @brief Initialize the ADS1115 helper system
 * 
 * Initializes all ADS1115 devices and starts the retry task for automatic recovery.
 * Must be called before any other ADS1115 helper functions.
 * 
 * @return ESP_OK on success, ESP_FAIL if no devices could be initialized
 */
esp_err_t ads1115_helper_init(void);

/**
 * @brief Read a single channel from an ADS1115 device
 * 
 * Performs a single-shot conversion on the specified channel.
 * Automatically marks device as failed if communication errors occur.
 * 
 * @param device_id Device index (0 to ADS1115_DEVICE_COUNT-1)
 * @param channel Input channel/MUX setting (ADS111X_MUX_*)
 * @param raw Pointer to store raw ADC value (required)
 * @param voltage Pointer to store calculated voltage (optional, can be NULL)
 * @return ESP_OK on successful read
 * @return ESP_ERR_INVALID_ARG if device_id is invalid or raw is NULL
 * @return ESP_ERR_INVALID_STATE if device is not initialized
 * @return ESP_ERR_TIMEOUT if conversion times out
 * @return ESP_ERR_* on communication errors
 */
esp_err_t ads1115_read_channel(uint8_t device_id, ads111x_mux_t channel, int16_t *raw, float *voltage);

/**
 * @brief Check if a specific ADS1115 device is ready for use
 * 
 * @param device_id Device index (0 to ADS1115_DEVICE_COUNT-1)
 * @return true if device is initialized and ready
 * @return false if device is not initialized or device_id is invalid
 */
bool ads1115_is_device_ready(uint8_t device_id);

/**
 * @brief Get the latest voltage readings for display purposes
 * 
 * Thread-safe access to the latest ADC readings across all devices.
 * 
 * @param voltages 2D array to store voltages [device][channel]
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if voltages is NULL
 */
esp_err_t ads1115_get_latest_voltages(float voltages[ADS1115_DEVICE_COUNT][4]);

/**
 * @brief Get device status information
 * 
 * @param device_id Device index (0 to ADS1115_DEVICE_COUNT-1)
 * @param device_info Pointer to store device information (required)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if device_id is invalid or device_info is NULL
 */
esp_err_t ads1115_get_device_info(uint8_t device_id, ads1115_device_t *device_info);

#ifdef __cplusplus
}
#endif

#endif // ADS1115_HELPER_H