#ifndef STELLARIA_H
#define STELLARIA_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// ########################## STELLARIA Configuration ##########################

// GPIO Configuration
#define STELLARIA_ENABLE_GPIO       GPIO_NUM_0   // Driver enable signal
#define STELLARIA_PWM_GPIO          GPIO_NUM_45  // PWM intensity control

// PWM Configuration
#define STELLARIA_PWM_FREQUENCY     1000         // 1kHz PWM frequency
#define STELLARIA_PWM_RESOLUTION    LEDC_TIMER_10_BIT  // 10-bit resolution (0-1023)
#define STELLARIA_PWM_TIMER         LEDC_TIMER_1
#define STELLARIA_PWM_CHANNEL       LEDC_CHANNEL_0
#define STELLARIA_PWM_SPEED_MODE    LEDC_LOW_SPEED_MODE

// Intensity Limits
#define STELLARIA_MIN_INTENSITY     0            // Minimum intensity (0%)
#define STELLARIA_MAX_INTENSITY     1023         // Maximum intensity (100%)
#define STELLARIA_POWER_SAVE_LIMIT  205          // 20% intensity for power saving mode

// Mutex timeout constants
#define STELLARIA_MUTEX_TIMEOUT_MS  100          // Mutex timeout for operations

// Light Sensing Configuration
#define STELLARIA_LIGHT_TURN_ON_THRESHOLD   0.4f  // Turn ON when light < 0.4V (dark)
#define STELLARIA_LIGHT_TURN_OFF_THRESHOLD  0.3f  // Turn OFF when light > 0.3V (bright)
#define STELLARIA_LIGHT_CHECK_INTERVAL_MS   30000 // Check light every 30 seconds

// ########################## Data Structures ##########################

typedef enum {
    STELLARIA_STATE_DISABLED = 0,    // System disabled
    STELLARIA_STATE_ENABLED,         // System enabled, normal operation
    STELLARIA_STATE_POWER_SAVE,      // Power saving mode (limited intensity)
    STELLARIA_STATE_SHUTDOWN,        // Load shedding shutdown
    STELLARIA_STATE_AUTO             // Automatic light sensing mode
} stellaria_state_t;

typedef struct {
    stellaria_state_t state;         // Current system state
    uint16_t current_intensity;      // Current PWM intensity (0-1023)
    uint16_t target_intensity;       // Target intensity requested
    bool driver_enabled;             // Driver enable state
    bool initialized;                // Component initialization status
    bool auto_mode_active;           // Auto light sensing mode active
    float last_light_reading;        // Last averaged light reading (V)
} stellaria_status_t;

// ########################## Public API Functions ##########################

/**
 * @brief Initialize STELLARIA ambient lighting system
 * @return ESP_OK on success, ESP_FAIL on initialization failure
 */
esp_err_t stellaria_init(void);

/**
 * @brief Set ambient light intensity
 * @param intensity Intensity value (0-1023), will be clamped to valid range
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t stellaria_set_intensity(uint16_t intensity);

/**
 * @brief Get current intensity setting
 * @return Current intensity (0-1023), or 0 if not initialized
 */
uint16_t stellaria_get_intensity(void);

/**
 * @brief Enable ambient lighting system
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t stellaria_enable(void);

/**
 * @brief Disable ambient lighting system
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t stellaria_disable(void);

/**
 * @brief Set power saving mode (limits intensity to 20%)
 * @param enable True to enable power saving mode, false to disable
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t stellaria_set_power_save_mode(bool enable);

/**
 * @brief Set shutdown state (for load shedding)
 * @param shutdown True to shutdown, false to restore previous state
 * @return ESP_OK on success
 */
esp_err_t stellaria_set_shutdown(bool shutdown);

/**
 * @brief Get current system status (thread-safe)
 * @param[out] status Pointer to stellaria_status_t structure to fill
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t stellaria_get_status(stellaria_status_t *status);

/**
 * @brief Check if system is currently enabled
 * @return true if enabled and operational, false otherwise
 */
bool stellaria_is_enabled(void);

/**
 * @brief Enable automatic light sensing mode
 * @param intensity_when_on Intensity to use when automatically turned on (0-1023)
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t stellaria_set_auto_mode(uint16_t intensity_when_on);

/**
 * @brief Update light reading for automatic control (called by FLUCTUS)
 * @param averaged_light_voltage Averaged photoresistor voltage reading
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not in auto mode
 */
esp_err_t stellaria_update_light_reading(float averaged_light_voltage);

#endif // STELLARIA_H