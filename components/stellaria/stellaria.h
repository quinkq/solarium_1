#ifndef STELLARIA_H
#define STELLARIA_H

#include <stdbool.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// ########################## STELLARIA Configuration ##########################

// GPIO Configuration
#define STELLARIA_PWM_GPIO          GPIO_NUM_2   // PWM intensity control (external 100kΩ pull-down)

// PWM Configuration
#define STELLARIA_PWM_FREQUENCY     1000         // 1kHz PWM frequency
#define STELLARIA_PWM_RESOLUTION    LEDC_TIMER_10_BIT  // 10-bit resolution (0-1023)
#define STELLARIA_PWM_TIMER         LEDC_TIMER_1
#define STELLARIA_PWM_CHANNEL       LEDC_CHANNEL_0
#define STELLARIA_PWM_SPEED_MODE    LEDC_LOW_SPEED_MODE

// Intensity Limits (10-bit PWM: 0-1023)
#define STELLARIA_MIN_INTENSITY     51           // Minimum intensity (5% duty cycle - driver turn-on threshold)
#define STELLARIA_MAX_INTENSITY     512          // Maximum intensity (50%)
#define STELLARIA_POWER_SAVE_LIMIT  128          // 12.5% intensity for power saving mode

// Mutex timeout constants
#define STELLARIA_MUTEX_TIMEOUT_MS  100          // Mutex timeout for operations

// Intensity Ramping Configuration
#define STELLARIA_RAMP_RATE_PERCENT_PER_SEC  10.0f  // 10% per second ramping rate
#define STELLARIA_RAMP_UPDATE_INTERVAL_MS    100    // Update PWM every 100ms during ramp
#define STELLARIA_RAMP_STEP_SIZE             10     // ~10 intensity units per 100ms (10%/s for 1024 range)

// Automatic Light Sensing Configuration (uses FLUCTUS photoresistor readings)
#define STELLARIA_LIGHT_TURN_ON_THRESHOLD   0.4f  // Turn ON when light > 0.4V (dark conditions)
#define STELLARIA_LIGHT_TURN_OFF_THRESHOLD  0.3f  // Turn OFF when light < 0.3V (bright conditions)
// Note: Light readings are provided by FLUCTUS during solar tracking (every ~5s during tracking)

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
    time_t snapshot_timestamp;       // Timestamp of snapshot
} stellaria_snapshot_t;

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
 * @brief Get comprehensive STELLARIA data snapshot (single mutex operation)
 * @param[out] snapshot Pointer to stellaria_snapshot_t structure to fill
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t stellaria_get_data_snapshot(stellaria_snapshot_t *snapshot);

/**
 * @brief Check if system is currently enabled
 * @return true if enabled and operational, false otherwise
 */
bool stellaria_is_enabled(void);

/**
 * @brief Enable automatic light toggle mode using FLUCTUS photoresistor readings
 *
 * When enabled, STELLARIA will automatically turn ON/OFF based on ambient light levels
 * detected by FLUCTUS photoresistors. Uses hysteresis to prevent rapid toggling:
 * - Turns ON when voltage > 0.4V (dark conditions)
 * - Turns OFF when voltage < 0.3V (bright conditions)
 *
 * Light readings are provided by FLUCTUS during solar tracking operations (~5s intervals).
 * During nighttime when solar tracking stops, lights will remain in their last state (ON).
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 *
 * @note On initialization, uses power-save mode intensity (STELLARIA_POWER_SAVE_LIMIT).
 *       After first manual intensity change, remembers and uses that value when auto-toggling ON.
 */
esp_err_t stellaria_enable_auto_toggle(void);

/**
 * @brief Update light reading for automatic control (called by FLUCTUS)
 * @param averaged_light_voltage Averaged photoresistor voltage reading
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not in auto mode
 */
esp_err_t stellaria_update_light_reading(float averaged_light_voltage);

/**
 * @brief Request temporary dimming during irrigation for power management
 *
 * When irrigation pump/valves are active, this dims the ambient lights to minimum
 * intensity (STELLARIA_MIN_INTENSITY) to reduce load on the 12V bus. When irrigation
 * completes, intensity is restored to the user's preferred setting.
 *
 * This function is called by IMPLUVIUM component during watering operations.
 * All transitions use gradual ramping (10%/second) for smooth changes.
 *
 * @param dim True to dim to minimum intensity, false to restore previous setting
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 *
 * @note The restored intensity respects current power-save mode limits
 */
esp_err_t stellaria_request_irrigation_dim(bool dim);

#endif // STELLARIA_H