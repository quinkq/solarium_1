/**
 * @file stellaria.c
 * @brief STELLARIA - Adaptive LED lighting control
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Original implementation of STELLARIA - intelligent ambient lighting.
 *
 * Key features:
 * - Meanwell LDD-600L constant current driver control
 * - Three modes: Manual control (0-100%), Auto toggle (light reading based toggle), Power Save (12.5% clamp)
 * - Smooth 10%/second fade ramping (100ms task)
 * - Auto-dimming during irrigation to prevent algae growth
 * - Photoresistor feedback with hysteresis (0.3V/0.4V)
 * - Power management integration
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "stellaria.h"
#include "fluctus.h"
#include "telemetry.h"

#include "esp_log.h"
#include <stdio.h>
#include <string.h>


// ########################## Constants and Variables ##########################

static const char *TAG = "STELLARIA";

// Global status structure
static stellaria_snapshot_t stellaria_status = {
    .state = STELLARIA_STATE_DISABLED,
    .current_intensity = 0,
    .target_intensity = 0,
    .driver_enabled = false,
    .initialized = false,
    .auto_mode_active = false,
    .power_save_mode = false,
    .last_light_reading = -1.0f,
    .snapshot_timestamp = 0
};

// Previous state for shutdown recovery
static stellaria_state_t previous_state = STELLARIA_STATE_DISABLED;
static uint16_t previous_intensity = 0;
static bool previous_power_save_mode = false;

// Mutex for thread-safe operations
static SemaphoreHandle_t xStellariaMutex = NULL;

// Auto mode variables - remembers last used intensity
static uint16_t auto_mode_intensity = STELLARIA_POWER_SAVE_LIMIT;  // Default to power-save intensity

// Intensity ramping variables
static TaskHandle_t xStellariaRampTask = NULL;     // Ramping task handle
static uint16_t user_raw_intensity = 0;            // User's raw preference (only modified by user or power-save entry)
static uint16_t ramp_target_intensity = 0;         // Target for current ramp operation
static bool irrigation_dimming_active = false;     // Flag for irrigation dimming override

// ########################## Private Function Declarations ##########################

static esp_err_t stellaria_pwm_init(void);
static esp_err_t stellaria_apply_pwm(void);
static uint16_t stellaria_clamp_intensity(uint16_t intensity, stellaria_state_t state, bool power_save_mode);
static uint16_t stellaria_calculate_effective_target(void);
static void stellaria_start_ramp(uint16_t target);
static void stellaria_ramp_task(void *pvParameters);
static esp_err_t stellaria_on_state(void);
static esp_err_t stellaria_off_state(void);

// ########################## Private Functions ##########################

/**
 * @brief Initialize PWM for intensity control
 */
static esp_err_t stellaria_pwm_init(void)
{
    // Configure timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = STELLARIA_PWM_SPEED_MODE,
        .duty_resolution = STELLARIA_PWM_RESOLUTION,
        .timer_num = STELLARIA_PWM_TIMER,
        .freq_hz = STELLARIA_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    esp_err_t ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM timer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure channel
    ledc_channel_config_t channel_conf = {
        .gpio_num = STELLARIA_LED_PWM_GPIO,
        .speed_mode = STELLARIA_PWM_SPEED_MODE,
        .channel = STELLARIA_PWM_CHANNEL,
        .timer_sel = STELLARIA_PWM_TIMER,
        .duty = 0,  // Start with 0% duty cycle
        .hpoint = 0
    };
    
    ret = ledc_channel_config(&channel_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "PWM initialized - GPIO%d, %dHz, %d-bit (100kΩ external pull-down)",
             STELLARIA_LED_PWM_GPIO, STELLARIA_PWM_FREQUENCY,
             1 << STELLARIA_PWM_RESOLUTION);
    return ESP_OK;
}

/**
 * @brief Write current intensity to PWM hardware
 * @note Assumes current_intensity has already been clamped via stellaria_clamp_intensity()
 */
static esp_err_t stellaria_apply_pwm(void)
{
    // Write PWM duty cycle directly (intensity already clamped)
    esp_err_t ret = ledc_set_duty(STELLARIA_PWM_SPEED_MODE, STELLARIA_PWM_CHANNEL,
                                  stellaria_status.current_intensity);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM duty: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = ledc_update_duty(STELLARIA_PWM_SPEED_MODE, STELLARIA_PWM_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update PWM duty: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Clamp intensity based on current state, power save mode, and apply minimum threshold
 */
static uint16_t stellaria_clamp_intensity(uint16_t intensity, stellaria_state_t state, bool power_save_mode)
{
    // Check state first - disabled/shutdown always returns 0
    if (state == STELLARIA_STATE_DISABLED || state == STELLARIA_STATE_SHUTDOWN) {
        return 0;
    }

    // Apply power save limit if active
    if (power_save_mode && intensity > STELLARIA_POWER_SAVE_LIMIT) {
        intensity = STELLARIA_POWER_SAVE_LIMIT;
    }

    // Clamp to maximum for enabled state
    if (intensity > STELLARIA_MAX_INTENSITY) {
        intensity = STELLARIA_MAX_INTENSITY;
    }

    // Apply minimum threshold: if non-zero but below minimum, clamp to minimum
    // This ensures the LED driver receives sufficient duty cycle to turn on
    if (intensity > 0 && intensity < STELLARIA_MIN_INTENSITY) {
        intensity = STELLARIA_MIN_INTENSITY;
    }

    return intensity;
}

/**
 * @brief Calculate effective target intensity based on current state and overrides
 * @note Must be called with mutex already held
 */
static uint16_t stellaria_calculate_effective_target(void)
{
    uint16_t target = user_raw_intensity;

    // Apply state-specific clamping with power save mode
    target = stellaria_clamp_intensity(target, stellaria_status.state, stellaria_status.power_save_mode);

    // Irrigation dimming overrides everything (if lights would be on)
    if (irrigation_dimming_active && target > 0) {
        target = STELLARIA_MIN_INTENSITY;
    }

    return target;
}

/**
 * @brief Gradual intensity ramping task (suspended when not ramping)
 */
static void stellaria_ramp_task(void *pvParameters)
{
    while (1) {
        xSemaphoreTake(xStellariaMutex, portMAX_DELAY);
        telemetry_fetch_snapshot(TELEMETRY_SRC_STELLARIA);
        xSemaphoreGive(xStellariaMutex);
        // Suspend immediately - will be resumed when ramping needed
        vTaskSuspend(NULL);

        // Ramping loop - runs until target reached
        while (1) {
            if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                uint16_t current = stellaria_status.current_intensity;
                uint16_t target = ramp_target_intensity;

                // Check if we've reached the target
                if (current == target) {
                    xSemaphoreGive(xStellariaMutex);
                    break;  // Exit loop and suspend
                }

                // Calculate next step
                int16_t difference = (int16_t)target - (int16_t)current;
                int16_t step = (difference > 0) ? STELLARIA_RAMP_STEP_SIZE : -STELLARIA_RAMP_STEP_SIZE;

                // Don't overshoot target
                if (abs(difference) < STELLARIA_RAMP_STEP_SIZE) {
                    stellaria_status.current_intensity = target;
                } else {
                    stellaria_status.current_intensity = current + step;
                }

                // Apply to hardware
                stellaria_apply_pwm();

                // Give mutex BEFORE delay
                xSemaphoreGive(xStellariaMutex);

            } else {
                // Couldn't get mutex, exit ramp loop
                ESP_LOGW(TAG, "Ramp task failed to get mutex, suspending.");
                break;
            }

            // Wait before next step (OUTSIDE MUTEX)
            vTaskDelay(pdMS_TO_TICKS(STELLARIA_RAMP_UPDATE_INTERVAL_MS));
        }
    }
}

/**
 * @brief Start ramping to a new target intensity
 * @note Must be called with mutex already held. Returns with mutex held.
 */
static void stellaria_start_ramp(uint16_t target)
{
    ramp_target_intensity = target;
    stellaria_status.target_intensity = target;

    // Resume task if suspended (will do nothing if already running)
    if (xStellariaRampTask != NULL && eTaskGetState(xStellariaRampTask) == eSuspended) {
        vTaskResume(xStellariaRampTask);
    }
}

/**
 * @brief Turn lights ON - request bus power and start ramping to intensity
 * @note Must be called with mutex already held
 * @return ESP_OK on success, ESP_FAIL if bus power request fails
 */
static esp_err_t stellaria_on_state(void)
{
    // Request 12V bus power for LED drivers
    esp_err_t ret = fluctus_request_bus_power(POWER_BUS_12V, "STELLARIA");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to request 12V bus power: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    stellaria_status.driver_enabled = true;

    // Calculate effective target and start ramping (unless irrigation dimming active)
    uint16_t effective_target = stellaria_calculate_effective_target();
    if (!irrigation_dimming_active) {
        stellaria_start_ramp(effective_target);
    }

    ESP_LOGI(TAG, "Lights ON (target: %d)", effective_target);
    return ESP_OK;
}

/**
 * @brief Turn lights OFF - release bus power and ramp to zero
 * @note Must be called with mutex already held
 * @return ESP_OK on success
 */
static esp_err_t stellaria_off_state(void)
{
    stellaria_status.driver_enabled = false;

    // Ramp to 0
    stellaria_start_ramp(0);

    // Release 12V bus power
    fluctus_release_bus_power(POWER_BUS_12V, "STELLARIA");

    ESP_LOGI(TAG, "Lights OFF, 12V bus released");
    return ESP_OK;
}

// ########################## Public Functions ##########################

esp_err_t stellaria_init(void)
{
    ESP_LOGI(TAG, "Initializing STELLARIA ambient lighting system...");

    // Create mutex
    xStellariaMutex = xSemaphoreCreateMutex();
    if (xStellariaMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    // Initialize PWM
    esp_err_t ret = stellaria_pwm_init();
    if (ret != ESP_OK) {
        vSemaphoreDelete(xStellariaMutex);
        xStellariaMutex = NULL;
        return ret;
    }

    // Create ramping task (suspended initially)
    BaseType_t task_ret = xTaskCreate(
        stellaria_ramp_task,
        "stellaria_ramp",
        configMINIMAL_STACK_SIZE * 2,
        NULL,
        5,  // Priority
        &xStellariaRampTask
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ramping task");
        vSemaphoreDelete(xStellariaMutex);
        xStellariaMutex = NULL;
        return ESP_FAIL;
    }

    // Set initial status
    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        stellaria_status.state = STELLARIA_STATE_DISABLED;
        stellaria_status.current_intensity = 0;
        stellaria_status.target_intensity = 0;
        stellaria_status.driver_enabled = false;
        stellaria_status.initialized = true;
        user_raw_intensity = 0;
        ramp_target_intensity = 0;

        // Set PWM to 0 (already clamped to 0 by initialization)
        stellaria_apply_pwm();

        xSemaphoreGive(xStellariaMutex);
    } else {
        ESP_LOGE(TAG, "Failed to acquire mutex during initialization");
        vTaskDelete(xStellariaRampTask);
        vSemaphoreDelete(xStellariaMutex);
        xStellariaMutex = NULL;
        xStellariaRampTask = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "STELLARIA initialization complete (ramping task created)");
    return ESP_OK;
}

esp_err_t stellaria_set_intensity(uint16_t intensity)
{
    if (!stellaria_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        // Store RAW user preference (no clamping - will be applied when calculating effective target)
        user_raw_intensity = intensity;

        // If in auto mode and non-zero, remember this as auto_mode_intensity for next auto ON
        if (stellaria_status.auto_mode_active && intensity > 0) {
            auto_mode_intensity = intensity;
        }

        // Calculate effective target (applies state-specific clamping and irrigation override)
        uint16_t effective_target = stellaria_calculate_effective_target();

        // Start ramping (unless irrigation dimming blocks it)
        if (!irrigation_dimming_active) {
            stellaria_start_ramp(effective_target);
            ESP_LOGI(TAG, "User set intensity %d → ramping to effective target %d (from %d)",
                     intensity, effective_target, stellaria_status.current_intensity);
        } else {
            // During irrigation, just update the preference (will restore after irrigation)
            ESP_LOGI(TAG, "User set intensity %d (dimmed for irrigation, will apply after)", intensity);
        }

        xSemaphoreGive(xStellariaMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t stellaria_enable(void)
{
    if (!stellaria_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        // Idempotent: if already enabled, just return success
        if (stellaria_status.state == STELLARIA_STATE_ENABLED) {
            xSemaphoreGive(xStellariaMutex);
            ESP_LOGD(TAG, "STELLARIA already enabled, ignoring redundant call");
            return ESP_OK;
        }

        esp_err_t ret = ESP_OK;
        stellaria_status.state = STELLARIA_STATE_ENABLED;

        // Check if auto mode is active
        if (stellaria_status.auto_mode_active) {
            // Auto mode decides based on current light levels
            // Check current light reading to decide initial state
            if (stellaria_status.last_light_reading > STELLARIA_LIGHT_TURN_ON_THRESHOLD) {
                // Dark - turn lights ON
                ret = stellaria_on_state();
                ESP_LOGI(TAG, "STELLARIA enabled in AUTO mode - lights ON (dark: %.3fV)",
                         stellaria_status.last_light_reading);
            } else {
                // Bright - keep lights OFF
                ESP_LOGI(TAG, "STELLARIA enabled in AUTO mode - lights OFF (bright: %.3fV)",
                         stellaria_status.last_light_reading);
            }
        } else {
            // Manual mode - turn lights ON
            ret = stellaria_on_state();
            ESP_LOGI(TAG, "STELLARIA enabled in MANUAL mode - lights ON");
        }

        xSemaphoreGive(xStellariaMutex);
        return ret;
    }

    return ESP_FAIL;
}

esp_err_t stellaria_disable(void)
{
    if (!stellaria_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        // Turn lights OFF and release bus power
        esp_err_t ret = stellaria_off_state();

        // Set state to DISABLED but preserve auto_mode_active flag
        stellaria_status.state = STELLARIA_STATE_DISABLED;

        xSemaphoreGive(xStellariaMutex);

        ESP_LOGI(TAG, "STELLARIA disabled (auto mode config preserved: %s)",
                 stellaria_status.auto_mode_active ? "ON" : "OFF");
        return ret;
    }

    return ESP_FAIL;
}

/**
 * @brief Get current STELLARIA operational state (lightweight, no snapshot fetch)
 * @return Current stellaria_state_t value
 * @note Thread-safe, uses quick mutex with 100ms timeout. Returns DISABLED on mutex timeout.
 */
stellaria_state_t stellaria_get_state(void)
{
    stellaria_state_t state = STELLARIA_STATE_DISABLED;

    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        state = stellaria_status.state;
        xSemaphoreGive(xStellariaMutex);
    }

    return state;
}

esp_err_t stellaria_set_power_save_mode(bool enable)
{
    if (!stellaria_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        if (enable && !stellaria_status.power_save_mode) {
            // Enable power save mode
            stellaria_status.power_save_mode = true;

            // PERMANENTLY clamp user preference if above limit (one-way operation!)
            if (user_raw_intensity > STELLARIA_POWER_SAVE_LIMIT) {
                user_raw_intensity = STELLARIA_POWER_SAVE_LIMIT;
                ESP_LOGI(TAG, "Power-save mode: user preference PERMANENTLY clamped to %d", STELLARIA_POWER_SAVE_LIMIT);
            }

            // Recalculate effective target and ramp (unless irrigation dimming active)
            uint16_t effective_target = stellaria_calculate_effective_target();
            if (!irrigation_dimming_active && stellaria_status.state == STELLARIA_STATE_ENABLED) {
                stellaria_start_ramp(effective_target);
            }

            ESP_LOGI(TAG, "Power save mode enabled (intensity limited to %d)", STELLARIA_POWER_SAVE_LIMIT);

        } else if (!enable && stellaria_status.power_save_mode) {
            // Disable power save mode
            stellaria_status.power_save_mode = false;

            // user_raw_intensity stays at its current value (no restoration)
            // Just ensure it doesn't exceed maximum
            if (user_raw_intensity > STELLARIA_MAX_INTENSITY) {
                user_raw_intensity = STELLARIA_MAX_INTENSITY;
            }

            // Recalculate effective target and ramp (unless irrigation dimming active)
            uint16_t effective_target = stellaria_calculate_effective_target();
            if (!irrigation_dimming_active && stellaria_status.state == STELLARIA_STATE_ENABLED) {
                stellaria_start_ramp(effective_target);
            }

            ESP_LOGI(TAG, "Power save mode disabled (user preference: %d)", user_raw_intensity);
        }

        xSemaphoreGive(xStellariaMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t stellaria_set_shutdown(bool shutdown)
{
    if (!stellaria_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        esp_err_t ret = ESP_OK;

        if (shutdown) {
            // Save current state, power save mode, and user preference for recovery
            previous_state = stellaria_status.state;
            previous_intensity = user_raw_intensity;  // Save user preference, not target
            previous_power_save_mode = stellaria_status.power_save_mode;

            // Set shutdown state
            stellaria_status.state = STELLARIA_STATE_SHUTDOWN;

            // Turn off using proper state handler (ramps down + releases power)
            ret = stellaria_off_state();

            ESP_LOGI(TAG, "STELLARIA shutdown for load shedding (saved state: %d, intensity: %d, power_save: %d)",
                     previous_state, previous_intensity, previous_power_save_mode);
        } else {
            // Restore previous state and flags
            stellaria_status.state = previous_state;
            user_raw_intensity = previous_intensity;  // Restore user preference
            stellaria_status.power_save_mode = previous_power_save_mode;

            // If the restored state requires lights to be on, use proper state handler
            if (previous_state == STELLARIA_STATE_ENABLED) {
                // Turn on using proper state handler (requests power + smooth ramp)
                ret = stellaria_on_state();

                ESP_LOGI(TAG, "STELLARIA restored from shutdown (state: ENABLED, ramping to: %d, power_save: %d)",
                         stellaria_status.target_intensity, stellaria_status.power_save_mode);
            } else {
                // Previous state was DISABLED - just restore state, no action needed
                ESP_LOGI(TAG, "STELLARIA restored from shutdown to DISABLED state");
            }
        }

        xSemaphoreGive(xStellariaMutex);
        return ret;
    }

    return ESP_FAIL;
}

/**
 * @brief Write STELLARIA data directly to TELEMETRY cache
 *
 * Comprehensive snapshot for HMI/MQTT distribution with all fields.
 * Writes directly to TELEMETRY's cache buffer to eliminate intermediate copy.
 * Only STELLARIA mutex needed - TELEMETRY lock/unlock handled by caller.
 *
 * @param cache Pointer to TELEMETRY's stellaria_snapshot_t buffer
 * @return ESP_OK on success, ESP_FAIL if mutex timeout
 */
esp_err_t stellaria_write_to_telemetry_cache(stellaria_snapshot_t *cache)
{
    if (cache == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!stellaria_status.initialized) {
        memset(cache, 0, sizeof(stellaria_snapshot_t));
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        // Single efficient copy of entire structure to TELEMETRY cache
        memcpy(cache, &stellaria_status, sizeof(stellaria_snapshot_t));
        // Note: snapshot_timestamp set by TELEMETRY unlock function
        xSemaphoreGive(xStellariaMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief Enable or disable automatic light mode using FLUCTUS photoresistor readings
 */
esp_err_t stellaria_set_auto_mode(bool enable)
{
    if (!stellaria_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        esp_err_t ret = ESP_OK;

        if (enable) {
            // Enable auto mode
            stellaria_status.auto_mode_active = true;

            // Only affect lights if stellaria is currently enabled
            if (stellaria_status.state == STELLARIA_STATE_ENABLED) {
                // Check current light reading to decide state
                if (stellaria_status.last_light_reading > STELLARIA_LIGHT_TURN_ON_THRESHOLD) {
                    // Dark - ensure lights are ON
                    if (!stellaria_status.driver_enabled) {
                        ret = stellaria_on_state();
                    }
                    ESP_LOGI(TAG, "Auto mode enabled - lights ON (dark: %.3fV)",
                             stellaria_status.last_light_reading);
                } else {
                    // Bright - turn lights OFF
                    if (stellaria_status.driver_enabled) {
                        ret = stellaria_off_state();
                    }
                    ESP_LOGI(TAG, "Auto mode enabled - lights OFF (bright: %.3fV)",
                             stellaria_status.last_light_reading);
                }
            } else {
                ESP_LOGI(TAG, "Auto mode enabled (will apply when STELLARIA is enabled)");
            }
        } else {
            // Disable auto mode - return to manual control
            stellaria_status.auto_mode_active = false;

            // If currently enabled, ensure lights are in a consistent state
            if (stellaria_status.state == STELLARIA_STATE_ENABLED) {
                if (stellaria_status.driver_enabled) {
                    ESP_LOGI(TAG, "Auto mode disabled - remaining in ENABLED mode (lights ON)");
                } else {
                    // Lights are OFF - transition to DISABLED
                    stellaria_status.state = STELLARIA_STATE_DISABLED;
                    ESP_LOGI(TAG, "Auto mode disabled - transitioning to DISABLED mode (lights OFF)");
                }
            } else {
                ESP_LOGI(TAG, "Auto mode disabled (state unchanged)");
            }
        }

        xSemaphoreGive(xStellariaMutex);
        return ret;
    }

    return ESP_FAIL;
}

/**
 * @brief Update light reading for automatic control (called by FLUCTUS)
 * @param averaged_light_voltage Averaged photoresistor voltage reading
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not in auto mode or not enabled
 */
esp_err_t stellaria_update_light_intensity(float averaged_light_voltage)
{
    if (!stellaria_status.initialized || !stellaria_status.auto_mode_active) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        stellaria_status.last_light_reading = averaged_light_voltage;

        // Only control lights if stellaria is enabled
        if (stellaria_status.state != STELLARIA_STATE_ENABLED) {
            xSemaphoreGive(xStellariaMutex);
            return ESP_OK;  // Auto mode configured but stellaria disabled - do nothing
        }

        bool currently_on = stellaria_status.driver_enabled;
        bool should_be_on = false;

        // Hysteresis logic: low voltage = bright light, high voltage = dark
        if (currently_on) {
            // Currently on - turn OFF only if voltage < 0.3V (bright)
            should_be_on = (averaged_light_voltage >= STELLARIA_LIGHT_TURN_OFF_THRESHOLD);
        } else {
            // Currently off - turn ON only if voltage > 0.4V (dark)
            should_be_on = (averaged_light_voltage > STELLARIA_LIGHT_TURN_ON_THRESHOLD);
        }

        esp_err_t ret = ESP_OK;

        // Apply changes if state should change
        if (should_be_on && !currently_on) {
            ESP_LOGI(TAG, "Auto mode: Turning ON (%.3fV > %.3fV - dark)",
                     averaged_light_voltage, STELLARIA_LIGHT_TURN_ON_THRESHOLD);
            ret = stellaria_on_state();

        } else if (!should_be_on && currently_on) {
            ESP_LOGI(TAG, "Auto mode: Turning OFF (%.3fV < %.3fV - bright)",
                     averaged_light_voltage, STELLARIA_LIGHT_TURN_OFF_THRESHOLD);
            ret = stellaria_off_state();
        }

        xSemaphoreGive(xStellariaMutex);
        return ret;
    }

    return ESP_FAIL;
}


/**
 * @brief Request temporary dimming during irrigation for power management
 */
esp_err_t stellaria_request_irrigation_dim(bool dim)
{
    if (!stellaria_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        if (dim && !irrigation_dimming_active) {
            // Start dimming: activate flag and ramp to minimum
            irrigation_dimming_active = true;

            stellaria_start_ramp(STELLARIA_MIN_INTENSITY);
            ESP_LOGI(TAG, "Irrigation dimming: ramping to MIN (%d)", STELLARIA_MIN_INTENSITY);

        } else if (!dim && irrigation_dimming_active) {
            // End dimming: deactivate flag and restore appropriate target
            irrigation_dimming_active = false;

            // In auto mode, check light conditions to decide target
            if (stellaria_status.auto_mode_active && stellaria_status.state == STELLARIA_STATE_ENABLED) {
                // Use last light reading to determine if auto mode wants lights ON or OFF
                bool should_be_on = (stellaria_status.last_light_reading > STELLARIA_LIGHT_TURN_ON_THRESHOLD);
                if (should_be_on) {
                    // Lights should be ON - ensure bus power and ramp
                    if (!stellaria_status.driver_enabled) {
                        stellaria_on_state();
                    } else {
                        uint16_t restore_target = stellaria_clamp_intensity(auto_mode_intensity, stellaria_status.state, stellaria_status.power_save_mode);
                        stellaria_start_ramp(restore_target);
                    }
                    ESP_LOGI(TAG, "Irrigation complete: restoring auto mode ON");
                } else {
                    // Lights should be OFF - release bus power
                    if (stellaria_status.driver_enabled) {
                        stellaria_off_state();
                    }
                    ESP_LOGI(TAG, "Irrigation complete: restoring auto mode OFF");
                }
            } else {
                // Manual mode: restore using user preference
                uint16_t restore_target = stellaria_calculate_effective_target();
                stellaria_start_ramp(restore_target);
                ESP_LOGI(TAG, "Irrigation complete: restoring user preference to %d", restore_target);
            }
        }

        xSemaphoreGive(xStellariaMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}
