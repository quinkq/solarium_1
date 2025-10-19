#include <stdio.h>
#include <string.h>

#include "stellaria.h"
#include "fluctus.h"
#include "telemetry.h"
#include "esp_log.h"


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
    .last_light_reading = -1.0f,
    .snapshot_timestamp = 0
};

// Previous state for shutdown recovery
static stellaria_state_t previous_state = STELLARIA_STATE_DISABLED;
static uint16_t previous_intensity = 0;

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
static uint16_t stellaria_clamp_intensity(uint16_t intensity, stellaria_state_t state);
static uint16_t stellaria_calculate_effective_target(void);
static void stellaria_start_ramp(uint16_t target);
static void stellaria_ramp_task(void *pvParameters);

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
        .gpio_num = STELLARIA_PWM_GPIO,
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
             STELLARIA_PWM_GPIO, STELLARIA_PWM_FREQUENCY,
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
 * @brief Clamp intensity based on current state and apply minimum threshold
 */
static uint16_t stellaria_clamp_intensity(uint16_t intensity, stellaria_state_t state)
{
    // Apply state-specific limits first
    switch (state) {
        case STELLARIA_STATE_DISABLED:
        case STELLARIA_STATE_SHUTDOWN:
            return 0;

        case STELLARIA_STATE_POWER_SAVE:
            if (intensity > STELLARIA_POWER_SAVE_LIMIT) {
                intensity = STELLARIA_POWER_SAVE_LIMIT;
            }
            break;

        case STELLARIA_STATE_ENABLED:
        case STELLARIA_STATE_AUTO:
        default:
            // Clamp to maximum for enabled/auto states
            if (intensity > STELLARIA_MAX_INTENSITY) {
                intensity = STELLARIA_MAX_INTENSITY;
            }
            break;
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

    // Apply state-specific clamping
    target = stellaria_clamp_intensity(target, stellaria_status.state);

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

                // Inject to TELEMETRY (state changed - notify central hub)
                // Note: We're inside mutex, telemetry_update_stellaria() will take its own mutex
                xSemaphoreGive(xStellariaMutex);

                // Update TELEMETRY cache with new state
                telemetry_update_stellaria();

                // Re-acquire mutex for next iteration
                if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) != pdTRUE) {
                    break;  // Couldn't get mutex, exit ramp loop
                }
                xSemaphoreGive(xStellariaMutex);

            } else {
                break;  // Couldn't get mutex, exit ramp loop
            }

            // Wait before next step
            vTaskDelay(pdMS_TO_TICKS(STELLARIA_RAMP_UPDATE_INTERVAL_MS));
        }
    }
}

/**
 * @brief Start ramping to a new target intensity
 * @note Must be called with mutex already held
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

uint16_t stellaria_get_intensity(void)
{
    if (!stellaria_status.initialized) {
        return 0;
    }
    
    uint16_t intensity = 0;
    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        intensity = stellaria_status.target_intensity;
        xSemaphoreGive(xStellariaMutex);
    }
    
    return intensity;
}

esp_err_t stellaria_enable(void)
{
    if (!stellaria_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        // Request 12V bus power for LED drivers
        FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_12V, "STELLARIA", {
            xSemaphoreGive(xStellariaMutex);
            return ESP_FAIL;
        });
        
        stellaria_status.state = STELLARIA_STATE_ENABLED;
        stellaria_status.driver_enabled = true;
        stellaria_status.current_intensity = stellaria_clamp_intensity(
            stellaria_status.target_intensity, stellaria_status.state);

        esp_err_t ret = stellaria_apply_pwm();

        xSemaphoreGive(xStellariaMutex);

        ESP_LOGI(TAG, "STELLARIA enabled with 12V bus power");
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
        stellaria_status.state = STELLARIA_STATE_DISABLED;
        stellaria_status.driver_enabled = false;
        stellaria_status.current_intensity = 0;

        esp_err_t ret = stellaria_apply_pwm();

        // Release 12V bus power when disabled
        fluctus_release_bus_power(POWER_BUS_12V, "STELLARIA");

        xSemaphoreGive(xStellariaMutex);

        ESP_LOGI(TAG, "STELLARIA disabled, 12V bus released");
        return ret;
    }
    
    return ESP_FAIL;
}

esp_err_t stellaria_set_power_save_mode(bool enable)
{
    if (!stellaria_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        if (enable) {
            if (stellaria_status.state == STELLARIA_STATE_ENABLED) {
                stellaria_status.state = STELLARIA_STATE_POWER_SAVE;

                // PERMANENTLY clamp user preference if above limit (one-way operation!)
                if (user_raw_intensity > STELLARIA_POWER_SAVE_LIMIT) {
                    user_raw_intensity = STELLARIA_POWER_SAVE_LIMIT;
                    ESP_LOGI(TAG, "Power-save mode: user preference PERMANENTLY clamped to %d", STELLARIA_POWER_SAVE_LIMIT);
                }

                // Recalculate effective target and ramp (unless irrigation dimming active)
                uint16_t effective_target = stellaria_calculate_effective_target();
                if (!irrigation_dimming_active) {
                    stellaria_start_ramp(effective_target);
                }

                ESP_LOGI(TAG, "Power save mode enabled (intensity limited to %d)", STELLARIA_POWER_SAVE_LIMIT);
            }
        } else {
            if (stellaria_status.state == STELLARIA_STATE_POWER_SAVE) {
                stellaria_status.state = STELLARIA_STATE_ENABLED;

                // user_raw_intensity stays at its current value (no restoration)
                // Just ensure it doesn't exceed maximum for ENABLED state
                if (user_raw_intensity > STELLARIA_MAX_INTENSITY) {
                    user_raw_intensity = STELLARIA_MAX_INTENSITY;
                }

                // Recalculate effective target with new state and ramp (unless irrigation dimming active)
                uint16_t effective_target = stellaria_calculate_effective_target();
                if (!irrigation_dimming_active) {
                    stellaria_start_ramp(effective_target);
                }

                ESP_LOGI(TAG, "Power save mode disabled (user preference: %d)", user_raw_intensity);
            }
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
        if (shutdown) {
            // Save current state for recovery
            previous_state = stellaria_status.state;
            previous_intensity = stellaria_status.target_intensity;
            
            // Set shutdown state
            stellaria_status.state = STELLARIA_STATE_SHUTDOWN;
            stellaria_status.driver_enabled = false;
            stellaria_status.current_intensity = 0;
            
            ESP_LOGI(TAG, "STELLARIA shutdown for load shedding");
        } else {
            // Restore previous state
            stellaria_status.state = previous_state;
            stellaria_status.target_intensity = previous_intensity;
            stellaria_status.driver_enabled = (previous_state != STELLARIA_STATE_DISABLED);
            stellaria_status.current_intensity = stellaria_clamp_intensity(
                stellaria_status.target_intensity, stellaria_status.state);
            
            ESP_LOGI(TAG, "STELLARIA restored from shutdown");
        }

        esp_err_t ret = stellaria_apply_pwm();

        xSemaphoreGive(xStellariaMutex);
        return ret;
    }
    
    return ESP_FAIL;
}

esp_err_t stellaria_get_data_snapshot(stellaria_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!stellaria_status.initialized) {
        memset(snapshot, 0, sizeof(stellaria_snapshot_t));
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        memcpy(snapshot, &stellaria_status, sizeof(stellaria_snapshot_t));
        snapshot->snapshot_timestamp = time(NULL);
        xSemaphoreGive(xStellariaMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

bool stellaria_is_enabled(void)
{
    if (!stellaria_status.initialized) {
        return false;
    }
    
    bool enabled = false;
    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        enabled = (stellaria_status.state == STELLARIA_STATE_ENABLED || 
                  stellaria_status.state == STELLARIA_STATE_POWER_SAVE) &&
                  stellaria_status.driver_enabled;
        xSemaphoreGive(xStellariaMutex);
    }
    
    return enabled;
}

/**
 * @brief Enable automatic light toggle mode using FLUCTUS photoresistor readings
 */
esp_err_t stellaria_enable_auto_toggle(void)
{
    if (!stellaria_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        stellaria_status.state = STELLARIA_STATE_AUTO;
        stellaria_status.auto_mode_active = true;

        ESP_LOGI(TAG, "Auto light toggle mode enabled (will use intensity: %d when turning ON)",
                 auto_mode_intensity);

        xSemaphoreGive(xStellariaMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief Update light reading for automatic control (called by FLUCTUS)
 * @param averaged_light_voltage Averaged photoresistor voltage reading
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not in auto mode
 */
esp_err_t stellaria_update_light_reading(float averaged_light_voltage)
{
    if (!stellaria_status.initialized || !stellaria_status.auto_mode_active) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        stellaria_status.last_light_reading = averaged_light_voltage;

        bool currently_on = (stellaria_status.current_intensity > 0);
        bool should_be_on = false;

        // Hysteresis logic: low voltage = bright light, high voltage = dark
        if (currently_on) {
            // Currently on - turn OFF only if voltage < 0.3V (bright)
            should_be_on = (averaged_light_voltage >= STELLARIA_LIGHT_TURN_OFF_THRESHOLD);
        } else {
            // Currently off - turn ON only if voltage > 0.4V (dark)
            should_be_on = (averaged_light_voltage > STELLARIA_LIGHT_TURN_ON_THRESHOLD);
        }

        // Apply changes if state should change
        if (should_be_on && !currently_on) {
            ESP_LOGI(TAG, "Auto mode: Turning ON (%.3fV > %.3fV - dark)",
                     averaged_light_voltage, STELLARIA_LIGHT_TURN_ON_THRESHOLD);

            // Use auto_mode_intensity, NOT user_raw_intensity
            // Apply state-specific clamping (respects power-save limits)
            uint16_t target = stellaria_clamp_intensity(auto_mode_intensity, stellaria_status.state);

            // Apply irrigation dimming if active
            if (irrigation_dimming_active && target > 0) {
                target = STELLARIA_MIN_INTENSITY;
            }

            stellaria_start_ramp(target);

            // DO NOT MODIFY user_raw_intensity - it belongs to the user!

        } else if (!should_be_on && currently_on) {
            ESP_LOGI(TAG, "Auto mode: Turning OFF (%.3fV < %.3fV - bright)",
                     averaged_light_voltage, STELLARIA_LIGHT_TURN_OFF_THRESHOLD);

            // Ramp to 0
            stellaria_start_ramp(0);

            // DO NOT MODIFY user_raw_intensity - it belongs to the user!
        }

        xSemaphoreGive(xStellariaMutex);
        return ESP_OK;
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

            uint16_t restore_target;

            // In auto mode, check light conditions to decide target
            if (stellaria_status.auto_mode_active) {
                // Use last light reading to determine if auto mode wants lights ON or OFF
                bool should_be_on = (stellaria_status.last_light_reading > STELLARIA_LIGHT_TURN_ON_THRESHOLD);
                if (should_be_on) {
                    restore_target = stellaria_clamp_intensity(auto_mode_intensity, stellaria_status.state);
                    ESP_LOGI(TAG, "Irrigation complete: restoring auto mode ON to %d", restore_target);
                } else {
                    restore_target = 0;
                    ESP_LOGI(TAG, "Irrigation complete: restoring auto mode OFF");
                }
            } else {
                // Manual mode: restore using user preference
                restore_target = stellaria_calculate_effective_target();
                ESP_LOGI(TAG, "Irrigation complete: restoring user preference to %d", restore_target);
            }

            stellaria_start_ramp(restore_target);
        }

        xSemaphoreGive(xStellariaMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}
