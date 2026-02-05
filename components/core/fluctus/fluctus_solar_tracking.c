/**
 * @file fluctus_solar_tracking.c
 * @brief FLUCTUS solar tracking module
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Manages solar panel tracking system with dual-axis servo control:
 * - 4× photoresistor quadrant sensing (TL/TR/BL/BR via ADS1115-2)
 * - Differential error calculation (left-right, top-bottom)
 * - Proportional servo correction with safety limits
 * - 6-state state machine (DISABLED/SLEEPING/STANDBY/CORRECTING/PARKING/ERROR)
 * - 6.6V bus power gating (power-up only during correction cycles)
 * - Night parking (East 10%/60% for morning sun capture)
 * - Error parking (center 50%/50% for safe recovery)
 * - Sunrise auto-resume from SLEEPING state
 * - Notification-based task control (ENABLE/DISABLE/SUNRISE)
 *
 * STATE MACHINE:
 * - DISABLED: Manual enable required (user action or system recovery)
 * - SLEEPING: Nighttime auto-park, wake on sunrise (buffered -30min)
 * - STANDBY: 15-min intervals between correction cycles (configurable)
 * - CORRECTING: 5s active tracking with servo adjustments (5s max duration)
 * - PARKING: Transient state - parks servos and chooses next state
 * - ERROR: Timeout or sensor failure recovery (progressive backoff)
 *
 * POWER MANAGEMENT:
 * - 6.6V bus requested only during CORRECTING state (~5s cycles)
 * - Servo power-up delay: 200ms settling time
 * - Released immediately after correction complete or on sunset/error
 *
 * OPTIMIZATION:
 * - Cached photoresistor data shared with power metering (avoids redundant ADC reads)
 * - Average light intensity forwarded to STELLARIA for auto mode
 * - Single source of truth: solar_data (tracking state, servo positions, sensor readings)
 *
 * THREAD SAFETY:
 * - Protected by xSolarMutex (100ms timeout)
 * - All solar/servo data stored in solar_data (protected by xSolarMutex)
 * - Runs in solar tracking task (Med-5 priority)
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "fluctus.h"
#include "fluctus_private.h"
#include "ads1115_helper.h"
#include "stellaria.h"
#include "solar_calc.h"
#include "telemetry.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

static const char *TAG = "FLUCTUS_SOLAR";

// ########################## Module State ##########################
// Note: NO 'static' keyword - these are accessed via extern in fluctus_private.h

SemaphoreHandle_t xSolarMutex = NULL;  // Solar tracking mutex (initialized in fluctus.c)

// Solar state - SOURCE OF TRUTH for all solar/servo data
// Shared with telemetry and power metering, protected by xSolarMutex
fluctus_solar_data_t solar_data = {
    .tracking_state = SOLAR_TRACKING_DISABLED,
    .current_yaw_duty = FLUCTUS_SERVO_CENTER_DUTY,
    .current_pitch_duty = FLUCTUS_SERVO_CENTER_DUTY,
    .yaw_position_percent = 50,  // Center position
    .pitch_position_percent = 50,
    .yaw_error = 0.0f,
    .pitch_error = 0.0f,
    .photoresistor_readings = {0.0f, 0.0f, 0.0f, 0.0f},
    .valid = false,
    .timestamp = 0
};

// Correction cycle timing
int64_t last_correction_time = 0;      // Last correction cycle start time (for interval timing)
int64_t correction_start_time = 0;     // Current correction cycle start time (for timeout check)

// Parking state tracking
parking_reason_t current_parking_reason = PARKING_REASON_SUNSET;

// Error tracking for progressive backoff
uint8_t consecutive_tracking_errors = 0;
#define FLUCTUS_MAX_CONSECUTIVE_ERRORS 5  // Disable tracking after 5 consecutive failures

// Debug mode state (for continuous testing without daytime checks)
static bool solar_debug_mode_active = false;
static int64_t debug_mode_start_time = 0;
#define FLUCTUS_DEBUG_MODE_TIMEOUT_MS 90000  // 90 seconds auto-timeout

// Servo control task state (for smooth tracking refactor)
static uint8_t settling_counter = 0;           // Consecutive readings below threshold
static uint16_t total_correction_iterations = 0;  // Total iterations in current correction cycle
static bool servo_control_active = false;      // Task active flag (for debug logging)

// ########################## Servo PWM Initialization ##########################


/**
 * @brief Initialize servo PWM for solar tracking
 *
 * Configures LEDC timer and channels for dual-axis servo control:
 * - Timer: 50Hz PWM frequency (standard servo protocol)
 * - Yaw channel: GPIO41 (LEDC channel 0)
 * - Pitch channel: GPIO42 (LEDC channel 1)
 * - Initial position: Center (50% duty cycle)
 *
 * @return ESP_OK on success, ESP_FAIL if configuration fails
 */
esp_err_t fluctus_servo_pwm_init(void)
{
    ESP_LOGI(TAG, "Initializing solar tracking servo PWM...");
    
    // Configure timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = FLUCTUS_SERVO_PWM_SPEED_MODE,
        .duty_resolution = FLUCTUS_SERVO_PWM_RESOLUTION,
        .timer_num = FLUCTUS_SERVO_PWM_TIMER,
        .freq_hz = FLUCTUS_SERVO_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    esp_err_t ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure servo PWM timer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure yaw channel
    ledc_channel_config_t yaw_channel_conf = {
        .gpio_num = FLUCTUS_SERVO_PWM_YAW_GPIO,
        .speed_mode = FLUCTUS_SERVO_PWM_SPEED_MODE,
        .channel = FLUCTUS_SERVO_YAW_CHANNEL,
        .timer_sel = FLUCTUS_SERVO_PWM_TIMER,
        .duty = FLUCTUS_SERVO_CENTER_DUTY,
        .hpoint = 0
    };
    
    ret = ledc_channel_config(&yaw_channel_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure yaw servo channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure pitch channel  
    ledc_channel_config_t pitch_channel_conf = {
        .gpio_num = FLUCTUS_SERVO_PWM_PITCH_GPIO,
        .speed_mode = FLUCTUS_SERVO_PWM_SPEED_MODE,
        .channel = FLUCTUS_SERVO_PITCH_CHANNEL,
        .timer_sel = FLUCTUS_SERVO_PWM_TIMER,
        .duty = FLUCTUS_SERVO_CENTER_DUTY,
        .hpoint = 0
    };
    
    ret = ledc_channel_config(&pitch_channel_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure pitch servo channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Solar tracking servo PWM initialized");
    return ESP_OK;
}

// ########################## Photoresistor Reading ##########################

/**
 * @brief Read photoresistor values for solar tracking
 *
 * Optimized single-sample reading with 1s settling delay for ADS1115 at 128 SPS.
 *
 * Populates these fields in the output structure:
 *   - photoresistor_readings[4]
 *   - yaw_error, pitch_error
 *   - valid, timestamp
 *
 * Other fields (tracking_state, servo positions) are NOT modified.
 */
esp_err_t fluctus_read_photoresistors(fluctus_solar_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // ADS1115 @ 128 SPS: ~7.8ms conversion time per channel (hardware-determined)
    // Total time: ~44ms for 4 channels (11ms each including I2C overhead)
    ESP_LOGD(TAG, "Reading photoresistors (4 channels @ 128 SPS, ~44ms total)");

    // Read all 4 channels
    int16_t raw_value;
    float voltage;
    bool all_valid = true;

    for (int channel = 0; channel < 4; channel++) {
        esp_err_t ret = ads1115_helper_read_channel(2, ADS111X_MUX_0_GND + channel, &raw_value, &voltage);
        if (ret == ESP_OK) {
            data->photoresistor_readings[channel] = voltage;
            ESP_LOGD(TAG, "Photoresistor CH:%d %.3fV", channel, voltage);
        } else {
            ESP_LOGW(TAG, "Failed to read photoresistor ch%d: %s", channel, esp_err_to_name(ret));
            data->photoresistor_readings[channel] = 0.0f;
            all_valid = false;
        }
    }

    // Calculate tracking errors (photoresistor arrangement: TL, TR, BL, BR)
    float left_avg = (data->photoresistor_readings[0] + data->photoresistor_readings[2]) / 2.0f;   // TL + BL
    float right_avg = (data->photoresistor_readings[1] + data->photoresistor_readings[3]) / 2.0f;  // TR + BR
    float top_avg = (data->photoresistor_readings[0] + data->photoresistor_readings[1]) / 2.0f;    // TL + TR
    float bottom_avg = (data->photoresistor_readings[2] + data->photoresistor_readings[3]) / 2.0f; // BL + BR

    data->yaw_error = left_avg - right_avg;      // Negative = turn right, Positive = turn left
    data->pitch_error = top_avg - bottom_avg;    // Negative = tilt down, Positive = tilt up
    data->valid = all_valid;
    data->timestamp = time(NULL);

    return ESP_OK;
}

// ########################## Servo Control ##########################

/**
 * @brief Set servo position via PWM
 *
 * Sets servo PWM duty cycle with safety clamping to prevent mechanical damage.
 * Duty cycle range: FLUCTUS_SERVO_MIN_DUTY to FLUCTUS_SERVO_MAX_DUTY.
 *
 * @param channel LEDC channel (FLUCTUS_SERVO_YAW_CHANNEL or FLUCTUS_SERVO_PITCH_CHANNEL)
 * @param duty_cycle Desired duty cycle value (automatically clamped to safe range)
 * @return ESP_OK on success
 */
esp_err_t fluctus_servo_set_position(ledc_channel_t channel, uint32_t duty_cycle)
{
    // Clamp duty cycle to safe range
    if (duty_cycle < FLUCTUS_SERVO_MIN_DUTY) {
        duty_cycle = FLUCTUS_SERVO_MIN_DUTY;
    } else if (duty_cycle > FLUCTUS_SERVO_MAX_DUTY) {
        duty_cycle = FLUCTUS_SERVO_MAX_DUTY;
    }
    
    esp_err_t ret = ledc_set_duty(FLUCTUS_SERVO_PWM_SPEED_MODE, channel, duty_cycle);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return ledc_update_duty(FLUCTUS_SERVO_PWM_SPEED_MODE, channel);
}

/**
 * @brief Set servo position directly for debug/testing (PUBLIC API)
 *
 * This function is called by HMI servo debug mode to directly control servos.
 * It updates both the hardware PWM and the internal solar_data (source of truth)
 * so that telemetry and display show the correct position.
 *
 * Thread-safe with xSolarMutex.
 */
esp_err_t fluctus_servo_debug_set_position(ledc_channel_t channel, uint32_t duty_cycle)
{
    // Take mutex for thread safety
    if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take xSolarMutex for servo debug");
        return ESP_FAIL;
    }

    // Clamp duty cycle to safe range
    if (duty_cycle < FLUCTUS_SERVO_MIN_DUTY) {
        duty_cycle = FLUCTUS_SERVO_MIN_DUTY;
    } else if (duty_cycle > FLUCTUS_SERVO_MAX_DUTY) {
        duty_cycle = FLUCTUS_SERVO_MAX_DUTY;
    }

    // Set hardware PWM
    esp_err_t ret = fluctus_servo_set_position(channel, duty_cycle);

    if (ret == ESP_OK) {
        // Update solar_data (source of truth) so telemetry reflects the new position
        if (channel == FLUCTUS_SERVO_YAW_CHANNEL) {
            solar_data.current_yaw_duty = duty_cycle;
            solar_data.yaw_position_percent = (uint8_t)fluctus_duty_to_percent(duty_cycle);
        } else if (channel == FLUCTUS_SERVO_PITCH_CHANNEL) {
            solar_data.current_pitch_duty = duty_cycle;
            solar_data.pitch_position_percent = (uint8_t)fluctus_duty_to_percent(duty_cycle);
        }
    }

    xSemaphoreGive(xSolarMutex);
    return ret;
}

/**
 * @brief Apply servo corrections with smooth, small adjustments (refactored)
 *
 * Replaces fluctus_apply_servo_corrections() with smaller step size for continuous tracking.
 * Uses lower proportional gain (100 vs 1000) and smaller max step (15 vs 250) for smooth motion.
 *
 * Thread-safe: Takes xSolarMutex to read/write solar_data (source of truth for servo positions).
 *
 * @param tracking_data Tracking data with yaw/pitch errors
 * @return true if servos were updated
 */
bool fluctus_apply_servo_corrections_smooth(fluctus_solar_data_t *tracking_data)
{
    // Calculate desired adjustment direction with small step size
    int32_t yaw_adjustment = 0;
    int32_t pitch_adjustment = 0;

    if (fabsf(tracking_data->yaw_error) >= FLUCTUS_PHOTORESISTOR_THRESHOLD) {
        // Proportional with small max step (lower gain for 250ms loop vs old 3s)
        yaw_adjustment = (int32_t)(tracking_data->yaw_error * 100);  // Lower gain (was 1000)
        if (yaw_adjustment > FLUCTUS_SERVO_MAX_STEP_PER_ITERATION) {
            yaw_adjustment = FLUCTUS_SERVO_MAX_STEP_PER_ITERATION;
        } else if (yaw_adjustment < -FLUCTUS_SERVO_MAX_STEP_PER_ITERATION) {
            yaw_adjustment = -FLUCTUS_SERVO_MAX_STEP_PER_ITERATION;
        }
    }

    if (fabsf(tracking_data->pitch_error) >= FLUCTUS_PHOTORESISTOR_THRESHOLD) {
        pitch_adjustment = (int32_t)(tracking_data->pitch_error * 100);
        if (pitch_adjustment > FLUCTUS_SERVO_MAX_STEP_PER_ITERATION) {
            pitch_adjustment = FLUCTUS_SERVO_MAX_STEP_PER_ITERATION;
        } else if (pitch_adjustment < -FLUCTUS_SERVO_MAX_STEP_PER_ITERATION) {
            pitch_adjustment = -FLUCTUS_SERVO_MAX_STEP_PER_ITERATION;
        }
    }

    // Take mutex to safely read/write servo positions in solar_data
    if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take xSolarMutex for servo correction");
        return false;
    }

    // Apply adjustments with clamping
    bool updated = false;

    if (yaw_adjustment != 0) {
        int32_t new_yaw = (int32_t)solar_data.current_yaw_duty + yaw_adjustment;
        // Clamp to servo limits
        if (new_yaw < FLUCTUS_SERVO_MIN_DUTY) {
            new_yaw = FLUCTUS_SERVO_MIN_DUTY;
        } else if (new_yaw > FLUCTUS_SERVO_MAX_DUTY) {
            new_yaw = FLUCTUS_SERVO_MAX_DUTY;
        }

        if ((uint32_t)new_yaw != solar_data.current_yaw_duty) {
            fluctus_servo_set_position(FLUCTUS_SERVO_YAW_CHANNEL, (uint32_t)new_yaw);
            solar_data.current_yaw_duty = (uint32_t)new_yaw;
            solar_data.yaw_position_percent = (uint8_t)fluctus_duty_to_percent((uint32_t)new_yaw);
            updated = true;
        }
    }

    if (pitch_adjustment != 0) {
        int32_t new_pitch = (int32_t)solar_data.current_pitch_duty + pitch_adjustment;
        // Clamp to servo limits
        if (new_pitch < FLUCTUS_SERVO_MIN_DUTY) {
            new_pitch = FLUCTUS_SERVO_MIN_DUTY;
        } else if (new_pitch > FLUCTUS_SERVO_MAX_DUTY) {
            new_pitch = FLUCTUS_SERVO_MAX_DUTY;
        }

        if ((uint32_t)new_pitch != solar_data.current_pitch_duty) {
            fluctus_servo_set_position(FLUCTUS_SERVO_PITCH_CHANNEL, (uint32_t)new_pitch);
            solar_data.current_pitch_duty = (uint32_t)new_pitch;
            solar_data.pitch_position_percent = (uint8_t)fluctus_duty_to_percent((uint32_t)new_pitch);
            updated = true;
        }
    }

    if (updated) {
        ESP_LOGD(TAG, "Smooth correction - Yaw: %lu, Pitch: %lu, Errors: %.3fV/%.3fV",
                solar_data.current_yaw_duty, solar_data.current_pitch_duty,
                tracking_data->yaw_error, tracking_data->pitch_error);
    }

    xSemaphoreGive(xSolarMutex);
    return updated;
}

/**
 * @brief Update solar state with fresh sensor readings (refactored helper)
 *
 * Updates solar_data (SOURCE OF TRUTH) with fresh photoresistor readings and errors.
 * Also updates STELLARIA with average light intensity.
 *
 * IMPORTANT: solar_data already contains current servo positions/tracking state,
 * this function only updates sensor readings (photoresistors, errors, timestamp).
 *
 * @param reading Fresh photoresistor reading with errors calculated
 */
void fluctus_update_solar_cache(fluctus_solar_data_t *reading)
{
    if (reading == NULL) {
        return;
    }

    // Update solar_data with fresh sensor readings (servo positions already up-to-date)
    if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        // Copy photoresistor data and errors
        memcpy(solar_data.photoresistor_readings,
               reading->photoresistor_readings,
               sizeof(solar_data.photoresistor_readings));
        solar_data.yaw_error = reading->yaw_error;
        solar_data.pitch_error = reading->pitch_error;
        solar_data.valid = reading->valid;
        solar_data.timestamp = reading->timestamp;

        // Update derived percent values from current duty cycles
        solar_data.yaw_position_percent = (uint8_t)fluctus_duty_to_percent(solar_data.current_yaw_duty);
        solar_data.pitch_position_percent = (uint8_t)fluctus_duty_to_percent(solar_data.current_pitch_duty);

        xSemaphoreGive(xSolarMutex);
    }

    // Update STELLARIA with average light intensity
    float avg_light = (reading->photoresistor_readings[0] +
                       reading->photoresistor_readings[1] +
                       reading->photoresistor_readings[2] +
                       reading->photoresistor_readings[3]) / 4.0f;
    stellaria_update_light_intensity(avg_light);
}

// ########################## Utility Functions ##########################

/**
 * @brief Convert percentage (0-100) to servo duty cycle
 * @param percent Percentage value (0.0 = min, 100.0 = max)
 * @return Duty cycle value
 */
uint32_t fluctus_percent_to_duty(float percent)
{
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    uint32_t range = FLUCTUS_SERVO_MAX_DUTY - FLUCTUS_SERVO_MIN_DUTY;
    return FLUCTUS_SERVO_MIN_DUTY + (uint32_t)((percent / 100.0f) * range);
}

/**
 * @brief Convert servo duty cycle to percentage (inverse of fluctus_percent_to_duty)
 * @param duty Duty cycle value
 * @return Percentage value (0.0-100.0)
 */
float fluctus_duty_to_percent(uint32_t duty)
{
    if (duty <= FLUCTUS_SERVO_MIN_DUTY) {
        return 0.0f;
    } else if (duty >= FLUCTUS_SERVO_MAX_DUTY) {
        return 100.0f;
    }

    uint32_t range = FLUCTUS_SERVO_MAX_DUTY - FLUCTUS_SERVO_MIN_DUTY;
    uint32_t offset = duty - FLUCTUS_SERVO_MIN_DUTY;
    return ((float)offset / (float)range) * 100.0f;
}

// ########################## Parking Functions ##########################

/**
 * @brief Park servos in night position (east-facing for morning sun)
 *
 * Parks panel facing east with upward tilt to capture early morning sunlight.
 * Position: 10% yaw (left/east), 60% pitch (upward tilt)
 *
 * Includes servo power-up delay (200ms) to allow servos to settle before releasing power.
 * Updates solar_data (source of truth) with final servo positions.
 *
 * @return ESP_OK on success, ESP_FAIL if any servo command fails
 */
esp_err_t fluctus_park_servos_night(void)
{
    uint32_t yaw_duty = fluctus_percent_to_duty(FLUCTUS_SERVO_NIGHT_PARK_YAW_PERCENT);
    uint32_t pitch_duty = fluctus_percent_to_duty(FLUCTUS_SERVO_NIGHT_PARK_PITCH_PERCENT);

    esp_err_t ret1 = fluctus_servo_set_position(FLUCTUS_SERVO_YAW_CHANNEL, yaw_duty);
    esp_err_t ret2 = fluctus_servo_set_position(FLUCTUS_SERVO_PITCH_CHANNEL, pitch_duty);

    if (ret1 == ESP_OK && ret2 == ESP_OK) {
        // Update solar_data (source of truth) with mutex protection
        if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
            solar_data.current_yaw_duty = yaw_duty;
            solar_data.current_pitch_duty = pitch_duty;
            solar_data.yaw_position_percent = (uint8_t)FLUCTUS_SERVO_NIGHT_PARK_YAW_PERCENT;
            solar_data.pitch_position_percent = (uint8_t)FLUCTUS_SERVO_NIGHT_PARK_PITCH_PERCENT;
            xSemaphoreGive(xSolarMutex);
        }
        ESP_LOGI(TAG, "Servos parked in night position (yaw=%.0f%%, pitch=%.0f%%)",
                 FLUCTUS_SERVO_NIGHT_PARK_YAW_PERCENT, FLUCTUS_SERVO_NIGHT_PARK_PITCH_PERCENT);
    }
    vTaskDelay(pdMS_TO_TICKS(FLUCTUS_SERVO_POWERUP_DELAY_MS));

    return (ret1 == ESP_OK && ret2 == ESP_OK) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Park servos in error/center position
 *
 * Parks panel in center position (50%/50%) for safe recovery from errors.
 * Used when:
 * - User manually disables tracking
 * - Critical power state reached
 * - Repeated tracking errors exceed threshold
 * - Correction timeout occurs
 *
 * Includes servo power-up delay (200ms) to allow servos to settle before releasing power.
 * Updates solar_data (source of truth) with final servo positions.
 *
 * @return ESP_OK on success, ESP_FAIL if any servo command fails
 */
esp_err_t fluctus_park_servos_error(void)
{
    uint32_t yaw_duty = fluctus_percent_to_duty(FLUCTUS_SERVO_ERROR_PARK_YAW_PERCENT);
    uint32_t pitch_duty = fluctus_percent_to_duty(FLUCTUS_SERVO_ERROR_PARK_PITCH_PERCENT);

    esp_err_t ret1 = fluctus_servo_set_position(FLUCTUS_SERVO_YAW_CHANNEL, yaw_duty);
    esp_err_t ret2 = fluctus_servo_set_position(FLUCTUS_SERVO_PITCH_CHANNEL, pitch_duty);

    if (ret1 == ESP_OK && ret2 == ESP_OK) {
        // Update solar_data (source of truth) with mutex protection
        if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
            solar_data.current_yaw_duty = yaw_duty;
            solar_data.current_pitch_duty = pitch_duty;
            solar_data.yaw_position_percent = (uint8_t)FLUCTUS_SERVO_ERROR_PARK_YAW_PERCENT;
            solar_data.pitch_position_percent = (uint8_t)FLUCTUS_SERVO_ERROR_PARK_PITCH_PERCENT;
            xSemaphoreGive(xSolarMutex);
        }
        ESP_LOGW(TAG, "Servos parked in error/center position (yaw=%.0f%%, pitch=%.0f%%)",
                 FLUCTUS_SERVO_ERROR_PARK_YAW_PERCENT, FLUCTUS_SERVO_ERROR_PARK_PITCH_PERCENT);
    }
    vTaskDelay(pdMS_TO_TICKS(FLUCTUS_SERVO_POWERUP_DELAY_MS));

    return (ret1 == ESP_OK && ret2 == ESP_OK) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Check if tracking errors are within acceptable threshold
 * @param yaw_error Yaw error in volts
 * @param pitch_error Pitch error in volts
 * @return true if errors are below threshold
 */
bool fluctus_tracking_error_margin_check(float yaw_error, float pitch_error)
{
    return (fabsf(yaw_error) < FLUCTUS_PHOTORESISTOR_THRESHOLD &&
            fabsf(pitch_error) < FLUCTUS_PHOTORESISTOR_THRESHOLD);
}

/**
 * @brief Check if environmental conditions are suitable for solar operations
 *
 * SINGLE SOURCE OF TRUTH for daytime and light detection.
 * Independent of solar tracking enabled state - can be used to decide whether to enable tracking.
 *
 * Decision logic:
 * 1. Astronomical daytime check (cheap, always available)
 * 2. Photoresistor light intensity check (requires hardware)
 *
 * IMPORTANT: Updates global solar_data with fresh photoresistor readings.
 * This cache is used by snapshot functions to avoid redundant photoresistor reads.
 *
 * Used by:
 * - PV INA219 power management (daytime detection)
 * - Solar tracking auto-enable/disable decisions
 * - Sunset parking condition checks
 *
 * @return true if daytime AND sufficient light detected, false otherwise
 */
bool fluctus_is_daytime_with_sufficient_light(void)
{
    // First check: Astronomical daytime (cheap, always available)
    if (!solar_calc_is_daytime_buffered()) {
        return false;  // Definitely nighttime according to calculations
    }

    // Second check: Actual light intensity from photoresistors
    // Read fresh photoresistor data (populates photoresistor_readings, errors, valid, timestamp)
    fluctus_solar_data_t temp_data = {0};
    if (fluctus_read_photoresistors(&temp_data) != ESP_OK || !temp_data.valid) {
        return false;
    }

    // Update global cache with fresh photoresistor data (also updates STELLARIA)
    fluctus_update_solar_cache(&temp_data);

    // Calculate average light for threshold check
    float avg_light = (temp_data.photoresistor_readings[0] +
                      temp_data.photoresistor_readings[1] +
                      temp_data.photoresistor_readings[2] +
                      temp_data.photoresistor_readings[3]) / 4.0f;

    // Combined decision: astronomical daytime AND sufficient light detected
    return (avg_light >= FLUCTUS_SOLAR_LIGHT_THRESHOLD);
}

/**
 * @brief Thread-safe state transition with logging
 *
 * Updates solar_data.tracking_state (source of truth) with mutex protection.
 *
 * @param new_state Target state
 * @param log_msg Log message (NULL to skip logging)
 */
void fluctus_set_tracking_state(solar_tracking_state_t new_state, const char *log_msg)
{
    if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        solar_data.tracking_state = new_state;
        if (log_msg) {
            ESP_LOGI(TAG, "%s", log_msg);
        }
        xSemaphoreGive(xSolarMutex);
    }
}

// ########################## Solar Tracking State Handlers ##########################

/**
 * @brief Handle STANDBY state - periodic correction cycle checks
 *
 * Checks every 15 minutes whether to start a correction cycle.
 * Transitions to PARKING if true sunset detected, or CORRECTING if daytime with sufficient light.
 * In debug mode, bypasses daytime checks for continuous testing.
 *
 * @param current_time_ms Current time in milliseconds (for correction interval timing)
 */
void fluctus_solar_state_standby(int64_t current_time_ms)
{
    // In debug mode, skip daytime checks and force continuous operation
    if (!solar_debug_mode_active) {
        // Normal mode: Check for sunset parking condition using centralized environmental check
        if (!fluctus_is_daytime_with_sufficient_light()) {
            // Insufficient light detected - determine if it's true sunset or just cloudy
            if (solar_calc_is_daytime_buffered()) {
                // Astronomically daytime but cloudy - stay in STANDBY, retry in 15 minutes
                ESP_LOGW(TAG, "Insufficient light but still daytime - will retry in 15 minutes");
                return;
            } else {
                // True sunset (astronomically nighttime)
                current_parking_reason = PARKING_REASON_SUNSET;
                fluctus_set_tracking_state(SOLAR_TRACKING_PARKING, "Sunset - insufficient light for tracking");
                return;
            }
        }
    } else {
        // Debug mode: Always proceed with correction (bypass daytime checks)
        ESP_LOGD(TAG, "Debug mode active - bypassing daytime checks");
    }

    // Check if we should start correction cycle (from config, or immediate in debug mode)
    if (last_correction_time == 0 ||
        (current_time_ms - last_correction_time) >= fluctus_solar_correction_interval_ms ||
        solar_debug_mode_active) {

        // Request 6.6V servo bus power for correction cycle
        if (fluctus_request_bus_power(POWER_BUS_6V6, "FLUCTUS_SOLAR") == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(FLUCTUS_SERVO_POWERUP_DELAY_MS));
            correction_start_time = current_time_ms;
            last_correction_time = current_time_ms;

            // Resume servo control task for smooth tracking
            vTaskResume(xFluctusSolarServoControlTaskHandle);

            fluctus_set_tracking_state(SOLAR_TRACKING_CORRECTING,
                solar_debug_mode_active ? "Starting DEBUG correction cycle (6.6V bus powered on, servo task resumed)" :
                                         "Starting correction cycle (6.6V bus powered on, servo task resumed)");
        } else {
            ESP_LOGE(TAG, "Failed to power 6.6V bus for solar tracking");
        }
    }
}

/**
 * @brief Handle CORRECTING state - blocking entry function with exit condition handling
 *
 * This function is called ONCE when entering CORRECTING state. It performs:
 * 1. ONE-TIME daytime/light check at entry
 * 2. Power up 6.6V bus and resume servo control task
 * 3. BLOCK waiting for completion notifications (CONVERGED/TIMEOUT/DISABLE)
 * 4. Handle exit conditions and clean up (power off, set next state)
 *
 * BLOCKING DESIGN:
 * - Uses xTaskNotifyWait() to block until servo task completes or is interrupted
 * - 45s safety timeout prevents indefinite blocking if servo task hangs
 * - Can be interrupted by DISABLE notification (user or critical power)
 *
 * SEPARATION OF CONCERNS:
 * - This function: Entry checks, power management, blocking wait, cleanup
 * - Servo control task: Timeout monitoring, sensor reads, servo control, convergence detection
 *
 * In debug mode, bypasses daytime checks for continuous testing.
 */
void fluctus_solar_state_correcting(void)
{
    // ===== ENTRY: ONE-TIME DAYTIME/LIGHT CHECK =====
    if (!solar_debug_mode_active && !fluctus_is_daytime_with_sufficient_light()) {
        // Insufficient light detected - determine if it's true sunset or just cloudy
        if (solar_calc_is_daytime_buffered()) {
            // Cloudy during daytime - abort, retry next interval
            fluctus_set_tracking_state(SOLAR_TRACKING_STANDBY,
                "Insufficient light for correction - will retry");
        } else {
            // True sunset
            current_parking_reason = PARKING_REASON_SUNSET;
            fluctus_set_tracking_state(SOLAR_TRACKING_PARKING,
                "Sunset detected - parking");
        }
        return;
    }

    // ===== POWER UP AND START SERVO TASK =====
    if (fluctus_request_bus_power(POWER_BUS_6V6, "FLUCTUS_SOLAR") != ESP_OK) {
        ESP_LOGE(TAG, "Failed to power 6.6V bus for correction");
        fluctus_set_tracking_state(SOLAR_TRACKING_ERROR, NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(FLUCTUS_SERVO_POWERUP_DELAY_MS));
    correction_start_time = esp_timer_get_time() / 1000;

    // Resume servo task - it will notify us when done
    vTaskResume(xFluctusSolarServoControlTaskHandle);
    ESP_LOGI(TAG, "Correction cycle started (6.6V powered, servo task resumed)");

    // ===== BLOCK: WAIT FOR COMPLETION OR DISABLE (30s timeout) =====
    uint32_t notification = 0;
    BaseType_t notified = xTaskNotifyWait(
        0,  // Don't clear on entry
        FLUCTUS_NOTIFY_SERVO_CONVERGED | FLUCTUS_NOTIFY_SOLAR_DISABLE,
        &notification,
        pdMS_TO_TICKS(FLUCTUS_TRACKING_ADJUSTMENT_CYCLE_TIMEOUT_MS)  // 30s timeout
    );

    // ===== CHECK FOR TIMEOUT (no notification received in 30s) =====
    if (notified == pdFALSE || notification == 0) {
        ESP_LOGW(TAG, "Correction timeout after 30s - suspending servo task");
        vTaskSuspend(xFluctusSolarServoControlTaskHandle);
        // Don't release power - ERROR handler will release it
        fluctus_set_tracking_state(SOLAR_TRACKING_ERROR, "Correction timeout");
        return;
    }

    // ===== HANDLE DISABLE (interrupt during correction) =====
    if (notification & FLUCTUS_NOTIFY_SOLAR_DISABLE) {
        ESP_LOGI(TAG, "Disable requested during correction - aborting");
        vTaskSuspend(xFluctusSolarServoControlTaskHandle);
        // Don't release power - PARKING handler will release it

        // Check if it's user disable or critical power
        if (current_parking_reason != PARKING_REASON_CRITICAL_POWER) {
            current_parking_reason = PARKING_REASON_USER_DISABLE;
        }
        fluctus_set_tracking_state(SOLAR_TRACKING_PARKING, "Disabled during correction");
        return;
    }

    // ===== HANDLE CONVERGENCE =====
    if (notification & FLUCTUS_NOTIFY_SERVO_CONVERGED) {
        ESP_LOGI(TAG, "Correction converged successfully");
        consecutive_tracking_errors = 0;  // Reset error counter on success
        // Release power here - STANDBY doesn't handle power release
        fluctus_release_bus_power(POWER_BUS_6V6, "FLUCTUS_SOLAR");
        fluctus_set_tracking_state(SOLAR_TRACKING_STANDBY, "Correction complete");
    }
}

/**
 * @brief Handle PARKING state - park servos and choose next state
 *
 * Parks servos in night position (east-facing) and releases power.
 * Next state depends on parking reason:
 * - SUNSET: Transition to SLEEPING (auto-resume at sunrise)
 * - USER_DISABLE / CRITICAL_POWER: Transition to DISABLED (manual enable required)
 */
void fluctus_solar_state_parking(void)
{
    // Choose next state based on parking reason
    switch (current_parking_reason) {
        case PARKING_REASON_SUNSET:
            fluctus_park_servos_night();
            fluctus_set_tracking_state(SOLAR_TRACKING_SLEEPING, "Parked for night - will auto-resume at sunrise");
            break;

        case PARKING_REASON_USER_DISABLE:
            fluctus_park_servos_error();
            fluctus_set_tracking_state(SOLAR_TRACKING_DISABLED, "Disabled by user - manual enable required");
            break;

        case PARKING_REASON_CRITICAL_POWER:
            fluctus_park_servos_error();
            fluctus_set_tracking_state(SOLAR_TRACKING_DISABLED, "Disabled due to critical power - manual enable required");
            break;

        default:
            // Fallback
            fluctus_park_servos_error();
            fluctus_set_tracking_state(SOLAR_TRACKING_STANDBY, "Parked (unknown reason) - will retry at next interval");
            break;
    }

    fluctus_release_bus_power(POWER_BUS_6V6, "FLUCTUS_SOLAR");
}

/**
 * @brief Handle ERROR state - recovery from tracking errors
 *
 * Handles unintentional failures (timeouts, sensor issues, mechanical problems).
 * Tracks consecutive errors and disables tracking if threshold exceeded.
 * Parks servos in center position for safe recovery.
 */
void fluctus_solar_state_error(void)
{
    // Increment consecutive error counter
    consecutive_tracking_errors++;

    ESP_LOGW(TAG, "Solar tracking error #%d (max: %d)",
             consecutive_tracking_errors, FLUCTUS_MAX_CONSECUTIVE_ERRORS);

    // Check if too many consecutive errors - disable tracking
    if (consecutive_tracking_errors >= FLUCTUS_MAX_CONSECUTIVE_ERRORS) {
        ESP_LOGE(TAG, "Too many consecutive tracking errors (%d) - disabling solar tracking",
                 consecutive_tracking_errors);
        fluctus_park_servos_error();
        fluctus_release_bus_power(POWER_BUS_6V6, "FLUCTUS_SOLAR");
        consecutive_tracking_errors = 0;  // Reset counter
        fluctus_set_tracking_state(SOLAR_TRACKING_DISABLED,
                                   "Disabled due to repeated errors - manual enable required");
        return;
    }

    // Park in safe center position (error recovery)
    fluctus_park_servos_error();
    fluctus_release_bus_power(POWER_BUS_6V6, "FLUCTUS_SOLAR");

    // Add progressive delay based on error count (backoff strategy)
    uint32_t retry_delay_ms = consecutive_tracking_errors * 5000;  // 5s, 10s, 15s, 20s...
    if (retry_delay_ms > 0) {
        ESP_LOGW(TAG, "Adding %lu ms delay before retry due to error count", retry_delay_ms);
        vTaskDelay(pdMS_TO_TICKS(retry_delay_ms));
    }

    fluctus_set_tracking_state(SOLAR_TRACKING_STANDBY,
                               "Error recovery - parked in center, will retry on next cycle");
}

// ########################## Callbacks ##########################

/**
 * @brief Sunrise callback - wakes up SLEEPING solar tracking
 *
 * Called by solar_calc at sunrise time (buffered -30 minutes).
 * Sends notification to solar tracking task to transition from SLEEPING to STANDBY.
 */
void fluctus_on_sunrise_callback(void)
{
    ESP_LOGI(TAG, "Sunrise callback triggered - waking SLEEPING solar tracking");

    if (xFluctusSolarTrackingTaskHandle != NULL) {
        xTaskNotify(xFluctusSolarTrackingTaskHandle, FLUCTUS_NOTIFY_SUNRISE, eSetBits);
    }
}

// ########################## Servo Control Task (Refactored Smooth Tracking) ##########################

/**
 * @brief Servo control task for smooth, continuous solar tracking corrections
 *
 * Runs at 250ms intervals during CORRECTING state to provide fluid servo motion.
 * Replaces the old 3-second correction cycles with many small adjustments.
 *
 * BEHAVIOR:
 * - Created at init, starts SUSPENDED
 * - Resumed when state machine enters CORRECTING state
 * - Runs 250ms loop:
 *   1. Read all 4 photoresistors (~44ms)
 *   2. Calculate yaw/pitch errors
 *   3. Update solar_data for telemetry
 *   4. If errors below threshold: increment settling counter (12 iterations = 3s)
 *   5. If errors above threshold: reset counter, apply small correction (max 15 duty units)
 *   6. If settled for 3s: notify CONVERGED, suspend self
 * - Suspended on convergence (or forcibly by state machine on timeout/disable)
 *
 * SEPARATION OF CONCERNS:
 * - State machine: Daytime check, timeout (via xTaskNotifyWait), power management
 * - This task: Sensor reads, servo control, convergence detection
 *
 * Thread-safe with xSolarMutex for cache updates.
 */
void fluctus_solar_servo_correction_task(void *parameters)
{
    ESP_LOGI(TAG, "Servo control task started (will run suspended until CORRECTING state)");

    while (true) {
        servo_control_active = true;
        total_correction_iterations++;

        // ===== READ PHOTORESISTORS =====
        fluctus_solar_data_t reading = {0};
        esp_err_t read_ret = fluctus_read_photoresistors(&reading);
        if (read_ret != ESP_OK || !reading.valid) {
            ESP_LOGW(TAG, "Servo control: Failed to read photoresistors, retrying next cycle");
            vTaskDelay(pdMS_TO_TICKS(FLUCTUS_SERVO_CONTROL_LOOP_MS));
            continue;
        }

        // Update cache for telemetry (thread-safe, also updates STELLARIA)
        fluctus_update_solar_cache(&reading);

        // ===== CHECK ERROR THRESHOLD =====
        bool within_threshold = fluctus_tracking_error_margin_check(reading.yaw_error, reading.pitch_error);

        if (within_threshold) {
            // Errors acceptable - increment settling counter
            settling_counter++;
            ESP_LOGD(TAG, "Servo control: Within threshold (%d/%d) - Yaw: %.3fV, Pitch: %.3fV",
                     settling_counter, FLUCTUS_SERVO_SETTLING_COUNT,
                     reading.yaw_error, reading.pitch_error);

            if (settling_counter >= FLUCTUS_SERVO_SETTLING_COUNT) {
                // ===== CONVERGED =====
                ESP_LOGI(TAG, "Servo control: CONVERGED after %d iterations (%.1fs total, 3s settling)",
                         total_correction_iterations,
                         (float)total_correction_iterations * FLUCTUS_SERVO_CONTROL_LOOP_MS / 1000.0f);
                xTaskNotify(xFluctusSolarTrackingTaskHandle,
                           FLUCTUS_NOTIFY_SERVO_CONVERGED, eSetBits);
                settling_counter = 0;
                total_correction_iterations = 0;
                servo_control_active = false;
                vTaskSuspend(NULL);  // Suspend until next correction cycle
                // On resume, counters already reset
                continue;
            }
        } else {
            // Errors too large - reset settling counter and apply correction
            if (settling_counter > 0) {
                ESP_LOGD(TAG, "Servo control: Error exceeded threshold - resetting settling counter");
            }
            settling_counter = 0;
            fluctus_apply_servo_corrections_smooth(&reading);
        }

        // Wait for next control loop iteration
        vTaskDelay(pdMS_TO_TICKS(FLUCTUS_SERVO_CONTROL_LOOP_MS));
    }
}

// ########################## Solar Tracking Main Task ##########################

/**
 * @brief Solar tracking task with notification-based state machine and automatic scheduling
 *
 * Uses task notifications for efficient power management:
 * - DISABLED: Wait indefinitely for ENABLE notification (no CPU usage)
 * - STANDBY: 15-minute intervals for correction cycles
 * - CORRECTING: 3-second updates during active tracking
 * - PARKING/ERROR: Execute immediately
 *
 * Notifications:
 * - FLUCTUS_NOTIFY_SOLAR_ENABLE: Enable tracking and enter STANDBY state
 * - FLUCTUS_NOTIFY_SOLAR_DISABLE: Disable tracking and enter DISABLED state
 */
void fluctus_solar_tracking_task(void *parameters)
{
    ESP_LOGI(TAG, "Solar tracking task started.");
    solar_tracking_state_t current_state;

    while (true) {
        // Read current state and perform safety checks
        bool debug_mode_active_snapshot = false;  // Thread-safe local copy

        // Read solar tracking state (protected by xSolarMutex)
        if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
            current_state = solar_data.tracking_state;

            // Check debug mode timeout (90 seconds)
            if (solar_debug_mode_active && debug_mode_start_time > 0) {
                int64_t current_time_ms = esp_timer_get_time() / 1000;
                if ((current_time_ms - debug_mode_start_time) >= FLUCTUS_DEBUG_MODE_TIMEOUT_MS) {
                    solar_debug_mode_active = false;
                    debug_mode_start_time = 0;
                    ESP_LOGI(TAG, "Debug mode TIMED OUT after 90 seconds");
                }
            }

            // Take thread-safe snapshot of debug mode flag
            debug_mode_active_snapshot = solar_debug_mode_active;

            xSemaphoreGive(xSolarMutex);
        } else {
            continue;
        }

        // Safety check: Read power state (protected by xPowerBusMutex)
        bool safety_override = false;
        if (xSemaphoreTake(xPowerBusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
            if (system_status.safety_shutdown ||
                system_status.power_state >= FLUCTUS_POWER_STATE_CRITICAL) {
                safety_override = true;
            }
            xSemaphoreGive(xPowerBusMutex);
        }

        // Apply safety override if needed (write to solar_data)
        if (safety_override && current_state != SOLAR_TRACKING_DISABLED) {
            if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
                solar_data.tracking_state = SOLAR_TRACKING_DISABLED;
                current_state = SOLAR_TRACKING_DISABLED;

                // Also disable debug mode
                if (solar_debug_mode_active) {
                    solar_debug_mode_active = false;
                    debug_mode_start_time = 0;
                    ESP_LOGI(TAG, "Debug mode disabled due to safety/power state");
                }

                xSemaphoreGive(xSolarMutex);
                ESP_LOGW(TAG, "Solar tracking disabled due to safety or power state");
            }
        }

        // Determine timeout based on current state
        TickType_t timeout;
        switch (current_state) {
            case SOLAR_TRACKING_DISABLED:
            case SOLAR_TRACKING_SLEEPING:
                timeout = portMAX_DELAY;  // Wait indefinitely for notification
                break;
            case SOLAR_TRACKING_STANDBY:
                // In debug mode, run immediately to start correction cycles
                if (debug_mode_active_snapshot) {
                    timeout = 0;  // Immediate execution for debug
                } else {
                    timeout = pdMS_TO_TICKS(fluctus_solar_correction_interval_ms);  // From config/set by intervals
                }
                break;
            case SOLAR_TRACKING_CORRECTING:
                // CORRECTING blocks inside its function - should not reach here normally
                // If we do reach here, state was already set by correcting function
                timeout = 0;  // Execute immediately to handle new state
                break;
            case SOLAR_TRACKING_PARKING:
            case SOLAR_TRACKING_ERROR:
            default:
                timeout = 0;  // Execute immediately
                break;
        }

        // Wait for notification or timeout
        uint32_t notification_value = 0;
        BaseType_t notified = xTaskNotifyWait(
            0x00,                           // Don't clear bits on entry
            ULONG_MAX,                      // Clear all bits on exit
            &notification_value,            // Store notification value
            timeout                         // Variable timeout
        );

        // Handle notifications (immediate state changes)
        if (notified == pdTRUE) {
            if (notification_value & FLUCTUS_NOTIFY_SOLAR_ENABLE) {
                ESP_LOGI(TAG, "Solar tracking enable notification received");
                fluctus_set_tracking_state(SOLAR_TRACKING_STANDBY, "Enabled via notification");
                continue;  // Skip to next iteration
            }
            if (notification_value & FLUCTUS_NOTIFY_SOLAR_DISABLE) {
                ESP_LOGI(TAG, "Solar tracking disable notification received");
                // Set parking reason to USER_DISABLE unless critical power already set it
                if (current_parking_reason != PARKING_REASON_CRITICAL_POWER) {
                    current_parking_reason = PARKING_REASON_USER_DISABLE;
                }
                // Transition to PARKING - will park servos and choose final state based on reason
                fluctus_set_tracking_state(SOLAR_TRACKING_PARKING, "Disable requested - parking servos");
                continue;  // Skip to next iteration
            }
            if (notification_value & FLUCTUS_NOTIFY_SUNRISE) {
                ESP_LOGI(TAG, "Sunrise notification received");
                // Only act on sunrise if we're currently SLEEPING
                if (current_state == SOLAR_TRACKING_SLEEPING) {
                    // Unconditionally wake to STANDBY - 15-min correction intervals will handle cloudy conditions
                    fluctus_set_tracking_state(SOLAR_TRACKING_STANDBY, "Waking from SLEEPING - entering STANDBY");
                }
                continue;  // Skip to next iteration
            }
            // Note: SERVO_CONVERGED and SERVO_TIMEOUT notifications are handled inside
            // fluctus_solar_state_correcting() which blocks waiting for them
        }

        // Get current time for state machine execution
        int64_t current_time_ms = esp_timer_get_time() / 1000;

        // Execute state machine - delegate to state handlers
        switch (current_state) {
            case SOLAR_TRACKING_DISABLED:
                // Wait indefinitely for ENABLE notification (handled above)
                break;

            case SOLAR_TRACKING_SLEEPING:
                // Wait indefinitely for SUNRISE notification (handled above)
                // Will unconditionally transition to STANDBY on sunrise
                break;

            case SOLAR_TRACKING_STANDBY:
                fluctus_solar_state_standby(current_time_ms);
                break;

            case SOLAR_TRACKING_CORRECTING:
                fluctus_solar_state_correcting();
                break;

            case SOLAR_TRACKING_PARKING:
                fluctus_solar_state_parking();
                break;

            case SOLAR_TRACKING_ERROR:
                fluctus_solar_state_error();
                break;

            default:
                break;
        }
    }
}

// ########################## Public APIs ##########################

/**
 * @brief Enable solar tracking system
 *
 * Sends FLUCTUS_NOTIFY_SOLAR_ENABLE notification to solar tracking task.
 * Task will transition to STANDBY state and begin periodic correction cycles.
 *
 * Safety checks:
 * - Fails if system in safety shutdown
 * - Fails if power state is CRITICAL or SHUTDOWN
 *
 * Thread-safe with xSolarMutex (100ms timeout).
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized, safety shutdown, or critical power
 * @return ESP_FAIL if mutex timeout
 */
esp_err_t fluctus_enable_solar_tracking(void)
{
    if (!fluctus_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        // Check if solar tracking can be enabled (not in critical power or safety shutdown)
        if (system_status.safety_shutdown ||
            system_status.power_state >= FLUCTUS_POWER_STATE_CRITICAL) {
            xSemaphoreGive(xSolarMutex);
            ESP_LOGW(TAG, "Cannot enable solar tracking - system in critical state or safety shutdown");
            return ESP_ERR_INVALID_STATE;
        }

        xSemaphoreGive(xSolarMutex);
    } else {
        return ESP_FAIL;
    }

    // Send enable notification to solar tracking task (async)
    // Task will handle state transition and startup logic
    ESP_LOGI(TAG, "Solar tracking enable requested");
    xTaskNotify(xFluctusSolarTrackingTaskHandle, FLUCTUS_NOTIFY_SOLAR_ENABLE, eSetBits);

    return ESP_OK;
}

/**
 * @brief Disable solar tracking system
 *
 * Sends FLUCTUS_NOTIFY_SOLAR_DISABLE notification to solar tracking task.
 * Task will:
 * 1. Set parking reason to USER_DISABLE
 * 2. Transition to PARKING state
 * 3. Park servos in center position (50%/50%)
 * 4. Release 6.6V bus power
 * 5. Transition to DISABLED state (manual enable required)
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t fluctus_disable_solar_tracking(void)
{
    if (!fluctus_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Send disable notification to solar tracking task (async)
    // Task will handle parking servos and state transition to DISABLED
    ESP_LOGI(TAG, "Solar tracking disable requested");
    xTaskNotify(xFluctusSolarTrackingTaskHandle, FLUCTUS_NOTIFY_SOLAR_DISABLE, eSetBits);

    return ESP_OK;
}

/**
 * @brief Get current solar tracking state (lightweight, no snapshot fetch)
 *
 * Reads from solar_data.tracking_state (source of truth).
 *
 * @return Current solar_tracking_state_t value
 * @note Thread-safe, uses quick mutex with 100ms timeout. Returns DISABLED on mutex timeout.
 */
solar_tracking_state_t fluctus_get_solar_tracking_state(void)
{
    if (!fluctus_initialized) {
        return SOLAR_TRACKING_DISABLED;
    }

    solar_tracking_state_t state = SOLAR_TRACKING_DISABLED;
    if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        state = solar_data.tracking_state;
        xSemaphoreGive(xSolarMutex);
    }

    return state;
}

/**
 * @brief Enable solar tracking debug mode (continuous operation for 90 seconds)
 *
 * Forces solar tracking to run continuously regardless of daytime conditions.
 * Automatically times out after 90 seconds to prevent excessive servo wear.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if solar tracking is disabled
 */
esp_err_t fluctus_enable_solar_debug_mode(void)
{
    if (!fluctus_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Check if solar tracking is enabled (not disabled)
    solar_tracking_state_t current_state = fluctus_get_solar_tracking_state();
    if (current_state == SOLAR_TRACKING_DISABLED) {
        ESP_LOGW(TAG, "Cannot enable debug mode - solar tracking is disabled");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        solar_debug_mode_active = true;
        debug_mode_start_time = esp_timer_get_time() / 1000;  // Convert to milliseconds
        xSemaphoreGive(xSolarMutex);

        ESP_LOGI(TAG, "Solar debug mode ENABLED (90s timeout)");

        // Force immediate correction cycle by notifying the task
        if (xFluctusSolarTrackingTaskHandle != NULL) {
            xTaskNotify(xFluctusSolarTrackingTaskHandle, FLUCTUS_NOTIFY_SOLAR_ENABLE, eSetBits);
        }

        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief Disable solar tracking debug mode (return to normal operation)
 * @return ESP_OK on success
 */
esp_err_t fluctus_disable_solar_debug_mode(void)
{
    if (!fluctus_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        if (solar_debug_mode_active) {
            solar_debug_mode_active = false;
            debug_mode_start_time = 0;
            ESP_LOGI(TAG, "Solar debug mode DISABLED");
        }
        xSemaphoreGive(xSolarMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief Check if solar debug mode is currently active
 * @return true if debug mode active, false otherwise
 */
bool fluctus_is_solar_debug_mode_active(void)
{
    if (!fluctus_initialized) {
        return false;
    }

    bool active = false;
    if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        active = solar_debug_mode_active;
        xSemaphoreGive(xSolarMutex);
    }

    return active;
}