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
 * - Single source of truth: system_status.solar_tracking_state
 *
 * THREAD SAFETY:
 * - Protected by xSolarMutex (100ms timeout)
 * - State stored in system_status.solar_tracking_state (shared with power_bus)
 * - Servo positions stored in system_status.current_yaw_duty/pitch_duty
 * - Runs in solar tracking task (Med-5 priority)
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "fluctus.h"
#include "fluctus_private.h"
#include "solar_calc.h"
#include "telemetry.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

static const char *TAG = "FLUCTUS_SOLAR";

// ########################## Module State ##########################
// Note: NO 'static' keyword - these are accessed via extern in fluctus_private.h

SemaphoreHandle_t xSolarMutex = NULL;  // Solar tracking mutex (initialized in fluctus.c)

// Cached solar tracking data (populated by photoresistor reads)
// Shared with telemetry and power metering to avoid redundant ADC reads
fluctus_solar_snapshot_t cached_solar_data = {0};

// Correction cycle timing
int64_t last_correction_time = 0;      // Last correction cycle start time (for interval timing)
int64_t correction_start_time = 0;     // Current correction cycle start time (for timeout check)

// Parking state tracking
parking_reason_t current_parking_reason = PARKING_REASON_SUNSET;

// Error tracking for progressive backoff
uint8_t consecutive_tracking_errors = 0;
#define FLUCTUS_MAX_CONSECUTIVE_ERRORS 5  // Disable tracking after 5 consecutive failures

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
esp_err_t fluctus_read_photoresistors(fluctus_solar_snapshot_t *data)
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
        esp_err_t ret = ads1115_helper_read_channel(2, (ads111x_mux_t)channel, &raw_value, &voltage);
        if (ret == ESP_OK) {
            data->photoresistor_readings[channel] = voltage;
            ESP_LOGD(TAG, "Photoresistor ch%d: %.3fV", channel, voltage);
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
 * @brief Calculate servo adjustment based on tracking error
 *
 * Implements proportional control with clamping:
 * - Error < threshold: No adjustment (within acceptable range)
 * - Error ≥ threshold: Proportional adjustment (scaled by 1000)
 * - Max adjustment: ±FLUCTUS_MAX_SERVO_ADJUSTMENT per cycle
 *
 * @param error Tracking error in volts (from differential photoresistor calculation)
 * @param current_duty Current servo duty cycle value
 * @return New duty cycle value (clamped to safe servo limits)
 */
uint32_t fluctus_calculate_servo_correction(float error, uint32_t current_duty)
{
    if (fabs(error) < FLUCTUS_PHOTORESISTOR_THRESHOLD) {
        return current_duty; // No adjustment needed
    }
    
    // Proportional control
    int32_t adjustment = (int32_t)(error * 1000); // Scale factor
    
    // Limit adjustment magnitude
    if (adjustment > FLUCTUS_MAX_SERVO_ADJUSTMENT) {
        adjustment = FLUCTUS_MAX_SERVO_ADJUSTMENT;
    } else if (adjustment < -FLUCTUS_MAX_SERVO_ADJUSTMENT) {
        adjustment = -FLUCTUS_MAX_SERVO_ADJUSTMENT;
    }
    
    int32_t new_duty = (int32_t)current_duty + adjustment;
    
    // Clamp to servo limits
    if (new_duty < FLUCTUS_SERVO_MIN_DUTY) {
        new_duty = FLUCTUS_SERVO_MIN_DUTY;
    } else if (new_duty > FLUCTUS_SERVO_MAX_DUTY) {
        new_duty = FLUCTUS_SERVO_MAX_DUTY;
    }
    
    return (uint32_t)new_duty;
}

/**
 * @brief Apply servo corrections based on tracking data
 * @param tracking_data Tracking data with yaw/pitch errors
 * @return true if servos were updated
 */
bool fluctus_apply_servo_corrections(fluctus_solar_snapshot_t *tracking_data)
{
    uint32_t new_yaw_duty = fluctus_calculate_servo_correction(tracking_data->yaw_error,
                                                    system_status.current_yaw_duty);
    uint32_t new_pitch_duty = fluctus_calculate_servo_correction(tracking_data->pitch_error,
                                                      system_status.current_pitch_duty);

    bool updated = false;
    if (new_yaw_duty != system_status.current_yaw_duty) {
        if (fluctus_servo_set_position(FLUCTUS_SERVO_YAW_CHANNEL, new_yaw_duty) == ESP_OK) {
            system_status.current_yaw_duty = new_yaw_duty;
            updated = true;
        }
    }

    if (new_pitch_duty != system_status.current_pitch_duty) {
        if (fluctus_servo_set_position(FLUCTUS_SERVO_PITCH_CHANNEL, new_pitch_duty) == ESP_OK) {
            system_status.current_pitch_duty = new_pitch_duty;
            updated = true;
        }
    }

    if (updated) {
        ESP_LOGD(TAG, "Correcting - Yaw: %lu, Pitch: %lu, Errors: %.3fV/%.3fV",
                system_status.current_yaw_duty, system_status.current_pitch_duty,
                tracking_data->yaw_error, tracking_data->pitch_error);
    }

    return updated;
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
 * Updates system_status with final servo positions.
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
        system_status.current_yaw_duty = yaw_duty;
        system_status.current_pitch_duty = pitch_duty;
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
 * Updates system_status with final servo positions.
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
        system_status.current_yaw_duty = yaw_duty;
        system_status.current_pitch_duty = pitch_duty;
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
 * IMPORTANT: Updates global cached_solar_data with fresh photoresistor readings.
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
    fluctus_solar_snapshot_t temp_data = {0};
    if (fluctus_read_photoresistors(&temp_data) != ESP_OK || !temp_data.valid) {
        return false;
    }

    // Update global cache with fresh photoresistor data + current servo state
    if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        // Copy photoresistor data
        memcpy(cached_solar_data.photoresistor_readings, temp_data.photoresistor_readings, sizeof(cached_solar_data.photoresistor_readings));
        cached_solar_data.yaw_error = temp_data.yaw_error;
        cached_solar_data.pitch_error = temp_data.pitch_error;
        cached_solar_data.valid = temp_data.valid;
        cached_solar_data.timestamp = temp_data.timestamp;

        // Add current servo positions and state
        cached_solar_data.tracking_state = system_status.solar_tracking_state;
        cached_solar_data.current_yaw_duty = system_status.current_yaw_duty;
        cached_solar_data.current_pitch_duty = system_status.current_pitch_duty;
        cached_solar_data.yaw_position_percent = (uint8_t)fluctus_duty_to_percent(system_status.current_yaw_duty);
        cached_solar_data.pitch_position_percent = (uint8_t)fluctus_duty_to_percent(system_status.current_pitch_duty);

        xSemaphoreGive(xSolarMutex);
    }

    // Calculate average light and update STELLARIA
    float avg_light = (temp_data.photoresistor_readings[0] +
                      temp_data.photoresistor_readings[1] +
                      temp_data.photoresistor_readings[2] +
                      temp_data.photoresistor_readings[3]) / 4.0f;
    stellaria_update_light_intensity(avg_light);

    // Combined decision: astronomical daytime AND sufficient light detected
    return (avg_light >= FLUCTUS_PV_LIGHT_THRESHOLD);
}

/**
 * @brief Thread-safe state transition with logging
 * @param new_state Target state
 * @param log_msg Log message (NULL to skip logging)
 */
void fluctus_set_tracking_state(solar_tracking_state_t new_state, const char *log_msg)
{
    if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        system_status.solar_tracking_state = new_state;
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
 *
 * @param current_time_ms Current time in milliseconds (for correction interval timing)
 */
void fluctus_solar_state_standby(int64_t current_time_ms)
{
    // Check for sunset parking condition using centralized environmental check
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

    // Check if we should start correction cycle (from config)
    if (last_correction_time == 0 ||
        (current_time_ms - last_correction_time) >= fluctus_solar_correction_interval_ms) {

        // Request 6.6V servo bus power for correction cycle
        if (fluctus_request_bus_power(POWER_BUS_6V6, "FLUCTUS_SOLAR") == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(FLUCTUS_SERVO_POWERUP_DELAY_MS));
            correction_start_time = current_time_ms;
            last_correction_time = current_time_ms;
            fluctus_set_tracking_state(SOLAR_TRACKING_CORRECTING, "Starting correction cycle (6.6V bus powered on)");
        } else {
            ESP_LOGE(TAG, "Failed to power 6.6V bus for solar tracking");
        }
    }
}

/**
 * @brief Handle CORRECTING state - continuous servo adjustments
 *
 * Runs every 5 seconds during active tracking to adjust servo positions.
 * Checks for sunset, convergence, and timeout conditions.
 *
 * OPTIMIZATION: Uses cached photoresistor data from fluctus_is_daytime_with_sufficient_light()
 * to avoid redundant ADC reads (reduces correction cycle time by ~1s).
 *
 * @param current_time_ms Current time in milliseconds (for timeout check)
 */
void fluctus_solar_state_correcting(int64_t current_time_ms)
{
    // Check if sunset occurred - abort correction and park
    // NOTE: This call updates cached_solar_data with fresh photoresistor readings
    if (!fluctus_is_daytime_with_sufficient_light()) {
        fluctus_release_bus_power(POWER_BUS_6V6, "FLUCTUS_SOLAR");

        // Insufficient light detected - determine if it's true sunset or just cloudy
        if (solar_calc_is_daytime_buffered()) {
            // Cloudy during correction - abort but stay in STANDBY
            fluctus_set_tracking_state(SOLAR_TRACKING_STANDBY, "Insufficient light during correction - aborting cycle");
        } else {
            // True sunset during correction
            current_parking_reason = PARKING_REASON_SUNSET;
            fluctus_set_tracking_state(SOLAR_TRACKING_PARKING, "Sunset during correction - aborting");
        }
        return;
    }

    // Use cached photoresistor data (just updated by daytime check above)
    fluctus_solar_snapshot_t tracking_data;
    if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        memcpy(&tracking_data, &cached_solar_data, sizeof(fluctus_solar_snapshot_t));
        xSemaphoreGive(xSolarMutex);
    } else {
        ESP_LOGW(TAG, "Failed to access cached solar data during correction");
        return;
    }

    if (!tracking_data.valid) {
        ESP_LOGW(TAG, "Cached photoresistor data invalid during correction");
        return;
    }

    // NOTE: No need to explicitly update TELEMETRY here - the main cache is updated
    // via cached_solar_data which is populated by fluctus_is_daytime_with_sufficient_light()
    // The regular telemetry_update_fluctus() at line 1518 will pick up the latest cached data

    // Check if errors are acceptable (correction complete)
    if (fluctus_tracking_error_margin_check(tracking_data.yaw_error, tracking_data.pitch_error)) {
        fluctus_release_bus_power(POWER_BUS_6V6, "FLUCTUS_SOLAR");
        consecutive_tracking_errors = 0;  // Reset error counter on successful correction
        fluctus_set_tracking_state(SOLAR_TRACKING_STANDBY, "Correction complete - errors within threshold (6.6V bus powered off)");
        return;
    }

    // Check for timeout
    int64_t correction_duration = current_time_ms - correction_start_time;
    if (correction_duration >= FLUCTUS_TRACKING_CORRECTION_TIMEOUT_MS) {
        ESP_LOGW(TAG, "Correction timeout after %lld ms - entering error state", correction_duration);
        fluctus_set_tracking_state(SOLAR_TRACKING_ERROR, NULL);
        return;
    }

    // Apply servo corrections
    fluctus_apply_servo_corrections(&tracking_data);
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

// ########################## Solar Tracking Main Task ##########################

/**
 * @brief Solar tracking task with notification-based state machine and automatic scheduling
 *
 * Uses task notifications for efficient power management:
 * - DISABLED: Wait indefinitely for ENABLE notification (no CPU usage)
 * - STANDBY: 15-minute intervals for correction cycles
 * - CORRECTING: 5-second updates during active tracking
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
        if (xSemaphoreTake(xSolarMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
            current_state = system_status.solar_tracking_state;

            // Safety check: Disable tracking if shutdown or critical power
            if (system_status.safety_shutdown ||
                system_status.power_state >= FLUCTUS_POWER_STATE_CRITICAL) {
                if (current_state != SOLAR_TRACKING_DISABLED) {
                    system_status.solar_tracking_state = SOLAR_TRACKING_DISABLED;
                    current_state = SOLAR_TRACKING_DISABLED;
                    ESP_LOGW(TAG, "Solar tracking disabled due to safety or power state");
                }
            }

            xSemaphoreGive(xSolarMutex);
        } else {
            continue;
        }

        // Determine timeout based on current state
        TickType_t timeout;
        switch (current_state) {
            case SOLAR_TRACKING_DISABLED:
            case SOLAR_TRACKING_SLEEPING:
                timeout = portMAX_DELAY;  // Wait indefinitely for notification
                break;
            case SOLAR_TRACKING_STANDBY:
                timeout = pdMS_TO_TICKS(fluctus_solar_correction_interval_ms);  // From config
                break;
            case SOLAR_TRACKING_CORRECTING:
                timeout = pdMS_TO_TICKS(FLUCTUS_TRACKING_CORRECTION_DURATION_MS);  // 5 seconds
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
                fluctus_solar_state_correcting(current_time_ms);
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
        state = system_status.solar_tracking_state;
        xSemaphoreGive(xSolarMutex);
    }

    return state;
}