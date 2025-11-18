/**
 * @file impluvium_actuators.c
 * @brief Actuator control for IMPLUVIUM irrigation system
 *
 * Handles GPIO initialization, PWM pump control, flow sensor initialization,
 * pump speed management, ramp-up sequences, and adaptive pump control.
 */

#include "impluvium.h"
#include "impluvium_private.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include <math.h>

static const char *TAG = "IMPLUVIUM_ACT";

/**
 * @brief Initialize valve GPIO pins
 */
esp_err_t impluvium_gpio_init(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    // Configure valve control pins
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        io_conf.pin_bit_mask = (1ULL << zone_valve_gpios[i]);
        esp_err_t ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure valve GPIO %d: %s", zone_valve_gpios[i], esp_err_to_name(ret));
            return ret;
        }

        // Ensure valve start closed
        gpio_set_level(zone_valve_gpios[i], 0);
    }

    ESP_LOGI(TAG,
             "Irrigation valve GPIOs initialized - Valve GPIO%d - GPIO%d",
             irrigation_zones[0].valve_gpio,
             irrigation_zones[IRRIGATION_ZONE_COUNT - 1].valve_gpio);
    return ESP_OK;
}

/**
 * @brief Initialize PWM for pump speed control
 */
esp_err_t impluvium_pump_init(void)
{
    // Config timer
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = PUMP_PWM_TIMER,
        .duty_resolution = PUMP_PWM_RESOLUTION,
        .freq_hz = PUMP_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure pump PWM timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure pump PWM channel
    ledc_channel_config_t channel_config = {
        .gpio_num = PUMP_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = PUMP_PWM_CHANNEL,
        .timer_sel = PUMP_PWM_TIMER,
        .duty = 0, // Start with pump off
        .hpoint = 0,
    };

    ret = ledc_channel_config(&channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure pump PWM channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Pump PWM initialized - GPIO42, 1kHz, 10bit");
    return ESP_OK;
}

/**
 * @brief Initialize flow sensor pulse counting
 */
esp_err_t impluvium_flow_sensor_init(void)
{
    // Create pulse counter unit configuration
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = -32768,
    };

    esp_err_t ret = pcnt_new_unit(&unit_config, &flow_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create pulse counter unit: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create pulse counter channel configuration
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = FLOW_SENSOR_GPIO,
        .level_gpio_num = -1, // Not used
    };

    pcnt_channel_handle_t flow_pcnt_chan = NULL;
    ret = pcnt_new_channel(flow_pcnt_unit, &chan_config, &flow_pcnt_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create pulse counter channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set edge action (count on positive edge only)
    ret =
        pcnt_channel_set_edge_action(flow_pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set edge action: %s", esp_err_to_name(ret));
        return ret;
    }

    // Enable and start counting
    ret = pcnt_unit_enable(flow_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable pulse counter: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_clear_count(flow_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear pulse counter: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_start(flow_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start pulse counter: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Flow sensor initialized - GPIO19, pulse counting");
    return ESP_OK;
}

/**
 * @brief Set pump PWM duty cycle with safety limits
 */
esp_err_t impluvium_set_pump_speed(uint32_t pwm_duty)
{
    // Enforce pump speed limits
    if (pwm_duty > 1023) {
        pwm_duty = PUMP_MAX_DUTY;
    }
    if (pwm_duty > 0 && pwm_duty < PUMP_MIN_DUTY) {
        ESP_LOGW(TAG, "Pump duty %" PRIu32 " below minimum %d , adjusting", pwm_duty, PUMP_MIN_DUTY);
        pwm_duty = PUMP_MIN_DUTY;
    }

    irrigation_system.pump_pwm_duty = pwm_duty;

    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, PUMP_PWM_CHANNEL, pwm_duty);
    if (ret == ESP_OK) {
        ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, PUMP_PWM_CHANNEL);
    }

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Pump speed set to %" PRIu32 " /1023 (%.1f%%)", pwm_duty, (pwm_duty / 1023.0f) * 100.0f);
    }

    return ret;
}

/**
 * @brief Ramps up the pump speed over a defined period.
 *
 * Linearly increases the pump's PWM duty cycle from a minimum value
 * to the zone's learned optimal duty cycle. This gradual start-up
 * reduces mechanical stress on the pump and pipes.
 *
 * @param zone_id The ID of the zone being watered.
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t impluvium_pump_ramp_up(uint8_t zone_id)
{
    ESP_LOGI(TAG, "Ramping up pump for zone %d...", zone_id);

    uint32_t start_duty = PUMP_MIN_DUTY;
    uint32_t target_duty = irrigation_zones[zone_id].learning.calculated_pump_duty_cycle;
    uint32_t ramp_duration_ms = PUMP_RAMP_UP_TIME_MS;
    int steps = 50; // 50 steps for a smooth ramp
    uint32_t step_delay = ramp_duration_ms / steps;

    for (int i = 0; i <= steps; i++) {
        uint32_t duty = start_duty + ((target_duty - start_duty) * i) / steps;
        if (impluvium_set_pump_speed(duty) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set pump speed during ramp-up");
            return ESP_FAIL;
        }
        vTaskDelay(pdMS_TO_TICKS(step_delay));
    }

    ESP_LOGI(TAG, "Pump ramp-up complete for zone %d at %" PRIu32 " duty.", zone_id, target_duty);
    return ESP_OK;
}

/**
 * @brief Simple adaptive pump control based on moisture gain rate
 *
 * Adjusts pump speed up or down based on whether we're meeting the target moisture gain rate.
 *
 * @param current_gain_rate Current moisture gain rate (%/sec)
 * @param target_gain_rate Target moisture gain rate (%/sec)
 */
void impluvium_pump_adaptive_control(float current_gain_rate, float target_gain_rate)
{
    float rate_error = target_gain_rate - current_gain_rate;

    // Only adjust if error is significant
    if (fabs(rate_error) < PUMP_GAIN_RATE_TOLERANCE) {
        return; // Close enough, no adjustment needed
    }

    uint32_t current_duty = irrigation_system.pump_pwm_duty;
    uint32_t new_duty = current_duty;

    if (rate_error > 0) {
        // Need more moisture gain - increase pump speed
        new_duty = current_duty + PUMP_ADJUSTMENT_STEP;
        ESP_LOGD(TAG, "Increasing pump speed: %.2f < %.2f %%/sec", current_gain_rate, target_gain_rate);
    } else {
        // Too much moisture gain - decrease pump speed
        new_duty = current_duty - (PUMP_ADJUSTMENT_STEP / 2); // Decrease more slowly
        ESP_LOGD(TAG, "Decreasing pump speed: %.2f > %.2f %%/sec", current_gain_rate, target_gain_rate);
    }

    impluvium_set_pump_speed(new_duty);
}
