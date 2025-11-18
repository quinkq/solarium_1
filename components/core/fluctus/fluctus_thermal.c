/**
 * @file fluctus_thermal.c
 * @brief FLUCTUS thermal management module
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Handles thermal monitoring and cooling for the FLUCTUS power system:
 * - DS18B20 temperature sensor management (asynchronous, non-blocking)
 * - Cooling fan PWM control (25kHz, 4-pin Intel cooler)
 * - Hysteresis-based fan control (30°C ON / 28°C OFF)
 * - Proportional fan speed (10% @ 30°C, 100% @ 40°C)
 * - Thermal event monitoring (1-min intervals during thermal events)
 *
 * State machine approach prevents blocking the monitoring task during
 * DS18B20 conversion (750ms @ 12-bit resolution).
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "fluctus.h"
#include "fluctus_private.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "FLUCTUS_THERMAL";

// DS18B20 temperature sensor variables
onewire_addr_t ds18b20_addr = ONEWIRE_NONE;
bool ds18b20_found = false;
int64_t last_temp_read_time = 0;

temp_conversion_state_t temp_state = TEMP_STATE_IDLE;
int64_t temp_conversion_start_time = 0;

/**
 * @brief Initialize DS18B20 temperature sensor
 */
esp_err_t fluctus_ds18b20_init(void)
{
    ESP_LOGI(TAG, "Initializing DS18B20 temperature sensor...");

    // Search for DS18B20 device on the bus (family code 0x28)
    onewire_search_t search;
    onewire_search_start(&search);
    onewire_search_prefix(&search, 0x28);  // DS18B20 family code

    ds18b20_addr = onewire_search_next(&search, FLUCTUS_DS18B20_GPIO);

    if (ds18b20_addr == ONEWIRE_NONE) {
        ESP_LOGW(TAG, "DS18B20 not found on GPIO%d", FLUCTUS_DS18B20_GPIO);
        ds18b20_found = false;
        return ESP_ERR_NOT_FOUND;
    }

    // Verify CRC
    uint8_t addr_bytes[8];
    memcpy(addr_bytes, &ds18b20_addr, 8);
    uint8_t crc = onewire_crc8(addr_bytes, 7);

    if (crc != addr_bytes[7]) {
        ESP_LOGE(TAG, "DS18B20 CRC mismatch - sensor may be faulty");
        ds18b20_found = false;
        return ESP_FAIL;
    }

    ds18b20_found = true;
    ESP_LOGI(TAG, "DS18B20 found at address 0x%llx", ds18b20_addr);

    return ESP_OK;
}

/**
 * @brief Initialize cooling fan PWM
 */
esp_err_t fluctus_fan_pwm_init(void)
{
    ESP_LOGI(TAG, "Initializing cooling fan PWM...");
    
    // Configure fan timer
    ledc_timer_config_t fan_timer_conf = {
        .speed_mode = FLUCTUS_SERVO_PWM_SPEED_MODE,
        .duty_resolution = FLUCTUS_FAN_PWM_RESOLUTION,
        .timer_num = FLUCTUS_FAN_PWM_TIMER,
        .freq_hz = FLUCTUS_FAN_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    esp_err_t ret = ledc_timer_config(&fan_timer_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure fan PWM timer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure fan channel
    ledc_channel_config_t fan_channel_conf = {
        .gpio_num = FLUCTUS_FAN_PWM_GPIO,
        .speed_mode = FLUCTUS_SERVO_PWM_SPEED_MODE,
        .channel = FLUCTUS_FAN_PWM_CHANNEL,
        .timer_sel = FLUCTUS_FAN_PWM_TIMER,
        .duty = 0,  // Start with fan off
        .hpoint = 0,
        .flags.output_invert = 1
    };
    
    ret = ledc_channel_config(&fan_channel_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure fan PWM channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Cooling fan PWM initialized");
    return ESP_OK;
}


/**
 * @brief Update cooling fan speed based on temperature
 * @param temperature Current case temperature in °C
 */
void fluctus_update_fan_speed(float temperature)
{
    bool should_run = false;
    uint8_t fan_duty = 0;

    // Hysteresis logic
    if (monitoring_data.fan_active) {
        // Fan is currently on - turn off at 28°C
        should_run = (temperature >= FLUCTUS_FAN_TURN_OFF_TEMP_THRESHOLD);
    } else {
        // Fan is currently off - turn on at 30°C
        should_run = (temperature >= FLUCTUS_FAN_TURN_ON_TEMP_THRESHOLD);
    }

    if (should_run) {
        // Calculate fan duty cycle: 20% at 30°C, 100% at 40°C
        if (temperature >= FLUCTUS_FAN_MAX_TEMP_THRESHOLD) {
            fan_duty = FLUCTUS_FAN_MAX_DUTY;  // 100%
        } else if (temperature <= FLUCTUS_FAN_TURN_ON_TEMP_THRESHOLD) {
            fan_duty = FLUCTUS_FAN_MIN_DUTY;  // 10%
        } else {
            // Linear interpolation between 10% and 100%
            float temp_range = FLUCTUS_FAN_MAX_TEMP_THRESHOLD - FLUCTUS_FAN_TURN_ON_TEMP_THRESHOLD;
            float duty_range = FLUCTUS_FAN_MAX_DUTY - FLUCTUS_FAN_MIN_DUTY;
            float temp_offset = temperature - FLUCTUS_FAN_TURN_ON_TEMP_THRESHOLD;
            fan_duty = FLUCTUS_FAN_MIN_DUTY + (uint8_t)((temp_offset / temp_range) * duty_range);
        }

        // Request 12V bus power if not already active
        if (!monitoring_data.fan_active) {
            esp_err_t ret = fluctus_request_bus_power(POWER_BUS_12V, "FLUCTUS_FAN");
            if (ret == ESP_OK) {
                gpio_set_level(FLUCTUS_12V_FAN_ENABLE_GPIO, 1);
                monitoring_data.fan_active = true;
                monitoring_data.thermal_monitoring_active = true;
                ESP_LOGI(TAG, "Cooling fan activated at %.1f°C", temperature);
            } else {
                ESP_LOGE(TAG, "Failed to power 12V bus for cooling fan");
                return;
            }
        }

        // Set fan speed
        fluctus_set_cooling_fan_speed(fan_duty);
        ESP_LOGD(TAG, "Fan speed: %d%% (temp: %.1f°C)", fan_duty, temperature);

    } else {
        // Turn off fan
        if (monitoring_data.fan_active) {
            fluctus_set_cooling_fan_speed(0);
            gpio_set_level(FLUCTUS_12V_FAN_ENABLE_GPIO, 0);
            fluctus_release_bus_power(POWER_BUS_12V, "FLUCTUS_FAN");
            monitoring_data.fan_active = false;
            ESP_LOGI(TAG, "Cooling fan deactivated at %.1f°C", temperature);

            // Check if we should exit thermal monitoring mode
            if (temperature < FLUCTUS_FAN_TURN_OFF_TEMP_THRESHOLD) {
                monitoring_data.thermal_monitoring_active = false;
                ESP_LOGD(TAG, "Exiting thermal monitoring mode");
            }
        }
    }
}

/**
 * @brief Determine if temperature should be read now
 * @param monitoring_active Whether power monitoring is active
 * @return true if temperature should be read
 */
bool fluctus_should_read_temperature(bool monitoring_active)
{
    int64_t current_time_ms = esp_timer_get_time() / 1000;
    int64_t time_since_last_read = current_time_ms - last_temp_read_time;

    if (monitoring_data.thermal_monitoring_active) {
        // During thermal event: read every 1 minute
        return (time_since_last_read >= FLUCTUS_TEMP_THERMAL_INTERVAL_MS);
    } else if (monitoring_active) {
        // During active power monitoring: read every 5 seconds
        return (time_since_last_read >= FLUCTUS_TEMP_ACTIVE_INTERVAL_MS);
    }

    // Otherwise: opportunistic reads when 3.3V bus is powered
    return fluctus_is_bus_powered(POWER_BUS_3V3);
}

/**
 * @brief Handle temperature monitoring with non-blocking async conversion
 *
 * Uses ds18x20 library with async conversion to avoid blocking the monitoring task.
 * State machine approach:
 * - IDLE: Check if temp should be read, start conversion if needed
 * - CONVERTING: Check if 750ms elapsed, read result if ready
 *
 * @param monitoring_active Whether power monitoring is currently active
 */
void fluctus_handle_temperature_monitoring(bool monitoring_active)
{
    if (!ds18b20_found) {
        return;
    }

    int64_t current_time_ms = esp_timer_get_time() / 1000;

    switch (temp_state) {
        case TEMP_STATE_IDLE:
            // Check if we should start a new temperature conversion
            if (fluctus_should_read_temperature(monitoring_active)) {
                bool bus_was_off = false;

                // Request 3.3V bus power if needed for thermal monitoring
                if (monitoring_data.thermal_monitoring_active && !fluctus_is_bus_powered(POWER_BUS_3V3)) {
                    if (fluctus_request_bus_power(POWER_BUS_3V3, "FLUCTUS_TEMP") == ESP_OK) {
                        bus_was_off = true;
                        vTaskDelay(pdMS_TO_TICKS(100)); // Allow bus to stabilize
                    }
                }

                // Start conversion WITHOUT blocking (wait=false)
                if (ds18x20_measure(FLUCTUS_DS18B20_GPIO, ds18b20_addr, false) == ESP_OK) {
                    temp_state = TEMP_STATE_CONVERTING;
                    temp_conversion_start_time = current_time_ms;
                    ESP_LOGD(TAG, "DS18B20 conversion started (non-blocking, ~750ms)");
                } else {
                    ESP_LOGW(TAG, "Failed to start DS18B20 conversion");
                    monitoring_data.temperature_valid = false;
                }

                // Release 3.3V bus if we requested it
                if (bus_was_off) {
                    fluctus_release_bus_power(POWER_BUS_3V3, "FLUCTUS_TEMP");
                }
            }
            break;

        case TEMP_STATE_CONVERTING:
            // Check if conversion is complete (750ms for 12-bit resolution)
            if ((current_time_ms - temp_conversion_start_time) >= FLUCTUS_TEMP_CONVERSION_DELAY_MS) {
                float temperature;

                // Read the conversion result
                if (ds18b20_read_temperature(FLUCTUS_DS18B20_GPIO, ds18b20_addr, &temperature) == ESP_OK) {
                    monitoring_data.case_temperature = temperature;
                    monitoring_data.temperature_valid = true;
                    monitoring_data.temperature_timestamp = time(NULL);
                    last_temp_read_time = current_time_ms;

                    // Update fan speed based on temperature
                    fluctus_update_fan_speed(temperature);

                    ESP_LOGD(TAG, "DS18B20: %.2f°C", temperature);
                } else {
                    ESP_LOGW(TAG, "Failed to read DS18B20 temperature result");
                    monitoring_data.temperature_valid = false;
                }

                // Return to idle state
                temp_state = TEMP_STATE_IDLE;
            }
            // else: Still converting, check again on next monitoring cycle
            break;
    }
}

// Public API


esp_err_t fluctus_set_cooling_fan_speed(uint8_t duty_percent)
{
    if (!fluctus_initialized || duty_percent > 100) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Convert percentage to duty cycle (8-bit resolution = 256 levels)
    uint32_t duty_cycle = (duty_percent * 255) / 100;
    
    esp_err_t ret = ledc_set_duty(FLUCTUS_SERVO_PWM_SPEED_MODE, FLUCTUS_FAN_PWM_CHANNEL, duty_cycle);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = ledc_update_duty(FLUCTUS_SERVO_PWM_SPEED_MODE, FLUCTUS_FAN_PWM_CHANNEL);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Cooling fan speed set to %d%%", duty_percent);
    }
    
    return ret;
}
