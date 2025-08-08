#include <stdio.h>
#include <string.h>

#include "stellaria.h"
#include "fluctus.h"
#include "esp_log.h"


// ########################## Constants and Variables ##########################

static const char *TAG = "STELLARIA";

// Global status structure
static stellaria_status_t stellaria_status = {
    .state = STELLARIA_STATE_DISABLED,
    .current_intensity = 0,
    .target_intensity = 0,
    .driver_enabled = false,
    .initialized = false,
    .auto_mode_active = false,
    .last_light_reading = -1.0f
};

// Previous state for shutdown recovery
static stellaria_state_t previous_state = STELLARIA_STATE_DISABLED;
static uint16_t previous_intensity = 0;

// Mutex for thread-safe operations
static SemaphoreHandle_t xStellariaMutex = NULL;

// Auto mode variables
static uint16_t auto_mode_intensity = STELLARIA_MAX_INTENSITY;

// ########################## Private Function Declarations ##########################

static esp_err_t stellaria_gpio_init(void);
static esp_err_t stellaria_pwm_init(void);
static esp_err_t stellaria_update_hardware(void);
static uint16_t stellaria_clamp_intensity(uint16_t intensity, stellaria_state_t state);

// ########################## Private Functions ##########################

/**
 * @brief Initialize GPIO pins for STELLARIA
 */
static esp_err_t stellaria_gpio_init(void)
{
    // Configure enable GPIO as output
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << STELLARIA_ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&gpio_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure enable GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set initial state to disabled
    gpio_set_level(STELLARIA_ENABLE_GPIO, 0);
    
    ESP_LOGI(TAG, "GPIO initialized - Enable: GPIO%d", STELLARIA_ENABLE_GPIO);
    return ESP_OK;
}

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
    
    ESP_LOGI(TAG, "PWM initialized - GPIO%d, %dHz, %d-bit", 
             STELLARIA_PWM_GPIO, STELLARIA_PWM_FREQUENCY, 
             1 << STELLARIA_PWM_RESOLUTION);
    return ESP_OK;
}

/**
 * @brief Update hardware based on current status
 */
static esp_err_t stellaria_update_hardware(void)
{
    // Update enable GPIO
    gpio_set_level(STELLARIA_ENABLE_GPIO, stellaria_status.driver_enabled ? 1 : 0);
    
    // Update PWM duty cycle
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
 * @brief Clamp intensity based on current state
 */
static uint16_t stellaria_clamp_intensity(uint16_t intensity, stellaria_state_t state)
{
    // Clamp to absolute limits first
    if (intensity > STELLARIA_MAX_INTENSITY) {
        intensity = STELLARIA_MAX_INTENSITY;
    }
    
    // Apply state-specific limits
    switch (state) {
        case STELLARIA_STATE_DISABLED:
        case STELLARIA_STATE_SHUTDOWN:
            return 0;
            
        case STELLARIA_STATE_POWER_SAVE:
            return (intensity > STELLARIA_POWER_SAVE_LIMIT) ? STELLARIA_POWER_SAVE_LIMIT : intensity;
            
        case STELLARIA_STATE_ENABLED:
        default:
            return intensity;
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
    
    // Initialize GPIO
    esp_err_t ret = stellaria_gpio_init();
    if (ret != ESP_OK) {
        vSemaphoreDelete(xStellariaMutex);
        xStellariaMutex = NULL;
        return ret;
    }
    
    // Initialize PWM
    ret = stellaria_pwm_init();
    if (ret != ESP_OK) {
        vSemaphoreDelete(xStellariaMutex);
        xStellariaMutex = NULL;
        return ret;
    }
    
    // Set initial status
    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        stellaria_status.state = STELLARIA_STATE_DISABLED;
        stellaria_status.current_intensity = 0;
        stellaria_status.target_intensity = 0;
        stellaria_status.driver_enabled = false;
        stellaria_status.initialized = true;
        
        // Update hardware to match status
        stellaria_update_hardware();
        
        xSemaphoreGive(xStellariaMutex);
    } else {
        ESP_LOGE(TAG, "Failed to acquire mutex during initialization");
        vSemaphoreDelete(xStellariaMutex);
        xStellariaMutex = NULL;
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "STELLARIA initialization complete");
    return ESP_OK;
}

esp_err_t stellaria_set_intensity(uint16_t intensity)
{
    if (!stellaria_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        stellaria_status.target_intensity = intensity;
        stellaria_status.current_intensity = stellaria_clamp_intensity(intensity, stellaria_status.state);
        
        esp_err_t ret = stellaria_update_hardware();
        
        xSemaphoreGive(xStellariaMutex);
        
        ESP_LOGI(TAG, "Intensity set to %d (target: %d)", 
                 stellaria_status.current_intensity, stellaria_status.target_intensity);
        return ret;
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
        
        esp_err_t ret = stellaria_update_hardware();
        
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
        
        esp_err_t ret = stellaria_update_hardware();
        
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
                stellaria_status.current_intensity = stellaria_clamp_intensity(
                    stellaria_status.target_intensity, stellaria_status.state);
                ESP_LOGI(TAG, "Power save mode enabled (intensity limited to 20%%)");
            }
        } else {
            if (stellaria_status.state == STELLARIA_STATE_POWER_SAVE) {
                stellaria_status.state = STELLARIA_STATE_ENABLED;
                stellaria_status.current_intensity = stellaria_clamp_intensity(
                    stellaria_status.target_intensity, stellaria_status.state);
                ESP_LOGI(TAG, "Power save mode disabled");
            }
        }
        
        esp_err_t ret = stellaria_update_hardware();
        
        xSemaphoreGive(xStellariaMutex);
        return ret;
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
        
        esp_err_t ret = stellaria_update_hardware();
        
        xSemaphoreGive(xStellariaMutex);
        return ret;
    }
    
    return ESP_FAIL;
}

esp_err_t stellaria_get_status(stellaria_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!stellaria_status.initialized) {
        memset(status, 0, sizeof(stellaria_status_t));
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        memcpy(status, &stellaria_status, sizeof(stellaria_status_t));
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
 * @brief Enable automatic light sensing mode
 */
esp_err_t stellaria_set_auto_mode(uint16_t intensity_when_on)
{
    if (!stellaria_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(xStellariaMutex, pdMS_TO_TICKS(STELLARIA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        stellaria_status.state = STELLARIA_STATE_AUTO;
        stellaria_status.auto_mode_active = true;
        auto_mode_intensity = (intensity_when_on <= STELLARIA_MAX_INTENSITY) ? intensity_when_on : STELLARIA_MAX_INTENSITY;
        
        ESP_LOGI(TAG, "Auto light sensing mode enabled (intensity when on: %d)", auto_mode_intensity);
        
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
            stellaria_status.target_intensity = auto_mode_intensity;
            stellaria_status.current_intensity = stellaria_clamp_intensity(auto_mode_intensity, STELLARIA_STATE_AUTO);
            stellaria_update_hardware();
        } else if (!should_be_on && currently_on) {
            ESP_LOGI(TAG, "Auto mode: Turning OFF (%.3fV < %.3fV - bright)", 
                     averaged_light_voltage, STELLARIA_LIGHT_TURN_OFF_THRESHOLD);
            stellaria_status.target_intensity = 0;
            stellaria_status.current_intensity = 0;
            stellaria_update_hardware();
        }
        
        xSemaphoreGive(xStellariaMutex);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

