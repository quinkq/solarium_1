#include "fluctus.h"
#include "stellaria.h"
#include "irrigation.h"
#include "weather_station.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>
#include <time.h>
#include "ads111x.h"
#include "ads1115_helper.h"

// ########################## Constants and Variables ##########################

static const char *TAG = "FLUCTUS";

// Global system status
static fluctus_power_status_t system_status = {
    .bus_enabled = {false, false, false, false},
    .bus_ref_count = {0, 0, 0, 0},
    .power_state = FLUCTUS_POWER_STATE_NORMAL,
    .solar_tracking_enabled = false,
    .current_yaw_duty = FLUCTUS_SERVO_CENTER_DUTY,
    .current_pitch_duty = FLUCTUS_SERVO_CENTER_DUTY,
    .safety_shutdown = false,
    .manual_reset_required = false,
    .last_activity_time = 0
};

// Power monitoring data
static fluctus_monitoring_data_t monitoring_data = {0};

// INA219 device configurations
static ina219_t ina219_devices[FLUCTUS_INA219_DEVICE_COUNT];

// ADS1115 photoresistor readings using ads1115_helper component
// Device 0 contains photoresistors on channels 0-3

// Mutexes for thread safety
static SemaphoreHandle_t xFluctusPowerMutex = NULL;
static SemaphoreHandle_t xFluctusMonitoringMutex = NULL;

// Task handles
static TaskHandle_t xFluctusMonitoringTaskHandle = NULL;
static TaskHandle_t xFluctusSolarTrackingTaskHandle = NULL;

// Overcurrent protection variables
static int64_t overcurrent_start_time = 0;
static bool overcurrent_timer_active = false;

// System initialization flag
static bool fluctus_initialized = false;

// ########################## Private Function Declarations ##########################

// Initialization functions
static esp_err_t fluctus_gpio_init(void);
static esp_err_t fluctus_ina219_init(void);
static esp_err_t fluctus_servo_pwm_init(void);
static esp_err_t fluctus_fan_pwm_init(void);

// Power management functions
static esp_err_t fluctus_update_bus_hardware(power_bus_t bus);
static esp_err_t fluctus_add_consumer(power_bus_t bus, const char* consumer_id);
static esp_err_t fluctus_remove_consumer(power_bus_t bus, const char* consumer_id);
static void fluctus_check_power_state(void);
static void fluctus_handle_power_state_change(fluctus_power_state_t new_state);

// Solar tracking functions
static esp_err_t fluctus_read_photoresistors(fluctus_solar_tracking_data_t *data);
static esp_err_t fluctus_servo_set_position(ledc_channel_t channel, uint32_t duty_cycle);
static uint32_t fluctus_calc_servo_adjust(float error, uint32_t current_duty);
static esp_err_t fluctus_park_servos(void);

// Monitoring functions  
static esp_err_t fluctus_read_ina219_sensors(void);
static void fluctus_check_overcurrent(void);
static void fluctus_check_3v3_bus_voltage(void);
static void fluctus_log_active_consumers(const char* event_context);
static void fluctus_log_power_event(fluctus_event_type_t event_type, power_bus_t bus, 
                                   const char* consumer_id);

// FreeRTOS tasks
static void fluctus_monitoring_task(void *parameters);
static void fluctus_solar_tracking_task(void *parameters);

// ########################## Private Functions ##########################

/**
 * @brief Initialize GPIO pins for power control
 */
static esp_err_t fluctus_gpio_init(void)
{
    ESP_LOGI(TAG, "Initializing power control GPIOs...");
    
    // Configure buck converter enable pins (open-drain with pullups)
    gpio_config_t buck_gpio_conf = {
        .pin_bit_mask = (1ULL << FLUCTUS_BUCK_3V3_ENABLE_GPIO) |
                       (1ULL << FLUCTUS_BUCK_5V_ENABLE_GPIO) |
                       (1ULL << FLUCTUS_BUCK_6V2_ENABLE_GPIO) |
                       (1ULL << FLUCTUS_BUCK_12V_ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT_OD,  // Open-drain
        .pull_up_en = GPIO_PULLUP_DISABLE,  // External pullups
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&buck_gpio_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure buck enable GPIOs: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set all buses to disabled state initially (float = high via pullup = disabled)
    for (int i = 0; i < POWER_BUS_COUNT; i++) {
        switch (i) {
            case POWER_BUS_3V3:
                gpio_set_level(FLUCTUS_BUCK_3V3_ENABLE_GPIO, 1); // Float high = disabled
                break;
            case POWER_BUS_5V:
                gpio_set_level(FLUCTUS_BUCK_5V_ENABLE_GPIO, 1);
                break;
            case POWER_BUS_6V2:
                gpio_set_level(FLUCTUS_BUCK_6V2_ENABLE_GPIO, 1);
                break;
            case POWER_BUS_12V:
                gpio_set_level(FLUCTUS_BUCK_12V_ENABLE_GPIO, 1);
                break;
        }
    }
    
    ESP_LOGI(TAG, "Power control GPIOs initialized (all buses disabled)");
    return ESP_OK;
}

/**
 * @brief Initialize INA219 power monitoring sensors
 */
static esp_err_t fluctus_ina219_init(void)
{
    ESP_LOGI(TAG, "Initializing INA219 power monitoring sensors...");
    
    // Initialize "Solar PV" sensor (0x40) descriptor
    
    esp_err_t ret = ina219_init_desc(&ina219_devices[0], FLUCTUS_INA219_SOLAR_PV_ADDR, 
                                     I2C_NUM_0, CONFIG_I2CDEV_DEFAULT_SDA_PIN, CONFIG_I2CDEV_DEFAULT_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Solar PV INA219: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = ina219_init(&ina219_devices[0]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Solar PV INA219: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure Solar PV INA219
    ret = ina219_configure(&ina219_devices[0], INA219_BUS_RANGE_32V, INA219_GAIN_0_5,
                          INA219_RES_12BIT_8S, INA219_RES_12BIT_8S, INA219_MODE_CONT_SHUNT_BUS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Solar PV INA219: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Calibrate with 0.1 ohm shunt for Solar PV
    ret = ina219_calibrate(&ina219_devices[0], 0.1f);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate Solar PV INA219: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize "Battery Output" sensor (0x41) descriptor
    
    ret = ina219_init_desc(&ina219_devices[1], FLUCTUS_INA219_BATTERY_OUT_ADDR,
                          I2C_NUM_0, CONFIG_I2CDEV_DEFAULT_SDA_PIN, CONFIG_I2CDEV_DEFAULT_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Battery Output INA219: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = ina219_init(&ina219_devices[1]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Battery Output INA219: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure Battery Output INA219 (higher current range)
    ret = ina219_configure(&ina219_devices[1], INA219_BUS_RANGE_32V, INA219_GAIN_0_125,
                          INA219_RES_12BIT_8S, INA219_RES_12BIT_8S, INA219_MODE_CONT_SHUNT_BUS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Battery Output INA219: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Calibrate with 0.01 ohm shunt for Battery Output (higher current range)
    ret = ina219_calibrate(&ina219_devices[1], 0.01f);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate Battery Output INA219: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "INA219 sensors initialized successfully");
    return ESP_OK;
}

/**
 * @brief Initialize servo PWM for solar tracking
 */
static esp_err_t fluctus_servo_pwm_init(void)
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
        .gpio_num = FLUCTUS_SERVO_YAW_GPIO,
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
        .gpio_num = FLUCTUS_SERVO_PITCH_GPIO,
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

/**
 * @brief Initialize cooling fan PWM
 */
static esp_err_t fluctus_fan_pwm_init(void)
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
        .gpio_num = FLUCTUS_12V_FAN_PWM_GPIO,
        .speed_mode = FLUCTUS_SERVO_PWM_SPEED_MODE,
        .channel = FLUCTUS_FAN_PWM_CHANNEL,
        .timer_sel = FLUCTUS_FAN_PWM_TIMER,
        .duty = 0,  // Start with fan off
        .hpoint = 0
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
 * @brief Update hardware state for specific power bus
 */
static esp_err_t fluctus_update_bus_hardware(power_bus_t bus)
{
    if (bus >= POWER_BUS_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    bool enable = system_status.bus_enabled[bus] && !system_status.safety_shutdown;
    int gpio_level = enable ? 0 : 1;  // 0 = enabled (low), 1 = disabled (float high)
    
    switch (bus) {
        case POWER_BUS_3V3:
            gpio_set_level(FLUCTUS_BUCK_3V3_ENABLE_GPIO, gpio_level);
            break;
        case POWER_BUS_5V:
            gpio_set_level(FLUCTUS_BUCK_5V_ENABLE_GPIO, gpio_level);
            break;
        case POWER_BUS_6V2:
            gpio_set_level(FLUCTUS_BUCK_6V2_ENABLE_GPIO, gpio_level);
            break;
        case POWER_BUS_12V:
            gpio_set_level(FLUCTUS_BUCK_12V_ENABLE_GPIO, gpio_level);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "Bus %d hardware updated: %s", bus, enable ? "ENABLED" : "DISABLED");
    return ESP_OK;
}

/**
 * @brief Add consumer to bus tracking
 */
static esp_err_t fluctus_add_consumer(power_bus_t bus, const char* consumer_id)
{
    if (bus >= POWER_BUS_COUNT || consumer_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find empty slot or existing consumer
    for (int i = 0; i < 8; i++) {
        if (strlen(system_status.bus_consumers[bus][i]) == 0 || 
            strcmp(system_status.bus_consumers[bus][i], consumer_id) == 0) {
            strncpy(system_status.bus_consumers[bus][i], consumer_id, 15);
            system_status.bus_consumers[bus][i][15] = '\0';
            return ESP_OK;
        }
    }
    
    ESP_LOGW(TAG, "No free consumer slots for bus %d", bus);
    return ESP_ERR_NO_MEM;
}

/**
 * @brief Remove consumer from bus tracking
 */
static esp_err_t fluctus_remove_consumer(power_bus_t bus, const char* consumer_id)
{
    if (bus >= POWER_BUS_COUNT || consumer_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find and remove consumer
    for (int i = 0; i < 8; i++) {
        if (strcmp(system_status.bus_consumers[bus][i], consumer_id) == 0) {
            memset(system_status.bus_consumers[bus][i], 0, 16);
            return ESP_OK;
        }
    }
    
    return ESP_ERR_NOT_FOUND;
}

/**
 * @brief Check and update power management state based on battery voltage
 */
static void fluctus_check_power_state(void)
{
    float battery_voltage = monitoring_data.battery_voltage;
    fluctus_power_state_t current_state = system_status.power_state;
    fluctus_power_state_t new_state = current_state;
    
    // Safety shutdown overrides everything
    if (system_status.safety_shutdown) {
        new_state = FLUCTUS_POWER_STATE_SHUTDOWN;
    } else {
        // Check voltage thresholds with hysteresis
        if (current_state <= FLUCTUS_POWER_STATE_NORMAL) {
            // Falling thresholds (turn off systems)
            if (battery_voltage <= FLUCTUS_BATTERY_LEVEL_CRITICAL) {
                new_state = FLUCTUS_POWER_STATE_CRITICAL;
            } else if (battery_voltage <= FLUCTUS_BATTERY_LEVEL_VERY_LOW) {
                new_state = FLUCTUS_POWER_STATE_VERY_LOW;
            } else if (battery_voltage <= FLUCTUS_BATTERY_LEVEL_LOW_POWER) {
                new_state = FLUCTUS_POWER_STATE_LOW_POWER;
            } else if (battery_voltage <= FLUCTUS_BATTERY_LEVEL_POWER_SAVING) {
                new_state = FLUCTUS_POWER_STATE_POWER_SAVING;
            }
        } else {
            // Rising thresholds (turn on systems with hysteresis)
            if (battery_voltage >= FLUCTUS_BATTERY_LEVEL_POWER_SAVING_ON) {
                new_state = FLUCTUS_POWER_STATE_NORMAL;
            } else if (battery_voltage >= FLUCTUS_BATTERY_LEVEL_LOW_POWER_ON && 
                      current_state >= FLUCTUS_POWER_STATE_LOW_POWER) {
                new_state = FLUCTUS_POWER_STATE_POWER_SAVING;
            } else if (battery_voltage >= FLUCTUS_BATTERY_LEVEL_VERY_LOW_ON && 
                      current_state >= FLUCTUS_POWER_STATE_VERY_LOW) {
                new_state = FLUCTUS_POWER_STATE_LOW_POWER;
            } else if (battery_voltage >= FLUCTUS_BATTERY_LEVEL_CRITICAL_ON && 
                      current_state >= FLUCTUS_POWER_STATE_CRITICAL) {
                new_state = FLUCTUS_POWER_STATE_VERY_LOW;
            }
        }
    }
    
    if (new_state != current_state) {
        fluctus_handle_power_state_change(new_state);
    }
}

/**
 * @brief Handle power state changes and load shedding
 */
static void fluctus_handle_power_state_change(fluctus_power_state_t new_state)
{
    fluctus_power_state_t old_state = system_status.power_state;
    system_status.power_state = new_state;
    
    ESP_LOGI(TAG, "Power state change: %s -> %s", 
             fluctus_power_state_to_string(old_state),
             fluctus_power_state_to_string(new_state));
    
    fluctus_log_power_event(FLUCTUS_EVENT_POWER_STATE_CHANGE, POWER_BUS_COUNT, NULL);
    
    // Implement load shedding logic based on power state transitions
    switch (new_state) {
        case FLUCTUS_POWER_STATE_NORMAL:
            // Full operation - all systems enabled
            ESP_LOGI(TAG, "Power state: NORMAL - all systems operational");
            stellaria_set_power_save_mode(false);  // Disable power save mode
            stellaria_set_shutdown(false);         // Restore from shutdown
            impluvium_set_power_save_mode(false);  // Normal moisture check intervals
            impluvium_set_shutdown(false);         // Restore irrigation operations
            tempesta_set_power_save_mode(false);   // Normal 15min weather collection
            tempesta_set_shutdown(false);          // Restore weather monitoring
            tempesta_set_pms5003_enabled(true);    // Enable air quality sensor
            break;
            
        case FLUCTUS_POWER_STATE_POWER_SAVING:
            // Limited operation - STELLARIA power save mode (20% intensity)
            ESP_LOGI(TAG, "Power state: POWER_SAVING - STELLARIA limited to 20%% intensity");
            stellaria_set_power_save_mode(true);   // Enable power save mode (20% intensity)
            stellaria_set_shutdown(false);         // Ensure not in shutdown
            impluvium_set_power_save_mode(true);   // 60min moisture check interval
            impluvium_set_shutdown(false);         // Keep irrigation operational
            tempesta_set_power_save_mode(false);   // Keep normal weather collection
            tempesta_set_shutdown(false);          // Keep weather monitoring
            tempesta_set_pms5003_enabled(true);    // Keep air quality sensor enabled
            break;
            
        case FLUCTUS_POWER_STATE_LOW_POWER:
            // Reduced operation - STELLARIA shutdown, TEMPESTA power save
            ESP_LOGI(TAG, "Power state: LOW_POWER - STELLARIA disabled, TEMPESTA power save");
            stellaria_set_shutdown(true);          // Shutdown STELLARIA
            impluvium_set_power_save_mode(true);   // Keep 60min moisture check interval
            impluvium_set_shutdown(false);         // Keep irrigation operational
            tempesta_set_power_save_mode(true);    // 60min weather collection interval
            tempesta_set_shutdown(false);          // Keep weather monitoring
            tempesta_set_pms5003_enabled(true);    // Keep air quality sensor enabled
            break;
            
        case FLUCTUS_POWER_STATE_VERY_LOW:
            // Minimal operation - STELLARIA + IMPLUVIUM shutdown, TEMPESTA skip PMS5003
            ESP_LOGI(TAG, "Power state: VERY_LOW - STELLARIA and IMPLUVIUM disabled, TEMPESTA PMS5003 skipped");
            stellaria_set_shutdown(true);          // Shutdown STELLARIA
            impluvium_set_shutdown(true);          // Shutdown all irrigation operations
            tempesta_set_power_save_mode(true);    // 60min weather collection interval
            tempesta_set_shutdown(false);          // Keep weather monitoring
            tempesta_set_pms5003_enabled(false);   // Skip air quality sensor to save power
            break;
            
        case FLUCTUS_POWER_STATE_CRITICAL:
            // Emergency operation - only essential systems (TEMPESTA + solar tracking off)
            ESP_LOGI(TAG, "Power state: CRITICAL - TEMPESTA and solar tracking disabled");
            stellaria_set_shutdown(true);          // Shutdown STELLARIA
            impluvium_set_shutdown(true);          // Shutdown all irrigation operations
            tempesta_set_shutdown(true);           // Shutdown weather monitoring
            if (system_status.solar_tracking_enabled) {
                fluctus_disable_solar_tracking();
            }
            break;
        case FLUCTUS_POWER_STATE_SHUTDOWN:
            // Emergency shutdown - disable all buses
            for (int i = 0; i < POWER_BUS_COUNT; i++) {
                system_status.bus_enabled[i] = false;
                fluctus_update_bus_hardware(i);
            }
            system_status.solar_tracking_enabled = false;
            break;
        default:
            break;
    }
}

/**
 * @brief Read photoresistor values for solar tracking
 */
static esp_err_t fluctus_read_photoresistors(fluctus_solar_tracking_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Enhanced photoresistor reading: 3 samples with 500ms delays for averaging
    const uint8_t sample_count = 3;
    const uint32_t sample_delay_ms = 500;
    float sample_accumulator[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t valid_samples[4] = {0, 0, 0, 0};
    
    ESP_LOGD(TAG, "Starting enhanced photoresistor reading (%d samples over %.1fs)", 
             sample_count, (sample_count - 1) * sample_delay_ms / 1000.0f);
    
    // Take multiple samples for each channel
    for (uint8_t sample = 0; sample < sample_count; sample++) {
        int16_t raw_value;
        float voltage;
        
        for (int channel = 0; channel < 4; channel++) {
            esp_err_t ret = ads1115_read_channel(0, (ads111x_mux_t)channel, &raw_value, &voltage);
            if (ret == ESP_OK) {
                sample_accumulator[channel] += voltage;
                valid_samples[channel]++;
            } else {
                ESP_LOGW(TAG, "Failed to read photoresistor ch%d sample%d: %s", 
                         channel, sample + 1, esp_err_to_name(ret));
            }
        }
        
        // Delay between samples (except after the last sample)
        if (sample < sample_count - 1) {
            vTaskDelay(pdMS_TO_TICKS(sample_delay_ms));
        }
    }
    
    // Calculate averages from valid samples
    for (int channel = 0; channel < 4; channel++) {
        if (valid_samples[channel] > 0) {
            data->photoresistor_readings[channel] = sample_accumulator[channel] / valid_samples[channel];
            ESP_LOGD(TAG, "Photoresistor ch%d: %.3fV (avg of %d samples)", 
                     channel, data->photoresistor_readings[channel], valid_samples[channel]);
        } else {
            ESP_LOGW(TAG, "No valid samples for photoresistor channel %d, using 0.0V", channel);
            data->photoresistor_readings[channel] = 0.0f;
        }
    }
    
    // Calculate tracking errors with averaging (photoresistor arrangement: TL, TR, BL, BR)
    // Top-left, Top-right, Bottom-left, Bottom-right
    float left_avg = (data->photoresistor_readings[0] + data->photoresistor_readings[2]) / 2.0f;   // TL + BL
    float right_avg = (data->photoresistor_readings[1] + data->photoresistor_readings[3]) / 2.0f;  // TR + BR
    float top_avg = (data->photoresistor_readings[0] + data->photoresistor_readings[1]) / 2.0f;    // TL + TR  
    float bottom_avg = (data->photoresistor_readings[2] + data->photoresistor_readings[3]) / 2.0f; // BL + BR
    
    data->yaw_error = left_avg - right_avg;      // Negative = turn right, Positive = turn left
    data->pitch_error = top_avg - bottom_avg;    // Negative = tilt down, Positive = tilt up
    
    data->valid = true;
    return ESP_OK;
}

/**
 * @brief Set servo position via PWM
 */
static esp_err_t fluctus_servo_set_position(ledc_channel_t channel, uint32_t duty_cycle)
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
 */
static uint32_t fluctus_calc_servo_adjust(float error, uint32_t current_duty)
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
 * @brief Park servos in center position
 */
static esp_err_t fluctus_park_servos(void)
{
    esp_err_t ret1 = fluctus_servo_set_position(FLUCTUS_SERVO_YAW_CHANNEL, FLUCTUS_SERVO_CENTER_DUTY);
    esp_err_t ret2 = fluctus_servo_set_position(FLUCTUS_SERVO_PITCH_CHANNEL, FLUCTUS_SERVO_CENTER_DUTY);
    
    if (ret1 == ESP_OK && ret2 == ESP_OK) {
        system_status.current_yaw_duty = FLUCTUS_SERVO_CENTER_DUTY;
        system_status.current_pitch_duty = FLUCTUS_SERVO_CENTER_DUTY;
        ESP_LOGI(TAG, "Servos parked in center position");
    }
    
    return (ret1 == ESP_OK && ret2 == ESP_OK) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Read all INA219 sensors and update monitoring data
 */
static esp_err_t fluctus_read_ina219_sensors(void)
{
    esp_err_t ret = ESP_OK;
    
    // Read Solar PV sensor
    float voltage, current, power;
    if (ina219_get_bus_voltage(&ina219_devices[0], &voltage) == ESP_OK &&
        ina219_get_current(&ina219_devices[0], &current) == ESP_OK &&
        ina219_get_power(&ina219_devices[0], &power) == ESP_OK) {
        
        monitoring_data.solar_pv.voltage = voltage;
        monitoring_data.solar_pv.current = current;
        monitoring_data.solar_pv.power = power;
        monitoring_data.solar_pv.valid = true;
        monitoring_data.solar_pv.timestamp = time(NULL);
    } else {
        monitoring_data.solar_pv.valid = false;
        ret = ESP_FAIL;
    }
    
    // Read Battery Output sensor
    if (ina219_get_bus_voltage(&ina219_devices[1], &voltage) == ESP_OK &&
        ina219_get_current(&ina219_devices[1], &current) == ESP_OK &&
        ina219_get_power(&ina219_devices[1], &power) == ESP_OK) {
        
        monitoring_data.battery_out.voltage = voltage;
        monitoring_data.battery_out.current = current;
        monitoring_data.battery_out.power = power;
        monitoring_data.battery_out.valid = true;
        monitoring_data.battery_out.timestamp = time(NULL);
        
        // Use battery output voltage as system battery voltage
        monitoring_data.battery_voltage = voltage;
        monitoring_data.total_current = current;
        monitoring_data.total_power = power;
    } else {
        monitoring_data.battery_out.valid = false;
        ret = ESP_FAIL;
    }
    
    return ret;
}

/**
 * @brief Check for overcurrent conditions and handle safety shutdown
 */
static void fluctus_check_overcurrent(void)
{
    if (!monitoring_data.battery_out.valid) {
        return;
    }
    
    float current = monitoring_data.total_current;
    int64_t current_time_ms = esp_timer_get_time() / 1000;
    
    // Check immediate shutdown threshold (3A)
    if (current >= FLUCTUS_OVERCURRENT_THRESHOLD_2) {
        ESP_LOGE(TAG, "IMMEDIATE OVERCURRENT SHUTDOWN: %.2fA >= %.2fA", 
                 current, FLUCTUS_OVERCURRENT_THRESHOLD_2);
        
        fluctus_log_active_consumers("IMMEDIATE OVERCURRENT SHUTDOWN");
        
        system_status.safety_shutdown = true;
        system_status.manual_reset_required = true;
        fluctus_log_power_event(FLUCTUS_EVENT_OVERCURRENT_SHUTDOWN, POWER_BUS_COUNT, NULL);
        fluctus_handle_power_state_change(FLUCTUS_POWER_STATE_SHUTDOWN);
        return;
    }
    
    // Check delayed shutdown threshold (1.5A for 3 seconds)
    if (current >= FLUCTUS_OVERCURRENT_THRESHOLD_1) {
        if (!overcurrent_timer_active) {
            overcurrent_start_time = current_time_ms;
            overcurrent_timer_active = true;
            ESP_LOGW(TAG, "Overcurrent warning: %.2fA >= %.2fA (timer started)", 
                     current, FLUCTUS_OVERCURRENT_THRESHOLD_1);
            
            fluctus_log_active_consumers("OVERCURRENT WARNING");
            fluctus_log_power_event(FLUCTUS_EVENT_OVERCURRENT_WARNING, POWER_BUS_COUNT, NULL);
        } else if ((current_time_ms - overcurrent_start_time) >= FLUCTUS_OVERCURRENT_DELAY_MS) {
            ESP_LOGE(TAG, "DELAYED OVERCURRENT SHUTDOWN: %.2fA for %dms", 
                     current, FLUCTUS_OVERCURRENT_DELAY_MS);
            
            fluctus_log_active_consumers("DELAYED OVERCURRENT SHUTDOWN");
            
            system_status.safety_shutdown = true;
            system_status.manual_reset_required = true;
            overcurrent_timer_active = false;
            fluctus_log_power_event(FLUCTUS_EVENT_OVERCURRENT_SHUTDOWN, POWER_BUS_COUNT, NULL);
            fluctus_handle_power_state_change(FLUCTUS_POWER_STATE_SHUTDOWN);
        }
    } else {
        // Current below threshold, reset timer
        if (overcurrent_timer_active) {
            ESP_LOGI(TAG, "Overcurrent condition cleared");
            overcurrent_timer_active = false;
        }
    }
}

/**
 * @brief Check 3.3V bus voltage level for drift
 * Reads ADS1115 device #2, channel 3 and warns if voltage is outside ±0.1V of 3.3V
 */
static void fluctus_check_3v3_bus_voltage(void)
{
    int16_t raw_value;
    float voltage;
    
    // Read 3V3 bus voltage from ADS1115 device #2, channel 3
    esp_err_t ret = ads1115_read_channel(2, ADS111X_MUX_3_GND, &raw_value, &voltage);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Failed to read 3V3 bus voltage: %s", esp_err_to_name(ret));
        return;
    }
    
    // Check if voltage is within acceptable range (3.3V ± 0.1V)
    const float target_voltage = 3.3f;
    const float tolerance = 0.1f;
    float voltage_error = fabsf(voltage - target_voltage);
    
    if (voltage_error > tolerance) {
        ESP_LOGW(TAG, "3V3 bus voltage drift detected: %.3fV (target: %.1fV ±%.1fV)", 
                 voltage, target_voltage, tolerance);
    } else {
        ESP_LOGD(TAG, "3V3 bus voltage OK: %.3fV", voltage);
    }
}

/**
 * @brief Log all currently active consumers on powered buses
 */
static void fluctus_log_active_consumers(const char* event_context)
{
    const char* bus_names[] = {"3V3", "5V", "6V2", "12V"};
    bool any_consumers = false;
    
    ESP_LOGE(TAG, "%s - Active consumers:", event_context);
    
    for (int bus = 0; bus < POWER_BUS_COUNT; bus++) {
        if (system_status.bus_enabled[bus] && system_status.bus_ref_count[bus] > 0) {
            ESP_LOGE(TAG, "  %s bus (%d consumers):", bus_names[bus], system_status.bus_ref_count[bus]);
            for (int i = 0; i < 8; i++) {
                if (strlen(system_status.bus_consumers[bus][i]) > 0) {
                    ESP_LOGE(TAG, "    - %s", system_status.bus_consumers[bus][i]);
                    any_consumers = true;
                }
            }
        }
    }
    
    if (!any_consumers) {
        ESP_LOGE(TAG, "  No active consumers found (possible hardware issue)");
    }
}

/**
 * @brief Log power management events
 */
static void fluctus_log_power_event(fluctus_event_type_t event_type, power_bus_t bus, 
                                   const char* consumer_id)
{
    const char* event_names[] = {
        "BUS_ON", "BUS_OFF", "CONSUMER_REQUEST", "CONSUMER_RELEASE",
        "OVERCURRENT_WARNING", "OVERCURRENT_SHUTDOWN", "BATTERY_LOW",
        "SOLAR_TRACKING_PARK", "POWER_STATE_CHANGE"
    };
    
    const char* bus_names[] = {"3V3", "5V", "6V2", "12V", "SYSTEM"};
    
    if (event_type < sizeof(event_names)/sizeof(event_names[0])) {
        if (bus < POWER_BUS_COUNT) {
            ESP_LOGI(TAG, "Event: %s [%s] %s", event_names[event_type], bus_names[bus],
                     consumer_id ? consumer_id : "");
        } else {
            ESP_LOGI(TAG, "Event: %s %s", event_names[event_type],
                     consumer_id ? consumer_id : "");
        }
    }
}

/**
 * @brief Power monitoring FreeRTOS task
 */
static void fluctus_monitoring_task(void *parameters)
{
    ESP_LOGI(TAG, "Power monitoring task started");
    
    bool monitoring_active = false;
    
    while (true) {
        uint32_t notification_value = 0;
        TickType_t timeout;
        
        // Determine timeout based on monitoring state
        if (monitoring_active) {
            // Active monitoring - short timeout for continuous monitoring
            timeout = pdMS_TO_TICKS(FLUCTUS_POWER_MONITOR_INTERVAL_MS);
        } else {
            // Idle monitoring - long timeout for periodic checks
            timeout = pdMS_TO_TICKS(FLUCTUS_POWER_MONITOR_IDLE_INTERVAL_MS);
        }
        
        // Wait for notification or timeout
        BaseType_t notified = xTaskNotifyWait(0x00,              // Don't clear on entry
                                              ULONG_MAX,          // Clear all on exit
                                              &notification_value, // Store notification
                                              timeout);            // Timeout
        
        // Handle notification
        if (notified == pdTRUE && (notification_value & FLUCTUS_NOTIFY_POWER_ACTIVITY)) {
            ESP_LOGD(TAG, "Power activity notification received - starting active monitoring");
            monitoring_active = true;
        }
        
        // Read power sensors
        if (xSemaphoreTake(xFluctusMonitoringMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
            fluctus_read_ina219_sensors();
            fluctus_check_3v3_bus_voltage(); // Check 3.3V bus voltage
            xSemaphoreGive(xFluctusMonitoringMutex);
        }
        
        // Check power management conditions
        if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
            fluctus_check_overcurrent();
            fluctus_check_power_state();
            
            // Check if any buses are still active
            bool system_active = false;
            for (int i = 0; i < POWER_BUS_COUNT; i++) {
                if (system_status.bus_enabled[i]) {
                    system_active = true;
                    break;
                }
            }
            
            // If no buses active, switch to idle monitoring
            if (!system_active && monitoring_active) {
                ESP_LOGD(TAG, "No active buses - switching to idle monitoring");
                monitoring_active = false;
            }
            
            xSemaphoreGive(xFluctusPowerMutex);
        }
        
        // If timeout occurred during idle monitoring, log periodic check
        if (notified == pdFALSE && !monitoring_active) {
            ESP_LOGD(TAG, "Periodic power monitoring check (idle mode)");
        }
    }
}

/**
 * @brief Solar tracking task
 */
static void fluctus_solar_tracking_task(void *parameters)
{
    ESP_LOGI(TAG, "Solar tracking task started");
    
    while (true) {
        bool tracking_enabled = false;
        
        if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
            tracking_enabled = system_status.solar_tracking_enabled && 
                             !system_status.safety_shutdown &&
                             (system_status.power_state < FLUCTUS_POWER_STATE_CRITICAL);
            xSemaphoreGive(xFluctusPowerMutex);
        }
        
        if (tracking_enabled) {
            fluctus_solar_tracking_data_t tracking_data;
            if (fluctus_read_photoresistors(&tracking_data) == ESP_OK && tracking_data.valid) {
                
                // Update STELLARIA with averaged light reading
                float avg_light = (tracking_data.photoresistor_readings[0] + 
                                  tracking_data.photoresistor_readings[1] + 
                                  tracking_data.photoresistor_readings[2] + 
                                  tracking_data.photoresistor_readings[3]) / 4.0f;
                stellaria_update_light_reading(avg_light);
                
                // Calculate new servo positions
                uint32_t new_yaw_duty = fluctus_calc_servo_adjust(tracking_data.yaw_error, 
                                                                system_status.current_yaw_duty);
                uint32_t new_pitch_duty = fluctus_calc_servo_adjust(tracking_data.pitch_error, 
                                                                  system_status.current_pitch_duty);
                
                // Update servos if position changed
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
                    ESP_LOGD(TAG, "Solar tracking update - Yaw: %lu, Pitch: %lu", 
                            system_status.current_yaw_duty, system_status.current_pitch_duty);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(FLUCTUS_TRACKING_INTERVAL_MS));
    }
}

// ########################## Public API Functions ##########################

esp_err_t fluctus_init(void)
{
    ESP_LOGI(TAG, "Initializing FLUCTUS power management and solar tracking system...");
    
    if (fluctus_initialized) {
        ESP_LOGW(TAG, "FLUCTUS already initialized");
        return ESP_OK;
    }
    
    // Create mutexes
    xFluctusPowerMutex = xSemaphoreCreateMutex();
    if (xFluctusPowerMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create power mutex");
        return ESP_FAIL;
    }
    
    xFluctusMonitoringMutex = xSemaphoreCreateMutex();
    if (xFluctusMonitoringMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create monitoring mutex");
        vSemaphoreDelete(xFluctusPowerMutex);
        return ESP_FAIL;
    }
    
    // Initialize subsystems
    esp_err_t ret = fluctus_gpio_init();
    if (ret != ESP_OK) {
        goto cleanup;
    }
    
    ret = fluctus_ina219_init();
    if (ret != ESP_OK) {
        goto cleanup;
    }
    
    ret = fluctus_servo_pwm_init();
    if (ret != ESP_OK) {
        goto cleanup;
    }
    
    ret = fluctus_fan_pwm_init();
    if (ret != ESP_OK) {
        goto cleanup;
    }
    
    // Initialize system status
    memset(&system_status, 0, sizeof(fluctus_power_status_t));
    system_status.power_state = FLUCTUS_POWER_STATE_NORMAL;
    system_status.current_yaw_duty = FLUCTUS_SERVO_CENTER_DUTY;
    system_status.current_pitch_duty = FLUCTUS_SERVO_CENTER_DUTY;
    
    // Create monitoring task
    BaseType_t task_ret = xTaskCreate(
        fluctus_monitoring_task,
        "fluctus_monitor",
        4096,
        NULL,
        5,
        &xFluctusMonitoringTaskHandle
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitoring task");
        ret = ESP_FAIL;
        goto cleanup;
    }
    
    // Create solar tracking task
    task_ret = xTaskCreate(
        fluctus_solar_tracking_task,
        "fluctus_solar",
        3072,
        NULL,
        4,
        &xFluctusSolarTrackingTaskHandle
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create solar tracking task");
        ret = ESP_FAIL;
        goto cleanup;
    }
    
    fluctus_initialized = true;
    ESP_LOGI(TAG, "FLUCTUS initialization complete");
    return ESP_OK;
    
cleanup:
    if (xFluctusMonitoringTaskHandle) {
        vTaskDelete(xFluctusMonitoringTaskHandle);
        xFluctusMonitoringTaskHandle = NULL;
    }
    if (xFluctusSolarTrackingTaskHandle) {
        vTaskDelete(xFluctusSolarTrackingTaskHandle);
        xFluctusSolarTrackingTaskHandle = NULL;
    }
    if (xFluctusMonitoringMutex) {
        vSemaphoreDelete(xFluctusMonitoringMutex);
        xFluctusMonitoringMutex = NULL;
    }
    if (xFluctusPowerMutex) {
        vSemaphoreDelete(xFluctusPowerMutex);
        xFluctusPowerMutex = NULL;
    }
    return ret;
}

esp_err_t fluctus_request_bus_power(power_bus_t bus, const char* consumer_id)
{
    if (bus >= POWER_BUS_COUNT || consumer_id == NULL || !fluctus_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
        if (system_status.safety_shutdown) {
            xSemaphoreGive(xFluctusPowerMutex);
            return ESP_ERR_INVALID_STATE;
        }
        
        // Add consumer and increment reference count
        fluctus_add_consumer(bus, consumer_id);
        system_status.bus_ref_count[bus]++;
        
        // Enable bus if not already enabled
        if (!system_status.bus_enabled[bus]) {
            system_status.bus_enabled[bus] = true;
            fluctus_update_bus_hardware(bus);
            fluctus_log_power_event(FLUCTUS_EVENT_BUS_ON, bus, consumer_id);
            
            // Notify monitoring task of power activity
            if (xFluctusMonitoringTaskHandle != NULL) {
                xTaskNotify(xFluctusMonitoringTaskHandle, FLUCTUS_NOTIFY_POWER_ACTIVITY, eSetBits);
            }
        } else {
            fluctus_log_power_event(FLUCTUS_EVENT_CONSUMER_REQUEST, bus, consumer_id);
        }
        
        system_status.last_activity_time = time(NULL);
        
        xSemaphoreGive(xFluctusPowerMutex);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

esp_err_t fluctus_release_bus_power(power_bus_t bus, const char* consumer_id)
{
    if (bus >= POWER_BUS_COUNT || consumer_id == NULL || !fluctus_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
        // Remove consumer and decrement reference count
        fluctus_remove_consumer(bus, consumer_id);
        if (system_status.bus_ref_count[bus] > 0) {
            system_status.bus_ref_count[bus]--;
        }
        
        // Disable bus if no more consumers
        if (system_status.bus_ref_count[bus] == 0 && system_status.bus_enabled[bus]) {
            system_status.bus_enabled[bus] = false;
            fluctus_update_bus_hardware(bus);
            fluctus_log_power_event(FLUCTUS_EVENT_BUS_OFF, bus, consumer_id);
        } else {
            fluctus_log_power_event(FLUCTUS_EVENT_CONSUMER_RELEASE, bus, consumer_id);
        }
        
        xSemaphoreGive(xFluctusPowerMutex);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

bool fluctus_is_bus_powered(power_bus_t bus)
{
    if (bus >= POWER_BUS_COUNT || !fluctus_initialized) {
        return false;
    }
    
    bool powered = false;
    if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        powered = system_status.bus_enabled[bus] && !system_status.safety_shutdown;
        xSemaphoreGive(xFluctusPowerMutex);
    }
    
    return powered;
}

fluctus_power_state_t fluctus_get_power_state(void)
{
    if (!fluctus_initialized) {
        return FLUCTUS_POWER_STATE_SHUTDOWN;
    }
    
    fluctus_power_state_t state = FLUCTUS_POWER_STATE_SHUTDOWN;
    if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        state = system_status.power_state;
        xSemaphoreGive(xFluctusPowerMutex);
    }
    
    return state;
}

bool fluctus_is_safety_shutdown(void)
{
    if (!fluctus_initialized) {
        return true;
    }
    
    bool shutdown = true;
    if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        shutdown = system_status.safety_shutdown;
        xSemaphoreGive(xFluctusPowerMutex);
    }
    
    return shutdown;
}

esp_err_t fluctus_manual_safety_reset(void)
{
    if (!fluctus_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
        if (!system_status.manual_reset_required) {
            xSemaphoreGive(xFluctusPowerMutex);
            return ESP_ERR_INVALID_STATE;
        }
        
        system_status.safety_shutdown = false;
        system_status.manual_reset_required = false;
        overcurrent_timer_active = false;
        
        // Reset to normal power state
        system_status.power_state = FLUCTUS_POWER_STATE_NORMAL;
        
        ESP_LOGI(TAG, "Manual safety reset performed - system ready");
        
        xSemaphoreGive(xFluctusPowerMutex);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

esp_err_t fluctus_enable_solar_tracking(void)
{
    if (!fluctus_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
        if (system_status.safety_shutdown || 
            system_status.power_state >= FLUCTUS_POWER_STATE_CRITICAL) {
            xSemaphoreGive(xFluctusPowerMutex);
            return ESP_ERR_INVALID_STATE;
        }
        
        // Request 6.2V servo bus power
        esp_err_t ret = fluctus_request_bus_power(POWER_BUS_6V2, "FLUCTUS_SOLAR");
        if (ret == ESP_OK) {
            system_status.solar_tracking_enabled = true;
            ESP_LOGI(TAG, "Solar tracking enabled");
        }
        
        xSemaphoreGive(xFluctusPowerMutex);
        return ret;
    }
    
    return ESP_FAIL;
}

esp_err_t fluctus_disable_solar_tracking(void)
{
    if (!fluctus_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
        if (system_status.solar_tracking_enabled) {
            // Park servos in center position
            fluctus_park_servos();
            
            // Release servo bus power
            fluctus_release_bus_power(POWER_BUS_6V2, "FLUCTUS_SOLAR");
            
            system_status.solar_tracking_enabled = false;
            ESP_LOGI(TAG, "Solar tracking disabled and servos parked");
        }
        
        xSemaphoreGive(xFluctusPowerMutex);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

bool fluctus_is_solar_tracking_enabled(void)
{
    if (!fluctus_initialized) {
        return false;
    }
    
    bool enabled = false;
    if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        enabled = system_status.solar_tracking_enabled && !system_status.safety_shutdown;
        xSemaphoreGive(xFluctusPowerMutex);
    }
    
    return enabled;
}

esp_err_t fluctus_get_servo_positions(uint32_t *yaw_duty, uint32_t *pitch_duty)
{
    if (yaw_duty == NULL || pitch_duty == NULL || !fluctus_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        *yaw_duty = system_status.current_yaw_duty;
        *pitch_duty = system_status.current_pitch_duty;
        xSemaphoreGive(xFluctusPowerMutex);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

esp_err_t fluctus_get_monitoring_data(fluctus_monitoring_data_t *data)
{
    if (data == NULL || !fluctus_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(xFluctusMonitoringMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        memcpy(data, &monitoring_data, sizeof(fluctus_monitoring_data_t));
        xSemaphoreGive(xFluctusMonitoringMutex);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

float fluctus_get_battery_voltage(void)
{
    if (!fluctus_initialized) {
        return -1.0f;
    }
    
    float voltage = -1.0f;
    if (xSemaphoreTake(xFluctusMonitoringMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        voltage = monitoring_data.battery_voltage;
        xSemaphoreGive(xFluctusMonitoringMutex);
    }
    
    return voltage;
}

float fluctus_get_total_current(void)
{
    if (!fluctus_initialized) {
        return -1.0f;
    }
    
    float current = -1.0f;
    if (xSemaphoreTake(xFluctusMonitoringMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        current = monitoring_data.total_current;
        xSemaphoreGive(xFluctusMonitoringMutex);
    }
    
    return current;
}

esp_err_t fluctus_get_system_status(fluctus_power_status_t *status)
{
    if (status == NULL || !fluctus_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        memcpy(status, &system_status, sizeof(fluctus_power_status_t));
        xSemaphoreGive(xFluctusPowerMutex);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

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

const char* fluctus_power_state_to_string(fluctus_power_state_t state)
{
    switch (state) {
        case FLUCTUS_POWER_STATE_NORMAL:        return "NORMAL";
        case FLUCTUS_POWER_STATE_POWER_SAVING:  return "POWER_SAVING";
        case FLUCTUS_POWER_STATE_LOW_POWER:     return "LOW_POWER";
        case FLUCTUS_POWER_STATE_VERY_LOW:      return "VERY_LOW";
        case FLUCTUS_POWER_STATE_CRITICAL:      return "CRITICAL";
        case FLUCTUS_POWER_STATE_SHUTDOWN:      return "SHUTDOWN";
        default:                                return "UNKNOWN";
    }
}