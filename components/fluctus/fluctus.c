#include "fluctus.h"
#include "stellaria.h"
#include "irrigation.h"
#include "weather_station.h"
#include "telemetry.h"
#include "solar_calc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>
#include <time.h>
#include "ads111x.h"
#include "ads1115_helper.h"
#include "onewire.h"

// ########################## Constants and Variables ##########################

static const char *TAG = "FLUCTUS";

// Global system status
static fluctus_power_status_t system_status = {
    .bus_enabled = {false, false, false, false},
    .bus_ref_count = {0, 0, 0, 0},
    .power_state = FLUCTUS_POWER_STATE_NORMAL,
    .solar_tracking_state = SOLAR_TRACKING_DISABLED,
    .current_yaw_duty = FLUCTUS_SERVO_CENTER_DUTY,
    .current_pitch_duty = FLUCTUS_SERVO_CENTER_DUTY,
    .safety_shutdown = false,
    .manual_reset_required = false,
    .last_activity_time = 0
};

// Power monitoring data
static fluctus_monitoring_data_t monitoring_data = {0};

// INA219 device configurations
static ina219_t ina219_dev[FLUCTUS_INA219_DEVICE_COUNT];

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

// Hardware state cache to avoid redundant GPIO operations
static bool hardware_bus_state[POWER_BUS_COUNT] = {false, false, false, false};

// DS18B20 temperature sensor variables
static onewire_addr_t ds18b20_addr = ONEWIRE_NONE;
static bool ds18b20_found = false;
static bool fan_active = false;
static bool thermal_monitoring_active = false;
static int64_t last_temp_read_time = 0;

// Power metering state variables
static bool pv_ina_powered = true;        // PV INA219 power state (true = active)
static bool battery_ina_powered = true;   // Battery INA219 power state (true = active)
static int64_t steady_state_probe_start = 0;  // Steady state probe timer

// Solar tracking state variables
static int64_t last_correction_time = 0;      // Last correction cycle start time
static int64_t correction_start_time = 0;     // Current correction cycle start time

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
static uint32_t fluctus_calculate_servo_correction(float error, uint32_t current_duty);
static uint32_t fluctus_percent_to_duty(float percent);
static esp_err_t fluctus_park_servos_night(void);
static esp_err_t fluctus_park_servos_error(void);
static bool fluctus_tracking_error_margin_check(float yaw_error, float pitch_error);
static float fluctus_read_avg_light_update_stellaria(void);
static void fluctus_set_tracking_state(solar_tracking_state_t new_state, const char *log_msg);
static bool fluctus_apply_servo_corrections(fluctus_solar_tracking_data_t *tracking_data);

// Monitoring functions
static esp_err_t fluctus_read_ina219_sensors(void);
static void fluctus_check_overcurrent(void);
static void fluctus_check_3v3_bus_voltage(void);
static void fluctus_log_active_consumers(const char* event_context);
static void fluctus_log_power_event(fluctus_event_type_t event_type, power_bus_t bus,
                                   const char* consumer_id);

// DS18B20 temperature sensor functions
static esp_err_t fluctus_ds18b20_init(void);
static esp_err_t fluctus_read_temperature(float *temperature);
static void fluctus_update_fan_speed(float temperature);
static bool fluctus_should_read_temperature(bool monitoring_active);
static void fluctus_handle_temperature_monitoring(bool monitoring_active);

// Power metering functions
static esp_err_t fluctus_ina219_set_power_mode(uint8_t device_index, bool active);
static void fluctus_update_power_metering_state(bool monitoring_active);
static void fluctus_integrate_telemetry_sample(void);

// FreeRTOS tasks
static void fluctus_monitoring_task(void *parameters);
static void fluctus_solar_tracking_task(void *parameters);

// ########################## Private Functions ##########################

/**
 * @brief Initialize GPIO pins for power control - New Hardware Configuration
 *
 * 3.3V Bus: Push-pull output, LOW=OFF, HIGH=ON (controls N-MOSFET via SN74AHCT125N)
 * 5V Bus:   Open-drain output, LOW=OFF, INPUT=ON (100kΩ series to buck EN)
 * 6.6V Bus: Open-drain output, LOW=ON, INPUT=OFF (inverted logic, 100kΩ pullup to 3.3V)
 * 12V Bus:  Open-drain output, LOW=OFF, INPUT=ON (100kΩ series to buck EN)
 */
static esp_err_t fluctus_gpio_init(void)
{
    ESP_LOGI(TAG, "Initializing power control GPIOs (new hardware config)...");
    esp_err_t ret;

    // 3.3V Bus: Push-pull output (controls N-MOSFET via buffer)
    gpio_config_t gpio_3v3_conf = {
        .pin_bit_mask = (1ULL << FLUCTUS_3V3_BUS_ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,  // Push-pull (NOT open-drain)
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ret = gpio_config(&gpio_3v3_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure 3.3V bus GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(FLUCTUS_3V3_BUS_ENABLE_GPIO, 0);  // Start LOW (OFF)

    // 5V, 6.6V, 12V Buses: Open-drain outputs
    gpio_config_t buck_od_conf = {
        .pin_bit_mask = (1ULL << FLUCTUS_5V_BUS_ENABLE_GPIO) |
                       (1ULL << FLUCTUS_6V6_BUS_ENABLE_GPIO) |
                       (1ULL << FLUCTUS_12V_BUS_ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT_OD,  // Open-drain
        .pull_up_en = GPIO_PULLUP_DISABLE,  // No internal pull-ups (external hardware)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ret = gpio_config(&buck_od_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure open-drain buck GPIOs: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set initial states (all buses OFF)
    gpio_set_level(FLUCTUS_5V_BUS_ENABLE_GPIO, 0);   // 5V: LOW = OFF
    gpio_set_level(FLUCTUS_6V6_BUS_ENABLE_GPIO, 0);  // 6.6V: LOW = ON (inverted), but we want OFF, so...
    // For 6.6V inverted logic: Set as input (high-Z) to allow pullup to turn it OFF
    gpio_set_direction(FLUCTUS_6V6_BUS_ENABLE_GPIO, GPIO_MODE_INPUT);
    gpio_set_level(FLUCTUS_12V_BUS_ENABLE_GPIO, 0);  // 12V: LOW = OFF

    // Initialize hardware state cache to disabled
    for (int i = 0; i < POWER_BUS_COUNT; i++) {
        hardware_bus_state[i] = false;
    }

    ESP_LOGI(TAG, "Power control GPIOs initialized (3.3V=push-pull, others=open-drain, all OFF)");
    ESP_LOGI(TAG, "  3.3V: LOW (OFF), 5V: LOW (OFF), 6.6V: INPUT/HIGH (OFF-inverted), 12V: LOW (OFF)");
    return ESP_OK;
}

/**
 * @brief Initialize both INA219 power monitoring sensors
 */
static esp_err_t fluctus_ina219_init(void)
{
    ESP_LOGI(TAG, "Initializing INA219 power monitoring sensors...");
    
    // Initialize "Solar PV" sensor (0x40) descriptor
    
    esp_err_t ret = ina219_init_desc(&ina219_dev[0], FLUCTUS_INA219_SOLAR_PV_ADDR, 
                                     I2C_NUM_0, CONFIG_I2CDEV_DEFAULT_SDA_PIN, CONFIG_I2CDEV_DEFAULT_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Solar PV INA219: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = ina219_init(&ina219_dev[0]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Solar PV INA219: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure Solar PV INA219
    ret = ina219_configure(&ina219_dev[0], INA219_BUS_RANGE_32V, INA219_GAIN_0_5,
                          INA219_RES_12BIT_8S, INA219_RES_12BIT_8S, INA219_MODE_CONT_SHUNT_BUS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Solar PV INA219: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Calibrate with 0.1 ohm shunt for Solar PV
    ret = ina219_calibrate(&ina219_dev[0], 0.1f);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate Solar PV INA219: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize "Battery Output" sensor (0x41) descriptor
    
    ret = ina219_init_desc(&ina219_dev[1], FLUCTUS_INA219_BATTERY_OUT_ADDR,
                          I2C_NUM_0, CONFIG_I2CDEV_DEFAULT_SDA_PIN, CONFIG_I2CDEV_DEFAULT_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Battery Output INA219: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = ina219_init(&ina219_dev[1]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Battery Output INA219: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure Battery Output INA219 (higher current range)
    ret = ina219_configure(&ina219_dev[1], INA219_BUS_RANGE_32V, INA219_GAIN_0_125,
                          INA219_RES_12BIT_8S, INA219_RES_12BIT_8S, INA219_MODE_CONT_SHUNT_BUS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Battery Output INA219: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Calibrate with 0.01 ohm shunt for Battery Output (higher current range)
    ret = ina219_calibrate(&ina219_dev[1], 0.01f);
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
 * @brief Initialize DS18B20 temperature sensor
 */
static esp_err_t fluctus_ds18b20_init(void)
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
 * @brief Read temperature from DS18B20 sensor
 * @param[out] temperature Pointer to store temperature value in °C
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t fluctus_read_temperature(float *temperature)
{
    if (!ds18b20_found || temperature == NULL) {
        return ESP_FAIL;
    }

    // Reset and select device
    if (!onewire_reset(FLUCTUS_DS18B20_GPIO)) {
        ESP_LOGW(TAG, "DS18B20 not responding");
        return ESP_FAIL;
    }

    if (!onewire_select(FLUCTUS_DS18B20_GPIO, ds18b20_addr)) {
        ESP_LOGE(TAG, "Failed to select DS18B20");
        return ESP_FAIL;
    }

    // Start temperature conversion (0x44)
    uint8_t convert_cmd = 0x44;
    if (!onewire_write(FLUCTUS_DS18B20_GPIO, convert_cmd)) {
        ESP_LOGE(TAG, "Failed to start DS18B20 conversion");
        return ESP_FAIL;
    }

    // Wait for conversion to complete (750ms for 12-bit)
    vTaskDelay(pdMS_TO_TICKS(FLUCTUS_TEMP_CONVERSION_DELAY_MS));

    // Reset and select device again
    if (!onewire_reset(FLUCTUS_DS18B20_GPIO)) {
        ESP_LOGW(TAG, "DS18B20 not responding after conversion");
        return ESP_FAIL;
    }

    if (!onewire_select(FLUCTUS_DS18B20_GPIO, ds18b20_addr)) {
        ESP_LOGE(TAG, "Failed to select DS18B20 for read");
        return ESP_FAIL;
    }

    // Read scratchpad (0xBE)
    uint8_t read_cmd = 0xBE;
    if (!onewire_write(FLUCTUS_DS18B20_GPIO, read_cmd)) {
        ESP_LOGE(TAG, "Failed to send read command");
        return ESP_FAIL;
    }

    // Read 9 bytes of scratchpad data
    uint8_t scratchpad[9];
    if (!onewire_read_bytes(FLUCTUS_DS18B20_GPIO, scratchpad, 9)) {
        ESP_LOGE(TAG, "Failed to read DS18B20 scratchpad");
        return ESP_FAIL;
    }

    // Verify CRC
    uint8_t crc = onewire_crc8(scratchpad, 8);
    if (crc != scratchpad[8]) {
        ESP_LOGW(TAG, "DS18B20 scratchpad CRC mismatch");
        return ESP_FAIL;
    }

    // Calculate temperature
    int16_t raw_temp = (scratchpad[1] << 8) | scratchpad[0];
    *temperature = (float)raw_temp / 16.0f;

    ESP_LOGD(TAG, "DS18B20 temperature: %.2f°C", *temperature);

    return ESP_OK;
}

/**
 * @brief Update cooling fan speed based on temperature
 * @param temperature Current case temperature in °C
 */
static void fluctus_update_fan_speed(float temperature)
{
    bool should_run = false;
    uint8_t fan_duty = 0;

    // Hysteresis logic
    if (fan_active) {
        // Fan is currently on - turn off at 28°C
        should_run = (temperature >= FLUCTUS_TEMP_TURN_OFF_THRESHOLD);
    } else {
        // Fan is currently off - turn on at 30°C
        should_run = (temperature >= FLUCTUS_TEMP_TURN_ON_THRESHOLD);
    }

    if (should_run) {
        // Calculate fan duty cycle: 20% at 30°C, 100% at 40°C
        if (temperature >= FLUCTUS_TEMP_MAX_FAN_THRESHOLD) {
            fan_duty = FLUCTUS_TEMP_MAX_FAN_DUTY;  // 100%
        } else if (temperature <= FLUCTUS_TEMP_TURN_ON_THRESHOLD) {
            fan_duty = FLUCTUS_TEMP_MIN_FAN_DUTY;  // 20%
        } else {
            // Linear interpolation between 20% and 100%
            float temp_range = FLUCTUS_TEMP_MAX_FAN_THRESHOLD - FLUCTUS_TEMP_TURN_ON_THRESHOLD;
            float duty_range = FLUCTUS_TEMP_MAX_FAN_DUTY - FLUCTUS_TEMP_MIN_FAN_DUTY;
            float temp_offset = temperature - FLUCTUS_TEMP_TURN_ON_THRESHOLD;
            fan_duty = FLUCTUS_TEMP_MIN_FAN_DUTY + (uint8_t)((temp_offset / temp_range) * duty_range);
        }

        // Request 12V bus power if not already active
        if (!fan_active) {
            esp_err_t ret = fluctus_request_bus_power(POWER_BUS_12V, "FLUCTUS_FAN");
            if (ret == ESP_OK) {
                fan_active = true;
                thermal_monitoring_active = true;
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
        if (fan_active) {
            fluctus_set_cooling_fan_speed(0);
            fluctus_release_bus_power(POWER_BUS_12V, "FLUCTUS_FAN");
            fan_active = false;
            ESP_LOGI(TAG, "Cooling fan deactivated at %.1f°C", temperature);

            // Check if we should exit thermal monitoring mode
            if (temperature < FLUCTUS_TEMP_TURN_OFF_THRESHOLD) {
                thermal_monitoring_active = false;
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
static bool fluctus_should_read_temperature(bool monitoring_active)
{
    int64_t current_time_ms = esp_timer_get_time() / 1000;
    int64_t time_since_last_read = current_time_ms - last_temp_read_time;

    if (thermal_monitoring_active) {
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
 * @brief Handle temperature monitoring with smart power management
 * @param monitoring_active Whether power monitoring is currently active
 */
static void fluctus_handle_temperature_monitoring(bool monitoring_active)
{
    if (!ds18b20_found || !fluctus_should_read_temperature(monitoring_active)) {
        return;
    }

    bool bus_was_off = false;

    // Request 3.3V bus power if needed for thermal monitoring
    if (thermal_monitoring_active && !fluctus_is_bus_powered(POWER_BUS_3V3)) {
        if (fluctus_request_bus_power(POWER_BUS_3V3, "FLUCTUS_TEMP") == ESP_OK) {
            bus_was_off = true;
            vTaskDelay(pdMS_TO_TICKS(100)); // Allow bus to stabilize
        }
    }

    // Read temperature
    float temperature;
    if (fluctus_read_temperature(&temperature) == ESP_OK) {
        monitoring_data.case_temperature = temperature;
        monitoring_data.temperature_valid = true;
        monitoring_data.temperature_timestamp = time(NULL);
        last_temp_read_time = esp_timer_get_time() / 1000;

        // Update fan speed based on temperature
        fluctus_update_fan_speed(temperature);
    } else {
        monitoring_data.temperature_valid = false;
    }

    // Release 3.3V bus if we requested it
    if (bus_was_off) {
        fluctus_release_bus_power(POWER_BUS_3V3, "FLUCTUS_TEMP");
    }
}

/**
 * @brief Update hardware state for specific power bus
 *
 * Hardware control logic per bus:
 * - 3.3V: Push-pull, HIGH=ON, LOW=OFF (buffer to N-MOSFET)
 * - 5V:   Open-drain, INPUT=ON (float), OUTPUT+LOW=OFF (100kΩ to buck EN)
 * - 6.6V: Open-drain, OUTPUT+LOW=ON, INPUT=OFF (inverted logic, 100kΩ pullup)
 * - 12V:  Open-drain, INPUT=ON (float), OUTPUT+LOW=OFF (100kΩ to buck EN)
 */
static esp_err_t fluctus_update_bus_hardware(power_bus_t bus)
{
    if (bus >= POWER_BUS_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    bool enable = system_status.bus_enabled[bus] && !system_status.safety_shutdown;

    // Check if hardware state actually needs to change
    if (hardware_bus_state[bus] == enable) {
        ESP_LOGV(TAG, "Bus %d hardware already in correct state: %s", bus, enable ? "ENABLED" : "DISABLED");
        return ESP_OK;  // No change needed
    }

    esp_err_t ret = ESP_OK;

    switch (bus) {
        case POWER_BUS_3V3:
            // 3.3V: Push-pull output, HIGH=ON, LOW=OFF
            gpio_set_level(FLUCTUS_3V3_BUS_ENABLE_GPIO, enable ? 1 : 0);
            break;

        case POWER_BUS_5V:
            // 5V: Open-drain, floating=ON, LOW=OFF
            if (enable) {
                // Float the pin (INPUT mode = high-Z)
                ret = gpio_set_direction(FLUCTUS_5V_BUS_ENABLE_GPIO, GPIO_MODE_INPUT);
            } else {
                // Pull to ground (OUTPUT mode + LOW)
                ret = gpio_set_direction(FLUCTUS_5V_BUS_ENABLE_GPIO, GPIO_MODE_OUTPUT_OD);
                if (ret == ESP_OK) {
                    gpio_set_level(FLUCTUS_5V_BUS_ENABLE_GPIO, 0);
                }
            }
            break;

        case POWER_BUS_6V6:
            // 6.6V: Open-drain with inverted logic, LOW=ON, INPUT=OFF
            if (enable) {
                // Pull to ground (OUTPUT mode + LOW = ON)
                ret = gpio_set_direction(FLUCTUS_6V6_BUS_ENABLE_GPIO, GPIO_MODE_OUTPUT_OD);
                if (ret == ESP_OK) {
                    gpio_set_level(FLUCTUS_6V6_BUS_ENABLE_GPIO, 0);
                }
            } else {
                // Float the pin (INPUT mode = high-Z, pullup keeps it OFF)
                ret = gpio_set_direction(FLUCTUS_6V6_BUS_ENABLE_GPIO, GPIO_MODE_INPUT);
            }
            break;

        case POWER_BUS_12V:
            // 12V: Open-drain, floating=ON, LOW=OFF
            if (enable) {
                // Float the pin (INPUT mode = high-Z)
                ret = gpio_set_direction(FLUCTUS_12V_BUS_ENABLE_GPIO, GPIO_MODE_INPUT);
            } else {
                // Pull to ground (OUTPUT mode + LOW)
                ret = gpio_set_direction(FLUCTUS_12V_BUS_ENABLE_GPIO, GPIO_MODE_OUTPUT_OD);
                if (ret == ESP_OK) {
                    gpio_set_level(FLUCTUS_12V_BUS_ENABLE_GPIO, 0);
                }
            }
            break;

        default:
            return ESP_ERR_INVALID_ARG;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update bus %d hardware: %s", bus, esp_err_to_name(ret));
        return ret;
    }

    // Update hardware state cache
    hardware_bus_state[bus] = enable;
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
            // Limited operation - STELLARIA power save mode (max 20% intensity)
            ESP_LOGI(TAG, "Power state: POWER_SAVING - STELLARIA limited to 20%% intensity");
            stellaria_set_power_save_mode(true);   // Enable power save mode (max 20% intensity)
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
            if (system_status.solar_tracking_state != SOLAR_TRACKING_DISABLED) {
                fluctus_disable_solar_tracking();
            }
            break;
        case FLUCTUS_POWER_STATE_SHUTDOWN:
            // Emergency shutdown - disable all buses
            ESP_LOGI(TAG, "Power state: SHUTDOWN - disabling all power buses");
            for (int i = 0; i < POWER_BUS_COUNT; i++) {
                system_status.bus_enabled[i] = false;
                fluctus_update_bus_hardware(i);
            }
            system_status.solar_tracking_state = SOLAR_TRACKING_DISABLED;
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
static uint32_t fluctus_calculate_servo_correction(float error, uint32_t current_duty)
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
 * @brief Convert percentage (0-100) to servo duty cycle
 * @param percent Percentage value (0.0 = min, 100.0 = max)
 * @return Duty cycle value
 */
static uint32_t fluctus_percent_to_duty(float percent)
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
static float fluctus_duty_to_percent(uint32_t duty)
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

/**
 * @brief Park servos in night position (east-facing for morning sun)
 * 10% yaw (left/east), 60% pitch (upward)
 */
static esp_err_t fluctus_park_servos_night(void)
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

    return (ret1 == ESP_OK && ret2 == ESP_OK) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Park servos in error/center position
 */
static esp_err_t fluctus_park_servos_error(void)
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

    return (ret1 == ESP_OK && ret2 == ESP_OK) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Check if tracking errors are within acceptable threshold
 * @param yaw_error Yaw error in volts
 * @param pitch_error Pitch error in volts
 * @return true if errors are below threshold
 */
static bool fluctus_tracking_error_margin_check(float yaw_error, float pitch_error)
{
    return (fabsf(yaw_error) < FLUCTUS_PHOTORESISTOR_THRESHOLD &&
            fabsf(pitch_error) < FLUCTUS_PHOTORESISTOR_THRESHOLD);
}

/**
 * @brief Read all INA219 sensors and update monitoring data
 */
static esp_err_t fluctus_read_ina219_sensors(void)
{
    esp_err_t ret = ESP_OK;
    
    // Read Solar PV sensor
    float voltage, current, power;
    if (ina219_get_bus_voltage(&ina219_dev[0], &voltage) == ESP_OK &&
        ina219_get_current(&ina219_dev[0], &current) == ESP_OK &&
        ina219_get_power(&ina219_dev[0], &power) == ESP_OK) {
        
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
    if (ina219_get_bus_voltage(&ina219_dev[1], &voltage) == ESP_OK &&
        ina219_get_current(&ina219_dev[1], &current) == ESP_OK &&
        ina219_get_power(&ina219_dev[1], &power) == ESP_OK) {
        
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
    
    // Check immediate shutdown threshold (3.5A)
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
    
    // Check delayed shutdown threshold (3A for 3 seconds)
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
    
    // Read 3V3 bus voltage from ADS1115 device #2, Channel 3
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
    const char* bus_names[] = {"3V3", "5V", "6V6", "12V"};
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
    
    const char* bus_names[] = {"3V3", "5V", "6V6", "12V", "SYSTEM"};
    
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
 * @brief Set INA219 power mode (active or power-down)
 * @param device_index INA219 device index (0 = PV, 1 = Battery)
 * @param active true to activate, false to power down
 * @return ESP_OK on success
 */
static esp_err_t fluctus_ina219_set_power_mode(uint8_t device_index, bool active)
{
    if (device_index >= FLUCTUS_INA219_DEVICE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    // INA219 operating mode bits (bits 0-2 of config register)
    // 0b000 = Power-down, 0b111 = Continuous shunt+bus
    ina219_mode_t mode = active ? INA219_MODE_CONT_SHUNT_BUS : INA219_MODE_POWER_DOWN;

    // Reconfigure with new mode (preserve other settings)
    esp_err_t ret = ina219_configure(&ina219_dev[device_index],
                                    INA219_BUS_RANGE_32V,
                                    device_index == 0 ? INA219_GAIN_0_5 : INA219_GAIN_0_125,
                                    INA219_RES_12BIT_8S,
                                    INA219_RES_12BIT_8S,
                                    mode);

    if (ret == ESP_OK) {
        const char *ina_name = (device_index == 0) ? "PV" : "Battery";
        ESP_LOGD(TAG, "INA219 %s %s", ina_name, active ? "activated" : "powered down");
    }

    return ret;
}

/**
 * @brief Update power metering state based on system activity
 * Manages PV day/night scheduling and Battery active/steady state logic
 * @param monitoring_active Is power monitoring currently active?
 */
static void fluctus_update_power_metering_state(bool monitoring_active)
{
    // Get averaged photoresistor reading for day/night detection
    float avg_light = 0.0f;
    if (system_status.solar_tracking_state != SOLAR_TRACKING_DISABLED) {
        // Use solar tracking photoresistor data if available
        fluctus_solar_tracking_data_t tracking_data;
        if (fluctus_read_photoresistors(&tracking_data) == ESP_OK && tracking_data.valid) {
            avg_light = (tracking_data.photoresistor_readings[0] +
                        tracking_data.photoresistor_readings[1] +
                        tracking_data.photoresistor_readings[2] +
                        tracking_data.photoresistor_readings[3]) / 4.0f;
        }
    }

    // Determine if PV INA should be active (daytime check)
    bool should_pv_be_active = telemetry_should_pv_be_active(avg_light);

    // Update PV INA power state
    if (should_pv_be_active != pv_ina_powered) {
        if (fluctus_ina219_set_power_mode(0, should_pv_be_active) == ESP_OK) {
            pv_ina_powered = should_pv_be_active;
            monitoring_data.pv_ina_active = pv_ina_powered;
            ESP_LOGI(TAG, "PV INA219 %s (%s)", should_pv_be_active ? "activated" : "powered down",
                     should_pv_be_active ? "daytime" : "nighttime");
        }
    }

    // Determine Battery INA state based on system activity
    bool any_bus_active = false;
    bool stellaria_only = false;

    for (int i = 0; i < POWER_BUS_COUNT; i++) {
        if (system_status.bus_enabled[i]) {
            any_bus_active = true;
            break;
        }
    }

    // Check for Stellaria-only scenario
    if (system_status.bus_enabled[POWER_BUS_12V] &&
        system_status.bus_ref_count[POWER_BUS_12V] == 1 &&
        !system_status.bus_enabled[POWER_BUS_3V3] &&
        !system_status.bus_enabled[POWER_BUS_5V] &&
        !system_status.bus_enabled[POWER_BUS_6V6]) {
        // Only 12V bus active with 1 consumer = likely Stellaria
        stellaria_only = true;
    }

    // Get recommended metering state from telemetry
    power_meter_state_t recommended_state = telemetry_get_recommended_state(any_bus_active, stellaria_only);

    // Handle Battery INA power state based on recommended state
    if (recommended_state == POWER_METER_STATE_STEADY) {
        // Steady state: Probe for 3 seconds every 60 seconds
        int64_t current_time_ms = esp_timer_get_time() / 1000;

        if (steady_state_probe_start == 0) {
            // Start new probe cycle
            steady_state_probe_start = current_time_ms;
            if (!battery_ina_powered) {
                fluctus_ina219_set_power_mode(1, true);
                battery_ina_powered = true;
                monitoring_data.battery_ina_active = true;
                ESP_LOGD(TAG, "Battery INA: Starting steady state probe");
            }
        } else if ((current_time_ms - steady_state_probe_start) < FLUCTUS_POWER_STEADY_PROBE_DURATION_MS) {
            // Within probe duration - keep active
            if (!battery_ina_powered) {
                fluctus_ina219_set_power_mode(1, true);
                battery_ina_powered = true;
                monitoring_data.battery_ina_active = true;
            }
        } else if ((current_time_ms - steady_state_probe_start) < FLUCTUS_POWER_STEADY_MONITOR_INTERVAL_MS) {
            // Between probes - power down
            if (battery_ina_powered) {
                fluctus_ina219_set_power_mode(1, false);
                battery_ina_powered = false;
                monitoring_data.battery_ina_active = false;
                ESP_LOGD(TAG, "Battery INA: Ending steady state probe");
            }
        } else {
            // Cycle complete - reset for next probe
            steady_state_probe_start = 0;
        }
    } else {
        // Active state: Keep Battery INA powered
        if (!battery_ina_powered) {
            fluctus_ina219_set_power_mode(1, true);
            battery_ina_powered = true;
            monitoring_data.battery_ina_active = true;
            ESP_LOGD(TAG, "Battery INA: Activated for active monitoring");
        }
        steady_state_probe_start = 0;  // Reset steady state timer
    }
}

/**
 * @brief Integrate power sample into telemetry system
 */
static void fluctus_integrate_telemetry_sample(void)
{
    if (!monitoring_data.solar_pv.valid || !monitoring_data.battery_out.valid) {
        return;
    }

    telemetry_update_power_sample(
        monitoring_data.solar_pv.voltage,
        monitoring_data.solar_pv.current,
        monitoring_data.solar_pv.power,
        monitoring_data.battery_out.voltage,
        monitoring_data.battery_out.current,
        monitoring_data.battery_out.power,
        pv_ina_powered,
        battery_ina_powered
    );
}

/**
 * @brief Power monitoring FreeRTOS task
 */
static void fluctus_monitoring_task(void *parameters)
{
    ESP_LOGI(TAG, "Power monitoring task started");

    bool monitoring_active = false;
    static uint32_t last_telemetry_update = 0;  // Track last state injection

    while (true) {
        uint32_t notification_value = 0;
        TickType_t timeout;
        
        // Determine timeout based on monitoring state
        if (monitoring_active) {
            // Active monitoring - short timeout for continuous monitoring
            timeout = pdMS_TO_TICKS(FLUCTUS_POWER_ACTIVE_MONITOR_INTERVAL_MS);
        } else {
            // Idle monitoring - long timeout for periodic checks
            timeout = pdMS_TO_TICKS(FLUCTUS_POWER_PERIODIC_CHECK_INTERVAL_MS);
        }
        
        // Wait for notification or timeout
        BaseType_t notified = xTaskNotifyWait(0x00,              // Don't clear on entry
                                              ULONG_MAX,          // Clear all on exit
                                              &notification_value, // Store notification
                                              timeout);            // Timeout
        
        // Handle notification
        if (notified == pdTRUE && (notification_value & FLUCTUS_NOTIFY_POWER_ACTIVITY)) {
            ESP_LOGI(TAG, "Power activity notification received - starting active monitoring");
            monitoring_active = true;
        }
        
        // Update power metering state (INA power-down management)
        fluctus_update_power_metering_state(monitoring_active);

        // Read power sensors
        if (xSemaphoreTake(xFluctusMonitoringMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
            fluctus_read_ina219_sensors();
            fluctus_check_3v3_bus_voltage(); // Check 3.3V bus voltage
            fluctus_handle_temperature_monitoring(monitoring_active); // Temperature monitoring with smart scheduling

            // Integrate power samples into telemetry system
            fluctus_integrate_telemetry_sample();

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

            // Inject state data to TELEMETRY at periodic intervals (15 minutes)
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - last_telemetry_update >= FLUCTUS_POWER_PERIODIC_CHECK_INTERVAL_MS) {
                telemetry_update_fluctus_state();
                last_telemetry_update = now;
            }
        }
    }
}

/**
 * @brief Read photoresistors, calculate average light, and update Stellaria
 * @return Average light level in volts (0.0f on error)
 */
static float fluctus_read_avg_light_update_stellaria(void)
{
    fluctus_solar_tracking_data_t tracking_data;
    if (fluctus_read_photoresistors(&tracking_data) != ESP_OK || !tracking_data.valid) {
        return 0.0f;
    }

    float avg_light = (tracking_data.photoresistor_readings[0] +
                      tracking_data.photoresistor_readings[1] +
                      tracking_data.photoresistor_readings[2] +
                      tracking_data.photoresistor_readings[3]) / 4.0f;
    stellaria_update_light_reading(avg_light);
    return avg_light;
}

/**
 * @brief Thread-safe state transition with logging
 * @param new_state Target state
 * @param log_msg Log message (NULL to skip logging)
 */
static void fluctus_set_tracking_state(solar_tracking_state_t new_state, const char *log_msg)
{
    if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        system_status.solar_tracking_state = new_state;
        if (log_msg) {
            ESP_LOGI(TAG, "%s", log_msg);
        }
        xSemaphoreGive(xFluctusPowerMutex);
    }
}

/**
 * @brief Apply servo corrections based on tracking data
 * @param tracking_data Tracking data with yaw/pitch errors
 * @return true if servos were updated
 */
static bool fluctus_apply_servo_corrections(fluctus_solar_tracking_data_t *tracking_data)
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

/**
 * @brief Solar tracking task with state machine and automatic scheduling
 */
/**
 * TODO: MAJOR SOLAR TRACKING REDESIGN NEEDED
 *
 * Current implementation uses constant 5-second polling which wastes CPU and power.
 *
 * REDESIGN GOALS:
 * 1. Use task notifications (similar to fluctus_monitoring_task) instead of constant polling
 * 2. Implement variable delay based on state:
 *    - PAUSED: 15-minute intervals (FLUCTUS_TRACKING_CORRECTION_INTERVAL_MS)
 *    - CORRECTING: 5-second updates during active correction
 *    - PARKING: 1-second updates during parking sequence
 *    - DISABLED: Long sleep (1+ hour) until re-enable notification
 *
 * 3. Add notification triggers:
 *    - FLUCTUS_NOTIFY_SOLAR_ENABLE: Wake from DISABLED state
 *    - FLUCTUS_NOTIFY_SOLAR_DISABLE: Enter DISABLED state
 *    - Timer-based wake for 15-minute correction cycles
 *
 * 4. Integrate telemetry injection points:
 *    - Call telemetry_update_fluctus_solar() during CORRECTING state (real-time updates)
 *    - Call at end of each correction cycle
 *
 * 5. Benefits:
 *    - Reduced CPU usage (no constant 5s wake-ups)
 *    - Lower power consumption when paused
 *    - Cleaner state machine logic
 *    - Real-time telemetry during active tracking
 *
 * DEFER THIS REDESIGN until after telemetry infrastructure is working.
 */
static void fluctus_solar_tracking_task(void *parameters)
{
    ESP_LOGI(TAG, "Solar tracking task started");

    while (true) {
        int64_t current_time_ms = esp_timer_get_time() / 1000;
        time_t now = time(NULL);

        // Get solar times for scheduling
        const solar_times_t *solar_times = solar_calc_get_times();
        time_t tracking_start_time = solar_times->sunrise_time - (FLUCTUS_TRACKING_SUNRISE_BUFFER_MINUTES * 60);
        time_t tracking_end_time = solar_times->sunset_time - (FLUCTUS_TRACKING_SUNSET_BUFFER_MINUTES * 60);

        solar_tracking_state_t current_state = SOLAR_TRACKING_DISABLED;

        if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
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

            xSemaphoreGive(xFluctusPowerMutex);
        }

        // Main state machine
        switch (current_state) {
            case SOLAR_TRACKING_DISABLED:
                // Tracking disabled - just sleep
                vTaskDelay(pdMS_TO_TICKS(5000));
                break;

            case SOLAR_TRACKING_PAUSED: {
                // Check if we're within tracking window
                bool within_window = (now >= tracking_start_time && now < tracking_end_time);

                // Read photoresistors and update Stellaria
                float avg_light = fluctus_read_avg_light_update_stellaria();

                // Check for sunset parking condition
                if (now >= tracking_end_time && avg_light < FLUCTUS_PV_LIGHT_THRESHOLD) {
                    fluctus_set_tracking_state(SOLAR_TRACKING_PARKING, "Sunset reached and low light - parking for night");
                    break;
                }

                // Check if we should start correction cycle
                if (within_window &&
                    (last_correction_time == 0 ||
                     (current_time_ms - last_correction_time) >= FLUCTUS_TRACKING_CORRECTION_INTERVAL_MS)) {

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

                vTaskDelay(pdMS_TO_TICKS(5000));
                break;
            }

            case SOLAR_TRACKING_CORRECTING: {
                // Read photoresistors
                fluctus_solar_tracking_data_t tracking_data;
                if (fluctus_read_photoresistors(&tracking_data) != ESP_OK || !tracking_data.valid) {
                    ESP_LOGW(TAG, "Failed to read photoresistors during correction");
                    vTaskDelay(pdMS_TO_TICKS(FLUCTUS_TRACKING_INTERVAL_MS));
                    break;
                }

                // Update STELLARIA
                stellaria_update_light_reading((tracking_data.photoresistor_readings[0] +
                                               tracking_data.photoresistor_readings[1] +
                                               tracking_data.photoresistor_readings[2] +
                                               tracking_data.photoresistor_readings[3]) / 4.0f);

                // Check if errors are acceptable (correction complete)
                if (fluctus_tracking_error_margin_check(tracking_data.yaw_error, tracking_data.pitch_error)) {
                    fluctus_release_bus_power(POWER_BUS_6V6, "FLUCTUS_SOLAR");
                    fluctus_set_tracking_state(SOLAR_TRACKING_PAUSED, "Correction complete - errors within threshold (6.6V bus powered off)");
                    break;
                }

                // Check for timeout
                int64_t correction_duration = current_time_ms - correction_start_time;
                if (correction_duration >= FLUCTUS_TRACKING_CORRECTION_TIMEOUT_MS) {
                    fluctus_release_bus_power(POWER_BUS_6V6, "FLUCTUS_SOLAR");
                    ESP_LOGW(TAG, "Correction timeout after %lld ms - entering error state (6.6V bus powered off)", correction_duration);
                    fluctus_set_tracking_state(SOLAR_TRACKING_ERROR, NULL);
                    break;
                }

                // Apply servo corrections
                fluctus_apply_servo_corrections(&tracking_data);

                vTaskDelay(pdMS_TO_TICKS(FLUCTUS_TRACKING_INTERVAL_MS));
                break;
            }

            case SOLAR_TRACKING_PARKING:
                fluctus_park_servos_night();
                fluctus_release_bus_power(POWER_BUS_6V6, "FLUCTUS_SOLAR");
                fluctus_set_tracking_state(SOLAR_TRACKING_DISABLED, "Parked for night and released servo power");
                break;

            case SOLAR_TRACKING_ERROR:
                fluctus_park_servos_error();
                fluctus_set_tracking_state(SOLAR_TRACKING_PAUSED, "Error recovery - parked in center, will retry on next cycle");
                break;

            default:
                vTaskDelay(pdMS_TO_TICKS(5000));
                break;
        }
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

    // Initialize DS18B20 temperature sensor (non-critical - warn but continue on failure)
    ret = fluctus_ds18b20_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "DS18B20 initialization failed - temperature monitoring disabled");
        // Don't goto cleanup - continue without temperature monitoring
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
            
            // Notify monitoring task of power activity to start constant monitoring
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

        // Enable tracking in paused state (6.6V bus will be toggled during correction cycles)
        system_status.solar_tracking_state = SOLAR_TRACKING_PAUSED;
        ESP_LOGI(TAG, "Solar tracking enabled (paused state, bus will power on during corrections)");

        xSemaphoreGive(xFluctusPowerMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t fluctus_disable_solar_tracking(void)
{
    if (!fluctus_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
        if (system_status.solar_tracking_state != SOLAR_TRACKING_DISABLED) {
            solar_tracking_state_t current_state = system_status.solar_tracking_state;
            bool bus_was_off = (current_state == SOLAR_TRACKING_PAUSED ||
                               current_state == SOLAR_TRACKING_ERROR ||
                               current_state == SOLAR_TRACKING_DISABLED);

            xSemaphoreGive(xFluctusPowerMutex);

            // Request bus power if it's currently off (for parking servos)
            if (bus_was_off) {
                fluctus_request_bus_power(POWER_BUS_6V6, "FLUCTUS_SOLAR");
                vTaskDelay(pdMS_TO_TICKS(FLUCTUS_SERVO_POWERUP_DELAY_MS));
            }

            // Park servos in night position (east-facing for morning sun)
            fluctus_park_servos_night();

            // Always release servo bus power
            fluctus_release_bus_power(POWER_BUS_6V6, "FLUCTUS_SOLAR");

            // Update state
            if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
                system_status.solar_tracking_state = SOLAR_TRACKING_DISABLED;
                ESP_LOGI(TAG, "Solar tracking disabled and servos parked in night position");
                xSemaphoreGive(xFluctusPowerMutex);
            }

            return ESP_OK;
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
        enabled = (system_status.solar_tracking_state != SOLAR_TRACKING_DISABLED) &&
                  !system_status.safety_shutdown;
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

const fluctus_monitoring_data_t* fluctus_get_monitoring_data_ptr(void)
{
    if (!fluctus_initialized) {
        return NULL;
    }
    
    // Note: Caller must handle mutex if needed for thread safety
    // For most read-only access, the risk is minimal with atomic reads
    return &monitoring_data;
}

// TODO: Endpoint for passing data further down the line?
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

float fluctus_get_case_temperature(void)
{
    if (!fluctus_initialized) {
        return -127.0f;
    }

    float temperature = -127.0f;
    if (xSemaphoreTake(xFluctusMonitoringMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        if (monitoring_data.temperature_valid) {
            temperature = monitoring_data.case_temperature;
        }
        xSemaphoreGive(xFluctusMonitoringMutex);
    }

    return temperature;
}

/**
 * @brief Calculate battery SOC percentage from voltage (12V AGM battery)
 */
static float calculate_battery_soc(float voltage)
{
    // SOC mapping for 12V AGM battery (50% SOC = 0% displayed)
    // 12.08V = 0% (critical), 12.58V = 100% (power saving threshold)
    if (voltage < FLUCTUS_BATTERY_LEVEL_CRITICAL) {
        return 0.0f;
    } else if (voltage >= FLUCTUS_BATTERY_LEVEL_POWER_SAVING_ON) {
        return 100.0f;
    }

    // Linear interpolation between critical and power saving
    float voltage_range = FLUCTUS_BATTERY_LEVEL_POWER_SAVING_ON - FLUCTUS_BATTERY_LEVEL_CRITICAL;
    float voltage_offset = voltage - FLUCTUS_BATTERY_LEVEL_CRITICAL;
    return (voltage_offset / voltage_range) * 100.0f;
}

esp_err_t fluctus_get_data_snapshot(fluctus_snapshot_t *snapshot)
{
    if (snapshot == NULL || !fluctus_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    // Take both mutexes for comprehensive snapshot
    if (xSemaphoreTake(xFluctusPowerMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) != pdTRUE) {
        return ESP_FAIL;
    }

    if (xSemaphoreTake(xFluctusMonitoringMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) != pdTRUE) {
        xSemaphoreGive(xFluctusPowerMutex);
        return ESP_FAIL;
    }

    // Power buses
    snapshot->bus_3v3_enabled = system_status.bus_enabled[POWER_BUS_3V3];
    snapshot->bus_5v_enabled = system_status.bus_enabled[POWER_BUS_5V];
    snapshot->bus_6v6_enabled = system_status.bus_enabled[POWER_BUS_6V6];
    snapshot->bus_12v_enabled = system_status.bus_enabled[POWER_BUS_12V];

    snapshot->bus_3v3_consumers = system_status.bus_ref_count[POWER_BUS_3V3];
    snapshot->bus_5v_consumers = system_status.bus_ref_count[POWER_BUS_5V];
    snapshot->bus_6v6_consumers = system_status.bus_ref_count[POWER_BUS_6V6];
    snapshot->bus_12v_consumers = system_status.bus_ref_count[POWER_BUS_12V];

    // Battery monitoring
    snapshot->battery_voltage = monitoring_data.battery_voltage;
    snapshot->battery_current = monitoring_data.battery_out.current;
    snapshot->battery_soc_percent = calculate_battery_soc(monitoring_data.battery_voltage);
    snapshot->battery_power_w = monitoring_data.battery_out.power;
    snapshot->battery_data_valid = monitoring_data.battery_out.valid;

    // Solar monitoring
    snapshot->solar_voltage = monitoring_data.solar_pv.voltage;
    snapshot->solar_current = monitoring_data.solar_pv.current;
    snapshot->solar_power_w = monitoring_data.solar_pv.power;
    snapshot->solar_pv_active = monitoring_data.pv_ina_active;
    snapshot->solar_data_valid = monitoring_data.solar_pv.valid;

    // Solar tracking
    snapshot->tracking_state = system_status.solar_tracking_state;
    snapshot->yaw_position_percent = (uint8_t)fluctus_duty_to_percent(system_status.current_yaw_duty);
    snapshot->pitch_position_percent = (uint8_t)fluctus_duty_to_percent(system_status.current_pitch_duty);

    // Get tracking errors from ADS1115 photoresistors
    float photoresistor_readings[4];
    int16_t raw_values[4];
    esp_err_t ads_ret = ads1115_read_channel(0, ADS111X_MUX_0_GND, &raw_values[0], &photoresistor_readings[0]);
    ads_ret |= ads1115_read_channel(0, ADS111X_MUX_1_GND, &raw_values[1], &photoresistor_readings[1]);
    ads_ret |= ads1115_read_channel(0, ADS111X_MUX_2_GND, &raw_values[2], &photoresistor_readings[2]);
    ads_ret |= ads1115_read_channel(0, ADS111X_MUX_3_GND, &raw_values[3], &photoresistor_readings[3]);

    if (ads_ret == ESP_OK) {
        float left_avg = (photoresistor_readings[0] + photoresistor_readings[2]) / 2.0f;
        float right_avg = (photoresistor_readings[1] + photoresistor_readings[3]) / 2.0f;
        float top_avg = (photoresistor_readings[0] + photoresistor_readings[1]) / 2.0f;
        float bottom_avg = (photoresistor_readings[2] + photoresistor_readings[3]) / 2.0f;

        snapshot->yaw_error = left_avg - right_avg;
        snapshot->pitch_error = top_avg - bottom_avg;
    } else {
        snapshot->yaw_error = 0.0f;
        snapshot->pitch_error = 0.0f;
    }

    // Thermal management
    snapshot->case_temperature = monitoring_data.case_temperature;
    snapshot->temperature_valid = monitoring_data.temperature_valid;

    // Calculate fan speed percentage
    if (fan_active) {
        // Linear interpolation between turn-on and max thresholds
        if (monitoring_data.case_temperature >= FLUCTUS_TEMP_MAX_FAN_THRESHOLD) {
            snapshot->fan_speed_percent = 100;
        } else if (monitoring_data.case_temperature <= FLUCTUS_TEMP_TURN_ON_THRESHOLD) {
            snapshot->fan_speed_percent = FLUCTUS_TEMP_MIN_FAN_DUTY;
        } else {
            float temp_range = FLUCTUS_TEMP_MAX_FAN_THRESHOLD - FLUCTUS_TEMP_TURN_ON_THRESHOLD;
            float temp_offset = monitoring_data.case_temperature - FLUCTUS_TEMP_TURN_ON_THRESHOLD;
            float duty_range = FLUCTUS_TEMP_MAX_FAN_DUTY - FLUCTUS_TEMP_MIN_FAN_DUTY;
            snapshot->fan_speed_percent = FLUCTUS_TEMP_MIN_FAN_DUTY +
                (uint8_t)((temp_offset / temp_range) * duty_range);
        }
    } else {
        snapshot->fan_speed_percent = 0;
    }

    // System state
    snapshot->power_state = system_status.power_state;
    snapshot->safety_shutdown = system_status.safety_shutdown;
    snapshot->manual_reset_required = system_status.manual_reset_required;
    snapshot->last_activity_time = system_status.last_activity_time;
    snapshot->snapshot_timestamp = time(NULL);

    xSemaphoreGive(xFluctusMonitoringMutex);
    xSemaphoreGive(xFluctusPowerMutex);

    return ESP_OK;
}

const fluctus_power_status_t* fluctus_get_system_status_ptr(void)
{
    if (!fluctus_initialized) {
        return NULL;
    }
    
    // Note: Caller must handle mutex if needed for thread safety
    // For most read-only access, the risk is minimal with atomic reads
    return &system_status;
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

esp_err_t fluctus_set_pv_ina_power(bool active)
{
    if (!fluctus_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xFluctusMonitoringMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        esp_err_t ret = fluctus_ina219_set_power_mode(0, active);
        if (ret == ESP_OK) {
            pv_ina_powered = active;
            monitoring_data.pv_ina_active = active;
            ESP_LOGI(TAG, "PV INA219 manually %s", active ? "activated" : "powered down");
        }
        xSemaphoreGive(xFluctusMonitoringMutex);
        return ret;
    }

    return ESP_FAIL;
}

esp_err_t fluctus_set_battery_ina_power(bool active)
{
    if (!fluctus_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xFluctusMonitoringMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        esp_err_t ret = fluctus_ina219_set_power_mode(1, active);
        if (ret == ESP_OK) {
            battery_ina_powered = active;
            monitoring_data.battery_ina_active = active;
            ESP_LOGI(TAG, "Battery INA219 manually %s", active ? "activated" : "powered down");
        }
        xSemaphoreGive(xFluctusMonitoringMutex);
        return ret;
    }

    return ESP_FAIL;
}