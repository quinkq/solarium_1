/**
 * @file fluctus.c
 * @brief Power management and solar tracking system
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Original implementation of FLUCTUS - the central power management system.
 *
 * Key features:
 * - Four-bus power distribution (3.3V/5V/6.6V/12V) with reference counting
 * - Five-stage load shedding coordinated across all components
 * - Dual INA219 power monitoring (solar PV and battery)
 * - Dual-axis solar tracking with NOAA position calculations
 * - Adaptive power metering (500ms to 15min intervals)
 * - Sensor-level power gating for maximum efficiency
 * - Thermal management with PWM fan control
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "fluctus.h"
#include "stellaria.h"
#include "impluvium.h"
#include "tempesta.h"
#include "telemetry.h"
#include "solar_calc.h"
#include "wifi_helper.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>
#include <time.h>
#include "ads111x.h"
#include "ads1115_helper.h"
#include "ds18x20.h"

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

// Cached solar tracking data (populated by photoresistor reads)
static fluctus_solar_snapshot_t cached_solar_data = {0};

// RTC RAM power accumulator (survives ESP32 resets, persists across deep sleep)
// Migrated from telemetry.c (v3.3) - FLUCTUS owns all power domain logic
static RTC_DATA_ATTR power_accumulator_rtc_t rtc_accumulator = {0};

// INA219 device configurations
static ina219_t ina219_dev[FLUCTUS_INA219_DEVICE_COUNT];

// Mutex for thread safety (single mutex protects all fluctus data)
static SemaphoreHandle_t xFluctusMutex = NULL;

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
static int64_t last_temp_read_time = 0;

// DS18B20 async conversion state machine
typedef enum {
    TEMP_STATE_IDLE,        // No conversion in progress
    TEMP_STATE_CONVERTING   // Conversion started, waiting for completion
} temp_conversion_state_t;

static temp_conversion_state_t temp_state = TEMP_STATE_IDLE;
static int64_t temp_conversion_start_time = 0;

// Power metering state variables
static int64_t steady_state_probe_start = 0;  // Steady state probe timer

// Solar tracking state variables
static int64_t last_correction_time = 0;      // Last correction cycle start time
static int64_t correction_start_time = 0;     // Current correction cycle start time

// Parking reason tracker (determines SLEEPING vs DISABLED after parking)
typedef enum {
    PARKING_REASON_SUNSET,          // Natural sunset - transition to SLEEPING
    PARKING_REASON_USER_DISABLE,    // Manual disable - transition to DISABLED
    PARKING_REASON_CRITICAL_POWER   // Critical power - transition to DISABLED
} parking_reason_t;
static parking_reason_t current_parking_reason = PARKING_REASON_SUNSET;

// Error tracking for solar tracking
static uint8_t consecutive_tracking_errors = 0;
#define FLUCTUS_MAX_CONSECUTIVE_ERRORS 5

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
static esp_err_t fluctus_read_photoresistors(fluctus_solar_snapshot_t *data);
static esp_err_t fluctus_servo_set_position(ledc_channel_t channel, uint32_t duty_cycle);
static uint32_t fluctus_calculate_servo_correction(float error, uint32_t current_duty);
static uint32_t fluctus_percent_to_duty(float percent);
static esp_err_t fluctus_park_servos_night(void);
static esp_err_t fluctus_park_servos_error(void);
static bool fluctus_tracking_error_margin_check(float yaw_error, float pitch_error);
static void fluctus_set_tracking_state(solar_tracking_state_t new_state, const char *log_msg);
static bool fluctus_apply_servo_corrections(fluctus_solar_snapshot_t *tracking_data);
static bool fluctus_is_daytime_with_sufficient_light(void);

// Solar tracking state handlers
static void fluctus_solar_state_standby(int64_t current_time_ms);
static void fluctus_solar_state_correcting(int64_t current_time_ms);
static void fluctus_solar_state_parking(void);
static void fluctus_solar_state_error(void);

// Monitoring functions
static esp_err_t fluctus_read_ina219_sensors(void);
static void fluctus_check_overcurrent(void);
static void fluctus_check_3v3_bus_voltage(void);
static void fluctus_log_active_consumers(const char* event_context);
static void fluctus_log_power_event(fluctus_event_type_t event_type, power_bus_t bus,
                                   const char* consumer_id);

// DS18B20 temperature sensor functions

static esp_err_t fluctus_ds18b20_init(void);
static void fluctus_update_fan_speed(float temperature);
static bool fluctus_should_read_temperature(bool monitoring_active);
static void fluctus_handle_temperature_monitoring(bool monitoring_active);

// Power metering functions

static esp_err_t fluctus_ina219_set_power_mode(uint8_t device_index, bool active);
static void fluctus_set_ina_metering_mode(bool monitoring_active);

// Energy tracking functions

static void fluctus_update_energy_accumulator(float pv_power_w, float battery_power_w);
static bool fluctus_check_hourly_rollover(void);
static void fluctus_midnight_callback(void);

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

    // Hall array MOSFET control: Push-pull output, LOW=OFF, HIGH=ON (N-MOSFET with external pull-down)
    gpio_config_t hall_mosfet_conf = {
        .pin_bit_mask = (1ULL << FLUCTUS_3V3_HALL_ARRAY_ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,  // Push-pull output
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // External pull-down on gate
        .intr_type = GPIO_INTR_DISABLE
    };

    ret = gpio_config(&hall_mosfet_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure hall array MOSFET GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(FLUCTUS_3V3_HALL_ARRAY_ENABLE_GPIO, 0);  // Start LOW (OFF)
    ESP_LOGI(TAG, "Hall array MOSFET control initialized (GPIO%d, LOW=OFF)", FLUCTUS_3V3_HALL_ARRAY_ENABLE_GPIO);

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
 * @brief Update cooling fan speed based on temperature
 * @param temperature Current case temperature in °C
 */
static void fluctus_update_fan_speed(float temperature)
{
    bool should_run = false;
    uint8_t fan_duty = 0;

    // Hysteresis logic
    if (monitoring_data.fan_active) {
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
        if (!monitoring_data.fan_active) {
            esp_err_t ret = fluctus_request_bus_power(POWER_BUS_12V, "FLUCTUS_FAN");
            if (ret == ESP_OK) {
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
            fluctus_release_bus_power(POWER_BUS_12V, "FLUCTUS_FAN");
            monitoring_data.fan_active = false;
            ESP_LOGI(TAG, "Cooling fan deactivated at %.1f°C", temperature);

            // Check if we should exit thermal monitoring mode
            if (temperature < FLUCTUS_TEMP_TURN_OFF_THRESHOLD) {
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
static bool fluctus_should_read_temperature(bool monitoring_active)
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
static void fluctus_handle_temperature_monitoring(bool monitoring_active)
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
            wifi_helper_set_power_save_mode(false);              // WiFi normal mode (~100mA)
            wifi_helper_set_shutdown(false);                     // Ensure WiFi enabled
            telemetry_enable_telemetry_publishing(true);         // Enable normal MQTT publishing
            telemetry_force_realtime_monitoring_disable(false);  // Restore realtime mode user preference
            break;

        case FLUCTUS_POWER_STATE_POWER_SAVING:
            // Limited operation - STELLARIA power save mode (max 20% intensity)
            ESP_LOGI(TAG, "Power state: POWER_SAVING - STELLARIA limited to 20%% intensity");
            stellaria_set_power_save_mode(true);   // Enable power save mode (max 12% intensity)
            stellaria_set_shutdown(false);         // Ensure not in shutdown
            impluvium_set_power_save_mode(true);   // 60min moisture check interval
            impluvium_set_shutdown(false);         // Keep irrigation operational
            tempesta_set_power_save_mode(false);   // Keep normal weather collection
            tempesta_set_shutdown(false);          // Keep weather monitoring
            tempesta_set_pms5003_enabled(true);    // Keep air quality sensor enabled
            wifi_helper_set_power_save_mode(false);              // Keep WiFi normal mode
            wifi_helper_set_shutdown(false);                     // Keep WiFi enabled
            telemetry_enable_telemetry_publishing(true);         // Keep normal MQTT publishing enabled
            telemetry_force_realtime_monitoring_disable(false);  // Keep realtime mode at user preference
            break;

        case FLUCTUS_POWER_STATE_LOW_POWER:
            // Reduced operation - STELLARIA shutdown, TEMPESTA power save, realtime telemetry disabled
            ESP_LOGI(TAG, "Power state: LOW_POWER - STELLARIA disabled, TEMPESTA power save, realtime telemetry off, WiFi power save");
            stellaria_set_shutdown(true);          // Shutdown STELLARIA
            impluvium_set_power_save_mode(true);   // Keep 60min moisture check interval
            impluvium_set_shutdown(false);         // Keep irrigation operational
            tempesta_set_power_save_mode(true);    // 60min weather collection interval
            tempesta_set_shutdown(false);          // Keep weather monitoring
            tempesta_set_pms5003_enabled(true);    // Keep air quality sensor enabled
            wifi_helper_set_power_save_mode(true);               // WiFi modem power save (~20mA avg)
            wifi_helper_set_shutdown(false);                     // Keep WiFi enabled for MQTT
            telemetry_enable_telemetry_publishing(true);        // Keep normal MQTT publishing
            telemetry_force_realtime_monitoring_disable(true);  // Force disable realtime mode (SOC <25%)
            break;

        case FLUCTUS_POWER_STATE_VERY_LOW:
            // Minimal operation - STELLARIA + IMPLUVIUM shutdown, TEMPESTA skip PMS5003, MQTT buffering only
            ESP_LOGI(TAG, "Power state: VERY_LOW - STELLARIA and IMPLUVIUM disabled, MQTT buffering only, WiFi power save");
            stellaria_set_shutdown(true);          // Shutdown STELLARIA
            impluvium_set_shutdown(true);          // Shutdown all irrigation operations
            tempesta_set_power_save_mode(true);    // 60min weather collection interval
            tempesta_set_shutdown(false);          // Keep weather monitoring
            tempesta_set_pms5003_enabled(false);   // Skip air quality sensor to save power
            wifi_helper_set_power_save_mode(true);               // WiFi modem power save (~20mA avg)
            wifi_helper_set_shutdown(false);                     // Keep WiFi for MQTT buffering
            telemetry_enable_telemetry_publishing(false);       // Buffering only (SOC <15%)
            telemetry_force_realtime_monitoring_disable(true);  // Force disable realtime mode
            break;

        case FLUCTUS_POWER_STATE_CRITICAL:
            // Emergency operation - only essential systems (TEMPESTA + solar tracking + WiFi off), MQTT buffering only
            ESP_LOGI(TAG, "Power state: CRITICAL - TEMPESTA, solar tracking, and WiFi disabled, MQTT buffering only");
            stellaria_set_shutdown(true);          // Shutdown STELLARIA
            impluvium_set_shutdown(true);          // Shutdown all irrigation operations
            tempesta_set_shutdown(true);           // Shutdown weather monitoring
            wifi_helper_set_shutdown(true);                      // Shutdown WiFi (save ~20mA)
            telemetry_enable_telemetry_publishing(false);       // Buffering only (critical power)
            telemetry_force_realtime_monitoring_disable(true);  // Force disable realtime mode
            if (system_status.solar_tracking_state != SOLAR_TRACKING_DISABLED) {
                current_parking_reason = PARKING_REASON_CRITICAL_POWER;
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
static esp_err_t fluctus_read_photoresistors(fluctus_solar_snapshot_t *data)
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
 * @brief Apply servo corrections based on tracking data
 * @param tracking_data Tracking data with yaw/pitch errors
 * @return true if servos were updated
 */
static bool fluctus_apply_servo_corrections(fluctus_solar_snapshot_t *tracking_data)
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
    vTaskDelay(pdMS_TO_TICKS(FLUCTUS_SERVO_POWERUP_DELAY_MS));

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
    vTaskDelay(pdMS_TO_TICKS(FLUCTUS_SERVO_POWERUP_DELAY_MS));

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
    
    // Read 3V3 bus voltage from ADS1115 device #1, Channel 3
    esp_err_t ret = ads1115_helper_read_channel(1, ADS111X_MUX_3_GND, &raw_value, &voltage);
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
static void fluctus_set_ina_metering_mode(bool monitoring_active)
{
    // ====== PV INA219 Power Management (Daytime Only) ======

    // Use centralized daytime/light detection (independent of tracking state)
    bool should_pv_be_active = fluctus_is_daytime_with_sufficient_light();

    // Update PV INA power state
    if (should_pv_be_active != monitoring_data.pv_ina_active) {
        if (fluctus_ina219_set_power_mode(0, should_pv_be_active) == ESP_OK) {
            monitoring_data.pv_ina_active = should_pv_be_active;
            monitoring_data.pv_ina_active = monitoring_data.pv_ina_active;
            ESP_LOGI(TAG, "PV INA219 %s (%s)", should_pv_be_active ? "activated" : "powered down",
                     should_pv_be_active ? "daytime" : "nighttime");
        }
    }

    // ====== Battery INA219 Power Management (Active/Steady State) ======

    // Determine system activity level
    bool any_bus_active = false;
    bool stellaria_only = false;

    for (int i = 0; i < POWER_BUS_COUNT; i++) {
        if (system_status.bus_enabled[i]) {
            any_bus_active = true;
            break;
        }
    }

    // Check for Stellaria-only scenario (12V bus only with 1 consumer)
    if (system_status.bus_enabled[POWER_BUS_12V] &&
        system_status.bus_ref_count[POWER_BUS_12V] == 1 &&
        !system_status.bus_enabled[POWER_BUS_3V3] &&
        !system_status.bus_enabled[POWER_BUS_5V] &&
        !system_status.bus_enabled[POWER_BUS_6V6]) {
        stellaria_only = true;
    }

    // Determine metering state (inlined logic from fluctus_get_recommended_meter_state)
    fluctus_power_meter_state_t recommended_state;
    if (!any_bus_active) {
        // No buses active = steady state
        recommended_state = FLUCTUS_POWER_METER_STATE_STEADY;
    } else if (stellaria_only) {
        // Only Stellaria active = steady state (extrapolate)
        recommended_state = FLUCTUS_POWER_METER_STATE_STEADY;
    } else {
        // Any other bus active = active monitoring
        recommended_state = FLUCTUS_POWER_METER_STATE_ACTIVE;
    }

    // Handle Battery INA power state based on recommended state
    if (recommended_state == FLUCTUS_POWER_METER_STATE_STEADY) {
        // Steady state: Probe for 15 seconds every 15 minutes.
        int64_t current_time_ms = esp_timer_get_time() / 1000;

        if (steady_state_probe_start == 0) {
            // Start new probe cycle
            steady_state_probe_start = current_time_ms;
            if (!monitoring_data.battery_ina_active) {
                fluctus_ina219_set_power_mode(1, true);
                monitoring_data.battery_ina_active = true;
                monitoring_data.battery_ina_active = true;
                ESP_LOGD(TAG, "Battery INA: Starting steady state probe");
            }
        } else if ((current_time_ms - steady_state_probe_start) < FLUCTUS_POWER_STEADY_PROBE_DURATION_MS) {
            // Within probe duration - keep active
            if (!monitoring_data.battery_ina_active) {
                fluctus_ina219_set_power_mode(1, true);
                monitoring_data.battery_ina_active = true;
                monitoring_data.battery_ina_active = true;
            }
        } else if ((current_time_ms - steady_state_probe_start) < FLUCTUS_POWER_STEADY_MONITOR_INTERVAL_MS) {
            // Between probes - power down
            if (monitoring_data.battery_ina_active) {
                fluctus_ina219_set_power_mode(1, false);
                monitoring_data.battery_ina_active = false;
                monitoring_data.battery_ina_active = false;
                ESP_LOGD(TAG, "Battery INA: Ending steady state probe");
            }
        } else {
            // Cycle complete - reset for next probe
            steady_state_probe_start = 0;
        }
    } else {
        // Active state: Keep Battery INA powered
        if (!monitoring_data.battery_ina_active) {
            fluctus_ina219_set_power_mode(1, true);
            monitoring_data.battery_ina_active = true;
            monitoring_data.battery_ina_active = true;
            ESP_LOGD(TAG, "Battery INA: Activated for active monitoring");
        }
        steady_state_probe_start = 0;  // Reset steady state timer
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

        // Determine timeout based on monitoring state and time of day
        if (monitoring_active) {
            // Active monitoring - short timeout for continuous monitoring
            timeout = pdMS_TO_TICKS(FLUCTUS_POWER_ACTIVE_MONITOR_INTERVAL_MS);
        } else {
            // Idle monitoring - use day/night aware interval
            uint32_t idle_interval_ms = solar_calc_is_daytime_buffered() ?
                                       FLUCTUS_POWER_PERIODIC_CHECK_INTERVAL_MS :
                                       FLUCTUS_POWER_PERIODIC_CHECK_INTERVAL_NIGHT_MS;
            timeout = pdMS_TO_TICKS(idle_interval_ms);
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
        fluctus_set_ina_metering_mode(monitoring_active);

        // Read power sensors
        if (xSemaphoreTake(xFluctusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
            fluctus_read_ina219_sensors();
            fluctus_check_3v3_bus_voltage(); // Check 3.3V bus voltage
            fluctus_handle_temperature_monitoring(monitoring_active); // Temperature monitoring with smart scheduling

            // Update energy accumulator with current power readings
            fluctus_update_energy_accumulator(
                monitoring_data.solar_pv.power,
                monitoring_data.battery_out.power
            );

            // Check for hourly rollover (updates daily totals, resets hourly counters)
            fluctus_check_hourly_rollover();

            xSemaphoreGive(xFluctusMutex);
        }
        
        // Check power management conditions
        if (xSemaphoreTake(xFluctusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
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
            
            xSemaphoreGive(xFluctusMutex);
        }

        if (telemetry_is_realtime_enabled()) {
            telemetry_fetch_snapshot(TELEMETRY_SRC_FLUCTUS_RT);
        }
        // Update telemetry cache after every monitoring cycle (follow task frequency)
        telemetry_fetch_snapshot(TELEMETRY_SRC_FLUCTUS);
    }
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
static bool fluctus_is_daytime_with_sufficient_light(void)
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
    if (xSemaphoreTake(xFluctusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
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

        xSemaphoreGive(xFluctusMutex);
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
static void fluctus_set_tracking_state(solar_tracking_state_t new_state, const char *log_msg)
{
    if (xSemaphoreTake(xFluctusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        system_status.solar_tracking_state = new_state;
        if (log_msg) {
            ESP_LOGI(TAG, "%s", log_msg);
        }
        xSemaphoreGive(xFluctusMutex);
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
static void fluctus_solar_state_standby(int64_t current_time_ms)
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

    // Check if we should start correction cycle
    if (last_correction_time == 0 ||
        (current_time_ms - last_correction_time) >= FLUCTUS_TRACKING_CORRECTION_INTERVAL_MS) {

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
static void fluctus_solar_state_correcting(int64_t current_time_ms)
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
    if (xSemaphoreTake(xFluctusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        memcpy(&tracking_data, &cached_solar_data, sizeof(fluctus_solar_snapshot_t));
        xSemaphoreGive(xFluctusMutex);
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
static void fluctus_solar_state_parking(void)
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
static void fluctus_solar_state_error(void)
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

/**
 * @brief Sunrise callback - wakes up SLEEPING solar tracking
 *
 * Called by solar_calc at sunrise time (buffered -30 minutes).
 * Sends notification to solar tracking task to transition from SLEEPING to STANDBY.
 */
static void fluctus_on_sunrise_callback(void)
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
static void fluctus_solar_tracking_task(void *parameters)
{
    ESP_LOGI(TAG, "Solar tracking task started.");
    solar_tracking_state_t current_state;

    while (true) {
        // Read current state and perform safety checks
        if (xSemaphoreTake(xFluctusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
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

            xSemaphoreGive(xFluctusMutex);
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
                timeout = pdMS_TO_TICKS(FLUCTUS_TRACKING_CORRECTION_INTERVAL_MS);  // 15 minutes
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

// ########################## Public API Functions ##########################

esp_err_t fluctus_init(void)
{
    ESP_LOGI(TAG, "Initializing FLUCTUS power management and solar tracking system...");
    
    if (fluctus_initialized) {
        ESP_LOGW(TAG, "FLUCTUS already initialized");
        return ESP_OK;
    }
    
    // Create mutex
    xFluctusMutex = xSemaphoreCreateMutex();
    if (xFluctusMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create FLUCTUS mutex");
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

    // Register sunrise callback for automatic SLEEPING → STANDBY transitions
    ret = solar_calc_register_sunrise_callback(fluctus_on_sunrise_callback);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register sunrise callback - auto-resume at sunrise disabled");
        // Non-critical - continue initialization
    }

    // Register midnight callback for daily energy summary and reset (v3.3)
    ret = solar_calc_register_midnight_callback(fluctus_midnight_callback);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register midnight callback - daily energy reset disabled");
        // Non-critical - continue initialization
    } else {
        ESP_LOGI(TAG, "Midnight callback registered for daily energy tracking");
    }

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
        6,
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
    if (xFluctusMutex) {
        vSemaphoreDelete(xFluctusMutex);
        xFluctusMutex = NULL;
    }
    return ret;
}

esp_err_t fluctus_request_bus_power(power_bus_t bus, const char* consumer_id)
{
    if (bus >= POWER_BUS_COUNT || consumer_id == NULL || !fluctus_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(xFluctusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
        if (system_status.safety_shutdown) {
            xSemaphoreGive(xFluctusMutex);
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
        
        xSemaphoreGive(xFluctusMutex);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

esp_err_t fluctus_release_bus_power(power_bus_t bus, const char* consumer_id)
{
    if (bus >= POWER_BUS_COUNT || consumer_id == NULL || !fluctus_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(xFluctusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
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
        
        xSemaphoreGive(xFluctusMutex);
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
    if (xSemaphoreTake(xFluctusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        powered = system_status.bus_enabled[bus] && !system_status.safety_shutdown;
        xSemaphoreGive(xFluctusMutex);
    }
    
    return powered;
}

fluctus_power_state_t fluctus_get_power_state(void)
{
    if (!fluctus_initialized) {
        return FLUCTUS_POWER_STATE_SHUTDOWN;
    }
    
    fluctus_power_state_t state = FLUCTUS_POWER_STATE_SHUTDOWN;
    if (xSemaphoreTake(xFluctusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        state = system_status.power_state;
        xSemaphoreGive(xFluctusMutex);
    }
    
    return state;
}

esp_err_t fluctus_manual_safety_reset(void)
{
    if (!fluctus_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(xFluctusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
        if (!system_status.manual_reset_required) {
            xSemaphoreGive(xFluctusMutex);
            return ESP_ERR_INVALID_STATE;
        }
        
        system_status.safety_shutdown = false;
        system_status.manual_reset_required = false;
        overcurrent_timer_active = false;
        
        // Reset to normal power state
        system_status.power_state = FLUCTUS_POWER_STATE_NORMAL;
        
        ESP_LOGI(TAG, "Manual safety reset performed - system ready");
        
        xSemaphoreGive(xFluctusMutex);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

esp_err_t fluctus_enable_solar_tracking(void)
{
    if (!fluctus_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xFluctusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        // Check if solar tracking can be enabled (not in critical power or safety shutdown)
        if (system_status.safety_shutdown ||
            system_status.power_state >= FLUCTUS_POWER_STATE_CRITICAL) {
            xSemaphoreGive(xFluctusMutex);
            ESP_LOGW(TAG, "Cannot enable solar tracking - system in critical state or safety shutdown");
            return ESP_ERR_INVALID_STATE;
        }

        xSemaphoreGive(xFluctusMutex);
    } else {
        return ESP_FAIL;
    }

    // Send enable notification to solar tracking task (async)
    // Task will handle state transition and startup logic
    ESP_LOGI(TAG, "Solar tracking enable requested");
    xTaskNotify(xFluctusSolarTrackingTaskHandle, FLUCTUS_NOTIFY_SOLAR_ENABLE, eSetBits);

    return ESP_OK;
}

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

/**
 * @brief Write FLUCTUS data directly to TELEMETRY cache (full snapshot)
 *
 * Comprehensive snapshot for HMI/MQTT distribution with all fields.
 * Writes directly to TELEMETRY's cache buffer to eliminate intermediate copy.
 * Only FLUCTUS mutexes needed - TELEMETRY lock/unlock handled by caller.
 *
 * @param cache Pointer to TELEMETRY's fluctus_snapshot_t buffer
 * @return ESP_OK on success, ESP_FAIL if mutex timeout
 */
esp_err_t fluctus_write_to_telemetry_cache(fluctus_snapshot_t *cache)
{
    if (cache == NULL || !fluctus_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    // Take mutex for comprehensive snapshot (both monitoring_data and system_status)
    if (xSemaphoreTake(xFluctusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Power buses - current state snapshot
    cache->bus_3v3_enabled = system_status.bus_enabled[POWER_BUS_3V3];
    cache->bus_5v_enabled = system_status.bus_enabled[POWER_BUS_5V];
    cache->bus_6v6_enabled = system_status.bus_enabled[POWER_BUS_6V6];
    cache->bus_12v_enabled = system_status.bus_enabled[POWER_BUS_12V];
    cache->bus_3v3_consumers = system_status.bus_ref_count[POWER_BUS_3V3];
    cache->bus_5v_consumers = system_status.bus_ref_count[POWER_BUS_5V];
    cache->bus_6v6_consumers = system_status.bus_ref_count[POWER_BUS_6V6];
    cache->bus_12v_consumers = system_status.bus_ref_count[POWER_BUS_12V];

    // Battery - calculate 15-minute averages from accumulator sums
    if (rtc_accumulator.sample_count_15min > 0) {
        cache->battery_voltage_avg_15min = rtc_accumulator.battery_voltage_sum_15min / rtc_accumulator.sample_count_15min;
        cache->battery_current_avg_15min = rtc_accumulator.battery_current_sum_15min / rtc_accumulator.sample_count_15min;
        cache->battery_power_avg_15min = rtc_accumulator.battery_power_sum_15min / rtc_accumulator.sample_count_15min;
        cache->battery_soc_avg_15min = calculate_battery_soc(cache->battery_voltage_avg_15min);
    } else {
        // No samples yet - use current instantaneous values
        cache->battery_voltage_avg_15min = monitoring_data.battery_voltage;
        cache->battery_current_avg_15min = monitoring_data.battery_out.current;
        cache->battery_power_avg_15min = monitoring_data.battery_out.power;
        cache->battery_soc_avg_15min = calculate_battery_soc(monitoring_data.battery_voltage);
    }

    // Solar - calculate 15-minute averages from accumulator sums
    if (rtc_accumulator.sample_count_15min > 0) {
        cache->solar_voltage_avg_15min = rtc_accumulator.pv_voltage_sum_15min / rtc_accumulator.sample_count_15min;
        cache->solar_current_avg_15min = rtc_accumulator.pv_current_sum_15min / rtc_accumulator.sample_count_15min;
        cache->solar_power_avg_15min = rtc_accumulator.pv_power_sum_15min / rtc_accumulator.sample_count_15min;
    } else {
        // No samples yet - use current instantaneous values
        cache->solar_voltage_avg_15min = monitoring_data.solar_pv.voltage;
        cache->solar_current_avg_15min = monitoring_data.solar_pv.current;
        cache->solar_power_avg_15min = monitoring_data.solar_pv.power;
    }
    cache->solar_pv_active = monitoring_data.pv_ina_active;

    // Solar tracking - current state (minimal, no debug data)
    cache->tracking_state = system_status.solar_tracking_state;
    cache->yaw_position_percent = (uint8_t)fluctus_duty_to_percent(system_status.current_yaw_duty);
    cache->pitch_position_percent = (uint8_t)fluctus_duty_to_percent(system_status.current_pitch_duty);

    // Thermal management - current state
    cache->case_temperature = monitoring_data.case_temperature;
    cache->temperature_valid = monitoring_data.temperature_valid;
    if (monitoring_data.fan_active) {
        if (monitoring_data.case_temperature >= FLUCTUS_TEMP_MAX_FAN_THRESHOLD) {
            cache->fan_speed_percent = 100;
        } else if (monitoring_data.case_temperature <= FLUCTUS_TEMP_TURN_ON_THRESHOLD) {
            cache->fan_speed_percent = FLUCTUS_TEMP_MIN_FAN_DUTY;
        } else {
            float temp_range = FLUCTUS_TEMP_MAX_FAN_THRESHOLD - FLUCTUS_TEMP_TURN_ON_THRESHOLD;
            float temp_offset = monitoring_data.case_temperature - FLUCTUS_TEMP_TURN_ON_THRESHOLD;
            float duty_range = FLUCTUS_TEMP_MAX_FAN_DUTY - FLUCTUS_TEMP_MIN_FAN_DUTY;
            cache->fan_speed_percent = FLUCTUS_TEMP_MIN_FAN_DUTY +
                (uint8_t)((temp_offset / temp_range) * duty_range);
        }
    } else {
        cache->fan_speed_percent = 0;
    }

    // Energy statistics - hourly totals and peaks
    cache->current_hour_start = rtc_accumulator.current_hour_start;
    cache->pv_energy_wh_hour = rtc_accumulator.pv_energy_wh_accumulator;
    cache->battery_energy_wh_hour = rtc_accumulator.battery_energy_wh_accumulator;
    cache->pv_peak_w_hour = rtc_accumulator.pv_peak_w_hour;
    cache->battery_peak_w_hour = rtc_accumulator.battery_peak_w_hour;

    // Energy statistics - daily totals and peaks
    cache->current_day_start = rtc_accumulator.current_day_start;
    cache->pv_energy_wh_day = rtc_accumulator.pv_energy_wh_day;
    cache->battery_consumed_wh_day = rtc_accumulator.battery_consumed_wh_day;
    cache->pv_peak_w_day = rtc_accumulator.pv_peak_w_day;
    cache->battery_peak_w_day = rtc_accumulator.battery_peak_w_day;
    cache->hours_active_day = rtc_accumulator.hours_active_day;

    // System state
    cache->power_state = system_status.power_state;
    cache->safety_shutdown = system_status.safety_shutdown;
    cache->manual_reset_required = system_status.manual_reset_required;
    cache->last_activity_time = system_status.last_activity_time;

    // Validity flags
    cache->battery_data_valid = monitoring_data.battery_out.valid;
    cache->solar_data_valid = monitoring_data.solar_pv.valid;

    // Note: snapshot_timestamp set by TELEMETRY unlock function

    xSemaphoreGive(xFluctusMutex);

    return ESP_OK;
}

/**
 * @brief Write FLUCTUS realtime data to TELEMETRY cache (rich snapshot)
 *
 * High-frequency power monitoring snapshot for realtime MQTT streaming (QoS 0).
 * Contains instantaneous sensor readings and full debug data (photoresistors, duty cycles).
 * Optimized for 500ms update rate during active monitoring.
 * "Rich" version includes debug fields for removal after debug period.
 *
 * @param cache Pointer to TELEMETRY's fluctus_snapshot_rt_t buffer
 * @return ESP_OK on success, ESP_FAIL if mutex timeout
 */
esp_err_t fluctus_write_realtime_to_telemetry_cache(fluctus_snapshot_t *cache)
{
    if (cache == NULL || !fluctus_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    // Take mutex for realtime snapshot (both monitoring_data and system_status)
    if (xSemaphoreTake(xFluctusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Power buses - dynamic state (catches transient activity at 500ms)
    cache->bus_3v3_enabled = system_status.bus_enabled[POWER_BUS_3V3];
    cache->bus_5v_enabled = system_status.bus_enabled[POWER_BUS_5V];
    cache->bus_6v6_enabled = system_status.bus_enabled[POWER_BUS_6V6];
    cache->bus_12v_enabled = system_status.bus_enabled[POWER_BUS_12V];
    cache->bus_3v3_consumers = system_status.bus_ref_count[POWER_BUS_3V3];
    cache->bus_5v_consumers = system_status.bus_ref_count[POWER_BUS_5V];
    cache->bus_6v6_consumers = system_status.bus_ref_count[POWER_BUS_6V6];
    cache->bus_12v_consumers = system_status.bus_ref_count[POWER_BUS_12V];

    // Battery - instantaneous readings
    cache->battery_voltage_inst = monitoring_data.battery_voltage;
    cache->battery_current_inst = monitoring_data.battery_out.current;
    cache->battery_power_inst = monitoring_data.battery_out.power;
    cache->battery_soc_inst = calculate_battery_soc(monitoring_data.battery_voltage);  // DEBUG: for removal

    // Solar - instantaneous readings
    cache->solar_voltage_inst = monitoring_data.solar_pv.voltage;
    cache->solar_current_inst = monitoring_data.solar_pv.current;
    cache->solar_power_inst = monitoring_data.solar_pv.power;
    cache->solar_pv_active = monitoring_data.pv_ina_active;

    // Solar tracking - dynamic state with full debug data
    cache->tracking_state = system_status.solar_tracking_state;
    cache->yaw_position_percent = (uint8_t)fluctus_duty_to_percent(system_status.current_yaw_duty);
    cache->pitch_position_percent = (uint8_t)fluctus_duty_to_percent(system_status.current_pitch_duty);
    cache->yaw_error = cached_solar_data.yaw_error;
    cache->pitch_error = cached_solar_data.pitch_error;
    cache->current_yaw_duty = system_status.current_yaw_duty;       // DEBUG: raw PWM duty
    cache->current_pitch_duty = system_status.current_pitch_duty;   // DEBUG: raw PWM duty
    memcpy(cache->photoresistor_readings, cached_solar_data.photoresistor_readings, sizeof(cache->photoresistor_readings));  // DEBUG

    // Thermal management - instantaneous
    cache->case_temperature = monitoring_data.case_temperature;
    cache->temperature_valid = monitoring_data.temperature_valid;
    if (monitoring_data.fan_active) {
        if (monitoring_data.case_temperature >= FLUCTUS_TEMP_MAX_FAN_THRESHOLD) {
            cache->fan_speed_percent = 100;
        } else if (monitoring_data.case_temperature <= FLUCTUS_TEMP_TURN_ON_THRESHOLD) {
            cache->fan_speed_percent = FLUCTUS_TEMP_MIN_FAN_DUTY;
        } else {
            float temp_range = FLUCTUS_TEMP_MAX_FAN_THRESHOLD - FLUCTUS_TEMP_TURN_ON_THRESHOLD;
            float temp_offset = monitoring_data.case_temperature - FLUCTUS_TEMP_TURN_ON_THRESHOLD;
            float duty_range = FLUCTUS_TEMP_MAX_FAN_DUTY - FLUCTUS_TEMP_MIN_FAN_DUTY;
            cache->fan_speed_percent = FLUCTUS_TEMP_MIN_FAN_DUTY +
                (uint8_t)((temp_offset / temp_range) * duty_range);
        }
    } else {
        cache->fan_speed_percent = 0;
    }

    // Validity flags
    cache->battery_data_valid = monitoring_data.battery_out.valid;
    cache->solar_data_valid = monitoring_data.solar_pv.valid;

    // Note: snapshot_timestamp set by TELEMETRY unlock function

    xSemaphoreGive(xFluctusMutex);

    return ESP_OK;
}

// ########################## Energy Tracking Functions (v3.3) ##########################

/**
 * @brief Update power energy accumulator (called every 500ms from monitoring task)
 *
 * @param pv_power_w Current PV power (W)
 * @param battery_power_w Current battery power (W, positive = consumption)
 */
static void fluctus_update_energy_accumulator(float pv_power_w, float battery_power_w)
{
    time_t now = time(NULL);

    // Initialize accumulator on first call or after power loss
    if (!rtc_accumulator.initialized || rtc_accumulator.current_hour_start == 0) {
        ESP_LOGI(TAG, "Initializing RTC power accumulator (first boot or power loss)");
        rtc_accumulator.current_hour_start = now;
        rtc_accumulator.current_day_start = now;
        rtc_accumulator.interval_start_15min = now;
        rtc_accumulator.last_sample_time = now;
        rtc_accumulator.initialized = true;
        return;
    }

    // Calculate time delta for energy accumulation
    time_t delta_sec = now - rtc_accumulator.last_sample_time;

    // Sanity check: skip if time delta is invalid (>10s indicates clock issue or missed samples)
    if (delta_sec <= 0 || delta_sec > 10) {
        ESP_LOGW(TAG, "Invalid time delta: %ld seconds, skipping energy accumulation", (long)delta_sec);
        rtc_accumulator.last_sample_time = now;
        return;
    }

    // Accumulate energy: Wh = W * hours
    float delta_hour = delta_sec / 3600.0f;
    rtc_accumulator.pv_energy_wh_accumulator += pv_power_w * delta_hour;
    rtc_accumulator.battery_energy_wh_accumulator += battery_power_w * delta_hour;

    // Update hourly peaks
    if (pv_power_w > rtc_accumulator.pv_peak_w_hour) {
        rtc_accumulator.pv_peak_w_hour = pv_power_w;
    }
    if (battery_power_w > rtc_accumulator.battery_peak_w_hour) {
        rtc_accumulator.battery_peak_w_hour = battery_power_w;
    }

    // Update daily peaks
    if (pv_power_w > rtc_accumulator.pv_peak_w_day) {
        rtc_accumulator.pv_peak_w_day = pv_power_w;
    }
    if (battery_power_w > rtc_accumulator.battery_peak_w_day) {
        rtc_accumulator.battery_peak_w_day = battery_power_w;
    }

    // Update 15-minute averaging sums
    rtc_accumulator.pv_power_sum_15min += pv_power_w;
    rtc_accumulator.pv_voltage_sum_15min += monitoring_data.solar_pv.voltage;
    rtc_accumulator.pv_current_sum_15min += monitoring_data.solar_pv.current;
    rtc_accumulator.battery_power_sum_15min += battery_power_w;
    rtc_accumulator.battery_voltage_sum_15min += monitoring_data.battery_voltage;
    rtc_accumulator.battery_current_sum_15min += monitoring_data.battery_out.current;
    rtc_accumulator.sample_count_15min++;

    rtc_accumulator.last_sample_time = now;
}

/**
 * @brief Check for hourly rollover and update daily totals
 * Called from fluctus_monitoring_task every loop (500ms)
 *
 * @return true if hour changed (hourly rollover occurred), false otherwise
 */
static bool fluctus_check_hourly_rollover(void)
{
    if (!rtc_accumulator.initialized) {
        return false;
    }

    time_t now = time(NULL);
    struct tm tm_now, tm_hour_start;
    gmtime_r(&now, &tm_now);
    gmtime_r(&rtc_accumulator.current_hour_start, &tm_hour_start);

    // Check if hour changed
    if (tm_now.tm_hour != tm_hour_start.tm_hour ||
        tm_now.tm_yday != tm_hour_start.tm_yday ||
        tm_now.tm_year != tm_hour_start.tm_year) {

        // Update daily totals with completed hour data
        rtc_accumulator.pv_energy_wh_day += rtc_accumulator.pv_energy_wh_accumulator;
        rtc_accumulator.battery_consumed_wh_day += rtc_accumulator.battery_energy_wh_accumulator;

        // Count active hours (hour with significant PV generation)
        if (rtc_accumulator.pv_energy_wh_accumulator > 0.1f) {
            rtc_accumulator.hours_active_day++;
        }

        ESP_LOGI(TAG, "Hourly rollover: PV %.1f Wh, Battery %.1f Wh consumed",
                 rtc_accumulator.pv_energy_wh_accumulator,
                 rtc_accumulator.battery_energy_wh_accumulator);

        // Reset hourly counters
        rtc_accumulator.current_hour_start = now;
        rtc_accumulator.pv_energy_wh_accumulator = 0.0f;
        rtc_accumulator.battery_energy_wh_accumulator = 0.0f;
        rtc_accumulator.pv_peak_w_hour = 0.0f;
        rtc_accumulator.battery_peak_w_hour = 0.0f;

        return true;  // Signal hourly event
    }

    return false;
}

/**
 * @brief Midnight callback - daily energy summary and reset
 * Registered with solar_calc during fluctus_init()
 * Called automatically at midnight (00:00:00 local time)
 */
static void fluctus_midnight_callback(void)
{
    ESP_LOGI(TAG, "=== Midnight Callback: Daily Energy Summary ===");
    ESP_LOGI(TAG, "  PV total:         %.1f Wh", rtc_accumulator.pv_energy_wh_day);
    ESP_LOGI(TAG, "  Battery consumed: %.1f Wh", rtc_accumulator.battery_consumed_wh_day);
    ESP_LOGI(TAG, "  Net energy:       %.1f Wh",
             rtc_accumulator.pv_energy_wh_day - rtc_accumulator.battery_consumed_wh_day);
    ESP_LOGI(TAG, "  Hours active:     %d", rtc_accumulator.hours_active_day);
    ESP_LOGI(TAG, "  PV peak (day):    %.1f W", rtc_accumulator.pv_peak_w_day);
    ESP_LOGI(TAG, "  Battery peak:     %.1f W", rtc_accumulator.battery_peak_w_day);
    ESP_LOGI(TAG, "===============================================");

    // TODO: Trigger MQTT publish with current snapshot (includes daily data)
    // This will be implemented in Phase 2 with unified cache API
    // telemetry_fetch_snapshot(TELEMETRY_SRC_FLUCTUS);

    // Reset daily counters AFTER logging (MQTT publish will use these values)
    rtc_accumulator.current_day_start = time(NULL);
    rtc_accumulator.pv_energy_wh_day = 0.0f;
    rtc_accumulator.battery_consumed_wh_day = 0.0f;
    rtc_accumulator.pv_peak_w_day = 0.0f;
    rtc_accumulator.battery_peak_w_day = 0.0f;
    rtc_accumulator.hours_active_day = 0;

    ESP_LOGI(TAG, "Daily energy counters reset");
}

// ########################## End Energy Tracking Functions ##########################

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

void fluctus_hall_array_enable(bool enable)
{
    gpio_set_level(FLUCTUS_3V3_HALL_ARRAY_ENABLE_GPIO, enable ? 1 : 0);
    ESP_LOGD(TAG, "Hall array power: %s", enable ? "ON" : "OFF");
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