#include <stdio.h>
#include <string.h>
#include <math.h>
// #include <inttypes.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
// #include "freertos/event_groups.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "bmp280.h"
#include "sht4x.h"
#include "irrigation.h"


#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

#include "esp_log.h"
#include "esp_err.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_sntp.h"
#include <time.h>


#include "main.h"
#include "wifi_credentials.h"


// ########################## Global Variable Definitions ##########################
#define TAG "SOLARIUM_1"

// Mutex for protecting shared display data
SemaphoreHandle_t xDisplayDataMutex = NULL;

// ADS1115 Devices
ads1115_device_t ads1115_devices[ADS1115_DEVICE_COUNT] = {
    {.name = "Photoresistors", .initialized = false, .retry_count = 0, .next_retry_delay_ms = 60000},   // Dev#0
    {.name = "Moisture_Sensors", .initialized = false, .retry_count = 0, .next_retry_delay_ms = 60000}, // Dev#1
    {.name = "Mixed_Sensors", .initialized = false, .retry_count = 0, .next_retry_delay_ms = 60000}};   // Dev#2

float latest_ads_voltages[ADS1115_DEVICE_COUNT][4] = {
    {-999.9, -999.9, -999.9, -999.9}, // Dev#0: Photoresistors
    {-999.9, -999.9, -999.9, -999.9}, // Dev#1: Moisture sensors
    {-999.9, -999.9, -999.9, -999.9}  // Dev#2: Zone5 moisture + pressure + spare
};

// INA219 Devices
ina219_sensor_t ina219_devices[INA219_DEVICE_COUNT] = {
    {.type = INA219_TYPE_GENERIC,
     .addr = INA219_ADDR_GND_GND, // 0x40
     .shunt_ohms = 0.1f,          // R100
     .gain = INA219_GAIN_0_5,     // ±2A range
     .name = "GEN_3.2A",
     .initialized = false},
    // TESTING WITHOUT 2ND
    /*{
        .type = INA219_TYPE_DFROBOT,
        .addr = INA219_ADDR_GND_VS,   // 0x41
        .shunt_ohms = 0.01f,          // R010
        .gain = INA219_GAIN_0_125,    // ±8A range
        .name = "DFR_8A",
        .initialized = false
    }*/
};

float latest_ina219_voltage[INA219_DEVICE_COUNT] = {-999.9};
float latest_ina219_current[INA219_DEVICE_COUNT] = {-999.9};
float latest_ina219_power[INA219_DEVICE_COUNT] = {-999.9};

// AS5600 Device
as5600_dev_t as5600_dev;

// Debuging display variables
float latest_sht_temp = -999.9;
float latest_sht_hum = -999.9;
float latest_bmp_temp = -999.9;
float latest_bmp_press = -999.9;
float latest_bmp_hum = -999.9;
float latest_as5600_angle = -999.9;
uint16_t latest_as5600_raw = 0;

// Flag to indicate if time has been synchronized
static bool g_time_synced = false;


// ########################## FUNCTION DECLARATIONS ################################
// ADS1115 Functions
static esp_err_t ads1115_init_all(void);
static esp_err_t ads1115_init_device(uint8_t device_id);
// Solar Tracking Functions
static esp_err_t solar_t_read_photoresistors(photoresistor_readings_t *readings);
static esp_err_t solar_t_servo_pwm_init(void);
static esp_err_t solar_t_servo_set_position(ledc_channel_t channel, uint32_t duty_cycle);
static uint32_t solar_t_calc_servo_adjust(float error, uint32_t current_duty);
static esp_err_t solar_tracker_loop(void);
// Power Management Functions
static esp_err_t power_management_init(void);
static esp_err_t power_on_servo_bus(void);
static esp_err_t power_off_servo_bus(void);

// ########################## FUNCTIONS ################################
// ------- WiFI and Simple Network Time Protocol Synchronization -------

// Wi-Fi credentials are in wifi_credentials.h

static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Time synchronized");
    g_time_synced = true;
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        initialize_sntp();
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta =
            {
                .ssid = WIFI_SSID,
                .password = WIFI_PASSWORD,
            },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

// -----------------------------------------------------------------------
// ############################## FUNCTIONS ##############################
// -----------------------------------------------------------------------

// ########################## ADS1115 Functions ##########################

/**
 * @brief Initialize all ADS1115 devices
 *
 * Attempts to initialize all ADS1115 devices defined in the system.
 * Continues operation even if some devices fail to initialize.
 *
 * @return ESP_OK if at least one device initialized successfully
 * @return ESP_FAIL if no devices could be initialized
 */
static esp_err_t ads1115_init_all(void)
{
    ESP_LOGI(TAG, "Initializing %d ADS1115 devices...", ADS1115_DEVICE_COUNT);

    // Initialize status tracking
    for (uint8_t device = 0; device < ADS1115_DEVICE_COUNT; device++) {
        ads1115_devices[device].initialized = false;
        ads1115_devices[device].last_retry_time = 0;
        ads1115_devices[device].retry_count = 0;
        ads1115_devices[device].next_retry_delay_ms = 60000; // Start with 1 minute
    }

    uint8_t successful_devices = 0;

    // Try to initialize each device
    for (uint8_t device = 0; device < ADS1115_DEVICE_COUNT; device++) {
        ESP_LOGI(TAG,
                 "Initializing ADS1115 device %d (%s) at 0x%02x",
                 device,
                 ads1115_devices[device].name,
                 ads1115_addresses[device]);

        esp_err_t result = ads1115_init_device(device);
        if (result == ESP_OK) {
            ads1115_devices[device].initialized = true;
            successful_devices++;
            ESP_LOGI(TAG, "ADS1115 # %d (%s) initialized successfully", device, ads1115_devices[device].name);
        } else {
            ESP_LOGW(TAG,
                     "ADS1115 # %d (%s) failed to initialize: %s",
                     device,
                     ads1115_devices[device].name,
                     esp_err_to_name(result));
            ESP_LOGW(TAG, "Will retry in %" PRIu32 " seconds", ads1115_devices[device].next_retry_delay_ms / 1000);
        }
    }

    ESP_LOGI(TAG, "ADS1115 initialization complete: %d/%d devices working", successful_devices, ADS1115_DEVICE_COUNT);

    if (successful_devices == 0) {
        ESP_LOGE(TAG, "No ADS1115 devices initialized! System functionality will be limited.");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Initialize a single ADS1115 device
 *
 * Initializes a specific ADS1115 device with default settings:
 * - Single-shot mode
 * - 32 SPS data rate
 * - Configured gain setting
 * - Performs test read to verify communication
 *
 * @param device_id Device index (0 to ADS1115_DEVICE_COUNT-1)
 * @return ESP_OK on successful initialization
 * @return ESP_ERR_INVALID_ARG if device_id is invalid
 * @return ESP_ERR_* on communication or configuration errors
 */
static esp_err_t ads1115_init_device(uint8_t device_id)
{
    if (device_id >= ADS1115_DEVICE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_dev_t *dev = &ads1115_devices[device_id].device;
    esp_err_t ret;

    // 1. Initialize descriptor
    ret = ads111x_init_desc(dev,
                            ads1115_addresses[device_id],
                            I2C_PORT_NUM,
                            CONFIG_I2CDEV_DEFAULT_SDA_PIN,
                            CONFIG_I2CDEV_DEFAULT_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed ads111x_init_desc: %s", esp_err_to_name(ret));
        return ret;
    }

    // 2. Set single-shot mode
    ret = ads111x_set_mode(dev, ADS111X_MODE_SINGLE_SHOT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed ads111x_set_mode: %s", esp_err_to_name(ret));
        ads111x_free_desc(dev); // Cleanup on failure
        return ret;
    }

    // 3. Set data rate
    ret = ads111x_set_data_rate(dev, ADS111X_DATA_RATE_32);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed ads111x_set_data_rate: %s", esp_err_to_name(ret));
        ads111x_free_desc(dev); // Cleanup on failure
        return ret;
    }

    // 4. Set gain
    ret = ads111x_set_gain(dev, GAIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed ads111x_set_gain: %s", esp_err_to_name(ret));
        ads111x_free_desc(dev); // Cleanup on failure
        return ret;
    }

    // 5. Test read to verify device is responsive
    int16_t test_raw;
    ret = ads111x_set_input_mux(dev, ADS111X_MUX_0_GND);
    if (ret == ESP_OK) {
        ret = ads111x_start_conversion(dev);
        if (ret == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(50)); // Wait for conversion
            ret = ads111x_get_value(dev, &test_raw);
        }
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed test read: %s", esp_err_to_name(ret));
        ads111x_free_desc(dev); // Cleanup on failure
        return ret;
    }

    ESP_LOGD(TAG, "Test read successful (raw: %d)", test_raw);
    return ESP_OK;
}

/**
 * @brief Read a single channel from an ADS1115 device
 *
 * Performs a single-shot conversion on the specified channel.
 * Automatically marks device as failed if communication errors occur.
 *
 * @param device_id Device index (0 to ADS1115_DEVICE_COUNT-1)
 * @param channel Input channel/MUX setting (ADS111X_MUX_*)
 * @param raw Pointer to store raw ADC value (required)
 * @param voltage Pointer to store calculated voltage (optional, can be NULL)
 * @return ESP_OK on successful read
 * @return ESP_ERR_INVALID_ARG if device_id is invalid or raw is NULL
 * @return ESP_ERR_INVALID_STATE if device is not initialized
 * @return ESP_ERR_TIMEOUT if conversion times out
 * @return ESP_ERR_* on communication errors
 */
esp_err_t ads1115_read_channel(uint8_t device_id, ads111x_mux_t channel, int16_t *raw, float *voltage)
{
    if (!raw) {
        return ESP_ERR_INVALID_ARG;
    }

    if (device_id >= ADS1115_DEVICE_COUNT) {
        ESP_LOGE(TAG, "[Dev %d] Invalid ADS1115 device index!", device_id);
        return ESP_ERR_INVALID_ARG;
    }

    // Check if device is initialized
    if (!ads1115_devices[device_id].initialized) {
        ESP_LOGD(TAG, "[Dev %d] Device not initialized, skipping read", device_id);
        return ESP_ERR_INVALID_STATE;
    }

    i2c_dev_t *dev = &ads1115_devices[device_id].device;
    esp_err_t ret;

    // 1. Set MUX (channel selection)
    ret = ads111x_set_input_mux(dev, channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed to set MUX: %s (%d)", device_id, esp_err_to_name(ret), ret);
        // Mark device as failed for retry
        ads1115_devices[device_id].initialized = false;
        return ret;
    }

    // 2. Start Conversion
    ret = ads111x_start_conversion(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed to start conversion: %s (%d)", device_id, esp_err_to_name(ret), ret);
        ads1115_devices[device_id].initialized = false;
        return ret;
    }

    // ADD A SMALL DELAY HERE for stability
    vTaskDelay(pdMS_TO_TICKS(1));

    // 3. Wait for conversion to complete (with timeout)
    // Max conversion time for 32 SPS is ~32ms. Add some margin. Timeout after ~100ms.
    int conversion_timeout_ms = 100; // Increased timeout
    int delay_ms = 5;                // Check every 5ms
    int elapsed_ms = 0;
    bool busy = true;

    do {
        ret = ads111x_is_busy(dev, &busy);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[Dev %d] Failed to check busy status: %s (%d)", device_id, esp_err_to_name(ret), ret);
            ads1115_devices[device_id].initialized = false;
            return ret;
        }

        vTaskDelay(1);

        if (busy) {
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
            elapsed_ms += delay_ms;
            if (elapsed_ms > conversion_timeout_ms) {
                ESP_LOGE(TAG, "[Dev %d] Conversion timeout after %d ms", device_id, conversion_timeout_ms);
                ads1115_devices[device_id].initialized = false;
                return ESP_ERR_TIMEOUT;
            }
        }
    } while (busy);

    // 4. Read Value
    ret = ads111x_get_value(dev, raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed to get value: %s (%d)", device_id, esp_err_to_name(ret), ret);
        ads1115_devices[device_id].initialized = false;
        return ret;
    }
    // ESP_LOGI(TAG, "[Dev %d] Read value OK (Raw: %d).", device_index, *raw_value);

    // 5. Calculate Voltage (if requested)
    if (voltage) {
        *voltage = (*raw / (float) ADS111X_MAX_VALUE) * GAIN;
        // ESP_LOGI(TAG, "[Dev %d] Calculated voltage: %.4fV", device_index, *voltage);
    }

    return ESP_OK;
}

// ########################## Solar Tracking Functions ##########################
// -------------------------- Servo Handling Functions --------------------------

/**
 * @brief Initialize PWM channels for servo control
 *
 * Sets up LEDC PWM channels for yaw and pitch servo control.
 * Configures 50Hz frequency with 16-bit resolution.
 *
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
static esp_err_t solar_t_servo_pwm_init(void)
{
    // Config timer
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = SERVO_PWM_TIMER,
        .duty_resolution = SERVO_PWM_RESOLUTION,
        .freq_hz = SERVO_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Config yaw servo channel
    ledc_channel_config_t yaw_channel_config = {
        .gpio_num = SERVO_YAW_GPIO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = SERVO_YAW_CHANNEL,
        .timer_sel = SERVO_PWM_TIMER,
        .duty = SERVO_CENTER_DUTY,
        .hpoint = 0,
    };

    ret = ledc_channel_config(&yaw_channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure yaw servo channel: %s", esp_err_to_name(ret));
    }

    // Config pitch servo channel
    ledc_channel_config_t pitch_channel_config = {
        .gpio_num = SERVO_PITCH_GPIO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = SERVO_PITCH_CHANNEL,
        .timer_sel = SERVO_PWM_TIMER,
        .duty = SERVO_CENTER_DUTY,
        .hpoint = 0,
    };

    ret = ledc_channel_config(&pitch_channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure yaw servo channel: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Servo PWM initialized - Yaw: GPIO%d, Pitch: GPIO%d", SERVO_YAW_GPIO_PIN, SERVO_PITCH_GPIO_PIN);
    return ESP_OK;
}

/**
 * @brief Set servo position via PWM duty cycle
 *
 * @param channel LEDC channel (SERVO_YAW_CHANNEL or SERVO_PITCH_CHANNEL)
 * @param duty_cycle PWM duty cycle (SERVO_MIN_DUTY to SERVO_MAX_DUTY)
 * @return ESP_OK on success
 */
static esp_err_t solar_t_servo_set_position(ledc_channel_t channel, uint32_t duty_cycle)
{
    // Clamp duty cycle to valid range
    if (duty_cycle < SERVO_MIN_DUTY)
        duty_cycle = SERVO_MIN_DUTY;
    if (duty_cycle > SERVO_MAX_DUTY)
        duty_cycle = SERVO_MAX_DUTY;

    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty_cycle);
    if (ret == ESP_OK) {
        ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    }
    return ret;
}

/**
 * @brief Read all photoresistor channels and calculate tracking errors
 *
 * @param readings Pointer to structure to store readings and errors
 * @return ESP_OK if all readings successful, ESP_ERR_* otherwise
 */
static esp_err_t solar_t_read_photoresistors(photoresistor_readings_t *readings)
{
    if (!readings) {
        return ESP_ERR_INVALID_ARG;
    }
    // Check if photoresistor ADS1115 #0 is available
    if (!ads1115_devices[0].initialized) {
        ESP_LOGW(TAG, "Photoresistor ADS1115 #0 is not available, aborting photoresistor read");
        return ESP_ERR_INVALID_STATE;
    }

    int16_t raw[4];
    float voltages[4];
    esp_err_t ret;

    // Read all 4 channels
    const ads111x_mux_t channels[4] = {
        ADS111X_MUX_0_GND, // Ch0: Left-Top
        ADS111X_MUX_1_GND, // Ch1: Right-Top
        ADS111X_MUX_2_GND, // Ch2: Left-Bottom
        ADS111X_MUX_3_GND  // Ch3: Right-Bottom
    };

    for (int i = 0; i < 4; i++) {
        ret = ads1115_read_channel(0, channels[i], &raw[i], &voltages[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read photoresistor channel %d: %s", i, esp_err_to_name(ret));
            return ret;
        }

        // Small delay between readings for stability
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Store individual readings
    readings->left_top = voltages[0];
    readings->right_top = voltages[1];
    readings->left_bottom = voltages[2];
    readings->right_bottom = voltages[3];

    // Calculate average readings
    float left_avg = (readings->left_top + readings->left_bottom) / 2.0f;
    float right_avg = (readings->right_top + readings->right_bottom) / 2.0f;
    float top_avg = (readings->left_top + readings->right_top) / 2.0f;
    float bottom_avg = (readings->left_bottom + readings->right_bottom) / 2.0f;

    // Calculate tracking errors
    readings->yaw_error = left_avg - right_avg;
    readings->pitch_error = top_avg - bottom_avg;

    ESP_LOGI(TAG,
             "Photoresistors: LT=%.3f RT=%.3f LB=%.3f RB=%.3f",
             readings->left_top,
             readings->right_top,
             readings->left_bottom,
             readings->right_bottom);
    ESP_LOGI(TAG, "Tracking errors: Yaw=%.3f Pitch=%.3f", readings->yaw_error, readings->pitch_error);

    return ESP_OK;
}

/**
 * @brief Convert tracking error to servo adjustment
 *
 * @param error Tracking error in volts
 * @param current_position Current servo PWM duty cycle
 * @return New servo PWM duty cycle
 */
static uint32_t solar_t_calc_servo_adjust(float error, uint32_t current_position)
{
    // Skip adjustment if error is below threshold (deadband)
    if (fabs(error) < PHOTORESISTOR_THRESHOLD) {
        return current_position;
    }

    // Calculate proportional adjustment
    // Larger errors = larger adjustments, capped at MAX_SERVO_ADJUSTMENT
    uint32_t adjustment = (uint32_t) (error * 500.0f); // Scale factor: 500 PWM units per volt (14-bit resolution)

    // Limit adjustment magnitude
    if (adjustment > MAX_SERVO_ADJUSTMENT)
        adjustment = MAX_SERVO_ADJUSTMENT;
    if (adjustment < -MAX_SERVO_ADJUSTMENT)
        adjustment = -MAX_SERVO_ADJUSTMENT;

    // Apply adjustment to current position
    uint32_t new_position = (int32_t) current_position + adjustment;

    // Clamp to valid servo range
    if (new_position < SERVO_MIN_DUTY)
        new_position = SERVO_MIN_DUTY;
    if (new_position > SERVO_MAX_DUTY)
        new_position = SERVO_MAX_DUTY;

    return (uint32_t) new_position;
}

/**
 * @brief Perform closed-loop solar tracking with verification
 *
 * @return ESP_OK if tracking successful, ESP_ERR_* otherwise
 */
static esp_err_t solar_tracker_loop(void)
{
    const uint8_t MAX_ADJUSTMENT_ATTEMPTS = 3;
    const uint32_t SERVO_SETTLE_TIME_MS = 1500; // Wait for servos to settle
    const float FINE_THRESHOLD = 0.020f;        // 20mV should be good enough

    photoresistor_readings_t readings;

    // Initial reading
    esp_err_t ret = solar_t_read_photoresistors(&readings);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read photoresistors BEFORE adjustment: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG,
             "Starting tracking cycle - initial errors: Yaw=%.3fV, Pitch=%.3fV",
             readings.yaw_error,
             readings.pitch_error);

    // Check if we're already well-aligned
    if (fabs(readings.yaw_error) < FINE_THRESHOLD && fabs(readings.pitch_error) < FINE_THRESHOLD) {
        ESP_LOGI(TAG, "Already well-aligned, skipping tracking cycle");
        return ESP_OK;
    }

    // Adjustment loop with verification
    for (uint8_t attempt = 0; attempt < MAX_ADJUSTMENT_ATTEMPTS; attempt++) {
        bool adjustment_made = false;

        // Calculate new servo positions
        uint32_t new_yaw_duty = solar_t_calc_servo_adjust(readings.yaw_error, current_yaw_duty);
        uint32_t new_pitch_duty = solar_t_calc_servo_adjust(readings.pitch_error, current_pitch_duty);

        // Power on servos only if adjustment is needed
        if (new_yaw_duty != current_yaw_duty || new_pitch_duty != current_pitch_duty) {
            power_on_servo_bus();
        }

        // Apply yaw adjustment if needed
        if (new_yaw_duty != current_yaw_duty) {
            esp_err_t yaw_result = solar_t_servo_set_position(SERVO_YAW_CHANNEL, new_yaw_duty);
            if (yaw_result == ESP_OK) {
                ESP_LOGI(TAG,
                         "Yaw adjusted: %" PRIu32 " -> %" PRIu32 " (error: %.3fV)",
                         current_yaw_duty,
                         new_yaw_duty,
                         readings.yaw_error);
                current_yaw_duty = new_yaw_duty;
                adjustment_made = true;
            } else {
                ESP_LOGE(TAG, "Failed to set yaw servo: %s", esp_err_to_name(yaw_result));
            }
        }

        if (new_pitch_duty != current_pitch_duty) {
            esp_err_t pitch_result = solar_t_servo_set_position(SERVO_PITCH_CHANNEL, new_pitch_duty);
            if (pitch_result == ESP_OK) {
                ESP_LOGI(TAG,
                         "Pitch adjusted: %" PRIu32 " -> %" PRIu32 " (error: %.3fV)",
                         current_pitch_duty,
                         new_pitch_duty,
                         readings.pitch_error);
                current_pitch_duty = new_pitch_duty;
                adjustment_made = true;
            } else {
                ESP_LOGE(TAG, "Failed to set pitch servo: %s", esp_err_to_name(pitch_result));
            }
        }

        // Power off servos after adjustment
        if (adjustment_made) {
            // Wait for servos to settle, then power off
            vTaskDelay(pdMS_TO_TICKS(SERVO_SETTLE_TIME_MS));
            power_off_servo_bus();
        }

        // If no adjustmet was made (errors below deadband/threshold), we're done
        if (!adjustment_made) {
            ESP_LOGI(TAG, "No adjustments needed, tracking cycle completed");
            break;
        }

        // Wait for servos to settle
        ESP_LOGD(TAG, "Waiting %" PRIu32 " ms for servos to settle", SERVO_SETTLE_TIME_MS);
        vTaskDelay(pdMS_TO_TICKS(SERVO_SETTLE_TIME_MS));

        // Verify the adjustment by re-reading
        ret = solar_t_read_photoresistors(&readings);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read photoresistors after adjustment: %s", esp_err_to_name(ret));
            break;
        }

        ESP_LOGI(TAG, "Post adjustment errors: Yaw=%.3fV, Pitch=%.3fV", readings.yaw_error, readings.pitch_error);

        // Check if we're within the fine threshold
        if (fabs(readings.yaw_error) < FINE_THRESHOLD && fabs(readings.pitch_error) < FINE_THRESHOLD) {
            ESP_LOGI(TAG, "Tracking cycle completed successfully after %d attempts", attempt + 1);
            return ESP_OK;
        }

        // Log improvement (or lack thereof)
        ESP_LOGD(TAG, "Errors still above threshold, will attempt further adjustment");
    }

    // If we got here, we hit max attempts
    ESP_LOGW(TAG,
             "Reached max adjustment attempts (%d). Final errors: Yaw=%.3fV, Pitch=%.3fV",
             MAX_ADJUSTMENT_ATTEMPTS,
             readings.yaw_error,
             readings.pitch_error);

    return ESP_OK; // Not an error condition, just suboptimal alignment
}

// ########################## Power System Functions ##########################

/**
 * @brief Initialize power management GPIOs
 */
static esp_err_t power_management_init(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    // Configure power control pins
    const gpio_num_t power_gpios[] = {MOSFET_3V3_SENSOR_BUS_CUTOFF_GPIO,
                                      MOSFET_5V_BUS_CUTOFF_GPIO,
                                      MOSFET_6V2_SERVO_BUS_CUTOFF_GPIO,
                                      MOSFET_12V_FAN_PWM_GPIO};

    for (size_t i = 0; i < sizeof(power_gpios) / sizeof(power_gpios[0]); i++) {
        io_conf.pin_bit_mask = (1ULL << power_gpios[i]);
        esp_err_t ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure power management GPIO %d: %s", power_gpios[i], esp_err_to_name(ret));
            return ret;
        }

        // Start with all power buses OFF (LOW = OFF for MOSFET gates)
        gpio_set_level(power_gpios[i], 0);
    }

    ESP_LOGI(TAG, "Power management GPIOs initialized - All buses OFF");
    return ESP_OK;
}

/**
 * @brief Power on 3.3V sensor bus
 */
esp_err_t power_on_3V3_sensor_bus(void)
{
    if (!irrigation_system.sensors_powered) {
        gpio_set_level(MOSFET_3V3_SENSOR_BUS_CUTOFF_GPIO, 1);
        irrigation_system.sensors_powered = true;
        ESP_LOGI(TAG, "3.3V sensor bus powered ON - waiting %dms for stabilization", SENSOR_POWERUP_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(SENSOR_POWERUP_DELAY_MS));
    }
    return ESP_OK;
}

/**
 * @brief Power off 3.3V sensor bus
 */
esp_err_t power_off_3V3_sensor_bus(void)
{
    if (irrigation_system.sensors_powered) {
        gpio_set_level(MOSFET_3V3_SENSOR_BUS_CUTOFF_GPIO, 0);
        irrigation_system.sensors_powered = false;
        ESP_LOGI(TAG, "3.3V sensor bus powered OFF");
    }
    return ESP_OK;
}

/**
 * @brief Power on 6.2V servo bus
 */
static esp_err_t power_on_servo_bus(void)
{
    // if (!irrigation_system.servos_powered) {
    if (true) {
        gpio_set_level(MOSFET_6V2_SERVO_BUS_CUTOFF_GPIO, 1);
        // irrigation_system.servos_powered = true;
        ESP_LOGI(TAG, "6.2V servo bus powered ON - waiting %dms for stabilization", SERVO_POWERUP_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(SERVO_POWERUP_DELAY_MS));
    }
    return ESP_OK;
}

/**
 * @brief Power off 6.2V servo bus
 */
static esp_err_t power_off_servo_bus(void)
{
    // if (irrigation_system.servos_powered) {
    if (true) {
        gpio_set_level(MOSFET_6V2_SERVO_BUS_CUTOFF_GPIO, 0);
        // irrigation_system.servos_powered = false;
        ESP_LOGI(TAG, "6.2V servo bus powered OFF");
    }
    return ESP_OK;
}


// -----------------#################################-----------------
// -----------------############# TASKS #############-----------------
// -----------------#################################-----------------


// Task for periodically checking PV panel alignment via photoresistors (ADS1115 Dev#2)
void solar_tracking_task(void *pvParameters)
{
    const char *task_tag = "solar_tracking";
    ESP_LOGI(task_tag, "Solar Tracking Task started. Tracking interval: %d minutes.", TRACKING_INTERVAL_MS / 60000);

    // Initialize servo PWM
    esp_err_t ret = solar_t_servo_pwm_init();
    if (ret != ESP_OK) {
        ESP_LOGE(task_tag, "Failed to initialize servo PWM: %s", esp_err_to_name(ret));
        ESP_LOGE(task_tag, "Solar tracking task terminating");
        vTaskDelete(NULL);
        return;
    }

    // Power on servo bus and move servos to center position initially
    power_on_servo_bus();
    solar_t_servo_set_position(SERVO_YAW_CHANNEL, SERVO_CENTER_DUTY);
    solar_t_servo_set_position(SERVO_PITCH_CHANNEL, SERVO_CENTER_DUTY);
    ESP_LOGI(task_tag, "Servos powered on and moved to center position");

    // Power off servos when not tracking
    power_off_servo_bus();

    // waiting for other systems to initialize
    vTaskDelay(pdMS_TO_TICKS(2000));

    uint32_t tracking_cycles = 0;
    uint32_t successful_reads = 0;
    uint32_t servo_adjustments = 0;

    while (1) {
        tracking_cycles++;

        ESP_LOGD(task_tag, "Starting tracking cycle %" PRIu32 "", tracking_cycles);

        esp_err_t cycle_result = solar_tracker_loop();
        if (cycle_result == ESP_OK) {
            successful_reads++;

            // Update global debug display variables with final readings
            photoresistor_readings_t final_readings;
            if (solar_t_read_photoresistors(&final_readings) == ESP_OK) {
                if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    latest_ads_voltages[0][0] = final_readings.left_top;
                    latest_ads_voltages[0][1] = final_readings.right_top;
                    latest_ads_voltages[0][2] = final_readings.left_bottom;
                    latest_ads_voltages[0][3] = final_readings.right_bottom;
                    xSemaphoreGive(xDisplayDataMutex);
                }
            }
        } else {
            ESP_LOGE(task_tag, "Tracking cycle failed: %s", esp_err_to_name(cycle_result));
        }

        // Periodic status report
        if (tracking_cycles % 12 == 0) // !During tests every 1 minute
        {
            ESP_LOGI(task_tag,
                     "Tracking stats: %" PRIu32 "/%" PRIu32 " successful reads, %" PRIu32 " servo adjustments",
                     successful_reads,
                     tracking_cycles,
                     servo_adjustments);
        }

        // Wait until next tracking cycle
        vTaskDelay(pdMS_TO_TICKS(TRACKING_INTERVAL_MS));
    }
}


// Task for periodically reading AS5600 angle
void as5600_read_task(void *pvParameters)
{
    ESP_LOGI(TAG, "AS5600 Read Task started. Initializing sensor...");

    // Initialize AS5600 Sensor within the task
    esp_err_t as5600_init_result = as5600_init_desc(&as5600_dev,
                                                    I2C_PORT_NUM,
                                                    AS5600_I2C_ADDR,
                                                    CONFIG_I2CDEV_DEFAULT_SDA_PIN,
                                                    CONFIG_I2CDEV_DEFAULT_SCL_PIN);
    if (as5600_init_result != ESP_OK) {
        ESP_LOGE(TAG, "AS5600: Failed to initialize descriptor: %s", esp_err_to_name(as5600_init_result));
        ESP_LOGE(TAG, "AS5600: Task terminating due to initialization failure");
        vTaskDelete(NULL);
        return;
    }

    // Then initialize the device itself
    as5600_init_result = as5600_init(&as5600_dev);
    if (as5600_init_result != ESP_OK) {
        ESP_LOGE(TAG, "AS5600: Failed to initialize device: %s", esp_err_to_name(as5600_init_result));
        ESP_LOGE(TAG, "AS5600: Task terminating due to initialization failure");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "AS5600: Sensor initialized successfully");

    // Wait a moment for the sensor's internal update task to start
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        float angle_deg = 0;
        uint16_t raw_counts = 0;
        float accumulated_counts = 0;

        esp_err_t ret_angle = as5600_read_angle_degrees(&as5600_dev, &angle_deg);
        esp_err_t ret_raw = as5600_read_raw_counts(&as5600_dev, &raw_counts);
        esp_err_t ret_acc = as5600_read_accumulated_counts(&as5600_dev, &accumulated_counts);

        if (ret_angle == ESP_OK && ret_raw == ESP_OK && ret_acc == ESP_OK) {
            // ESP_LOGW("AS5600", "Angle: %.2f deg, Raw: %u, Accumulated: %.1f", angle_deg,
            // raw_counts, accumulated_counts);

            // Debug display variables
            if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) { // Wait max 100ms
                latest_as5600_angle = angle_deg;
                latest_as5600_raw = raw_counts;
                xSemaphoreGive(xDisplayDataMutex);
            } else {
                ESP_LOGW("AS5600", "Could not get display mutex to update globals");
            }
        } else {
            ESP_LOGE("AS5600", "Failed to read - Angle: %d, Raw: %d, Acc: %d", ret_angle, ret_raw, ret_acc);
        }

        // Wait for 1 second
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// Re-add SHT4x Task definition
void SHT4xReadTask(void *pvParameters)
{
    static sht4x_t sht4x_dev;

    TaggedSensorData sht4xdata;
    sht4xdata.tag = SENSOR_SHT4X;
    sht4xdata.pressure = 0;

    // Note: Static variables are automatically zero-initialized, memset not needed
    ESP_ERROR_CHECK(
        sht4x_init_desc(&sht4x_dev, I2C_PORT_NUM, CONFIG_I2CDEV_DEFAULT_SDA_PIN, CONFIG_I2CDEV_DEFAULT_SCL_PIN));
    ESP_LOGI("sht4x", ">> Entering sht4x_init_desc()");
    ESP_ERROR_CHECK(sht4x_init(&sht4x_dev));

    // Get the measurement duration for high repeatability;
    uint8_t duration = sht4x_get_measurement_duration(&sht4x_dev);
    while (1) {
        // Trigger one measurement in single shot mode with high repeatability.
        ESP_ERROR_CHECK(sht4x_start_measurement(&sht4x_dev));
        // Wait until measurement is ready (duration returned from
        // *sht4x_get_measurement_duration*).
        vTaskDelay(duration); // duration is in ticks

        // retrieve the values and send it to the queue
        if (sht4x_get_results(&sht4x_dev, &sht4xdata.temperature, &sht4xdata.humidity) == ESP_OK) {
            // ESP_LOGI("SHT40", "Timestamp: %lu, SHT40  - Temperature: %.2f °C, Humidity: %.2f %%",
            // (unsigned long)xTaskGetTickCount(), sht4xdata.temperature, sht4xdata.humidity);
            if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) { // Wait max 100ms
                latest_sht_temp = sht4xdata.temperature;
                latest_sht_hum = sht4xdata.humidity;
                xSemaphoreGive(xDisplayDataMutex);
            } else {
                ESP_LOGW("SHT40", "Could not get display mutex to update globals");
            }
        } else {
            ESP_LOGI("SHT40", "SHT40 Failed to read sensor data.");
        }
        // 10 s delay
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Re-add BME280 Task definition
void BME280ReadTask(void *pvParameters)
{
    static bmp280_t bme280_dev;
    bmp280_params_t params;

    TaggedSensorData bme280data;
    bme280data.tag = SENSOR_BME280;

    // Initialize the sensor
    bmp280_init_default_params(&params);
    // Note: Static variables are automatically zero-initialized, memset not needed
    ESP_ERROR_CHECK(bmp280_init_desc(&bme280_dev,
                                     BMP280_I2C_ADDRESS_0,
                                     I2C_PORT_NUM,
                                     CONFIG_I2CDEV_DEFAULT_SDA_PIN,
                                     CONFIG_I2CDEV_DEFAULT_SCL_PIN));
    ESP_ERROR_CHECK(bmp280_init(&bme280_dev, &params));
    bool bme280p = bme280_dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", (bme280p ? "BME280" : "BMP280"));

    while (1) {
        // Set the sensor to forced mode to initiate a measurement
        ESP_ERROR_CHECK(bmp280_force_measurement(&bme280_dev));

        // Wait for the measurement to complete
        bool busy;
        do {
            ESP_ERROR_CHECK(bmp280_is_measuring(&bme280_dev, &busy));
            if (busy)
                vTaskDelay(pdMS_TO_TICKS(5)); // Wait for 5ms before checking again
        } while (busy);

        // Read the measurement results
        if (bmp280_read_float(&bme280_dev, &bme280data.temperature, &bme280data.pressure, &bme280data.humidity) ==
            ESP_OK) {
            // ESP_LOGI("BME280", "Timestamp: %lu, BME280 - Temperature: %.2f °C, Humidity: %.2f %%,
            // Pressure: %.2f hPa", (unsigned long)xTaskGetTickCount(),
            //  bme280data.temperature, bme280data.humidity, bme280data.pressure/100);

            // Debug display variables - Wait max 100ms
            if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                latest_bmp_temp = bme280data.temperature;
                latest_bmp_hum = bme280data.humidity;
                latest_bmp_press = bme280data.pressure;
                xSemaphoreGive(xDisplayDataMutex);
            } else {
                ESP_LOGW("BME280", "Could not get display mutex to update globals");
            }
        } else {
            ESP_LOGI("BME280", "BME280 Failed to read sensor data.");
        }
        // 10 s delay
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Task for reading INA219 voltage and current sensors
void ina219_read_task(void *pvParameters)
{
    ESP_LOGI(TAG, "INA219 Read Task started.");

    // Initialize each INA219 sensor
    for (int i = 0; i < INA219_DEVICE_COUNT; i++) {
        ina219_sensor_t *ina219_sensor = &ina219_devices[i];

        ESP_LOGI(TAG,
                 "Initializing INA219 %s (type: %d, addr: 0x%02x, shunt: %.3f ohm)...",
                 ina219_sensor->name,
                 ina219_sensor->type,
                 ina219_sensor->addr,
                 ina219_sensor->shunt_ohms);

        // Initialize descriptor
        esp_err_t init_desc_res = ina219_init_desc(&ina219_sensor->dev,
                                                   ina219_sensor->addr,
                                                   I2C_PORT_NUM,
                                                   CONFIG_I2CDEV_DEFAULT_SDA_PIN,
                                                   CONFIG_I2CDEV_DEFAULT_SCL_PIN);
        if (init_desc_res != ESP_OK) {
            ESP_LOGE(TAG,
                     "  Failed to initialize INA219 %s descriptor: %s",
                     ina219_sensor->name,
                     esp_err_to_name(init_desc_res));
            continue;
        }

        // Initialize device
        esp_err_t init_res = ina219_init(&ina219_sensor->dev);
        if (init_res != ESP_OK) {
            ESP_LOGE(TAG, "  Failed to initialize INA219 %s: %s", ina219_sensor->name, esp_err_to_name(init_res));
            continue;
        }

        // Configure device
        esp_err_t cfg_res = ina219_configure(&ina219_sensor->dev,
                                             INA219_BUS_RANGE_32V,        // 32V range for all sensors
                                             ina219_sensor->gain,         // Gain from sensor config
                                             INA219_RES_12BIT_8S,         // 12-bit resolution with 8 samples
                                             INA219_RES_12BIT_8S,         // 12-bit resolution with 8 samples
                                             INA219_MODE_CONT_SHUNT_BUS); // Continuous mode
        if (cfg_res != ESP_OK) {
            ESP_LOGE(TAG, "  Failed to configure INA219 %s: %s", ina219_sensor->name, esp_err_to_name(cfg_res));
            continue;
        }

        // Calibrate with the appropriate shunt resistor value
        ESP_LOGI(TAG, "  Calibrating INA219 %s with %.3f ohm shunt", ina219_sensor->name, ina219_sensor->shunt_ohms);
        esp_err_t cal_res = ina219_calibrate(&ina219_sensor->dev, ina219_sensor->shunt_ohms);
        if (cal_res != ESP_OK) {
            ESP_LOGE(TAG, "  Failed to calibrate INA219 %s: %s", ina219_sensor->name, esp_err_to_name(cal_res));
            continue;
        }

        // Mark as initialized if all steps succeeded
        ina219_sensor->initialized = true;
        ESP_LOGI(TAG, "  INA219 %s initialized successfully", ina219_sensor->name);
    }

    // Main task loop
    while (1) {
        for (int i = 0; i < INA219_DEVICE_COUNT; i++) {
            ina219_sensor_t *ina219_sensor = &ina219_devices[i];

            // Skip sensors that failed initialization
            if (!ina219_sensor->initialized) {
                continue;
            }

            float voltage = -999.9;
            float current = -999.9;
            float power = -999.9;

            esp_err_t v_res = ina219_get_bus_voltage(&ina219_sensor->dev, &voltage);
            esp_err_t c_res = ina219_get_current(&ina219_sensor->dev, &current);
            esp_err_t p_res = ina219_get_power(&ina219_sensor->dev, &power);

            if (v_res == ESP_OK && c_res == ESP_OK && p_res == ESP_OK) {
                // Update global values with mutex protection
                if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    latest_ina219_voltage[i] = voltage;
                    latest_ina219_current[i] = current;
                    latest_ina219_power[i] = power;
                    xSemaphoreGive(xDisplayDataMutex);
                } else {
                    ESP_LOGW(TAG, "INA219 %s: Could not get display mutex to update globals", ina219_sensor->name);
                }
            } else {
                ESP_LOGE(TAG,
                         "INA219 %s: Read error - Voltage: %s, Current: %s, Power: %s",
                         ina219_sensor->name,
                         esp_err_to_name(v_res),
                         esp_err_to_name(c_res),
                         esp_err_to_name(p_res));
            }
        }

        // Read every 2 seconds
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void local_power_monitoring_task(void *pvParameters)
{
    while (1) {
        // TODO: Implement power monitoring functionality
        /*
        // Read battery voltage/current
        // read_battery_status();

        // Read solar panel power generation
        // read_solar_generation();

        // Read system consumption
        // read_system_consumption();

        // Trigger power management decisions
        // update_power_management_strategy();
        */

        // Placeholder delay
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void serial_display_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Serial Display Task started.");
    vTaskDelay(pdMS_TO_TICKS(1000));
    // ANSI escape codes
    const char *ANSI_CLEAR_SCREEN = "\033[2J";
    const char *ANSI_CURSOR_HOME = "\033[H";

    char buffer[100]; // Buffer for formatting lines

    // Declare local variables for sensor readings outside the loop
    float sht_t, sht_h, bmp_t, bmp_p, bmp_h, as_a;
    uint16_t as_r;
    float ads_voltages[ADS1115_DEVICE_COUNT][4];
    float ina219_v[INA219_DEVICE_COUNT], ina219_c[INA219_DEVICE_COUNT], ina219_p[INA219_DEVICE_COUNT];

    while (1) {
        // --- Read Global Variables Safely ---
        if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // SHT4x
            sht_t = latest_sht_temp;
            sht_h = latest_sht_hum;
            // BMP280
            bmp_t = latest_bmp_temp;
            bmp_p = latest_bmp_press;
            bmp_h = latest_bmp_hum;
            // Copy ADS1115 arrays
            for (int dev = 0; dev < ADS1115_DEVICE_COUNT; dev++) {
                for (int ch = 0; ch < 4; ch++) {
                    ads_voltages[dev][ch] = latest_ads_voltages[dev][ch];
                }
            }
            // AS5600
            as_a = latest_as5600_angle;
            as_r = latest_as5600_raw;
            // Read INA219 values
            for (int i = 0; i < INA219_DEVICE_COUNT; i++) {
                ina219_v[i] = latest_ina219_voltage[i];
                ina219_c[i] = latest_ina219_current[i];
                ina219_p[i] = latest_ina219_power[i];
            }

            xSemaphoreGive(xDisplayDataMutex);
        } else {
            ESP_LOGW(TAG, "Serial display task could not get mutex");
            // Optionally print an error message to serial?
            printf("%s%s!!! Failed to get mutex !!!\n", ANSI_CLEAR_SCREEN, ANSI_CURSOR_HOME);
            vTaskDelay(pdMS_TO_TICKS(SERIAL_DISPLAY_INTERVAL_MS));
            continue;
        }
        // --- End Read ---

        // --- Format and Print to Serial Monitor ---
        // Clear screen and move cursor to top-left

        // printf("%s%s", ANSI_CLEAR_SCREEN, ANSI_CURSOR_HOME);

        // Print header
        printf("--- Sensor Readings ---\n");

        // Print sensor data (chat says using snprintf for safety is good practice)
        snprintf(buffer, sizeof(buffer), "SHT4x: Temp=%.1f C, Hum=%.1f %%\n", sht_t, sht_h);
        printf("%s", buffer);

        snprintf(buffer,
                 sizeof(buffer),
                 "BMP280: Temp=%.1f C, Press=%.0f hPa, Hum=%.1f %%\n",
                 bmp_t,
                 bmp_p / 100,
                 bmp_h); // Assuming BME & Pa -> hPa
        printf("%s", buffer);

        // Display ADS1115 readings organized by function
        snprintf(buffer,
                 sizeof(buffer),
                 "ADS0 (Photo): LT=%.3f RT=%.3f LB=%.3f RB=%.3f\n",
                 ads_voltages[0][0],
                 ads_voltages[0][1],
                 ads_voltages[0][2],
                 ads_voltages[0][3]);
        printf("%s", buffer);

        snprintf(buffer,
                 sizeof(buffer),
                 "ADS1 (Moist): Z1=%.3f Z2=%.3f Z3=%.3f Z4=%.3f\n",
                 ads_voltages[1][0],
                 ads_voltages[1][1],
                 ads_voltages[1][2],
                 ads_voltages[1][3]);
        printf("%s", buffer);

        snprintf(buffer,
                 sizeof(buffer),
                 "ADS2 (Mixed): Z5=%.3f Press=%.3f Ch2=%.3f Ch3=%.3f\n",
                 ads_voltages[2][0],
                 ads_voltages[2][1],
                 ads_voltages[2][2],
                 ads_voltages[2][3]);
        printf("%s", buffer);

        snprintf(buffer, sizeof(buffer), "AS5600: Angle=%.1f deg, Raw=%u\n", as_a, as_r);
        printf("%s", buffer);

        // Add INA219 display
        for (int i = 0; i < INA219_DEVICE_COUNT; i++) {
            snprintf(buffer,
                     sizeof(buffer),
                     "INA219_%s: V=%.2f V, I=%.2f mA, P=%.2f mW\n",
                     ina219_devices[i].name,
                     ina219_v[i],
                     ina219_c[i] * 1000,
                     ina219_p[i] * 1000);
            printf("%s", buffer);
        }

        printf("-----------------------\n");
        // Flush stdout buffer to ensure data is sent immediately
        fflush(stdout);

        // --- End Format and Print ---

        vTaskDelay(pdMS_TO_TICKS(SERIAL_DISPLAY_INTERVAL_MS));
    }
}

/**
 * @brief Background task for retrying failed ADS1115 devices
 *
 * Continuously monitors for failed devices and attempts to recover them
 * using exponential backoff strategy (1min -> 3min -> 10min max).
 * Runs in background to provide automatic recovery without user intervention.
 *
 * @param pvParameters Unused task parameter
 */
static void ads1115_retry_task(void *pvParameters)
{
    ESP_LOGI(TAG, "ADS1115 retry task started");

    const uint32_t CHECK_INTERVAL_MS = 5000;    // Check every 5 seconds
    const uint32_t MAX_RETRY_DELAY_MS = 600000; // Max 10 minutes between retries

    while (1) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        bool retry_attempted = false;

        // Check each device for retry
        for (uint8_t device = 0; device < ADS1115_DEVICE_COUNT; device++) {
            ads1115_device_t *status = &ads1115_devices[device];

            // Skip if device is already working
            if (status->initialized) {
                continue;
            }

            // Check if it's time to retry
            uint32_t time_since_last_retry = current_time - status->last_retry_time;
            if (time_since_last_retry >= status->next_retry_delay_ms) {
                ESP_LOGI(TAG,
                         "Retrying ADS1115 device %d (%s) - attempt %d",
                         device,
                         ads1115_devices[device].name,
                         status->retry_count + 1);

                // Attempt to reinitialize
                esp_err_t result = ads1115_init_device(device);
                status->last_retry_time = current_time;
                status->retry_count++;
                retry_attempted = true;

                if (result == ESP_OK) {
                    status->initialized = true;
                    status->retry_count = 0;             // Reset retry count on success
                    status->next_retry_delay_ms = 60000; // Reset delay to 1 minute
                    ESP_LOGI(TAG,
                             "✓ ADS1115 device %d (%s) successfully recovered!",
                             device,
                             ads1115_devices[device].name);
                } else {
                    ESP_LOGW(TAG,
                             "✗ ADS1115 device %d (%s) retry failed: %s",
                             device,
                             ads1115_devices[device].name,
                             esp_err_to_name(result));

                    // Exponential backoff: 1min -> 3min -> 10min -> 10min (max)
                    if (status->retry_count == 1) {
                        status->next_retry_delay_ms = 180000; // 3 minutes
                    } else if (status->retry_count >= 2) {
                        status->next_retry_delay_ms = MAX_RETRY_DELAY_MS; // 10 minutes
                    }

                    ESP_LOGW(TAG, "    Next retry in %" PRIu32 " minutes", status->next_retry_delay_ms / 60000);
                }
            }
        }

        // Log status periodically
        if (retry_attempted) {
            uint8_t working_count = 0;
            for (uint8_t i = 0; i < ADS1115_DEVICE_COUNT; i++) {
                if (ads1115_devices[i].initialized) {
                    working_count++;
                }
            }
            ESP_LOGI(TAG, "ADS1115 status: %d/%d devices working", working_count, ADS1115_DEVICE_COUNT);
        }

        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_MS));
    }
}

// -----------------##############################-----------------
// -----------------############# MAIN ###########-----------------
// -----------------##############################-----------------

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Starting ESP...");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


    /* Display Mutex creation */
    xDisplayDataMutex = xSemaphoreCreateMutex();
    if (xDisplayDataMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create display data mutex!");
        // Handle error appropriately, maybe halt or return an error code
    } else {
        ESP_LOGI(TAG, "Display data mutex created successfully.");
    }

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }
    printf("%" PRIu32 "MB %s flash\n",
           flash_size / (uint32_t) (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // Initialize Wi-Fi and SNTP
    wifi_init_sta();

    // Initialize Power Management
    power_management_init();

    // Initialize I2C subsystem
    ESP_LOGI(TAG, "Initializing I2C...");
    ESP_ERROR_CHECK(i2cdev_init());

    // Initialize all ADS1115 devices
    esp_err_t ads_init_result = ads1115_init_all();
    if (ads_init_result == ESP_FAIL) {
        ESP_LOGW(TAG,
                 "No ADS1115 devices available at startup - system will continue with limited "
                 "functionality");
        ESP_LOGW(TAG, "The retry task will attempt to recover failed devices automatically");
    } else {
        ESP_LOGI(TAG, "ADS1115 initialization completed successfully");
    }

    //---------- TASKS ----------
    ESP_LOGI(TAG, "Creating tasks...");
    // Core sensor tasks
    xTaskCreate(solar_tracking_task, "SolarTrackingTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
    xTaskCreate(irrigation_task, "IrrigationTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
    // xTaskCreate(weather_task, "WeatherTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

    xTaskCreate(SHT4xReadTask, "SHT4xReadTask", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(BME280ReadTask, "BME280ReadTask", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(as5600_read_task, "AS5600ReadTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

    // Local power monitoring task
    // xTaskCreate(local_power_monitoring_task, "LocalPowerMonitoringTask", configMINIMAL_STACK_SIZE
    // * 4, NULL, 5, NULL);
    xTaskCreate(ina219_read_task, "INA219ReadTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

    // ADS1115 retry task for automatic recovery
    xTaskCreate(ads1115_retry_task, "Ads1115RetryTask", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

    // Communication tasks
    // xTaskCreate(telemetry_task, "TelemetryTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
    // xTaskCreate(mppt_communication_task, "mppt_communication_task", configMINIMAL_STACK_SIZE * 4,
    // NULL, 5, NULL);

    // Misc tasks
    xTaskCreate(serial_display_task, "serial_display_task", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

    ESP_LOGI(TAG, "Tasks created.");
}
