#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include "bmp280.h"
#include "sht4x.h"
#include "ads111x.h"
#include "as5600.h"
#include "ina219.h"

#include "esp_log.h"
#include "esp_err.h"

// ########################## Global settings and variables ##########################
#define TAG "ESP_SOLARIUM_1"

// Debuging display variables
#define SERIAL_DISPLAY_INTERVAL_MS 2000 // Update display every 2 seconds

SemaphoreHandle_t xDisplayDataMutex = NULL;

float latest_sht_temp = -999.9;
float latest_sht_hum = -999.9;
float latest_bmp_temp = -999.9;
float latest_bmp_press = -999.9;
float latest_bmp_hum = -999.9;       // If BME280
float latest_ads0_ch0_volt = -999.9; // Example PV Tracking
float latest_ads1_ch0_volt = -999.9; // Example Moisture
float latest_as5600_angle = -999.9;
uint16_t latest_as5600_raw = 0;

typedef enum { SENSOR_NONE = 0, SENSOR_BME280, SENSOR_SHT4X } SensorType;

typedef struct
{
    SensorType tag;
    float temperature;
    float humidity;
    float pressure;
} TaggedSensorData;

// ########################## I2C Settings ##########################
#define I2C_PORT_NUM I2C_NUM_0
// Determine lowest common speed required by all devices on this port
// ADS111x: 400kHz max (as per your note)
// AS5600: 1MHz max (but running at 400kHz)
// SHT4x: 1MHz max
// BMP280: 3.4MHz max
// Lowest is 400kHz
// #define I2C_COMMON_SPEED_HZ 400000

// ########################## AS5600 Settings ##########################

#define AS5600_I2C_ADDR AS5600_DEFAULT_ADDRESS // 0x36
// Global AS5600 device descriptor
static as5600_dev_t as5600_dev;

// ########################## ADS1115 Settings ##########################
#define ADS1115_DEVICE_COUNT 3                  // Define the number of ADS1115 modules used
#define GAIN                 ADS111X_GAIN_4V096 // +-4.096V

// ADS1115 addresses
static const uint8_t ads1115_addr[ADS1115_DEVICE_COUNT] = {
    ADS111X_ADDR_GND, // 0x48 - Mixed sensors
    ADS111X_ADDR_VCC, // 0x49 - Moisture sensors
    ADS111X_ADDR_SDA  // 0x4A - Photoresistor array
};

static i2c_dev_t ads1115_devices[ADS1115_DEVICE_COUNT]; // Descriptors
static float gain_val;                                  // Gain value

// ########################## Photoresistor Settings ##########################

// Photoresistor layout:
// [Ch0: Left-Top]  [Ch1: Right-Top]
// [Ch2: Left-Bot]  [Ch3: Right-Bot]

typedef struct
{
    float left_top;     // Ch0
    float right_top;    // Ch1
    float left_bottom;  // Ch2
    float right_bottom; // Ch3
    float yaw_error;    // (left_avg - right_avg)
    float pitch_error;  // (top_avg - bottom_avg)
} photoresistor_readings_t;

// ########################## INA219 Settings ##########################
// Define INA219 sensor types
typedef enum {
    INA219_TYPE_GENERIC, // Generic INA219 with 0.1Ω shunt resistor (3.2A max)
    INA219_TYPE_DFROBOT  // DFRobot SEN0291 with 0.01Ω shunt resistor (8A max)
} ina219_sensor_type_t;

// INA219 sensor configuration structure
typedef struct
{
    ina219_t dev;              // Device handle
    ina219_sensor_type_t type; // Sensor type
    uint8_t addr;              // I2C address
    float shunt_ohms;          // Shunt resistor value in ohms
    ina219_gain_t gain;        // Gain setting
    const char *name;          // Human-readable name
    bool initialized;          // Whether initialization succeeded
} ina219_sensor_t;

// INA219 global variables
#define INA219_DEVICE_COUNT 1 // Define number of INA219 modules (planning for 2, 1 for now)
float latest_ina219_voltage[INA219_DEVICE_COUNT] = { -999.9 };
float latest_ina219_current[INA219_DEVICE_COUNT] = { -999.9 };
float latest_ina219_power[INA219_DEVICE_COUNT] = { -999.9 };

// INA219 sensor configurations
static ina219_sensor_t ina219_devices[INA219_DEVICE_COUNT] = {
    { .type = INA219_TYPE_GENERIC,
        .addr = INA219_ADDR_GND_GND, // 0x40
        .shunt_ohms = 0.1f,          // R100
        .gain = INA219_GAIN_0_5,     // ±2A range
        .name = "GEN_3.2A",
        .initialized = false },
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

// ########################## FUNCTIONS ################################

// Helper function for handling of reading a single channel in single-shot mode
static esp_err_t read_ads1115_channel(uint8_t device_index, ads111x_mux_t mux, int16_t *raw_value, float *voltage)
{
    // ESP_LOGI(TAG, "[Dev %d] Reading channel for mux %d...", device_index, mux);
    if (!raw_value)
    { // Ensure raw_value pointer is valid
        return ESP_ERR_INVALID_ARG;
    }
    if (device_index >= ADS1115_DEVICE_COUNT)
    {
        ESP_LOGE(TAG, "[Dev %d] Invalid ADS1115 device index!", device_index);
        return ESP_ERR_INVALID_ARG;
    }

    i2c_dev_t *dev = &ads1115_devices[device_index];
    esp_err_t ret;

    // 1. Set MUX
    // ESP_LOGI(TAG, "[Dev %d] Setting MUX...", device_index);
    ret = ads111x_set_input_mux(dev, mux);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "[Dev %d] Failed to set MUX: %s (%d)", device_index, esp_err_to_name(ret), ret);
        // reconfigure_ads1115_device(device_index); // Attempt reconfig
        return ret;
    }
    // ESP_LOGI(TAG, "[Dev %d] MUX set OK.", device_index);

    // 2. Start Conversion
    // ESP_LOGI(TAG, "[Dev %d] Starting conversion...", device_index);
    ret = ads111x_start_conversion(dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "[Dev %d] Failed to start conversion: %s (%d)", device_index, esp_err_to_name(ret), ret);
        // reconfigure_ads1115_device(device_index); // Attempt reconfig
        return ret;
    }
    // ESP_LOGI(TAG, "[Dev %d] Conversion started OK.", device_index);

    // ADD A SMALL DELAY HERE
    vTaskDelay(pdMS_TO_TICKS(1));

    // 3. Wait Busy (with timeout)
    // ESP_LOGI(TAG, "[Dev %d] Waiting for conversion to complete...", device_index);
    // Max conversion time for 32 SPS is ~32ms. Add some margin. Timeout after ~100ms.
    int conversion_timeout_ms = 100; // Increased timeout
    int delay_ms = 5;                // Check every 5ms
    int elapsed_ms = 0;
    bool busy = true;
    do
    {
        ret = ads111x_is_busy(dev, &busy);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "[Dev %d] Failed to check busy status: %s (%d)", device_index, esp_err_to_name(ret), ret);
            // reconfigure_ads1115_device(device_index); // Attempt reconfig
            return ret;
        }
        // ESP_LOGI(TAG, "[Dev %d] Checked busy status: ret=%d, busy=%d", device_index, ret, busy);

        // Add a small delay after each busy check to avoid polling too rapidly
        vTaskDelay(1);

        if (busy)
        {
            // ESP_LOGI(TAG, "[Dev %d] Still busy...", device_index); // Optional: uncomment for extreme detail
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
            elapsed_ms += delay_ms;
            if (elapsed_ms > conversion_timeout_ms)
            {
                ESP_LOGE(TAG, "[Dev %d] Conversion timeout after %d ms", device_index, conversion_timeout_ms);
                // reconfigure_ads1115_device(device_index); // Attempt reconfig on timeout
                return ESP_ERR_TIMEOUT;
            }
        }
    }
    while (busy);
    // ESP_LOGI(TAG, "[Dev %d] Conversion complete (waited ~%d ms).", device_index, elapsed_ms);

    // 4. Read Value
    // ESP_LOGI(TAG, "[Dev %d] Reading value...", device_index);
    ret = ads111x_get_value(dev, raw_value);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "[Dev %d] Failed to get value: %s (%d)", device_index, esp_err_to_name(ret), ret);
        // reconfigure_ads1115_device(device_index); // Attempt reconfig
        return ret;
    }
    // ESP_LOGI(TAG, "[Dev %d] Read value OK (Raw: %d).", device_index, *raw_value);

    // 5. Calculate Voltage (if requested)
    if (voltage)
    {
        *voltage = (*raw_value / (float)ADS111X_MAX_VALUE) * gain_val;
        // ESP_LOGI(TAG, "[Dev %d] Calculated voltage: %.4fV", device_index, *voltage);
    }

    // 6. Return ESP_OK
    // ESP_LOGI(TAG, "[Dev %d] Channel read finished successfully.", device_index);
    return ESP_OK;
}

// Task for periodically checking PV panel alignment via photoresistors (ADS1115 Dev 0)
void pv_tracking_task(void *pvParameters)
{
    const int PV_CHECK_INTERVAL_MIN = 5;
    ESP_LOGI(TAG, "PV Tracking Task started. Checking every %d minutes.", PV_CHECK_INTERVAL_MIN);

    while (1)
    {
        int16_t raw_val;
        float voltage;

        // Example: Read from Channel 0 on Device 0 (PV Tracking)
        esp_err_t ret = read_ads1115_channel(0, ADS111X_MUX_0_GND, &raw_val, &voltage);

        if (ret == ESP_OK)
        {
            // ESP_LOGW(TAG, "[PV Track Dev 0/Ch 0] Raw: %d, Voltage: %.4fV", raw_val, voltage);
            if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            { // Wait max 100ms
                latest_ads0_ch0_volt = voltage;
                xSemaphoreGive(xDisplayDataMutex);
            }
            else
            {
                ESP_LOGW("ads0_ch0_potentiometr", "Could not get display mutex to update globals");
            }
            // TODO: Add logic here:
            // 1. Compare voltage/raw_val against baseline/target or compare multiple photoresistor values.
            // 2. If adjustment needed: control servo.
            // 3. Possibly re-read after adjustment to verify.
        }
        else
        {
            ESP_LOGE(TAG, "[PV Track Dev 0/Ch 0] Failed to read: %s", esp_err_to_name(ret));
        }

        // Delay until next check
        // vTaskDelay(pdMS_TO_TICKS(PV_CHECK_INTERVAL_MIN * 60 * 1000));
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// Task for periodically reading moisture sensor (ADS1115 Dev 1)
void scheduled_moisture_task(void *pvParameters)
{
    const int MOISTURE_CHECK_INTERVAL_MIN = 60;
    ESP_LOGI(TAG, "Scheduled Moisture Task started. Checking every %d minutes.", MOISTURE_CHECK_INTERVAL_MIN);

    while (1)
    {
        int16_t raw_val;
        float voltage;

        // Example: Read from Channel 0 on Device 1 (Scheduled Moisture)
        // TODO: Adapt to read from the actual channel connected to the moisture sensor
        esp_err_t ret = read_ads1115_channel(1, ADS111X_MUX_0_GND, &raw_val, &voltage);
        // esp_err_t ret = read_ads1115_channel(2, ADS111X_MUX_0_GND, &raw_val, &voltage); // Device 2/Ch 0 future moisture sensor

        if (ret == ESP_OK)
        {
            // ESP_LOGW(TAG, "[Moisture Dev 1/Ch 0] Raw: %d, Voltage: %.4fV", raw_val, voltage);
            if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            { // Wait max 100ms
                latest_ads1_ch0_volt = voltage;
                xSemaphoreGive(xDisplayDataMutex);
            }
            else
            {
                ESP_LOGW("ads1_ch0_moisture", "Could not get display mutex to update globals");
            }
            // TODO: Add logic here to process/store/transmit the moisture reading.
        }
        else
        {
            ESP_LOGE(TAG, "[Moisture Dev 1/Ch 0] Failed to read: %s", esp_err_to_name(ret));
        }

        // Delay until next check
        // vTaskDelay(pdMS_TO_TICKS(MOISTURE_CHECK_INTERVAL_MIN * 60 * 1000));
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// NOTE: On-demand reading for Device 2 doesn't need a task here.
//       Call read_ads1115_channel(2, mux, &raw, &voltage) from the
//       event handler (e.g., button press, network command) that triggers it.

/*
// Function to print sensor data based on the sensor type
static void print_sensor_data(const char *label, const TaggedSensorData *data) {
    // ... existing code ...
}
*/

// Task for periodically reading AS5600 angle
void as5600_read_task(void *pvParameters)
{
    ESP_LOGI(TAG, "AS5600 Read Task started. Initializing sensor...");

    // Initialize AS5600 Sensor within the task
    esp_err_t as5600_init_result = as5600_init_desc(&as5600_dev, I2C_PORT_NUM, AS5600_I2C_ADDR, CONFIG_I2CDEV_DEFAULT_SDA_PIN, CONFIG_I2CDEV_DEFAULT_SCL_PIN);
    if (as5600_init_result != ESP_OK)
    {
        ESP_LOGE(TAG, "AS5600: Failed to initialize descriptor: %s", esp_err_to_name(as5600_init_result));
        ESP_LOGE(TAG, "AS5600: Task terminating due to initialization failure");
        vTaskDelete(NULL);
        return;
    }

    // Then initialize the device itself
    as5600_init_result = as5600_init(&as5600_dev);
    if (as5600_init_result != ESP_OK)
    {
        ESP_LOGE(TAG, "AS5600: Failed to initialize device: %s", esp_err_to_name(as5600_init_result));
        ESP_LOGE(TAG, "AS5600: Task terminating due to initialization failure");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "AS5600: Sensor initialized successfully");

    // Wait a moment for the sensor's internal update task to start
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1)
    {
        float angle_deg = 0;
        uint16_t raw_counts = 0;
        float accumulated_counts = 0;

        esp_err_t ret_angle = as5600_read_angle_degrees(&as5600_dev, &angle_deg);
        esp_err_t ret_raw = as5600_read_raw_counts(&as5600_dev, &raw_counts);
        esp_err_t ret_acc = as5600_read_accumulated_counts(&as5600_dev, &accumulated_counts);

        if (ret_angle == ESP_OK && ret_raw == ESP_OK && ret_acc == ESP_OK)
        {
            // ESP_LOGW("AS5600", "Angle: %.2f deg, Raw: %u, Accumulated: %.1f", angle_deg, raw_counts, accumulated_counts);

            // Debug display variables
            if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            { // Wait max 100ms
                latest_as5600_angle = angle_deg;
                latest_as5600_raw = raw_counts;
                xSemaphoreGive(xDisplayDataMutex);
            }
            else
            {
                ESP_LOGW("AS5600", "Could not get display mutex to update globals");
            }
        }
        else
        {
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
    ESP_ERROR_CHECK(sht4x_init_desc(&sht4x_dev, I2C_PORT_NUM, CONFIG_I2CDEV_DEFAULT_SDA_PIN, CONFIG_I2CDEV_DEFAULT_SCL_PIN));
    ESP_LOGI("sht4x", ">> Entering sht4x_init_desc()");
    ESP_ERROR_CHECK(sht4x_init(&sht4x_dev));

    // Get the measurement duration for high repeatability;
    uint8_t duration = sht4x_get_measurement_duration(&sht4x_dev);
    while (1)
    {
        // Trigger one measurement in single shot mode with high repeatability.
        ESP_ERROR_CHECK(sht4x_start_measurement(&sht4x_dev));
        // Wait until measurement is ready (duration returned from *sht4x_get_measurement_duration*).
        vTaskDelay(duration); // duration is in ticks

        // retrieve the values and send it to the queue
        if (sht4x_get_results(&sht4x_dev, &sht4xdata.temperature, &sht4xdata.humidity) == ESP_OK)
        {
            // ESP_LOGI("SHT40", "Timestamp: %lu, SHT40  - Temperature: %.2f °C, Humidity: %.2f %%", (unsigned long)xTaskGetTickCount(), sht4xdata.temperature, sht4xdata.humidity);
            if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            { // Wait max 100ms
                latest_sht_temp = sht4xdata.temperature;
                latest_sht_hum = sht4xdata.humidity;
                xSemaphoreGive(xDisplayDataMutex);
            }
            else
            {
                ESP_LOGW("SHT40", "Could not get display mutex to update globals");
            }
        }
        else
        {
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
    ESP_ERROR_CHECK(bmp280_init_desc(&bme280_dev, BMP280_I2C_ADDRESS_0, I2C_PORT_NUM, CONFIG_I2CDEV_DEFAULT_SDA_PIN, CONFIG_I2CDEV_DEFAULT_SCL_PIN));
    ESP_ERROR_CHECK(bmp280_init(&bme280_dev, &params));
    bool bme280p = bme280_dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", (bme280p ? "BME280" : "BMP280"));

    while (1)
    {
        // Set the sensor to forced mode to initiate a measurement
        ESP_ERROR_CHECK(bmp280_force_measurement(&bme280_dev));

        // Wait for the measurement to complete
        bool busy;
        do
        {
            ESP_ERROR_CHECK(bmp280_is_measuring(&bme280_dev, &busy));
            if (busy)
                vTaskDelay(pdMS_TO_TICKS(5)); // Wait for 5ms before checking again
        }
        while (busy);

        // Read the measurement results
        if (bmp280_read_float(&bme280_dev, &bme280data.temperature, &bme280data.pressure, &bme280data.humidity) == ESP_OK)
        {
            // ESP_LOGI("BME280", "Timestamp: %lu, BME280 - Temperature: %.2f °C, Humidity: %.2f %%, Pressure: %.2f hPa", (unsigned long)xTaskGetTickCount(),
            //  bme280data.temperature, bme280data.humidity, bme280data.pressure/100);

            // Debug display variables - Wait max 100ms
            if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                latest_bmp_temp = bme280data.temperature;
                latest_bmp_hum = bme280data.humidity;
                latest_bmp_press = bme280data.pressure;
                xSemaphoreGive(xDisplayDataMutex);
            }
            else
            {
                ESP_LOGW("BME280", "Could not get display mutex to update globals");
            }
        }
        else
        {
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
    for (int i = 0; i < INA219_DEVICE_COUNT; i++)
    {
        ina219_sensor_t *ina219_sensor = &ina219_devices[i];

        ESP_LOGI(TAG, "Initializing INA219 %s (type: %d, addr: 0x%02x, shunt: %.3f ohm)...", ina219_sensor->name, ina219_sensor->type, ina219_sensor->addr, ina219_sensor->shunt_ohms);

        // Initialize descriptor
        esp_err_t init_desc_res = ina219_init_desc(&ina219_sensor->dev, ina219_sensor->addr, I2C_PORT_NUM, CONFIG_I2CDEV_DEFAULT_SDA_PIN, CONFIG_I2CDEV_DEFAULT_SCL_PIN);
        if (init_desc_res != ESP_OK)
        {
            ESP_LOGE(TAG, "  Failed to initialize INA219 %s descriptor: %s", ina219_sensor->name, esp_err_to_name(init_desc_res));
            continue;
        }

        // Initialize device
        esp_err_t init_res = ina219_init(&ina219_sensor->dev);
        if (init_res != ESP_OK)
        {
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
        if (cfg_res != ESP_OK)
        {
            ESP_LOGE(TAG, "  Failed to configure INA219 %s: %s", ina219_sensor->name, esp_err_to_name(cfg_res));
            continue;
        }

        // Calibrate with the appropriate shunt resistor value
        ESP_LOGI(TAG, "  Calibrating INA219 %s with %.3f ohm shunt", ina219_sensor->name, ina219_sensor->shunt_ohms);
        esp_err_t cal_res = ina219_calibrate(&ina219_sensor->dev, ina219_sensor->shunt_ohms);
        if (cal_res != ESP_OK)
        {
            ESP_LOGE(TAG, "  Failed to calibrate INA219 %s: %s", ina219_sensor->name, esp_err_to_name(cal_res));
            continue;
        }

        // Mark as initialized if all steps succeeded
        ina219_sensor->initialized = true;
        ESP_LOGI(TAG, "  INA219 %s initialized successfully", ina219_sensor->name);
    }

    // Main task loop
    while (1)
    {
        for (int i = 0; i < INA219_DEVICE_COUNT; i++)
        {
            ina219_sensor_t *ina219_sensor = &ina219_devices[i];

            // Skip sensors that failed initialization
            if (!ina219_sensor->initialized)
            {
                continue;
            }

            float voltage = -999.9;
            float current = -999.9;
            float power = -999.9;

            esp_err_t v_res = ina219_get_bus_voltage(&ina219_sensor->dev, &voltage);
            esp_err_t c_res = ina219_get_current(&ina219_sensor->dev, &current);
            esp_err_t p_res = ina219_get_power(&ina219_sensor->dev, &power);

            if (v_res == ESP_OK && c_res == ESP_OK && p_res == ESP_OK)
            {
                // Update global values with mutex protection
                if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    latest_ina219_voltage[i] = voltage;
                    latest_ina219_current[i] = current;
                    latest_ina219_power[i] = power;
                    xSemaphoreGive(xDisplayDataMutex);
                }
                else
                {
                    ESP_LOGW(TAG, "INA219 %s: Could not get display mutex to update globals", ina219_sensor->name);
                }
            }
            else
            {
                ESP_LOGE(TAG, "INA219 %s: Read error - Voltage: %s, Current: %s, Power: %s", ina219_sensor->name, esp_err_to_name(v_res), esp_err_to_name(c_res), esp_err_to_name(p_res));
            }
        }

        // Read every 2 seconds
        vTaskDelay(pdMS_TO_TICKS(500));
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
    float sht_t, sht_h, bmp_t, bmp_p, bmp_h, ads0_v, ads1_v, as_a;
    uint16_t as_r;
    float ina219_v[INA219_DEVICE_COUNT], ina219_c[INA219_DEVICE_COUNT], ina219_p[INA219_DEVICE_COUNT];

    while (1)
    {
        // --- Read Global Variables Safely ---
        if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // SHT4x
            sht_t = latest_sht_temp;
            sht_h = latest_sht_hum;
            // BMP280
            bmp_t = latest_bmp_temp;
            bmp_p = latest_bmp_press;
            bmp_h = latest_bmp_hum;
            // ADS1115
            ads0_v = latest_ads0_ch0_volt;
            ads1_v = latest_ads1_ch0_volt;
            // AS5600
            as_a = latest_as5600_angle;
            as_r = latest_as5600_raw;
            // Read INA219 values
            for (int i = 0; i < INA219_DEVICE_COUNT; i++)
            {
                ina219_v[i] = latest_ina219_voltage[i];
                ina219_c[i] = latest_ina219_current[i];
                ina219_p[i] = latest_ina219_power[i];
            }

            xSemaphoreGive(xDisplayDataMutex);
        }
        else
        {
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

        // Print sensor data (using snprintf for safety is good practice)
        snprintf(buffer, sizeof(buffer), "SHT4x: Temp=%.1f C, Hum=%.1f %%\n", sht_t, sht_h);
        printf("%s", buffer);

        snprintf(buffer, sizeof(buffer), "BMP280: Temp=%.1f C, Press=%.0f hPa, Hum=%.1f %%\n", bmp_t, bmp_p / 100, bmp_h); // Assuming BME & Pa -> hPa
        printf("%s", buffer);

        snprintf(buffer, sizeof(buffer), "ADS0 (PV): Ch0=%.3f V\n", ads0_v); // Example
        printf("%s", buffer);

        snprintf(buffer, sizeof(buffer), "ADS1 (Mst): Ch0=%.3f V\n", ads1_v); // Example
        printf("%s", buffer);

        snprintf(buffer, sizeof(buffer), "AS5600: Angle=%.1f deg, Raw=%u\n", as_a, as_r);
        printf("%s", buffer);

        // Add INA219 display
        for (int i = 0; i < INA219_DEVICE_COUNT; i++)
        {
            snprintf(buffer, sizeof(buffer), "INA219_%s: V=%.2f V, I=%.2f mA, P=%.2f mW\n", ina219_devices[i].name, ina219_v[i], ina219_c[i] * 1000, ina219_p[i] * 1000);
            printf("%s", buffer);
        }

        printf("-----------------------\n");
        // Flush stdout buffer to ensure data is sent immediately
        fflush(stdout);

        // --- End Format and Print ---

        vTaskDelay(pdMS_TO_TICKS(SERIAL_DISPLAY_INTERVAL_MS));
    }
}

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Starting ESP...");

    /* Display Mutex creation */
    xDisplayDataMutex = xSemaphoreCreateMutex();
    if (xDisplayDataMutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create display data mutex!");
        // Handle error appropriately, maybe halt or return an error code
    }
    else
    {
        ESP_LOGI(TAG, "Display data mutex created successfully.");
    }

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ", CONFIG_IDF_TARGET, chip_info.cores, (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
        (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "", (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK)
    {
        printf("Get flash size failed");
        return;
    }
    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024), (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // Initialize I2C subsystem
    ESP_LOGI(TAG, "Initializing I2C...");
    ESP_ERROR_CHECK(i2cdev_init());

    // Initialize ADS1115 Devices
    ESP_LOGI(TAG, "Initializing %d ADS1115 devices...", ADS1115_DEVICE_COUNT);
    gain_val = ads111x_gain_values[GAIN];
    // Note: Static arrays are automatically zero-initialized, memset not needed
    for (size_t i = 0; i < ADS1115_DEVICE_COUNT; i++)
    {
        ESP_LOGI(TAG, "  Initializing ADS1115 device %d (Addr 0x%02x)", i, ads1115_addr[i]);
        esp_err_t init_res = ads111x_init_desc(&ads1115_devices[i], ads1115_addr[i], I2C_PORT_NUM, CONFIG_I2CDEV_DEFAULT_SDA_PIN, CONFIG_I2CDEV_DEFAULT_SCL_PIN);
        ESP_LOGI(TAG, "  [Dev %d] ads111x_init_desc result: %s (%d)", i, esp_err_to_name(init_res), init_res);
        ESP_ERROR_CHECK(init_res);

        esp_err_t mode_res = ads111x_set_mode(&ads1115_devices[i], ADS111X_MODE_SINGLE_SHOT);
        ESP_LOGI(TAG, "  [Dev %d] ads111x_set_mode result: %s (%d)", i, esp_err_to_name(mode_res), mode_res);
        ESP_ERROR_CHECK(mode_res);

        esp_err_t rate_res = ads111x_set_data_rate(&ads1115_devices[i], ADS111X_DATA_RATE_32);
        ESP_LOGI(TAG, "  [Dev %d] ads111x_set_data_rate result: %s (%d)", i, esp_err_to_name(rate_res), rate_res);
        ESP_ERROR_CHECK(rate_res);

        esp_err_t gain_res = ads111x_set_gain(&ads1115_devices[i], GAIN);
        ESP_LOGI(TAG, "  [Dev %d] ads111x_set_gain result: %s (%d)", i, esp_err_to_name(gain_res), gain_res);
        ESP_ERROR_CHECK(gain_res);
    }
    ESP_LOGI(TAG, "ADS1115 initialization complete.");

    // Create tasks
    ESP_LOGI(TAG, "Creating tasks...");
    // Create the new ADS1115 tasks
    xTaskCreate(pv_tracking_task, "PVTrackingTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
    xTaskCreate(scheduled_moisture_task, "ScheduledMoistureTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

    xTaskCreate(SHT4xReadTask, "SHT4xReadTask", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(BME280ReadTask, "BME280ReadTask", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

    xTaskCreate(as5600_read_task, "AS5600ReadTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

    // Create INA219 read task
    xTaskCreate(ina219_read_task, "INA219ReadTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

    xTaskCreate(serial_display_task, "serial_display_task", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

    ESP_LOGI(TAG, "Tasks created. ESP Started.");
}
