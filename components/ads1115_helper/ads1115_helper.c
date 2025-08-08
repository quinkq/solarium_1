#include "ads1115_helper.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "i2cdev.h"
#include <string.h>

// ########################## Private Constants ##########################

static const char *TAG = "ADS1115_HELPER";

// ADS1115 I2C addresses
static const uint8_t ads1115_addresses[ADS1115_DEVICE_COUNT] = {
    ADS111X_ADDR_GND, // 0x48 - Photoresistor array (Dev#0)
    ADS111X_ADDR_VCC, // 0x49 - Moisture sensors (Dev#1)
    ADS111X_ADDR_SDA  // 0x4A - Zone5 moisture + pressure (Dev#2)
};

// ########################## Private Variables ##########################

// Device management array
ads1115_device_t ads1115_devices[ADS1115_DEVICE_COUNT] = {
    {.name = "Photoresistors", .initialized = false, .retry_count = 0, .next_retry_delay_ms = ADS1115_RETRY_DELAY_MAX_MS},
    {.name = "Moisture_Sensors", .initialized = false, .retry_count = 0, .next_retry_delay_ms = ADS1115_RETRY_DELAY_MAX_MS},
    {.name = "Mixed_Sensors", .initialized = false, .retry_count = 0, .next_retry_delay_ms = ADS1115_RETRY_DELAY_MAX_MS}
};

// Latest voltage readings for display and sharing
static float latest_voltages[ADS1115_DEVICE_COUNT][4] = {
    {-999.9, -999.9, -999.9, -999.9}, // Dev#0: Photoresistors
    {-999.9, -999.9, -999.9, -999.9}, // Dev#1: Moisture sensors
    {-999.9, -999.9, -999.9, -999.9}  // Dev#2: Zone5 moisture + pressure + spare
};

// Mutex for thread-safe access
static SemaphoreHandle_t xADS1115Mutex = NULL;

// Task handle for retry task
static TaskHandle_t xADS1115RetryTaskHandle = NULL;

// System initialization flag
static bool ads1115_initialized = false;

// ########################## Private Function Declarations ##########################

static esp_err_t ads1115_init_device(uint8_t device_id);
static void ads1115_retry_task(void *pvParameters);

// ########################## Private Function Implementations ##########################

/**
 * @brief Initialize a single ADS1115 device
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
                            ADS1115_I2C_PORT,
                            CONFIG_I2CDEV_DEFAULT_SDA_PIN,
                            CONFIG_I2CDEV_DEFAULT_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed ads111x_init_desc: %s", device_id, esp_err_to_name(ret));
        return ret;
    }

    // 2. Set single-shot mode
    ret = ads111x_set_mode(dev, ADS111X_MODE_SINGLE_SHOT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed ads111x_set_mode: %s", device_id, esp_err_to_name(ret));
        ads111x_free_desc(dev); // Cleanup on failure
        return ret;
    }

    // 3. Set data rate
    ret = ads111x_set_data_rate(dev, ADS111X_DATA_RATE_32);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed ads111x_set_data_rate: %s", device_id, esp_err_to_name(ret));
        ads111x_free_desc(dev); // Cleanup on failure
        return ret;
    }

    // 4. Set gain
    ret = ads111x_set_gain(dev, ADS1115_GAIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed ads111x_set_gain: %s", device_id, esp_err_to_name(ret));
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
        ESP_LOGE(TAG, "[Dev %d] Failed test read: %s", device_id, esp_err_to_name(ret));
        ads111x_free_desc(dev); // Cleanup on failure
        return ret;
    }

    ESP_LOGD(TAG, "[Dev %d] Test read successful (raw: %d)", device_id, test_raw);
    return ESP_OK;
}

/**
 * @brief Background task for retrying failed ADS1115 devices
 * 
 * Continuously monitors for failed devices and attempts to recover them
 * using exponential backoff strategy (1min → 3min → 10min max).
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

                    // Exponential backoff: 1min → 3min → 10min → 10min (max)
                    if (status->retry_count == 1) {
                        status->next_retry_delay_ms = 180000; // 3 minutes
                    } else if (status->retry_count >= 2) {
                        status->next_retry_delay_ms = MAX_RETRY_DELAY_MS; // 10 minutes
                    }

                    ESP_LOGW(TAG, "    Next retry in %lu minutes", status->next_retry_delay_ms / 60000);
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

// ########################## Public Function Implementations ##########################

esp_err_t ads1115_helper_init(void)
{
    if (ads1115_initialized) {
        ESP_LOGW(TAG, "ADS1115 helper already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing ADS1115 helper system...");

    // Create mutex
    xADS1115Mutex = xSemaphoreCreateMutex();
    if (xADS1115Mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create ADS1115 mutex");
        return ESP_ERR_NO_MEM;
    }

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
            ESP_LOGI(TAG, "ADS1115 #%d (%s) initialized successfully", device, ads1115_devices[device].name);
        } else {
            ESP_LOGW(TAG,
                     "ADS1115 #%d (%s) failed to initialize: %s",
                     device,
                     ads1115_devices[device].name,
                     esp_err_to_name(result));
            ESP_LOGW(TAG, "Will retry in %lu seconds", ads1115_devices[device].next_retry_delay_ms / 1000);
        }
    }

    ESP_LOGI(TAG, "ADS1115 initialization complete: %d/%d devices working", successful_devices, ADS1115_DEVICE_COUNT);

    // Start retry task regardless of initial success
    BaseType_t task_result = xTaskCreate(ads1115_retry_task, 
                                        "ADS1115_Retry", 
                                        configMINIMAL_STACK_SIZE * 3, 
                                        NULL, 
                                        5, 
                                        &xADS1115RetryTaskHandle);
    
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ADS1115 retry task");
        vSemaphoreDelete(xADS1115Mutex);
        return ESP_ERR_NO_MEM;
    }

    ads1115_initialized = true;

    if (successful_devices == 0) {
        ESP_LOGE(TAG, "No ADS1115 devices initialized! System functionality will be limited.");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ADS1115 helper system initialized successfully");
    return ESP_OK;
}

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

    // Take mutex for thread safety
    if (xSemaphoreTake(xADS1115Mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "[Dev %d] Failed to take mutex", device_id);
        return ESP_ERR_TIMEOUT;
    }

    // 1. Set MUX (channel selection)
    ret = ads111x_set_input_mux(dev, channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed to set MUX: %s (%d)", device_id, esp_err_to_name(ret), ret);
        // Mark device as failed for retry
        ads1115_devices[device_id].initialized = false;
        xSemaphoreGive(xADS1115Mutex);
        return ret;
    }

    // 2. Start Conversion
    ret = ads111x_start_conversion(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed to start conversion: %s (%d)", device_id, esp_err_to_name(ret), ret);
        ads1115_devices[device_id].initialized = false;
        xSemaphoreGive(xADS1115Mutex);
        return ret;
    }

    // Add a small delay for stability
    vTaskDelay(pdMS_TO_TICKS(1));

    // 3. Wait for conversion to complete (with timeout)
    int conversion_timeout_ms = 100; // Increased timeout
    int delay_ms = 5;                // Check every 5ms
    int elapsed_ms = 0;
    bool busy = true;

    do {
        ret = ads111x_is_busy(dev, &busy);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[Dev %d] Failed to check busy status: %s (%d)", device_id, esp_err_to_name(ret), ret);
            ads1115_devices[device_id].initialized = false;
            xSemaphoreGive(xADS1115Mutex);
            return ret;
        }

        vTaskDelay(1);

        if (busy) {
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
            elapsed_ms += delay_ms;
            if (elapsed_ms > conversion_timeout_ms) {
                ESP_LOGE(TAG, "[Dev %d] Conversion timeout after %d ms", device_id, conversion_timeout_ms);
                ads1115_devices[device_id].initialized = false;
                xSemaphoreGive(xADS1115Mutex);
                return ESP_ERR_TIMEOUT;
            }
        }
    } while (busy);

    // 4. Read Value
    ret = ads111x_get_value(dev, raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed to get value: %s (%d)", device_id, esp_err_to_name(ret), ret);
        ads1115_devices[device_id].initialized = false;
        xSemaphoreGive(xADS1115Mutex);
        return ret;
    }

    // 5. Calculate Voltage (if requested)
    float calculated_voltage = 0.0;
    if (voltage || (device_id < ADS1115_DEVICE_COUNT && channel < 4)) {
        calculated_voltage = (*raw / (float) ADS111X_MAX_VALUE) * ADS1115_GAIN;
        if (voltage) {
            *voltage = calculated_voltage;
        }
    }

    // Update latest voltages array if within bounds
    if (channel >= ADS111X_MUX_0_GND && channel <= ADS111X_MUX_3_GND) {
        uint8_t ch_idx = channel - ADS111X_MUX_0_GND;
        latest_voltages[device_id][ch_idx] = calculated_voltage;
    }

    xSemaphoreGive(xADS1115Mutex);
    return ESP_OK;
}

bool ads1115_is_device_ready(uint8_t device_id)
{
    if (device_id >= ADS1115_DEVICE_COUNT) {
        return false;
    }
    
    return ads1115_devices[device_id].initialized;
}

esp_err_t ads1115_get_latest_voltages(float voltages[ADS1115_DEVICE_COUNT][4])
{
    if (!voltages) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xADS1115Mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for voltage read");
        return ESP_ERR_TIMEOUT;
    }

    // Copy latest voltages
    for (int dev = 0; dev < ADS1115_DEVICE_COUNT; dev++) {
        for (int ch = 0; ch < 4; ch++) {
            voltages[dev][ch] = latest_voltages[dev][ch];
        }
    }

    xSemaphoreGive(xADS1115Mutex);
    return ESP_OK;
}

esp_err_t ads1115_get_device_info(uint8_t device_id, ads1115_device_t *device_info)
{
    if (device_id >= ADS1115_DEVICE_COUNT || !device_info) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xADS1115Mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for device info");
        return ESP_ERR_TIMEOUT;
    }

    // Copy device information (but not the i2c_dev_t structure itself)
    device_info->initialized = ads1115_devices[device_id].initialized;
    device_info->last_retry_time = ads1115_devices[device_id].last_retry_time;
    device_info->retry_count = ads1115_devices[device_id].retry_count;
    device_info->next_retry_delay_ms = ads1115_devices[device_id].next_retry_delay_ms;
    device_info->name = ads1115_devices[device_id].name;

    xSemaphoreGive(xADS1115Mutex);
    return ESP_OK;
}