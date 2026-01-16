/**
 * @file ads1115_helper.c
 * @brief Unified ADS1115 ADC interface with power-aware retry logic
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Unified multi-device ADC management with automatic fault recovery.
 *
 * Key features:
 * - Four-device management (moisture sensors, mixed sensors, photoresistors, hall array)
 * - Built-in read retry logic (3 attempts, 50ms delays) - consumers don't need to retry
 * - Two-tier device recovery strategy:
 *   - Fast retries: 5s, 10s, 20s (power/transient issues)
 *   - Slow retries: 1min, 5min, 10min (persistent hardware faults)
 * - Power-aware operation (FLUCTUS 3.3V bus integration)
 * - Notification-based recovery task (no polling overhead)
 * - Voltage caching and thread safety
 * - Health monitoring (needs_slow_retries flag for diagnostics)
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "ads1115_helper.h"
#include "fluctus.h"
#include "main.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <string.h>

// ########################## Private Constants ##########################

static const char *TAG = "ADS1115_HELPER";

// ADS1115 I2C addresses
static const uint8_t ads1115_addresses[ADS1115_DEVICE_COUNT] = {
    ADS111X_ADDR_GND, // 0x48 - Moisture sensors Zone 0 - 4 (Dev#0)
    ADS111X_ADDR_VCC, // 0x49 - Zone5 moisture + pressure + 3v3 Bus voltage (Dev#1)
    ADS111X_ADDR_SCL, // 0x4A - Photoresistor array (Dev#2)
    ADS111X_ADDR_SDA  // 0x4B - Hall sensor array (Dev#3)
};

// ########################## Private Variables ##########################

// Device management array
ads1115_device_t ads1115_devices[ADS1115_DEVICE_COUNT] = {
    {
        .name = "Moisture_Sensors",
        .initialized = false,
        .retry_count = 0,
        .next_retry_delay_ms = ADS1115_INIT_FAST_RETRY_DELAY_MS,
        .needs_slow_retries = false,
        .mode = ADS111X_MODE_SINGLE_SHOT,
        .data_rate = ADS111X_DATA_RATE_16,
        .gain = ADS111X_GAIN_4V096
    },
    {
        .name = "Mixed_Sensors",
        .initialized = false,
        .retry_count = 0,
        .next_retry_delay_ms = ADS1115_INIT_FAST_RETRY_DELAY_MS,
        .needs_slow_retries = false,
        .mode = ADS111X_MODE_SINGLE_SHOT,
        .data_rate = ADS111X_DATA_RATE_16,
        .gain = ADS111X_GAIN_4V096
    },
    {
        .name = "Photoresistors",
        .initialized = false,
        .retry_count = 0,
        .next_retry_delay_ms = ADS1115_INIT_FAST_RETRY_DELAY_MS,
        .needs_slow_retries = false,
        .mode = ADS111X_MODE_SINGLE_SHOT,
        .data_rate = ADS111X_DATA_RATE_128,
        .gain = ADS111X_GAIN_4V096
    },
    {
        .name = "Hall_array",
        .initialized = false,
        .retry_count = 0,
        .next_retry_delay_ms = ADS1115_INIT_FAST_RETRY_DELAY_MS,
        .needs_slow_retries = false,
        .mode = ADS111X_MODE_SINGLE_SHOT,
        .data_rate = ADS111X_DATA_RATE_128,
        .gain = ADS111X_GAIN_4V096
    }
};

// Mutex for thread-safe access
static SemaphoreHandle_t xADS1115Mutex = NULL;

// Task handle for retry task
static TaskHandle_t xADS1115RetryTaskHandle = NULL;

// System initialization flag
static bool ads1115_helper_initialized = false;

// ########################## Private Function Declarations ##########################

static esp_err_t ads1115_helper_init_device(uint8_t device_id);
static void ads1115_helper_retry_task(void *pvParameters);

// ########################## Private Function Implementations ##########################

/**
 * @brief Initialize a single ADS1115 device
 * 
 * @param device_id Device index (0 to ADS1115_DEVICE_COUNT-1)
 * @return ESP_OK on successful initialization
 * @return ESP_ERR_INVALID_ARG if device_id is invalid
 * @return ESP_ERR_* on communication or configuration errors
 */
static esp_err_t ads1115_helper_init_device(uint8_t device_id)
{
    if (device_id >= ADS1115_DEVICE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    // Request 3.3V bus power for I2C communication (reference counting handles overlaps)
    esp_err_t power_ret = fluctus_request_bus_power(POWER_BUS_3V3, "ADS1115_INI_DEV");
    if (power_ret != ESP_OK) {
        ESP_LOGD(TAG, "[Dev %d] 3.3V bus unavailable: %s", device_id, esp_err_to_name(power_ret));
        return ESP_ERR_INVALID_STATE;
    }
    vTaskDelay(pdMS_TO_TICKS(50));  // Power-up stabilization delay (increased from 10ms)

    i2c_dev_t *dev = &ads1115_devices[device_id].device;
    esp_err_t ret;

    // 1. Initialize descriptor (Bus B - toggled power)
    ret = ads111x_init_desc(dev,
                            ads1115_addresses[device_id],
                            I2C_BUS_B_PORT,
                            I2C_BUS_B_SDA_PIN,
                            I2C_BUS_B_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed ads111x_init_desc: %s", device_id, esp_err_to_name(ret));
        fluctus_release_bus_power(POWER_BUS_3V3, "ADS1115_INI_DEV");
        return ret;
    }

    // 2. Set operating mode (use device-specific configuration)
    ret = ads111x_set_mode(dev, ads1115_devices[device_id].mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed ads111x_set_mode: %s", device_id, esp_err_to_name(ret));
        ads111x_free_desc(dev); // Cleanup on failure
        fluctus_release_bus_power(POWER_BUS_3V3, "ADS1115_INI_DEV");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Allow any inadvertent conversion to clear

    // 3. Set data rate (use device-specific configuration)
    ret = ads111x_set_data_rate(dev, ads1115_devices[device_id].data_rate);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed ads111x_set_data_rate: %s", device_id, esp_err_to_name(ret));
        ads111x_free_desc(dev); // Cleanup on failure
        fluctus_release_bus_power(POWER_BUS_3V3, "ADS1115_INI_DEV");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Allow any inadvertent conversion to clear

    // 4. Set gain (use device-specific configuration)
    ret = ads111x_set_gain(dev, ads1115_devices[device_id].gain);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed ads111x_set_gain: %s", device_id, esp_err_to_name(ret));
        ads111x_free_desc(dev); // Cleanup on failure
        fluctus_release_bus_power(POWER_BUS_3V3, "ADS1115_INI_DEV");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Allow any inadvertent conversion to clear

    // 5. Test read to verify device is responsive
    // For 16 SPS devices: 62.5ms conversion time + margin
    // For 128 SPS devices: 7.8ms conversion time + margin
    uint32_t conversion_delay_ms = (ads1115_devices[device_id].data_rate == ADS111X_DATA_RATE_16) ? 80 : 20;

    int16_t test_raw;
    ret = ads111x_set_input_mux(dev, ADS111X_MUX_0_GND);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));  // Allow mux change to settle
        ret = ads111x_start_conversion(dev);
        if (ret == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(conversion_delay_ms));  // Wait for conversion based on data rate
            ret = ads111x_get_value(dev, &test_raw);
        }
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Dev %d] Failed test read: %s", device_id, esp_err_to_name(ret));
        ads111x_free_desc(dev); // Cleanup on failure
        fluctus_release_bus_power(POWER_BUS_3V3, "ADS1115_INI_DEV");
        return ret;
    }

    ESP_LOGD(TAG, "[Dev %d] Test read successful (raw: %d)", device_id, test_raw);

    // Release power - init complete
    fluctus_release_bus_power(POWER_BUS_3V3, "ADS1115_INI_DEV");
    return ESP_OK;
}

/**
 * @brief Helper function to attempt re-initialization of all failed devices
 *
 * @return true if any devices remain failed, false if all devices are working
 */
static bool retry_all_failed_devices(void)
{
    // Check if 3.3V bus is available before attempting (power-aware)
    esp_err_t power_check = fluctus_request_bus_power(POWER_BUS_3V3, "ADS1115_RETRY");
    if (power_check != ESP_OK) {
        ESP_LOGD(TAG, "3.3V bus unavailable for device retry, deferring");
        return true;  // Still have failed devices, but can't retry now
    }
    fluctus_release_bus_power(POWER_BUS_3V3, "ADS1115_RETRY");  // Release immediately, init_device will request again

    bool any_failed = false;
    uint8_t working_count = 0;

    for (uint8_t device = 0; device < ADS1115_DEVICE_COUNT; device++) {
        ads1115_device_t *status = &ads1115_devices[device];

        if (status->initialized) {
            working_count++;
            continue;
        }

        // Attempt re-initialization
        ESP_LOGI(TAG, "Retrying ADS1115 device %d (%s)", device, status->name);
        esp_err_t result = ads1115_helper_init_device(device);

        if (result == ESP_OK) {
            status->initialized = true;
            status->retry_count = 0;
            status->next_retry_delay_ms = ADS1115_INIT_FAST_RETRY_DELAY_MS;
            working_count++;
            ESP_LOGI(TAG, "ADS1115 device %d (%s) successfully recovered!", device, status->name);
        } else {
            any_failed = true;
            ESP_LOGW(TAG, "ADS1115 device %d (%s) retry failed: %s", device, status->name, esp_err_to_name(result));
        }
    }

    ESP_LOGI(TAG, "ADS1115 retry complete: %d/%d devices working", working_count, ADS1115_DEVICE_COUNT);
    return any_failed;
}

/**
 * @brief Background task for retrying failed ADS1115 devices
 *
 * Notification-based task that wakes when devices fail, then uses two-tier retry strategy:
 * - Fast retries: 3 attempts with exponential backoff (5s, 10s, 20s) for power/transient issues
 * - Slow retries: 3 attempts with long delays (1min, 5min, 10min) for persistent hardware issues
 * - Continuous monitoring: Every 10 minutes until all devices recovered
 *
 * Sets needs_slow_retries flag on devices that require slow retry phase.
 *
 * @param pvParameters Unused task parameter
 */
static void ads1115_helper_retry_task(void *pvParameters)
{
    ESP_LOGI(TAG, "ADS1115 retry task started (notification-based)");
    uint32_t notification_value = 0;

    while (true) {
        // Wait for notification (blocks indefinitely)
        notification_value = 0;
        xTaskNotifyWait(0x00, ULONG_MAX, &notification_value, portMAX_DELAY);

        ESP_LOGI(TAG, "ADS1115 retry task triggered (notification received)");

        // === PHASE 1: Fast Retries (for power-related and transient issues) ===
        bool any_failed = true;
        for (uint8_t quick_retry = 0; quick_retry < ADS1115_INIT_FAST_RETRY_COUNT && any_failed; quick_retry++) {
            if (quick_retry > 0) {
                // Exponential backoff: 5s, 10s, 20s
                uint32_t delay_ms = ADS1115_INIT_FAST_RETRY_DELAY_MS << (quick_retry - 1);
                ESP_LOGI(TAG, "Fast retry %d/%d in %lu seconds...", quick_retry + 1, ADS1115_INIT_FAST_RETRY_COUNT, delay_ms / 1000);
                vTaskDelay(pdMS_TO_TICKS(delay_ms));
            }

            any_failed = retry_all_failed_devices();

            if (!any_failed) {
                ESP_LOGI(TAG, "All devices recovered during fast retry phase!");
                break;  // All recovered, suspend task
            }
        }

        if (!any_failed) {
            continue;  // Wait for next notification
        }

        // === PHASE 2: Slow Retries (for persistent hardware issues) ===
        ESP_LOGW(TAG, "Fast retries exhausted, entering slow retry phase (potential hardware issue)");

        // Mark devices that need slow retries (diagnostic flag)
        for (uint8_t device = 0; device < ADS1115_DEVICE_COUNT; device++) {
            if (!ads1115_devices[device].initialized) {
                ads1115_devices[device].needs_slow_retries = true;
                ESP_LOGW(TAG, "Device %d (%s) flagged as potentially faulty (requires slow retries)",
                         device, ads1115_devices[device].name);
            }
        }

        uint32_t slow_delays[] = {
            ADS1115_INIT_SLOW_RETRY_DELAY_1_MS,  // 1 minute
            ADS1115_INIT_SLOW_RETRY_DELAY_2_MS,  // 5 minutes
            ADS1115_INIT_SLOW_RETRY_DELAY_3_MS   // 10 minutes
        };

        for (int i = 0; i < 3 && any_failed; i++) {
            ESP_LOGI(TAG, "Slow retry %d/3 in %lu minutes...", i + 1, slow_delays[i] / 60000);
            vTaskDelay(pdMS_TO_TICKS(slow_delays[i]));

            any_failed = retry_all_failed_devices();

            if (!any_failed) {
                ESP_LOGI(TAG, "All devices recovered during slow retry phase");
                break;
            }
        }

        if (!any_failed) {
            continue;  // Wait for next notification
        }

        // === PHASE 3: Continuous Monitoring (every 10 minutes until recovery) ===
        ESP_LOGW(TAG, "Entering continuous monitoring mode (10-minute intervals)");
        while (any_failed) {
            vTaskDelay(pdMS_TO_TICKS(ADS1115_INIT_SLOW_RETRY_DELAY_3_MS));  // 10 minutes
            any_failed = retry_all_failed_devices();
        }

        ESP_LOGI(TAG, "All devices recovered! Retry task going back to sleep");
    }
}

// ########################## Public Function Implementations ##########################

esp_err_t ads1115_helper_init(void)
{
    if (ads1115_helper_initialized) {
        ESP_LOGW(TAG, "ADS1115 helper already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing ADS1115 helper system...");

    // Request 3.3V bus power for initialization (requires fluctus_init already called)
    esp_err_t ret = fluctus_request_bus_power(POWER_BUS_3V3, "ADS1115_INIT");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to request 3.3V bus power: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Allow bus to stabilize

    // Create mutex
    xADS1115Mutex = xSemaphoreCreateMutex();
    if (xADS1115Mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create ADS1115 mutex");
        fluctus_release_bus_power(POWER_BUS_3V3, "ADS1115_INIT");
        return ESP_ERR_NO_MEM;
    }

    // Initialize status tracking
    for (uint8_t device = 0; device < ADS1115_DEVICE_COUNT; device++) {
        ads1115_devices[device].initialized = false;
        ads1115_devices[device].last_retry_time = 0;
        ads1115_devices[device].retry_count = 0;
        ads1115_devices[device].next_retry_delay_ms = ADS1115_INIT_FAST_RETRY_DELAY_MS;
        ads1115_devices[device].needs_slow_retries = false;
    }

    uint8_t successful_devices = 0;

    // Try to initialize each device
    for (uint8_t device = 0; device < ADS1115_DEVICE_COUNT; device++) {
        ESP_LOGI(TAG,
                 "Initializing ADS1115 device %d (%s) at 0x%02x",
                 device,
                 ads1115_devices[device].name,
                 ads1115_addresses[device]);

        esp_err_t result = ads1115_helper_init_device(device);
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
    BaseType_t task_result = xTaskCreate(ads1115_helper_retry_task, 
                                        "ADS1115_Retry", 
                                        configMINIMAL_STACK_SIZE * 3, 
                                        NULL, 
                                        5, 
                                        &xADS1115RetryTaskHandle);
    
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ADS1115 retry task");
        vSemaphoreDelete(xADS1115Mutex);
        fluctus_release_bus_power(POWER_BUS_3V3, "ADS1115_INIT");
        return ESP_ERR_NO_MEM;
    }

    ads1115_helper_initialized = true;

    // Release init power - runtime operations request power as needed
    fluctus_release_bus_power(POWER_BUS_3V3, "ADS1115_INIT");
    ESP_LOGI(TAG, "Released 3.3V bus power (init complete)");

    // Trigger retry task if any devices failed during initialization
    if (successful_devices < ADS1115_DEVICE_COUNT) {
        ESP_LOGI(TAG, "Triggering retry task for %d failed device(s)", ADS1115_DEVICE_COUNT - successful_devices);
        xTaskNotify(xADS1115RetryTaskHandle, 1, eSetBits);
    }

    if (successful_devices == 0) {
        ESP_LOGE(TAG, "No ADS1115 devices initialized! System functionality will be limited.");
        ESP_LOGI(TAG, "Retry task will attempt recovery");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ADS1115 helper system initialized successfully");
    return ESP_OK;
}

esp_err_t ads1115_helper_read_channel(uint8_t device_id, ads111x_mux_t channel, int16_t *raw, float *voltage)
{
    if (!ads1115_helper_initialized) {
    ESP_LOGD(TAG, "[Dev %d] System init in progress, skipping read", device_id);
    return ESP_ERR_INVALID_STATE;  // Same return as "device not initialized"
    }

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

    // Request 3.3V bus power for I2C communication (reference counting handles overlaps)
    esp_err_t power_ret = fluctus_request_bus_power(POWER_BUS_3V3, "ADS1115_CH_READ");
    if (power_ret != ESP_OK) {
        ESP_LOGW(TAG, "[Dev %d] 3.3V bus unavailable: %s", device_id, esp_err_to_name(power_ret));
        return ESP_ERR_INVALID_STATE;
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Brief stabilization delay

    i2c_dev_t *dev = &ads1115_devices[device_id].device;
    esp_err_t ret = ESP_FAIL;

    // Retry loop: 3 attempts with 50ms delays
    for (uint8_t attempt = 0; attempt < ADS1115_READ_RETRY_ATTEMPTS; attempt++) {
        // Take mutex for thread safety
        if (xSemaphoreTake(xADS1115Mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "[Dev %d] Attempt %d: Failed to take mutex", device_id, attempt + 1);
            ret = ESP_ERR_TIMEOUT;
            goto retry_delay;
        }

        // 1. Reconfigure device settings (required after power cycle of 3.3V bus)
        // ADS1115 resets to defaults (±2.048V gain, 128 SPS) when power is lost
        ret = ads111x_set_gain(dev, ads1115_devices[device_id].gain);
        if (ret != ESP_OK) {
            ESP_LOGD(TAG, "[Dev %d] Attempt %d: Failed to set gain: %s", device_id, attempt + 1, esp_err_to_name(ret));
            xSemaphoreGive(xADS1115Mutex);
            goto retry_delay;
        }

        ret = ads111x_set_data_rate(dev, ads1115_devices[device_id].data_rate);
        if (ret != ESP_OK) {
            ESP_LOGD(TAG, "[Dev %d] Attempt %d: Failed to set data rate: %s", device_id, attempt + 1, esp_err_to_name(ret));
            xSemaphoreGive(xADS1115Mutex);
            goto retry_delay;
        }

        // 2. Set MUX (channel selection)
        ret = ads111x_set_input_mux(dev, channel);
        if (ret != ESP_OK) {
            ESP_LOGD(TAG, "[Dev %d] Attempt %d: Failed to set MUX: %s", device_id, attempt + 1, esp_err_to_name(ret));
            xSemaphoreGive(xADS1115Mutex);
            goto retry_delay;
        }

        // MUX settling time: Allow input capacitor to charge after channel switch
        // 10ms is sufficient for RC settling (1kΩ * pF input cap = sub-ms time constant)
        vTaskDelay(pdMS_TO_TICKS(10));

        // 3. Start Conversion
        ret = ads111x_start_conversion(dev);
        if (ret != ESP_OK) {
            ESP_LOGD(TAG, "[Dev %d] Attempt %d: Failed to start conversion: %s", device_id, attempt + 1, esp_err_to_name(ret));
            xSemaphoreGive(xADS1115Mutex);
            goto retry_delay;
        }

        // 4. Wait for conversion to complete using fixed delay
        // Note: OS bit polling doesn't work reliably on some ADS1115 modules, so we use
        // fixed delays based on the data rate. This is more deterministic and avoids
        // I2C bus contention from repeated config register polling.
        // Conversion times: 16 SPS = 62.5ms, 128 SPS = 7.8ms
        // Add ~12% margin for oscillator tolerance
        uint32_t conversion_delay_ms = (ads1115_devices[device_id].data_rate == ADS111X_DATA_RATE_16) ? 70 : 15;
        vTaskDelay(pdMS_TO_TICKS(conversion_delay_ms));

        // 5. Read Value
        ret = ads111x_get_value(dev, raw);
        if (ret != ESP_OK) {
            ESP_LOGD(TAG, "[Dev %d] Attempt %d: Failed to get value: %s", device_id, attempt + 1, esp_err_to_name(ret));
            xSemaphoreGive(xADS1115Mutex);
            goto retry_delay;
        }

        // 6. Calculate Voltage (if requested)
        float calculated_voltage = 0.0;
        if (voltage || (device_id < ADS1115_DEVICE_COUNT && channel < 4)) {
            // Use per-device gain setting for voltage calculation
            // ads111x_gain_values[] converts enum (0-7) to actual voltage (6.144V - 0.256V)
            float gain_voltage = ads111x_gain_values[ads1115_devices[device_id].gain];
            calculated_voltage = (*raw / (float) ADS111X_MAX_VALUE) * gain_voltage;
            if (voltage) {
                *voltage = calculated_voltage;
            }
        }

        // Success!
        xSemaphoreGive(xADS1115Mutex);
        fluctus_release_bus_power(POWER_BUS_3V3, "ADS1115_CH_READ");

        if (attempt > 0) {
            ESP_LOGD(TAG, "[Dev %d] Read succeeded on attempt %d", device_id, attempt + 1);
        }
        return ESP_OK;

retry_delay:
        // Delay before next attempt (except on last attempt)
        if (attempt < ADS1115_READ_RETRY_ATTEMPTS - 1) {
            vTaskDelay(pdMS_TO_TICKS(ADS1115_READ_RETRY_DELAY_MS));
        }
    }

    // All retries exhausted - mark device as failed and notify retry task
    ESP_LOGW(TAG, "[Dev %d] All %d read attempts failed: %s", device_id, ADS1115_READ_RETRY_ATTEMPTS, esp_err_to_name(ret));
    ads1115_devices[device_id].initialized = false;
    xTaskNotify(xADS1115RetryTaskHandle, 1, eSetBits);

    fluctus_release_bus_power(POWER_BUS_3V3, "ADS1115_CH_READ");
    return ret;
}

bool ads1115_helper_is_device_ready(uint8_t device_id)
{
    if (device_id >= ADS1115_DEVICE_COUNT) {
        return false;
    }
    
    return ads1115_devices[device_id].initialized;
}

esp_err_t ads1115_helper_get_device_info(uint8_t device_id, ads1115_device_t *device_info)
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