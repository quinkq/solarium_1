#include <math.h>
#include <string.h>
#include <inttypes.h> // For PRIu32
#include "as5600.h"
#include "esp_log.h"
#include "esp_idf_lib_helpers.h"

#define I2C_FREQ_HZ 400000 // Set to 400 KHz to match other devices

#define AS5600_ANGLE_REGISTER_H 0x0E // Raw Angle (11:0) H byte
#define AS5600_ANGLE_REGISTER_L 0x0F // Raw Angle (11:0) L byte
#define AS5600_PULSES_PER_REVOLUTION 4096.0f
#define AS5600_READING_MASK 0x0FFF         // 12 bits
#define AS5600_UPDATE_TASK_STACK_SIZE 4096 // Increased stack size
#define AS5600_UPDATE_TASK_PRIORITY 5
#define AS5600_UPDATE_INTERVAL_MS 20 // How often to read (Increased to ensure > 0 ticks)
#define AS5600_MUTEX_TIMEOUT_MS 100

static const char *TAG = "AS5600";

// Threshold to detect wrap-around (slightly less than full range)
static const float encoder_wrap_threshold = AS5600_PULSES_PER_REVOLUTION * 0.90f;
static const float counts_to_deg = 360.0f / AS5600_PULSES_PER_REVOLUTION;
static const float counts_to_rad = (2.0f * M_PI) / AS5600_PULSES_PER_REVOLUTION;

#define CHECK(x)                \
    do                          \
    {                           \
        esp_err_t __;           \
        if ((__ = x) != ESP_OK) \
            return __;          \
    } while (0)
#define CHECK_ARG(VAL)                  \
    do                                  \
    {                                   \
        if (!(VAL))                     \
            return ESP_ERR_INVALID_ARG; \
    } while (0)
#define CHECK_LOGE(dev, x, msg, ...)           \
    do                                         \
    {                                          \
        esp_err_t __;                          \
        if ((__ = x) != ESP_OK)                \
        {                                      \
            I2C_DEV_GIVE_MUTEX(&dev->i2c_dev); \
            ESP_LOGE(TAG, msg, ##__VA_ARGS__); \
            return __;                         \
        }                                      \
    } while (0)

// Forward declarations of internal functions
static void as5600_update_task(void *pvParameters);
static esp_err_t _as5600_read_raw_internal(as5600_dev_t *dev, uint16_t *raw_angle);

/* Public API Functions */

esp_err_t as5600_init_desc(as5600_dev_t *dev, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr != AS5600_DEFAULT_ADDRESS)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
        return ESP_ERR_INVALID_ARG;
    }

    if (port >= I2C_NUM_MAX)
    {
        ESP_LOGE(TAG, "Invalid I2C port: %d", port);
        return ESP_ERR_INVALID_ARG;
    }

    // Zero-initialize the structure
    memset(dev, 0, sizeof(as5600_dev_t));
    dev->initialized = false;

    // Initialize I2C device descriptor
    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.addr_bit_len = I2C_ADDR_BIT_LEN_7;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
    dev->i2c_dev.cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    dev->i2c_dev.cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;

    ESP_LOGD(TAG, "Descriptor initialized. Port: %d, Addr: 0x%02X, SDA: %d, SCL: %d",
             port, addr, sda_gpio, scl_gpio);

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t as5600_free_desc(as5600_dev_t *dev)
{
    CHECK_ARG(dev);

    // If device was fully initialized, need to stop the task first
    if (dev->initialized && dev->update_task_handle)
    {
        vTaskDelete(dev->update_task_handle);
        dev->update_task_handle = NULL;
        ESP_LOGD(TAG, "[0x%02x] Update task deleted", dev->i2c_dev.addr);
    }

    // Delete the data mutex if it exists
    if (dev->data_mutex)
    {
        vSemaphoreDelete(dev->data_mutex);
        dev->data_mutex = NULL;
        ESP_LOGD(TAG, "[0x%02x] Data mutex deleted", dev->i2c_dev.addr);
    }

    dev->initialized = false;
    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t as5600_init(as5600_dev_t *dev)
{
    CHECK_ARG(dev);

    ESP_LOGI(TAG, "[0x%02x] Initializing AS5600 sensor", dev->i2c_dev.addr);

    // First, ensure the device has a valid descriptor - this might be failing
    if (dev->i2c_dev.addr != AS5600_DEFAULT_ADDRESS)
    {
        ESP_LOGE(TAG, "Invalid device address! Call as5600_init_desc first with correct parameters");
        return ESP_ERR_INVALID_STATE;
    }

    // Create mutex for protecting shared data (accumulated, previous)
    dev->data_mutex = xSemaphoreCreateMutex();
    if (!dev->data_mutex)
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to create data mutex", dev->i2c_dev.addr);
        return ESP_ERR_NO_MEM;
    }

    // Initialize accumulated_counts and zero_offset
    dev->accumulated_counts = 0.0f;
    dev->previous_raw_counts = 0.0f;
    dev->zero_offset = 0;
    dev->initialized = false; // Will be set to true only after successful initialization

    // First try simple I2C read to see if device exists
    esp_err_t ret = ESP_OK;
    if ((ret = i2c_dev_take_mutex(&dev->i2c_dev)) != ESP_OK)
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to take mutex during init: %s",
                 dev->i2c_dev.addr, esp_err_to_name(ret));
        vSemaphoreDelete(dev->data_mutex);
        dev->data_mutex = NULL;
        return ret;
    }

    // Perform an initial read to verify communication
    uint16_t temp_raw;
    ret = _as5600_read_raw_internal(dev, &temp_raw);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "[0x%02x] Failed initial communication check with sensor: %s",
                 dev->i2c_dev.addr, esp_err_to_name(ret));
        i2c_dev_give_mutex(&dev->i2c_dev);
        vSemaphoreDelete(dev->data_mutex);
        dev->data_mutex = NULL;
        return ret;
    }

    i2c_dev_give_mutex(&dev->i2c_dev);

    ESP_LOGI(TAG, "[0x%02x] Initial communication successful. Raw reading: %u", dev->i2c_dev.addr, temp_raw);

    // Store the initial reading
    if (xSemaphoreTake(dev->data_mutex, pdMS_TO_TICKS(AS5600_MUTEX_TIMEOUT_MS)) == pdTRUE)
    {
        dev->previous_raw_counts = (float)temp_raw;
        xSemaphoreGive(dev->data_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to take data mutex during init", dev->i2c_dev.addr);
        vSemaphoreDelete(dev->data_mutex);
        dev->data_mutex = NULL;
        return ESP_ERR_TIMEOUT;
    }

    // Create the background update task
    BaseType_t task_created = xTaskCreate(as5600_update_task,
                                          "as5600_update",
                                          AS5600_UPDATE_TASK_STACK_SIZE,
                                          (void *)dev,
                                          AS5600_UPDATE_TASK_PRIORITY,
                                          &dev->update_task_handle);

    if (task_created != pdPASS)
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to create update task", dev->i2c_dev.addr);
        vSemaphoreDelete(dev->data_mutex);
        dev->data_mutex = NULL;
        return ESP_FAIL;
    }

    // Now explicitly set initialized
    dev->initialized = true;
    ESP_LOGI(TAG, "[0x%02x] AS5600 initialized successfully (initialized flag set to true)", dev->i2c_dev.addr);

    return ESP_OK;
}

esp_err_t as5600_read_raw_counts(as5600_dev_t *dev, uint16_t *counts)
{
    CHECK_ARG(dev && counts);

    // Set default error value
    *counts = 0;

    if (!dev)
    {
        ESP_LOGE(TAG, "Device pointer is NULL!");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized)
    {
        ESP_LOGE(TAG, "[0x%02x] Device not initialized, flag is false", dev->i2c_dev.addr);
        return ESP_ERR_INVALID_STATE;
    }

    if (!dev->data_mutex)
    {
        ESP_LOGE(TAG, "[0x%02x] Data mutex is NULL!", dev->i2c_dev.addr);
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(dev->data_mutex, pdMS_TO_TICKS(AS5600_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to take mutex for reading raw counts", dev->i2c_dev.addr);
        return ESP_ERR_TIMEOUT;
    }

    *counts = (uint16_t)dev->previous_raw_counts;
    xSemaphoreGive(dev->data_mutex);

    return ESP_OK;
}

esp_err_t as5600_read_relative_counts(as5600_dev_t *dev, uint16_t *counts)
{
    CHECK_ARG(dev && counts);

    if (!dev->initialized)
        return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(dev->data_mutex, pdMS_TO_TICKS(AS5600_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to take mutex for reading relative counts", dev->i2c_dev.addr);
        return ESP_ERR_TIMEOUT;
    }

    // Calculate relative counts: (current_raw - offset + full_range) % full_range
    int32_t relative = (int32_t)dev->previous_raw_counts - (int32_t)dev->zero_offset;
    relative = (relative + (int32_t)AS5600_PULSES_PER_REVOLUTION) % (int32_t)AS5600_PULSES_PER_REVOLUTION;
    *counts = (uint16_t)relative;

    xSemaphoreGive(dev->data_mutex);

    return ESP_OK;
}

esp_err_t as5600_read_accumulated_counts(as5600_dev_t *dev, float *counts)
{
    CHECK_ARG(dev && counts);

    if (!dev->initialized)
        return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(dev->data_mutex, pdMS_TO_TICKS(AS5600_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to take mutex for reading accumulated counts", dev->i2c_dev.addr);
        return ESP_ERR_TIMEOUT;
    }

    // The accumulated value already includes the base for the current wrap cycle.
    // We just need to add the current position within this cycle.
    *counts = dev->accumulated_counts + dev->previous_raw_counts;

    xSemaphoreGive(dev->data_mutex);

    return ESP_OK;
}

esp_err_t as5600_read_angle_degrees(as5600_dev_t *dev, float *angle)
{
    CHECK_ARG(dev && angle);

    // Default error value
    *angle = -999.9f;

    if (!dev->initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t relative_counts;
    esp_err_t ret = as5600_read_relative_counts(dev, &relative_counts);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read - Angle: %d, Raw: %d, Acc: %d",
                 ret, ret, ret);
        return ret;
    }

    *angle = (float)relative_counts * counts_to_deg;

    return ESP_OK;
}

esp_err_t as5600_read_angle_radians(as5600_dev_t *dev, float *angle)
{
    CHECK_ARG(dev && angle);

    // Default error value
    *angle = -999.9f;

    if (!dev->initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t relative_counts;
    esp_err_t ret = as5600_read_relative_counts(dev, &relative_counts);
    if (ret != ESP_OK)
    {
        return ret;
    }

    *angle = (float)relative_counts * counts_to_rad;

    return ESP_OK;
}

esp_err_t as5600_set_zero_offset(as5600_dev_t *dev)
{
    CHECK_ARG(dev);

    if (!dev->initialized)
        return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(dev->data_mutex, pdMS_TO_TICKS(AS5600_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to take mutex for setting zero offset", dev->i2c_dev.addr);
        return ESP_ERR_TIMEOUT;
    }

    dev->zero_offset = (uint16_t)dev->previous_raw_counts;
    ESP_LOGI(TAG, "[0x%02x] Zero offset set to %u counts", dev->i2c_dev.addr, dev->zero_offset);

    xSemaphoreGive(dev->data_mutex);

    return ESP_OK;
}

float as5600_get_counts_per_revolution(as5600_dev_t *dev)
{
    (void)dev; // Parameter not needed for this constant value
    return AS5600_PULSES_PER_REVOLUTION;
}

/* Internal Functions */

// Internal function to read the raw 12-bit angle
static esp_err_t _as5600_read_raw_internal(as5600_dev_t *dev, uint16_t *raw_angle)
{
    if (!dev || !raw_angle)
        return ESP_ERR_INVALID_ARG;

    uint8_t read_buffer[2];
    esp_err_t ret;

    // Read the two bytes of the angle register
    ret = i2c_dev_read_reg(&dev->i2c_dev, AS5600_ANGLE_REGISTER_H, read_buffer, 2);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to read angle register: %d (%s)",
                 dev->i2c_dev.addr, ret, esp_err_to_name(ret));
        return ret;
    }

    // Combine the two bytes into a 16-bit value
    *raw_angle = ((uint16_t)read_buffer[0] << 8) | read_buffer[1];
    *raw_angle &= AS5600_READING_MASK; // Apply 12-bit mask

    return ESP_OK;
}

// Background task to continuously read the sensor and update accumulated counts
static void as5600_update_task(void *pvParameters)
{
    as5600_dev_t *dev = (as5600_dev_t *)pvParameters;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t update_interval_ticks = pdMS_TO_TICKS(AS5600_UPDATE_INTERVAL_MS);

    ESP_LOGI(TAG, "[0x%02x] Update task started", dev->i2c_dev.addr);

    // Main update loop
    uint8_t fail_count = 0;

    while (1)
    {
        uint16_t current_raw;
        bool read_success = false;

        if (i2c_dev_take_mutex(&dev->i2c_dev) == ESP_OK)
        {
            esp_err_t read_ret = _as5600_read_raw_internal(dev, &current_raw);
            i2c_dev_give_mutex(&dev->i2c_dev);

            if (read_ret == ESP_OK)
            {
                read_success = true;
                fail_count = 0; // Reset fail counter on success

                if (xSemaphoreTake(dev->data_mutex, pdMS_TO_TICKS(AS5600_MUTEX_TIMEOUT_MS)) == pdTRUE)
                {
                    float current_f = (float)current_raw;
                    float delta = current_f - dev->previous_raw_counts;

                    // Check for wrap-around
                    if (fabsf(delta) >= encoder_wrap_threshold)
                    {
                        // Determine direction of wrap
                        if (delta < 0.0f)
                        { // Wrapped from low to high (e.g., 10 -> 4090)
                            dev->accumulated_counts += AS5600_PULSES_PER_REVOLUTION;
                            ESP_LOGD(TAG, "[0x%02x] Wrap detected (+), delta: %.1f, prev: %.1f, curr: %.1f",
                                     dev->i2c_dev.addr, delta, dev->previous_raw_counts, current_f);
                        }
                        else
                        { // Wrapped from high to low (e.g., 4090 -> 10)
                            dev->accumulated_counts -= AS5600_PULSES_PER_REVOLUTION;
                            ESP_LOGD(TAG, "[0x%02x] Wrap detected (-), delta: %.1f, prev: %.1f, curr: %.1f",
                                     dev->i2c_dev.addr, delta, dev->previous_raw_counts, current_f);
                        }
                    }

                    // Update previous value for next iteration
                    dev->previous_raw_counts = current_f;
                    xSemaphoreGive(dev->data_mutex);

                    // Periodically log the raw reading to help diagnose issues
                    static int log_counter = 0;
                    if (++log_counter >= 100)
                    { // Log every ~2 seconds at 20ms interval
                        ESP_LOGD(TAG, "[0x%02x] Raw reading: %u, Accumulated: %.1f",
                                 dev->i2c_dev.addr, current_raw, dev->accumulated_counts);
                        log_counter = 0;
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "[0x%02x] Could not take mutex in update task", dev->i2c_dev.addr);
                }
            }
            else
            {
                fail_count++;
                ESP_LOGE(TAG, "[0x%02x] Failed to read sensor in update task: %d (%s), fails: %d",
                         dev->i2c_dev.addr, read_ret, esp_err_to_name(read_ret), fail_count);

                // If persistent failures, try reinitializing the I2C communication
                if (fail_count > 10)
                {
                    ESP_LOGW(TAG, "[0x%02x] Persistent read failures, consider reinitializing sensor",
                             dev->i2c_dev.addr);
                    fail_count = 0; // Reset counter to avoid flooding logs
                }
            }
        }
        else
        {
            ESP_LOGW(TAG, "[0x%02x] Failed to take I2C mutex in update task", dev->i2c_dev.addr);
        }

        // If read failed, add a small delay before proceeding to avoid flooding with errors
        if (!read_success)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Wait for the next interval
        vTaskDelayUntil(&last_wake_time, update_interval_ticks);
    }
}