/**
 * @file as5600.c
 * @brief AS5600 magnetic rotary position sensor driver
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Original driver implementation for AS5600 magnetic encoder.
 *
 * Key features:
 * - 12-bit resolution (4096 positions per revolution)
 * - I2C interface at 400kHz
 * - Magnet detection and strength monitoring
 * - Low-power mode support (LPM3 saves ~5mA)
 * - Angular position and velocity tracking
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include <math.h>
#include <string.h>
#include <inttypes.h> // For PRIu32
#include "as5600.h"
#include "esp_log.h"
#include "esp_idf_lib_helpers.h"

#define I2C_FREQ_HZ 400000 // Set to 400 KHz to match other devices

#define AS5600_ANGLE_REGISTER_H 0x0E // Raw Angle (11:0) H byte
#define AS5600_ANGLE_REGISTER_L 0x0F // Raw Angle (11:0) L byte
#define AS5600_STATUS_REGISTER  0x0B // Status register
#define AS5600_CONF_REGISTER_H 0x07  // Configuration register (high byte)
#define AS5600_CONF_REGISTER_L 0x08  // Configuration register (low byte)
#define AS5600_PULSES_PER_REVOLUTION 4096.0f

// Status register bits (address 0x0B)
#define AS5600_STATUS_MD 0x20  // Bit 5: Magnet Detected (1 = detected)
#define AS5600_STATUS_ML 0x10  // Bit 4: Magnet too weak (AGC min overflow)
#define AS5600_STATUS_MH 0x08  // Bit 3: Magnet too strong (AGC max overflow)

// Power mode bits (PM[1:0]) in CONF register low byte
#define AS5600_PM_NORMAL 0x00  // Normal mode (NOM): Always on, 6.5mA
#define AS5600_PM_LPM1   0x01  // Low power mode 1: 5ms polling, 3.4mA
#define AS5600_PM_LPM2   0x02  // Low power mode 2: 20ms polling, 1.8mA
#define AS5600_PM_LPM3   0x03  // Low power mode 3: 100ms polling, 1.5mA
#define AS5600_PM_MASK   0x03  // Mask for PM bits (bits 1:0)
#define AS5600_READING_MASK 0x0FFF         // 12 bits
#define AS5600_MUTEX_TIMEOUT_MS 100

static const char *TAG = "AS5600";

// Threshold to detect wrap-around (50% = Nyquist criterion for encoder wrap detection)
// If movement > 2048 counts between samples, assume shortest path wrapped through zero
static const int32_t encoder_wrap_threshold = (int32_t)(AS5600_PULSES_PER_REVOLUTION / 2.0f); // 2048
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

// Forward declaration of internal function
static esp_err_t _as5600_read_raw_internal(as5600_t *dev, uint16_t *raw_angle);

/* Public API Functions */

esp_err_t as5600_init_desc(as5600_t *dev, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (port >= I2C_NUM_MAX)
    {
        ESP_LOGE(TAG, "Invalid I2C port: %d", port);
        return ESP_ERR_INVALID_ARG;
    }

    // Zero-initialize the structure
    memset(dev, 0, sizeof(as5600_t));
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

esp_err_t as5600_free_desc(as5600_t *dev)
{
    CHECK_ARG(dev);

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

esp_err_t as5600_init(as5600_t *dev)
{
    CHECK_ARG(dev);

    ESP_LOGI(TAG, "[0x%02x] Initializing AS5600 sensor", dev->i2c_dev.addr);

    // Create mutex for protecting shared data (accumulated, previous)
    dev->data_mutex = xSemaphoreCreateMutex();
    if (!dev->data_mutex)
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to create data mutex", dev->i2c_dev.addr);
        return ESP_ERR_NO_MEM;
    }

    // Initialize accumulated_counts and zero_offset
    dev->accumulated_counts = 0;
    dev->previous_raw_counts = 0;
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
        dev->previous_raw_counts = (int32_t)temp_raw;
        xSemaphoreGive(dev->data_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to take data mutex during init", dev->i2c_dev.addr);
        vSemaphoreDelete(dev->data_mutex);
        dev->data_mutex = NULL;
        return ESP_ERR_TIMEOUT;
    }

    // Mark as initialized - now using synchronous on-demand reads instead of autonomous task
    dev->initialized = true;
    ESP_LOGI(TAG, "[0x%02x] AS5600 initialized successfully (on-demand read mode)", dev->i2c_dev.addr);

    return ESP_OK;
}

esp_err_t as5600_read_raw_counts(as5600_t *dev, uint16_t *counts)
{
    CHECK_ARG(dev && counts);

    // Set default error value
    *counts = 0;

    if (!dev->initialized)
    {
        ESP_LOGE(TAG, "[0x%02x] Device not initialized", dev->i2c_dev.addr);
        return ESP_ERR_INVALID_STATE;
    }

    // Take I2C mutex and perform actual read
    esp_err_t ret = i2c_dev_take_mutex(&dev->i2c_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to take I2C mutex: %s", dev->i2c_dev.addr, esp_err_to_name(ret));
        return ret;
    }

    uint16_t current_raw;
    ret = _as5600_read_raw_internal(dev, &current_raw);
    i2c_dev_give_mutex(&dev->i2c_dev);

    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "[0x%02x] I2C read failed: %s", dev->i2c_dev.addr, esp_err_to_name(ret));
        return ret;
    }

    // Update accumulated counts with wrap-around handling (protected by data mutex)
    if (xSemaphoreTake(dev->data_mutex, pdMS_TO_TICKS(AS5600_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to take data mutex for accumulation", dev->i2c_dev.addr);
        return ESP_ERR_TIMEOUT;
    }

    int32_t current = (int32_t)current_raw;
    int32_t delta = current - dev->previous_raw_counts;

    // Detect wrap-around using 50% threshold (Nyquist criterion for rotary encoders)
    // If delta > 2048 counts (>180°), shortest path is backward wrap (crossed 0 going 4095→0)
    // If delta < -2048 counts (<-180°), shortest path is forward wrap (crossed 0 going 0→4095)
    if (delta > encoder_wrap_threshold)
    {
        // Example: previous=10, current=3086 → delta=+3076 (appears to jump forward)
        // Forward path: 10→3086 = 3076 counts (270°)
        // Backward path: 10→0→4095→3086 = 1020 counts (90°) ← shortest!
        // Correction: subtract one full revolution from accumulator
        dev->accumulated_counts -= (int32_t)AS5600_PULSES_PER_REVOLUTION;
        ESP_LOGD(TAG, "[0x%02x] Wrap detected (backward), delta: %ld", dev->i2c_dev.addr, (long)delta);
    }
    else if (delta < -encoder_wrap_threshold)
    {
        // Example: previous=3086, current=10 → delta=-3076 (appears to jump backward)
        // Forward path: 3086→4095→0→10 = 1020 counts (90°) ← shortest!
        // Backward path: 3086→10 = 3076 counts (270°)
        // Correction: add one full revolution to accumulator
        dev->accumulated_counts += (int32_t)AS5600_PULSES_PER_REVOLUTION;
        ESP_LOGD(TAG, "[0x%02x] Wrap detected (forward), delta: %ld", dev->i2c_dev.addr, (long)delta);
    }

    dev->previous_raw_counts = current;
    *counts = current_raw;

    xSemaphoreGive(dev->data_mutex);

    return ESP_OK;
}

esp_err_t as5600_read_relative_counts(as5600_t *dev, uint16_t *counts)
{
    CHECK_ARG(dev && counts);

    if (!dev->initialized)
        return ESP_ERR_INVALID_STATE;

    // First get fresh data from sensor (updates previous_raw_counts and accumulator)
    uint16_t raw;
    esp_err_t ret = as5600_read_raw_counts(dev, &raw);
    if (ret != ESP_OK)
        return ret;

    // Now calculate relative position with fresh data
    if (xSemaphoreTake(dev->data_mutex, pdMS_TO_TICKS(AS5600_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to take mutex for reading relative counts", dev->i2c_dev.addr);
        return ESP_ERR_TIMEOUT;
    }

    // Calculate relative counts: (current_raw - offset + full_range) % full_range
    int32_t relative = (int32_t)raw - (int32_t)dev->zero_offset;
    if (relative < 0)
        relative += (int32_t)AS5600_PULSES_PER_REVOLUTION;
    *counts = (uint16_t)(relative % (int32_t)AS5600_PULSES_PER_REVOLUTION);

    xSemaphoreGive(dev->data_mutex);

    return ESP_OK;
}

esp_err_t as5600_read_accumulated_counts(as5600_t *dev, int32_t *counts)
{
    CHECK_ARG(dev && counts);

    if (!dev->initialized)
        return ESP_ERR_INVALID_STATE;

    // First get fresh data from sensor (updates previous_raw_counts and accumulator)
    uint16_t raw;
    esp_err_t ret = as5600_read_raw_counts(dev, &raw);
    if (ret != ESP_OK)
        return ret;

    // Now read accumulated total with fresh data
    if (xSemaphoreTake(dev->data_mutex, pdMS_TO_TICKS(AS5600_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to take mutex for reading accumulated counts", dev->i2c_dev.addr);
        return ESP_ERR_TIMEOUT;
    }

    // Return absolute accumulated counts (ignores zero_offset)
    // The accumulated value tracks full revolutions, add current position within revolution
    *counts = dev->accumulated_counts + dev->previous_raw_counts;

    xSemaphoreGive(dev->data_mutex);

    return ESP_OK;
}

esp_err_t as5600_read_accumulated_counts_relative(as5600_t *dev, int32_t *counts)
{
    CHECK_ARG(dev && counts);

    if (!dev->initialized)
        return ESP_ERR_INVALID_STATE;

    // First get fresh data from sensor (updates previous_raw_counts and accumulator)
    uint16_t raw;
    esp_err_t ret = as5600_read_raw_counts(dev, &raw);
    if (ret != ESP_OK)
        return ret;

    // Now read relative accumulated total with fresh data
    if (xSemaphoreTake(dev->data_mutex, pdMS_TO_TICKS(AS5600_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to take mutex for reading relative accumulated counts", dev->i2c_dev.addr);
        return ESP_ERR_TIMEOUT;
    }

    // Return accumulated counts relative to zero_offset
    // Get total raw position, then subtract the zero offset
    int32_t total_raw_counts = dev->accumulated_counts + dev->previous_raw_counts;
    *counts = total_raw_counts - (int32_t)dev->zero_offset;

    xSemaphoreGive(dev->data_mutex);

    return ESP_OK;
}

esp_err_t as5600_read_angle_degrees(as5600_t *dev, float *angle)
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

esp_err_t as5600_read_angle_radians(as5600_t *dev, float *angle)
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

esp_err_t as5600_set_zero_offset(as5600_t *dev)
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

float as5600_get_counts_per_revolution(as5600_t *dev)
{
    (void)dev; // Parameter not needed for this constant value
    return AS5600_PULSES_PER_REVOLUTION;
}

esp_err_t as5600_set_power_mode(as5600_t *dev, uint8_t power_mode)
{
    CHECK_ARG(dev);

    // Validate power mode
    if (power_mode > AS5600_PM_LPM3) {
        ESP_LOGE(TAG, "[0x%02x] Invalid power mode: %u (valid: 0-3)", dev->i2c_dev.addr, power_mode);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;
    if ((ret = i2c_dev_take_mutex(&dev->i2c_dev)) != ESP_OK)
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to take mutex during power mode setup: %s",
                 dev->i2c_dev.addr, esp_err_to_name(ret));
        return ret;
    }

    // Read current CONF register low byte
    uint8_t conf_low = 0;
    ret = i2c_dev_read_reg(&dev->i2c_dev, AS5600_CONF_REGISTER_L, &conf_low, 1);
    if (ret != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        ESP_LOGE(TAG, "[0x%02x] Failed to read CONF register: %s", dev->i2c_dev.addr, esp_err_to_name(ret));
        return ret;
    }

    // Modify PM bits (bits 1:0) while preserving other bits
    conf_low = (conf_low & ~AS5600_PM_MASK) | (power_mode & AS5600_PM_MASK);

    // Write back to CONF register
    ret = i2c_dev_write_reg(&dev->i2c_dev, AS5600_CONF_REGISTER_L, &conf_low, 1);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[0x%02x] Failed to write CONF register: %s", dev->i2c_dev.addr, esp_err_to_name(ret));
        return ret;
    }

    const char *mode_str[] = {"NORMAL", "LPM1", "LPM2", "LPM3"};
    ESP_LOGD(TAG, "[0x%02x] Power mode set to %s", dev->i2c_dev.addr, mode_str[power_mode]);

    return ESP_OK;
}

esp_err_t as5600_read_status(as5600_t *dev, as5600_status_t *status)
{
    CHECK_ARG(dev);
    CHECK_ARG(status);

    esp_err_t ret = ESP_OK;
    if ((ret = i2c_dev_take_mutex(&dev->i2c_dev)) != ESP_OK)
    {
        ESP_LOGE(TAG, "[0x%02x] Failed to take mutex during status read: %s",
                 dev->i2c_dev.addr, esp_err_to_name(ret));
        return ret;
    }


    // Read status register
    uint8_t status_reg = 0;
    ret = i2c_dev_read_reg(&dev->i2c_dev, AS5600_STATUS_REGISTER, &status_reg, 1);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[0x%02x] Failed to read STATUS register: %s", dev->i2c_dev.addr, esp_err_to_name(ret));
        return ret;
    }

    // Parse status bits
    status->magnet_detected = (status_reg & AS5600_STATUS_MD) != 0;
    status->magnet_too_weak = (status_reg & AS5600_STATUS_ML) != 0;
    status->magnet_too_strong = (status_reg & AS5600_STATUS_MH) != 0;

    ESP_LOGD(TAG, "[0x%02x] Status: MD=%d ML=%d MH=%d",
             dev->i2c_dev.addr,
             status->magnet_detected,
             status->magnet_too_weak,
             status->magnet_too_strong);

    return ESP_OK;
}

/* Internal Functions */

// Internal function to read the raw 12-bit angle
static esp_err_t _as5600_read_raw_internal(as5600_t *dev, uint16_t *raw_angle)
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