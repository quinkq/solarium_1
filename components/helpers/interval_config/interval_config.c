/**
 * @file interval_config.c
 * @brief Interval configuration implementation
 */

#include "interval_config.h"
#include "esp_log.h"
#include "esp_crc.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "interval_config";

#define INTERVAL_CONFIG_PATH "/data/intervals.dat"

// Default configuration (Balanced preset)
static const interval_config_t DEFAULT_INTERVALS = {
    .fluctus_power_day_min = 15,
    .fluctus_power_night_min = 60,
    .fluctus_solar_correction_min = 15,
    .tempesta_normal_min = 15,
    .tempesta_power_save_min = 60,
    .impluvium_optimal_min = 15,
    .impluvium_cool_min = 30,
    .impluvium_power_save_min = 60,
    .impluvium_night_min_hours = 3,
    .crc32 = 0  // Calculated on save
};

// Preset definitions
static const interval_config_t PRESET_AGGRESSIVE = {
    .fluctus_power_day_min = 5,
    .fluctus_power_night_min = 30,
    .fluctus_solar_correction_min = 5,
    .tempesta_normal_min = 5,
    .tempesta_power_save_min = 30,
    .impluvium_optimal_min = 5,
    .impluvium_cool_min = 10,
    .impluvium_power_save_min = 30,
    .impluvium_night_min_hours = 1,
    .crc32 = 0
};

static const interval_config_t PRESET_BALANCED = {
    .fluctus_power_day_min = 15,
    .fluctus_power_night_min = 60,
    .fluctus_solar_correction_min = 15,
    .tempesta_normal_min = 15,
    .tempesta_power_save_min = 60,
    .impluvium_optimal_min = 15,
    .impluvium_cool_min = 30,
    .impluvium_power_save_min = 60,
    .impluvium_night_min_hours = 3,
    .crc32 = 0
};

static const interval_config_t PRESET_CONSERVATIVE = {
    .fluctus_power_day_min = 30,
    .fluctus_power_night_min = 120,
    .fluctus_solar_correction_min = 30,
    .tempesta_normal_min = 30,
    .tempesta_power_save_min = 120,
    .impluvium_optimal_min = 30,
    .impluvium_cool_min = 60,
    .impluvium_power_save_min = 120,
    .impluvium_night_min_hours = 6,
    .crc32 = 0
};

// Global configuration instance
interval_config_t g_interval_config = {0};

/**
 * @brief Calculate CRC32 for configuration structure
 */
static uint32_t calculate_crc32(const interval_config_t *config)
{
    // CRC over all fields except crc32 itself
    return esp_crc32_le(0, (const uint8_t *)config,
                       sizeof(interval_config_t) - sizeof(uint32_t));
}

/**
 * @brief Compare two configurations (ignoring CRC)
 */
static bool configs_equal(const interval_config_t *a, const interval_config_t *b)
{
    return (a->fluctus_power_day_min == b->fluctus_power_day_min &&
            a->fluctus_power_night_min == b->fluctus_power_night_min &&
            a->fluctus_solar_correction_min == b->fluctus_solar_correction_min &&
            a->tempesta_normal_min == b->tempesta_normal_min &&
            a->tempesta_power_save_min == b->tempesta_power_save_min &&
            a->impluvium_optimal_min == b->impluvium_optimal_min &&
            a->impluvium_cool_min == b->impluvium_cool_min &&
            a->impluvium_power_save_min == b->impluvium_power_save_min &&
            a->impluvium_night_min_hours == b->impluvium_night_min_hours);
}

bool interval_config_validate(const interval_config_t *config)
{
    // FLUCTUS power monitoring
    if (config->fluctus_power_day_min < INTERVAL_FLUCTUS_POWER_DAY_MIN ||
        config->fluctus_power_day_min > INTERVAL_FLUCTUS_POWER_DAY_MAX) {
        ESP_LOGW(TAG, "Invalid fluctus_power_day_min: %lu (range: %d-%d)",
                 config->fluctus_power_day_min, INTERVAL_FLUCTUS_POWER_DAY_MIN, INTERVAL_FLUCTUS_POWER_DAY_MAX);
        return false;
    }
    if (config->fluctus_power_night_min < INTERVAL_FLUCTUS_POWER_NIGHT_MIN ||
        config->fluctus_power_night_min > INTERVAL_FLUCTUS_POWER_NIGHT_MAX) {
        ESP_LOGW(TAG, "Invalid fluctus_power_night_min: %lu (range: %d-%d)",
                 config->fluctus_power_night_min, INTERVAL_FLUCTUS_POWER_NIGHT_MIN, INTERVAL_FLUCTUS_POWER_NIGHT_MAX);
        return false;
    }

    // FLUCTUS solar tracking
    if (config->fluctus_solar_correction_min < INTERVAL_FLUCTUS_SOLAR_MIN ||
        config->fluctus_solar_correction_min > INTERVAL_FLUCTUS_SOLAR_MAX) {
        ESP_LOGW(TAG, "Invalid fluctus_solar_correction_min: %lu (range: %d-%d)",
                 config->fluctus_solar_correction_min, INTERVAL_FLUCTUS_SOLAR_MIN, INTERVAL_FLUCTUS_SOLAR_MAX);
        return false;
    }

    // TEMPESTA weather collection
    if (config->tempesta_normal_min < INTERVAL_TEMPESTA_NORMAL_MIN ||
        config->tempesta_normal_min > INTERVAL_TEMPESTA_NORMAL_MAX) {
        ESP_LOGW(TAG, "Invalid tempesta_normal_min: %lu (range: %d-%d)",
                 config->tempesta_normal_min, INTERVAL_TEMPESTA_NORMAL_MIN, INTERVAL_TEMPESTA_NORMAL_MAX);
        return false;
    }
    if (config->tempesta_power_save_min < INTERVAL_TEMPESTA_POWER_SAVE_MIN ||
        config->tempesta_power_save_min > INTERVAL_TEMPESTA_POWER_SAVE_MAX) {
        ESP_LOGW(TAG, "Invalid tempesta_power_save_min: %lu (range: %d-%d)",
                 config->tempesta_power_save_min, INTERVAL_TEMPESTA_POWER_SAVE_MIN, INTERVAL_TEMPESTA_POWER_SAVE_MAX);
        return false;
    }

    // IMPLUVIUM moisture checks
    if (config->impluvium_optimal_min < INTERVAL_IMPLUVIUM_OPTIMAL_MIN ||
        config->impluvium_optimal_min > INTERVAL_IMPLUVIUM_OPTIMAL_MAX) {
        ESP_LOGW(TAG, "Invalid impluvium_optimal_min: %lu (range: %d-%d)",
                 config->impluvium_optimal_min, INTERVAL_IMPLUVIUM_OPTIMAL_MIN, INTERVAL_IMPLUVIUM_OPTIMAL_MAX);
        return false;
    }
    if (config->impluvium_cool_min < INTERVAL_IMPLUVIUM_COOL_MIN ||
        config->impluvium_cool_min > INTERVAL_IMPLUVIUM_COOL_MAX) {
        ESP_LOGW(TAG, "Invalid impluvium_cool_min: %lu (range: %d-%d)",
                 config->impluvium_cool_min, INTERVAL_IMPLUVIUM_COOL_MIN, INTERVAL_IMPLUVIUM_COOL_MAX);
        return false;
    }
    if (config->impluvium_power_save_min < INTERVAL_IMPLUVIUM_POWER_SAVE_MIN ||
        config->impluvium_power_save_min > INTERVAL_IMPLUVIUM_POWER_SAVE_MAX) {
        ESP_LOGW(TAG, "Invalid impluvium_power_save_min: %lu (range: %d-%d)",
                 config->impluvium_power_save_min, INTERVAL_IMPLUVIUM_POWER_SAVE_MIN, INTERVAL_IMPLUVIUM_POWER_SAVE_MAX);
        return false;
    }
    if (config->impluvium_night_min_hours < INTERVAL_IMPLUVIUM_NIGHT_MIN ||
        config->impluvium_night_min_hours > INTERVAL_IMPLUVIUM_NIGHT_MAX) {
        ESP_LOGW(TAG, "Invalid impluvium_night_min_hours: %lu (range: %d-%d)",
                 config->impluvium_night_min_hours, INTERVAL_IMPLUVIUM_NIGHT_MIN, INTERVAL_IMPLUVIUM_NIGHT_MAX);
        return false;
    }

    return true;
}

esp_err_t interval_config_init(void)
{
    FILE *f = fopen(INTERVAL_CONFIG_PATH, "rb");
    if (f == NULL) {
        ESP_LOGW(TAG, "Interval config not found, using defaults");
        memcpy(&g_interval_config, &DEFAULT_INTERVALS, sizeof(interval_config_t));
        return interval_config_save();  // Create file with defaults
    }

    size_t read = fread(&g_interval_config, 1, sizeof(interval_config_t), f);
    fclose(f);

    if (read != sizeof(interval_config_t)) {
        ESP_LOGE(TAG, "Corrupted interval config (size mismatch: %u != %u), using defaults",
                 read, sizeof(interval_config_t));
        memcpy(&g_interval_config, &DEFAULT_INTERVALS, sizeof(interval_config_t));
        return interval_config_save();
    }

    // Validate CRC32
    uint32_t calculated_crc = calculate_crc32(&g_interval_config);
    if (calculated_crc != g_interval_config.crc32) {
        ESP_LOGE(TAG, "Corrupted interval config (CRC mismatch: 0x%08lX != 0x%08lX), using defaults",
                 calculated_crc, g_interval_config.crc32);
        memcpy(&g_interval_config, &DEFAULT_INTERVALS, sizeof(interval_config_t));
        return interval_config_save();
    }

    // Validate ranges (corrupted values can pass CRC but be invalid)
    if (!interval_config_validate(&g_interval_config)) {
        ESP_LOGE(TAG, "Invalid interval values in config, using defaults");
        memcpy(&g_interval_config, &DEFAULT_INTERVALS, sizeof(interval_config_t));
        return interval_config_save();
    }

    ESP_LOGI(TAG, "Interval config loaded successfully");
    ESP_LOGI(TAG, "  FLUCTUS: %lum/%lum (day/night), %lum (solar)",
             g_interval_config.fluctus_power_day_min,
             g_interval_config.fluctus_power_night_min,
             g_interval_config.fluctus_solar_correction_min);
    ESP_LOGI(TAG, "  TEMPESTA: %lum/%lum (normal/power save)",
             g_interval_config.tempesta_normal_min,
             g_interval_config.tempesta_power_save_min);
    ESP_LOGI(TAG, "  IMPLUVIUM: %lum/%lum/%lum (opt/cool/pwr), %luh (night)",
             g_interval_config.impluvium_optimal_min,
             g_interval_config.impluvium_cool_min,
             g_interval_config.impluvium_power_save_min,
             g_interval_config.impluvium_night_min_hours);

    return ESP_OK;
}

esp_err_t interval_config_save(void)
{
    // Validate before saving
    if (!interval_config_validate(&g_interval_config)) {
        ESP_LOGE(TAG, "Refusing to save invalid configuration");
        return ESP_ERR_INVALID_ARG;
    }

    // Update CRC32 before saving
    g_interval_config.crc32 = calculate_crc32(&g_interval_config);

    FILE *f = fopen(INTERVAL_CONFIG_PATH, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open interval config for writing");
        return ESP_FAIL;
    }

    size_t written = fwrite(&g_interval_config, 1, sizeof(interval_config_t), f);
    fclose(f);

    if (written != sizeof(interval_config_t)) {
        ESP_LOGE(TAG, "Failed to write interval config (wrote %u of %u bytes)",
                 written, sizeof(interval_config_t));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Interval config saved successfully (CRC: 0x%08lX)", g_interval_config.crc32);
    return ESP_OK;
}

void interval_config_reset_defaults(void)
{
    memcpy(&g_interval_config, &DEFAULT_INTERVALS, sizeof(interval_config_t));
    interval_config_save();
    ESP_LOGI(TAG, "Interval config reset to defaults (Balanced preset)");
}

esp_err_t interval_config_set_fluctus_power(uint32_t day_min, uint32_t night_min)
{
    if (day_min < INTERVAL_FLUCTUS_POWER_DAY_MIN || day_min > INTERVAL_FLUCTUS_POWER_DAY_MAX) {
        ESP_LOGW(TAG, "Invalid day_min: %lu (range: %d-%d)", day_min,
                 INTERVAL_FLUCTUS_POWER_DAY_MIN, INTERVAL_FLUCTUS_POWER_DAY_MAX);
        return ESP_ERR_INVALID_ARG;
    }
    if (night_min < INTERVAL_FLUCTUS_POWER_NIGHT_MIN || night_min > INTERVAL_FLUCTUS_POWER_NIGHT_MAX) {
        ESP_LOGW(TAG, "Invalid night_min: %lu (range: %d-%d)", night_min,
                 INTERVAL_FLUCTUS_POWER_NIGHT_MIN, INTERVAL_FLUCTUS_POWER_NIGHT_MAX);
        return ESP_ERR_INVALID_ARG;
    }

    g_interval_config.fluctus_power_day_min = day_min;
    g_interval_config.fluctus_power_night_min = night_min;

    return interval_config_save();
}

esp_err_t interval_config_set_fluctus_solar(uint32_t correction_min)
{
    if (correction_min < INTERVAL_FLUCTUS_SOLAR_MIN || correction_min > INTERVAL_FLUCTUS_SOLAR_MAX) {
        ESP_LOGW(TAG, "Invalid correction_min: %lu (range: %d-%d)", correction_min,
                 INTERVAL_FLUCTUS_SOLAR_MIN, INTERVAL_FLUCTUS_SOLAR_MAX);
        return ESP_ERR_INVALID_ARG;
    }

    g_interval_config.fluctus_solar_correction_min = correction_min;

    return interval_config_save();
}

esp_err_t interval_config_set_tempesta(uint32_t normal_min, uint32_t power_save_min)
{
    if (normal_min < INTERVAL_TEMPESTA_NORMAL_MIN || normal_min > INTERVAL_TEMPESTA_NORMAL_MAX) {
        ESP_LOGW(TAG, "Invalid normal_min: %lu (range: %d-%d)", normal_min,
                 INTERVAL_TEMPESTA_NORMAL_MIN, INTERVAL_TEMPESTA_NORMAL_MAX);
        return ESP_ERR_INVALID_ARG;
    }
    if (power_save_min < INTERVAL_TEMPESTA_POWER_SAVE_MIN || power_save_min > INTERVAL_TEMPESTA_POWER_SAVE_MAX) {
        ESP_LOGW(TAG, "Invalid power_save_min: %lu (range: %d-%d)", power_save_min,
                 INTERVAL_TEMPESTA_POWER_SAVE_MIN, INTERVAL_TEMPESTA_POWER_SAVE_MAX);
        return ESP_ERR_INVALID_ARG;
    }

    g_interval_config.tempesta_normal_min = normal_min;
    g_interval_config.tempesta_power_save_min = power_save_min;

    return interval_config_save();
}

esp_err_t interval_config_set_impluvium(uint32_t optimal_min, uint32_t cool_min,
                                        uint32_t power_save_min, uint32_t night_hours)
{
    if (optimal_min < INTERVAL_IMPLUVIUM_OPTIMAL_MIN || optimal_min > INTERVAL_IMPLUVIUM_OPTIMAL_MAX) {
        ESP_LOGW(TAG, "Invalid optimal_min: %lu (range: %d-%d)", optimal_min,
                 INTERVAL_IMPLUVIUM_OPTIMAL_MIN, INTERVAL_IMPLUVIUM_OPTIMAL_MAX);
        return ESP_ERR_INVALID_ARG;
    }
    if (cool_min < INTERVAL_IMPLUVIUM_COOL_MIN || cool_min > INTERVAL_IMPLUVIUM_COOL_MAX) {
        ESP_LOGW(TAG, "Invalid cool_min: %lu (range: %d-%d)", cool_min,
                 INTERVAL_IMPLUVIUM_COOL_MIN, INTERVAL_IMPLUVIUM_COOL_MAX);
        return ESP_ERR_INVALID_ARG;
    }
    if (power_save_min < INTERVAL_IMPLUVIUM_POWER_SAVE_MIN || power_save_min > INTERVAL_IMPLUVIUM_POWER_SAVE_MAX) {
        ESP_LOGW(TAG, "Invalid power_save_min: %lu (range: %d-%d)", power_save_min,
                 INTERVAL_IMPLUVIUM_POWER_SAVE_MIN, INTERVAL_IMPLUVIUM_POWER_SAVE_MAX);
        return ESP_ERR_INVALID_ARG;
    }
    if (night_hours < INTERVAL_IMPLUVIUM_NIGHT_MIN || night_hours > INTERVAL_IMPLUVIUM_NIGHT_MAX) {
        ESP_LOGW(TAG, "Invalid night_hours: %lu (range: %d-%d)", night_hours,
                 INTERVAL_IMPLUVIUM_NIGHT_MIN, INTERVAL_IMPLUVIUM_NIGHT_MAX);
        return ESP_ERR_INVALID_ARG;
    }

    g_interval_config.impluvium_optimal_min = optimal_min;
    g_interval_config.impluvium_cool_min = cool_min;
    g_interval_config.impluvium_power_save_min = power_save_min;
    g_interval_config.impluvium_night_min_hours = night_hours;

    return interval_config_save();
}

esp_err_t interval_config_apply_preset(interval_preset_t preset)
{
    const interval_config_t *source = NULL;

    switch (preset) {
        case INTERVAL_PRESET_AGGRESSIVE:
            source = &PRESET_AGGRESSIVE;
            break;
        case INTERVAL_PRESET_BALANCED:
            source = &PRESET_BALANCED;
            break;
        case INTERVAL_PRESET_CONSERVATIVE:
            source = &PRESET_CONSERVATIVE;
            break;
        default:
            ESP_LOGW(TAG, "Invalid preset: %d", preset);
            return ESP_ERR_INVALID_ARG;
    }

    memcpy(&g_interval_config, source, sizeof(interval_config_t));
    esp_err_t ret = interval_config_save();

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Applied preset: %s", interval_config_get_preset_name(preset));
    }

    return ret;
}

interval_preset_t interval_config_get_current_preset(void)
{
    if (configs_equal(&g_interval_config, &PRESET_AGGRESSIVE)) {
        return INTERVAL_PRESET_AGGRESSIVE;
    } else if (configs_equal(&g_interval_config, &PRESET_BALANCED)) {
        return INTERVAL_PRESET_BALANCED;
    } else if (configs_equal(&g_interval_config, &PRESET_CONSERVATIVE)) {
        return INTERVAL_PRESET_CONSERVATIVE;
    } else {
        return INTERVAL_PRESET_CUSTOM;
    }
}

const char* interval_config_get_preset_name(interval_preset_t preset)
{
    switch (preset) {
        case INTERVAL_PRESET_AGGRESSIVE:   return "Aggressive";
        case INTERVAL_PRESET_BALANCED:     return "Balanced";
        case INTERVAL_PRESET_CONSERVATIVE: return "Conservative";
        case INTERVAL_PRESET_CUSTOM:       return "Custom";
        default:                           return "Unknown";
    }
}
