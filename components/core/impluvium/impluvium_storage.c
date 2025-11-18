/**
 * @file impluvium_storage.c
 * @brief LittleFS storage management for IMPLUVIUM irrigation system
 *
 * Handles persistent storage of zone configuration, learning data, and RTC
 * accumulator management for water usage tracking.
 */

#include "impluvium.h"
#include "impluvium_private.h"
#include "esp_vfs.h"

static const char *TAG = "IMPLUVIUM_STORAGE";

/**
 * @brief Calculate CRC16-CCITT checksum
 *
 * Standard CRC16-CCITT algorithm (polynomial 0x1021) for data integrity verification.
 *
 * @param[in] data   Pointer to data buffer
 * @param[in] length Length of data in bytes
 * @return CRC16 checksum value
 */
static uint16_t impluvium_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Save zone configuration to LittleFS
 *
 * Writes user-editable zone settings to /irrigation/config.dat.
 * Called when user updates configuration via MQTT (future feature).
 *
 * @return ESP_OK on success, ESP_FAIL on write error
 */
esp_err_t impluvium_save_zone_config(void)
{
    irrigation_config_file_t config_file = {0};

    config_file.magic = 0x494D5043;  // "IMPC"
    config_file.version = 1;
    config_file.last_saved = time(NULL);

    // Copy zone configuration from RAM
    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex for config save");
        return ESP_FAIL;
    }

    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        config_file.zones[i].enabled = irrigation_zones[i].watering_enabled;
        config_file.zones[i].target_moisture_percent = irrigation_zones[i].target_moisture_percent;
        config_file.zones[i].moisture_deadband_percent = irrigation_zones[i].moisture_deadband_percent;
        config_file.zones[i].padding = 0;
    }

    xSemaphoreGive(xIrrigationMutex);

    // Calculate CRC on zones data
    config_file.crc16 = impluvium_crc16((uint8_t *)&config_file.zones, sizeof(config_file.zones));

    // Write to LittleFS
    FILE *f = fopen("/littlefs/irrigation/config.dat", "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open config.dat for writing");
        return ESP_FAIL;
    }

    size_t written = fwrite(&config_file, 1, sizeof(config_file), f);
    fclose(f);

    if (written != sizeof(config_file)) {
        ESP_LOGE(TAG, "Failed to write complete config file (wrote %d/%d bytes)", written, sizeof(config_file));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Zone configuration saved to LittleFS (%d bytes)", sizeof(config_file));
    return ESP_OK;
}

/**
 * @brief Load zone configuration from LittleFS
 *
 * Reads user-editable zone settings from /irrigation/config.dat.
 * Called during impluvium_init() on system startup.
 * Falls back to hardcoded defaults if file doesn't exist or CRC fails.
 *
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if file doesn't exist,
 *         ESP_ERR_INVALID_CRC if checksum fails
 */
esp_err_t impluvium_load_zone_config(void)
{
    irrigation_config_file_t config_file = {0};

    // Try to open config file
    FILE *f = fopen("/littlefs/irrigation/config.dat", "rb");
    if (f == NULL) {
        ESP_LOGW(TAG, "Config file not found - using defaults");
        return ESP_ERR_NOT_FOUND;
    }

    // Read file
    size_t bytes_read = fread(&config_file, 1, sizeof(config_file), f);
    fclose(f);

    if (bytes_read != sizeof(config_file)) {
        ESP_LOGW(TAG, "Config file incomplete (read %d/%d bytes) - using defaults", bytes_read, sizeof(config_file));
        return ESP_FAIL;
    }

    // Verify magic number and version
    if (config_file.magic != 0x494D5043) {
        ESP_LOGW(TAG, "Config file magic mismatch (0x%08lX) - using defaults", config_file.magic);
        return ESP_FAIL;
    }

    if (config_file.version != 1) {
        ESP_LOGW(TAG, "Config file version mismatch (%d) - using defaults", config_file.version);
        return ESP_FAIL;
    }

    // Verify CRC
    uint16_t calculated_crc = impluvium_crc16((uint8_t *)&config_file.zones, sizeof(config_file.zones));
    if (calculated_crc != config_file.crc16) {
        ESP_LOGW(TAG, "Config file CRC mismatch (calculated 0x%04X, stored 0x%04X) - using defaults",
                 calculated_crc, config_file.crc16);
        return ESP_ERR_INVALID_CRC;
    }

    // CRC valid - load configuration into RAM
    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex for config load");
        return ESP_FAIL;
    }

    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        irrigation_zones[i].watering_enabled = config_file.zones[i].enabled;
        irrigation_zones[i].target_moisture_percent = config_file.zones[i].target_moisture_percent;
        irrigation_zones[i].moisture_deadband_percent = config_file.zones[i].moisture_deadband_percent;

        ESP_LOGI(TAG, "Zone %d config loaded: enabled=%d, target=%.1f%%, deadband=%.1f%%",
                 i, config_file.zones[i].enabled, config_file.zones[i].target_moisture_percent,
                 config_file.zones[i].moisture_deadband_percent);
    }

    xSemaphoreGive(xIrrigationMutex);

    ESP_LOGI(TAG, "Zone configuration loaded from LittleFS (saved at %ld)", config_file.last_saved);
    return ESP_OK;
}

/**
 * @brief Save learning data for all zones to LittleFS
 *
 * Writes learned parameters and recent history to /irrigation/learning.dat.
 * Called daily at midnight via impluvium_daily_reset_callback().
 * Stores only the 5 most recent valid (non-anomalous) watering events per zone.
 *
 * @return ESP_OK on success, ESP_FAIL on write error
 */
esp_err_t impluvium_save_learning_data_all_zones(void)
{
    irrigation_learning_file_t learning_file = {0};

    learning_file.magic = 0x494D504C;  // "IMPL"
    learning_file.version = 1;
    learning_file.last_saved = time(NULL);

    // Copy learning data from RAM
    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex for learning data save");
        return ESP_FAIL;
    }

    for (uint8_t zone_id = 0; zone_id < IRRIGATION_ZONE_COUNT; zone_id++) {
        zone_learning_t *learning = &irrigation_zones[zone_id].learning;

        // Copy learned parameters
        learning_file.zones[zone_id].calculated_ppmp_ratio = learning->calculated_ppmp_ratio;
        learning_file.zones[zone_id].calculated_pump_duty_cycle = learning->calculated_pump_duty_cycle;
        learning_file.zones[zone_id].target_moisture_gain_rate = learning->target_moisture_gain_rate;
        learning_file.zones[zone_id].confidence_level = learning->confidence_level;
        learning_file.zones[zone_id].successful_predictions = learning->successful_predictions;
        learning_file.zones[zone_id].total_predictions = learning->total_predictions;

        // Copy 5 most recent valid entries from circular buffer
        uint8_t stored_count = 0;
        for (uint8_t i = 0; i < learning->history_entry_count && stored_count < LEARNING_STORED_HISTORY; i++) {
            // Start from most recent entry and work backwards
            uint8_t idx = (learning->history_index - i - 1 + LEARNING_HISTORY_SIZE) % LEARNING_HISTORY_SIZE;

            // Only store valid (non-anomalous) entries
            if (!learning->anomaly_flags[idx]) {
                learning_file.zones[zone_id].pulses_used[stored_count] = learning->pulses_used_history[idx];
                learning_file.zones[zone_id].moisture_increase_percent[stored_count] =
                    learning->moisture_increase_percent_history[idx];
                learning_file.zones[zone_id].anomaly_flags[stored_count] = false;
                stored_count++;
            }
        }

        learning_file.zones[zone_id].history_count = stored_count;
        learning_file.zones[zone_id].history_write_index = 0;  // Reset to start on load
    }

    xSemaphoreGive(xIrrigationMutex);

    // Calculate CRC on zones data
    learning_file.crc16 = impluvium_crc16((uint8_t *)&learning_file.zones, sizeof(learning_file.zones));

    // Write to LittleFS
    FILE *f = fopen("/littlefs/irrigation/learning.dat", "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open learning.dat for writing");
        return ESP_FAIL;
    }

    size_t written = fwrite(&learning_file, 1, sizeof(learning_file), f);
    fclose(f);

    if (written != sizeof(learning_file)) {
        ESP_LOGE(TAG, "Failed to write complete learning file (wrote %d/%d bytes)", written, sizeof(learning_file));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Learning data saved to LittleFS (%d bytes)", sizeof(learning_file));
    return ESP_OK;
}

/**
 * @brief Load learning data for all zones from LittleFS
 *
 * Reads learned parameters and recent history from /irrigation/learning.dat.
 * Called during impluvium_init() on system startup.
 * Falls back to defaults if file doesn't exist or CRC fails.
 *
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if file doesn't exist,
 *         ESP_ERR_INVALID_CRC if checksum fails
 */
esp_err_t impluvium_load_learning_data_all_zones(void)
{
    irrigation_learning_file_t learning_file = {0};

    // Try to open learning file
    FILE *f = fopen("/littlefs/irrigation/learning.dat", "rb");
    if (f == NULL) {
        ESP_LOGW(TAG, "Learning data file not found - using defaults");
        return ESP_ERR_NOT_FOUND;
    }

    // Read file
    size_t bytes_read = fread(&learning_file, 1, sizeof(learning_file), f);
    fclose(f);

    if (bytes_read != sizeof(learning_file)) {
        ESP_LOGW(TAG, "Learning file incomplete (read %d/%d bytes) - using defaults",
                 bytes_read, sizeof(learning_file));
        return ESP_FAIL;
    }

    // Verify magic number and version
    if (learning_file.magic != 0x494D504C) {
        ESP_LOGW(TAG, "Learning file magic mismatch (0x%08lX) - using defaults", learning_file.magic);
        return ESP_FAIL;
    }

    if (learning_file.version != 1) {
        ESP_LOGW(TAG, "Learning file version mismatch (%d) - using defaults", learning_file.version);
        return ESP_FAIL;
    }

    // Verify CRC
    uint16_t calculated_crc = impluvium_crc16((uint8_t *)&learning_file.zones, sizeof(learning_file.zones));
    if (calculated_crc != learning_file.crc16) {
        ESP_LOGW(TAG, "Learning file CRC mismatch (calculated 0x%04X, stored 0x%04X) - using defaults",
                 calculated_crc, learning_file.crc16);
        return ESP_ERR_INVALID_CRC;
    }

    // CRC valid - load learning data into RAM
    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex for learning data load");
        return ESP_FAIL;
    }

    for (uint8_t zone_id = 0; zone_id < IRRIGATION_ZONE_COUNT; zone_id++) {
        zone_learning_t *learning = &irrigation_zones[zone_id].learning;

        // Load learned parameters
        learning->calculated_ppmp_ratio = learning_file.zones[zone_id].calculated_ppmp_ratio;
        learning->calculated_pump_duty_cycle = learning_file.zones[zone_id].calculated_pump_duty_cycle;
        learning->target_moisture_gain_rate = learning_file.zones[zone_id].target_moisture_gain_rate;
        learning->confidence_level = learning_file.zones[zone_id].confidence_level;
        learning->successful_predictions = learning_file.zones[zone_id].successful_predictions;
        learning->total_predictions = learning_file.zones[zone_id].total_predictions;

        // Load recent history
        uint8_t stored_count = learning_file.zones[zone_id].history_count;
        if (stored_count > LEARNING_STORED_HISTORY) {
            stored_count = LEARNING_STORED_HISTORY;  // Sanity check
        }

        for (uint8_t i = 0; i < stored_count; i++) {
            learning->pulses_used_history[i] = learning_file.zones[zone_id].pulses_used[i];
            learning->moisture_increase_percent_history[i] =
                learning_file.zones[zone_id].moisture_increase_percent[i];
            learning->anomaly_flags[i] = learning_file.zones[zone_id].anomaly_flags[i];
        }

        learning->history_entry_count = stored_count;
        learning->history_index = stored_count % LEARNING_HISTORY_SIZE;

        ESP_LOGI(TAG, "Zone %d learning loaded: ppmp=%.1f, pump_duty=%lu, confidence=%.2f, history=%d entries",
                 zone_id, learning->calculated_ppmp_ratio, learning->calculated_pump_duty_cycle,
                 learning->confidence_level, stored_count);
    }

    xSemaphoreGive(xIrrigationMutex);

    ESP_LOGI(TAG, "Learning data loaded from LittleFS (saved at %ld)", learning_file.last_saved);
    return ESP_OK;
}

/**
 * @brief Update RTC accumulator with water usage for a zone
 *
 * Tracks hourly and daily water consumption per zone in RTC RAM.
 * Initializes accumulator on first call or after power loss.
 *
 * @param zone_id Zone that was watered (0-4)
 * @param water_used_ml Amount of water used in milliliters
 */
void impluvium_update_accumulator(uint8_t zone_id, float water_used_ml)
{
    if (zone_id >= IRRIGATION_ZONE_COUNT) {
        ESP_LOGW(TAG, "Invalid zone_id %d in accumulator update", zone_id);
        return;
    }

    // Initialize accumulator on first call or after power loss
    if (!rtc_impluvium_accumulator.initialized || rtc_impluvium_accumulator.current_hour_start == 0) {
        time_t now = time(NULL);
        ESP_LOGI(TAG, "Initializing RTC irrigation accumulator (first boot or power loss)");
        rtc_impluvium_accumulator.current_hour_start = now;
        rtc_impluvium_accumulator.current_day_start = now;

        // Zero all per-zone counters
        for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
            rtc_impluvium_accumulator.zone_water_used_hour_ml[i] = 0.0f;
            rtc_impluvium_accumulator.zone_events_hour[i] = 0;
            rtc_impluvium_accumulator.zone_water_used_day_ml[i] = 0.0f;
            rtc_impluvium_accumulator.zone_events_day[i] = 0;
        }

        rtc_impluvium_accumulator.initialized = true;
    }

    // Accumulate water usage for this zone
    rtc_impluvium_accumulator.zone_water_used_hour_ml[zone_id] += water_used_ml;
    rtc_impluvium_accumulator.zone_events_hour[zone_id]++;

    ESP_LOGD(TAG, "Zone %d accumulator: +%.1f mL, zone hour total: %.1f mL, events: %d",
             zone_id, water_used_ml,
             rtc_impluvium_accumulator.zone_water_used_hour_ml[zone_id],
             rtc_impluvium_accumulator.zone_events_hour[zone_id]);
}

/**
 * @brief Check for hourly rollover and update daily totals
 *
 * Called periodically during watering operations.
 * When hour changes, adds per-zone hourly totals to daily accumulator and resets hourly counters.
 *
 * @return true if hour changed (rollover occurred), false otherwise
 */
bool impluvium_check_hourly_rollover(void)
{
    if (!rtc_impluvium_accumulator.initialized) {
        return false;
    }

    time_t now = time(NULL);
    struct tm tm_now, tm_hour_start;
    gmtime_r(&now, &tm_now);
    gmtime_r(&rtc_impluvium_accumulator.current_hour_start, &tm_hour_start);

    // Check if hour changed
    if (tm_now.tm_hour != tm_hour_start.tm_hour ||
        tm_now.tm_yday != tm_hour_start.tm_yday ||
        tm_now.tm_year != tm_hour_start.tm_year) {

        // Calculate system totals for logging
        float total_hour_ml = 0.0f;
        uint8_t total_events = 0;

        // Update daily totals with completed hour data (per zone)
        for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
            rtc_impluvium_accumulator.zone_water_used_day_ml[i] += rtc_impluvium_accumulator.zone_water_used_hour_ml[i];
            rtc_impluvium_accumulator.zone_events_day[i] += rtc_impluvium_accumulator.zone_events_hour[i];

            total_hour_ml += rtc_impluvium_accumulator.zone_water_used_hour_ml[i];
            total_events += rtc_impluvium_accumulator.zone_events_hour[i];

            // Reset hourly counters for this zone
            rtc_impluvium_accumulator.zone_water_used_hour_ml[i] = 0.0f;
            rtc_impluvium_accumulator.zone_events_hour[i] = 0;
        }

        ESP_LOGI(TAG, "Hourly rollover: %.1f mL used, %d events (all zones)", total_hour_ml, total_events);

        // Reset hour timestamp
        rtc_impluvium_accumulator.current_hour_start = now;

        return true;  // Signal hourly event
    }

    return false;
}

/**
 * @brief Handle midnight reset logic
 *
 * Logs daily summary, resets per-zone daily counters, and saves learning data.
 * Called from impluvium_task within mutex protection.
 * Must be called with xIrrigationMutex held.
 */
void impluvium_handle_midnight_reset(void)
{
    ESP_LOGI(TAG, "=== Midnight Reset: Daily Irrigation Summary ===");

    // Calculate system totals for logging
    float total_day_ml = 0.0f;
    uint8_t total_events = 0;

    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        total_day_ml += rtc_impluvium_accumulator.zone_water_used_day_ml[i];
        total_events += rtc_impluvium_accumulator.zone_events_day[i];

        if (rtc_impluvium_accumulator.zone_water_used_day_ml[i] > 0.0f) {
            ESP_LOGI(TAG, "  Zone %d: %.1f mL, %d events",
                     i, rtc_impluvium_accumulator.zone_water_used_day_ml[i], rtc_impluvium_accumulator.zone_events_day[i]);
        }
    }

    ESP_LOGI(TAG, "  System total: %.1f mL, %d events", total_day_ml, total_events);
    ESP_LOGI(TAG, "==================================================");

    // Reset accumulator daily counters
    rtc_impluvium_accumulator.current_day_start = time(NULL);
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        rtc_impluvium_accumulator.zone_water_used_day_ml[i] = 0.0f;
        rtc_impluvium_accumulator.zone_events_day[i] = 0;
    }
    ESP_LOGI(TAG, "RTC accumulator daily counters reset");

    // Save learning data backup (once per day at midnight)
    ESP_LOGI(TAG, "Saving daily learning data backup");
    esp_err_t ret = impluvium_save_learning_data_all_zones();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save learning data: %s", esp_err_to_name(ret));
    }
}

// ########################## Public API - Zone Configuration ##########################

/**
 * @brief Update zone configuration and save to LittleFS
 *
 * @param[in] zone_id Zone identifier (0-4)
 * @param[in] target_moisture_percent Target moisture level (0-100%)
 * @param[in] moisture_deadband_percent Tolerance around target (0-50%)
 * @param[in] enabled Enable/disable zone for watering
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid parameters, ESP_FAIL on save failure
 */
esp_err_t impluvium_update_zone_config(uint8_t zone_id,
                                       float target_moisture_percent,
                                       float moisture_deadband_percent,
                                       bool enabled)
{
    // Validate parameters
    if (zone_id >= IRRIGATION_ZONE_COUNT) {
        ESP_LOGE(TAG, "Invalid zone_id %d (must be 0-%d)", zone_id, IRRIGATION_ZONE_COUNT - 1);
        return ESP_ERR_INVALID_ARG;
    }

    if (target_moisture_percent < 0.0f || target_moisture_percent > 100.0f) {
        ESP_LOGE(TAG, "Invalid target_moisture_percent %.1f%% (must be 0-100%%)", target_moisture_percent);
        return ESP_ERR_INVALID_ARG;
    }

    if (moisture_deadband_percent < 0.0f || moisture_deadband_percent > 50.0f) {
        ESP_LOGE(TAG, "Invalid moisture_deadband_percent %.1f%% (must be 0-50%%)", moisture_deadband_percent);
        return ESP_ERR_INVALID_ARG;
    }

    // Update zone configuration in RAM
    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex for zone config update");
        return ESP_FAIL;
    }

    irrigation_zones[zone_id].watering_enabled = enabled;
    irrigation_zones[zone_id].target_moisture_percent = target_moisture_percent;
    irrigation_zones[zone_id].moisture_deadband_percent = moisture_deadband_percent;

    ESP_LOGI(TAG, "Zone %d config updated: enabled=%d, target=%.1f%%, deadband=%.1f%%",
             zone_id, enabled, target_moisture_percent, moisture_deadband_percent);

    xSemaphoreGive(xIrrigationMutex);

    // Save updated configuration to LittleFS for persistence
    esp_err_t ret = impluvium_save_zone_config();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save updated zone config to LittleFS: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Zone %d configuration saved to LittleFS", zone_id);
    return ESP_OK;
}
