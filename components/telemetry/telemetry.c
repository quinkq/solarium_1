#include "telemetry.h"
#include "solar_calc.h"
#include "stellaria.h"
#include "fluctus.h"
#include "weather_station.h"
#include "irrigation.h"
#include "esp_log.h"
#include "esp_littlefs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

// ########################## Constants and Variables ##########################

static const char *TAG = "TELEMETRY";

// RTC RAM storage (survives deep sleep, lost on reset)
static RTC_DATA_ATTR power_accumulator_rtc_t rtc_accumulator = {0};

// Current hour data being accumulated
static power_telemetry_hour_t current_hour_data = {0};

// Circular buffer for historical data (in RAM, will be persisted to LittleFS)
static power_telemetry_hour_t historical_buffer[TELEMETRY_ROLLING_BUFFER_HOURS] = {0};
static uint16_t buffer_write_index = 0;
static uint16_t buffer_valid_entries = 0;

// Mutex for thread safety
static SemaphoreHandle_t xTelemetryMutex = NULL;

// Initialization flags
static bool telemetry_initialized = false;
static bool littlefs_mounted = false;

// Last sample time for delta calculations
static int64_t last_sample_timestamp_us = 0;

// ########################## Private Function Declarations ##########################

static esp_err_t telemetry_mount_littlefs(void);
static esp_err_t telemetry_load_historical_data(void);
static esp_err_t telemetry_save_hour_to_littlefs(const power_telemetry_hour_t *hour_data);
static void telemetry_check_hour_rollover(void);
static void telemetry_finalize_current_hour(void);
static void telemetry_reset_current_hour(void);
static void telemetry_add_to_circular_buffer(const power_telemetry_hour_t *hour_data);

// ########################## Private Functions ##########################

/**
 * @brief Mount LittleFS filesystem
 */
static esp_err_t telemetry_mount_littlefs(void)
{
    ESP_LOGI(TAG, "Mounting LittleFS...");

    esp_vfs_littlefs_conf_t conf = {
        .base_path = TELEMETRY_LITTLEFS_MOUNT_POINT,
        .partition_label = "littlefs",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    esp_err_t ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find LittleFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize LittleFS: %s", esp_err_to_name(ret));
        }
        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info("littlefs", &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "LittleFS: %d KB total, %d KB used", total / 1024, used / 1024);
    }

    littlefs_mounted = true;
    return ESP_OK;
}

/**
 * @brief Load historical data from LittleFS
 */
static esp_err_t telemetry_load_historical_data(void)
{
    if (!littlefs_mounted) {
        return ESP_ERR_INVALID_STATE;
    }

    FILE *f = fopen(TELEMETRY_POWER_DATA_FILE, "rb");
    if (f == NULL) {
        ESP_LOGW(TAG, "No existing power data file found, starting fresh");
        return ESP_ERR_NOT_FOUND;
    }

    // Read header: write_index and valid_entries
    if (fread(&buffer_write_index, sizeof(uint16_t), 1, f) != 1 ||
        fread(&buffer_valid_entries, sizeof(uint16_t), 1, f) != 1) {
        ESP_LOGE(TAG, "Failed to read historical data header");
        fclose(f);
        return ESP_FAIL;
    }

    // Read circular buffer data
    size_t entries_to_read = (buffer_valid_entries < TELEMETRY_ROLLING_BUFFER_HOURS) ?
                             buffer_valid_entries : TELEMETRY_ROLLING_BUFFER_HOURS;

    if (fread(historical_buffer, sizeof(power_telemetry_hour_t), entries_to_read, f) != entries_to_read) {
        ESP_LOGE(TAG, "Failed to read historical buffer data");
        fclose(f);
        return ESP_FAIL;
    }

    fclose(f);
    ESP_LOGI(TAG, "Loaded %d hours of historical data", buffer_valid_entries);
    return ESP_OK;
}

/**
 * @brief Save hour data to LittleFS
 */
static esp_err_t telemetry_save_hour_to_littlefs(const power_telemetry_hour_t *hour_data)
{
    if (!littlefs_mounted || hour_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    FILE *f = fopen(TELEMETRY_POWER_DATA_FILE, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open power data file for writing");
        return ESP_FAIL;
    }

    // Write header
    if (fwrite(&buffer_write_index, sizeof(uint16_t), 1, f) != 1 ||
        fwrite(&buffer_valid_entries, sizeof(uint16_t), 1, f) != 1) {
        ESP_LOGE(TAG, "Failed to write header");
        fclose(f);
        return ESP_FAIL;
    }

    // Write entire circular buffer
    size_t entries_to_write = (buffer_valid_entries < TELEMETRY_ROLLING_BUFFER_HOURS) ?
                              buffer_valid_entries : TELEMETRY_ROLLING_BUFFER_HOURS;

    if (fwrite(historical_buffer, sizeof(power_telemetry_hour_t), entries_to_write, f) != entries_to_write) {
        ESP_LOGE(TAG, "Failed to write buffer data");
        fclose(f);
        return ESP_FAIL;
    }

    fclose(f);
    ESP_LOGD(TAG, "Saved hourly data to LittleFS");
    return ESP_OK;
}

/**
 * @brief Add completed hour to circular buffer
 */
static void telemetry_add_to_circular_buffer(const power_telemetry_hour_t *hour_data)
{
    if (hour_data == NULL) return;

    // Write to circular buffer
    memcpy(&historical_buffer[buffer_write_index], hour_data, sizeof(power_telemetry_hour_t));

    // Update write index (circular)
    buffer_write_index = (buffer_write_index + 1) % TELEMETRY_ROLLING_BUFFER_HOURS;

    // Update valid entries count (max is buffer size)
    if (buffer_valid_entries < TELEMETRY_ROLLING_BUFFER_HOURS) {
        buffer_valid_entries++;
    }

    ESP_LOGI(TAG, "Added hour to buffer (index: %d, valid entries: %d)",
             buffer_write_index, buffer_valid_entries);
}

/**
 * @brief Finalize current hour and save to storage
 */
static void telemetry_finalize_current_hour(void)
{
    ESP_LOGI(TAG, "Finalizing hour: %.1f Wh PV, %.1f Wh battery consumed",
             current_hour_data.pv_energy_wh, current_hour_data.battery_consumed_wh);

    // Calculate averages
    if (current_hour_data.pv_sample_count > 0) {
        current_hour_data.pv_avg_power_w /= current_hour_data.pv_sample_count;
        current_hour_data.pv_avg_voltage_v /= current_hour_data.pv_sample_count;
        current_hour_data.pv_avg_current_a /= current_hour_data.pv_sample_count;
    }

    if (current_hour_data.battery_sample_count > 0) {
        current_hour_data.battery_avg_power_w /= current_hour_data.battery_sample_count;
        current_hour_data.battery_avg_current_a /= current_hour_data.battery_sample_count;
    }

    // Add to circular buffer
    telemetry_add_to_circular_buffer(&current_hour_data);

    // Save to LittleFS
    telemetry_save_hour_to_littlefs(&current_hour_data);
}

/**
 * @brief Reset current hour accumulator
 */
static void telemetry_reset_current_hour(void)
{
    memset(&current_hour_data, 0, sizeof(power_telemetry_hour_t));
    current_hour_data.timestamp_utc = time(NULL);

    // Reset RTC accumulator
    rtc_accumulator.current_hour_start = current_hour_data.timestamp_utc;
    rtc_accumulator.pv_energy_wh_accumulator = 0.0f;
    rtc_accumulator.battery_energy_wh_accumulator = 0.0f;
    rtc_accumulator.last_sample_time = current_hour_data.timestamp_utc;
    rtc_accumulator.initialized = true;

    ESP_LOGD(TAG, "Reset current hour accumulator");
}

/**
 * @brief Check if hour has rolled over and finalize if needed
 */
static void telemetry_check_hour_rollover(void)
{
    time_t now = time(NULL);
    struct tm now_tm, hour_start_tm;

    gmtime_r(&now, &now_tm);
    gmtime_r(&rtc_accumulator.current_hour_start, &hour_start_tm);

    // Check if we're in a new hour
    if (now_tm.tm_hour != hour_start_tm.tm_hour ||
        now_tm.tm_yday != hour_start_tm.tm_yday ||
        now_tm.tm_year != hour_start_tm.tm_year) {

        ESP_LOGI(TAG, "Hour rollover detected");
        telemetry_finalize_current_hour();
        telemetry_reset_current_hour();
    }
}

// ########################## Public API Functions ##########################

esp_err_t telemetry_init(void)
{
    ESP_LOGI(TAG, "Initializing telemetry subsystem...");

    if (telemetry_initialized) {
        ESP_LOGW(TAG, "Telemetry already initialized");
        return ESP_OK;
    }

    // Create mutex
    xTelemetryMutex = xSemaphoreCreateMutex();
    if (xTelemetryMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create telemetry mutex");
        return ESP_FAIL;
    }

    // Initialize solar calculator
    solar_calc_init();

    // Mount LittleFS
    esp_err_t ret = telemetry_mount_littlefs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount LittleFS");
        return ret;
    }

    // Load historical data
    telemetry_load_historical_data();

    // Initialize or recover RTC accumulator
    if (!rtc_accumulator.initialized || rtc_accumulator.current_hour_start == 0) {
        ESP_LOGI(TAG, "Initializing RTC accumulator (first boot or power loss)");
        telemetry_reset_current_hour();
    } else {
        ESP_LOGI(TAG, "Recovered RTC accumulator from deep sleep");
        // Copy RTC data to current hour
        current_hour_data.pv_energy_wh = rtc_accumulator.pv_energy_wh_accumulator;
        current_hour_data.battery_consumed_wh = rtc_accumulator.battery_energy_wh_accumulator;
        current_hour_data.timestamp_utc = rtc_accumulator.current_hour_start;

        // Check if hour has rolled over while we were asleep
        telemetry_check_hour_rollover();
    }

    telemetry_initialized = true;
    ESP_LOGI(TAG, "Telemetry initialization complete");
    return ESP_OK;
}

esp_err_t telemetry_update_power_sample(
    float pv_voltage, float pv_current, float pv_power,
    float battery_voltage, float battery_current, float battery_power,
    bool is_pv_active, bool is_battery_active)
{
    if (!telemetry_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xTelemetryMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Check for hour rollover
    telemetry_check_hour_rollover();

    // Calculate time delta
    int64_t current_time_us = esp_timer_get_time();
    if (last_sample_timestamp_us == 0) {
        last_sample_timestamp_us = current_time_us;
    }
    float time_delta_hours = (current_time_us - last_sample_timestamp_us) / 3600000000.0f;
    last_sample_timestamp_us = current_time_us;

    // Update PV metrics
    if (is_pv_active) {
        // Energy accumulation: Energy += Power * Time
        float pv_energy_delta = pv_power * time_delta_hours;
        rtc_accumulator.pv_energy_wh_accumulator += pv_energy_delta;
        current_hour_data.pv_energy_wh = rtc_accumulator.pv_energy_wh_accumulator;

        // Track peaks and accumulate for averages
        if (pv_power > current_hour_data.pv_peak_power_w) {
            current_hour_data.pv_peak_power_w = pv_power;
        }
        current_hour_data.pv_avg_power_w += pv_power;

        if (pv_voltage > current_hour_data.pv_max_voltage_v) {
            current_hour_data.pv_max_voltage_v = pv_voltage;
        }
        current_hour_data.pv_avg_voltage_v += pv_voltage;

        if (pv_current > current_hour_data.pv_max_current_a) {
            current_hour_data.pv_max_current_a = pv_current;
        }
        current_hour_data.pv_avg_current_a += pv_current;

        current_hour_data.pv_sample_count++;
    }

    // Update battery metrics
    if (is_battery_active) {
        // Energy accumulation
        float battery_energy_delta = battery_power * time_delta_hours;
        rtc_accumulator.battery_energy_wh_accumulator += battery_energy_delta;
        current_hour_data.battery_consumed_wh = rtc_accumulator.battery_energy_wh_accumulator;

        // Track peaks and accumulate for averages
        if (battery_power > current_hour_data.battery_peak_power_w) {
            current_hour_data.battery_peak_power_w = battery_power;
        }
        current_hour_data.battery_avg_power_w += battery_power;

        // Update voltage tracking
        current_hour_data.battery_voltage_v = battery_voltage;  // Latest reading
        if (current_hour_data.battery_sample_count == 0 ||
            battery_voltage < current_hour_data.battery_min_voltage_v) {
            current_hour_data.battery_min_voltage_v = battery_voltage;
        }
        if (battery_voltage > current_hour_data.battery_max_voltage_v) {
            current_hour_data.battery_max_voltage_v = battery_voltage;
        }

        if (battery_current > current_hour_data.battery_max_current_a) {
            current_hour_data.battery_max_current_a = battery_current;
        }
        current_hour_data.battery_avg_current_a += battery_current;

        current_hour_data.battery_sample_count++;
    }

    // Update RTC timestamp
    rtc_accumulator.last_sample_time = time(NULL);

    xSemaphoreGive(xTelemetryMutex);
    return ESP_OK;
}

bool telemetry_should_pv_be_active(float avg_light_level)
{
    // First check: astronomical calculations
    bool should_be_day = solar_calc_is_daytime();

    // Second check: photoresistor verification (threshold: 0.1V)
    const float LIGHT_THRESHOLD = 0.1f;
    bool light_detected = (avg_light_level > LIGHT_THRESHOLD);

    // Combined logic: both must agree (prevents false activation)
    return should_be_day && light_detected;
}

power_meter_state_t telemetry_get_recommended_state(bool any_bus_active, bool stellaria_only)
{
    if (!any_bus_active) {
        // No buses active = steady state
        return POWER_METER_STATE_STEADY;
    } else if (stellaria_only) {
        // Only Stellaria active = steady state (extrapolate)
        return POWER_METER_STATE_STEADY;
    } else {
        // Any other bus active = active monitoring
        return POWER_METER_STATE_ACTIVE;
    }
}

esp_err_t telemetry_get_current_hour_data(power_telemetry_hour_t *data)
{
    if (!telemetry_initialized || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xTelemetryMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    memcpy(data, &current_hour_data, sizeof(power_telemetry_hour_t));

    xSemaphoreGive(xTelemetryMutex);
    return ESP_OK;
}

esp_err_t telemetry_get_daily_summary(power_telemetry_day_t *summary)
{
    if (!telemetry_initialized || summary == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xTelemetryMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    memset(summary, 0, sizeof(power_telemetry_day_t));

    time_t now = time(NULL);
    struct tm now_tm;
    gmtime_r(&now, &now_tm);
    now_tm.tm_hour = 0;
    now_tm.tm_min = 0;
    now_tm.tm_sec = 0;
    summary->day_start_utc = mktime(&now_tm);

    // Sum up all hours from today
    for (uint16_t i = 0; i < buffer_valid_entries && i < TELEMETRY_ROLLING_BUFFER_HOURS; i++) {
        uint16_t index = (buffer_write_index - i - 1 + TELEMETRY_ROLLING_BUFFER_HOURS) % TELEMETRY_ROLLING_BUFFER_HOURS;
        power_telemetry_hour_t *hour = &historical_buffer[index];

        // Check if this hour is from today
        struct tm hour_tm;
        gmtime_r(&hour->timestamp_utc, &hour_tm);
        if (hour_tm.tm_yday == now_tm.tm_yday && hour_tm.tm_year == now_tm.tm_year) {
            summary->pv_total_energy_wh += hour->pv_energy_wh;
            summary->battery_total_consumed_wh += hour->battery_consumed_wh;

            if (hour->pv_peak_power_w > summary->pv_peak_power_w) {
                summary->pv_peak_power_w = hour->pv_peak_power_w;
            }
            if (hour->battery_peak_power_w > summary->battery_peak_power_w) {
                summary->battery_peak_power_w = hour->battery_peak_power_w;
            }

            summary->hours_collected++;
        }
    }

    xSemaphoreGive(xTelemetryMutex);
    return ESP_OK;
}

esp_err_t telemetry_get_historical_hour(uint16_t hours_ago, power_telemetry_hour_t *data)
{
    if (!telemetry_initialized || data == NULL || hours_ago >= TELEMETRY_ROLLING_BUFFER_HOURS) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xTelemetryMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    if (hours_ago >= buffer_valid_entries) {
        xSemaphoreGive(xTelemetryMutex);
        return ESP_ERR_NOT_FOUND;
    }

    uint16_t index = (buffer_write_index - hours_ago - 1 + TELEMETRY_ROLLING_BUFFER_HOURS) %
                     TELEMETRY_ROLLING_BUFFER_HOURS;
    memcpy(data, &historical_buffer[index], sizeof(power_telemetry_hour_t));

    xSemaphoreGive(xTelemetryMutex);
    return ESP_OK;
}

esp_err_t telemetry_mqtt_publish_daily(void)
{
    ESP_LOGW(TAG, "MQTT publish_daily called - PLACEHOLDER, not yet implemented");
    // TODO: Implement MQTT publishing
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t telemetry_mqtt_publish_hourly(void)
{
    ESP_LOGW(TAG, "MQTT publish_hourly called - PLACEHOLDER, not yet implemented");
    // TODO: Implement MQTT publishing
    return ESP_ERR_NOT_SUPPORTED;
}

void telemetry_print_power_status(void)
{
    if (!telemetry_initialized) {
        ESP_LOGW(TAG, "Telemetry not initialized");
        return;
    }

    power_telemetry_hour_t current;
    if (telemetry_get_current_hour_data(&current) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get current hour data");
        return;
    }

    ESP_LOGI(TAG, "=== Current Hour Power Status ===");
    ESP_LOGI(TAG, "PV Generation:");
    ESP_LOGI(TAG, "  Energy: %.2f Wh", current.pv_energy_wh);
    ESP_LOGI(TAG, "  Peak Power: %.2f W", current.pv_peak_power_w);
    ESP_LOGI(TAG, "  Avg Power: %.2f W", current.pv_avg_power_w);
    ESP_LOGI(TAG, "  Voltage: %.2f V (max: %.2f V)", current.pv_avg_voltage_v, current.pv_max_voltage_v);
    ESP_LOGI(TAG, "  Current: %.3f A (max: %.3f A)", current.pv_avg_current_a, current.pv_max_current_a);
    ESP_LOGI(TAG, "  Samples: %d", current.pv_sample_count);

    ESP_LOGI(TAG, "Battery Consumption:");
    ESP_LOGI(TAG, "  Energy: %.2f Wh", current.battery_consumed_wh);
    ESP_LOGI(TAG, "  Peak Power: %.2f W", current.battery_peak_power_w);
    ESP_LOGI(TAG, "  Avg Power: %.2f W", current.battery_avg_power_w);
    ESP_LOGI(TAG, "  Voltage: %.2f V (min: %.2f, max: %.2f V)",
             current.battery_voltage_v, current.battery_min_voltage_v, current.battery_max_voltage_v);
    ESP_LOGI(TAG, "  Current: %.3f A (max: %.3f A)", current.battery_avg_current_a, current.battery_max_current_a);
    ESP_LOGI(TAG, "  Samples: %d", current.battery_sample_count);
}

void telemetry_print_daily_summary(void)
{
    if (!telemetry_initialized) {
        ESP_LOGW(TAG, "Telemetry not initialized");
        return;
    }

    power_telemetry_day_t summary;
    if (telemetry_get_daily_summary(&summary) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get daily summary");
        return;
    }

    ESP_LOGI(TAG, "=== Today's Power Summary ===");
    ESP_LOGI(TAG, "PV Generated: %.2f Wh (%.3f kWh)",
             summary.pv_total_energy_wh, summary.pv_total_energy_wh / 1000.0f);
    ESP_LOGI(TAG, "Battery Consumed: %.2f Wh (%.3f kWh)",
             summary.battery_total_consumed_wh, summary.battery_total_consumed_wh / 1000.0f);
    ESP_LOGI(TAG, "PV Peak Power: %.2f W", summary.pv_peak_power_w);
    ESP_LOGI(TAG, "Battery Peak Power: %.2f W", summary.battery_peak_power_w);
    ESP_LOGI(TAG, "Hours Collected: %d/24", summary.hours_collected);

    float net_energy = summary.pv_total_energy_wh - summary.battery_total_consumed_wh;
    ESP_LOGI(TAG, "Net Energy: %.2f Wh %s",
             fabs(net_energy), net_energy >= 0 ? "(surplus)" : "(deficit)");
}

// ################ Component Data Hub (Central Storage) ################

// Static storage for component data caches
static telemetry_stellaria_t cached_stellaria_data = {0};
static telemetry_fluctus_t cached_fluctus_data = {0};
static telemetry_tempesta_t cached_tempesta_data = {0};
static telemetry_impluvium_t cached_impluvium_data = {0};

// ################ Component Update Functions ################

/**
 * @brief Update STELLARIA data in TELEMETRY cache
 * Called by STELLARIA component (injection point)
 */
esp_err_t telemetry_update_stellaria(void)
{
    if (!telemetry_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Get snapshot from STELLARIA component
    stellaria_snapshot_t stellaria_snapshot;
    esp_err_t ret = stellaria_get_data_snapshot(&stellaria_snapshot);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get STELLARIA snapshot: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update cache with mutex protection
    if (xSemaphoreTake(xTelemetryMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Copy snapshot to TELEMETRY cache
    cached_stellaria_data.state = stellaria_snapshot.state;
    cached_stellaria_data.current_intensity = stellaria_snapshot.current_intensity;
    cached_stellaria_data.target_intensity = stellaria_snapshot.target_intensity;
    cached_stellaria_data.driver_enabled = stellaria_snapshot.driver_enabled;
    cached_stellaria_data.initialized = stellaria_snapshot.initialized;
    cached_stellaria_data.auto_mode_active = stellaria_snapshot.auto_mode_active;
    cached_stellaria_data.last_light_reading = stellaria_snapshot.last_light_reading;
    cached_stellaria_data.last_update_time = time(NULL);
    cached_stellaria_data.data_valid = true;

    xSemaphoreGive(xTelemetryMutex);

    ESP_LOGD(TAG, "Updated STELLARIA cache: intensity=%d, enabled=%d",
             cached_stellaria_data.current_intensity, cached_stellaria_data.driver_enabled);

    return ESP_OK;
}

// ################ Component Get Functions ################

/**
 * @brief Get cached STELLARIA data for HMI display
 */
esp_err_t telemetry_get_stellaria_data(telemetry_stellaria_t *data)
{
    if (!telemetry_initialized || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xTelemetryMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    memcpy(data, &cached_stellaria_data, sizeof(telemetry_stellaria_t));

    xSemaphoreGive(xTelemetryMutex);

    return ESP_OK;
}

/**
 * @brief Update FLUCTUS state data in TELEMETRY cache
 * Called by FLUCTUS component (injection point)
 */
esp_err_t telemetry_update_fluctus_state(void)
{
    if (!telemetry_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Get snapshot from FLUCTUS component
    fluctus_snapshot_t fluctus_snapshot;
    esp_err_t ret = fluctus_get_data_snapshot(&fluctus_snapshot);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get FLUCTUS snapshot: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update cache with mutex protection
    if (xSemaphoreTake(xTelemetryMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Copy snapshot to TELEMETRY cache
    cached_fluctus_data.bus_3v3_enabled = fluctus_snapshot.bus_3v3_enabled;
    cached_fluctus_data.bus_5v_enabled = fluctus_snapshot.bus_5v_enabled;
    cached_fluctus_data.bus_6v6_enabled = fluctus_snapshot.bus_6v6_enabled;
    cached_fluctus_data.bus_12v_enabled = fluctus_snapshot.bus_12v_enabled;
    cached_fluctus_data.bus_3v3_consumers = fluctus_snapshot.bus_3v3_consumers;
    cached_fluctus_data.bus_5v_consumers = fluctus_snapshot.bus_5v_consumers;
    cached_fluctus_data.bus_6v6_consumers = fluctus_snapshot.bus_6v6_consumers;
    cached_fluctus_data.bus_12v_consumers = fluctus_snapshot.bus_12v_consumers;

    cached_fluctus_data.battery_voltage = fluctus_snapshot.battery_voltage;
    cached_fluctus_data.battery_current = fluctus_snapshot.battery_current;
    cached_fluctus_data.battery_soc_percent = fluctus_snapshot.battery_soc_percent;
    cached_fluctus_data.battery_power_w = fluctus_snapshot.battery_power_w;

    cached_fluctus_data.solar_voltage = fluctus_snapshot.solar_voltage;
    cached_fluctus_data.solar_current = fluctus_snapshot.solar_current;
    cached_fluctus_data.solar_power_w = fluctus_snapshot.solar_power_w;
    cached_fluctus_data.solar_pv_active = fluctus_snapshot.solar_pv_active;

    cached_fluctus_data.tracking_state = fluctus_snapshot.tracking_state;
    cached_fluctus_data.yaw_position_percent = fluctus_snapshot.yaw_position_percent;
    cached_fluctus_data.pitch_position_percent = fluctus_snapshot.pitch_position_percent;
    cached_fluctus_data.yaw_error = fluctus_snapshot.yaw_error;
    cached_fluctus_data.pitch_error = fluctus_snapshot.pitch_error;

    cached_fluctus_data.case_temperature = fluctus_snapshot.case_temperature;
    cached_fluctus_data.temperature_valid = fluctus_snapshot.temperature_valid;
    cached_fluctus_data.fan_speed_percent = fluctus_snapshot.fan_speed_percent;

    cached_fluctus_data.power_state = fluctus_snapshot.power_state;
    cached_fluctus_data.safety_shutdown = fluctus_snapshot.safety_shutdown;
    cached_fluctus_data.manual_reset_required = fluctus_snapshot.manual_reset_required;
    cached_fluctus_data.last_activity_time = fluctus_snapshot.last_activity_time;

    cached_fluctus_data.battery_data_valid = fluctus_snapshot.battery_data_valid;
    cached_fluctus_data.solar_data_valid = fluctus_snapshot.solar_data_valid;

    cached_fluctus_data.last_update_time = time(NULL);
    cached_fluctus_data.data_valid = true;

    xSemaphoreGive(xTelemetryMutex);

    ESP_LOGD(TAG, "Updated FLUCTUS cache: battery=%.2fV (%.0f%% SOC), solar=%.2fW",
             cached_fluctus_data.battery_voltage, cached_fluctus_data.battery_soc_percent,
             cached_fluctus_data.solar_power_w);

    return ESP_OK;
}

/**
 * @brief Get cached FLUCTUS data for HMI display
 */
esp_err_t telemetry_get_fluctus_data(telemetry_fluctus_t *data)
{
    if (!telemetry_initialized || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xTelemetryMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    memcpy(data, &cached_fluctus_data, sizeof(telemetry_fluctus_t));

    xSemaphoreGive(xTelemetryMutex);

    return ESP_OK;
}

/**
 * @brief Update TEMPESTA data in TELEMETRY cache
 * Called by TEMPESTA component (injection point)
 */
esp_err_t telemetry_update_tempesta(void)
{
    if (!telemetry_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Get snapshot from TEMPESTA component
    tempesta_snapshot_t tempesta_snapshot;
    esp_err_t ret = tempesta_get_data_snapshot(&tempesta_snapshot);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get TEMPESTA snapshot: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update cache with mutex protection
    if (xSemaphoreTake(xTelemetryMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Copy snapshot to TELEMETRY cache
    cached_tempesta_data.temperature = tempesta_snapshot.temperature;
    cached_tempesta_data.humidity = tempesta_snapshot.humidity;
    cached_tempesta_data.pressure = tempesta_snapshot.pressure;
    cached_tempesta_data.air_quality_pm25 = tempesta_snapshot.air_quality_pm25;
    cached_tempesta_data.air_quality_pm10 = tempesta_snapshot.air_quality_pm10;
    cached_tempesta_data.wind_speed_rpm = tempesta_snapshot.wind_speed_rpm;
    cached_tempesta_data.wind_speed_ms = tempesta_snapshot.wind_speed_ms;
    cached_tempesta_data.rainfall_mm = tempesta_snapshot.rainfall_mm;

    cached_tempesta_data.temp_sensor_status = tempesta_snapshot.temp_sensor_status;
    cached_tempesta_data.humidity_sensor_status = tempesta_snapshot.humidity_sensor_status;
    cached_tempesta_data.pressure_sensor_status = tempesta_snapshot.pressure_sensor_status;
    cached_tempesta_data.air_quality_status = tempesta_snapshot.air_quality_status;
    cached_tempesta_data.wind_sensor_status = tempesta_snapshot.wind_sensor_status;
    cached_tempesta_data.rain_sensor_status = tempesta_snapshot.rain_sensor_status;

    cached_tempesta_data.last_update_time = time(NULL);
    cached_tempesta_data.data_valid = true;

    xSemaphoreGive(xTelemetryMutex);

    ESP_LOGD(TAG, "Updated TEMPESTA cache: temp=%.1f°C, humidity=%.0f%%, pressure=%.1fhPa",
             cached_tempesta_data.temperature, cached_tempesta_data.humidity, cached_tempesta_data.pressure);

    return ESP_OK;
}

/**
 * @brief Get cached TEMPESTA data for HMI display
 */
esp_err_t telemetry_get_tempesta_data(telemetry_tempesta_t *data)
{
    if (!telemetry_initialized || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xTelemetryMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    memcpy(data, &cached_tempesta_data, sizeof(telemetry_tempesta_t));

    xSemaphoreGive(xTelemetryMutex);

    return ESP_OK;
}

/**
 * @brief Update IMPLUVIUM data in TELEMETRY cache
 * Called by IMPLUVIUM component (injection point)
 */
esp_err_t telemetry_update_impluvium(void)
{
    if (!telemetry_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Get snapshot from IMPLUVIUM component
    impluvium_snapshot_t impluvium_snapshot;
    esp_err_t ret = impluvium_get_data_snapshot(&impluvium_snapshot);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get IMPLUVIUM snapshot: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update cache with mutex protection
    if (xSemaphoreTake(xTelemetryMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Copy snapshot to TELEMETRY cache (simplified for HMI)
    cached_impluvium_data.state = impluvium_snapshot.state;
    cached_impluvium_data.system_initialized = impluvium_snapshot.system_initialized;
    cached_impluvium_data.monitoring_active = impluvium_snapshot.monitoring_active;

    cached_impluvium_data.water_level_percent = impluvium_snapshot.water_level_percent;
    cached_impluvium_data.tank_pressure_kpa = impluvium_snapshot.tank_pressure_kpa;

    cached_impluvium_data.active_zone_id = impluvium_snapshot.active_zone_id;
    cached_impluvium_data.pump_pwm_duty = impluvium_snapshot.pump_pwm_duty;
    cached_impluvium_data.current_flow_rate_mlpm = impluvium_snapshot.current_flow_rate_mlpm;

    cached_impluvium_data.total_volume_used_today_ml = impluvium_snapshot.total_volume_used_today_ml;

    // Copy per-zone essentials (simplified)
    for (int i = 0; i < 4; i++) {
        cached_impluvium_data.zones[i].current_moisture_percent =
            impluvium_snapshot.zones[i].current_moisture_percent;
        cached_impluvium_data.zones[i].watering_enabled =
            impluvium_snapshot.zones[i].watering_enabled;
        cached_impluvium_data.zones[i].last_watered_time =
            impluvium_snapshot.zones[i].last_watered_time;
    }

    cached_impluvium_data.current_anomaly_type = impluvium_snapshot.current_anomaly_type;

    cached_impluvium_data.last_update_time = time(NULL);
    cached_impluvium_data.data_valid = true;

    xSemaphoreGive(xTelemetryMutex);

    ESP_LOGD(TAG, "Updated IMPLUVIUM cache: state=%d, water_level=%.1f%%, active_zone=%d",
             cached_impluvium_data.state, cached_impluvium_data.water_level_percent,
             cached_impluvium_data.active_zone_id);

    return ESP_OK;
}

/**
 * @brief Get cached IMPLUVIUM data for HMI display
 */
esp_err_t telemetry_get_impluvium_data(telemetry_impluvium_t *data)
{
    if (!telemetry_initialized || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xTelemetryMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    memcpy(data, &cached_impluvium_data, sizeof(telemetry_impluvium_t));

    xSemaphoreGive(xTelemetryMutex);

    return ESP_OK;
}
