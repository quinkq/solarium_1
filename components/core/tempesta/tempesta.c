/**
 * @file weather_station.c
 * @brief TEMPESTA - Multi-sensor weather station with power optimization
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Original implementation of TEMPESTA - comprehensive environmental monitoring.
 *
 * Key features:
 * - Eight-sensor array: temperature, humidity, pressure, wind, air quality, rainfall
 * - Power-aware polling (15min normal / 60min power-save)
 * - Sensor-level gating (hall array 10ms per cycle, PMS5003 UART sleep)
 * - Per-sensor status tracking (OK/UNAVAILABLE/ERROR/WARMING_UP)
 * - Temperature data sharing with IMPLUVIUM for watering correction
 * - AS5600 wind speed with low-power mode (5mA savings)
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "tempesta.h"
#include "tempesta_private.h"
#include "telemetry.h"
#include "solar_calc.h"
#include "interval_config.h"
#include "main.h"

#include <string.h>

// ########################## Global Variables ##########################

static const char *TAG = "TEMPESTA";

// Thread-safe data protection
SemaphoreHandle_t xTempestaDataMutex = NULL;
tempesta_snapshot_t weather_data = {0};
weather_calculation_data_t calculation_data = {0};

// State machine
tempesta_state_t tempesta_state = TEMPESTA_STATE_DISABLED;  // Current operational state
tempesta_state_t tempesta_previous_state = TEMPESTA_STATE_DISABLED;  // Previous state (for SHUTDOWN recovery)

// Power and hardware configuration
bool power_save_mode = false;                 // Power save mode flag (60min vs 15min cycle)
bool previous_power_save_mode = false;        // Previous power save mode (for SHUTDOWN recovery)
bool weather_pms5003_enabled = true;          // PMS5003 air quality sensor hardware enabled

// Interval configuration (loaded from interval_config at init)
uint32_t tempesta_normal_interval_ms;         // Normal mode collection interval (from config)
uint32_t tempesta_power_save_interval_ms;     // Power save mode collection interval (from config)

// Task and timer handles
TaskHandle_t xTempestaMainTaskHandle = NULL;
TaskHandle_t xTempestaAS5600TaskHandle = NULL;
TimerHandle_t xTempestaCollectionTimer = NULL;

// Sensor device handles
sht4x_t sht4x_dev = {0};
bmp280_t bmp280_dev = {0};
as5600_dev_t as5600_dev = {0};

// Hardware handles
pcnt_unit_handle_t rain_pcnt_unit_handle = NULL;
pcnt_unit_handle_t tank_intake_pcnt_unit_handle = NULL;

// ########################## Private Function Declarations ##########################

// State machine functions
static void tempesta_change_state(tempesta_state_t new_state);
// Power management functions
static esp_err_t tempesta_request_power_buses(bool requires_5v);
static void tempesta_release_power_buses(bool had_5v);
// Utility functions
static void tempesta_update_timer_period(void);
static void tempesta_timer_callback(TimerHandle_t xTimer);
//static void tempesta_log_summary(void);
// Task functions
static void tempesta_main_task(void *pvParameters);

// ########################## Initialization ##########################

/**
 * @brief Initialize weather station component
 */
esp_err_t tempesta_station_init(void)
{
    ESP_LOGI(TAG, "Initializing weather station component...");

    // Create mutex for thread-safe data access
    xTempestaDataMutex = xSemaphoreCreateMutex();
    if (xTempestaDataMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create weather data mutex");
        return ESP_FAIL;
    }

    // Initialize weather data with invalid values
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
        weather_data.temperature = WEATHER_INVALID_VALUE;
        weather_data.humidity = WEATHER_INVALID_VALUE;
        weather_data.pressure = WEATHER_INVALID_VALUE;
        weather_data.air_quality_pm25 = WEATHER_INVALID_VALUE;
        weather_data.air_quality_pm10 = WEATHER_INVALID_VALUE;
        weather_data.wind_speed_rpm = WEATHER_INVALID_VALUE;
        weather_data.wind_speed_ms = WEATHER_INVALID_VALUE;
        weather_data.wind_direction_deg = WEATHER_INVALID_VALUE;
        weather_data.wind_direction_cardinal = "---";
        weather_data.rainfall_last_hour_mm = 0.0f;
        weather_data.rainfall_current_hour_mm = 0.0f;
        weather_data.rainfall_daily_mm = 0.0f;
        weather_data.rainfall_weekly_mm = 0.0f;
        weather_data.tank_intake_last_hour_ml = 0.0f;
        weather_data.tank_intake_current_hour_ml = 0.0f;
        weather_data.tank_intake_daily_ml = 0.0f;
        weather_data.tank_intake_weekly_ml = 0.0f;
        weather_data.snapshot_timestamp = 0;

        // Initialize state machine and flags
        weather_data.state = TEMPESTA_STATE_DISABLED;
        weather_data.power_save_mode = false;

        // Initialize all sensor statuses as unavailable
        weather_data.temp_sensor_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.humidity_sensor_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.pressure_sensor_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.air_quality_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.wind_sensor_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.wind_direction_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.rain_gauge_status = WEATHER_SENSOR_UNAVAILABLE;
        weather_data.tank_intake_status = WEATHER_SENSOR_UNAVAILABLE;

        xSemaphoreGive(xTempestaDataMutex);
    } else {
        ESP_LOGE(TAG, "Failed to initialize weather data - mutex timeout");
        return ESP_FAIL;
    }

    // Initialize humidity averaging data
    for (int i = 0; i < WEATHER_AVERAGING_SAMPLES; i++) {
        calculation_data.humidity_history[i] = WEATHER_INVALID_VALUE;
    }
    calculation_data.humidity_history_index = 0;
    calculation_data.humidity_history_count = 0;

    // Initialize temperature averaging data
    for (int i = 0; i < WEATHER_AVERAGING_SAMPLES; i++) {
        calculation_data.temperature_history[i] = WEATHER_INVALID_VALUE;
    }
    calculation_data.temp_history_index = 0;
    calculation_data.temp_history_count = 0;

    // Initialize rainfall tracking (using monotonic time)
    int64_t current_time_ms = esp_timer_get_time() / 1000;
    calculation_data.rainfall_hourly_start_time_ms = current_time_ms;
    calculation_data.rainfall_hourly_start_pulses = 0;
    calculation_data.rainfall_daily_base_pulses = 0;
    calculation_data.rainfall_weekly_base_pulses = 0;

    // Initialize tank intake tracking (using monotonic time)
    calculation_data.tank_intake_hourly_start_time_ms = current_time_ms;
    calculation_data.tank_intake_hourly_start_pulses = 0;
    calculation_data.tank_intake_daily_base_pulses = 0;
    calculation_data.tank_intake_weekly_base_pulses = 0;

    // Initialize hardware and sensors
    esp_err_t ret = tempesta_hardware_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Hardware initialization failed: %s", esp_err_to_name(ret));
        goto cleanup_mutex;
    }

    // Request 3.3V bus power for I2C sensor initialization
    ESP_LOGI(TAG, "Requesting 3.3V bus power for I2C sensor initialization");
    ret = fluctus_request_bus_power(POWER_BUS_3V3, "TEMPESTA_INIT");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to request 3.3V bus power: %s - continuing without I2C sensors", esp_err_to_name(ret));
    } else {
        vTaskDelay(pdMS_TO_TICKS(100));  // Allow bus to stabilize

        ret = tempesta_i2c_sensors_init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Some sensors failed to initialize: %s", esp_err_to_name(ret));
            // Continue anyway - individual sensors will be marked as unavailable
        }

        // Release init power - runtime operations will request as needed
        fluctus_release_bus_power(POWER_BUS_3V3, "TEMPESTA_INIT");
        ESP_LOGI(TAG, "Released 3.3V bus power (init complete)");
    }

    // Register midnight callback for daily reset
    ret = solar_calc_register_midnight_callback(tempesta_daily_reset_callback);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register midnight callback: %s", esp_err_to_name(ret));
        // Continue anyway - daily reset won't work but hourly tracking will
    }

    // Load interval configuration from global config
    tempesta_normal_interval_ms = g_interval_config.tempesta_normal_min * 60 * 1000;
    tempesta_power_save_interval_ms = g_interval_config.tempesta_power_save_min * 60 * 1000;
    ESP_LOGI(TAG, "Collection intervals: %lums (normal), %lums (power save)",
             tempesta_normal_interval_ms, tempesta_power_save_interval_ms);

    // Create collection timer - auto-reload repeating timer
    xTempestaCollectionTimer = xTimerCreate("WeatherTimer",
                                           pdMS_TO_TICKS(tempesta_normal_interval_ms),
                                           pdTRUE, // Auto-reload
                                           (void *) 0,
                                           tempesta_timer_callback);
    if (xTempestaCollectionTimer == NULL) {
        ESP_LOGE(TAG, "Failed to create collection timer");
        goto cleanup_mutex;
    }

    // Create main weather task
    BaseType_t xResult = xTaskCreate(tempesta_main_task,
                                     "WeatherMain",
                                     2048, // Reduced stack - mostly I2C operations
                                     NULL,
                                     5,
                                     &xTempestaMainTaskHandle);
    if (xResult != pdPASS) {
        ESP_LOGE(TAG, "Failed to create main weather task");
        goto cleanup_timer;
    }

    // Create AS5600 sampling task
    xResult = xTaskCreate(tempesta_as5600_sampling_task,
                          "WeatherAS5600",
                          2048, // Reduced stack - simple calculations
                          NULL,
                          6, // Higher priority for time-sensitive wind measurements
                          &xTempestaAS5600TaskHandle);
    if (xResult != pdPASS) {
        ESP_LOGE(TAG, "Failed to create AS5600 sampling task");
        goto cleanup_main_task;
    }

    // Start collection timer (uses dynamic interval configuration)
    if (xTimerStart(xTempestaCollectionTimer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start collection timer");
        goto cleanup_as5600_task;
    }

    ESP_LOGI(TAG, "Weather station initialized successfully");
    return ESP_OK;

    // Cleanup on failure
cleanup_as5600_task:
    if (xTempestaAS5600TaskHandle) {
        vTaskDelete(xTempestaAS5600TaskHandle);
        xTempestaAS5600TaskHandle = NULL;
    }
cleanup_main_task:
    if (xTempestaMainTaskHandle) {
        vTaskDelete(xTempestaMainTaskHandle);
        xTempestaMainTaskHandle = NULL;
    }
cleanup_timer:
    if (xTempestaCollectionTimer) {
        xTimerDelete(xTempestaCollectionTimer, portMAX_DELAY);
        xTempestaCollectionTimer = NULL;
    }
cleanup_mutex:
    if (xTempestaDataMutex) {
        vSemaphoreDelete(xTempestaDataMutex);
        xTempestaDataMutex = NULL;
    }
    return ESP_FAIL;
}
// ########################## State Machine ##########################

/**
 * @brief Change TEMPESTA operational state with logging
 * @param new_state Target state to transition to
 * @note This function is not thread-safe - caller must hold xTempestaDataMutex if needed
 */
static void tempesta_change_state(tempesta_state_t new_state)
{
    if (tempesta_state == new_state) {
        return;  // No change needed
    }

    // Log state transition
    const char *old_state_str = "UNKNOWN";
    const char *new_state_str = "UNKNOWN";

    switch (tempesta_state) {
        case TEMPESTA_STATE_DISABLED:   old_state_str = "DISABLED"; break;
        case TEMPESTA_STATE_IDLE:       old_state_str = "IDLE"; break;
        case TEMPESTA_STATE_READING:    old_state_str = "READING"; break;
        case TEMPESTA_STATE_SHUTDOWN:   old_state_str = "SHUTDOWN"; break;
        case TEMPESTA_STATE_ERROR:      old_state_str = "ERROR"; break;
    }

    switch (new_state) {
        case TEMPESTA_STATE_DISABLED:   new_state_str = "DISABLED"; break;
        case TEMPESTA_STATE_IDLE:       new_state_str = "IDLE"; break;
        case TEMPESTA_STATE_READING:    new_state_str = "READING"; break;
        case TEMPESTA_STATE_SHUTDOWN:   new_state_str = "SHUTDOWN"; break;
        case TEMPESTA_STATE_ERROR:      new_state_str = "ERROR"; break;
    }

    ESP_LOGI(TAG, "State transition: %s -> %s", old_state_str, new_state_str);

    // Save previous state (for SHUTDOWN recovery)
    if (new_state == TEMPESTA_STATE_SHUTDOWN) {
        tempesta_previous_state = tempesta_state;
    }

    tempesta_state = new_state;

    // Update state in snapshot (for telemetry/HMI)
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        weather_data.state = new_state;
        xSemaphoreGive(xTempestaDataMutex);
    }
}

// ============================== ERROR STATE TODO ==============================
/*
 * TODO: Implement TEMPESTA_STATE_ERROR handling
 *
 * REQUIREMENTS:
 * 1. Transition to ERROR state when sensor failures exceed threshold
 *    - Count consecutive read failures per sensor
 *    - Trigger ERROR state when multiple critical sensors fail (e.g., 3+ sensors)
 *    - Continue reading remaining functional sensors in ERROR state
 *
 * 2. ERROR state behavior:
 *    - Follow normal reading cycles (15min/60min) with operational sensors
 *    - Mark failed sensors as WEATHER_SENSOR_ERROR in snapshot
 *    - Log detailed error information for diagnostics
 *    - Display ERROR state prominently in HMI
 *
 * 3. Recovery mechanism:
 *    - Require user confirmation via HMI or MQTT to exit ERROR state
 *    - Add tempesta_clear_error() function (HMI button handler)
 *    - Transition ERROR → IDLE on user confirmation
 *    - Reset sensor failure counters on recovery
 *
 * 4. TELEMETRY integration:
 *    - ERROR state transitions trigger immediate injection
 *    - Include failure masks/counts in ERROR state snapshot
 *
 * IMPLEMENTATION LOCATIONS:
 * - Sensor reading functions: Add failure counting logic
 * - Main task: Check failure threshold, transition to ERROR
 * - HMI: Add "Clear Error" button in TEMPESTA controls
 * - Add tempesta_clear_error() public function in header
 *
 * SAFETY: System remains partially operational in ERROR state - does not disable
 * all monitoring. User must explicitly acknowledge sensor issues.
 */
// ==============================================================================

// ########################## Power Management Functions ##########################

/**
 * @brief Unified power management for weather station based on sensor requirements
 * @param requires_5v Whether 5V bus is needed (for PMS5003)
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t tempesta_request_power_buses(bool requires_5v)
{
    // Always request 3V3 for basic sensors
    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_3V3, "TEMPESTA", return ESP_FAIL);

    // Request 5V if needed (decision made at task level)
    if (requires_5v) {
        FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_5V, "TEMPESTA", {
            fluctus_release_bus_power(POWER_BUS_3V3, "TEMPESTA");
            return ESP_FAIL;
        });
    }

    return ESP_OK;
}

/**
 * @brief Release power buses based on what was requested
 * @param had_5v Whether 5V bus was previously requested
 */
static void tempesta_release_power_buses(bool had_5v)
{
    if (had_5v) {
        fluctus_release_bus_power(POWER_BUS_5V, "TEMPESTA");
    }
    fluctus_release_bus_power(POWER_BUS_3V3, "TEMPESTA");
}

/**
 * @brief Helper function to update timer period based on power save mode flag
 */
static void tempesta_update_timer_period(void)
{
    if (xTempestaCollectionTimer != NULL) {
        TickType_t new_period = power_save_mode ?
            pdMS_TO_TICKS(tempesta_power_save_interval_ms) :
            pdMS_TO_TICKS(tempesta_normal_interval_ms);

        xTimerChangePeriod(xTempestaCollectionTimer, new_period, portMAX_DELAY);
    }
}

// ########################## Task Helper Functions ##########################

/**
 * @brief Log comprehensive weather summary with sensor status
 */
/*
static void tempesta_log_summary(void)
{
    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        ESP_LOGI(TAG,
                 "Weather: T=%.1f°C, H=%.1f%%, P=%.1f hPa, PM2.5=%.0f µg/m³",
                 weather_data.temperature,
                 weather_data.humidity,
                 weather_data.pressure,
                 weather_data.air_quality_pm25);

        ESP_LOGI(TAG,
                 "Wind: Speed=%.1f RPM (%.1f m/s), Direction=%.1f° (%s)",
                 weather_data.wind_speed_rpm,
                 weather_data.wind_speed_ms,
                 weather_data.wind_direction_deg,
                 weather_data.wind_direction_cardinal);

        ESP_LOGI(TAG,
                 "Rain: Last=%.2f mm, Current=%.2f mm, Daily=%.2f mm, Weekly=%.2f mm",
                 weather_data.rainfall_last_hour_mm,
                 weather_data.rainfall_current_hour_mm,
                 weather_data.rainfall_daily_mm,
                 weather_data.rainfall_weekly_mm);

        ESP_LOGI(TAG,
                 "Tank: Last=%.1f mL, Current=%.1f mL, Daily=%.1f mL, Weekly=%.1f mL",
                 weather_data.tank_intake_last_hour_ml,
                 weather_data.tank_intake_current_hour_ml,
                 weather_data.tank_intake_daily_ml,
                 weather_data.tank_intake_weekly_ml);

        if (weather_data.temp_sensor_status != WEATHER_SENSOR_OK ||
            weather_data.humidity_sensor_status != WEATHER_SENSOR_OK ||
            weather_data.pressure_sensor_status != WEATHER_SENSOR_OK ||
            weather_data.air_quality_status != WEATHER_SENSOR_OK ||
            weather_data.wind_sensor_status != WEATHER_SENSOR_OK ||
            weather_data.wind_direction_status != WEATHER_SENSOR_OK ||
            weather_data.rain_gauge_status != WEATHER_SENSOR_OK ||
            weather_data.tank_intake_status != WEATHER_SENSOR_OK) {
            ESP_LOGW(TAG, "Weather station is not fully operational, check sensor status");
        }
        xSemaphoreGive(xTempestaDataMutex);
    }
}
*/

// ########################## Utility Functions ##########################

/**
 * @brief Timer callback to trigger data collection cycle
 */
static void tempesta_timer_callback(TimerHandle_t xTimer)
{
    // Notify main task to start collection cycle
    if (xTempestaMainTaskHandle != NULL) {
        xTaskNotify(xTempestaMainTaskHandle, 1, eSetBits);
    }
}

// ########################## Task Functions ##########################

/**
 * @brief Main weather station coordination task
 */
static void tempesta_main_task(void *pvParameters)
{
    const char *task_tag = "WeatherMain";
    ESP_LOGI(task_tag, "Weather station main task started");

    // Wait for system initialization
    vTaskDelay(pdMS_TO_TICKS(3000));

    while (1) {
        // Wait for collection cycle notification
        uint32_t notification_value = 0;
        xTaskNotifyWait(0x00, ULONG_MAX, &notification_value, portMAX_DELAY);

        // Check if collection should proceed based on state
        if (tempesta_state == TEMPESTA_STATE_DISABLED ||
            tempesta_state == TEMPESTA_STATE_SHUTDOWN ||
            tempesta_state == TEMPESTA_STATE_ERROR) {
            ESP_LOGD(task_tag, "System not operational (state %d) - skipping weather collection", tempesta_state);
            continue;
        }
        // Safety check: Don't collect if FLUCTUS is in CRITICAL (last line of defense)
        fluctus_power_state_t power_state = fluctus_get_power_state();
        if (power_state == FLUCTUS_POWER_STATE_CRITICAL) {
            ESP_LOGW(task_tag, "Critical power state - skipping weather collection cycle");
            continue;
        }

        // Save current state to restore after reading
        tempesta_state_t return_state = tempesta_state;

        // Transition to READING state
        tempesta_change_state(TEMPESTA_STATE_READING);

        ESP_LOGI(task_tag, "Starting weather data collection cycle");

        // Determine if PMS5003 should be read this cycle (FLUCTUS controls via enable flag)
        bool should_read_pms5003 = weather_pms5003_enabled;
        esp_err_t ret = tempesta_request_power_buses(should_read_pms5003);
        if (ret != ESP_OK) {
            ESP_LOGE(task_tag, "Failed to request power buses");
            tempesta_change_state(return_state);  // Restore previous state on error
            continue;
        }

        ESP_LOGI(task_tag, "Sensor buses powered (%s), starting measurements",
                 should_read_pms5003 ? "3V3+5V" : "3V3 only");

        // Record start time for PMS5003 warmup tracking and send wake command
        TickType_t warmup_start_time = xTaskGetTickCount();
        if (should_read_pms5003) {
            tempesta_pms5003_send_wake_command();
            ESP_LOGI(task_tag, "PMS5003 wake command sent, warmup starting");
        }

        // Allow I2C sensors brief stabilization and trigger wind sampling
        vTaskDelay(pdMS_TO_TICKS(1000));
        xTaskNotify(xTempestaAS5600TaskHandle, 1, eSetBits);

        // Read and process essential I2C sensors
        consolidated_sensor_data_t sensor_data;
        tempesta_read_env_sensors(&sensor_data);
        tempesta_process_temperature(&sensor_data);
        tempesta_process_humidity(&sensor_data);
        tempesta_process_pressure(&sensor_data);

        // Read pulse counter sensors (rainfall and tank intake)
        tempesta_read_and_process_rainfall();
        tempesta_read_and_process_tank_intake();

        // Read wind direction from Hall sensor array (ADS1115)
        tempesta_read_and_process_wind_direction();

        // Handle PMS5003 air quality sensor (with warmup management)
        tempesta_handle_pms5003_reading(warmup_start_time, should_read_pms5003);

        // Update timestamp and power down
        if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_UPDATE_MS)) == pdTRUE) {
            weather_data.snapshot_timestamp = time(NULL);
            xSemaphoreGive(xTempestaDataMutex);
        }

        // Inject data to TELEMETRY central hub
        telemetry_fetch_snapshot(TELEMETRY_SRC_TEMPESTA);

        tempesta_release_power_buses(should_read_pms5003);

        // Return to previous state (IDLE or POWER_SAVE)
        tempesta_change_state(return_state);

        ESP_LOGI(task_tag, "Weather data collection cycle completed");
        //tempesta_log_summary();
    }
}

// ########################## Public API Functions ##########################

/**
 * @brief Get comprehensive weather data snapshot (single mutex operation)
 */
esp_err_t tempesta_get_data_snapshot(tempesta_snapshot_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        // Single efficient copy of entire structure
        memcpy(data, &weather_data, sizeof(tempesta_snapshot_t));
        xSemaphoreGive(xTempestaDataMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief Write TEMPESTA data directly to TELEMETRY cache
 *
 * Comprehensive snapshot for HMI/MQTT distribution with all fields.
 * Writes directly to TELEMETRY's cache buffer to eliminate intermediate copy.
 * Only TEMPESTA mutex needed - TELEMETRY lock/unlock handled by caller.
 *
 * @param cache Pointer to TELEMETRY's tempesta_snapshot_t buffer
 * @return ESP_OK on success, ESP_FAIL if mutex timeout
 */
esp_err_t tempesta_write_to_telemetry_cache(tempesta_snapshot_t *cache)
{
    if (cache == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        // Single efficient copy of entire structure to TELEMETRY cache
        memcpy(cache, &weather_data, sizeof(tempesta_snapshot_t));
        xSemaphoreGive(xTempestaDataMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief Get current TEMPESTA operational state (lightweight, no snapshot fetch)
 * @return Current tempesta_state_t value
 * @note Thread-safe, uses quick mutex with 100ms timeout. Returns DISABLED on mutex timeout.
 */
tempesta_state_t tempesta_get_state(void)
{
    tempesta_state_t state = TEMPESTA_STATE_DISABLED;

    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        state = tempesta_state;
        xSemaphoreGive(xTempestaDataMutex);
    }

    return state;
}

/**
 * @brief Get current temperature for IMPLUVIUM integration (thread-safe)
 * @note This is a legacy function for irrigation temperature correction
 */
float tempesta_get_temperature(void)
{
    float temperature = WEATHER_INVALID_VALUE;

    if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        temperature = weather_data.temperature;
        xSemaphoreGive(xTempestaDataMutex);
    }

    return temperature;
}

// ########################## Load Shedding Functions ##########################

/**
 * @brief Set power saving mode for TEMPESTA weather station
 */
esp_err_t tempesta_set_power_save_mode(bool enable)
{
    if (xTempestaDataMutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (enable != power_save_mode) {
        power_save_mode = enable;

        // Update timer period if system is operational (IDLE state)
        if (tempesta_state == TEMPESTA_STATE_IDLE) {
            tempesta_update_timer_period();
        }

        // Update flag in snapshot
        if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
            weather_data.power_save_mode = enable;
            xSemaphoreGive(xTempestaDataMutex);
        }

        ESP_LOGI(TAG, "Power save mode %s (polling interval: %s)",
                 enable ? "enabled" : "disabled",
                 enable ? "60min" : "15min");
    }

    return ESP_OK;
}

/**
 * @brief Set TEMPESTA collection intervals
 *
 * Allows runtime adjustment of collection intervals for normal and power save modes.
 * Updates both the configuration file and runtime timer period if system is operational.
 *
 * @param normal_min Normal mode interval (5-60 minutes)
 * @param power_save_min Power save mode interval (15-120 minutes)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out of range
 */
esp_err_t tempesta_set_collection_intervals(uint32_t normal_min, uint32_t power_save_min)
{
    // Validate using interval_config API (enforces ranges)
    esp_err_t ret = interval_config_set_tempesta(normal_min, power_save_min);
    if (ret != ESP_OK) {
        return ret;  // Invalid ranges, error already logged
    }

    // Update runtime variables
    tempesta_normal_interval_ms = normal_min * 60 * 1000;
    tempesta_power_save_interval_ms = power_save_min * 60 * 1000;

    // Update timer period immediately if system is operational
    if (tempesta_state == TEMPESTA_STATE_IDLE) {
        tempesta_update_timer_period();
    }

    ESP_LOGI(TAG, "Collection intervals updated: %lum (normal), %lum (power save)",
             normal_min, power_save_min);

    return ESP_OK;
}

/**
 * @brief Set shutdown state for TEMPESTA weather station (for load shedding)
 */
esp_err_t tempesta_set_shutdown(bool shutdown)
{
    if (xTempestaDataMutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (shutdown) {
        // Any state → SHUTDOWN
        if (tempesta_state != TEMPESTA_STATE_SHUTDOWN && tempesta_state != TEMPESTA_STATE_DISABLED) {
            // Save power save mode for recovery
            previous_power_save_mode = power_save_mode;

            tempesta_change_state(TEMPESTA_STATE_SHUTDOWN);
            // Stop the collection timer
            if (xTempestaCollectionTimer != NULL) {
                xTimerStop(xTempestaCollectionTimer, portMAX_DELAY);
            }
            // Ensure hall array is powered off during shutdown
            fluctus_hall_array_enable(false);

            ESP_LOGI(TAG, "TEMPESTA shutdown (saved power_save: %d)", previous_power_save_mode);
        }
    } else {
        // SHUTDOWN → previous state (or IDLE if previous was DISABLED)
        if (tempesta_state == TEMPESTA_STATE_SHUTDOWN) {
            tempesta_state_t target_state = tempesta_previous_state;
            if (target_state == TEMPESTA_STATE_DISABLED || target_state == TEMPESTA_STATE_SHUTDOWN) {
                target_state = TEMPESTA_STATE_IDLE;  // Default to IDLE if previous state was invalid
            }

            // Restore power save mode
            power_save_mode = previous_power_save_mode;
            if (xSemaphoreTake(xTempestaDataMutex, pdMS_TO_TICKS(WEATHER_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
                weather_data.power_save_mode = power_save_mode;
                xSemaphoreGive(xTempestaDataMutex);
            }

            tempesta_change_state(target_state);
            // Restart the collection timer with correct period (uses restored power_save_mode)
            tempesta_update_timer_period();
            if (xTempestaCollectionTimer != NULL) {
                xTimerStart(xTempestaCollectionTimer, portMAX_DELAY);
            }

            ESP_LOGI(TAG, "TEMPESTA restored from shutdown (state: %d, power_save: %d)",
                     target_state, power_save_mode);
        }
    }

    return ESP_OK;
}

/**
 * @brief Enable/disable PMS5003 air quality sensor (for load shedding)
 */
esp_err_t tempesta_set_pms5003_enabled(bool enable)
{
    weather_pms5003_enabled = enable;

    if (enable) {
        ESP_LOGI(TAG, "PMS5003 air quality sensor enabled");
    } else {
        ESP_LOGI(TAG, "PMS5003 air quality sensor disabled for load shedding");
    }

    return ESP_OK;
}

/**
 * @brief Enable/disable TEMPESTA weather station system
 */
esp_err_t tempesta_set_system_enabled(bool enable)
{
    if (xTempestaDataMutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (enable) {
        // Enable: DISABLED → IDLE (or stay in current state if already enabled)
        if (tempesta_state == TEMPESTA_STATE_DISABLED) {
            tempesta_change_state(TEMPESTA_STATE_IDLE);
            // Start the collection timer
            if (xTempestaCollectionTimer != NULL) {
                tempesta_update_timer_period();
                xTimerStart(xTempestaCollectionTimer, portMAX_DELAY);
            }
            // Trigger immediate data collection
            if (xTempestaMainTaskHandle != NULL) {
                xTaskNotify(xTempestaMainTaskHandle, 1, eSetBits);
                ESP_LOGI(TAG, "Triggered immediate data collection on enable");
            }
        } else {
            ESP_LOGD(TAG, "System already enabled (current state: %d)", tempesta_state);
        }
    } else {
        // Disable: Any state → DISABLED
        if (tempesta_state != TEMPESTA_STATE_DISABLED) {
            tempesta_change_state(TEMPESTA_STATE_DISABLED);
            // Stop the collection timer
            if (xTempestaCollectionTimer != NULL) {
                xTimerStop(xTempestaCollectionTimer, portMAX_DELAY);
            }
        }
    }

    return ESP_OK;
}

/**
 * @brief Force immediate data collection cycle (outside normal schedule)
 */
esp_err_t tempesta_force_collection(void)
{
    if (xTempestaDataMutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Only allow forced collection if in IDLE state (operational, regardless of power_save_mode)
    if (tempesta_state != TEMPESTA_STATE_IDLE) {
        ESP_LOGW(TAG, "Cannot force collection - system not in operational state (current: %d)", tempesta_state);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Forcing immediate data collection cycle");

    // Trigger main task notification
    if (xTempestaMainTaskHandle != NULL) {
        xTaskNotify(xTempestaMainTaskHandle, 1, eSetBits);
        return ESP_OK;
    }

    return ESP_FAIL;
}
