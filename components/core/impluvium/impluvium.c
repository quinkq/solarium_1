/**
 * @file irrigation.c
 * @brief IMPLUVIUM - Learning irrigation control system
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Original implementation of IMPLUVIUM - intelligent multi-zone irrigation.
 *
 * Key features:
 * - Five-zone irrigation with independent learning algorithms
 * - Four-phase learning cycle with 15-event history
 * - Temperature-adaptive watering (±1%/°C from 20°C baseline)
 * - Anomaly detection (rain events, extreme temperatures)
 * - Comprehensive safety interlocks (pressure, flow, timeout)
 * - State machine with persistent storage (LittleFS)
 * - Emergency diagnostics and fault isolation
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "impluvium.h"
#include "ads1115_helper.h"
#include "tempesta.h"
#include "fluctus.h"
#include "stellaria.h"
#include "telemetry.h"
#include "solar_calc.h"

#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#define VALIDATE_PTR(ptr)                                                                                              \
    if (!ptr)                                                                                                          \
    return ESP_ERR_INVALID_ARG

// Sensor retry helper macro - to reduce duplication in retry logic
#define SENSOR_READ_WITH_RETRY(read_operation, sensor_name, context_id) do { \
    esp_err_t _ret = ESP_FAIL; \
    for (uint8_t _attempt = 0; _attempt < SENSOR_READ_MAX_RETRIES; _attempt++) { \
        _ret = (read_operation); \
        if (_ret == ESP_OK) { \
            if (_attempt > 0) { \
                ESP_LOGD(TAG, "%s %d succeeded on attempt %d", sensor_name, context_id, _attempt + 1); \
            } \
            break; \
        } \
        if (_attempt < SENSOR_READ_MAX_RETRIES - 1) { \
            ESP_LOGD(TAG, "%s %d attempt %d failed: %s, retrying...", \
                     sensor_name, context_id, _attempt + 1, esp_err_to_name(_ret)); \
            vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_RETRY_DELAY_MS)); \
        } \
    } \
    if (_ret != ESP_OK) { \
        ESP_LOGW(TAG, "Failed to read %s %d after %d attempts: %s", \
                 sensor_name, context_id, SENSOR_READ_MAX_RETRIES, esp_err_to_name(_ret)); \
    } \
    ret = _ret; \
} while(0)

// ########################## Global Variable Definitions ##########################
#define TAG "IMPLUVIUM"

irrigation_zone_t irrigation_zones[IRRIGATION_ZONE_COUNT];
irrigation_system_t irrigation_system;

static SemaphoreHandle_t xIrrigationMutex = NULL;
static TaskHandle_t xIrrigationTaskHandle = NULL;
static TaskHandle_t xIrrigationMonitoringTaskHandle = NULL;
static TimerHandle_t xMoistureCheckTimer = NULL;

// GPIO mapping for zones
static const gpio_num_t zone_valve_gpios[IRRIGATION_ZONE_COUNT] = {VALVE_GPIO_ZONE_1,
                                                                   VALVE_GPIO_ZONE_2,
                                                                   VALVE_GPIO_ZONE_3,
                                                                   VALVE_GPIO_ZONE_4,
                                                                   VALVE_GPIO_ZONE_5};

// ABP sensor handle
abp_t abp_dev = {0};

// ########################## RTC RAM Accumulator ##########################

/**
 * @brief Irrigation accumulator structure (stored in RTC RAM for persistence)
 * Survives ESP32 resets but not power loss
 * Tracks hourly and daily water usage statistics per zone
 *
 * Design: Store per-zone granular data, derive system totals at cache write time
 * Benefits: Single source of truth, no redundancy, full persistence
 */
typedef struct {
    // Per-zone hourly tracking (reset at top of hour)
    time_t current_hour_start;                               // Start time of current hour
    float zone_water_used_hour_ml[IRRIGATION_ZONE_COUNT];    // Hourly per-zone (mL)
    uint8_t zone_events_hour[IRRIGATION_ZONE_COUNT];         // Events per zone this hour

    // Per-zone daily tracking (reset at midnight)
    time_t current_day_start;                                // Day start timestamp
    float zone_water_used_day_ml[IRRIGATION_ZONE_COUNT];     // Daily per-zone (mL)
    uint8_t zone_events_day[IRRIGATION_ZONE_COUNT];          // Events per zone today

    // Metadata
    bool initialized;                                        // Accumulator initialization flag
} irrigation_accumulator_rtc_t;

// RTC RAM accumulator (survives ESP32 resets, persists across deep sleep)
static RTC_DATA_ATTR irrigation_accumulator_rtc_t rtc_accumulator = {0};

// ########################## LittleFS Storage Structures ##########################

#define LEARNING_STORED_HISTORY 5  // Store 5 most recent valid events per zone

/**
 * @brief Zone configuration file structure (user-editable settings)
 *
 * Stored in /irrigation/config.dat (~50 bytes)
 * Write trigger: When user updates via MQTT (future feature)
 */
typedef struct {
    uint32_t magic;                    // 0x494D5043 ("IMPC")
    uint16_t version;                  // Format version (1)
    uint16_t crc16;                    // CRC16-CCITT of zones[] data

    struct {
        bool enabled;                       // Zone enabled/disabled
        float target_moisture_percent;      // Desired moisture level
        float moisture_deadband_percent;    // Tolerance around target
        uint8_t padding;                    // Alignment
    } zones[IRRIGATION_ZONE_COUNT];         // 10 bytes × 5 = 50 bytes

    uint32_t last_saved;               // Timestamp (epoch seconds)
} __attribute__((packed)) irrigation_config_file_t;

/**
 * @brief Learning data file structure (all zones)
 *
 * Stored in /irrigation/learning.dat (~350 bytes)
 * Write trigger: Daily at midnight via impluvium_daily_reset_callback()
 */
typedef struct {
    uint32_t magic;                    // 0x494D504C ("IMPL")
    uint16_t version;                  // Format version (1)
    uint16_t crc16;                    // CRC16-CCITT of zones[] data

    struct {
        // Learned parameters (20 bytes)
        float calculated_ppmp_ratio;              // Pulses per moisture percent
        uint32_t calculated_pump_duty_cycle;      // Optimal pump speed
        float target_moisture_gain_rate;          // Adaptive gain rate (%/sec)
        float confidence_level;                   // 0.0-1.0
        uint32_t successful_predictions;          // Success count
        uint32_t total_predictions;               // Total count

        // Recent history (5 entries = 47 bytes)
        uint8_t history_count;                    // Valid entries (0-5)
        uint8_t history_write_index;              // Circular buffer position
        uint16_t padding;                         // Alignment
        float pulses_used[LEARNING_STORED_HISTORY];                // 20 bytes
        float moisture_increase_percent[LEARNING_STORED_HISTORY];  // 20 bytes
        bool anomaly_flags[LEARNING_STORED_HISTORY];               // 5 bytes
        uint8_t padding2[2];                      // Alignment

    } zones[IRRIGATION_ZONE_COUNT];    // 69 bytes × 5 = 345 bytes

    uint32_t last_saved;               // Timestamp (epoch seconds)
} __attribute__((packed)) irrigation_learning_file_t;

// ########################## Function Prototypes ##########################
// -------------------------- System Initialization ---------------------------
static esp_err_t impluvium_gpio_init(void);
static esp_err_t impluvium_pump_init(void);
static esp_err_t impluvium_flow_sensor_init(void);

// ------------------------ Zone Configuration and Storage Functions ------------------------
static uint16_t impluvium_crc16(const uint8_t *data, size_t length);
static esp_err_t impluvium_save_zone_config(void);
static esp_err_t impluvium_load_zone_config(void);
static esp_err_t impluvium_save_learning_data_all_zones(void);
static esp_err_t impluvium_load_learning_data_all_zones(void);

// ------------------------ Sensor Reading Functions ------------------------
static esp_err_t impluvium_read_moisture_sensor(uint8_t zone_id, float *moisture_percent);
static esp_err_t impluvium_read_pressure(float *outlet_pressure_bar);
static esp_err_t impluvium_read_water_level(float *water_level_percent);

// ------------------------ Power Bus Helper Functions ------------------------
typedef enum {
    POWER_ONLY_SENSORS = 0,    // 3V3 + 5V buses (measuring)
    POWER_ALL_DEVICES = 1,   // 3V3 + 5V + 12V buses (watering)
} impluvium_power_level_t;

static esp_err_t impluvium_request_power_buses(impluvium_power_level_t level, const char *tag);
static void impluvium_release_power_buses(impluvium_power_level_t level, const char *tag);

// ------------------------ Pump Control Functions ------------------------
static esp_err_t impluvium_set_pump_speed(uint32_t pwm_duty);
static esp_err_t impluvium_pump_ramp_up(uint8_t zone_id);
static void impluvium_pump_adaptive_control(float current_gain_rate, float target_gain_rate);

// ------------------------ Learning and Prediction Functions ------------------------
static esp_err_t impluvium_calculate_zone_watering_predictions(void);
static esp_err_t impluvium_process_zone_watering_data(uint8_t zone_id,
                                                   uint32_t pulses_used,
                                                   float moisture_increase_percent,
                                                   bool learning_valid);

// ------------------------ State Machine Functions ------------------------
static esp_err_t impluvium_change_state(irrigation_state_t new_state);
static esp_err_t impluvium_state_measuring(void);
static esp_err_t impluvium_state_watering(void);
static esp_err_t impluvium_state_stopping(void);
static esp_err_t impluvium_state_maintenance(void);

// ------------------------ Timer Callbacks ---------------------------------
static void vTimerCallbackMoistureCheck(TimerHandle_t xTimer);

// ------------------------ Accumulator Functions ------------------------
static void impluvium_update_accumulator(uint8_t zone_id, float water_used_ml);
static bool impluvium_check_hourly_rollover(void);
static void impluvium_handle_midnight_reset(void);

// ------------------------ Monitoring and Safety Functions ------------------------
void impluvium_task(void *pvParameters);
static void impluvium_monitoring_task(void *pvParameters);
static esp_err_t impluvium_pre_check(void);
static void impluvium_calc_flow_rate(const char *task_tag,
                                      uint32_t *last_pulse_count,
                                      uint32_t *last_flow_time,
                                      uint32_t current_time);
static esp_err_t impluvium_watering_cutoffs_check(const char *task_tag, uint32_t time_since_start_ms);
static bool impluvium_periodic_safety_check(uint32_t *error_count, uint32_t time_since_start_ms);
static void impluvium_daily_reset_callback(void);

// ------------------------ Emergency Diagnostics Functions ------------------------
static esp_err_t impluvium_emergency_stop(const char *reason);
static esp_err_t emergency_diagnostics_init(void);
static esp_err_t emergency_diagnostics_start(const char *reason);
static esp_err_t emergency_diagnostics_check_moisture_levels(void);
static esp_err_t emergency_diagnostics_test_zone(uint8_t zone_id);
static esp_err_t emergency_diagnostics_analyze_results(void);
static esp_err_t emergency_diagnostics_resolve(void);


// ########################## Irrigation Definitions ##########################
// -------------------------- System Initialization ---------------------------

/**
 * @brief Initialize the IMPLUVIUM irrigation system
 *
 * Initializes the irrigation system hardware, creates required tasks,
 * and loads configuration from NVS. Must be called before any irrigation operations.
 *
 * @return ESP_OK on successful initialization
 * @return ESP_FAIL on initialization failure
 */
esp_err_t impluvium_init(void)
{
    ESP_LOGI(TAG, "Initializing IMPLUVIUM irrigation system...");

    // Initialize mutex and semaphores
    xIrrigationMutex = xSemaphoreCreateMutex();
    if (xIrrigationMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create irrigation mutex");
        return ESP_FAIL;
    }

    // Create the software timer for moisture checking
    xMoistureCheckTimer = xTimerCreate("MoistureCheckTimer",
                                       pdMS_TO_TICKS(MOISTURE_CHECK_INTERVAL_MS),
                                       pdTRUE, // Auto-reload
                                       (void *) 0,
                                       vTimerCallbackMoistureCheck);
    if (xMoistureCheckTimer == NULL) {
        ESP_LOGE(TAG, "Failed to create moisture check timer");
        return ESP_FAIL;
    }

    // Initialize system state (static variables are zeroinitialized)
    irrigation_system.state = IRRIGATION_STANDBY;
    irrigation_system.active_zone = NO_ACTIVE_ZONE_ID; // No active zone
    irrigation_system.queue_index = 0;                 // Start at beginning of queue
    irrigation_system.state_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    irrigation_system.system_start_time = irrigation_system.state_start_time;
    irrigation_system.power_save_mode = false;         // Start in normal operation
    irrigation_system.load_shed_shutdown = true;      // Start with load shedding disabled

    // Initialize zone configurations
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        irrigation_zones[i].zone_id = i;
        irrigation_zones[i].valve_gpio = zone_valve_gpios[i];
        irrigation_zones[i].target_moisture_percent = 40.0f;  // 40% default target
        irrigation_zones[i].moisture_deadband_percent = 5.0f; // +/- 5% deadband
        irrigation_zones[i].watering_enabled = true;

        // Set ADS1115 device and channel mapping
        // Dev#0: Moisture sensors Zone 1-4, Dev#1: Moisture sensor Zone 5
        if (i < 4) {
            irrigation_zones[i].moisture_ads_device = 0; // ADS1115 #0 (Moisture sensors)
            irrigation_zones[i].moisture_channel = (ads111x_mux_t) (ADS111X_MUX_0_GND + i);
        } else {
            irrigation_zones[i].moisture_ads_device = 1; // ADS1115 #1 (Zone 5 on Ch0)
            irrigation_zones[i].moisture_channel = ADS111X_MUX_0_GND;
        }

        // Initialize learning algorithm data
        // no memset - static variables are zero-initialized
        irrigation_zones[i].learning.calculated_ppmp_ratio = DEFAULT_PULSES_PER_PERCENT; // Default pulses per 1% moisture
        irrigation_zones[i].learning.calculated_pump_duty_cycle = PUMP_DEFAULT_DUTY;
        irrigation_zones[i].learning.target_moisture_gain_rate = TARGET_MOISTURE_GAIN_RATE;
    }

    // Initialize hardware
    esp_err_t ret = impluvium_gpio_init();
    if (ret != ESP_OK)
        return ret;

    ret = impluvium_pump_init();
    if (ret != ESP_OK)
        return ret;

    ret = impluvium_flow_sensor_init();
    if (ret != ESP_OK)
        return ret;

    // Initialize ABP sensor for ±1 psi differential pressure
    // SPI2 bus is initialized in main.c (shared with HMI display)
    ret = abp_init(&abp_dev, ABP_SPI_HOST, ABP_CS_PIN, ABP_RANGE_001PD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ABP sensor: %s", esp_err_to_name(ret));
        return ret;
    }

    // Load zone configurations from LittleFS (user-editable settings)
    ret = impluvium_load_zone_config();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Could not load zone config from LittleFS, using defaults");
    }

    // Load learning data from LittleFS (learned parameters and recent history)
    ret = impluvium_load_learning_data_all_zones();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Could not load learning data from LittleFS, using defaults");
    }

    // Initialize emergency diagnostics
    emergency_diagnostics_init();

    // Create the main irrigation task
    BaseType_t xResult = xTaskCreate(impluvium_task,
                                     "IMPLUVIUM",
                                     configMINIMAL_STACK_SIZE * 6,
                                     NULL,
                                     5, // Medium priority
                                     &xIrrigationTaskHandle);
    if (xResult != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMPLUVIUM irrigation task");
        return ESP_FAIL;
    }

    // Create the monitoring task
    xResult = xTaskCreate(impluvium_monitoring_task,
                          "IMPLUVIUM_Monitor",
                          configMINIMAL_STACK_SIZE * 4,
                          NULL,
                          6, // Higher priority for monitoring
                          &xIrrigationMonitoringTaskHandle);
    if (xResult != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMPLUVIUM monitoring task");
        // Clean up main task
        if (xIrrigationTaskHandle != NULL) {
            vTaskDelete(xIrrigationTaskHandle);
            xIrrigationTaskHandle = NULL;
        }
        return ESP_FAIL;
    }

    // Register daily reset callback with TELEMETRY
    ret = solar_calc_register_midnight_callback(impluvium_daily_reset_callback);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register daily reset callback: %s", esp_err_to_name(ret));
        // Non-critical - continue initialization
    }

    ESP_LOGI(TAG, "IMPLUVIUM irrigation system initialized successfully");
    return ESP_OK;
}

/**
 * @brief Initialize the GPIOs for valve control
 */
static esp_err_t impluvium_gpio_init(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    // Configure valve control pins
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        io_conf.pin_bit_mask = (1ULL << zone_valve_gpios[i]);
        esp_err_t ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure valve GPIO %d: %s", zone_valve_gpios[i], esp_err_to_name(ret));
            return ret;
        }

        // Ensure valve start closed
        gpio_set_level(zone_valve_gpios[i], 0);
    }

    ESP_LOGI(TAG,
             "Irrigation valve GPIOs initialized - Valve GPIO%d - GPIO%d",
             irrigation_zones[0].valve_gpio,
             irrigation_zones[IRRIGATION_ZONE_COUNT - 1].valve_gpio);
    return ESP_OK;
}

/**
 * @brief Initialize PWM for pump speed control
 */
static esp_err_t impluvium_pump_init(void)
{
    // Config timer
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = PUMP_PWM_TIMER,
        .duty_resolution = PUMP_PWM_RESOLUTION,
        .freq_hz = PUMP_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure pump PWM timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure pump PWM channel
    ledc_channel_config_t channel_config = {
        .gpio_num = PUMP_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = PUMP_PWM_CHANNEL,
        .timer_sel = PUMP_PWM_TIMER,
        .duty = 0, // Start with pump off
        .hpoint = 0,
    };

    ret = ledc_channel_config(&channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure pump PWM channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Pump PWM initialized - GPIO42, 1kHz, 10bit");
    return ESP_OK;
}

// Global pulse counter handle
static pcnt_unit_handle_t flow_pcnt_unit = NULL;

/**
 * @brief Initialize flow sensor pulse counting
 */
static esp_err_t impluvium_flow_sensor_init(void)
{
    // Create pulse counter unit configuration
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = 0,
    };

    esp_err_t ret = pcnt_new_unit(&unit_config, &flow_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create pulse counter unit: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create pulse counter channel configuration
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = FLOW_SENSOR_GPIO,
        .level_gpio_num = -1, // Not used
    };

    pcnt_channel_handle_t flow_pcnt_chan = NULL;
    ret = pcnt_new_channel(flow_pcnt_unit, &chan_config, &flow_pcnt_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create pulse counter channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set edge action (count on positive edge only)
    ret =
        pcnt_channel_set_edge_action(flow_pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set edge action: %s", esp_err_to_name(ret));
        return ret;
    }

    // Enable and start counting
    ret = pcnt_unit_enable(flow_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable pulse counter: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_clear_count(flow_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear pulse counter: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_start(flow_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start pulse counter: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Flow sensor initialized - GPIO19, pulse counting");
    return ESP_OK;
}

// ########################## LittleFS Storage Implementation ##########################

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
static esp_err_t impluvium_save_zone_config(void)
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
static esp_err_t impluvium_load_zone_config(void)
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
static esp_err_t impluvium_save_learning_data_all_zones(void)
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
static esp_err_t impluvium_load_learning_data_all_zones(void)
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

// ########################## Irrigation System ##########################
// ---------------------- Sensor Reading Functions -----------------------

/**
 * @brief Read moisture sensor for specific zone
 *
 * Reads ADS1115 voltage, converts to percentage using calibration constants,
 * and updates debug display mutex-protected variables.
 *
 * @param[in]  zone_id          Zone ID (0-4)
 * @param[out] moisture_percent Calculated moisture percentage (0-100%)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid parameters,
 *         ESP_ERR_INVALID_STATE if ADS1115 not available
 */
static esp_err_t impluvium_read_moisture_sensor(uint8_t zone_id, float *moisture_percent)
{
    if (zone_id >= IRRIGATION_ZONE_COUNT || !moisture_percent) {
        return ESP_ERR_INVALID_ARG;
    }

    irrigation_zone_t *zone = &irrigation_zones[zone_id];

    // Check if ADS1115 device is available
    if (!ads1115_devices[zone->moisture_ads_device].initialized) {
        ESP_LOGW(TAG, "Zone %d moisture sensor ADS1115 #%d not available", zone_id, zone->moisture_ads_device);
        return ESP_ERR_INVALID_STATE;
    }

    // Read raw voltage from ADS1115 with retry logic
    int16_t raw;
    float voltage;
    esp_err_t ret = ESP_FAIL;

    SENSOR_READ_WITH_RETRY(
        ads1115_helper_read_channel(zone->moisture_ads_device, zone->moisture_channel, &raw, &voltage),
        "moisture sensor zone", zone_id
    );

    if (ret == ESP_OK) {
        // Success - convert voltage to percentage
        // Clamp voltage to calibrated range
        float clamped_voltage = voltage;
        if (clamped_voltage < MOISTURE_SENSOR_DRY_V)
            clamped_voltage = MOISTURE_SENSOR_DRY_V;
        if (clamped_voltage > MOISTURE_SENSOR_WET_V)
            clamped_voltage = MOISTURE_SENSOR_WET_V;

        // Linear mapping to percentage
        *moisture_percent =
            ((clamped_voltage - MOISTURE_SENSOR_DRY_V) / (MOISTURE_SENSOR_WET_V - MOISTURE_SENSOR_DRY_V)) * 100.0f;

        ESP_LOGD(TAG, "Zone %d moisture sensor read: %.1f%% (%.3fV)", zone_id, *moisture_percent, voltage);
    }

    return ret;
}

/**
 * @brief Read system outlet pressure sensor
 *
 * Reads outlet pressure from ADS1115 #2 Ch2, converts to bar using calibration,
 * and updates debug display variables.
 *
 * @param[out] outlet_pressure_bar System pressure in bar (0-3.0)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer,
 *         ESP_ERR_INVALID_STATE if ADS1115 not available
 */
static esp_err_t impluvium_read_pressure(float *outlet_pressure_bar)
{
    if (!outlet_pressure_bar) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read pump pressure from ADS1115 #2 Ch2
    if (!ads1115_devices[1].initialized) {
        ESP_LOGD(TAG, "Pressure sensor ADS1115 #2 not available");
        return ESP_ERR_INVALID_STATE;
    }

    // Read pressure from ADS1115 with retry logic
    int16_t raw;
    float voltage;
    esp_err_t ret = ESP_FAIL;

    SENSOR_READ_WITH_RETRY(
        ads1115_helper_read_channel(1, ADS111X_MUX_1_GND, &raw, &voltage), // ADS111X_MUX_1_GND is 2nd channel on ADS1115
        "pressure sensor", 0
    );

    if (ret == ESP_OK) {
        // Success - convert voltage to pressure
        // TODO: calibration needed
        // Assuming linear conversion for now
        *outlet_pressure_bar = voltage * 3.0f; // For development time/tests: 3 bar per volt
        
        ESP_LOGD(TAG, "System pressure: %.2f bar (%.3fV)", *outlet_pressure_bar, voltage);
    }

    return ret;
}

/**
 * @brief Read water level sensor from ABP pressure sensor
 *
 * Reads water level using differential pressure measurement,
 * converts to percentage using calibration constants.
 *
 * @param[out] water_level_percent Water level percentage (0-100%)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 * @note Currently only a placeholder implementation
 */
static esp_err_t impluvium_read_water_level(float *water_level_percent)
{
    if (!water_level_percent) {
        return ESP_ERR_INVALID_ARG;
    }

    float water_level_pressure_mbar = 0.0f;
    esp_err_t ret = ESP_FAIL;

    SENSOR_READ_WITH_RETRY(
        abp_read_pressure_mbar(&abp_dev, &water_level_pressure_mbar),
        "water level sensor", 0
    );

    if (ret == ESP_OK) {
        // Clamp pressure to calibrated range
        if (water_level_pressure_mbar < WATER_LEVEL_MIN_MBAR)
            water_level_pressure_mbar = WATER_LEVEL_MIN_MBAR;
        if (water_level_pressure_mbar > WATER_LEVEL_MAX_MBAR)
            water_level_pressure_mbar = WATER_LEVEL_MAX_MBAR;

        // Linear mapping to percentage
        *water_level_percent =
            ((water_level_pressure_mbar - WATER_LEVEL_MIN_MBAR) / (WATER_LEVEL_MAX_MBAR - WATER_LEVEL_MIN_MBAR)) *
            100.0f;

        ESP_LOGD(TAG, "Water level: %.1f%% (%.1f mbar)", *water_level_percent, water_level_pressure_mbar);
    }

    return ret;
}

/**
 * @brief Request power buses for irrigation operations
 *
 * @param[in] level Power level (sensors or watering)
 * @param[in] tag Tag for power bus requests (e.g., "IMPLUVIUM", "IMPLUVIUM_DIAG")
 * @return ESP_OK on success, ESP_FAIL on any bus request failure
 */
static esp_err_t impluvium_request_power_buses(impluvium_power_level_t level, const char *tag)
{
    // Always request sensor buses first
    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_3V3, tag, {
        return ESP_FAIL;
    });
    
    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_5V, tag, {
        fluctus_release_bus_power(POWER_BUS_3V3, tag);
        return ESP_FAIL;
    });

    // Request 12V bus for watering operations
    if (level == POWER_ALL_DEVICES) {
        FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_12V, tag, {
            fluctus_release_bus_power(POWER_BUS_5V, tag);
            fluctus_release_bus_power(POWER_BUS_3V3, tag);
            return ESP_FAIL;
        });
    }

    return ESP_OK;
}

/**
 * @brief Release power buses for irrigation operations
 *
 * @param[in] level Power level (sensors or watering) 
 * @param[in] tag Tag used for power bus requests
 */
static void impluvium_release_power_buses(impluvium_power_level_t level, const char *tag)
{
    // Release in reverse order
    if (level == POWER_ALL_DEVICES) {
        fluctus_release_bus_power(POWER_BUS_12V, tag);
    }
    fluctus_release_bus_power(POWER_BUS_5V, tag);
    fluctus_release_bus_power(POWER_BUS_3V3, tag);
}

// ########################## Irrigation System ##########################
// ------------------------ Pump Control Functions -----------------------

/**
 * @brief Set pump PWM duty cycle with safety limits
 *
 * Controls pump speed via PWM, enforcing minimum duty cycle
 * to prevent pump stall and maximum for safety.
 *
 * @param[in] pwm_duty PWM duty cycle (0-1023, 10-bit)
 * @return ESP_OK on success, ESP_FAIL on PWM configuration error
 */
static esp_err_t impluvium_set_pump_speed(uint32_t pwm_duty)
{
    // Enforce pump speed limits
    if (pwm_duty > 1023) {
        pwm_duty = PUMP_MAX_DUTY;
    }
    if (pwm_duty > 0 && pwm_duty < PUMP_MIN_DUTY) {
        ESP_LOGW(TAG, "Pump duty %" PRIu32 " below minimum %d , adjusting", pwm_duty, PUMP_MIN_DUTY);
        pwm_duty = PUMP_MIN_DUTY;
    }

    irrigation_system.pump_pwm_duty = pwm_duty;

    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, PUMP_PWM_CHANNEL, pwm_duty);
    if (ret == ESP_OK) {
        ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, PUMP_PWM_CHANNEL);
    }

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Pump speed set to %" PRIu32 " /1023 (%.1f%%)", pwm_duty, (pwm_duty / 1023.0f) * 100.0f);
    }

    return ret;
}

/**
 * @brief Ramps up the pump speed over a defined period.
 *
 * Linearly increases the pump's PWM duty cycle from a minimum value
 * to the zone's learned optimal duty cycle. This gradual start-up
 * reduces mechanical stress on the pump and pipes.
 *
 * @param zone_id The ID of the zone being watered.
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
static esp_err_t impluvium_pump_ramp_up(uint8_t zone_id)
{
    ESP_LOGI(TAG, "Ramping up pump for zone %d...", zone_id);

    uint32_t start_duty = PUMP_MIN_DUTY;
    uint32_t target_duty = irrigation_zones[zone_id].learning.calculated_pump_duty_cycle;
    uint32_t ramp_duration_ms = PUMP_RAMP_UP_TIME_MS;
    int steps = 50; // 50 steps for a smooth ramp
    uint32_t step_delay = ramp_duration_ms / steps;

    for (int i = 0; i <= steps; i++) {
        uint32_t duty = start_duty + ((target_duty - start_duty) * i) / steps;
        if (impluvium_set_pump_speed(duty) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set pump speed during ramp-up");
            return ESP_FAIL;
        }
        vTaskDelay(pdMS_TO_TICKS(step_delay));
    }

    ESP_LOGI(TAG, "Pump ramp-up complete for zone %d at %" PRIu32 " duty.", zone_id, target_duty);
    return ESP_OK;
}

/**
 * @brief Simple adaptive pump control based on moisture gain rate
 *
 * Adjusts pump speed up or down based on whether we're meeting the target moisture gain rate.
 *
 * @param current_gain_rate Current moisture gain rate (%/sec)
 * @param target_gain_rate Target moisture gain rate (%/sec)
 */
static void impluvium_pump_adaptive_control(float current_gain_rate, float target_gain_rate)
{
    float rate_error = target_gain_rate - current_gain_rate;

    // Only adjust if error is significant
    if (fabs(rate_error) < PUMP_GAIN_RATE_TOLERANCE) {
        return; // Close enough, no adjustment needed
    }

    uint32_t current_duty = irrigation_system.pump_pwm_duty;
    uint32_t new_duty = current_duty;

    if (rate_error > 0) {
        // Need more moisture gain - increase pump speed
        new_duty = current_duty + PUMP_ADJUSTMENT_STEP;
        ESP_LOGD(TAG, "Increasing pump speed: %.2f < %.2f %%/sec", current_gain_rate, target_gain_rate);
    } else {
        // Too much moisture gain - decrease pump speed
        new_duty = current_duty - (PUMP_ADJUSTMENT_STEP / 2); // Decrease more slowly
        ESP_LOGD(TAG, "Decreasing pump speed: %.2f > %.2f %%/sec", current_gain_rate, target_gain_rate);
    }

    impluvium_set_pump_speed(new_duty);
}

// ########################## Irrigation System ##########################
// ------------------- Learning Algorithm Functions -----------------------

/**
 * @brief Calculate dynamic moisture check interval based on temperature
 *
 * @param[in] current_temperature Current temperature in °C
 * @return Moisture check interval in milliseconds
 */
static uint32_t impluvium_calc_moisture_check_interval(float current_temperature)
{
    uint32_t interval;

    // Check for power save mode override
    if (irrigation_system.power_save_mode) {
        ESP_LOGD(TAG, "Power save mode active - using 60min interval");
        interval = MOISTURE_CHECK_INTERVAL_POWER_SAVE_MS;
    }
    else if (current_temperature == WEATHER_INVALID_VALUE) {
        ESP_LOGW(TAG, "Invalid temperature - using default interval");
        interval = MOISTURE_CHECK_INTERVAL_MS;
    }
    else if (current_temperature < MIN_TEMPERATURE_WATERING) {
        ESP_LOGI(TAG, "Temperature %.1f°C below watering threshold - skipping moisture checks", current_temperature);
        return UINT32_MAX; // Skip moisture checks completely
    }
    else if (current_temperature >= TEMPERATURE_OPTIMAL_THRESHOLD) {
        ESP_LOGD(TAG, "Optimal temperature %.1f°C - using 15min interval", current_temperature);
        interval = MOISTURE_CHECK_INTERVAL_OPTIMAL_MS;
    }
    else {
        ESP_LOGD(TAG, "Cool temperature %.1f°C - using 30min interval", current_temperature);
        interval = MOISTURE_CHECK_INTERVAL_COLD_MS;
    }

    // Apply nighttime hard minimum (3 hours) to reduce unnecessary checks during darkness
    if (!solar_calc_is_daytime_buffered()) {
        const uint32_t NIGHT_MINIMUM_INTERVAL_MS = 3 * 60 * 60 * 1000; // 3 hours
        if (interval < NIGHT_MINIMUM_INTERVAL_MS) {
            ESP_LOGD(TAG, "Nighttime - extending interval from %lums to %lums", interval, NIGHT_MINIMUM_INTERVAL_MS);
            interval = NIGHT_MINIMUM_INTERVAL_MS;
        }
    }

    return interval;
}

/**
 * @brief Calculate temperature correction factor for watering predictions
 *
 * @param[in] zone_learning Zone learning data structure
 * @return Temperature correction factor (typically 0.8 - 1.2)
 */
static float impluvium_calculate_temperature_correction(zone_learning_t *learning)
{
    // Calculate temperature correction factor
    // Formula: 1.0 + (current_temperature - baseline_temp) * correction_factor
    // Example: At 30°C: 1.0 + (30-20) * 0.01 = 1.10 (10% more water)
    //          At 10°C: 1.0 + (10-20) * 0.01 = 0.90 (10% less water)
    float current_temperature = tempesta_get_temperature();

    if (current_temperature == WEATHER_INVALID_VALUE) {
        ESP_LOGW(TAG, "Temperature sensors failed - using baseline correction");
        learning->last_temperature_correction = 1.0f;
        return 1.0f;
    }

    float calculated_temp_correction = 1.0f + ((current_temperature - TEMPERATURE_BASELINE) * TEMP_CORRECTION_FACTOR);
    learning->last_temperature_correction = calculated_temp_correction;
    ESP_LOGD(TAG, "Temp %.1f°C, correction factor %.2f", current_temperature, calculated_temp_correction);
    return calculated_temp_correction;
}

/**
 * @brief Calculate weighted learning ratio from historical data
 *
 * @param[in] learning Zone learning data
 * @param[out] valid_cycles Number of valid historical cycles found
 * @return Calculated pulses per percent ratio, or 0.0 if insufficient data
 */
static float impluvium_calculate_pulse_per_moisture_percent(zone_learning_t *learning, uint8_t *valid_cycles)
{
    float weighted_ratio = 0.0f;
    float total_weight = 0.0f;
    *valid_cycles = 0;

    for (uint8_t h = 0; h < learning->history_entry_count; h++) {
        // Skip anomalous cycles (rain, manual watering, etc.)
        if (!learning->anomaly_flags[h] && learning->moisture_increase_percent_history[h] > 0) {
            // Calculate recency: most recent entries get higher weight
            // Recent entries are at (history_index - 1), (history_index - 2), etc. with wraparound
            uint8_t relative_age = (learning->history_index - h - 1 + LEARNING_HISTORY_SIZE) % LEARNING_HISTORY_SIZE;
            float weight = (relative_age < 3) ? LEARNING_WEIGHT_RECENT : (1.0f - LEARNING_WEIGHT_RECENT);

            // Calculate pulses per moisture percent for this cycle
            float ratio = learning->pulses_used_history[h] / learning->moisture_increase_percent_history[h];

            weighted_ratio += ratio * weight;
            total_weight += weight;
            (*valid_cycles)++;

            ESP_LOGD(TAG,
                     "Cycle %d (age %d): %d pulses, %.2f%% increase, ratio %.1f, weight %.2f",
                     h,
                     relative_age,
                     (int) learning->pulses_used_history[h],
                     learning->moisture_increase_percent_history[h],
                     ratio,
                     weight);
        }
    }

    return (total_weight > 0 && *valid_cycles >= 2) ? (weighted_ratio / total_weight) : 0.0f;
}

/**
 * @brief Calculate target pulses for a single zone
 *
 * @param[in] zone_id Zone identifier
 * @param[in] queue_index Index in watering queue
 * @return ESP_OK on success
 */
static esp_err_t impluvium_calculate_zone_target_pulses(uint8_t zone_id, uint8_t queue_index)
{
    irrigation_zone_t *zone = &irrigation_zones[zone_id];
    zone_learning_t *learning = &zone->learning;

    // Calculate temperature correction
    float calculated_temp_correction = impluvium_calculate_temperature_correction(learning);

    // Check if we have sufficient learning data
    if (learning->history_entry_count >= LEARNING_MIN_CYCLES) {
        uint8_t valid_cycles;
        float calculated_ppmp_ratio = impluvium_calculate_pulse_per_moisture_percent(learning, &valid_cycles);

        if (calculated_ppmp_ratio > 0.0f) {
            learning->calculated_ppmp_ratio = calculated_ppmp_ratio;

            // Calculate learned prediction
            float learned_target_pulses = irrigation_system.watering_queue[queue_index].moisture_deficit_percent *
                                         calculated_ppmp_ratio * calculated_temp_correction;

            // Calculate default prediction for blending
            float default_target_pulses = irrigation_system.watering_queue[queue_index].moisture_deficit_percent *
                                        DEFAULT_PULSES_PER_PERCENT * calculated_temp_correction;

            // Apply confidence-based blending
            float target_pulses;
            float confidence = learning->confidence_level;
            
            if (confidence >= 0.70f) {
                // High confidence: Use full learned prediction
                target_pulses = learned_target_pulses;
                ESP_LOGI(TAG, "Zone %d: High confidence (%.0f%%), using learned prediction", 
                        zone_id, confidence * 100.0f);
            } else if (confidence >= 0.40f) {
                // Medium confidence: Blend learned + default
                float blend_factor = (confidence - 0.40f) / 0.30f; // 0.0 to 1.0 scale
                target_pulses = (learned_target_pulses * blend_factor) + (default_target_pulses * (1.0f - blend_factor));
                ESP_LOGI(TAG, "Zone %d: Medium confidence (%.0f%%), blending predictions (%.0f%% learned)", 
                        zone_id, confidence * 100.0f, blend_factor * 100.0f);
            } else {
                // Low confidence: Use mostly default with slight learned influence
                target_pulses = (default_target_pulses * 0.8f) + (learned_target_pulses * 0.2f);
                ESP_LOGI(TAG, "Zone %d: Low confidence (%.0f%%), using mostly default prediction", 
                        zone_id, confidence * 100.0f);
            }

            // Limit to reasonable range
            if (target_pulses < MINIMUM_TARGET_PULSES)
                target_pulses = MINIMUM_TARGET_PULSES;
            if (target_pulses > MAXIMUM_TARGET_PULSES)
                target_pulses = MAXIMUM_TARGET_PULSES;

            irrigation_system.watering_queue[queue_index].target_pulses = (uint16_t) target_pulses;

            ESP_LOGI(TAG,
                     "Zone %d: Final prediction %d pulses (learned: %.1f, default: %.1f pulses/%%, temp: %.2f°C)",
                     zone_id,
                     irrigation_system.watering_queue[queue_index].target_pulses,
                     calculated_ppmp_ratio,
                     DEFAULT_PULSES_PER_PERCENT,
                     calculated_temp_correction);
        } else {
            // Not enough valid learning data
            irrigation_system.watering_queue[queue_index].target_pulses = DEFAULT_TARGET_PULSES;
            ESP_LOGW(TAG,
                     "Zone %d: Insufficient valid learning data (%d cycles), using default %d pulses",
                     zone_id,
                     valid_cycles,
                     DEFAULT_TARGET_PULSES);
        }
    } else {
        // Not enough learning cycles - use default
        irrigation_system.watering_queue[queue_index].target_pulses = DEFAULT_TARGET_PULSES;
        ESP_LOGI(TAG,
                 "Zone %d: Learning phase (%d/%d cycles), using default %d pulses",
                 zone_id,
                 learning->history_entry_count,
                 LEARNING_MIN_CYCLES,
                 DEFAULT_TARGET_PULSES);
    }

    ESP_LOGI(TAG,
             "Queue[%d]: Zone %d, deficit %.2f%%, target %d pulses",
             queue_index,
             zone_id,
             irrigation_system.watering_queue[queue_index].moisture_deficit_percent,
             irrigation_system.watering_queue[queue_index].target_pulses);

    return ESP_OK;
}

/**
 * @brief Calculate watering predictions for all zones in queue
 *
 * Orchestrates the learning algorithm to predict optimal water amounts
 * for each zone in the watering queue.
 *
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
static esp_err_t impluvium_calculate_zone_watering_predictions(void)
{
    ESP_LOGI(TAG, "Calculating watering predictions for %d zones", irrigation_system.watering_queue_size);

    for (uint8_t i = 0; i < irrigation_system.watering_queue_size; i++) {
        uint8_t zone_id = irrigation_system.watering_queue[i].zone_id;
        esp_err_t ret = impluvium_calculate_zone_target_pulses(zone_id, i);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to calculate target pulses for zone %d: %s", zone_id, esp_err_to_name(ret));
            return ret;
        }
    }

    return ESP_OK;
}

/**
 * @brief Update learning algorithm with post-watering data
 *
 * This function records the results of a watering cycle for future learning:
 * 1. Stores pulses used and moisture increase achieved
 * 2. Marks anomalous cycles (rain, manual watering, sensor errors)
 * 3. Maintains circular buffer of historical data
 * 4. Updates learning statistics
 *
 * Anomaly Detection Criteria:
 * - Excessive moisture increase (>0.3V indicates rain/manual watering)
 * - Extreme temperature conditions (<5°C or >45°C)
 * - Flow anomalies detected during watering
 * - Other system-detected anomalies
 *
 * @param zone_id Zone that was watered (0-4)
 * @param pulses_used Number of flow sensor pulses during watering
 * @param moisture_increase_percent Percentage increase observed in moisture sensor (final - initial)
 * @param learning_valid Whether this cycle should be used for learning
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid parameters
 */
static esp_err_t impluvium_process_zone_watering_data(uint8_t zone_id,
                                                   uint32_t pulses_used,
                                                   float moisture_increase_percent,
                                                   bool learning_valid)
{
    if (zone_id >= IRRIGATION_ZONE_COUNT) {
        ESP_LOGE(TAG, "Invalid zone_id %d for learning update", zone_id);
        return ESP_ERR_INVALID_ARG;
    }

    irrigation_zone_t *zone = &irrigation_zones[zone_id];
    zone_learning_t *learning = &zone->learning;

    // Get current write position in circular buffer
    uint8_t index = learning->history_index;

    // Store the learning data
    learning->pulses_used_history[index] = pulses_used;
    learning->moisture_increase_percent_history[index] = moisture_increase_percent;
    learning->anomaly_flags[index] = !learning_valid; // Invert: true = anomaly (stored for future calculations)

    // Advance circular buffer index
    learning->history_index = (learning->history_index + 1) % LEARNING_HISTORY_SIZE;

    // Update count (saturates at LEARNING_HISTORY_SIZE)
    if (learning->history_entry_count < LEARNING_HISTORY_SIZE) {
        learning->history_entry_count++;
    }

    // Calculate basic statistics for this cycle (pulses per 1% moisture)
    float measured_ppmp_ratio = (moisture_increase_percent > 0) ? (pulses_used / moisture_increase_percent) : 0;

    ESP_LOGI(TAG,
             "Zone %d learning update: %lu pulses → %.2f%% increase (%.1f pulses/%%), %s",
             zone_id,
             pulses_used,
             moisture_increase_percent,
             measured_ppmp_ratio,
             learning_valid ? "valid" : "anomaly");

    // Real-time learning updates (only for valid cycles)
    if (learning_valid && measured_ppmp_ratio > 0.1f && measured_ppmp_ratio < 50.0f) { // Reasonable measured_ppmp_ratio range
        // Update learned measured_ppmp_ratio with exponential moving average (10% new, 90% old)
        learning->calculated_ppmp_ratio = 
            (learning->calculated_ppmp_ratio * 0.9f) + (measured_ppmp_ratio * 0.1f);

        ESP_LOGD(TAG, "Zone %d: Real-time measured_ppmp_ratio update: %.1f pulses/%% (was %.1f)", 
                 zone_id, learning->calculated_ppmp_ratio, measured_ppmp_ratio);
    }

    // ### Confidence tracking calculations (only for valid predictions)
    if (learning_valid) {
        learning->total_predictions++;
        
        // Check if prediction was reasonably accurate (within 20% of expected)
        if (learning->calculated_ppmp_ratio > 0) {
            float expected_increase = (pulses_used / learning->calculated_ppmp_ratio);
            float accuracy = fabs(moisture_increase_percent - expected_increase) / expected_increase;
            if (accuracy <= 0.20f) { // Within 20% tolerance
                learning->successful_predictions++;
            }
        }

        // Update confidence level based on success rate
        if (learning->total_predictions > 0) {
            learning->confidence_level = (float) learning->successful_predictions / learning->total_predictions;
        }
    }

    // Update learned parameters and log progress (only after sufficient learning cycles)
    if (learning->history_entry_count >= LEARNING_MIN_CYCLES) {
        // Update pump duty cycle (only for valid cycles)
        if (learning_valid) {
            // Exponential moving average for pump duty cycle (10% new, 90% old)
            learning->calculated_pump_duty_cycle =
                (uint32_t)((learning->calculated_pump_duty_cycle * 0.9f) + (irrigation_system.pump_pwm_duty * 0.1f));

            ESP_LOGI(TAG,
                     "Zone %d: Updated learned pump duty cycle to %" PRIu32 ", confidence %.2f",
                     zone_id,
                     learning->calculated_pump_duty_cycle,
                     learning->confidence_level);
        }

        // Count valid entries for logging (cached calculation instead of loop each time)
        uint8_t valid_entries = 0;
        for (uint8_t i = 0; i < learning->history_entry_count; i++) {
            if (!learning->anomaly_flags[i])
                valid_entries++;
        }
        
        ESP_LOGI(TAG,
                 "Zone %d learning: %d total cycles, %d valid, %lu/%lu predictions successful (%.1f%% confidence)",
                 zone_id,
                 learning->history_entry_count,
                 valid_entries,
                 learning->successful_predictions,
                 learning->total_predictions,
                 learning->confidence_level * 100.0f);
    }

    return ESP_OK;
}

// ########################## Irrigation System ##########################
// ---------------------- State Machine Functions ------------------------

/**
 * @brief Change irrigation system state with logging
 */
static esp_err_t impluvium_change_state(irrigation_state_t new_state)
{
    if (irrigation_system.state != new_state) {
        // Convert states to strings for logging
        const char *old_state_str = "UNKNOWN";
        const char *new_state_str = "UNKNOWN";

        switch (irrigation_system.state) {
            case IRRIGATION_STANDBY: old_state_str = "STANDBY"; break;
            case IRRIGATION_MEASURING: old_state_str = "MEASURING"; break;
            case IRRIGATION_WATERING: old_state_str = "WATERING"; break;
            case IRRIGATION_STOPPING: old_state_str = "STOPPING"; break;
            case IRRIGATION_MAINTENANCE: old_state_str = "MAINTENANCE"; break;
            case IRRIGATION_DISABLED: old_state_str = "DISABLED"; break;
        }

        switch (new_state) {
            case IRRIGATION_STANDBY: new_state_str = "STANDBY"; break;
            case IRRIGATION_MEASURING: new_state_str = "MEASURING"; break;
            case IRRIGATION_WATERING: new_state_str = "WATERING"; break;
            case IRRIGATION_STOPPING: new_state_str = "STOPPING"; break;
            case IRRIGATION_MAINTENANCE: new_state_str = "MAINTENANCE"; break;
            case IRRIGATION_DISABLED: new_state_str = "DISABLED"; break;
        }

        ESP_LOGI(TAG, "State change: %s -> %s", old_state_str, new_state_str);

        irrigation_system.state = new_state;
        irrigation_system.state_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Push state change to telemetry central hub
        telemetry_fetch_snapshot(TELEMETRY_SRC_IMPLUVIUM);
    }
    return ESP_OK;
}

/**
 * @brief Execute MEASURING state operations
 *
 * Powers on sensor bus, performs safety checks, scans all zones
 * for watering needs, builds prioritized watering queue.
 *
 * @return ESP_OK on success, ESP_FAIL on safety check failure
 */
static esp_err_t impluvium_state_measuring(void)
{
    // Check FLUCTUS power state before requesting power
    fluctus_power_state_t power_state = fluctus_get_power_state();
    if (power_state == FLUCTUS_POWER_STATE_CRITICAL) {
        ESP_LOGW(TAG, "Critical power state - skipping irrigation cycle");
        impluvium_change_state(IRRIGATION_STANDBY);
        return ESP_OK;
    }

    if (power_state >= FLUCTUS_POWER_STATE_VERY_LOW) {
        ESP_LOGW(TAG, "Very low power state - irrigation disabled for battery conservation");
        impluvium_change_state(IRRIGATION_STANDBY);
        return ESP_OK;
    }

    // Power on sensors (3V3 for sensors, 5V for pressure transmitter)
    if (impluvium_request_power_buses(POWER_ONLY_SENSORS, "IMPLUVIUM") != ESP_OK) {
        impluvium_change_state(IRRIGATION_MAINTENANCE);
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Perform pre-start safety checks directly
    if (impluvium_pre_check() != ESP_OK) {
        ESP_LOGE(TAG, "Pre-start safety check failed.");
        impluvium_release_power_buses(POWER_ONLY_SENSORS, "IMPLUVIUM");
        impluvium_change_state(IRRIGATION_MAINTENANCE);
        return ESP_FAIL;
    }

    // **MANUAL WATERING MODE**: Bypass normal moisture checks and directly queue the manual zone
    if (irrigation_system.manual_watering_active) {
        uint8_t zone_id = irrigation_system.manual_water_zone;

        ESP_LOGI(TAG, "Manual watering mode active for zone %d (duration: %d seconds)",
                 zone_id, irrigation_system.manual_water_duration_sec);

        // Add manual zone directly to queue (bypass moisture check and interval check)
        irrigation_system.watering_queue_size = 0;
        irrigation_system.watering_queue[0].zone_id = zone_id;
        irrigation_system.watering_queue[0].measured_moisture_percent = 0.0f; // Not applicable for manual watering
        irrigation_system.watering_queue[0].moisture_deficit_percent = 0.0f;  // Not applicable
        irrigation_system.watering_queue[0].target_pulses = 0;                // Time-based cutoff instead
        irrigation_system.watering_queue[0].watering_completed = false;
        irrigation_system.watering_queue_size = 1;

        // Start watering immediately
        irrigation_system.queue_index = 0;
        ESP_LOGI(TAG, "Starting manual watering for zone %d", zone_id);
        impluvium_change_state(IRRIGATION_WATERING);

        return ESP_OK;
    }

    // Check each zone for watering needs
    for (uint8_t zone_id = 0; zone_id < IRRIGATION_ZONE_COUNT; zone_id++) {
        if (!irrigation_zones[zone_id].watering_enabled)
            continue;

        float moisture_percent;
        esp_err_t ret = impluvium_read_moisture_sensor(zone_id, &moisture_percent);
        if (ret == ESP_OK) {
            irrigation_zone_t *zone = &irrigation_zones[zone_id];
            float moisture_deficit_percent = zone->target_moisture_percent - moisture_percent;

            // Check if watering is needed
            if (moisture_deficit_percent > zone->moisture_deadband_percent) {
                // Check minimum interval since last watering (using monotonic time)
                int64_t current_time_ms = esp_timer_get_time() / 1000;
                if (zone->last_watered_time_ms == 0 ||
                    (current_time_ms - zone->last_watered_time_ms) >= MIN_WATERING_INTERVAL_MS) {
                    ESP_LOGI(TAG,
                             "Zone %d needs water: %.1f%% < %.1f%% (deficit: %.1f%%)",
                             zone_id,
                             moisture_percent,
                             zone->target_moisture_percent,
                             moisture_deficit_percent);

                    // Add to watering queue
                    if (irrigation_system.watering_queue_size < IRRIGATION_ZONE_COUNT) {
                        irrigation_system.watering_queue[irrigation_system.watering_queue_size].zone_id = zone_id;
                        irrigation_system.watering_queue[irrigation_system.watering_queue_size]
                            .measured_moisture_percent = moisture_percent;
                        irrigation_system.watering_queue[irrigation_system.watering_queue_size]
                            .moisture_deficit_percent = moisture_deficit_percent;
                        irrigation_system.watering_queue[irrigation_system.watering_queue_size].watering_completed = false;
                        irrigation_system.watering_queue_size++;
                    }
                }
            }
        }
    }

    // Process the watering queue
    if (irrigation_system.watering_queue_size > 0) {
        // Sort queue by moisture deficit (highest priority first) - Insertion Sort
        for (uint8_t i = 1; i < irrigation_system.watering_queue_size; i++) {
            watering_queue_item_t key = irrigation_system.watering_queue[i];
            int8_t j = i - 1;

            // Move elements with lower deficit one position ahead
            while (j >= 0 &&
                   irrigation_system.watering_queue[j].moisture_deficit_percent < key.moisture_deficit_percent) {
                irrigation_system.watering_queue[j + 1] = irrigation_system.watering_queue[j];
                j--;
            }
            irrigation_system.watering_queue[j + 1] = key;
        }

        // Calculate predicted pulses for each zone using learning algorithm
        impluvium_calculate_zone_watering_predictions();

        // Start watering queue - keep sensors powered
        irrigation_system.queue_index = 0;
        ESP_LOGI(TAG, "Starting watering queue with %d zones", irrigation_system.watering_queue_size);
        impluvium_change_state(IRRIGATION_WATERING);
    } else {
        // No zones need watering - power off sensors and return to idle
        impluvium_release_power_buses(POWER_ONLY_SENSORS, "IMPLUVIUM");
        irrigation_system.last_moisture_check = xTaskGetTickCount() * portTICK_PERIOD_MS;
        impluvium_change_state(IRRIGATION_STANDBY);
    }

    return ESP_OK;
}

/**
 * @brief Execute WATERING state initialization
 *
 * Sets up watering for current zone: opens valve, ramps up pump,
 * initializes flow counting, notifies monitoring task.
 *
 * @return ESP_OK on success, ESP_FAIL on pump/valve failure
 */
static esp_err_t impluvium_state_watering(void)
{
    // Now called only once when transitioning into the WATERING state.
    // It sets up the watering for the current zone in the queue.

    // Get current zone from queue
    if (irrigation_system.queue_index >= irrigation_system.watering_queue_size) {
        ESP_LOGE(TAG, "Invalid queue index in WATERING state");
        impluvium_change_state(IRRIGATION_STOPPING);
        return ESP_FAIL;
    }

    uint8_t zone_id = irrigation_system.watering_queue[irrigation_system.queue_index].zone_id;
    irrigation_zone_t *zone = &irrigation_zones[zone_id];
    ESP_LOGI(TAG, "Watering sequence started for zone %d", zone_id);
    
    // Request 12V power for valve & pump operation (sensors already powered from measuring state)
    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_12V, "IMPLUVIUM", {
        gpio_set_level(zone->valve_gpio, 0);
        impluvium_release_power_buses(POWER_ONLY_SENSORS, "IMPLUVIUM");
        impluvium_change_state(IRRIGATION_MAINTENANCE);
        return ESP_FAIL;
    });

    // Request STELLARIA to dim lights for power management during irrigation
    stellaria_request_irrigation_dim(true);

    // Open valve
    gpio_set_level(zone->valve_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(VALVE_OPEN_DELAY_MS)); // Wait for valve to open

    // Ramp up pump speed
    if (impluvium_pump_ramp_up(zone_id) != ESP_OK) {
        ESP_LOGE(TAG, "Pump ramp-up failed for zone %d, turning off power buses", zone_id);
        gpio_set_level(zone->valve_gpio, 0);
        impluvium_release_power_buses(POWER_ALL_DEVICES, "IMPLUVIUM");
        impluvium_change_state(IRRIGATION_MAINTENANCE);
        return ESP_FAIL;
    }

    // Reset flow sensor counter and update system state
    pcnt_unit_clear_count(flow_pcnt_unit);
    pcnt_unit_get_count(flow_pcnt_unit, (int *) &irrigation_system.watering_start_pulses);
    irrigation_system.active_zone = zone_id;
    zone->watering_in_progress = true;
    zone->last_watered_time_ms = esp_timer_get_time() / 1000;  // Monotonic time
    irrigation_system.watering_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Store starting moisture for learning algorithm
    irrigation_system.watering_queue[irrigation_system.queue_index].moisture_at_start_percent =
        irrigation_system.watering_queue[irrigation_system.queue_index].measured_moisture_percent;

    // Notify monitoring task to start continuous monitoring
    xTaskNotify(xIrrigationMonitoringTaskHandle, MONITORING_TASK_NOTIFY_START_MONITORING, eSetBits);

    // Now, the main task will simply wait in the while loop for a notification
    // to stop watering. The monitoring task is in full control.
    return ESP_OK;
}

/**
 * @brief STOPPING state - stop pump, update learning, process next zone in queue
 */
static esp_err_t impluvium_state_stopping(void)
{
    uint8_t active_zone = irrigation_system.active_zone;

    // Notify monitoring task to stop
    xTaskNotify(xIrrigationMonitoringTaskHandle, MONITORING_TASK_NOTIFY_STOP_MONITORING, eSetBits);

    // Stop pump
    impluvium_set_pump_speed(0);
    ESP_LOGI(TAG, "Pump stopped - delaying %dms for pressure equalization", PRESSURE_EQUALIZE_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(PRESSURE_EQUALIZE_DELAY_MS));

    // Restore STELLARIA to previous intensity (irrigation complete)
    stellaria_request_irrigation_dim(false);

    // Close current zone valve
    if (active_zone < IRRIGATION_ZONE_COUNT) {
        gpio_set_level(irrigation_zones[active_zone].valve_gpio, 0);
        irrigation_zones[active_zone].watering_in_progress = false;
    } else {
        ESP_LOGE(TAG, "Invalid active zone in STOPPING state: %d", active_zone);
    }

    // Calculate volume used and update learning algorithm
    if (active_zone < IRRIGATION_ZONE_COUNT && irrigation_system.queue_index < irrigation_system.watering_queue_size) {
        int total_pulses;
        pcnt_unit_get_count(flow_pcnt_unit, &total_pulses);
        uint32_t pulses_used = total_pulses - irrigation_system.watering_start_pulses;
        float volume_ml = (pulses_used / FLOW_CALIBRATION_PULSES_PER_LITER) * 1000.0f;

        // Update RTC accumulator with water usage (single source of truth - persistent!)
        impluvium_update_accumulator(active_zone, volume_ml);
        impluvium_check_hourly_rollover();

        // **MANUAL WATERING MODE**: Skip learning algorithm, just log volume used
        if (irrigation_system.manual_watering_active) {
            ESP_LOGI(TAG, "Manual watering completed for zone %d: %lu pulses, %.1f mL (%d seconds)",
                     active_zone, pulses_used, volume_ml, irrigation_system.manual_water_duration_sec);

            // Clear manual watering flags
            irrigation_system.manual_watering_active = false;
            irrigation_system.manual_water_zone = 0;
            irrigation_system.manual_water_duration_sec = 0;
            irrigation_system.manual_water_end_time = 0;

            ESP_LOGI(TAG, "Manual watering mode deactivated");
        }
        // Normal watering: Read final moisture level for learning
        else {
            float final_moisture_percent;
            bool learning_valid = true;
            if (impluvium_read_moisture_sensor(active_zone, &final_moisture_percent) == ESP_OK) {
            watering_queue_item_t *queue_item = &irrigation_system.watering_queue[irrigation_system.queue_index];
            float moisture_increase_percent = final_moisture_percent - queue_item->moisture_at_start_percent;

            // Check for anomalies that would invalidate learning data
            // Note: Temperature check is for learning validity only - safety was already verified before watering
            float current_temperature = tempesta_get_temperature();

            if (irrigation_system.current_anomaly.type != ANOMALY_NONE ||
                current_temperature == WEATHER_INVALID_VALUE || 
                current_temperature < TEMP_EXTREME_LOW || current_temperature > TEMP_EXTREME_HIGH) {
                learning_valid = false;
                ESP_LOGD(TAG,
                         "Zone %d: Learning data invalidated (anomaly=%d, temp=%.1f°C) - extreme conditions affect soil behavior",
                         active_zone,
                         irrigation_system.current_anomaly.type,
                         current_temperature);
            }

            // Calculate watering duration for gain rate learning
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            uint32_t watering_duration_ms = current_time - irrigation_system.watering_start_time;
            
            // Update learning algorithm with results
            impluvium_process_zone_watering_data(active_zone, pulses_used, moisture_increase_percent, learning_valid);
            
            // Learn optimal target moisture gain rate (only for valid, reasonable cycles)
            if (learning_valid && watering_duration_ms > 5000 && watering_duration_ms < 60000) { // 5s to 60s range
                float actual_gain_rate = moisture_increase_percent / (watering_duration_ms / 1000.0f);
                if (actual_gain_rate > 0.1f && actual_gain_rate < 2.0f) { // Reasonable gain rate range
                    zone_learning_t *learning = &irrigation_zones[active_zone].learning;
                    // Update target gain rate with exponential moving average (5% new, 95% old for slower adaptation)
                    learning->target_moisture_gain_rate = 
                        (learning->target_moisture_gain_rate * 0.95f) + (actual_gain_rate * 0.05f);
                    
                    ESP_LOGI(TAG, "Zone %d: Updated target gain rate: %.2f %%/sec (measured %.2f %%/sec over %lu s)", 
                             active_zone, learning->target_moisture_gain_rate, actual_gain_rate, watering_duration_ms / 1000);
                }
            }

            ESP_LOGI(TAG,
                     "Zone %d: Used %lu pulses, %.1fmL, moisture %.1f%%->%.1f%% (+%.1f%%)",
                     active_zone,
                     pulses_used,
                     volume_ml,
                     queue_item->moisture_at_start_percent,
                     final_moisture_percent,
                     moisture_increase_percent);
            }
        } // End of normal watering (else block)

        // Mark current queue item as completed
        irrigation_system.watering_queue[irrigation_system.queue_index].watering_completed = true;
    }

    // Reset anomaly detection for next zone
    memset(&irrigation_system.current_anomaly, 0, sizeof(watering_anomaly_t));

    // Move to next zone in queue
    irrigation_system.queue_index++;
    irrigation_system.active_zone = NO_ACTIVE_ZONE_ID; // Reset active zone

    if (irrigation_system.queue_index < irrigation_system.watering_queue_size) {
        // More zones to water - go back to WATERING state for the next zone
        ESP_LOGI(TAG,
                 "Moving to next zone in queue: %d/%d",
                 irrigation_system.queue_index + 1,
                 irrigation_system.watering_queue_size);
        impluvium_change_state(IRRIGATION_WATERING);
    } else {
        // All zones completed - finish session
        ESP_LOGI(TAG, "All %d zones in queue completed", irrigation_system.watering_queue_size);

        // Power off all buses now that all zones are done
        fluctus_release_bus_power(POWER_BUS_12V, "IMPLUVIUM");
        fluctus_release_bus_power(POWER_BUS_5V, "IMPLUVIUM");
        fluctus_release_bus_power(POWER_BUS_3V3, "IMPLUVIUM");

        // Reset queue and state
        irrigation_system.queue_index = 0;
        irrigation_system.watering_queue_size = 0;
        irrigation_system.last_moisture_check = xTaskGetTickCount() * portTICK_PERIOD_MS;

        impluvium_change_state(IRRIGATION_MAINTENANCE);
    }

    return ESP_OK;
}

/**
 * @brief MAINTENANCE state - perform system maintenance tasks and emergency diagnostics
 *
 * Handles:
 * - Emergency diagnostics execution and analysis
 * - Daily volume resets
 * - NVS configuration saves
 */
static esp_err_t impluvium_state_maintenance(void)
{
    // If an emergency was triggered, start the diagnostic process.
    if (irrigation_system.emergency_stop) {
        ESP_LOGI(TAG, "Emergency detected, beginning diagnostics...");
        emergency_diagnostics_start(irrigation_system.emergency.failure_reason);
        irrigation_system.emergency_stop = false; // Reset trigger
        // emergency_diagnostics_start() sets the emergency state to TRIGGERED.
        // The state machine will loop and re-evaluate this function, entering the logic below.
        return ESP_OK;
    }

    // If we are in an emergency, the diagnostic logic will handle the flow.
    // If not, we perform normal maintenance and return to idle.
    if (irrigation_system.emergency.state != EMERGENCY_NONE) {
        // Convert emergency state to string for logging
        const char *emergency_state_str = "UNKNOWN";
        switch (irrigation_system.emergency.state) {
            case EMERGENCY_NONE: emergency_state_str = "NONE"; break;
            case EMERGENCY_TRIGGERED: emergency_state_str = "TRIGGERED"; break;
            case EMERGENCY_DIAGNOSING: emergency_state_str = "DIAGNOSING"; break;
            case EMERGENCY_USER_REQUIRED: emergency_state_str = "USER_REQUIRED"; break;
            case EMERGENCY_RESOLVED: emergency_state_str = "RESOLVED"; break;
        }

        ESP_LOGI(TAG, "Processing emergency diagnostics (state: %s)", emergency_state_str);

        switch (irrigation_system.emergency.state) {
            case EMERGENCY_TRIGGERED:
                // Start diagnostics by checking moisture levels
                ESP_LOGI(TAG, "Starting emergency diagnostics - checking moisture levels");
                emergency_diagnostics_check_moisture_levels();
                break;

            case EMERGENCY_DIAGNOSING:
                // Find next eligible zone to test
                uint8_t next_zone_to_test = IRRIGATION_ZONE_COUNT;
                for (uint8_t i = irrigation_system.emergency.test_zone; i < IRRIGATION_ZONE_COUNT; i++) {
                    if ((irrigation_system.emergency.eligible_zones_mask & (1 << i))) {
                        next_zone_to_test = i;
                        break;
                    }
                }

                if (next_zone_to_test < IRRIGATION_ZONE_COUNT) {
                    irrigation_system.emergency.test_zone = next_zone_to_test;
                    emergency_diagnostics_test_zone(irrigation_system.emergency.test_zone);
                    // Increment here to ensure we test the next eligible zone in the following cycle
                    irrigation_system.emergency.test_zone++;
                } else {
                    // All eligible zones tested - analyze results
                    emergency_diagnostics_analyze_results();
                }
                break;

            case EMERGENCY_USER_REQUIRED:
                ESP_LOGE(TAG, "Emergency diagnostics failed - USER INTERVENTION REQUIRED");
                ESP_LOGE(TAG, "Failure: %s", irrigation_system.emergency.failure_reason);
                ESP_LOGE(TAG, "Failed zones mask: 0x%02X", irrigation_system.emergency.failed_zones_mask);
                // Power off all buses to conserve power while waiting for user
                fluctus_release_bus_power(POWER_BUS_12V, "IMPLUVIUM");
                fluctus_release_bus_power(POWER_BUS_5V, "IMPLUVIUM");
                fluctus_release_bus_power(POWER_BUS_3V3, "IMPLUVIUM");
                // Stay in this state until user manually resets system. The task will block waiting for a notification.
                return ESP_OK;

            case EMERGENCY_RESOLVED:
                emergency_diagnostics_resolve();
                break;

            default:
                break;
        }
    } else {
        // Normal maintenance after a watering cycle
        ESP_LOGI(TAG, "Maintenance tasks completed, returning to IDLE");
        impluvium_change_state(IRRIGATION_STANDBY);
    }

    return ESP_OK;
}

// ########################## Irrigation System ##########################
// -------------------- Safety Monitoring Functions ----------------------

/**
 * @brief Perform one-time safety checks
 */
static esp_err_t impluvium_pre_check(void)
{
    // Check temperature
    float current_temperature = tempesta_get_temperature();

    if (current_temperature == WEATHER_INVALID_VALUE) {
        ESP_LOGE(TAG, "Pre-check failed: Temperature sensors unavailable");
        return ESP_FAIL;
    }
    
    // Global temperature safety limits (wider range for system protection)
    if (current_temperature < MIN_TEMPERATURE_GLOBAL || current_temperature > MAX_TEMPERATURE_GLOBAL) {
        ESP_LOGE(TAG,
                 "Pre-check failed: Temperature %.1f°C outside global safety range (%.1f°C to %.1f°C)",
                 current_temperature,
                 MIN_TEMPERATURE_GLOBAL,
                 MAX_TEMPERATURE_GLOBAL);
        impluvium_emergency_stop("Temperature outside global safety range");
        return ESP_FAIL;
    }
    
    // Watering-specific temperature limits (narrower range for optimal operation)
    if (current_temperature < MIN_TEMPERATURE_WATERING) {
        ESP_LOGW(TAG,
                 "Pre-check failed: Temperature %.1f°C is below watering minimum %.1f°C",
                 current_temperature,
                 MIN_TEMPERATURE_WATERING);
        return ESP_FAIL;
    }

    // Read current water level in tank
    impluvium_read_water_level(&irrigation_system.water_level);
    if (irrigation_system.water_level < MIN_WATER_LEVEL_PERCENT) {
        ESP_LOGW(TAG,
                 "Pre-check failed: Water level %.1f%% is below minimum %.1f%%",
                 irrigation_system.water_level,
                 MIN_WATER_LEVEL_PERCENT);
        return ESP_FAIL;
    }

    // Read current pressure
    if (impluvium_read_pressure(&irrigation_system.outlet_pressure) != ESP_OK) {
        ESP_LOGW(TAG, "Pre-check failed: Could not read system's outlet pressure.");
        return ESP_FAIL;
    }

    if (irrigation_system.outlet_pressure > MAX_PRESSURE_BAR) {
        ESP_LOGE(TAG,
                 "Pre-check failed: Outlet pressure %.2f bar is above maximum %.2f bar",
                 irrigation_system.outlet_pressure,
                 MAX_PRESSURE_BAR);
        impluvium_emergency_stop("System's outlet pressure too high");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG,
             "Safety pre-check complete: temp %.1f°C, water level %.1f %%, outlet pressure %.2f bar, allowed YES",
             current_temperature,
             irrigation_system.water_level,
             irrigation_system.outlet_pressure);

    return ESP_OK; // All checks passed
}

/**
 * @brief Update flow rate calculation
 */
static void impluvium_calc_flow_rate(const char *task_tag,
                                      uint32_t *last_pulse_count,
                                      uint32_t *last_flow_time,
                                      uint32_t current_time)
{
    int current_pulse_count;
    if (pcnt_unit_get_count(flow_pcnt_unit, &current_pulse_count) == ESP_OK) {
        if (*last_flow_time > 0 && current_time - *last_flow_time >= FLOW_RATE_CALC_PERIOD_MS) {
            uint32_t pulse_diff = current_pulse_count - *last_pulse_count;
            uint32_t time_diff_ms = current_time - *last_flow_time;

            // Calculate Liters per Second, then convert to Liters per Hour
            float flow_rate_lps = (pulse_diff / FLOW_CALIBRATION_PULSES_PER_LITER) / (time_diff_ms / 1000.0f);
            irrigation_system.current_flow_rate = flow_rate_lps * 3600.0f; // Convert L/s to L/h

            ESP_LOGD(task_tag,
                     "Flow rate updated: %lu pulses in %lums = %.1f L/h",
                     pulse_diff,
                     time_diff_ms,
                     irrigation_system.current_flow_rate);

            *last_pulse_count = current_pulse_count;
            *last_flow_time = current_time;
        }
    }
}

/**
 * @brief Perform smart watering cutoffs based on real-time conditions
 * @return ESP_OK if cutoff triggered, ESP_FAIL to continue watering
 */
static esp_err_t impluvium_watering_cutoffs_check(const char *task_tag, uint32_t time_since_start_ms)
{
    if (irrigation_system.active_zone >= IRRIGATION_ZONE_COUNT ||
        irrigation_system.queue_index >= irrigation_system.watering_queue_size) {
        return ESP_FAIL;
    }

    irrigation_zone_t *active_zone = &irrigation_zones[irrigation_system.active_zone];
    watering_queue_item_t *queue_item = &irrigation_system.watering_queue[irrigation_system.queue_index];

    // **MANUAL WATERING MODE**: Use time-based cutoff only, skip moisture checks
    if (irrigation_system.manual_watering_active) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Check if manual watering duration has elapsed
        if (current_time >= irrigation_system.manual_water_end_time) {
            ESP_LOGI(task_tag, "Manual watering time completed for zone %d (%d seconds)",
                     irrigation_system.active_zone, irrigation_system.manual_water_duration_sec);
            xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_WATERING_CUTOFF, eSetBits);
            return ESP_OK;
        }

        // Continue manual watering (pressure safety checks still performed in periodic_safety_check)
        return ESP_FAIL;
    }

    // Check maximum watering time hasn't been exceeded
    if (time_since_start_ms > MAX_WATERING_TIME_MS) {
        ESP_LOGW(TAG, "Zone %d max watering time reached - stopping", irrigation_system.active_zone);
        xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_WATERING_CUTOFF, eSetBits);
        return ESP_OK;
    }

    // Check if target pulses reached
    int pulse_count;
    if (pcnt_unit_get_count(flow_pcnt_unit, &pulse_count) == ESP_OK) {
        uint32_t pulses_used = pulse_count - irrigation_system.watering_start_pulses;

        if (pulses_used >= queue_item->target_pulses) {
            ESP_LOGI(task_tag,
                     "Zone %d target pulses reached: %lu >= %d",
                     irrigation_system.active_zone,
                     pulses_used,
                     queue_item->target_pulses);
            xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_WATERING_CUTOFF, eSetBits);
            return ESP_OK;
        }
    }

    // Check moisture level and safety margin
    float measured_moisture_percent;
    if (impluvium_read_moisture_sensor(irrigation_system.active_zone, &measured_moisture_percent) == ESP_OK) {
        // Calculate, store, and check the current moisture gain rate
        float moisture_increase = measured_moisture_percent - queue_item->moisture_at_start_percent;

        if (time_since_start_ms > 1000) { // Calculate after 1 second to get a meaningful rate
            irrigation_system.current_moisture_gain_rate = (moisture_increase / (time_since_start_ms / 1000.0f));

            // Check for anomalous spike after 2 seconds
            if (time_since_start_ms > 2000 &&
                irrigation_system.current_moisture_gain_rate > MOISTURE_SPIKE_RATE_THRESHOLD_PER_SEC) {
                ESP_LOGW(task_tag,
                         "Zone %d moisture spike detected: %.2f%%/sec increase - possible rain/manual watering",
                         irrigation_system.active_zone,
                         irrigation_system.current_moisture_gain_rate);
                irrigation_system.current_anomaly.type = ANOMALY_MOISTURE_SPIKE;
                irrigation_system.current_anomaly.anomaly_timestamp = time(NULL);
                irrigation_system.current_anomaly.expected_vs_actual = irrigation_system.current_moisture_gain_rate;
                xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_WATERING_CUTOFF, eSetBits);
                return ESP_OK; // Stop watering
            }
        } else {
            irrigation_system.current_moisture_gain_rate = 0.0f;
        }

        // Check if moisture reached target - with safety margin (target - 2%)
        float safety_target = active_zone->target_moisture_percent - 2.0f;
        if (measured_moisture_percent >= safety_target) {
            ESP_LOGI(task_tag,
                     "Zone %d safety margin reached: %.1f%% >= %.1f%%",
                     irrigation_system.active_zone,
                     measured_moisture_percent,
                     safety_target);
            xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_WATERING_CUTOFF, eSetBits);
            return ESP_OK;
        }

        // Store for anomaly detection
        irrigation_system.last_moisture_reading_percent = measured_moisture_percent;
    }

    return ESP_FAIL; // Continue watering
}

/**
 * @brief Perform comprehensive safety monitoring while watering
 * @return true if safe to continue, false if emergency stop was triggered
 */
static bool impluvium_periodic_safety_check(uint32_t *error_count, uint32_t time_since_start_ms)
{
    // Allow 4 consecutive errors before emergency stop (immediate if current exceeds limits set in FLUCTUS)
    const uint32_t MAX_ERROR_COUNT = 4;
    const char *failure_reason = NULL;

    // Read outlet pressure and level sensors
    esp_err_t ret = impluvium_read_pressure(&irrigation_system.outlet_pressure);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Safety check: Failed to read outlet pressure sensor: %s", esp_err_to_name(ret));
        failure_reason = "Outlet pressure sensor read failed";
    }
    ret = impluvium_read_water_level(&irrigation_system.water_level);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Safety check: Failed to read water level: %s", esp_err_to_name(ret));
        failure_reason = "Water level sensor read failed";
    }

    // Overcurrent monitoring delegated to Fluctus (power management) system.

    // Check safety parameters
    if (failure_reason == NULL) {
        if (irrigation_system.outlet_pressure > MAX_PRESSURE_BAR) {
            ESP_LOGW(TAG, "Outlet pressure too high: %.2f > %.2f bar", irrigation_system.outlet_pressure, MAX_PRESSURE_BAR);
            failure_reason = "Outlet pressure too high";
        }
        if (time_since_start_ms > 1000 && irrigation_system.current_flow_rate < MIN_FLOW_RATE_LH) {
            ESP_LOGW(TAG, "Flow rate too low: %.1f < %.1f L/h", irrigation_system.current_flow_rate, MIN_FLOW_RATE_LH);
            failure_reason = "Flow rate too low.";
        }
        if (irrigation_system.water_level < MIN_WATER_LEVEL_PERCENT - 3.0f) { // Stop watering below 2%
            ESP_LOGW(TAG, "Water level too low, currently at: %.1f %%", irrigation_system.water_level);
            failure_reason = "Water level critically low";
        }
    }

    // Handle safety failures
    if (failure_reason != NULL) {
        // Increment error count
        (*error_count)++;

        ESP_LOGW(TAG,
                 "Safety failure %" PRIu32 "/%" PRIu32 ": %s (Outlet pressure: %.2f, Flow: %.1f)",
                 *error_count,
                 (uint32_t) MAX_ERROR_COUNT,
                 failure_reason,
                 irrigation_system.outlet_pressure,
                 irrigation_system.current_flow_rate);

        if (*error_count >= MAX_ERROR_COUNT) {
            ESP_LOGE(TAG, "Maximum safety failures reached for: %s", failure_reason);
            impluvium_emergency_stop(failure_reason);
            return false; // Emergency triggered
        } else if (*error_count == 1) {
            ESP_LOGW(TAG, "Reducing pump speed due to safety issue");
            impluvium_set_pump_speed(PUMP_MIN_DUTY);
        }
    }

    return true; // Safe to continue
}

// ########################## Irrigation System ##########################
// ------------------- Emergency Diagnostics Functions -------------------

/**
 * @brief Emergency stop - immediate shutdown
 */
static esp_err_t impluvium_emergency_stop(const char *reason)
{
    ESP_LOGE(TAG, "EMERGENCY STOP TRIGGERED: %s - Immediate irrigation shutdown", reason);

    // Only trigger if not already in an emergency shutdown
    if (!irrigation_system.emergency_stop) {
        irrigation_system.emergency_stop = true;
        // Store the reason for the diagnostics phase
        irrigation_system.emergency.failure_reason = reason;
        // Signal the main task to handle the emergency
        xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_EMERGENCY_SHUTDOWN, eSetBits);
    }

    return ESP_OK;
}

/**
 * @brief Initialize emergency diagnostics system
 */
static esp_err_t emergency_diagnostics_init(void)
{
    // memset is redundant - static variables are zero-initialized
    irrigation_system.emergency.state = EMERGENCY_NONE;

    ESP_LOGI(TAG, "Emergency diagnostics system initialized");
    return ESP_OK;
}

/**
 * @brief Start emergency diagnostics with given reason
 */
static esp_err_t emergency_diagnostics_start(const char *reason)
{
    ESP_LOGW(TAG, "=== EMERGENCY DIAGNOSTICS INITIATED ===");
    ESP_LOGW(TAG, "Reason: %s", reason);

    // Power on sensors for the duration of the diagnostics
    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_3V3, "IMPLUVIUM_DIAG", return ESP_FAIL);
    
    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_5V, "IMPLUVIUM_DIAG", {
        fluctus_release_bus_power(POWER_BUS_3V3, "IMPLUVIUM_DIAG");
        return ESP_FAIL;
    });

    // Reset diagnostics state
    memset(&irrigation_system.emergency, 0, sizeof(emergency_diagnostics_t));
    irrigation_system.emergency.state = EMERGENCY_TRIGGERED;
    irrigation_system.emergency.diagnostic_start_time_ms = esp_timer_get_time() / 1000;  // Monotonic time
    irrigation_system.emergency.failure_reason = reason;

    // Transition to maintenance state to handle diagnostics
    impluvium_change_state(IRRIGATION_MAINTENANCE);

    ESP_LOGI(TAG, "Emergency diagnostics scheduled - switching to MAINTENANCE state");
    return ESP_OK;
}

/**
 * @brief Check moisture levels before starting diagnostics
 */
static esp_err_t emergency_diagnostics_check_moisture_levels(void)
{
    ESP_LOGI(TAG, "Checking moisture levels before diagnostics...");

    irrigation_system.emergency.eligible_zones_count = 0;
    irrigation_system.emergency.eligible_zones_mask = 0;

    for (uint8_t zone_id = 0; zone_id < IRRIGATION_ZONE_COUNT; zone_id++) {
        if (!irrigation_zones[zone_id].watering_enabled)
            continue;

        float moisture_percent;
        if (impluvium_read_moisture_sensor(zone_id, &moisture_percent) == ESP_OK) {
            ESP_LOGI(TAG, "Zone %d moisture: %.1f%%", zone_id, moisture_percent);

            if (moisture_percent < EMERGENCY_MOISTURE_THRESHOLD) {
                ESP_LOGI(TAG,
                         "Zone %d is eligible for testing (%.1f%% < %.1f%%)",
                         zone_id,
                         moisture_percent,
                         EMERGENCY_MOISTURE_THRESHOLD);
                irrigation_system.emergency.eligible_zones_count++;
                irrigation_system.emergency.eligible_zones_mask |= (1 << zone_id);
            }
        } else {
            ESP_LOGE(TAG, "Failed to read moisture for zone %d", zone_id);
        }
    }

    if (irrigation_system.emergency.eligible_zones_count >= 2) {
        irrigation_system.emergency.initial_moisture_check_passed = true;

        ESP_LOGI(TAG,
                 "Moisture levels sufficient for diagnostics (%d eligible zones) - proceeding",
                 irrigation_system.emergency.eligible_zones_count);
        irrigation_system.emergency.state = EMERGENCY_DIAGNOSING;
        irrigation_system.emergency.test_zone = 0; // Start search from zone 0
        irrigation_system.emergency.test_cycle_count = 0;
    } else {
        ESP_LOGE(TAG,
                 "Not enough eligible zones (%d < 2) for diagnostics - manual intervention required",
                 irrigation_system.emergency.eligible_zones_count);
        irrigation_system.emergency.state = EMERGENCY_USER_REQUIRED;
        irrigation_system.emergency.failure_reason = "Moisture levels too high for diagnostic testing";
    }

    return ESP_OK;
}

/**
 * @brief Test a specific zone with short watering cycle
 */
static esp_err_t emergency_diagnostics_test_zone(uint8_t zone_id)
{
    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_12V, "IMPLUVIUM_DIAG", return ESP_FAIL);
    
    if (zone_id >= IRRIGATION_ZONE_COUNT) {
        ESP_LOGE(TAG, "Invalid zone for diagnostics: %d", zone_id);
        return ESP_ERR_INVALID_ARG;
    }

    irrigation_zone_t *zone = &irrigation_zones[zone_id];

    ESP_LOGI(TAG,
             "Testing zone %d (diagnostic cycle %d)...",
             zone_id,
             irrigation_system.emergency.test_cycle_count + 1);

    // Open valve for this zone
    gpio_set_level(zone->valve_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(VALVE_OPEN_DELAY_MS)); // Wait for valve to open

    // Start pump at reduced speed for safety
    esp_err_t ret = impluvium_set_pump_speed(PUMP_MIN_DUTY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start pump for zone %d test", zone_id);
        gpio_set_level(zone->valve_gpio, 0);
        irrigation_system.emergency.failed_zones_mask |= (1 << zone_id);
        irrigation_system.emergency.test_flow_rates[zone_id] = 0.0f;
        irrigation_system.emergency.test_pressures[zone_id] = 0.0f;
        return ESP_FAIL;
    }

    // Record test start time
    irrigation_system.emergency.test_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Reset flow counter
    pcnt_unit_clear_count(flow_pcnt_unit);

    // Wait for test duration, checking for overcurrent midway
    vTaskDelay(pdMS_TO_TICKS(EMERGENCY_TEST_DURATION_MS / 5));

    // Overcurrent monitoring delegated to Fluctus (power management) system.

    vTaskDelay(pdMS_TO_TICKS(EMERGENCY_TEST_DURATION_MS));

    // Measure results
    int test_pulses;
    pcnt_unit_get_count(flow_pcnt_unit, &test_pulses);
    float test_volume_ml = (test_pulses / FLOW_CALIBRATION_PULSES_PER_LITER) * 1000.0f;
    float test_flow_rate = (test_volume_ml / 1000.0f) * (3600.0f / (EMERGENCY_TEST_DURATION_MS / 1000.0f)); // L/h

    // Read outlet pressure
    float test_pressure;
    if (impluvium_read_pressure(&test_pressure) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read outlet pressure during diagnostic test for zone %d", zone_id);
        test_pressure = -1.0f; // Indicate failure
    }

    // Stop pump and close valve
    impluvium_set_pump_speed(0);
    vTaskDelay(pdMS_TO_TICKS(PRESSURE_EQUALIZE_DELAY_MS)); // Wait for outlet pressure to drop
    gpio_set_level(zone->valve_gpio, 0);
    fluctus_release_bus_power(POWER_BUS_12V, "IMPLUVIUM_DIAG");

    // Store test results
    irrigation_system.emergency.test_flow_rates[zone_id] = test_flow_rate;
    irrigation_system.emergency.test_pressures[zone_id] = test_pressure;

    // Analyze test results
    bool test_passed = true;
    const char *failure_reason = "";

    if (test_flow_rate < EMERGENCY_TEST_MIN_FLOW_RATE) {
        test_passed = false;
        failure_reason = "Low flow rate";
        ESP_LOGW(TAG,
                 "Zone %d FAILED: Low flow rate %.1f L/h < %.1f L/h",
                 zone_id,
                 test_flow_rate,
                 EMERGENCY_TEST_MIN_FLOW_RATE);
    }

    if (test_pressure > EMERGENCY_TEST_MAX_PRESSURE) {
        test_passed = false;
        failure_reason = "High outlet pressure";
        ESP_LOGW(TAG,
                 "Zone %d FAILED: High outlet pressure %.2f bar > %.2f bar",
                 zone_id,
                 test_pressure,
                 EMERGENCY_TEST_MAX_PRESSURE);
    }

    if (test_passed) {
        ESP_LOGI(TAG,
                 "Zone %d PASSED: Flow %.1f L/h, Outlet pressure %.2f bar, Volume %.1f mL",
                 zone_id,
                 test_flow_rate,
                 test_pressure,
                 test_volume_ml);
    } else {
        ESP_LOGE(TAG,
                 "Zone %d FAILED: %s (Flow %.1f L/h, Outlet pressure %.2f bar)",
                 zone_id,
                 failure_reason,
                 test_flow_rate,
                 test_pressure);
        irrigation_system.emergency.failed_zones_mask |= (1 << zone_id);
    }

    // Move to next zone
    irrigation_system.emergency.test_cycle_count++;

    // Small delay between tests
    vTaskDelay(pdMS_TO_TICKS(2000));

    return test_passed ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Analyze diagnostic test results and determine next action
 */
static esp_err_t emergency_diagnostics_analyze_results(void)
{
    ESP_LOGI(TAG, "Analyzing emergency diagnostic results...");

    uint8_t failed_zones = 0;

    // Count failures among eligible zones
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        // Only consider zones that were eligible for testing
        if (irrigation_system.emergency.eligible_zones_mask & (1 << i)) {
            if (irrigation_system.emergency.failed_zones_mask & (1 << i)) {
                failed_zones++;
                ESP_LOGW(TAG,
                         "Eligible Zone %d FAILED: Flow %.1f L/h, Outlet pressure %.2f bar",
                         i,
                         irrigation_system.emergency.test_flow_rates[i],
                         irrigation_system.emergency.test_pressures[i]);
            } else {
                ESP_LOGI(TAG,
                         "Eligible Zone %d PASSED: Flow %.1f L/h, Outlet pressure %.2f bar",
                         i,
                         irrigation_system.emergency.test_flow_rates[i],
                         irrigation_system.emergency.test_pressures[i]);
            }
        }
    }

    ESP_LOGI(TAG,
             "Diagnostic summary: %d/%d eligible zones failed",
             failed_zones,
             irrigation_system.emergency.eligible_zones_count);

    // Determine course of action
    if (failed_zones == 0) {
        // All eligible zones passed - system appears healthy
        ESP_LOGI(TAG, "All eligible zones passed diagnostics - system appears healthy");
        irrigation_system.emergency.state = EMERGENCY_RESOLVED;
        irrigation_system.emergency.failure_reason = "Auto-recovered: All diagnostic tests passed";

    } else if (failed_zones == irrigation_system.emergency.eligible_zones_count) {
        // All eligible zones failed - system-wide issue
        ESP_LOGE(TAG, "All eligible zones failed - system-wide issue detected");
        irrigation_system.emergency.consecutive_failures++;

        if (irrigation_system.emergency.consecutive_failures >= EMERGENCY_MAX_CONSECUTIVE_FAILS) {
            irrigation_system.emergency.state = EMERGENCY_USER_REQUIRED;
            irrigation_system.emergency.failure_reason = "System-wide failure: pump, filter, or supply issue";
        } else {
            ESP_LOGW(TAG,
                     "Retrying diagnostics (attempt %d/%d)",
                     irrigation_system.emergency.consecutive_failures,
                     EMERGENCY_MAX_CONSECUTIVE_FAILS);
            // Reset for another diagnostic cycle
            irrigation_system.emergency.state = EMERGENCY_TRIGGERED;
            irrigation_system.emergency.test_zone = 0;
            irrigation_system.emergency.failed_zones_mask = 0;
        }

    } else {
        // Some zones failed - zone-specific issues
        ESP_LOGW(TAG, "Partial failure - %d zones have issues", failed_zones);

        if (failed_zones <= (irrigation_system.emergency.eligible_zones_count / 2)) {
            // Less than half failed - disable failed zones and continue
            ESP_LOGI(TAG, "Disabling failed zones and continuing operation");
            for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
                if (irrigation_system.emergency.failed_zones_mask & (1 << i)) {
                    irrigation_zones[i].watering_enabled = false;
                    ESP_LOGW(TAG, "Zone %d disabled due to diagnostic failure", i);
                }
            }
            irrigation_system.emergency.state = EMERGENCY_RESOLVED;
            irrigation_system.emergency.failure_reason = "Partial recovery: some zones disabled";
        } else {
            // More than half failed - requires manual intervention
            irrigation_system.emergency.state = EMERGENCY_USER_REQUIRED;
            irrigation_system.emergency.failure_reason = "Too many zone failures for automatic recovery";
        }
    }

    return ESP_OK;
}

/**
 * @brief Resolve emergency diagnostics and return to normal operation
 */
static esp_err_t emergency_diagnostics_resolve(void)
{
    int64_t diagnostic_duration_ms = (esp_timer_get_time() / 1000) - irrigation_system.emergency.diagnostic_start_time_ms;
    int64_t diagnostic_duration_seconds = diagnostic_duration_ms / 1000;

    ESP_LOGW(TAG, "=== EMERGENCY DIAGNOSTICS COMPLETED ===");
    ESP_LOGI(TAG, "Resolution: %s", irrigation_system.emergency.failure_reason);
    ESP_LOGI(TAG, "Duration: %lld seconds", (long long) diagnostic_duration_seconds);
    ESP_LOGI(TAG, "Failed zones mask: 0x%02X", irrigation_system.emergency.failed_zones_mask);

    // Log detailed results for maintenance records
    ESP_LOGI(TAG, "Diagnostic test results:");
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        if (irrigation_zones[i].watering_enabled || (irrigation_system.emergency.failed_zones_mask & (1 << i))) {
            ESP_LOGI(TAG,
                     "  Zone %d: Flow %.1f L/h, Outlet pressure %.2f bar, %s",
                     i,
                     irrigation_system.emergency.test_flow_rates[i],
                     irrigation_system.emergency.test_pressures[i],
                     (irrigation_system.emergency.failed_zones_mask & (1 << i)) ? "FAILED" : "PASSED");
        }
    }

    // Power off sensors now that diagnostics are complete
    fluctus_release_bus_power(POWER_BUS_3V3, "IMPLUVIUM_DIAG");
    fluctus_release_bus_power(POWER_BUS_5V, "IMPLUVIUM_DIAG");

    // Clear emergency state
    irrigation_system.emergency.state = EMERGENCY_NONE;

    // Return to normal operation
    ESP_LOGI(TAG, "Returning to normal irrigation operation");
    xTimerStart(xMoistureCheckTimer, 0); // Restart periodic checks
    impluvium_change_state(IRRIGATION_STANDBY);

    return ESP_OK;
}

// -----------------#################################-----------------
// -----------------############# TASKS #############-----------------
// -----------------#################################-----------------

/**
 * @brief Timer callback for moisture checking with dynamic temperature-based intervals
 */
static void vTimerCallbackMoistureCheck(TimerHandle_t xTimer)
{
    // Get current temperature to determine next interval
    float current_temperature = tempesta_get_temperature();
    uint32_t next_interval_ms = impluvium_calc_moisture_check_interval(current_temperature);

    // If temperature is too low, skip moisture checks entirely
    if (next_interval_ms == UINT32_MAX) {
        ESP_LOGI("MoistureTimer", "Skipping moisture check due to low temperature (%.1f°C)", current_temperature);
        // Set timer to check again in 1 hour
        next_interval_ms = 60 * 60 * 1000;
        return; // Don't trigger moisture check
    }

    // Update timer interval if it has changed
    TickType_t current_period = xTimerGetPeriod(xTimer);
    TickType_t new_period = pdMS_TO_TICKS(next_interval_ms);

    if (current_period != new_period) {
        ESP_LOGI("MoistureTimer",
                 "Updating moisture check interval: %" PRIu32 "min (temp: %.1f°C)",
                 next_interval_ms / (60 * 1000),
                 current_temperature);
        xTimerChangePeriod(xTimer, new_period, 0);
    }

    // Notify the main irrigation task to perform moisture check
    xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_REQUEST_MOISTURE_CHECK, eSetBits);
}

/**
 * @brief Main irrigation control task (State Machine)
 */
void impluvium_task(void *pvParameters)
{
    const char *task_tag = "irrigation";
    ESP_LOGI(task_tag, "IMPLUVIUM irrigation task started");

    // Wait for other systems to initialize
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(task_tag, "Starting irrigation state machine in IDLE state");

    // Start the periodic moisture check timer
    xTimerStart(xMoistureCheckTimer, 0);

    while (1) {
        uint32_t notification_value = 0;
        // Wait indefinitely for a notification
        xTaskNotifyWait(0x00,                /* Don't clear any bits on entry */
                        ULONG_MAX,           /* Clear all bits on exit */
                        &notification_value, /* Receives the notification value */
                        portMAX_DELAY);      /* Block indefinitely */

        // Take mutex to ensure exclusive access to the state machine
        if (xSemaphoreTake(xIrrigationMutex, portMAX_DELAY) == pdTRUE) {
            // --- Event Handling ---
            if (notification_value & IRRIGATION_TASK_NOTIFY_REQUEST_MOISTURE_CHECK) {
                ESP_LOGI(task_tag, "Notification: Moisture check requested");
                if (irrigation_system.load_shed_shutdown) {
                    ESP_LOGD(task_tag, "Load shedding shutdown active - skipping moisture check");
                } else if (irrigation_system.state == IRRIGATION_STANDBY) {
                    impluvium_change_state(IRRIGATION_MEASURING);
                }
            }
            if (notification_value & IRRIGATION_TASK_NOTIFY_WATERING_CUTOFF) {
                ESP_LOGI(task_tag, "Notification: Watering cutoff requested");
                if (irrigation_system.state == IRRIGATION_WATERING) {
                    impluvium_change_state(IRRIGATION_STOPPING);
                }
            }
            if (notification_value & IRRIGATION_TASK_NOTIFY_EMERGENCY_SHUTDOWN) {
                ESP_LOGE(task_tag, "Notification: Emergency shutdown requested");
                // Stop the moisture check timer during an emergency
                xTimerStop(xMoistureCheckTimer, 0);
                // The emergency_stop function has already set the reason/flag. We just change state.
                if (irrigation_system.state == IRRIGATION_WATERING) {
                    impluvium_change_state(IRRIGATION_STOPPING);
                } else {
                    impluvium_change_state(IRRIGATION_MAINTENANCE);
                }
            }
            if (notification_value & IRRIGATION_TASK_NOTIFY_MIDNIGHT_RESET) {
                ESP_LOGI(task_tag, "Notification: Midnight reset");
                impluvium_handle_midnight_reset();
            }
            if (notification_value & IRRIGATION_TASK_NOTIFY_MANUAL_WATER) {
                ESP_LOGI(task_tag, "Notification: Manual watering requested (zone %d, %ds)",
                         irrigation_system.manual_water_zone + 1,
                         irrigation_system.manual_water_duration_sec);
                // State already changed to MEASURING by API function
                // State machine will execute below and process it
            }

            // --- State Machine Execution ---
            // We loop here to handle sequential state transitions within one task activation
            bool state_changed;
            do {
                state_changed = false;
                irrigation_state_t current_state = irrigation_system.state;

                switch (current_state) {
                    case IRRIGATION_STANDBY:
                        // Do nothing, wait for notification
                        break;

                    case IRRIGATION_DISABLED:
                        // System disabled - do nothing, ignore moisture check notifications
                        break;

                    case IRRIGATION_MEASURING:
                        impluvium_state_measuring();
                        break;

                    case IRRIGATION_WATERING:
                        // If we are entering the watering state, start the sequence.
                        // Otherwise, do nothing and let the monitoring task handle it.
                        if (irrigation_system.active_zone == NO_ACTIVE_ZONE_ID) {
                            impluvium_state_watering();
                        }
                        break;

                    case IRRIGATION_STOPPING:
                        impluvium_state_stopping();
                        break;

                    case IRRIGATION_MAINTENANCE:
                        impluvium_state_maintenance();
                        break;

                    default:
                        ESP_LOGE(task_tag, "Unknown irrigation state: %d", irrigation_system.state);
                        impluvium_change_state(IRRIGATION_STANDBY);
                        break;
                }
                // If the state changed during execution, loop to process the new state immediately
                if (current_state != irrigation_system.state) {
                    state_changed = true;
                }
            } while (state_changed);

            xSemaphoreGive(xIrrigationMutex);
        }
    }
}

/**
 * @brief Event-driven monitoring task with real-time watering control
 *
 * This task operates in different modes based on task notifications.
 * During continuous monitoring, performs real-time:
 * - Flow rate calculation and validation
 * - Moisture level monitoring with smart cutoffs
 * - Safety checks and emergency stop triggers
 * - Anomaly detection (rain, manual watering, etc.)
 */
static void impluvium_monitoring_task(void *pvParameters)
{
    const char *task_tag = "Irrigation Safety Monitor";
    ESP_LOGI(task_tag, "Irrigation monitoring task started");

    // Monitoring state variables
    static bool continuous_monitoring = false;
    static uint32_t error_count = 0;
    static uint32_t last_pulse_count = 0;
    static uint32_t last_flow_measurement_time = 0;

    while (1) {
        uint32_t notification_value = 0;
        // Wait for a notification to trigger action
        xTaskNotifyWait(0x00,
                        ULONG_MAX,
                        &notification_value,
                        continuous_monitoring ? pdMS_TO_TICKS(MONITORING_INTERVAL_MS) : portMAX_DELAY);

        // Handle specific notifications
        if (notification_value & MONITORING_TASK_NOTIFY_START_MONITORING) {
            ESP_LOGI(task_tag, "Starting continuous monitoring for watering");
            continuous_monitoring = true;
            error_count = 0;
            last_flow_measurement_time = 0; // Reset flow measurement
            pcnt_unit_get_count(flow_pcnt_unit, (int *) &last_pulse_count);
        }

        if (notification_value & MONITORING_TASK_NOTIFY_STOP_MONITORING) {
            ESP_LOGI(task_tag, "Stopping continuous monitoring");
            continuous_monitoring = false;
        }

        // Continuous monitoring only during watering
        if (continuous_monitoring) {
            // Take mutex before accessing any shared data
            if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(10)) == pdTRUE) {

                // Double-check state inside mutex to handle race conditions
                if (irrigation_system.state != IRRIGATION_WATERING) {
                    xSemaphoreGive(xIrrigationMutex);
                    continuous_monitoring = false; // State changed, stop monitoring
                    continue;
                }

                uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                uint32_t time_since_start_ms =
                    current_time - irrigation_system.watering_start_time;

                // Update flow rate
                impluvium_calc_flow_rate(task_tag, &last_pulse_count, &last_flow_measurement_time, current_time);

                // Safety monitoring
                if (!impluvium_periodic_safety_check(&error_count, time_since_start_ms)) {
                    // Emergency condition detected. The stop function notifies the main task.
                    continuous_monitoring = false;
                }

                // Real-time moisture monitoring and smart cutoffs
                impluvium_watering_cutoffs_check(task_tag, time_since_start_ms);

                // Use the gain rate calculated in the check function for adaptive control
                uint8_t zone_id = irrigation_system.active_zone;
                if (zone_id < IRRIGATION_ZONE_COUNT) {
                    impluvium_pump_adaptive_control(irrigation_system.current_moisture_gain_rate,
                                                     irrigation_zones[zone_id].learning.target_moisture_gain_rate);
                }

                // Release mutex before calling telemetry (avoid deadlock)
                xSemaphoreGive(xIrrigationMutex);

                // Update TELEMETRY realtime cache OUTSIDE mutex to avoid deadlock
                // (telemetry will take its own mutex, then call back to irrigation)
                if (telemetry_is_realtime_enabled()) {
                    esp_err_t ret = telemetry_fetch_snapshot(TELEMETRY_SRC_IMPLUVIUM_RT);
                    if (ret != ESP_OK) {
                        ESP_LOGW(task_tag, "Failed to update TELEMETRY realtime cache: %s", esp_err_to_name(ret));
                    }
                }

            } else {
                ESP_LOGW(task_tag, "Failed to get mutex for monitoring cycle");
            }
        }
    }
}

// ########################## Daily Reset Callback ##########################

/**
 * @brief Daily reset callback for midnight counter resets and learning data backup
 *
 * Called by TELEMETRY when a new day is detected (via solar_calc day change).
 * Resets daily counters (volume_used_today) for all zones and saves learning data to LittleFS.
 */
// ########################## Accumulator Functions ##########################

/**
 * @brief Update irrigation accumulator with water usage from completed watering event
 *
 * Called after each watering event completes. Updates per-zone hourly and daily totals.
 * Uses time-independent accumulation - safe to call at any interval.
 *
 * @param zone_id Zone that was watered (0-4)
 * @param water_used_ml Water volume used in this watering event (mL)
 */
static void impluvium_update_accumulator(uint8_t zone_id, float water_used_ml)
{
    if (zone_id >= IRRIGATION_ZONE_COUNT) {
        ESP_LOGW(TAG, "Invalid zone_id %d in accumulator update", zone_id);
        return;
    }

    // Initialize accumulator on first call or after power loss
    if (!rtc_accumulator.initialized || rtc_accumulator.current_hour_start == 0) {
        time_t now = time(NULL);
        ESP_LOGI(TAG, "Initializing RTC irrigation accumulator (first boot or power loss)");
        rtc_accumulator.current_hour_start = now;
        rtc_accumulator.current_day_start = now;

        // Zero all per-zone counters
        for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
            rtc_accumulator.zone_water_used_hour_ml[i] = 0.0f;
            rtc_accumulator.zone_events_hour[i] = 0;
            rtc_accumulator.zone_water_used_day_ml[i] = 0.0f;
            rtc_accumulator.zone_events_day[i] = 0;
        }

        rtc_accumulator.initialized = true;
    }

    // Accumulate water usage for this zone
    rtc_accumulator.zone_water_used_hour_ml[zone_id] += water_used_ml;
    rtc_accumulator.zone_events_hour[zone_id]++;

    ESP_LOGD(TAG, "Zone %d accumulator: +%.1f mL, zone hour total: %.1f mL, events: %d",
             zone_id, water_used_ml,
             rtc_accumulator.zone_water_used_hour_ml[zone_id],
             rtc_accumulator.zone_events_hour[zone_id]);
}

/**
 * @brief Check for hourly rollover and update daily totals
 *
 * Called periodically during watering operations.
 * When hour changes, adds per-zone hourly totals to daily accumulator and resets hourly counters.
 *
 * @return true if hour changed (rollover occurred), false otherwise
 */
static bool impluvium_check_hourly_rollover(void)
{
    if (!rtc_accumulator.initialized) {
        return false;
    }

    time_t now = time(NULL);
    struct tm tm_now, tm_hour_start;
    gmtime_r(&now, &tm_now);
    gmtime_r(&rtc_accumulator.current_hour_start, &tm_hour_start);

    // Check if hour changed
    if (tm_now.tm_hour != tm_hour_start.tm_hour ||
        tm_now.tm_yday != tm_hour_start.tm_yday ||
        tm_now.tm_year != tm_hour_start.tm_year) {

        // Calculate system totals for logging
        float total_hour_ml = 0.0f;
        uint8_t total_events = 0;

        // Update daily totals with completed hour data (per zone)
        for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
            rtc_accumulator.zone_water_used_day_ml[i] += rtc_accumulator.zone_water_used_hour_ml[i];
            rtc_accumulator.zone_events_day[i] += rtc_accumulator.zone_events_hour[i];

            total_hour_ml += rtc_accumulator.zone_water_used_hour_ml[i];
            total_events += rtc_accumulator.zone_events_hour[i];

            // Reset hourly counters for this zone
            rtc_accumulator.zone_water_used_hour_ml[i] = 0.0f;
            rtc_accumulator.zone_events_hour[i] = 0;
        }

        ESP_LOGI(TAG, "Hourly rollover: %.1f mL used, %d events (all zones)", total_hour_ml, total_events);

        // Reset hour timestamp
        rtc_accumulator.current_hour_start = now;

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
static void impluvium_handle_midnight_reset(void)
{
    ESP_LOGI(TAG, "=== Midnight Reset: Daily Irrigation Summary ===");

    // Calculate system totals for logging
    float total_day_ml = 0.0f;
    uint8_t total_events = 0;

    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        total_day_ml += rtc_accumulator.zone_water_used_day_ml[i];
        total_events += rtc_accumulator.zone_events_day[i];

        if (rtc_accumulator.zone_water_used_day_ml[i] > 0.0f) {
            ESP_LOGI(TAG, "  Zone %d: %.1f mL, %d events",
                     i, rtc_accumulator.zone_water_used_day_ml[i], rtc_accumulator.zone_events_day[i]);
        }
    }

    ESP_LOGI(TAG, "  System total: %.1f mL, %d events", total_day_ml, total_events);
    ESP_LOGI(TAG, "==================================================");

    // Reset accumulator daily counters
    rtc_accumulator.current_day_start = time(NULL);
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        rtc_accumulator.zone_water_used_day_ml[i] = 0.0f;
        rtc_accumulator.zone_events_day[i] = 0;
    }
    ESP_LOGI(TAG, "RTC accumulator daily counters reset");

    // Save learning data backup (once per day at midnight)
    ESP_LOGI(TAG, "Saving daily learning data backup");
    esp_err_t ret = impluvium_save_learning_data_all_zones();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save learning data: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief Daily reset callback - called at midnight by solar_calc
 *
 * Notifies main task to perform reset, save learning data, and log daily summary.
 * All actual work is delegated to impluvium_task for thread-safety.
 */
static void impluvium_daily_reset_callback(void)
{
    ESP_LOGI(TAG, "Midnight callback received, notifying main task");

    // Delegate all work to main task for proper mutex protection
    if (xIrrigationTaskHandle != NULL) {
        xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_MIDNIGHT_RESET, eSetBits);
    }
}

// ########################## Public API Functions ##########################

/**
 * @brief Write irrigation data directly to TELEMETRY cache
 *
 * Comprehensive snapshot for HMI/MQTT distribution with all fields.
 * Writes directly to TELEMETRY's cache buffer to eliminate intermediate copy.
 * Only IMPLUVIUM mutex needed - TELEMETRY lock/unlock handled by caller.
 *
 * @param cache Pointer to TELEMETRY's impluvium_snapshot_t buffer
 * @return ESP_OK on success, ESP_FAIL if mutex timeout
 */
esp_err_t impluvium_write_to_telemetry_cache(impluvium_snapshot_t *cache)
{
    if (cache == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    // System state
    cache->state = irrigation_system.state;
    cache->active_zone = irrigation_system.active_zone;
    cache->emergency_stop = irrigation_system.emergency_stop;
    cache->power_save_mode = irrigation_system.power_save_mode;
    cache->load_shed_shutdown = irrigation_system.load_shed_shutdown;
    cache->snapshot_timestamp = time(NULL);

    // Physical sensors
    cache->water_level_percent = irrigation_system.water_level;

    // Hourly/daily statistics from RTC accumulator (calculate system totals from zones)
    cache->current_hour_start = rtc_accumulator.current_hour_start;

    float total_hour_ml = 0.0f;
    float total_day_ml = 0.0f;
    uint8_t events_hour = 0;
    uint8_t events_day = 0;

    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        total_hour_ml += rtc_accumulator.zone_water_used_hour_ml[i];
        total_day_ml += rtc_accumulator.zone_water_used_day_ml[i];
        events_hour += rtc_accumulator.zone_events_hour[i];
        events_day += rtc_accumulator.zone_events_day[i];
    }

    cache->total_water_used_hour_ml = total_hour_ml;
    cache->total_water_used_day_ml = total_day_ml;
    cache->watering_events_hour = events_hour;
    cache->watering_events_day = events_day;

    // Per-zone data (comprehensive - all 5 zones)
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        cache->zones[i].watering_enabled = irrigation_zones[i].watering_enabled;

        // TODO: Store last moisture reading in zone structure
        cache->zones[i].current_moisture_percent = 0.0f;

        cache->zones[i].target_moisture_percent = irrigation_zones[i].target_moisture_percent;
        cache->zones[i].moisture_deadband_percent = irrigation_zones[i].moisture_deadband_percent;

        // Read per-zone hourly/daily statistics from RTC accumulator (persistent!)
        cache->zones[i].volume_used_hour_ml = rtc_accumulator.zone_water_used_hour_ml[i];
        cache->zones[i].volume_used_today_ml = rtc_accumulator.zone_water_used_day_ml[i];
        cache->zones[i].events_hour = rtc_accumulator.zone_events_hour[i];
        cache->zones[i].events_day = rtc_accumulator.zone_events_day[i];

        // Calculate average hourly consumption since midnight
        time_t now = time(NULL);
        float hours_elapsed = (float)(now - rtc_accumulator.current_day_start) / 3600.0f;
        if (hours_elapsed > 0.0f) {
            cache->zones[i].avg_hourly_consumption_ml = rtc_accumulator.zone_water_used_day_ml[i] / hours_elapsed;
        } else {
            cache->zones[i].avg_hourly_consumption_ml = 0.0f;
        }

        // Convert monotonic milliseconds to time_t
        cache->zones[i].last_watered_time = (time_t)(irrigation_zones[i].last_watered_time_ms / 1000);

        // Learning algorithm data
        cache->zones[i].calculated_ppmp_ratio = irrigation_zones[i].learning.calculated_ppmp_ratio;
        cache->zones[i].calculated_pump_duty = irrigation_zones[i].learning.calculated_pump_duty_cycle;
        cache->zones[i].target_moisture_gain_rate = irrigation_zones[i].learning.target_moisture_gain_rate;
        cache->zones[i].confidence_level = irrigation_zones[i].learning.confidence_level;
        cache->zones[i].successful_predictions = irrigation_zones[i].learning.successful_predictions;
        cache->zones[i].total_predictions = irrigation_zones[i].learning.total_predictions;
        cache->zones[i].history_entry_count = irrigation_zones[i].learning.history_entry_count;
        cache->zones[i].last_temperature_correction = irrigation_zones[i].learning.last_temperature_correction;
    }

    // Emergency diagnostics
    cache->emergency_state = irrigation_system.emergency.state;
    cache->emergency_test_zone = irrigation_system.emergency.test_zone;
    cache->emergency_failed_zones_mask = irrigation_system.emergency.failed_zones_mask;
    cache->consecutive_failures = irrigation_system.emergency.consecutive_failures;
    cache->emergency_failure_reason = irrigation_system.emergency.failure_reason;

    // Anomaly tracking
    cache->current_anomaly_type = irrigation_system.current_anomaly.type;
    cache->anomaly_timestamp = irrigation_system.current_anomaly.anomaly_timestamp;

    xSemaphoreGive(xIrrigationMutex);

    return ESP_OK;
}

/**
 * @brief Write high-frequency irrigation data to TELEMETRY realtime cache
 *
 * Lightweight snapshot with only fast-changing sensor values.
 * Called every 500ms during WATERING state for real-time monitoring.
 * Writes directly to TELEMETRY's cache buffer to eliminate intermediate copy.
 * Only IMPLUVIUM mutex needed - TELEMETRY lock/unlock handled by caller.
 *
 * @param cache Pointer to TELEMETRY's impluvium_snapshot_rt_t buffer
 * @return ESP_OK on success, ESP_FAIL if mutex timeout
 */
esp_err_t impluvium_write_realtime_to_telemetry_cache(impluvium_snapshot_rt_t *cache)
{
    if (cache == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Fast-changing sensor readings (updated every 500ms during watering)
    cache->water_level_percent = irrigation_system.water_level;
    cache->outlet_pressure_bar = irrigation_system.outlet_pressure;
    cache->current_flow_rate_lh = irrigation_system.current_flow_rate;
    cache->current_moisture_gain_rate = irrigation_system.current_moisture_gain_rate;

    // Current operation status
    cache->pump_pwm_duty = irrigation_system.pump_pwm_duty;
    cache->active_zone = irrigation_system.active_zone;

    // Calculate pump duty percentage (0-100%)
    if (irrigation_system.pump_pwm_duty <= PUMP_MIN_DUTY) {
        cache->pump_duty_percent = 0;
    } else if (irrigation_system.pump_pwm_duty >= PUMP_MAX_DUTY) {
        cache->pump_duty_percent = 100;
    } else {
        uint32_t range = PUMP_MAX_DUTY - PUMP_MIN_DUTY;
        uint32_t offset = irrigation_system.pump_pwm_duty - PUMP_MIN_DUTY;
        cache->pump_duty_percent = (uint8_t)(((float)offset / (float)range) * 100.0f);
    }

    // System status flags
    cache->sensors_powered = irrigation_system.sensors_powered;
    cache->sensor_data_valid = irrigation_system.sensors_powered;  // Valid when sensors powered

    // Pressure/flow alarms (range checking)
    cache->pressure_alarm = (irrigation_system.outlet_pressure > MAX_PRESSURE_BAR) ||
                            (irrigation_system.outlet_pressure < 0.0f);
    cache->flow_alarm = (irrigation_system.current_flow_rate < MIN_FLOW_RATE_LH) &&
                        (irrigation_system.state == IRRIGATION_WATERING);

    // Watering queue (only first item for lightweight realtime snapshot)
    cache->watering_queue_size = irrigation_system.watering_queue_size;
    cache->queue_index = irrigation_system.queue_index;

    if (irrigation_system.watering_queue_size > 0 && irrigation_system.queue_index < irrigation_system.watering_queue_size) {
        uint8_t idx = irrigation_system.queue_index;
        cache->queue[0].zone_id = irrigation_system.watering_queue[idx].zone_id;
        cache->queue[0].measured_moisture_percent = irrigation_system.watering_queue[idx].measured_moisture_percent;
        cache->queue[0].moisture_deficit_percent = irrigation_system.watering_queue[idx].moisture_deficit_percent;
        cache->queue[0].target_pulses = irrigation_system.watering_queue[idx].target_pulses;
        cache->queue[0].watering_completed = irrigation_system.watering_queue[idx].watering_completed;
    } else {
        // No active queue item
        cache->queue[0].zone_id = NO_ACTIVE_ZONE_ID;
        cache->queue[0].measured_moisture_percent = 0.0f;
        cache->queue[0].moisture_deficit_percent = 0.0f;
        cache->queue[0].target_pulses = 0;
        cache->queue[0].watering_completed = false;
    }

    cache->snapshot_timestamp = time(NULL);

    xSemaphoreGive(xIrrigationMutex);

    return ESP_OK;
}

// ########################## Load Shedding Functions ##########################

/**
 * @brief Set power saving mode for IMPLUVIUM irrigation system
 */
esp_err_t impluvium_set_power_save_mode(bool enable)
{
    if (!irrigation_system.sensors_powered) {
        return ESP_ERR_INVALID_STATE;
    }
    
    irrigation_system.power_save_mode = enable;
    
    if (enable) {
        ESP_LOGI(TAG, "Power save mode enabled - moisture check interval extended to 60min");
    } else {
        ESP_LOGI(TAG, "Power save mode disabled - normal moisture check intervals restored");
    }
    
    return ESP_OK;
}

/**
 * @brief Enable/disable IMPLUVIUM system master switch
 */
esp_err_t impluvium_set_system_enabled(bool enable)
{
    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    if (!enable) {
        ESP_LOGI(TAG, "System disabled via master switch");
        // Allow current watering to finish if in progress
        if (irrigation_system.state == IRRIGATION_WATERING ||
            irrigation_system.state == IRRIGATION_STOPPING) {
            ESP_LOGI(TAG, "Watering in progress - will transition to DISABLED after completion");
            // Will transition to DISABLED naturally after STOPPING completes
        } else {
            // Immediately transition to DISABLED for other states
            impluvium_change_state(IRRIGATION_DISABLED);
        }
        // Stop moisture check timer
        xTimerStop(xMoistureCheckTimer, 0);
    } else {
        ESP_LOGI(TAG, "System enabled via master switch");
        // Transition back to STANDBY and restart moisture timer
        impluvium_change_state(IRRIGATION_STANDBY);
        xTimerStart(xMoistureCheckTimer, 0);
    }

    xSemaphoreGive(xIrrigationMutex);
    return ESP_OK;
}

/**
 * @brief Force immediate moisture check (bypasses scheduled interval)
 */
esp_err_t impluvium_force_moisture_check(void)
{
    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Only allow if system is in STANDBY state
    if (irrigation_system.state != IRRIGATION_STANDBY) {
        ESP_LOGW(TAG, "Cannot force moisture check - system not in STANDBY state (current: %d)",
                 irrigation_system.state);
        xSemaphoreGive(xIrrigationMutex);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Forcing immediate moisture check via manual request");

    // Send notification to irrigation task to trigger moisture check
    if (xIrrigationTaskHandle != NULL) {
        xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_REQUEST_MOISTURE_CHECK, eSetBits);
    }

    xSemaphoreGive(xIrrigationMutex);
    return ESP_OK;
}

/**
 * @brief Clear emergency stop flag (manual reset)
 */
esp_err_t impluvium_clear_emergency_stop(void)
{
    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    if (irrigation_system.emergency_stop) {
        ESP_LOGI(TAG, "Emergency stop flag cleared via manual reset");
        irrigation_system.emergency_stop = false;
        // System will wait for next moisture check cycle to resume
    } else {
        ESP_LOGD(TAG, "Emergency stop flag already clear");
    }

    xSemaphoreGive(xIrrigationMutex);
    return ESP_OK;
}

/**
 * @brief Clear diagnostic state and error history
 */
esp_err_t impluvium_clear_diagnostics(void)
{
    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Clearing diagnostic state and error history");

    irrigation_system.emergency.state = EMERGENCY_NONE;
    irrigation_system.emergency.consecutive_failures = 0;
    irrigation_system.emergency.failed_zones_mask = 0;
    irrigation_system.emergency.test_zone = 0;
    irrigation_system.emergency.eligible_zones_count = 0;
    irrigation_system.emergency.eligible_zones_mask = 0;
    irrigation_system.emergency.test_cycle_count = 0;
    irrigation_system.emergency.failure_reason = NULL;

    ESP_LOGI(TAG, "Diagnostic state cleared successfully");

    xSemaphoreGive(xIrrigationMutex);
    return ESP_OK;
}

/**
 * @brief Reset learning data for specific zone to defaults
 */
esp_err_t impluvium_reset_zone_learning(uint8_t zone_id)
{
    if (zone_id >= IRRIGATION_ZONE_COUNT) {
        ESP_LOGE(TAG, "Invalid zone_id %d (must be 0-%d)", zone_id, IRRIGATION_ZONE_COUNT - 1);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Resetting learning data for zone %d to defaults", zone_id);

    // Clear history arrays
    memset(irrigation_zones[zone_id].learning.pulses_used_history, 0,
           sizeof(irrigation_zones[zone_id].learning.pulses_used_history));
    memset(irrigation_zones[zone_id].learning.moisture_increase_percent_history, 0,
           sizeof(irrigation_zones[zone_id].learning.moisture_increase_percent_history));
    memset(irrigation_zones[zone_id].learning.anomaly_flags, 0,
           sizeof(irrigation_zones[zone_id].learning.anomaly_flags));

    // Reset counters
    irrigation_zones[zone_id].learning.history_entry_count = 0;
    irrigation_zones[zone_id].learning.history_index = 0;
    irrigation_zones[zone_id].learning.successful_predictions = 0;
    irrigation_zones[zone_id].learning.total_predictions = 0;

    // Reset calculated values to defaults
    irrigation_zones[zone_id].learning.calculated_ppmp_ratio = DEFAULT_PULSES_PER_PERCENT;
    irrigation_zones[zone_id].learning.calculated_pump_duty_cycle = PUMP_DEFAULT_DUTY;
    irrigation_zones[zone_id].learning.target_moisture_gain_rate = TARGET_MOISTURE_GAIN_RATE;
    irrigation_zones[zone_id].learning.confidence_level = 0.0f;
    irrigation_zones[zone_id].learning.last_temperature_correction = 1.0f;

    ESP_LOGI(TAG, "Zone %d learning data reset complete", zone_id);

    xSemaphoreGive(xIrrigationMutex);
    return ESP_OK;
}

/**
 * @brief Reset learning data for all zones to defaults
 */
esp_err_t impluvium_reset_all_learning(void)
{
    ESP_LOGI(TAG, "Resetting learning data for all zones");

    esp_err_t ret = ESP_OK;
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        esp_err_t zone_ret = impluvium_reset_zone_learning(i);
        if (zone_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to reset learning for zone %d", i);
            ret = zone_ret;
        }
    }

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "All zone learning data reset successfully");
    }

    return ret;
}

/**
 * @brief Force manual watering for specific zone (safety override)
 */
esp_err_t impluvium_force_water_zone(uint8_t zone_id, uint16_t duration_sec)
{
    // Validate parameters
    if (zone_id >= IRRIGATION_ZONE_COUNT) {
        ESP_LOGE(TAG, "Invalid zone_id %d (must be 0-%d)", zone_id, IRRIGATION_ZONE_COUNT - 1);
        return ESP_ERR_INVALID_ARG;
    }

    if (duration_sec < 5 || duration_sec > 300) {
        ESP_LOGE(TAG, "Invalid duration %d seconds (must be 5-300)", duration_sec);
        return ESP_ERR_INVALID_ARG;
    }

    // Enforce 5-second increments
    uint16_t adjusted_duration = (duration_sec / 5) * 5;
    if (adjusted_duration != duration_sec) {
        ESP_LOGW(TAG, "Duration adjusted from %d to %d seconds (5s increments)",
                 duration_sec, adjusted_duration);
        duration_sec = adjusted_duration;
    }

    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Check if system is busy
    if (irrigation_system.state != IRRIGATION_STANDBY) {
        ESP_LOGW(TAG, "Cannot force water - system busy (state: %d)", irrigation_system.state);
        xSemaphoreGive(xIrrigationMutex);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG, "MANUAL WATER: Forcing zone %d for %d seconds (SAFETY OVERRIDE ACTIVE)",
             zone_id + 1, duration_sec);

    // Set manual watering mode
    irrigation_system.manual_watering_active = true;
    irrigation_system.manual_water_zone = zone_id;
    irrigation_system.manual_water_duration_sec = duration_sec;
    irrigation_system.manual_water_end_time = (xTaskGetTickCount() * portTICK_PERIOD_MS) + (duration_sec * 1000);

    // Transition to MEASURING state (will then go to WATERING)
    // The state machine will detect manual mode and handle it specially
    impluvium_change_state(IRRIGATION_MEASURING);
    if (xIrrigationTaskHandle != NULL) {
        // Notify the main task to process the state change immediately
        xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_MANUAL_WATER, eSetBits);
    }

    ESP_LOGI(TAG, "Manual watering initiated - zone %d, duration %ds, end time: %lu ms",
             zone_id + 1, duration_sec, irrigation_system.manual_water_end_time);

    xSemaphoreGive(xIrrigationMutex);
    return ESP_OK;
}

/**
 * @brief Set shutdown state for IMPLUVIUM irrigation system (for load shedding)
 */
esp_err_t impluvium_set_shutdown(bool shutdown)
{
    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    if (shutdown) {
        ESP_LOGI(TAG, "Load shedding shutdown - transitioning to DISABLED state");
        // Allow current watering to finish if in progress
        if (irrigation_system.state == IRRIGATION_WATERING ||
            irrigation_system.state == IRRIGATION_STOPPING) {
            ESP_LOGI(TAG, "Watering in progress - will transition to DISABLED after completion");
            // State will transition to DISABLED naturally after STOPPING completes
        } else {
            // Immediately transition to DISABLED for other states
            impluvium_change_state(IRRIGATION_DISABLED);
        }
        // Stop moisture check timer
        xTimerStop(xMoistureCheckTimer, 0);
    } else {
        ESP_LOGI(TAG, "Load shedding shutdown lifted - restoring normal operation");
        // Transition back to STANDBY and restart moisture timer
        impluvium_change_state(IRRIGATION_STANDBY);
        xTimerStart(xMoistureCheckTimer, 0);
    }

    xSemaphoreGive(xIrrigationMutex);
    return ESP_OK;
}

/**
 * @brief Update zone configuration and save to LittleFS
 *
 * Public API for updating zone settings via MQTT or other external interfaces.
 * Updates zone configuration in RAM and immediately saves to LittleFS for persistence.
 * Intended to be called by TELEMETRY when MQTT config updates are received.
 *
 * @param[in] zone_id Zone identifier (0-4)
 * @param[in] target_moisture_percent Desired moisture level (0-100%)
 * @param[in] moisture_deadband_percent Tolerance around target (±%)
 * @param[in] enabled Zone enabled/disabled state
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if zone_id out of range or invalid parameters
 * @return ESP_FAIL on save error
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