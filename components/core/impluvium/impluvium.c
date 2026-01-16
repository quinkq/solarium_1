/**
 * @file impluvium.c
 * @brief IMPLUVIUM - Core orchestration module for learning irrigation control system
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Core orchestration for IMPLUVIUM - intelligent multi-zone irrigation.
 * This module contains only the high-level coordination logic, delegating
 * specialized functionality to dedicated submodules.
 *
 * Core responsibilities:
 * - System initialization and configuration loading
 * - Main state machine task coordination
 * - Timer-based moisture check scheduling
 * - Power bus management (3.3V/5V/12V)
 * - Telemetry cache writing
 * - High-level public API endpoints
 *
 * Specialized functionality delegated to:
 * - impluvium_sensors.c: Sensor reading and hardware interface
 * - impluvium_actuators.c: Pump/valve/flow control
 * - impluvium_learning.c: Learning algorithm and calculations
 * - impluvium_state_machine.c: State transition handlers
 * - impluvium_safety.c: Safety monitoring and emergency diagnostics
 * - impluvium_storage.c: LittleFS persistence and RTC accumulator
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "impluvium.h"
#include "impluvium_private.h"
#include "fluctus.h"
#include "tempesta.h"   
#include "telemetry.h"
#include "solar_calc.h"
#include "interval_config.h"


#define VALIDATE_PTR(ptr)                                                                                              \
    if (!ptr)                                                                                                          \
    return ESP_ERR_INVALID_ARG

// ########################## Global Variable Definitions ##########################

static const char *TAG = "IMPLUVIUM";

irrigation_zone_t irrigation_zones[IRRIGATION_ZONE_COUNT];
irrigation_system_t irrigation_system;

SemaphoreHandle_t xIrrigationMutex = NULL;
TaskHandle_t xIrrigationTaskHandle = NULL;
TaskHandle_t xIrrigationMonitoringTaskHandle = NULL;
TimerHandle_t xMoistureCheckTimer = NULL;

// Interval configuration (loaded from interval_config at init)
uint32_t impluvium_optimal_interval_ms;       // Optimal temperature interval (temp ≥20°C)
uint32_t impluvium_cool_interval_ms;          // Cool temperature interval (10-20°C)
uint32_t impluvium_power_save_interval_ms;    // Power save mode interval
uint32_t impluvium_night_minimum_ms;          // Nighttime minimum interval

// GPIO mapping for zones
const gpio_num_t zone_valve_gpios[IRRIGATION_ZONE_COUNT] = {VALVE_GPIO_ZONE_0,
                                                            VALVE_GPIO_ZONE_1,
                                                            VALVE_GPIO_ZONE_2,
                                                            VALVE_GPIO_ZONE_3,
                                                            VALVE_GPIO_ZONE_4};

// ABP sensor handle
abp_t abp_dev = {0};

// Flow sensor pulse counter handle (global, used by actuators and safety modules)
pcnt_unit_handle_t flow_pcnt_unit = NULL;

// RTC RAM accumulator (survives ESP32 resets, persists across deep sleep)
// Type defined in impluvium_private.h
RTC_DATA_ATTR irrigation_accumulator_rtc_t rtc_impluvium_accumulator = {0};

// ########################## Forward Declarations ##########################

static void vTimerCallbackMoistureCheck(TimerHandle_t xTimer);
static void impluvium_task(void *pvParameters);
static void impluvium_daily_reset_callback(void);

// ########################## System Initialization ##########################

/**
 * @brief Initialize IMPLUVIUM irrigation system
 *
 * Initializes all subsystems and loads configuration from persistent storage.
 * Creates main irrigation task and monitoring task.
 *
 * @return ESP_OK on success, ESP_FAIL on error
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

    // Initialize system state (static variables are zero-initialized)
    irrigation_system.state = IMPLUVIUM_DISABLED;
    irrigation_system.active_zone = NO_ACTIVE_ZONE_ID; // No active zone
    irrigation_system.queue_index = 0;                 // Start at beginning of queue
    irrigation_system.state_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    irrigation_system.system_start_time = irrigation_system.state_start_time;
    irrigation_system.power_save_mode = false;         // Start in normal operation
    irrigation_system.load_shed_shutdown = false;      // Start with load shedding disabled

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

    // Initialize hardware (delegated to actuators module)
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

    // Test read water level sensor (hardware validation)
    ESP_LOGI(TAG, "Testing water level sensor...");
    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_3V3, "IMPLUVIUM_TEST", {
        ESP_LOGW(TAG, "Failed to power on 3.3V bus for water level sensor test");
    });
    vTaskDelay(pdMS_TO_TICKS(SENSOR_POWERUP_DELAY_MS)); // Wait for sensor stabilization

    float test_water_level = 0.0f;
    ret = impluvium_read_water_level(&test_water_level);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Water level sensor test SUCCESS: %.1f%%", test_water_level);
    } else {
        ESP_LOGE(TAG, "Water level sensor test FAILED: %s", esp_err_to_name(ret));
        ESP_LOGW(TAG, "Check ABP sensor wiring, power supply, and SPI2 connection");
    }
    fluctus_release_bus_power(POWER_BUS_3V3, "IMPLUVIUM_TEST");
/*
    // TODO: TEMPORARY - Remove after testing
    // Continuous polling loop for water level sensor testing (30 readings @ 1Hz)
    ESP_LOGI(TAG, "=== STARTING 30-SECOND WATER LEVEL POLLING TEST ===");
    for (int i = 0; i < 30; i++) {
        float water_level_test = 0.0f;
        float water_level_mbar = 0.0f;

        // Read raw mbar value for better debugging
        esp_err_t poll_ret = abp_read_pressure_mbar(&abp_dev, &water_level_mbar);

        if (poll_ret == ESP_OK) {
            // Convert to percentage using same logic as impluvium_read_water_level
            if (water_level_mbar < WATER_LEVEL_MIN_MBAR)
                water_level_mbar = WATER_LEVEL_MIN_MBAR;
            if (water_level_mbar > WATER_LEVEL_MAX_MBAR)
                water_level_mbar = WATER_LEVEL_MAX_MBAR;

            water_level_test = ((water_level_mbar - WATER_LEVEL_MIN_MBAR) /
                               (WATER_LEVEL_MAX_MBAR - WATER_LEVEL_MIN_MBAR)) * 100.0f;

            ESP_LOGI(TAG, "[%02d/30] Water level: %.1f%% (%.2f mbar, raw)",
                     i + 1, water_level_test, water_level_mbar);
        } else {
            ESP_LOGE(TAG, "[%02d/30] Read FAILED: %s", i + 1, esp_err_to_name(poll_ret));
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second interval
    }
    ESP_LOGI(TAG, "=== WATER LEVEL POLLING TEST COMPLETE ===");

    
*/
    // Load zone configurations from LittleFS (user-editable settings)
    ret = impluvium_load_zone_config();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Could not load zone config from LittleFS, using defaults");
    }

    // Load interval configuration from global config
    impluvium_optimal_interval_ms = g_interval_config.impluvium_optimal_min * 60 * 1000;
    impluvium_cool_interval_ms = g_interval_config.impluvium_cool_min * 60 * 1000;
    impluvium_power_save_interval_ms = g_interval_config.impluvium_power_save_min * 60 * 1000;
    impluvium_night_minimum_ms = g_interval_config.impluvium_night_min_hours * 60 * 60 * 1000;
    ESP_LOGI(TAG, "Check intervals: %lums (opt), %lums (cool), %lums (pwr), %lums (night)",
             impluvium_optimal_interval_ms, impluvium_cool_interval_ms,
             impluvium_power_save_interval_ms, impluvium_night_minimum_ms);

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
                                     configMINIMAL_STACK_SIZE * 5,
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

    // Populate telemetry cache with initial zone configuration
    // This ensures HMI has valid data before first watering cycle
    telemetry_fetch_snapshot(TELEMETRY_SRC_IMPLUVIUM);
    ESP_LOGI(TAG, "Initial telemetry cache populated");

    ESP_LOGI(TAG, "IMPLUVIUM irrigation system initialized successfully");
    return ESP_OK;
}

// ########################## Power Management ##########################

/**
 * @brief Request power buses for irrigation operations
 *
 * @param[in] level Power level (sensors or watering)
 * @param[in] tag Tag for power bus requests (e.g., "IMPLUVIUM", "IMPLUVIUM_DIAG")
 * @return ESP_OK on success, ESP_FAIL on any bus request failure
 */
esp_err_t impluvium_request_power_buses(power_level_t level, const char *tag)
{
    // Always request sensor buses first
    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_3V3, tag, {
        return ESP_FAIL;
    });

    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_5V, tag, {
        fluctus_release_bus_power(POWER_BUS_3V3, tag);
        return ESP_FAIL;
    });

    // Request level shifter and 12V bus for watering operations
    if (level == POWER_ALL_DEVICES) {
        // Enable level shifter first (required for valve and pump MOSFET gate drivers)
        esp_err_t ret = fluctus_request_level_shifter("IMPLUVIUM");
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable level shifter for irrigation");
            fluctus_release_bus_power(POWER_BUS_5V, tag);
            fluctus_release_bus_power(POWER_BUS_3V3, tag);
            return ESP_FAIL;
        }

        // Then enable 12V bus
        FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_12V, tag, {
            fluctus_release_level_shifter("IMPLUVIUM");
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
 * @return ESP_OK on success
 */
esp_err_t impluvium_release_power_buses(power_level_t level, const char *tag)
{
    // Release in reverse order (12V bus, then level shifter, then sensor buses)
    if (level == POWER_ALL_DEVICES) {
        fluctus_release_bus_power(POWER_BUS_12V, tag);
        fluctus_release_level_shifter("IMPLUVIUM");
    }
    fluctus_release_bus_power(POWER_BUS_5V, tag);
    fluctus_release_bus_power(POWER_BUS_3V3, tag);

    return ESP_OK;
}

// ########################## Timer Callbacks ##########################

/**
 * @brief Timer callback for periodic moisture checks
 *
 * Called by FreeRTOS timer to trigger moisture check cycles.
 * Dynamically adjusts interval based on temperature.
 *
 * @param xTimer Timer handle
 */
static void vTimerCallbackMoistureCheck(TimerHandle_t xTimer)
{
    // Don't trigger moisture checks if system is disabled or in load shedding shutdown
    if (irrigation_system.state == IMPLUVIUM_DISABLED || irrigation_system.load_shed_shutdown) {
        ESP_LOGD("IMPLUVIUM(Timer)", "System disabled or shutdown active - skipping moisture check");
        return;
    }

    // Get current temperature to determine next interval
    float current_temperature = tempesta_get_temperature();
    uint32_t next_interval_ms = impluvium_calc_moisture_check_interval(current_temperature);

    // If temperature is too low, skip moisture checks entirely
    if (next_interval_ms == UINT32_MAX) {
        ESP_LOGI("IMPLUVIUM(Timer)", "Skipping moisture check due to low temperature (%.1f°C)", current_temperature);
        // Set timer to check again in 1 hour
        next_interval_ms = 60 * 60 * 1000;
        return; // Don't trigger moisture check
    }

    // Update timer interval if it has changed
    TickType_t current_period = xTimerGetPeriod(xTimer);
    TickType_t new_period = pdMS_TO_TICKS(next_interval_ms);

    if (current_period != new_period) {
        ESP_LOGI("IMPLUVIUM(Timer)",
                 "Updating moisture check interval: %" PRIu32 "min (temp: %.1f°C)",
                 next_interval_ms / (60 * 1000),
                 current_temperature);
        xTimerChangePeriod(xTimer, new_period, 0);
    }

    // Notify the main irrigation task to perform moisture check
    xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_REQUEST_MOISTURE_CHECK, eSetBits);
}

// ########################## Main Task ##########################

/**
 * @brief Main irrigation control task (State Machine)
 *
 * Event-driven task that responds to timer callbacks and notifications.
 * Coordinates state transitions and delegates work to specialized modules.
 *
 * @param pvParameters Task parameters (unused)
 */
static void impluvium_task(void *pvParameters)
{
    const char *task_tag = "IMPLUVIUM(Task)";
    ESP_LOGI(task_tag, "Main irrigation task started");

    // Wait for other systems to initialize
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Start the periodic moisture check timer
    xTimerStart(xMoistureCheckTimer, 0);
    uint32_t notification_value = 0;

    while (1) {
        notification_value = 0;
        // Wait indefinitely for a notification
        xTaskNotifyWait(0x00,                /* Don't clear any bits on entry */
                        ULONG_MAX,           /* Clear all bits on exit */
                        &notification_value, /* Receives the notification value */
                        portMAX_DELAY);      /* Block indefinitely */
                        
        // Monitor stack usage (debug - watermark shows minimum free stack ever reached)
        UBaseType_t stack_high_water = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGI(TAG, "[STACK] High water mark: %u bytes free (min ever)", stack_high_water * sizeof(StackType_t));

        // Take mutex to ensure exclusive access to the state machine
        if (xSemaphoreTake(xIrrigationMutex, portMAX_DELAY) == pdTRUE) {
            // --- Event Handling ---
            if (notification_value & IRRIGATION_TASK_NOTIFY_REQUEST_MOISTURE_CHECK) {
                ESP_LOGI(task_tag, "Notification: Moisture check requested");
                if (irrigation_system.load_shed_shutdown) {
                    ESP_LOGD(task_tag, "Load shedding shutdown active - skipping moisture check");
                } else if (irrigation_system.state == IMPLUVIUM_STANDBY) {
                    impluvium_change_state(IMPLUVIUM_MEASURING);
                }
            }
            if (notification_value & IRRIGATION_TASK_NOTIFY_WATERING_CUTOFF) {
                ESP_LOGI(task_tag, "Notification: Watering cutoff requested");
                if (irrigation_system.state == IMPLUVIUM_WATERING) {
                    impluvium_change_state(IMPLUVIUM_STOPPING);
                }
            }
            if (notification_value & IRRIGATION_TASK_NOTIFY_EMERGENCY_SHUTDOWN) {
                ESP_LOGE(task_tag, "Notification: Emergency shutdown requested");
                // Stop the moisture check timer during an emergency
                xTimerStop(xMoistureCheckTimer, 0);
                // The emergency_stop function has already set the reason/flag. We just change state.
                if (irrigation_system.state == IMPLUVIUM_WATERING) {
                    impluvium_change_state(IMPLUVIUM_STOPPING);
                } else {
                    impluvium_change_state(IMPLUVIUM_MAINTENANCE);
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
                impluvium_state_t current_state = irrigation_system.state;

                switch (current_state) {
                    case IMPLUVIUM_STANDBY:
                        // Do nothing, wait for notification
                        break;

                    case IMPLUVIUM_DISABLED:
                        // System disabled - do nothing, ignore moisture check notifications
                        break;

                    case IMPLUVIUM_MEASURING:
                        impluvium_state_measuring();
                        break;

                    case IMPLUVIUM_WATERING:
                        // If we are entering the watering state, start the sequence.
                        // Otherwise, do nothing and let the monitoring task handle it.
                        if (irrigation_system.active_zone == NO_ACTIVE_ZONE_ID) {
                            impluvium_state_watering();
                        }
                        break;

                    case IMPLUVIUM_STOPPING:
                        impluvium_state_stopping();
                        break;

                    case IMPLUVIUM_MAINTENANCE:
                        impluvium_state_maintenance();
                        break;

                    default:
                        ESP_LOGE(task_tag, "Unknown irrigation state: %d", irrigation_system.state);
                        impluvium_change_state(IMPLUVIUM_STANDBY);
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

// ########################## Daily Reset Callback ##########################

/**
 * @brief Daily reset callback for midnight counter resets and learning data backup
 *
 * Called by TELEMETRY when a new day is detected (via solar_calc day change).
 * Delegates work to storage module for proper mutex protection.
 */
static void impluvium_daily_reset_callback(void)
{
    ESP_LOGI(TAG, "Midnight callback received, notifying main task");

    // Delegate all work to main task for proper mutex protection
    if (xIrrigationTaskHandle != NULL) {
        xTaskNotify(xIrrigationTaskHandle, IRRIGATION_TASK_NOTIFY_MIDNIGHT_RESET, eSetBits);
    }
}

// ########################## Telemetry Cache Writers ##########################

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
    cache->current_hour_start = rtc_impluvium_accumulator.current_hour_start;

    float total_hour_ml = 0.0f;
    float total_day_ml = 0.0f;
    uint8_t events_hour = 0;
    uint8_t events_day = 0;

    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        total_hour_ml += rtc_impluvium_accumulator.zone_water_used_hour_ml[i];
        total_day_ml += rtc_impluvium_accumulator.zone_water_used_day_ml[i];
        events_hour += rtc_impluvium_accumulator.zone_events_hour[i];
        events_day += rtc_impluvium_accumulator.zone_events_day[i];
    }

    cache->total_water_used_hour_ml = total_hour_ml;
    cache->total_water_used_day_ml = total_day_ml;
    cache->watering_events_hour = events_hour;
    cache->watering_events_day = events_day;

    // Per-zone data (comprehensive - all 5 zones)
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        cache->zones[i].watering_enabled = irrigation_zones[i].watering_enabled;

        cache->zones[i].current_moisture_percent = irrigation_zones[i].last_moisture_percent;

        cache->zones[i].target_moisture_percent = irrigation_zones[i].target_moisture_percent;
        cache->zones[i].moisture_deadband_percent = irrigation_zones[i].moisture_deadband_percent;

        // Read per-zone hourly/daily statistics from RTC accumulator (persistent!)
        cache->zones[i].volume_used_hour_ml = rtc_impluvium_accumulator.zone_water_used_hour_ml[i];
        cache->zones[i].volume_used_today_ml = rtc_impluvium_accumulator.zone_water_used_day_ml[i];
        cache->zones[i].events_hour = rtc_impluvium_accumulator.zone_events_hour[i];
        cache->zones[i].events_day = rtc_impluvium_accumulator.zone_events_day[i];

        // Calculate average hourly consumption since midnight
        time_t now = time(NULL);
        float hours_elapsed = (float)(now - rtc_impluvium_accumulator.current_day_start) / 3600.0f;
        if (hours_elapsed > 0.0f) {
            cache->zones[i].avg_hourly_consumption_ml = rtc_impluvium_accumulator.zone_water_used_day_ml[i] / hours_elapsed;
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
                        (irrigation_system.state == IMPLUVIUM_WATERING);

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

// ########################## Emergency Shutdown Logic ##########################

/**
 * @brief Immediately stop all irrigation operations (internal helper)
 *
 * Performs emergency abort bypassing normal state machine flow.
 * Must be called with xIrrigationMutex already held.
 * Safe to call from any state - performs minimal necessary cleanup.
 *
 * Actions:
 * - Stop pump immediately (PWM = 0)
 * - Close all zone valves
 * - Release level shifter
 * - Release all power buses
 * - Notify monitoring task to stop
 * - Restore Stellaria lighting
 * - Transition to DISABLED state
 */
static void impluvium_perform_emergency_stop(void)
{
    ESP_LOGW(TAG, "EMERGENCY STOP - Immediate abort of all irrigation operations");

    // Stop pump immediately (no ramp-down)
    impluvium_set_pump_speed(0);

    // Close all valves (safety measure - close all even if only one was active)
    for (uint8_t i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        if (irrigation_zones[i].watering_in_progress) {
            impluvium_close_valve(i);
            irrigation_zones[i].watering_in_progress = false;
        }
    }

    // Release level shifter (SN74AHCT125)
    fluctus_release_level_shifter("IMPLUVIUM");

    // Release all power buses
    fluctus_release_bus_power(POWER_BUS_12V, "IMPLUVIUM");
    fluctus_release_bus_power(POWER_BUS_5V, "IMPLUVIUM");
    fluctus_release_bus_power(POWER_BUS_3V3, "IMPLUVIUM");

    // Notify monitoring task to stop (if active)
    if (xIrrigationMonitoringTaskHandle != NULL) {
        xTaskNotify(xIrrigationMonitoringTaskHandle, MONITORING_TASK_NOTIFY_STOP_MONITORING, eSetBits);
    }

    // Restore Stellaria lighting (cancel irrigation dimming)
    stellaria_request_irrigation_dim(false);

    // Reset active zone
    irrigation_system.active_zone = NO_ACTIVE_ZONE_ID;

    // Clear any manual watering flags
    irrigation_system.manual_watering_active = false;
    irrigation_system.manual_water_zone = 0;
    irrigation_system.manual_water_duration_sec = 0;
    irrigation_system.manual_water_end_time = 0;

    // Transition to DISABLED state
    impluvium_change_state(IMPLUVIUM_DISABLED);

    ESP_LOGI(TAG, "Emergency stop complete - all actuators disabled, buses released");
}

// ########################## Public API Functions ##########################

/**
 * @brief Set power saving mode for IMPLUVIUM irrigation system
 *
 * Extends moisture check intervals to conserve power during battery optimization.
 *
 * @param enable true to enable power save mode, false to disable
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if sensors not powered
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
 *
 * @param enable true to enable system, false to disable
 * @return ESP_OK on success, ESP_FAIL if mutex timeout
 */
esp_err_t impluvium_set_system_enabled(bool enable)
{
    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    if (!enable) {
        ESP_LOGI(TAG, "System disabled via master switch");

        // EMERGENCY OVERRIDE: Immediately abort watering if in progress
        if (irrigation_system.state == IMPLUVIUM_WATERING ||
            irrigation_system.state == IMPLUVIUM_STOPPING) {
            ESP_LOGW(TAG, "Watering in progress - performing emergency stop (override)");
            impluvium_perform_emergency_stop();
        } else {
            // Normal transition for other states
            impluvium_change_state(IMPLUVIUM_DISABLED);
        }

        // Stop moisture check timer
        xTimerStop(xMoistureCheckTimer, 0);
    } else {
        ESP_LOGI(TAG, "System enabled via master switch");
        // Transition back to STANDBY and restart moisture timer
        impluvium_change_state(IMPLUVIUM_STANDBY);
        xTimerStart(xMoistureCheckTimer, 0);
    }

    xSemaphoreGive(xIrrigationMutex);
    return ESP_OK;
}

/**
 * @brief Get current IMPLUVIUM operational state (lightweight, no snapshot fetch)
 * @return Current impluvium_state_t value
 * @note Thread-safe, uses quick mutex with 100ms timeout. Returns DISABLED on mutex timeout.
 */
impluvium_state_t impluvium_get_state(void)
{
    impluvium_state_t state = IMPLUVIUM_DISABLED;

    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        state = irrigation_system.state;
        xSemaphoreGive(xIrrigationMutex);
    }

    return state;
}

/**
 * @brief Force immediate moisture check (bypasses scheduled interval)
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not in STANDBY
 */
esp_err_t impluvium_force_moisture_check(void)
{
    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Only allow if system is in STANDBY state
    if (irrigation_system.state != IMPLUVIUM_STANDBY) {
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
 *
 * @return ESP_OK on success, ESP_FAIL if mutex timeout
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
 *
 * @return ESP_OK on success, ESP_FAIL if mutex timeout
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
 * @brief Force manual watering for specific zone (safety override)
 *
 * Bypasses learning algorithm and moisture checks for direct manual control.
 *
 * @param zone_id Zone identifier (0-4)
 * @param duration_sec Watering duration in seconds (5-300, in 5s increments)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters out of range
 * @return ESP_ERR_INVALID_STATE if system busy
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
    if (irrigation_system.state != IMPLUVIUM_STANDBY) {
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
    impluvium_change_state(IMPLUVIUM_MEASURING);
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
 *
 * Disables all irrigation operations during critical battery conditions.
 *
 * @param shutdown true to shutdown, false to restore operation
 * @return ESP_OK on success, ESP_FAIL if mutex timeout
 */
esp_err_t impluvium_set_shutdown(bool shutdown)
{
    // CRITICAL: Use longer timeout for load shedding shutdown (monitoring task may be holding mutex)
    // Safety-critical operation - MUST succeed even if monitoring task is active
    if (xSemaphoreTake(xIrrigationMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "CRITICAL: Failed to acquire mutex for load shedding shutdown - continuing anyway");
        // This should never happen, but log and return failure
        return ESP_FAIL;
    }

    irrigation_system.load_shed_shutdown = shutdown;

    if (shutdown) {
        ESP_LOGI(TAG, "Load shedding shutdown - transitioning to DISABLED state");

        // EMERGENCY OVERRIDE: Immediately abort watering if in progress
        // (FLUCTUS has revoked 12V bus power - cannot continue safely)
        if (irrigation_system.state == IMPLUVIUM_WATERING ||
            irrigation_system.state == IMPLUVIUM_STOPPING) {
            ESP_LOGW(TAG, "Watering in progress - performing emergency stop (load shedding override)");
            impluvium_perform_emergency_stop();
        } else {
            // Normal transition for other states
            impluvium_change_state(IMPLUVIUM_DISABLED);
        }

        // Stop moisture check timer
        xTimerStop(xMoistureCheckTimer, 0);
    } else {
        ESP_LOGI(TAG, "Load shedding shutdown lifted - restoring normal operation");
        // Transition back to STANDBY and restart moisture timer
        impluvium_change_state(IMPLUVIUM_STANDBY);
        xTimerStart(xMoistureCheckTimer, 0);
    }

    xSemaphoreGive(xIrrigationMutex);
    return ESP_OK;
}
