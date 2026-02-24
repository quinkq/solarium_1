/**
 * @file fluctus_power_bus.c
 * @brief FLUCTUS power bus control module
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Manages 4-bus power distribution system with reference counting:
 * - 3.3V sensor bus (open-drain to P-MOSFET gate)
 * - 5V bus (push-pull to buck converter EN pin)
 * - 6.6V servo bus (open-drain, inverted logic)
 * - 12V bus (push-pull to N-MOSFET gate)
 *
 * Level shifter management (SN74AHCT125):
 * - Reference-counted enable/disable for 12V actuator buffers
 * - Automatic power sequencing with stabilization delay
 * - Consumer tracking for safe concurrent actuator operation
 *
 * Features:
 * - Reference counting prevents premature bus/shifter shutdown
 * - Consumer tracking for debugging (max 8 per bus/shifter)
 * - Hardware state caching to minimize GPIO operations
 * - Hall array MOSFET control (N-MOSFET for TEMPESTA wind direction)
 * - Event logging for power management actions
 *
 * THREAD SAFETY:
 * - Protected by xPowerBusMutex (1000ms timeout)
 * - Safe to call from any task
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "fluctus.h"
#include "fluctus_private.h"
#include "mcp23008_helper.h"
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = "FLUCTUS_P_BUS";

// ########################## Module State ##########################
// Note: NO 'static' keyword - these are accessed via extern in fluctus_private.h

SemaphoreHandle_t xPowerBusMutex = NULL;

// Power bus system status
fluctus_power_status_t system_status = {.bus_enabled = {false, false, false, false},
                                        .bus_ref_count = {0, 0, 0, 0},
                                        .power_state = FLUCTUS_POWER_STATE_NORMAL,
                                        .safety_shutdown = false,
                                        .manual_reset_required = false,
                                        .last_activity_time = 0};

// Hardware state cache to avoid redundant GPIO operations
bool hardware_bus_state[POWER_BUS_COUNT] = {false, false, false, false};
bool hardware_level_shifter_state = false;

static char const *power_bus_names[POWER_BUS_COUNT] = {[POWER_BUS_3V3] = "POWER_BUS_3V3",
                                                       [POWER_BUS_5V] = "POWER_BUS_5V",
                                                       [POWER_BUS_6V6] = "POWER_BUS_6V6",
                                                       [POWER_BUS_12V] = "POWER_BUS_12V"};

// ########################## GPIO Initialization ##########################


/**
 * @brief Initialize GPIO pins for power control - Hardware Configuration
 *
 * 3.3V Bus: Open-drain, LOW=ON (P-MOSFET with external pullup on gate)
 * 5V Bus:   Push-pull, HIGH=ON (pulls converter EN to 3.3V via Schottky, 100kΩ pulldown=OFF)
 * 6.6V Bus: Open-drain, LOW=ON (inverted logic buck, 100kΩ pullup to 3.3V)
 * 12V Bus:  Push-pull, HIGH=ON (pulls converter EN to 3.3V via Schottky, 100kΩ pulldown=OFF)
 * Level Shifter: Open-drain, LOW=enable (SN74AHCT125 OE, active-low, external 10kΩ pullup)
 */
esp_err_t fluctus_gpio_init(void)
{
    ESP_LOGI(TAG, "Initializing power control GPIOs...");
    esp_err_t ret;

    // 5V, 12V Buses and devices: Push-pull output
    gpio_config_t gpio_pp_conf = {.pin_bit_mask = (1ULL << FLUCTUS_5V_BUS_ENABLE_GPIO) |  // 5V buck converter EN
                                                  (1ULL << FLUCTUS_12V_BUS_ENABLE_GPIO) | // 12V buck converter EN
                                                  (1ULL << FLUCTUS_12V_FAN_ENABLE_GPIO),  // 12V fan individual enable
                                  .mode = GPIO_MODE_OUTPUT,                               // Push-pull (NOT open-drain)
                                  .pull_up_en = GPIO_PULLUP_DISABLE,
                                  .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                  .intr_type = GPIO_INTR_DISABLE};

    ret = gpio_config(&gpio_pp_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure push-pull GPIOs: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set initial states (all OFF)
    gpio_set_level(FLUCTUS_5V_BUS_ENABLE_GPIO, 0);  // 5V: LOW = OFF
    gpio_set_level(FLUCTUS_12V_BUS_ENABLE_GPIO, 0); // 12V: LOW = OFF
    gpio_set_level(FLUCTUS_12V_FAN_ENABLE_GPIO, 0); // Fan: LOW = OFF

    // 3.3V Bus, 6.6V Bus, Level Shifter: Open-drain outputs
    // NOTE: Hall array enable now on MCP23008 GP5 - managed by mcp23008_helper
    gpio_config_t gpio_od_conf = {.pin_bit_mask = (1ULL << FLUCTUS_3V3_BUS_ENABLE_GPIO) |  // 3.3V P-MOSFET gate
                                                  (1ULL << FLUCTUS_6V6_BUS_ENABLE_GPIO) |  // 6.6V buck (inverted)
                                                  (1ULL << FLUCTUS_LEVEL_SHIFTER_OE_GPIO), // SN74AHCT125 OE
                                  .mode = GPIO_MODE_OUTPUT_OD,                             // Open-drain
                                  .pull_up_en = GPIO_PULLUP_DISABLE, // No internal pull-ups (external hardware)
                                  .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                  .intr_type = GPIO_INTR_DISABLE};

    ret = gpio_config(&gpio_od_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure open-drain GPIOs: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set initial states (all OFF/disabled)
    // 3.3V: Set as input (high-Z) to allow external pullup to keep P-MOSFET gate high (OFF)
    gpio_set_direction(FLUCTUS_3V3_BUS_ENABLE_GPIO, GPIO_MODE_INPUT);
    // 6.6V: Set as input (high-Z) for inverted logic (external pullup = OFF)
    gpio_set_direction(FLUCTUS_6V6_BUS_ENABLE_GPIO, GPIO_MODE_INPUT);
    // Level Shifter: Set as input (high-Z) to allow external pullup to disable OE (buffers tri-state)
    gpio_set_direction(FLUCTUS_LEVEL_SHIFTER_OE_GPIO, GPIO_MODE_INPUT);

    // Initialize hardware state cache to disabled
    for (int i = 0; i < POWER_BUS_COUNT; i++) {
        hardware_bus_state[i] = false;
    }
    hardware_level_shifter_state = false;

    ESP_LOGI(TAG, "Power control GPIOs initialized:");
    ESP_LOGI(TAG, "  3.3V: INPUT/HIGH (OFF), 5V: LOW (OFF), 6.6V: INPUT/HIGH (OFF), 12V: LOW (OFF)");
    ESP_LOGI(TAG, "  Level Shifter OE: INPUT/HIGH (disabled)");
    return ESP_OK;
}

// ########################## Bus Control Functions ##########################

/**
 * @brief Update hardware state for specific power bus
 *
 * Hardware control logic per bus:
 * - 3.3V: Open-drain, OUTPUT+LOW=ON (P-MOSFET), INPUT=OFF (external pullup)
 * - 5V:   Push-pull, HIGH=ON, LOW=OFF (pulls converter EN via Schottky)
 * - 6.6V: Open-drain, OUTPUT+LOW=ON, INPUT=OFF (inverted logic, 100kΩ pullup)
 * - 12V:  Push-pull, HIGH=ON, LOW=OFF (pulls converter EN via Schottky)
 */
esp_err_t fluctus_update_bus_hardware(power_bus_t bus)
{
    if (bus >= POWER_BUS_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    bool enable = system_status.bus_enabled[bus] && !system_status.safety_shutdown;

    // Check if hardware state actually needs to change
    if (hardware_bus_state[bus] == enable) {
        ESP_LOGV(TAG,
                 "Bus %s hardware already in correct state: %s",
                 power_bus_names[bus],
                 enable ? "ENABLED" : "DISABLED");
        return ESP_OK; // No change needed
    }

    esp_err_t ret = ESP_OK;

    switch (bus) {
        case POWER_BUS_3V3:
            // 3.3V: Open-drain with P-MOSFET, LOW=ON, FLOAT=OFF
            if (enable) {
                // Pull to ground (OUTPUT mode + LOW = ON, pulls P-MOSFET gate low)
                ret = gpio_set_direction(FLUCTUS_3V3_BUS_ENABLE_GPIO, GPIO_MODE_OUTPUT_OD);
                if (ret == ESP_OK) {
                    gpio_set_level(FLUCTUS_3V3_BUS_ENABLE_GPIO, 0);
                }
            } else {
                // Float the pin (INPUT mode = high-Z, external pullup keeps gate high = OFF)
                ret = gpio_set_direction(FLUCTUS_3V3_BUS_ENABLE_GPIO, GPIO_MODE_INPUT);
            }
            break;

        case POWER_BUS_5V:
            // 5V: Push-pull output, HIGH=ON (pulls converter EN to 3.3V), LOW=OFF
            gpio_set_level(FLUCTUS_5V_BUS_ENABLE_GPIO, enable ? 1 : 0);
            break;

        case POWER_BUS_6V6:
            // 6.6V: Open-drain with inverted logic, LOW=ON, INPUT=OFF
            if (enable) {
                // Pull to ground (OUTPUT mode + LOW = ON)
                ret = gpio_set_direction(FLUCTUS_6V6_BUS_ENABLE_GPIO, GPIO_MODE_OUTPUT_OD);
                if (ret == ESP_OK) {
                    gpio_set_level(FLUCTUS_6V6_BUS_ENABLE_GPIO, 0);
                }
            } else {
                // Float the pin (INPUT mode = high-Z, pullup keeps it OFF)
                ret = gpio_set_direction(FLUCTUS_6V6_BUS_ENABLE_GPIO, GPIO_MODE_INPUT);
            }
            break;

        case POWER_BUS_12V:
            // 12V: Push-pull output, HIGH=ON (pulls converter EN to 3.3V), LOW=OFF
            gpio_set_level(FLUCTUS_12V_BUS_ENABLE_GPIO, enable ? 1 : 0);
            break;

        default:
            return ESP_ERR_INVALID_ARG;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update bus %d hardware: %s", bus, esp_err_to_name(ret));
        return ret;
    }

    // Update hardware state cache
    hardware_bus_state[bus] = enable;
    ESP_LOGW(TAG, "%s hardware updated: %s", power_bus_names[bus], enable ? "ENABLED" : "DISABLED");
    return ESP_OK;
}

/**
 * @brief Add consumer to bus tracking
 */
/**
 * @brief Check if consumer is already tracked on a bus
 */
static bool fluctus_is_consumer_tracked(power_bus_t bus, const char *consumer_id)
{
    for (int i = 0; i < FLUCTUS_POWER_BUS_MAX_CONSUMERS; i++) {
        if (strcmp(system_status.bus_consumers[bus][i], consumer_id) == 0) {
            return true;
        }
    }
    return false;
}

esp_err_t fluctus_add_consumer(power_bus_t bus, const char *consumer_id)
{
    if (bus >= POWER_BUS_COUNT || consumer_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Find empty slot or existing consumer
    for (int i = 0; i < FLUCTUS_POWER_BUS_MAX_CONSUMERS; i++) {
        if (strlen(system_status.bus_consumers[bus][i]) == 0 ||
            strcmp(system_status.bus_consumers[bus][i], consumer_id) == 0) {
            strncpy(system_status.bus_consumers[bus][i], consumer_id, 15);
            system_status.bus_consumers[bus][i][15] = '\0';
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "No free consumer slots for bus - %s", power_bus_names[bus]);
    return ESP_ERR_NO_MEM;
}

/**
 * @brief Remove consumer from bus tracking
 *
 * Finds and removes the consumer from the bus consumer list.
 * Called during fluctus_release_bus_power().
 */
esp_err_t fluctus_remove_consumer(power_bus_t bus, const char *consumer_id)
{
    if (bus >= POWER_BUS_COUNT || consumer_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Find and remove consumer
    for (int i = 0; i < FLUCTUS_POWER_BUS_MAX_CONSUMERS; i++) {
        if (strcmp(system_status.bus_consumers[bus][i], consumer_id) == 0) {
            memset(system_status.bus_consumers[bus][i], 0, 16);
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

// ########################## Debugging & Logging ##########################

/**
 * @brief Log all currently active consumers on powered buses
 *
 * Useful for debugging power management issues and understanding
 * why a bus remains powered.
 */
void fluctus_log_active_consumers(const char *event_context)
{
    const char *bus_names[] = {"3V3", "5V", "6V6", "12V"};
    bool any_consumers = false;

    ESP_LOGE(TAG, "%s - Active consumers:", event_context);

    for (int bus = 0; bus < POWER_BUS_COUNT; bus++) {
        if (system_status.bus_enabled[bus] && system_status.bus_ref_count[bus] > 0) {
            ESP_LOGE(TAG, "  %s bus (%d consumers):", bus_names[bus], system_status.bus_ref_count[bus]);
            for (int i = 0; i < FLUCTUS_POWER_BUS_MAX_CONSUMERS; i++) {
                if (strlen(system_status.bus_consumers[bus][i]) > 0) {
                    ESP_LOGE(TAG, "    - %s", system_status.bus_consumers[bus][i]);
                    any_consumers = true;
                }
            }
        }
    }

    if (!any_consumers) {
        ESP_LOGE(TAG, "  No active consumers found (possible hardware issue)");
    }
}

/**
 * @brief Log power management events
 */
void fluctus_log_power_event(fluctus_event_type_t event_type, power_bus_t bus, const char *consumer_id)
{
    const char *event_names[] = {"BUS_ON",
                                 "BUS_OFF",
                                 "CONSUMER_REQUEST",
                                 "CONSUMER_RELEASE",
                                 "OVERCURRENT_WARNING",
                                 "OVERCURRENT_SHUTDOWN",
                                 "BATTERY_LOW",
                                 "SOLAR_TRACKING_PARK",
                                 "POWER_STATE_CHANGE"};

    const char *bus_names[] = {"3V3", "5V", "6V6", "12V", "SYSTEM"};

    if (event_type < sizeof(event_names) / sizeof(event_names[0])) {
        if (bus < POWER_BUS_COUNT) {
            ESP_LOGD(TAG,
                     "POWER BUS EVENT: %s [%s] - %s",
                     event_names[event_type],
                     bus_names[bus],
                     consumer_id ? consumer_id : "");
        } else {
            ESP_LOGD(TAG, "POWER EVENT: %s %s", event_names[event_type], consumer_id ? consumer_id : "");
        }
    }
}

// ########################## Public API ##########################

/**
 * @brief Request power bus activation with reference counting
 *
 * Enables the specified power bus if not already on, and increments
 * the reference count. Bus remains powered until all consumers release it.
 *
 * Thread-safe with xPowerBusMutex (1000ms timeout).
 *
 * @param bus Power bus to activate (POWER_BUS_3V3, POWER_BUS_5V, etc.)
 * @param consumer_id String identifier for the requesting consumer (max 15 chars)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if safety shutdown active
 * @return ESP_ERR_INVALID_ARG if bus invalid or not initialized
 * @return ESP_FAIL if mutex timeout
 */
esp_err_t fluctus_request_bus_power(power_bus_t bus, const char *consumer_id)
{
    if (bus >= POWER_BUS_COUNT || consumer_id == NULL || !fluctus_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xPowerBusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
        if (system_status.safety_shutdown) {
            xSemaphoreGive(xPowerBusMutex);
            return ESP_ERR_INVALID_STATE;
        }

        // Add consumer and increment reference count (skip if already tracked — prevents
        // ref count leak when the same consumer requests a bus multiple times, e.g. IMPLUVIUM
        // requesting 12V for each zone in a multi-zone watering session)
        bool already_tracked = fluctus_is_consumer_tracked(bus, consumer_id);
        fluctus_add_consumer(bus, consumer_id);
        if (!already_tracked) {
            system_status.bus_ref_count[bus]++;
        }

        // Enable bus if not already enabled
        if (!system_status.bus_enabled[bus]) {
            system_status.bus_enabled[bus] = true;
            fluctus_update_bus_hardware(bus);
            fluctus_log_power_event(FLUCTUS_EVENT_BUS_ON, bus, consumer_id);

            // Bus-dependent power sequencing delays for voltage stabilization
            // and capacitor charging (prevents inrush current spikes)
            uint32_t delay_ms = 0;

            switch (bus) {
                case POWER_BUS_12V:
                    delay_ms = 2000; // 12V: P-MOSFET soft-start + 470µF cap charging + pump/valve stabilization
                    break;
                case POWER_BUS_6V6:
                    delay_ms = 1000; // 6.6V: Servo power-up stabilization
                    break;
                case POWER_BUS_5V:
                    delay_ms = 100; // 5V: I2C/UART device power-on reset
                    break;
                case POWER_BUS_3V3:
                    delay_ms = 100; // 3.3V: Sensor bus stabilization
                    break;
                case POWER_BUS_COUNT:
                default:
                    // Invalid bus (already validated earlier), no delay
                    delay_ms = 0;
                    break;
            }

            if (delay_ms > 0) {
                vTaskDelay(pdMS_TO_TICKS(delay_ms));
                ESP_LOGD(TAG, "Power-up delay complete for %s bus", power_bus_names[bus]);
            }

            // Notify monitoring task of power activity to start constant monitoring.
            // Skip self-notification: if the monitoring task itself requests a bus (e.g. DS18B20_READ),
            // notifying it would cause a perpetual active-monitoring loop — the pending notification
            // makes the next xTaskNotifyWait() return immediately, re-setting monitoring_active=true
            // every 30s forever even after all external consumers have released the buses.
            if (xFluctusMonitoringTaskHandle != NULL &&
                xTaskGetCurrentTaskHandle() != xFluctusMonitoringTaskHandle) {
                xTaskNotify(xFluctusMonitoringTaskHandle, FLUCTUS_NOTIFY_POWER_ACTIVITY, eSetBits);
            }
        } else {
            fluctus_log_power_event(FLUCTUS_EVENT_CONSUMER_REQUEST, bus, consumer_id);
        }

        system_status.last_activity_time = time(NULL);

        xSemaphoreGive(xPowerBusMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief Release power bus (decrements reference count)
 *
 * Decrements the reference count for the specified bus. When the count
 * reaches zero, the bus is automatically disabled.
 *
 * Thread-safe with xPowerBusMutex (1000ms timeout).
 *
 * @param bus Power bus to release
 * @param consumer_id String identifier for the releasing consumer
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if bus invalid or not initialized
 * @return ESP_FAIL if mutex timeout
 */
esp_err_t fluctus_release_bus_power(power_bus_t bus, const char *consumer_id)
{
    if (bus >= POWER_BUS_COUNT || consumer_id == NULL || !fluctus_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xPowerBusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
        // Remove consumer and decrement reference count
        fluctus_remove_consumer(bus, consumer_id);
        if (system_status.bus_ref_count[bus] > 0) {
            system_status.bus_ref_count[bus]--;
        }
        ESP_LOGI(TAG, "%i consumers left on: %s", system_status.bus_ref_count[bus], power_bus_names[bus]);
        // Log consumer release
        fluctus_log_power_event(FLUCTUS_EVENT_CONSUMER_RELEASE, bus, consumer_id);

        // Check if bus should be disabled (last consumer gone)
        if (system_status.bus_ref_count[bus] == 0 && system_status.bus_enabled[bus]) {
            xSemaphoreGive(xPowerBusMutex); // Release mutex BEFORE delay

            // Debounce delay: Wait to see if another consumer requests the bus
            // Prevents rapid power cycling (wear, inrush current)
            vTaskDelay(pdMS_TO_TICKS(FLUCTUS_POWER_BUS_OFF_DELAY_MS));

            // Re-acquire mutex and check again
            if (xSemaphoreTake(xPowerBusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
                // Only turn off if STILL no consumers after delay
                if (system_status.bus_ref_count[bus] == 0 && system_status.bus_enabled[bus]) {
                    system_status.bus_enabled[bus] = false;
                    fluctus_update_bus_hardware(bus);
                    fluctus_log_power_event(FLUCTUS_EVENT_BUS_OFF, bus, consumer_id);
                }
                // else: Someone requested the bus during our delay - leave it on
                xSemaphoreGive(xPowerBusMutex);
                return ESP_OK;
            }

            return ESP_FAIL; // Failed to re-acquire mutex after delay
        } else {
            // Still has consumers OR bus already disabled
            xSemaphoreGive(xPowerBusMutex);
            return ESP_OK;
        }
    }

    return ESP_FAIL; // Failed to acquire mutex initially
}

/**
 * @brief Check if power bus is currently enabled
 *
 * Thread-safe with xPowerBusMutex (100ms timeout).
 *
 * @param bus Power bus to check
 * @return true if bus is powered and not in safety shutdown, false otherwise
 */
bool fluctus_is_bus_powered(power_bus_t bus)
{
    if (bus >= POWER_BUS_COUNT || !fluctus_initialized) {
        return false;
    }

    bool powered = false;
    if (xSemaphoreTake(xPowerBusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_QUICK_MS)) == pdTRUE) {
        powered = system_status.bus_enabled[bus] && !system_status.safety_shutdown;
        xSemaphoreGive(xPowerBusMutex);
    }

    return powered;
}

// ########################## SN74AHCT125 Level Shifter Control ##########################

/**
 * @brief Add consumer to level shifter tracking
 */
static bool fluctus_is_level_shifter_consumer_tracked(const char *consumer_id)
{
    for (int i = 0; i < FLUCTUS_LEVEL_SHIFTER_MAX_CONSUMERS; i++) {
        if (strcmp(system_status.level_shifter_consumers[i], consumer_id) == 0) {
            return true;
        }
    }
    return false;
}

static esp_err_t fluctus_add_level_shifter_consumer(const char *consumer_id)
{
    if (consumer_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Find empty slot or existing consumer
    for (int i = 0; i < FLUCTUS_LEVEL_SHIFTER_MAX_CONSUMERS; i++) {
        if (strlen(system_status.level_shifter_consumers[i]) == 0 ||
            strcmp(system_status.level_shifter_consumers[i], consumer_id) == 0) {
            strncpy(system_status.level_shifter_consumers[i], consumer_id, 15);
            system_status.level_shifter_consumers[i][15] = '\0';
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "No free consumer slots for level shifter");
    return ESP_ERR_NO_MEM;
}

/**
 * @brief Remove consumer from level shifter tracking
 */
static esp_err_t fluctus_remove_level_shifter_consumer(const char *consumer_id)
{
    if (consumer_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Find and remove consumer
    for (int i = 0; i < FLUCTUS_LEVEL_SHIFTER_MAX_CONSUMERS; i++) {
        if (strcmp(system_status.level_shifter_consumers[i], consumer_id) == 0) {
            memset(system_status.level_shifter_consumers[i], 0, 16);
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

/**
 * @brief Update SN74AHCT125 hardware state
 *
 * Hardware control logic:
 * - Open-drain output, LOW=enable (pulls OE to GND, buffers active)
 * - Input mode (high-Z)=disable (10kΩ external pullup keeps OE high, buffers tri-state)
 */
static esp_err_t fluctus_update_level_shifter_hardware(void)
{
    bool enable = system_status.level_shifter_enabled && !system_status.safety_shutdown;

    // Check if hardware state actually needs to change
    if (hardware_level_shifter_state == enable) {
        ESP_LOGV(TAG, "Level shifter hardware already in correct state: %s", enable ? "ENABLED" : "DISABLED");
        return ESP_OK; // No change needed
    }

    esp_err_t ret = ESP_OK;

    if (enable) {
        // Pull OE to ground (OUTPUT mode + LOW = enable buffers)
        ret = gpio_set_direction(FLUCTUS_LEVEL_SHIFTER_OE_GPIO, GPIO_MODE_OUTPUT_OD);
        if (ret == ESP_OK) {
            gpio_set_level(FLUCTUS_LEVEL_SHIFTER_OE_GPIO, 0);
            // Stabilization delay for level shifter propagation
            vTaskDelay(pdMS_TO_TICKS(FLUCTUS_LEVEL_SHIFTER_STABILIZATION_MS));
        }
    } else {
        // Float the pin (INPUT mode = high-Z, external pullup disables buffers)
        ret = gpio_set_direction(FLUCTUS_LEVEL_SHIFTER_OE_GPIO, GPIO_MODE_INPUT);
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update level shifter hardware: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update hardware state cache
    hardware_level_shifter_state = enable;
    ESP_LOGD(TAG, "Level shifter hardware updated: %s", enable ? "ENABLED" : "DISABLED");
    return ESP_OK;
}

/**
 * @brief Request SN74AHCT125 level shifter enable (reference counted)
 */
esp_err_t fluctus_request_level_shifter(const char *consumer_id)
{
    if (consumer_id == NULL || !fluctus_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xPowerBusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
        if (system_status.safety_shutdown) {
            xSemaphoreGive(xPowerBusMutex);
            return ESP_ERR_INVALID_STATE;
        }

        // Add consumer and increment reference count (skip if already tracked)
        bool already_tracked = fluctus_is_level_shifter_consumer_tracked(consumer_id);
        fluctus_add_level_shifter_consumer(consumer_id);
        if (!already_tracked) {
            system_status.level_shifter_ref_count++;
        }

        // Enable level shifter if not already enabled
        if (!system_status.level_shifter_enabled) {
            system_status.level_shifter_enabled = true;
            fluctus_update_level_shifter_hardware();
            ESP_LOGI(TAG, "Level shifter enabled for: %s", consumer_id);
        } else {
            ESP_LOGD(TAG,
                     "Level shifter consumer added: %s (ref_count=%d)",
                     consumer_id,
                     system_status.level_shifter_ref_count);
        }

        xSemaphoreGive(xPowerBusMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief Release SN74AHCT125 level shifter (decrements reference count)
 */
esp_err_t fluctus_release_level_shifter(const char *consumer_id)
{
    if (consumer_id == NULL || !fluctus_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xPowerBusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
        // Remove consumer and decrement reference count
        fluctus_remove_level_shifter_consumer(consumer_id);
        if (system_status.level_shifter_ref_count > 0) {
            system_status.level_shifter_ref_count--;
        }

        // Disable level shifter if no more consumers
        if (system_status.level_shifter_ref_count == 0 && system_status.level_shifter_enabled) {
            system_status.level_shifter_enabled = false;
            fluctus_update_level_shifter_hardware();
            ESP_LOGI(TAG, "Level shifter disabled (no consumers)");
        } else {
            ESP_LOGD(TAG,
                     "Level shifter consumer released: %s (ref_count=%d)",
                     consumer_id,
                     system_status.level_shifter_ref_count);
        }

        xSemaphoreGive(xPowerBusMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

// ########################## Hall Array Control ##########################

/**
 * @brief Enable or disable hall array power (N-MOSFET control)
 *
 * Controls power to the hall sensor array used for TEMPESTA wind direction.
 * NOTE: Now controlled via MCP23008 GP5 instead of direct GPIO.
 * Saves 14mA when disabled, turned ON only during wind direction readings.
 *
 * @param enable true to power on hall array, false to power off
 */
void fluctus_hall_array_enable(bool enable)
{
    esp_err_t ret = mcp23008_helper_set_hall_enable(enable);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set hall array enable: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Hall array power: %s", enable ? "ON" : "OFF");
    }
}
