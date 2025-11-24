/**
 * @file fluctus_power_bus.c
 * @brief FLUCTUS power bus control module
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Manages 4-bus power distribution system with reference counting:
 * - 3.3V sensor bus (push-pull via SN74AHCT125N buffer)
 * - 5V bus (open-drain to buck converter EN pin)
 * - 6.6V servo bus (open-drain, inverted logic)
 * - 12V bus (open-drain to buck converter EN pin)
 *
 * Features:
 * - Reference counting prevents premature bus shutdown
 * - Consumer tracking for debugging (max 8 per bus)
 * - Hardware state caching to minimize GPIO operations
 * - Hall array MOSFET control (N-MOSFET for TEMPESTA wind direction)
 * - Event logging for power management actions
 *
 * THREAD SAFETY:
 * - Protected by xPowerBusMutex (100ms timeout)
 * - Safe to call from any task
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "fluctus.h"
#include "fluctus_private.h"
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = "FLUCTUS_P_BUS";

// ########################## Module State ##########################
// Note: NO 'static' keyword - these are accessed via extern in fluctus_private.h

SemaphoreHandle_t xPowerBusMutex = NULL;

// Power bus system status
fluctus_power_status_t system_status = {
    .bus_enabled = {false, false, false, false},
    .bus_ref_count = {0, 0, 0, 0},
    .power_state = FLUCTUS_POWER_STATE_NORMAL,
    .solar_tracking_state = SOLAR_TRACKING_DISABLED,
    .current_yaw_duty = FLUCTUS_SERVO_CENTER_DUTY,
    .current_pitch_duty = FLUCTUS_SERVO_CENTER_DUTY,
    .safety_shutdown = false,
    .manual_reset_required = false,
    .last_activity_time = 0
};

// Hardware state cache to avoid redundant GPIO operations
bool hardware_bus_state[POWER_BUS_COUNT] = {false, false, false, false};

// ########################## GPIO Initialization ##########################


/**
 * @brief Initialize GPIO pins for power control - New Hardware Configuration
 *
 * 3.3V Bus: Push-pull output, LOW=OFF, HIGH=ON (controls N-MOSFET via SN74AHCT125N)
 * 5V Bus:   Open-drain output, LOW=OFF, INPUT=ON (100kΩ series to buck EN)
 * 6.6V Bus: Open-drain output, LOW=ON, INPUT=OFF (inverted logic, 100kΩ pullup to 3.3V)
 * 12V Bus:  Push-pull, HIGH=ON, LOW=OFF (N-MOSFET)
 */
esp_err_t fluctus_gpio_init(void)
{
    ESP_LOGI(TAG, "Initializing power control GPIOs...");
    esp_err_t ret;

    // 3.3V, 12 V Buses and 2 devices: Push-pull output (controls N-MOSFETs)
    gpio_config_t gpio_pp_conf = {
        .pin_bit_mask = (1ULL << FLUCTUS_3V3_BUS_ENABLE_GPIO) | // controls N-MOSFET via buffer
                        (1ULL << FLUCTUS_3V3_HALL_ARRAY_ENABLE_GPIO) |
                        (1ULL << FLUCTUS_12V_BUS_ENABLE_GPIO) |
                        (1ULL << FLUCTUS_12V_FAN_ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,  // Push-pull (NOT open-drain)
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ret = gpio_config(&gpio_pp_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure push-pull GPIOs: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set initial states (bus/individual devices OFF)
    gpio_set_level(FLUCTUS_3V3_BUS_ENABLE_GPIO, 0); // 3.3V: LOW = OFF
    gpio_set_level(FLUCTUS_3V3_HALL_ARRAY_ENABLE_GPIO, 0);
    gpio_set_level(FLUCTUS_12V_BUS_ENABLE_GPIO, 0); // 12V: LOW = OFF
    gpio_set_level(FLUCTUS_12V_FAN_ENABLE_GPIO, 0);

    // 5V, 6.6V Buses: Open-drain outputs
    gpio_config_t buck_od_conf = {
        .pin_bit_mask = (1ULL << FLUCTUS_5V_BUS_ENABLE_GPIO) |
                        (1ULL << FLUCTUS_6V6_BUS_ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT_OD,  // Open-drain
        .pull_up_en = GPIO_PULLUP_DISABLE,  // No internal pull-ups (external hardware)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ret = gpio_config(&buck_od_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure open-drain buck GPIOs: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set initial states (all buses OFF)
    gpio_set_level(FLUCTUS_5V_BUS_ENABLE_GPIO, 0);   // 5V: LOW = OFF
    // For 6.6V inverted logic: Set as input (high-Z) to keep it floating and allow external pullup to turn it OFF by appling 3.3V
    gpio_set_direction(FLUCTUS_6V6_BUS_ENABLE_GPIO, GPIO_MODE_INPUT);

    // Initialize hardware state cache to disabled
    for (int i = 0; i < POWER_BUS_COUNT; i++) {
        hardware_bus_state[i] = false;
    }

    ESP_LOGI(TAG, "Power control GPIOs initialized (3.3V, 12V=push-pull, 5V, 6.6V=open-drain, all OFF)");
    ESP_LOGI(TAG, "  3.3V: LOW (OFF), 5V: LOW (OFF), 6.6V: INPUT/HIGH (OFF-inverted), 12V: LOW (OFF)");
    return ESP_OK;
}

// ########################## Bus Control Functions ##########################

/**
 * @brief Update hardware state for specific power bus
 *
 * Hardware control logic per bus:
 * - 3.3V: Push-pull, HIGH=ON, LOW=OFF (buffer to N-MOSFET)
 * - 5V:   Open-drain, INPUT=ON (float), OUTPUT+LOW=OFF (extn 100kΩ to buck EN)
 * - 6.6V: Open-drain, OUTPUT+LOW=ON, INPUT=OFF (inverted logic, 100kΩ pullup)
 * - 12V:  Push-pull, HIGH=ON, LOW=OFF (N-MOSFET)
 */
esp_err_t fluctus_update_bus_hardware(power_bus_t bus)
{
    if (bus >= POWER_BUS_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    bool enable = system_status.bus_enabled[bus] && !system_status.safety_shutdown;

    // Check if hardware state actually needs to change
    if (hardware_bus_state[bus] == enable) {
        ESP_LOGV(TAG, "Bus %d hardware already in correct state: %s", bus, enable ? "ENABLED" : "DISABLED");
        return ESP_OK;  // No change needed
    }

    esp_err_t ret = ESP_OK;

    switch (bus) {
        case POWER_BUS_3V3:
            // 3.3V: Push-pull output, HIGH=ON, LOW=OFF
            gpio_set_level(FLUCTUS_3V3_BUS_ENABLE_GPIO, enable ? 1 : 0);
            break;

        case POWER_BUS_5V:
            // 5V: Open-drain, floating=ON, LOW=OFF
            if (enable) {
                // Float the pin (INPUT mode = high-Z)
                ret = gpio_set_direction(FLUCTUS_5V_BUS_ENABLE_GPIO, GPIO_MODE_INPUT);
            } else {
                // Pull to ground (OUTPUT mode + LOW)
                ret = gpio_set_direction(FLUCTUS_5V_BUS_ENABLE_GPIO, GPIO_MODE_OUTPUT_OD);
                if (ret == ESP_OK) {
                    gpio_set_level(FLUCTUS_5V_BUS_ENABLE_GPIO, 0);
                }
            }
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
                // 12V: Push-pull output, HIGH=ON, LOW=OFF
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
    ESP_LOGD(TAG, "Bus %d hardware updated: %s", bus, enable ? "ENABLED" : "DISABLED");
    return ESP_OK;
}

/**
 * @brief Add consumer to bus tracking
 */
esp_err_t fluctus_add_consumer(power_bus_t bus, const char* consumer_id)
{
    if (bus >= POWER_BUS_COUNT || consumer_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find empty slot or existing consumer
    for (int i = 0; i < 8; i++) {
        if (strlen(system_status.bus_consumers[bus][i]) == 0 || 
            strcmp(system_status.bus_consumers[bus][i], consumer_id) == 0) {
            strncpy(system_status.bus_consumers[bus][i], consumer_id, 15);
            system_status.bus_consumers[bus][i][15] = '\0';
            return ESP_OK;
        }
    }
    
    ESP_LOGW(TAG, "No free consumer slots for bus %d", bus);
    return ESP_ERR_NO_MEM;
}

/**
 * @brief Remove consumer from bus tracking
 *
 * Finds and removes the consumer from the bus consumer list.
 * Called during fluctus_release_bus_power().
 */
esp_err_t fluctus_remove_consumer(power_bus_t bus, const char* consumer_id)
{
    if (bus >= POWER_BUS_COUNT || consumer_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find and remove consumer
    for (int i = 0; i < 8; i++) {
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
void fluctus_log_active_consumers(const char* event_context)
{
    const char* bus_names[] = {"3V3", "5V", "6V6", "12V"};
    bool any_consumers = false;
    
    ESP_LOGE(TAG, "%s - Active consumers:", event_context);
    
    for (int bus = 0; bus < POWER_BUS_COUNT; bus++) {
        if (system_status.bus_enabled[bus] && system_status.bus_ref_count[bus] > 0) {
            ESP_LOGE(TAG, "  %s bus (%d consumers):", bus_names[bus], system_status.bus_ref_count[bus]);
            for (int i = 0; i < 8; i++) {
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
void fluctus_log_power_event(fluctus_event_type_t event_type, power_bus_t bus, 
                                   const char* consumer_id)
{
    const char* event_names[] = {
        "BUS_ON", "BUS_OFF", "CONSUMER_REQUEST", "CONSUMER_RELEASE",
        "OVERCURRENT_WARNING", "OVERCURRENT_SHUTDOWN", "BATTERY_LOW",
        "SOLAR_TRACKING_PARK", "POWER_STATE_CHANGE"
    };
    
    const char* bus_names[] = {"3V3", "5V", "6V6", "12V", "SYSTEM"};
    
    if (event_type < sizeof(event_names)/sizeof(event_names[0])) {
        if (bus < POWER_BUS_COUNT) {
            ESP_LOGI(TAG, "Event: %s [%s] %s", event_names[event_type], bus_names[bus],
                     consumer_id ? consumer_id : "");
        } else {
            ESP_LOGI(TAG, "Event: %s %s", event_names[event_type],
                     consumer_id ? consumer_id : "");
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
esp_err_t fluctus_request_bus_power(power_bus_t bus, const char* consumer_id)
{
    if (bus >= POWER_BUS_COUNT || consumer_id == NULL || !fluctus_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(xPowerBusMutex, pdMS_TO_TICKS(FLUCTUS_MUTEX_TIMEOUT_LONG_MS)) == pdTRUE) {
        if (system_status.safety_shutdown) {
            xSemaphoreGive(xPowerBusMutex);
            return ESP_ERR_INVALID_STATE;
        }
        
        // Add consumer and increment reference count
        fluctus_add_consumer(bus, consumer_id);
        system_status.bus_ref_count[bus]++;
        
        // Enable bus if not already enabled
        if (!system_status.bus_enabled[bus]) {
            system_status.bus_enabled[bus] = true;
            fluctus_update_bus_hardware(bus);
            fluctus_log_power_event(FLUCTUS_EVENT_BUS_ON, bus, consumer_id);
            
            // Notify monitoring task of power activity to start constant monitoring
            if (xFluctusMonitoringTaskHandle != NULL) {
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
esp_err_t fluctus_release_bus_power(power_bus_t bus, const char* consumer_id)
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
        
        // Disable bus if no more consumers
        if (system_status.bus_ref_count[bus] == 0 && system_status.bus_enabled[bus]) {
            system_status.bus_enabled[bus] = false;
            fluctus_update_bus_hardware(bus);
            fluctus_log_power_event(FLUCTUS_EVENT_BUS_OFF, bus, consumer_id);
        } else {
            fluctus_log_power_event(FLUCTUS_EVENT_CONSUMER_RELEASE, bus, consumer_id);
        }
        
        xSemaphoreGive(xPowerBusMutex);
        return ESP_OK;
    }
    
    return ESP_FAIL;
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

/**
 * @brief Enable or disable hall array power (N-MOSFET control)
 *
 * Controls power to the hall sensor array used for TEMPESTA wind direction.
 * Saves 14mA when disabled, turned ON only during wind direction readings.
 *
 * @param enable true to power on hall array, false to power off
 */
void fluctus_hall_array_enable(bool enable)
{
    gpio_set_level(FLUCTUS_3V3_HALL_ARRAY_ENABLE_GPIO, enable ? 1 : 0);
    ESP_LOGD(TAG, "Hall array power: %s", enable ? "ON" : "OFF");
}

