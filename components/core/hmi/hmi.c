/**
 * @file hmi.c
 * @brief HMI - Human-Machine Interface with custom rendering system
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Original implementation of HMI - complete UI system built from scratch.
 *
 * Key features:
 * - Custom 5x8 bitmap font (475 bytes, no external dependencies)
 * - 50-state hierarchical menu system
 * - SH1106 128x64 OLED with framebuffer rendering
 * - EC11 rotary encoder input (MCP23008 I2C expander)
 * - Variable refresh rates (1Hz static, 4Hz realtime)
 * - Power-aware with auto sleep and wake-on-input
 * - Complete system control and monitoring
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "hmi.h"
#include "hmi_private.h"
#include "mcp23008_helper.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>


// ########################## Constants and Variables ##########################

static const char *TAG = "HMI";

// Global status structure (shared via hmi_private.h)
hmi_status_t hmi_status = {
    .initialized = false,
    .display_active = false,
    .current_menu = HMI_MENU_MAIN,
    .selected_item = 0,
    .last_activity_time = 0,
    .encoder_count = 0,
    .blink_state = false,
    .blink_counter = 0,
    .scroll_offset = 0,
    .menu_scroll_memory = {0},  // Initialize all 47 elements to 0
    .menu_selected_item_memory = {0},  // Initialize all 47 elements to 0
    .current_page = 0,
    .total_pages = 0
};

// Mutex for thread-safe operations (shared via hmi_private.h)
SemaphoreHandle_t xHmiMutex = NULL;

// Task handles
static TaskHandle_t xHmiDisplayTask = NULL;

// Stellaria control page editing state (shared via hmi_private.h)
bool stellaria_intensity_editing = false;
uint8_t stellaria_intensity_percent = 50;  // 0-100%

// IMPLUVIUM zone editing state (shared via hmi_private.h)
bool zone_editing = false;
uint8_t editing_zone_id = 0;                    // 0-4
bool editing_zone_enabled = true;
float editing_zone_target = 45;              // 20-80%
float editing_zone_deadband = 5;             // 1-20%

// IMPLUVIUM manual water input state (shared via hmi_private.h)
bool manual_water_input = false;
uint8_t manual_water_zone = 0;
uint16_t manual_water_duration = 30;           // 5-300 seconds

// Interval editing state (for FLUCTUS/TEMPESTA/IMPLUVIUM/SYSTEM interval pages)
bool interval_editing = false;
uint32_t editing_interval_value = 0;           // Current value being edited (minutes or hours)

// Confirmation dialog state (shared by IMPLUVIUM, TEMPESTA, and SYSTEM)
// Note: confirm_action_t enum defined in hmi_private.h
confirm_action_t confirmation_action = CONFIRM_NONE;
uint8_t confirmation_param = 0;  // For zone ID or other params
hmi_menu_state_t confirmation_return_menu = HMI_MENU_MAIN;  // Menu to return to after confirmation

// Servo debug state (for FLUCTUS manual servo control, shared via hmi_private.h)
bool servo_debug_active = false;           // True when in servo control mode
time_t servo_debug_start_time = 0;         // Timestamp when control mode started (for timeout)
bool servo_debug_bus_requested = false;    // True if we have 6.6V bus power
uint32_t servo_debug_current_duty = 0;     // Current servo duty cycle (tracked locally during control)

// ########################## Private Function Declarations ##########################

// Task functions
static void hmi_display_task(void *pvParameters);

// Activity tracking
void hmi_update_activity(void);
static bool hmi_check_timeout(void);

// ########################## Activity Tracking ##########################

/**
 * @brief Update last activity timestamp
 */
void hmi_update_activity(void)
{
    hmi_status.last_activity_time = time(NULL);
}

/**
 * @brief Check if display timeout has been reached
 * Uses different timeouts for realtime pages (60s) vs normal pages (30s)
 */
static bool hmi_check_timeout(void)
{
    if (!hmi_status.display_active) {
        return false;  // Already off
    }

    time_t now = time(NULL);
    int64_t elapsed_ms = (now - hmi_status.last_activity_time) * 1000;

    // Use longer timeout for realtime pages
    int64_t timeout_ms = hmi_is_realtime_page() ?
                         HMI_DISPLAY_TIMEOUT_REALTIME_MS :
                         HMI_DISPLAY_TIMEOUT_NORMAL_MS;

    return (elapsed_ms >= timeout_ms);
}

// ########################## Task Functions ##########################

/**
 * @brief HMI display task - handles display refresh and timeout
 * Implements variable refresh rate: 250ms for realtime pages, 1000ms for normal pages
 */
static void hmi_display_task(void *pvParameters)
{
    ESP_LOGI(TAG, "HMI display task started");

    TickType_t last_wake_time = xTaskGetTickCount();
    int last_encoder_count = 0;
    uint32_t notification_value = 0;

    while (1) {
        // Check for button press notification (from ISR)
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &notification_value, 0) == pdTRUE) {
            ESP_LOGI(TAG, "Button press detected");

            // Wake display if off
            if (!hmi_status.display_active) {
                hmi_display_power_on();
            } else {
                // Handle button press (menu navigation)
                if (xSemaphoreTake(xHmiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    hmi_handle_button_press();
                    xSemaphoreGive(xHmiMutex);
                }
            }
        }

        // Read encoder count and check for changes (from MCP23008 helper)
        if (hmi_status.display_active) {
            int32_t current_count = mcp23008_helper_get_encoder_count();

            int delta = current_count - last_encoder_count;

            // Process any change (encoder helper now outputs detent-based counts)
            // NOTE: mcp23008_helper now handles detent division internally (4 quadrature states = 1 detent)
            if (delta != 0) {
                if (xSemaphoreTake(xHmiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    hmi_handle_encoder_change(delta);  // Already normalized by encoder helper
                    xSemaphoreGive(xHmiMutex);
                }
                last_encoder_count = current_count;
                ESP_LOGV(TAG, "Encoder delta: %d (count: %ld)", delta, current_count);
            }
        }

        // Check for display timeout
        if (hmi_check_timeout()) {
            ESP_LOGI(TAG, "Display timeout - powering off");
            hmi_display_power_off();
        }

        // Check servo control timeout (60 seconds)
        if (servo_debug_active) {
            time_t now = time(NULL);
            int64_t elapsed_s = now - servo_debug_start_time;
            if (elapsed_s >= 60) {
                ESP_LOGW(TAG, "Servo control timeout (%lld s) - exiting control mode", (long long)elapsed_s);

                // Release 6.6V bus power if we requested it
                if (servo_debug_bus_requested) {
                    fluctus_release_bus_power(POWER_BUS_6V6, "HMI_SERVO_DEBUG");
                    servo_debug_bus_requested = false;
                }

                servo_debug_active = false;
                servo_debug_start_time = 0;
                servo_debug_current_duty = 0;

                // Return to servo debug menu
                if (xSemaphoreTake(xHmiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    hmi_status.current_menu = HMI_MENU_FLUCTUS_SERVO_DEBUG;
                    hmi_status.selected_item = 0;
                    xSemaphoreGive(xHmiMutex);
                }
            }
        }

        // Update blink indicator and render menu if display is active
        if (hmi_status.display_active) {
            if (xSemaphoreTake(xHmiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                // Update blink state (toggle every 2 cycles = 500ms for 4Hz realtime, every cycle for 1Hz static)
                if (hmi_is_realtime_page()) {
                    // Realtime pages: blink at 1Hz (every 2 cycles @ 250ms = 500ms)
                    hmi_status.blink_counter++;
                    if (hmi_status.blink_counter >= 2) {
                        hmi_status.blink_state = !hmi_status.blink_state;
                        hmi_status.blink_counter = 0;
                    }
                } else {
                    // Static pages: blink at 0.5Hz (every cycle @ 1000ms = 1s)
                    hmi_status.blink_state = !hmi_status.blink_state;
                }

                hmi_fb_clear();
                hmi_render_menu();
                hmi_fb_flush();
                xSemaphoreGive(xHmiMutex);
            }
        }

        // Variable refresh rate: fast for realtime pages, slow for normal pages
        TickType_t delay_ms = hmi_is_realtime_page() ?
                             HMI_REFRESH_RATE_REALTIME_MS :
                             HMI_REFRESH_RATE_NORMAL_MS;
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(delay_ms));
    }
}

// ########################## Public API Functions ##########################

/**
 * @brief Initialize HMI system
 */
esp_err_t hmi_init(void)
{
    ESP_LOGI(TAG, "Initializing HMI system...");

    // Create mutex
    xHmiMutex = xSemaphoreCreateMutex();
    if (xHmiMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create HMI mutex");
        return ESP_FAIL;
    }

    // Initialize display
    esp_err_t ret = hmi_display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize display: %s", esp_err_to_name(ret));
        vSemaphoreDelete(xHmiMutex);
        return ret;
    }

    // Create display task
    BaseType_t task_ret = xTaskCreate(
        hmi_display_task,
        "hmi_display",
        4096,  // Stack size
        NULL,
        5,     // Priority
        &xHmiDisplayTask
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display task");
        vSemaphoreDelete(xHmiMutex);
        return ESP_FAIL;
    }

    // Register button callback with MCP23008 helper
    // NOTE: Encoder/button hardware managed by MCP23008 helper (initialized earlier in main)
    ret = mcp23008_helper_register_button_callback(xHmiDisplayTask);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register button callback: %s", esp_err_to_name(ret));
        // Continue anyway - non-critical, button just won't work
    } else {
        ESP_LOGI(TAG, "Button callback registered with MCP23008 helper");
    }

    // Mark as initialized
    hmi_status.initialized = true;

    ESP_LOGI(TAG, "HMI system initialized successfully (encoder via MCP23008)");
    return ESP_OK;
}

/**
 * @brief Get current HMI status
 */
esp_err_t hmi_get_status(hmi_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!hmi_status.initialized) {
        memset(status, 0, sizeof(hmi_status_t));
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xHmiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(status, &hmi_status, sizeof(hmi_status_t));
        xSemaphoreGive(xHmiMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief Manually wake display
 */
esp_err_t hmi_wake_display(void)
{
    if (!hmi_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xHmiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        esp_err_t ret = hmi_display_power_on();
        xSemaphoreGive(xHmiMutex);
        return ret;
    }

    return ESP_FAIL;
}
