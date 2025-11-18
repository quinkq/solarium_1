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
 * - EC11 rotary encoder input (PCNT-based)
 * - Variable refresh rates (1Hz static, 4Hz realtime)
 * - Power-aware with auto sleep and wake-on-input
 * - Complete system control and monitoring
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "hmi.h"
#include "hmi_private.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/pulse_cnt.h"
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
    .blink_counter = 0
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

// Hardware handles
static pcnt_unit_handle_t encoder_pcnt_unit = NULL;

// ########################## Private Function Declarations ##########################

// Encoder functions
static esp_err_t hmi_encoder_init(void);
static void hmi_encoder_button_isr(void *arg);

// Task functions
static void hmi_display_task(void *pvParameters);

// Activity tracking
void hmi_update_activity(void);
static bool hmi_check_timeout(void);

// ########################## Encoder Functions ##########################

/**
 * @brief Initialize EC11 rotary encoder with PCNT
 */
static esp_err_t hmi_encoder_init(void)
{
    ESP_LOGI(TAG, "Initializing EC11 rotary encoder (PCNT)...");

    // Create PCNT unit for quadrature decoding
    // Note: One PCNT unit can handle both encoder channels (A and B)
    pcnt_unit_config_t unit_config = {
        .high_limit = HMI_PCNT_HIGH_LIMIT,
        .low_limit = HMI_PCNT_LOW_LIMIT,
    };

    esp_err_t ret = pcnt_new_unit(&unit_config, &encoder_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT unit: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure channel A (primary encoder signal)
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = HMI_ENCODER_A_GPIO,
        .level_gpio_num = HMI_ENCODER_B_GPIO,  // Use B as control signal for quadrature
    };

    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ret = pcnt_new_channel(encoder_pcnt_unit, &chan_a_config, &pcnt_chan_a);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT channel A: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure channel B (secondary encoder signal)
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = HMI_ENCODER_B_GPIO,
        .level_gpio_num = HMI_ENCODER_A_GPIO,  // Use A as control signal for quadrature
    };

    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ret = pcnt_new_channel(encoder_pcnt_unit, &chan_b_config, &pcnt_chan_b);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT channel B: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set edge and level actions for quadrature decoding
    // Channel A: increment on positive edge when B is low, decrement when B is high
    pcnt_channel_set_edge_action(pcnt_chan_a,
                                  PCNT_CHANNEL_EDGE_ACTION_INCREASE,  // Pos edge
                                  PCNT_CHANNEL_EDGE_ACTION_DECREASE); // Neg edge
    pcnt_channel_set_level_action(pcnt_chan_a,
                                   PCNT_CHANNEL_LEVEL_ACTION_KEEP,    // B high
                                   PCNT_CHANNEL_LEVEL_ACTION_INVERSE); // B low

    // Channel B: increment on positive edge when A is high, decrement when A is low
    pcnt_channel_set_edge_action(pcnt_chan_b,
                                  PCNT_CHANNEL_EDGE_ACTION_INCREASE,  // Pos edge
                                  PCNT_CHANNEL_EDGE_ACTION_DECREASE); // Neg edge
    pcnt_channel_set_level_action(pcnt_chan_b,
                                   PCNT_CHANNEL_LEVEL_ACTION_INVERSE, // A high
                                   PCNT_CHANNEL_LEVEL_ACTION_KEEP);   // A low

    // Enable and start the PCNT unit
    ret = pcnt_unit_enable(encoder_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable PCNT unit: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_clear_count(encoder_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear PCNT count: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_start(encoder_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start PCNT unit: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure button GPIO (external pull-up already present)
    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << HMI_ENCODER_BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,    // External pull-up present
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,       // Trigger on both press and release
    };

    ret = gpio_config(&btn_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure button GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    // Install GPIO ISR service and attach handler
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means ISR service already installed (by another component)
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_isr_handler_add(HMI_ENCODER_BTN_GPIO, hmi_encoder_button_isr, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add button ISR handler: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Encoder initialized - PCNT unit handles both channels (A=%d, B=%d, BTN=%d)",
             HMI_ENCODER_A_GPIO, HMI_ENCODER_B_GPIO, HMI_ENCODER_BTN_GPIO);

    return ESP_OK;
}

/**
 * @brief Encoder button ISR handler
 */
static void IRAM_ATTR hmi_encoder_button_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Read button state
    int level = gpio_get_level(HMI_ENCODER_BTN_GPIO);

    if (level == 0) {
        // Button pressed (active low with external pull-up)
        // Notify display task to wake and handle button event
        if (xHmiDisplayTask != NULL) {
            xTaskNotifyFromISR(xHmiDisplayTask, 1, eSetBits, &xHigherPriorityTaskWoken);
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

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

        // Read encoder count and check for changes
        if (hmi_status.display_active) {
            int current_count = 0;
            pcnt_unit_get_count(encoder_pcnt_unit, &current_count);

            int delta = current_count - last_encoder_count;

            // Only process significant changes (debounce noise)
            if (delta >= 4 || delta <= -4) {
                if (xSemaphoreTake(xHmiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    hmi_handle_encoder_change(delta / 4);  // Normalize to +/-1 per detent
                    xSemaphoreGive(xHmiMutex);
                }
                last_encoder_count = current_count;
                ESP_LOGD(TAG, "Encoder delta: %d (count: %d)", delta / 4, current_count);
            }
        }

        // Check for display timeout
        if (hmi_check_timeout()) {
            ESP_LOGI(TAG, "Display timeout - powering off");
            hmi_display_power_off();
        }

        // Update blink indicator and render menu if display is active
        if (hmi_status.display_active) {
            if (xSemaphoreTake(xHmiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                // Update blink state (toggle every 2 cycles = 500ms with 250ms refresh)
                if (hmi_is_realtime_page()) {
                    hmi_status.blink_counter++;
                    if (hmi_status.blink_counter >= 2) {
                        hmi_status.blink_state = !hmi_status.blink_state;
                        hmi_status.blink_counter = 0;
                    }
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

    // Initialize encoder
    esp_err_t ret = hmi_encoder_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize encoder: %s", esp_err_to_name(ret));
        vSemaphoreDelete(xHmiMutex);
        return ret;
    }

    // Initialize display
    ret = hmi_display_init();
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

    // Mark as initialized
    hmi_status.initialized = true;

    ESP_LOGI(TAG, "HMI system initialized successfully");
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
