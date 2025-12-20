/**
 * @file mcp23008_helper.c
 * @brief MCP23008 I2C GPIO Expander Integration Implementation
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Implements interrupt-driven handling of encoder, pulse counters, and outputs
 * via MCP23008 I2C GPIO expander. Replaces hardware PCNT functionality with
 * software quadrature decoding and pulse counting.
 *
 * Architecture:
 * - ESP32 GPIO ISR notifies handler task via xTaskNotify
 * - Handler task reads INTCAP/INTF registers and dispatches to handlers
 * - Encoder: Software quadrature table-based decoding
 * - Pulse counters: Simple increment on interrupt
 * - Thread safety: Spinlock for shared counters
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "mcp23008_helper.h"
#include "main.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "MCP23008_HELPER";

// ########################## Global State ##########################

// MCP23008 device descriptor
static i2c_dev_t mcp_dev;
static bool mcp_initialized = false;

// Handler task handle
static TaskHandle_t xMcp23008HandlerTask = NULL;

// Spinlock for thread-safe counter access
static portMUX_TYPE mcp_spinlock = portMUX_INITIALIZER_UNLOCKED;

// Encoder state
static volatile int32_t encoder_count = 0;       // User-facing count (increments per detent)
static uint8_t last_ab = 0;                      // Last encoder A/B state (2 bits)
static int8_t detent_accumulator = 0;            // Accumulator for current detent (0-3 or 0 to -3)

// Pulse counters
static volatile uint32_t rainfall_pulse_count = 0;
static volatile uint32_t tank_pulse_count = 0;

// Button state
static volatile bool button_pressed = false;
static volatile bool last_button_pressed = false;  // For edge detection

// Button callback task handle
static TaskHandle_t button_notify_task = NULL;

// ########################## Quadrature Decoding ##########################

/**
 * Quadrature decoding lookup table
 * Index: [last_AB_state << 2 | current_AB_state]
 * Value: Delta to apply to encoder count (-1, 0, +1)
 *
 * Based on Gray code quadrature encoding:
 * - Valid transitions change only 1 bit
 * - Invalid transitions (noise) produce 0 delta
 */
static const int8_t quadrature_table[16] = {
    0,  -1,  1,  0,   // 00->00, 00->01, 00->10, 00->11
    1,   0,  0, -1,   // 01->00, 01->01, 01->10, 01->11
   -1,   0,  0,  1,   // 10->00, 10->01, 10->10, 10->11
    0,   1, -1,  0    // 11->00, 11->01, 11->10, 11->11
};

// ########################## Internal Handlers ##########################

/**
 * @brief Handle encoder state change
 *
 * Performs simple software quadrature decoding.
 * Called from handler task when encoder interrupt detected.
 *
 * Algorithm:
 * 1. Filter duplicate states (noise/multiple interrupts)
 * 2. Filter invalid transitions (impossible state jumps)
 * 3. Accumulate to ±4 steps = 1 user detent
 *
 * @param gpio_state Captured GPIO state from INTCAP register
 */
static void handle_encoder_change(uint8_t gpio_state)
{
    // Extract encoder A and B from captured state (GP0=A, GP1=B)
    uint8_t current_ab = gpio_state & 0x03;  // Bits 0-1

    // FILTER 1: Ignore duplicate states (noise/multiple interrupts for same position)
    if (current_ab == last_ab) {
        ESP_LOGD(TAG, "ENC: FILTERED duplicate (AB=%d%d)",
                 (current_ab >> 1) & 1, current_ab & 1);
        return;
    }

    // Look up direction in quadrature table
    uint8_t index = (last_ab << 2) | current_ab;
    int8_t delta = quadrature_table[index];

    // FILTER 2: Invalid transitions (impossible state jumps)
    if (delta == 0) {
        ESP_LOGD(TAG, "ENC: FILTERED invalid | AB: %d%d->%d%d",
                 (last_ab >> 1) & 1, last_ab & 1,
                 (current_ab >> 1) & 1, current_ab & 1);
        last_ab = current_ab;
        return;
    }

    // Valid transition - update accumulator
    // int8_t old_accum = detent_accumulator;
    detent_accumulator += delta;

    // DEBUG: Track cumulative delta to detect noise cancellation
    static int32_t total_delta = 0;
    total_delta += delta;

    // Check if we've completed a detent (±4 steps)
    // int32_t old_user_count;
    portENTER_CRITICAL(&mcp_spinlock);
    // old_user_count = encoder_count;

    if (detent_accumulator >= 4) {
        encoder_count++;
        detent_accumulator = 0;
    } else if (detent_accumulator <= -4) {
        encoder_count--;
        detent_accumulator = 0;
    }


    portEXIT_CRITICAL(&mcp_spinlock);
    /*
    int32_t new_user_count = encoder_count;
    ESP_LOGD(TAG, "ENC: %s%d | AB:%d%d->%d%d | acc=%d->%d | total=%ld | usr=%ld%s",
             delta > 0 ? "+" : "", delta,
             (last_ab >> 1) & 1, last_ab & 1,
             (current_ab >> 1) & 1, current_ab & 1,
             old_accum, detent_accumulator,
             total_delta,
             new_user_count,
             (new_user_count != old_user_count) ? " [DETENT]" : "");
    */
    // Update state tracking
    last_ab = current_ab;
}

/**
 * @brief Handle encoder button state change
 *
 * Updates cached button state and notifies registered task on press edge.
 * Detects falling edge (button press) and sends task notification to HMI.
 *
 * @param gpio_state Captured GPIO state from INTCAP register
 */
static void handle_button_change(uint8_t gpio_state)
{
    // GP2 is active-low (pressed = 0, released = 1)
    bool is_pressed = !(gpio_state & (1 << MCP_PIN_ENCODER_BTN));

    portENTER_CRITICAL(&mcp_spinlock);
    bool was_pressed = last_button_pressed;
    button_pressed = is_pressed;
    last_button_pressed = is_pressed;
    portEXIT_CRITICAL(&mcp_spinlock);

    // Notify registered task on falling edge (button press)
    // NOTE: Called from task context, not ISR, so use xTaskNotify (not FromISR variant)
    if (is_pressed && !was_pressed && button_notify_task != NULL) {
        xTaskNotify(button_notify_task, 1, eSetBits);
    }
}

/**
 * @brief Handle rainfall pulse
 *
 * Simple increment on pulse detection. TEMPESTA component reads and
 * resets this counter hourly for accumulation.
 */
static void handle_rainfall_pulse(void)
{
    portENTER_CRITICAL(&mcp_spinlock);
    rainfall_pulse_count++;
    portEXIT_CRITICAL(&mcp_spinlock);
}

/**
 * @brief Handle tank intake pulse
 *
 * Simple increment on pulse detection. TEMPESTA component reads and
 * resets this counter for tank volume tracking.
 */
static void handle_tank_pulse(void)
{
    portENTER_CRITICAL(&mcp_spinlock);
    tank_pulse_count++;
    portEXIT_CRITICAL(&mcp_spinlock);
}

// ########################## Interrupt Handling ##########################

// DEBUG: Counter to verify ISR is being called
static volatile uint32_t isr_call_count = 0;

/**
 * @brief ESP32 GPIO ISR for MCP23008 INT pin
 *
 * Direct ISR that notifies the handler task. Cannot perform I2C operations
 * from ISR context, so we use task notification for deferred processing.
 *
 * Level-triggered interrupt: Disable interrupt here, task will re-enable
 * after clearing MCP23008 interrupt flags to prevent interrupt storms.
 *
 * @param arg Unused
 */
static void IRAM_ATTR mcp23008_int_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // DEBUG: Count ISR calls
    isr_call_count++;

    // Disable interrupt to prevent storm (task will re-enable after I2C clear)
    gpio_intr_disable(MCP23008_INT_GPIO);

    // Notify handler task (cannot do I2C in ISR)
    if (xMcp23008HandlerTask != NULL) {
        xTaskNotifyFromISR(xMcp23008HandlerTask, 1, eSetBits,
                          &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// DEBUG: Counter for handler task notifications
static volatile uint32_t handler_call_count = 0;

// Error tracking for diagnostics and recovery
static uint32_t consecutive_i2c_errors = 0;
static uint32_t consecutive_spurious_interrupts = 0;
static uint32_t total_i2c_errors = 0;
static uint32_t total_spurious_interrupts = 0;
static uint32_t recovery_attempts = 0;
static uint32_t recovery_successes = 0;

// Throttling to prevent infinite loops during I2C failures
#define MAX_CONSECUTIVE_ERRORS 10
#define ERROR_BACKOFF_MS 100
#define SPURIOUS_INT_THRESHOLD 5

/**
 * @brief Attempt to recover MCP23008 from error state
 *
 * Tries to clear any pending interrupts by reading GPIO register
 * and verifying I2C communication is working.
 *
 * @return ESP_OK if recovery succeeded, error otherwise
 */
static esp_err_t attempt_mcp_recovery(void)
{
    recovery_attempts++;
    ESP_LOGW(TAG, "Attempting MCP23008 recovery (attempt #%lu)...", recovery_attempts);

    uint8_t gpio_val, intf_val, intcap_val;
    esp_err_t ret;

    // Try reading GPIO register to verify I2C is working
    ret = mcp23008_port_read(&mcp_dev, &gpio_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Recovery failed: Cannot read GPIO register (%s)", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Recovery: GPIO register = 0x%02x", gpio_val);

    // Try reading and clearing interrupt registers
    ret = mcp23008_port_get_interrupt_flags(&mcp_dev, &intf_val);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Recovery: INTF = 0x%02x", intf_val);

        ret = mcp23008_port_get_interrupt_capture(&mcp_dev, &intcap_val);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Recovery: INTCAP = 0x%02x (interrupt cleared)", intcap_val);
            recovery_successes++;
            ESP_LOGI(TAG, "MCP23008 recovery successful (%lu/%lu success rate)",
                     recovery_successes, recovery_attempts);
            return ESP_OK;
        }
    }

    ESP_LOGE(TAG, "Recovery failed: Cannot clear interrupts");
    return ret;
}

/**
 * @brief MCP23008 interrupt handler task
 *
 * Waits for notification from ISR, reads interrupt registers, and dispatches
 * to appropriate handlers based on which pins caused the interrupt.
 *
 * Includes I2C error detection, spurious interrupt filtering, and recovery logic.
 *
 * Latency: ~106-225µs typical (ISR trigger -> I2C read complete)
 *
 * @param pvParameters Unused
 */
static void mcp23008_handler_task(void *pvParameters)
{
    uint32_t notification_value;
    uint8_t captured_state = 0;
    uint8_t int_flags = 0;
    esp_err_t ret;

    ESP_LOGI(TAG, "MCP23008 handler task started");

    while (1) {
        // Wait for interrupt notification from ISR
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &notification_value,
                           portMAX_DELAY) == pdTRUE) {

            // DEBUG: Count handler task invocations
            handler_call_count++;
            ESP_LOGV(TAG, "Handler task notified! Count: %lu, ISR count: %lu",
                     handler_call_count, isr_call_count);

            // CRITICAL: Read INTF first, then INTCAP
            // Reading INTCAP clears INTF, so order matters!

            // Read INTF to see which pins caused the interrupt
            ret = mcp23008_port_get_interrupt_flags(&mcp_dev, &int_flags);
            if (ret != ESP_OK) {
                consecutive_i2c_errors++;
                total_i2c_errors++;

                ESP_LOGE(TAG, "I2C ERROR: Failed to read INTF: %s (consecutive: %lu, total: %lu)",
                         esp_err_to_name(ret), consecutive_i2c_errors, total_i2c_errors);

                // Throttle to prevent infinite loop
                if (consecutive_i2c_errors >= MAX_CONSECUTIVE_ERRORS) {
                    ESP_LOGE(TAG, "CRITICAL: %lu consecutive I2C errors - attempting recovery",
                             consecutive_i2c_errors);

                    // Disable interrupts during recovery
                    gpio_intr_disable(MCP23008_INT_GPIO);

                    vTaskDelay(pdMS_TO_TICKS(ERROR_BACKOFF_MS));

                    if (attempt_mcp_recovery() == ESP_OK) {
                        consecutive_i2c_errors = 0;
                        consecutive_spurious_interrupts = 0;
                        ESP_LOGI(TAG, "Recovery successful - resuming normal operation");

                        // Dump diagnostics after successful recovery
                        mcp23008_helper_print_diagnostics();
                    } else {
                        ESP_LOGE(TAG, "Recovery failed - disabling MCP23008 interrupts");

                        // Dump diagnostics before giving up
                        mcp23008_helper_print_diagnostics();

                        // Don't re-enable interrupt - system needs manual intervention
                        continue;
                    }
                }

                // Add small delay to prevent tight loop
                vTaskDelay(pdMS_TO_TICKS(10));
                gpio_intr_enable(MCP23008_INT_GPIO);
                continue;
            }

            // Read INTCAP to get captured pin states at interrupt time
            // This also clears the interrupt
            ret = mcp23008_port_get_interrupt_capture(&mcp_dev, &captured_state);
            if (ret != ESP_OK) {
                consecutive_i2c_errors++;
                total_i2c_errors++;

                ESP_LOGE(TAG, "I2C ERROR: Failed to read INTCAP: %s (consecutive: %lu, total: %lu)",
                         esp_err_to_name(ret), consecutive_i2c_errors, total_i2c_errors);

                // Same recovery logic as INTF failure
                if (consecutive_i2c_errors >= MAX_CONSECUTIVE_ERRORS) {
                    ESP_LOGE(TAG, "CRITICAL: %lu consecutive I2C errors - attempting recovery",
                             consecutive_i2c_errors);

                    gpio_intr_disable(MCP23008_INT_GPIO);
                    vTaskDelay(pdMS_TO_TICKS(ERROR_BACKOFF_MS));

                    if (attempt_mcp_recovery() == ESP_OK) {
                        consecutive_i2c_errors = 0;
                        consecutive_spurious_interrupts = 0;
                        ESP_LOGI(TAG, "Recovery successful - resuming normal operation");

                        // Dump diagnostics after successful recovery
                        mcp23008_helper_print_diagnostics();
                    } else {
                        ESP_LOGE(TAG, "Recovery failed - disabling MCP23008 interrupts");

                        // Dump diagnostics before giving up
                        mcp23008_helper_print_diagnostics();

                        continue;
                    }
                }

                vTaskDelay(pdMS_TO_TICKS(10));
                gpio_intr_enable(MCP23008_INT_GPIO);
                continue;
            }

            // I2C read successful - reset error counter
            consecutive_i2c_errors = 0;

            ESP_LOGV(TAG, "INTCAP: 0x%02x, INTF: 0x%02x", captured_state, int_flags);

            // Detect spurious interrupts (both registers zero)
            if (int_flags == 0x00 && captured_state == 0x00) {
                consecutive_spurious_interrupts++;
                total_spurious_interrupts++;

                ESP_LOGW(TAG, "SPURIOUS INTERRUPT detected (INTF=0x00, INTCAP=0x00) - "
                         "consecutive: %lu, total: %lu",
                         consecutive_spurious_interrupts, total_spurious_interrupts);

                if (consecutive_spurious_interrupts >= SPURIOUS_INT_THRESHOLD) {
                    ESP_LOGE(TAG, "CRITICAL: %lu consecutive spurious interrupts - attempting recovery",
                             consecutive_spurious_interrupts);

                    gpio_intr_disable(MCP23008_INT_GPIO);
                    vTaskDelay(pdMS_TO_TICKS(ERROR_BACKOFF_MS));

                    if (attempt_mcp_recovery() == ESP_OK) {
                        consecutive_i2c_errors = 0;
                        consecutive_spurious_interrupts = 0;
                        ESP_LOGI(TAG, "Recovery successful - resuming normal operation");

                        // Dump diagnostics after successful recovery
                        mcp23008_helper_print_diagnostics();
                    } else {
                        ESP_LOGE(TAG, "Recovery failed - disabling MCP23008 interrupts");

                        // Dump diagnostics before giving up
                        mcp23008_helper_print_diagnostics();

                        continue;
                    }
                }

                gpio_intr_enable(MCP23008_INT_GPIO);
                continue;
            }

            // Valid interrupt - reset spurious counter
            consecutive_spurious_interrupts = 0;

            // Dispatch to appropriate handlers based on interrupt flags
            // GP0-GP1: Encoder A/B
            if (int_flags & 0x03) {  // Bits 0-1
                ESP_LOGV(TAG, "Encoder A/B interrupt detected");
                handle_encoder_change(captured_state);
            }

            // GP2: Encoder button
            if (int_flags & (1 << MCP_PIN_ENCODER_BTN)) {
                ESP_LOGI(TAG, "Button interrupt detected");
                handle_button_change(captured_state);
            }

            // GP3: Rainfall pulse counter
            if (int_flags & (1 << MCP_PIN_RAINFALL)) {
                ESP_LOGI(TAG, "Rainfall pulse detected");
                handle_rainfall_pulse();
            }

            // GP4: Tank intake pulse counter
            if (int_flags & (1 << MCP_PIN_TANK_INTAKE)) {
                ESP_LOGI(TAG, "Tank pulse detected");
                handle_tank_pulse();
            }

            // Re-enable GPIO interrupt after processing
            // If MCP23008 has more pending interrupts (INT still LOW), ISR will fire again
            gpio_intr_enable(MCP23008_INT_GPIO);
        }
    }
}

// ########################## Public API Implementation ##########################

esp_err_t mcp23008_helper_init(void)
{
    esp_err_t ret;

    if (mcp_initialized) {
        ESP_LOGW(TAG, "MCP23008 helper already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing MCP23008 helper (I2C addr 0x%02x, INT GPIO %d)...",
             MCP23008_I2C_ADDR, MCP23008_INT_GPIO);

    // Initialize MCP23008 device descriptor (Bus A - always powered)
    ret = mcp23008_init_desc(&mcp_dev, MCP23008_I2C_ADDR, I2C_BUS_A_PORT,
                             I2C_BUS_A_SDA_PIN, I2C_BUS_A_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MCP23008 descriptor: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure pin directions
    // GP0-GP4: Inputs (1), GP5: Output (0), GP6: Input/high-Z (1), GP7: Reserved input (1)
    // GP6 starts as INPUT (high-Z) for power saving - will be driven OUTPUT HIGH only when display is active
    uint8_t pin_dir = 0b11011111;  // Bit=1 for input, bit=0 for output
    ret = mcp23008_port_set_mode(&mcp_dev, pin_dir);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set pin directions: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // Enable internal pull-ups for inputs (GP2-GP4, GP7)
    // GP0-GP1 (encoder A/B): External 10kΩ pull-ups on encoder board (internal disabled)
    // GP6 has no pull-up (will be driven HIGH when display active, high-Z when idle)
    uint8_t pullups = 0b10011000;  // GP2-GP4, GP7 enabled; GP0-GP1 disabled (external 10kΩ)
    ret = mcp23008_port_set_pullup(&mcp_dev, pullups);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable pull-ups: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // Configure interrupt output mode: Active-low open-drain
    ret = mcp23008_set_int_out_mode(&mcp_dev, MCP23008_OPEN_DRAIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set interrupt mode: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // Enable interrupt-on-change for all input pins (GP0-GP4)
    // TEMPORARY: Disable GP4 (tank) - causing interrupt storm (likely floating/noise)
    uint8_t int_enable = 0b00001111;  // GP0-GP3 only (disabled GP4)
    ret = mcp23008_port_set_interrupt(&mcp_dev, int_enable, MCP23008_INT_ANY_EDGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable interrupts: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    ESP_LOGW(TAG, "TEMPORARY: GP4 (tank intake) interrupt disabled due to noise");

    // Initialize hall enable output to LOW (disabled)
    ret = mcp23008_set_level(&mcp_dev, MCP_PIN_HALL_ENABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set hall enable initial state: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // GP6 (OLED reset) initialized as INPUT (high-Z) for power saving
    // Will be driven OUTPUT HIGH when display powers on, back to INPUT when display off
    ESP_LOGI(TAG, "GP6 (OLED reset) initialized as INPUT (high-Z) for power saving");

    // Configure ESP32 GPIO for INT pin
    gpio_config_t int_pin_config = {
        .pin_bit_mask = (1ULL << MCP23008_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,   // Internal pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_LOW_LEVEL,   // Level-triggered (prevents missed interrupts during bursts)
    };

    ret = gpio_config(&int_pin_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure INT GPIO: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // Create handler task (must exist before ISR is installed)
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        mcp23008_handler_task,
        "mcp23008_handler",
        MCP23008_HANDLER_TASK_STACK_SIZE,
        NULL,
        MCP23008_HANDLER_TASK_PRIORITY,
        &xMcp23008HandlerTask,
        MCP23008_HANDLER_TASK_CORE
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create handler task");
        ret = ESP_FAIL;
        goto cleanup;
    }

    // Install GPIO ISR service (may already be installed by other components)
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // Add ISR handler for INT pin
    ret = gpio_isr_handler_add(MCP23008_INT_GPIO, mcp23008_int_isr, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    mcp_initialized = true;
    ESP_LOGI(TAG, "MCP23008 helper initialized successfully");
    ESP_LOGI(TAG, "  Encoder: GP%d (A), GP%d (B), GP%d (BTN)",
             MCP_PIN_ENCODER_A, MCP_PIN_ENCODER_B, MCP_PIN_ENCODER_BTN);
    ESP_LOGI(TAG, "  Pulses: GP%d (rainfall), GP%d (tank)",
             MCP_PIN_RAINFALL, MCP_PIN_TANK_INTAKE);
    ESP_LOGI(TAG, "  Output: GP%d (hall enable)", MCP_PIN_HALL_ENABLE);

    // DEBUG: Verify GPIO ISR installation and initial state
    int gpio_level = gpio_get_level(MCP23008_INT_GPIO);
    ESP_LOGI(TAG, "DEBUG: INT pin (GPIO%d) initial level: %d", MCP23008_INT_GPIO, gpio_level);

    // DEBUG: Read MCP23008 registers to verify state
    uint8_t gpio_val, intf_val, intcap_val;
    esp_err_t dbg_ret;

    dbg_ret = mcp23008_port_read(&mcp_dev, &gpio_val);
    ESP_LOGI(TAG, "DEBUG: GPIO register: 0x%02x (ret=%s)", gpio_val, esp_err_to_name(dbg_ret));

    dbg_ret = mcp23008_port_get_interrupt_flags(&mcp_dev, &intf_val);
    ESP_LOGI(TAG, "DEBUG: INTF register: 0x%02x (ret=%s)", intf_val, esp_err_to_name(dbg_ret));

    dbg_ret = mcp23008_port_get_interrupt_capture(&mcp_dev, &intcap_val);
    ESP_LOGI(TAG, "DEBUG: INTCAP register: 0x%02x (ret=%s)", intcap_val, esp_err_to_name(dbg_ret));

    return ESP_OK;

cleanup:
    mcp23008_free_desc(&mcp_dev);
    if (xMcp23008HandlerTask != NULL) {
        vTaskDelete(xMcp23008HandlerTask);
        xMcp23008HandlerTask = NULL;
    }
    return ret;
}

int32_t mcp23008_helper_get_encoder_count(void)
{
    portENTER_CRITICAL(&mcp_spinlock);
    int32_t count = encoder_count;
    portEXIT_CRITICAL(&mcp_spinlock);

    return count;
}

void mcp23008_helper_reset_encoder_count(void)
{
    portENTER_CRITICAL(&mcp_spinlock);
    encoder_count = 0;
    portEXIT_CRITICAL(&mcp_spinlock);

    // Also reset accumulator (not protected by spinlock as it's only accessed from handler task)
    detent_accumulator = 0;
}

uint32_t mcp23008_helper_get_rainfall_pulses(void)
{
    portENTER_CRITICAL(&mcp_spinlock);
    uint32_t count = rainfall_pulse_count;
    portEXIT_CRITICAL(&mcp_spinlock);

    return count;
}

uint32_t mcp23008_helper_get_tank_intake_pulses(void)
{
    portENTER_CRITICAL(&mcp_spinlock);
    uint32_t count = tank_pulse_count;
    portEXIT_CRITICAL(&mcp_spinlock);

    return count;
}

void mcp23008_helper_reset_rainfall_pulses(void)
{
    portENTER_CRITICAL(&mcp_spinlock);
    rainfall_pulse_count = 0;
    portEXIT_CRITICAL(&mcp_spinlock);

    ESP_LOGI(TAG, "Rainfall pulse counter reset to 0");
}

void mcp23008_helper_reset_tank_intake_pulses(void)
{
    portENTER_CRITICAL(&mcp_spinlock);
    tank_pulse_count = 0;
    portEXIT_CRITICAL(&mcp_spinlock);

    ESP_LOGI(TAG, "Tank intake pulse counter reset to 0");
}

esp_err_t mcp23008_helper_set_hall_enable(bool enable)
{
    if (!mcp_initialized) {
        ESP_LOGE(TAG, "MCP23008 helper not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = mcp23008_set_level(&mcp_dev, MCP_PIN_HALL_ENABLE, enable ? 1 : 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set hall enable: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t mcp23008_helper_set_oled_reset(bool high)
{
    if (!mcp_initialized) {
        ESP_LOGE(TAG, "MCP23008 helper not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Configure pin as OUTPUT to drive the signal
    esp_err_t ret = mcp23008_set_mode(&mcp_dev, MCP_PIN_OLED_RESET, MCP23008_GPIO_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GP6 to OUTPUT: %s", esp_err_to_name(ret));
        return ret;
    }

    // Drive high or low
    ret = mcp23008_set_level(&mcp_dev, MCP_PIN_OLED_RESET, high ? 1 : 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set OLED reset level: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "GP6 (OLED reset) set to OUTPUT %s", high ? "HIGH" : "LOW");
    return ESP_OK;
}

esp_err_t mcp23008_helper_oled_reset_pulse(void)
{
    if (!mcp_initialized) {
        ESP_LOGE(TAG, "MCP23008 helper not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Configure GP6 as OUTPUT to drive reset signal
    esp_err_t ret = mcp23008_set_mode(&mcp_dev, MCP_PIN_OLED_RESET, MCP23008_GPIO_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GP6 to OUTPUT: %s", esp_err_to_name(ret));
        return ret;
    }

    // Pull reset low
    ret = mcp23008_set_level(&mcp_dev, MCP_PIN_OLED_RESET, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to pull OLED reset low: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait 10ms in reset (SH1106 requires >3µs, using 10ms for margin)
    vTaskDelay(pdMS_TO_TICKS(10));

    // Pull reset high
    ret = mcp23008_set_level(&mcp_dev, MCP_PIN_OLED_RESET, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to pull OLED reset high: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait another 10ms for display controller to stabilize after reset
    vTaskDelay(pdMS_TO_TICKS(10));

    // Return GP6 to INPUT (high-Z) to save power - reset is complete
    ret = mcp23008_set_mode(&mcp_dev, MCP_PIN_OLED_RESET, MCP23008_GPIO_INPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GP6 back to INPUT: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "OLED reset pulse completed (GP6 returned to high-Z)");
    return ESP_OK;
}

esp_err_t mcp23008_helper_oled_reset_release(void)
{
    if (!mcp_initialized) {
        ESP_LOGE(TAG, "MCP23008 helper not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Set GP6 back to INPUT (high-Z) for power saving
    esp_err_t ret = mcp23008_set_mode(&mcp_dev, MCP_PIN_OLED_RESET, MCP23008_GPIO_INPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GP6 to INPUT: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "GP6 (OLED reset) set to INPUT (high-Z) - power saving mode");
    return ESP_OK;
}

bool mcp23008_helper_get_button_state(void)
{
    portENTER_CRITICAL(&mcp_spinlock);
    bool state = button_pressed;
    portEXIT_CRITICAL(&mcp_spinlock);

    return state;
}

esp_err_t mcp23008_helper_register_button_callback(TaskHandle_t task_handle)
{
    portENTER_CRITICAL(&mcp_spinlock);
    button_notify_task = task_handle;
    portEXIT_CRITICAL(&mcp_spinlock);

    if (task_handle != NULL) {
        ESP_LOGI(TAG, "Button callback registered for task 0x%p", task_handle);
    } else {
        ESP_LOGI(TAG, "Button callback unregistered");
    }

    return ESP_OK;
}

esp_err_t mcp23008_helper_get_diagnostics(mcp23008_diagnostics_t *diag)
{
    if (diag == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Snapshot diagnostic counters (no need for critical section - reads are atomic on 32-bit aligned data)
    diag->total_interrupts = handler_call_count;
    diag->total_i2c_errors = total_i2c_errors;
    diag->total_spurious_interrupts = total_spurious_interrupts;
    diag->consecutive_i2c_errors = consecutive_i2c_errors;
    diag->consecutive_spurious = consecutive_spurious_interrupts;
    diag->recovery_attempts = recovery_attempts;
    diag->recovery_successes = recovery_successes;

    return ESP_OK;
}

void mcp23008_helper_print_diagnostics(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "    MCP23008 Diagnostics Report");
    ESP_LOGI(TAG, "========================================");

    // Interrupt statistics
    ESP_LOGI(TAG, "Interrupt Statistics:");
    ESP_LOGI(TAG, "  Total interrupts:    %lu", handler_call_count);
    ESP_LOGI(TAG, "  ISR calls:           %lu", isr_call_count);
    if (handler_call_count != isr_call_count) {
        ESP_LOGW(TAG, "  WARNING: ISR/Handler mismatch (%ld pending)",
                 (int32_t)(isr_call_count - handler_call_count));
    }

    // Error statistics
    ESP_LOGI(TAG, "Error Statistics:");
    ESP_LOGI(TAG, "  Total I2C errors:    %lu", total_i2c_errors);
    ESP_LOGI(TAG, "  Consecutive I2C:     %lu (threshold: %d)", consecutive_i2c_errors, MAX_CONSECUTIVE_ERRORS);
    ESP_LOGI(TAG, "  Total spurious:      %lu", total_spurious_interrupts);
    ESP_LOGI(TAG, "  Consecutive spurious:%lu (threshold: %d)", consecutive_spurious_interrupts, SPURIOUS_INT_THRESHOLD);

    if (total_i2c_errors > 0 || total_spurious_interrupts > 0) {
        float error_rate = (float)(total_i2c_errors + total_spurious_interrupts) / handler_call_count * 100.0f;
        ESP_LOGW(TAG, "  Error rate:          %.2f%%", error_rate);
    }

    // Recovery statistics
    if (recovery_attempts > 0) {
        ESP_LOGI(TAG, "Recovery Statistics:");
        ESP_LOGI(TAG, "  Attempts:            %lu", recovery_attempts);
        ESP_LOGI(TAG, "  Successes:           %lu", recovery_successes);
        float success_rate = (float)recovery_successes / recovery_attempts * 100.0f;
        if (success_rate < 100.0f) {
            ESP_LOGW(TAG, "  Success rate:        %.1f%%", success_rate);
        } else {
            ESP_LOGI(TAG, "  Success rate:        %.1f%%", success_rate);
        }
    } else {
        ESP_LOGI(TAG, "Recovery: No attempts (healthy)");
    }

    // Counter values
    ESP_LOGI(TAG, "Counter Values:");
    portENTER_CRITICAL(&mcp_spinlock);
    int32_t enc_count = encoder_count;
    uint32_t rain_count = rainfall_pulse_count;
    uint32_t tank_count = tank_pulse_count;
    bool btn_state = button_pressed;
    portEXIT_CRITICAL(&mcp_spinlock);

    ESP_LOGI(TAG, "  Encoder count:       %ld", enc_count);
    ESP_LOGI(TAG, "  Rainfall pulses:     %lu", rain_count);
    ESP_LOGI(TAG, "  Tank pulses:         %lu", tank_count);
    ESP_LOGI(TAG, "  Button state:        %s", btn_state ? "PRESSED" : "RELEASED");

    // Hardware state
    ESP_LOGI(TAG, "Hardware State:");
    int int_pin_level = gpio_get_level(MCP23008_INT_GPIO);
    ESP_LOGI(TAG, "  INT pin (GPIO%d):    %s", MCP23008_INT_GPIO,
             int_pin_level ? "HIGH (idle)" : "LOW (active)");

    if (mcp_initialized) {
        uint8_t gpio_val = 0, intf_val = 0;
        esp_err_t ret;

        ret = mcp23008_port_read(&mcp_dev, &gpio_val);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  GPIO register:       0x%02x", gpio_val);
        } else {
            ESP_LOGE(TAG, "  GPIO register:       ERROR (%s)", esp_err_to_name(ret));
        }

        ret = mcp23008_port_get_interrupt_flags(&mcp_dev, &intf_val);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  INTF register:       0x%02x %s", intf_val,
                     intf_val ? "(pending interrupts)" : "(clear)");
        } else {
            ESP_LOGE(TAG, "  INTF register:       ERROR (%s)", esp_err_to_name(ret));
        }
    }

    // Health assessment
    ESP_LOGI(TAG, "========================================");
    if (consecutive_i2c_errors >= MAX_CONSECUTIVE_ERRORS) {
        ESP_LOGE(TAG, "  HEALTH: CRITICAL - I2C failure");
    } else if (consecutive_spurious_interrupts >= SPURIOUS_INT_THRESHOLD) {
        ESP_LOGE(TAG, "  HEALTH: CRITICAL - Spurious interrupts");
    } else if (total_i2c_errors > 0 || total_spurious_interrupts > 0) {
        ESP_LOGW(TAG, "  HEALTH: DEGRADED - Errors detected but recovered");
    } else {
        ESP_LOGI(TAG, "  HEALTH: OK - No errors detected");
    }
    ESP_LOGI(TAG, "========================================");
}
