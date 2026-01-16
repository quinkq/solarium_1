/**
 * @file mcp23008_helper.h
 * @brief MCP23008 I2C GPIO Expander Integration Helper
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Integration layer for MCP23008 I2C GPIO expander handling:
 * - Encoder quadrature decoding (GP0, GP1, GP2)
 * - Pulse counting for rainfall and tank intake sensors (GP3, GP4)
 * - Hall array power enable control (GP5)
 * - Interrupt-driven event handling via ESP32 GPIO
 *
 * This helper abstracts the MCP23008 hardware from components (HMI, TEMPESTA, FLUCTUS)
 * and provides a clean API for encoder reading, pulse counting, and output control.
 *
 * Hardware Configuration:
 * - MCP23008 I2C Address: 0x20 (A2=A1=A0=0)
 * - INT pin connected to ESP32-S3 GPIO8
 * - I2C Bus: 400kHz (shared bus with other sensors)
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#ifndef MCP23008_HELPER_H
#define MCP23008_HELPER_H

#include "mcp23008.h"
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ########################## Configuration Constants ##########################

// MCP23008 Pin Mapping (as per MCP23008_MIGRATION_PLAN.md Section 3.1)
#define MCP_PIN_ENCODER_A       0   // GP0: Encoder A input
#define MCP_PIN_ENCODER_B       1   // GP1: Encoder B input
#define MCP_PIN_ENCODER_BTN     2   // GP2: Encoder button input
#define MCP_PIN_RAINFALL        3   // GP3: Rainfall pulse counter input
#define MCP_PIN_TANK_INTAKE     4   // GP4: Tank intake pulse counter input
#define MCP_PIN_HALL_ENABLE     5   // GP5: Hall array enable output
#define MCP_PIN_OLED_RESET      6   // GP6: OLED display reset output
// GP7: Reserved for future expansion

// MCP23008 Hardware Configuration
#define MCP23008_I2C_ADDR       0x20        // I2C address (A2=A1=A0=0)
#define MCP23008_INT_GPIO       GPIO_NUM_18  // ESP32 GPIO for INT pin

// Handler task configuration
#define MCP23008_HANDLER_TASK_STACK_SIZE    3072
#define MCP23008_HANDLER_TASK_PRIORITY      5    // Med-5 (match other interrupt handlers)
#define MCP23008_HANDLER_TASK_CORE          1    // Run on core 1

// ########################## Public Function Declarations ##########################

/**
 * @brief Initialize MCP23008 helper system
 *
 * Initializes the MCP23008 device, configures pins, sets up interrupt handling,
 * and starts the handler task. Must be called during system initialization
 * before any components that use encoder/pulse counters/hall enable.
 *
 * Pin Configuration:
 * - GP0-GP4: Inputs with internal pull-ups, interrupt-on-change enabled
 * - GP5: Output for hall array enable
 * - GP6: Output for OLED display reset
 * - GP7: Reserved (configured as input)
 *
 * Uses MCP23008_INT_GPIO (GPIO8) as the ESP32 interrupt pin.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_* on initialization failure
 */
esp_err_t mcp23008_helper_init(void);

/**
 * @brief Get current encoder count
 *
 * Returns the accumulated encoder count from software quadrature decoding.
 * This is a replacement for PCNT hardware quadrature decoding used by HMI.
 * Thread-safe via spinlock.
 *
 * @return int32_t Current encoder count (positive = clockwise, negative = counter-clockwise)
 */
int32_t mcp23008_helper_get_encoder_count(void);

/**
 * @brief Reset encoder count to zero
 *
 * Clears the accumulated encoder count. Thread-safe via spinlock.
 */
void mcp23008_helper_reset_encoder_count(void);

/**
 * @brief Get rainfall pulse count
 *
 * Returns the accumulated rainfall pulse count from the tipbucket sensor.
 * Counter is monotonically increasing - use explicit reset functions for clearing.
 * Thread-safe via spinlock.
 *
 * @return uint32_t Number of pulses counted since last reset
 */
uint32_t mcp23008_helper_get_rainfall_pulses(void);

/**
 * @brief Get tank intake pulse count
 *
 * Returns the accumulated tank intake pulse count from the tipbucket sensor.
 * Counter is monotonically increasing - use explicit reset functions for clearing.
 * Thread-safe via spinlock.
 *
 * @return uint32_t Number of pulses counted since last reset
 */
uint32_t mcp23008_helper_get_tank_intake_pulses(void);

/**
 * @brief Reset rainfall pulse counter to zero
 *
 * Clears the accumulated rainfall pulse count. Used by TEMPESTA for
 * weekly hardware counter resets to prevent overflow.
 * Thread-safe via spinlock.
 */
void mcp23008_helper_reset_rainfall_pulses(void);

/**
 * @brief Reset tank intake pulse counter to zero
 *
 * Clears the accumulated tank intake pulse count. Used by TEMPESTA for
 * weekly hardware counter resets to prevent overflow.
 * Thread-safe via spinlock.
 */
void mcp23008_helper_reset_tank_intake_pulses(void);

/**
 * @brief Control hall array power enable
 *
 * Sets the state of the hall array enable output (GP5).
 * Used by FLUCTUS to gate power to the hall sensor array during wind direction reads.
 *
 * @param enable true = enable hall array power (GP5 high), false = disable (GP5 low)
 * @return ESP_OK on success
 * @return ESP_ERR_* on I2C communication failure
 */
esp_err_t mcp23008_helper_set_hall_enable(bool enable);

/**
 * @brief Get button state
 *
 * Returns the current state of the encoder button (GP2).
 * Thread-safe - returns cached state from last interrupt.
 *
 * @return true if button is pressed (low), false if released (high)
 */
bool mcp23008_helper_get_button_state(void);

/**
 * @brief Register button event callback
 *
 * Registers a FreeRTOS task to be notified when the encoder button is pressed.
 * The registered task will receive a notification via xTaskNotify on button press.
 * This allows event-driven button handling without polling.
 *
 * @param task_handle Handle of task to notify on button press (or NULL to unregister)
 * @return ESP_OK on success
 */
esp_err_t mcp23008_helper_register_button_callback(TaskHandle_t task_handle);


/**
 * @brief Control OLED display reset line
 *
 * Sets the state of the OLED reset output (GP6).
 * Used by HMI to perform hardware reset of SH1106 display during initialization.
 *
 * @param high true = reset line high (normal operation), false = reset line low (in reset)
 * @return ESP_OK on success
 * @return ESP_ERR_* on I2C communication failure
 */
esp_err_t mcp23008_helper_set_oled_reset(bool high);

/**
 * @brief Perform OLED display reset pulse (self-contained, returns to high-Z)
 *
 * Complete reset sequence:
 * 1. Configure GP6 as OUTPUT
 * 2. Drive LOW for 10ms (reset pulse)
 * 3. Drive HIGH for 10ms (stabilization)
 * 4. Return GP6 to INPUT (high-Z) for power saving
 *
 * Self-contained function for one-time initialization at boot.
 * For display power-on during normal operation, use set_oled_reset(true) instead.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_* on I2C communication failure
 */
esp_err_t mcp23008_helper_oled_reset_pulse(void);

/**
 * @brief Release OLED reset pin to high-Z (power saving)
 *
 * Sets GP6 to INPUT mode (high-impedance) for power saving when display is off.
 * Call this when display powers down to eliminate the 3-4mA current draw.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_* on I2C communication failure
 */
esp_err_t mcp23008_helper_oled_reset_release(void);


#ifdef __cplusplus
}
#endif

#endif /* MCP23008_HELPER_H */
