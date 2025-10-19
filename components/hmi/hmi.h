#ifndef HMI_H
#define HMI_H

#include <stdbool.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "driver/gpio.h"

// ########################## HMI Configuration ##########################

// EC11 Rotary Encoder GPIO Configuration
#define HMI_ENCODER_A_GPIO      GPIO_NUM_16  // Quadrature signal A
#define HMI_ENCODER_B_GPIO      GPIO_NUM_17  // Quadrature signal B
#define HMI_ENCODER_BTN_GPIO    GPIO_NUM_18  // Push button

// SH1106 OLED Display SPI Configuration
#define HMI_DISPLAY_CS_GPIO     GPIO_NUM_13  // Chip select
#define HMI_DISPLAY_DC_GPIO     GPIO_NUM_14  // Data/Command
#define HMI_DISPLAY_SPI_HOST    SPI2_HOST    // Shared with ABP sensor

// Display Specifications
#define HMI_DISPLAY_WIDTH       128          // Pixels
#define HMI_DISPLAY_HEIGHT      64           // Pixels
#define HMI_DISPLAY_ROTATION    0            // 0, 90, 180, or 270 degrees

// Display Timing Configuration
#define HMI_DISPLAY_TIMEOUT_MS  (60 * 1000)  // 1 minute timeout
#define HMI_REFRESH_RATE_MS     100          // 10Hz display refresh rate

// PCNT Configuration for Encoder
#define HMI_PCNT_HIGH_LIMIT     100          // Upper count limit
#define HMI_PCNT_LOW_LIMIT      -100         // Lower count limit

// Menu Configuration
#define HMI_MENU_MAX_ITEMS      10           // Maximum items per menu page
#define HMI_MENU_TITLE_LENGTH   20           // Maximum title length
#define HMI_MENU_ITEM_LENGTH    32           // Maximum menu item text length

// ########################## Data Structures ##########################

/**
 * @brief Menu navigation states
 */
typedef enum {
    HMI_MENU_MAIN = 0,           // Main menu (component selection)
    HMI_MENU_FLUCTUS,            // FLUCTUS power management submenu
    HMI_MENU_FLUCTUS_POWER,      // Power bus status details
    HMI_MENU_FLUCTUS_BATTERY,    // Battery monitoring details
    HMI_MENU_FLUCTUS_SOLAR,      // Solar tracking details
    HMI_MENU_TEMPESTA,           // TEMPESTA weather station submenu
    HMI_MENU_TEMPESTA_ENV,       // Environmental sensors details
    HMI_MENU_TEMPESTA_WIND,      // Wind sensor details
    HMI_MENU_IMPLUVIUM,          // IMPLUVIUM irrigation submenu
    HMI_MENU_IMPLUVIUM_ZONES,    // Zone status details
    HMI_MENU_IMPLUVIUM_LEARNING, // Learning algorithm details
    HMI_MENU_STELLARIA,          // STELLARIA lighting submenu
    HMI_MENU_STELLARIA_CONTROL   // Lighting control details
} hmi_menu_state_t;

/**
 * @brief Encoder event types
 */
typedef enum {
    HMI_ENCODER_CW = 0,          // Clockwise rotation
    HMI_ENCODER_CCW,             // Counter-clockwise rotation
    HMI_ENCODER_BUTTON_PRESS,    // Button pressed
    HMI_ENCODER_BUTTON_RELEASE   // Button released
} hmi_encoder_event_t;

/**
 * @brief HMI system status
 */
typedef struct {
    bool initialized;            // Component initialization status
    bool display_active;         // Display powered on
    hmi_menu_state_t current_menu; // Current menu state
    uint8_t selected_item;       // Currently selected menu item
    time_t last_activity_time;   // Last user interaction timestamp
    int16_t encoder_count;       // Current encoder count
} hmi_status_t;

// ########################## Public API Functions ##########################

/**
 * @brief Initialize HMI system
 *
 * Initializes the rotary encoder (PCNT), push button (GPIO interrupt),
 * and SH1106 OLED display (SPI + esp_lcd).
 *
 * @return ESP_OK on success, ESP_FAIL on initialization failure
 */
esp_err_t hmi_init(void);

/**
 * @brief Get current HMI status (thread-safe)
 * @param[out] status Pointer to hmi_status_t structure to fill
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t hmi_get_status(hmi_status_t *status);

/**
 * @brief Manually wake display (resets timeout)
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t hmi_wake_display(void);

#endif // HMI_H
