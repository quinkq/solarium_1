#ifndef HMI_H
#define HMI_H

#include <stdbool.h>
#include <time.h>
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
#define HMI_DISPLAY_TIMEOUT_NORMAL_MS   (30 * 1000)  // 30 second timeout for normal menus
#define HMI_DISPLAY_TIMEOUT_REALTIME_MS (60 * 1000)  // 60 second timeout for realtime pages
#define HMI_REFRESH_RATE_NORMAL_MS      1000         // 1Hz for static menus (event-driven)
#define HMI_REFRESH_RATE_REALTIME_MS    250          // 4Hz for realtime pages

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
    HMI_MENU_FLUCTUS_OVERVIEW,   // System overview (state, temp, SOC) [1Hz]
    HMI_MENU_FLUCTUS_ENERGY,     // Energy statistics (hourly/daily) [1Hz]
    HMI_MENU_FLUCTUS_LIVE_POWER, // Live power monitoring (inst + avg) [4Hz REALTIME]
    HMI_MENU_FLUCTUS_BUSES,      // Power bus status [1Hz]
    HMI_MENU_FLUCTUS_TRACKING,   // Solar tracking position [1Hz]
    HMI_MENU_FLUCTUS_SOLAR_DEBUG,// Solar tracking debug (errors, sensors) [4Hz REALTIME]
    HMI_MENU_FLUCTUS_CONTROLS,   // Solar tracking & safety controls [1Hz]
    HMI_MENU_FLUCTUS_INTERVALS,  // Interval configuration (power day/night, solar correction) [1Hz]
    HMI_MENU_TEMPESTA,           // TEMPESTA weather station submenu
    HMI_MENU_TEMPESTA_ENV,       // Environmental sensors details
    HMI_MENU_TEMPESTA_WIND,      // Wind sensor details
    HMI_MENU_TEMPESTA_RAIN,      // Rain gauge details
    HMI_MENU_TEMPESTA_TANK,      // Tank intake details
    HMI_MENU_TEMPESTA_AIR,       // Air quality details
    HMI_MENU_TEMPESTA_CONTROLS,  // System controls (enable/disable, force collection, resets)
    HMI_MENU_TEMPESTA_INTERVALS, // Interval configuration (normal, power save) [1Hz]
    HMI_MENU_IMPLUVIUM,          // IMPLUVIUM irrigation submenu
    HMI_MENU_IMPLUVIUM_OVERVIEW, // System overview (state, water level, totals) [1Hz]
    HMI_MENU_IMPLUVIUM_STATISTICS, // Hourly/daily water usage statistics [1Hz]
    HMI_MENU_IMPLUVIUM_ZONES,    // Zones submenu selector
    HMI_MENU_IMPLUVIUM_ZONES_ALL, // All zones summary (default)
    HMI_MENU_IMPLUVIUM_ZONE_1,   // Zone 1 detail
    HMI_MENU_IMPLUVIUM_ZONE_2,   // Zone 2 detail
    HMI_MENU_IMPLUVIUM_ZONE_3,   // Zone 3 detail
    HMI_MENU_IMPLUVIUM_ZONE_4,   // Zone 4 detail
    HMI_MENU_IMPLUVIUM_ZONE_5,   // Zone 5 detail
    HMI_MENU_IMPLUVIUM_LEARNING, // Learning submenu selector
    HMI_MENU_IMPLUVIUM_LEARNING_ALL, // Learning summary (all zones, default)
    HMI_MENU_IMPLUVIUM_LEARNING_1, // Zone 1 learning detail
    HMI_MENU_IMPLUVIUM_LEARNING_2, // Zone 2 learning detail
    HMI_MENU_IMPLUVIUM_LEARNING_3, // Zone 3 learning detail
    HMI_MENU_IMPLUVIUM_LEARNING_4, // Zone 4 learning detail
    HMI_MENU_IMPLUVIUM_LEARNING_5, // Zone 5 learning detail
    HMI_MENU_IMPLUVIUM_MONITOR,  // System monitor [4Hz REALTIME]
    HMI_MENU_IMPLUVIUM_CONTROLS, // System controls (enable/disable, force check, resets) [1Hz]
    HMI_MENU_IMPLUVIUM_ZONE_CONFIG, // Zone configuration list [1Hz]
    HMI_MENU_IMPLUVIUM_ZONE_EDIT,   // Zone editing (enable/target/deadband/manual water)
    HMI_MENU_IMPLUVIUM_MANUAL_WATER, // Manual water time input
    HMI_MENU_IMPLUVIUM_INTERVALS, // Interval configuration (optimal/cool/power save/night min) [1Hz]
    HMI_MENU_STELLARIA,          // STELLARIA lighting submenu
    HMI_MENU_STELLARIA_STATUS,   // Status details
    HMI_MENU_STELLARIA_CONTROL,  // Manual control details
    HMI_MENU_STELLARIA_AUTO,     // Auto mode details
    HMI_MENU_SYSTEM,             // System menu (WiFi, controls)
    HMI_MENU_SYSTEM_INFO,        // System info (WiFi RSSI, uptime, version)
    HMI_MENU_SYSTEM_CONTROLS,    // System controls (flush & reset, WiFi reconnect)
    HMI_MENU_SYSTEM_INTERVALS,   // Global interval presets (Aggressive/Balanced/Conservative) [1Hz]
    HMI_MENU_CONFIRM             // Shared confirmation dialog (yes/no)
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
    bool blink_state;            // Blinking indicator state for [LIVE] pages
    uint32_t blink_counter;      // Counter for blink timing (250ms toggles = 500ms cycle)
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
