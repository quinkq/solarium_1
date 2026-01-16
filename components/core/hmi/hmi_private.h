/**
 * @file hmi_private.h
 * @brief HMI Component - Shared Internal Declarations
 *
 * This header is for internal use within the HMI component only.
 * It contains shared global state and function prototypes used by:
 * - hmi_display.c (hardware abstraction)
 * - hmi_navigation.c (state machine)
 * - hmi_render_*.c (rendering modules)
 */

#ifndef HMI_PRIVATE_H
#define HMI_PRIVATE_H

#include "hmi.h"
#include "fluctus.h"
#include "impluvium.h"
#include "tempesta.h"
#include "stellaria.h"
#include "telemetry.h"
#include "wifi_helper.h"
#include "interval_config.h"


// ########################## Scrolling and Pagination Constants ##########################

/**
 * @brief Maximum visible menu items on screen (including "< Back")
 * Calculated: 7 items × 7px spacing = 49px, fits in 54px content area
 */
#define HMI_MAX_VISIBLE_ITEMS 7

/**
 * @brief Y-coordinate where menu content starts (after title + divider)
 */
#define HMI_MENU_START_Y 14

/**
 * @brief Standard vertical spacing between menu items (pixels)
 */
#define HMI_MENU_ITEM_SPACING 7

// ########################## Confirmation Action Types ##########################

/**
 * @brief Confirmation dialog action types
 */
typedef enum {
    CONFIRM_NONE = 0,
    CONFIRM_RESET_ZONE_LEARNING,
    CONFIRM_RESET_ALL_LEARNING,
    CONFIRM_MANUAL_WATER,
    CONFIRM_TEMPESTA_RESET_DAILY,
    CONFIRM_TEMPESTA_RESET_WEEKLY,
    CONFIRM_TEMPESTA_RESET_RAIN_TOTAL,
    CONFIRM_TEMPESTA_RESET_TANK_TOTAL,
    CONFIRM_TEMPESTA_RESET_ALL,
    CONFIRM_SYSTEM_FLUSH_RESET
} confirm_action_t;

// ########################## Shared Global State ##########################

// HMI status structure (defined in hmi.c)
extern hmi_status_t hmi_status;

// Mutex for thread-safe operations (defined in hmi.c)
extern SemaphoreHandle_t xHmiMutex;

// ########################## Editing State Variables ##########################

// Stellaria control page editing state
extern bool stellaria_intensity_editing;
extern uint8_t stellaria_intensity_percent;

// IMPLUVIUM zone editing state
extern bool zone_editing;
extern uint8_t editing_zone_id;
extern bool editing_zone_enabled;
extern float editing_zone_target;
extern float editing_zone_deadband;

// IMPLUVIUM manual water input state
extern bool manual_water_input;
extern uint8_t manual_water_zone;
extern uint16_t manual_water_duration;

// Interval editing state (for FLUCTUS/TEMPESTA/IMPLUVIUM/SYSTEM interval pages)
extern bool interval_editing;
extern uint32_t editing_interval_value;

// Confirmation dialog state
extern confirm_action_t confirmation_action;
extern uint8_t confirmation_param;
extern hmi_menu_state_t confirmation_return_menu;

// Servo debug state (for FLUCTUS manual servo control)
extern bool servo_debug_active;           // True when in servo control mode
extern time_t servo_debug_start_time;     // Timestamp when control mode started (for timeout)
extern bool servo_debug_bus_requested;    // True if we have 6.6V bus power
extern uint32_t servo_debug_current_duty; // Current servo duty cycle (tracked locally during control)

// TEMPESTA diagnostic state (for wind sensor debugging)
extern bool tempesta_diag_active;         // True when in diagnostic mode
extern time_t tempesta_diag_start_time;   // Timestamp when diagnostic mode started (for 90s timeout)
extern bool tempesta_diag_bus_requested;  // True if we have 3.3V bus power

// ########################## Display Functions (hmi_display.c) ##########################

/**
 * @brief Initialize SH1106 display hardware
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t hmi_display_init(void);

/**
 * @brief Power on the display
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t hmi_display_power_on(void);

/**
 * @brief Power off the display
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t hmi_display_power_off(void);

/**
 * @brief Clear entire framebuffer
 */
void hmi_fb_clear(void);

/**
 * @brief Set a single pixel in the framebuffer
 * @param x X coordinate (0-127)
 * @param y Y coordinate (0-63)
 * @param on Pixel state (true = on, false = off)
 */
void hmi_fb_set_pixel(int x, int y, bool on);

/**
 * @brief Draw a single character using custom font
 * @param x X coordinate (0-127)
 * @param y Y coordinate (0-63)
 * @param c ASCII character to draw
 * @param inverted Invert colors (true = black text on white background)
 */
void hmi_fb_draw_char(int x, int y, char c, bool inverted);

/**
 * @brief Draw a text string using custom font
 * @param x X coordinate (0-127)
 * @param y Y coordinate (0-63)
 * @param str Null-terminated string to draw
 * @param inverted Invert colors (true = black text on white background)
 */
void hmi_fb_draw_string(int x, int y, const char *str, bool inverted);

/**
 * @brief Draw a horizontal line
 * @param x X start coordinate (0-127)
 * @param y Y coordinate (0-63)
 * @param width Line width in pixels
 */
void hmi_fb_draw_hline(int x, int y, int width);

/**
 * @brief Flush framebuffer to display via SPI
 */
void hmi_fb_flush(void);

/**
 * @brief Draw scroll indicators (▲/▼) for menus with >7 items
 * @param total_items Total number of items in current menu
 */
void hmi_draw_scroll_indicators(uint8_t total_items);

/**
 * @brief Draw navigation symbol in top-left corner (← for back)
 */
void hmi_draw_back_symbol(void);

/**
 * @brief Draw navigation symbol in top-right corner (↔ for pagination)
 */
void hmi_draw_pagination_symbol(void);

// ########################## Navigation Functions (hmi_navigation.c) ##########################

/**
 * @brief Handle button press event (dispatch to appropriate action)
 */
void hmi_handle_button_press(void);

/**
 * @brief Handle encoder rotation event
 * @param delta Rotation delta: positive = CW, negative = CCW
 */
void hmi_handle_encoder_change(int delta);

/**
 * @brief Get the number of selectable items for a menu state
 * @param menu Menu state to query
 * @return Number of items in the menu
 */
uint8_t hmi_get_menu_item_count(hmi_menu_state_t menu);

/**
 * @brief Check if current menu is a realtime page (4Hz refresh required)
 * @return true if page needs 4Hz refresh, false for 1Hz
 */
bool hmi_is_realtime_page(void);

/**
 * @brief Main rendering dispatcher (calls component-specific renderers)
 * Called from hmi_display_task in hmi.c
 */
  void hmi_render_menu(void);

/**
 * @brief Update last activity timestamp for timeout tracking
 * Called by navigation functions on user interaction
 */
  void hmi_update_activity(void);

// ########################## Rendering Functions (hmi_render_*.c) ##########################

/**
 * @brief FLUCTUS rendering dispatcher
 * Renders all FLUCTUS-related pages (power, solar tracking, etc.)
 */
void hmi_render_fluctus_pages(void);

/**
 * @brief TEMPESTA rendering dispatcher
 * Renders all TEMPESTA-related pages (weather sensors)
 */
void hmi_render_tempesta_pages(void);

/**
 * @brief IMPLUVIUM rendering dispatcher
 * Renders all IMPLUVIUM-related pages (irrigation zones, learning, etc.)
 */
void hmi_render_impluvium_pages(void);

/**
 * @brief SYSTEM/STELLARIA rendering dispatcher
 * Renders system info, Stellaria lighting, and main menu/confirmation dialog
 */
void hmi_render_system_pages(void);

#endif // HMI_PRIVATE_H
