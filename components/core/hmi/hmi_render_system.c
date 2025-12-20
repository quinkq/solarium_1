/**
 * @file hmi_render_system.c
 * @brief HMI Rendering Module - SYSTEM, STELLARIA, and Shared Pages
 *
 * Renders all system-level and shared display pages:
 * - Main Menu: Top-level component selection
 * - STELLARIA Menu + 3 pages: Status, Manual Control, Auto Mode
 * - SYSTEM Menu + 3 pages: System Info, Controls, Intervals (preset selection)
 * - Confirmation Dialog: Shared by all components for destructive actions
 */

#include "hmi_private.h"

#include <stdio.h>
#include <time.h>
#include "esp_timer.h"

// ########################## Main Menu ##########################

/**
 * @brief Render main menu (top-level component selection)
 */
static void hmi_render_main_menu(void)
{
    // Title
    hmi_fb_draw_string(20, 2, "SOLARIUM v1.0", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items
    const char *items[] = {
        "> FLUCTUS",
        "  TEMPESTA",
        "  IMPLUVIUM",
        "  STELLARIA",
        "  SYSTEM"
    };

    int y = 14;
    for (int i = 0; i < 5; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            // Draw selection indicator
            hmi_fb_draw_string(2, y, ">", false);
        }
        hmi_fb_draw_string(8, y, items[i] + 2, selected);  // Skip the "> " prefix
        y += 10;
    }
}

// ########################## STELLARIA Pages ##########################

/**
 * @brief Render STELLARIA submenu
 */
static void hmi_render_stellaria_menu(void)
{
    // Get data from TELEMETRY cache
    stellaria_snapshot_t data;
    telemetry_get_stellaria_data(&data);

    // Title
    hmi_fb_draw_string(26, 2, "STELLARIA", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items
    const char *items[] = {
        "< Back",
        "Status",
        "Manual Control",
        "Auto Mode"
    };

    int y = 14;
    for (int i = 0; i < 4; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }
        hmi_fb_draw_string(8, y, items[i], selected);
        y += 10;
    }

    // Show quick status at bottom
    char buf[32];
    snprintf(buf, sizeof(buf), "Output %s Int:%d%%",
             data.driver_output_enabled ? "ON " : "OFF",
             (data.current_intensity * 100) / 1023);
    hmi_fb_draw_string(2, 56, buf, false);
}

/**
 * @brief Render STELLARIA Status detail page
 */
static void hmi_render_stellaria_status_page(void)
{
    stellaria_snapshot_t data;
    telemetry_get_stellaria_data(&data);

    hmi_fb_draw_string(30, 2, "Status", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    const char *state_str = "DISABLED";
    switch (data.state) {
        case STELLARIA_STATE_DISABLED: state_str = "DISABLED"; break;
        case STELLARIA_STATE_ENABLED: state_str = "ENABLED"; break;
        case STELLARIA_STATE_SHUTDOWN: state_str = "SHUTDOWN"; break;
    }

    snprintf(buf, sizeof(buf), "State: %s", state_str);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Show auto mode flag if active
    if (data.auto_mode_active) {
        snprintf(buf, sizeof(buf), "Mode: AUTO");
        hmi_fb_draw_string(2, y, buf, false);
        y += 10;
    }

    // Show power save flag if active
    if (data.power_save_mode) {
        snprintf(buf, sizeof(buf), "Power: SAVE");
        hmi_fb_draw_string(2, y, buf, false);
        y += 10;
    }

    snprintf(buf, sizeof(buf), "Current: %d%%",
             (data.current_intensity * 100) / 1023);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Target:  %d%%",
             (data.target_intensity * 100) / 1023);
    hmi_fb_draw_string(2, y, buf, false);

    hmi_fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render STELLARIA Control detail page
 */
static void hmi_render_stellaria_control_page(void)
{
    // Get current system state
    stellaria_snapshot_t data;
    telemetry_get_stellaria_data(&data);

    // Title
    hmi_fb_draw_string(16, 2, "Manual Control", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items with current state indicators
    const char *items[3];
    char enable_text[32];
    char intensity_text[32];

    items[0] = "< Back";

    // Enable/Disable item (shows user's setting and operational state)
    const char *state_str = "OFF";
    if (data.state == STELLARIA_STATE_SHUTDOWN) {
        state_str = "SHUTDOWN";  // Load shedding override
    } else if (data.state == STELLARIA_STATE_ENABLED) {
        // System is enabled - check mode
        if (data.auto_mode_active) {
            state_str = "AUTO";  // Auto mode enabled
        } else {
            state_str = "ON";  // Manually enabled
        }
    } else {
        // System is disabled (STELLARIA_STATE_DISABLED)
        if (data.auto_mode_active) {
            state_str = "AUTO (OFF)";  // Auto mode configured but disabled
        } else {
            state_str = "OFF";  // Manually disabled
        }
    }
    snprintf(enable_text, sizeof(enable_text), "Enable: %s", state_str);
    items[1] = enable_text;

    // Intensity item (shows current or editing value)
    if (stellaria_intensity_editing) {
        snprintf(intensity_text, sizeof(intensity_text), "Intensity: %d%%", stellaria_intensity_percent);
    } else {
        uint8_t current_percent = (uint8_t)((data.target_intensity * 100) / 1023);
        snprintf(intensity_text, sizeof(intensity_text), "Intensity: %d%%", current_percent);
    }
    items[2] = intensity_text;

    // Render menu items
    int y = 14;
    for (int i = 0; i < 3; i++) {
        bool selected = (i == hmi_status.selected_item);
        bool is_editable = (i == 2);  // Only Intensity is editable

        // Cursor logic: blink on editable field when selected
        if (selected && is_editable) {
            if (hmi_status.blink_state) {
                hmi_fb_draw_string(2, y, ">", false);
            }
        } else if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }

        // Item text
        hmi_fb_draw_string(8, y, items[i], selected);

        // Draw blinking "*" separately (not inverted)
        if (stellaria_intensity_editing && selected && is_editable && hmi_status.blink_state) {
            hmi_fb_draw_string(104, y, "*", false);
        }

        y += 14;  // Larger spacing for better visibility
    }

    // Footer - context-sensitive help
    if (stellaria_intensity_editing) {
        hmi_fb_draw_string(2, 56, "Rotate:adjust Press:save", false);
    } else {
        hmi_fb_draw_string(2, 56, "Press to execute/edit", false);
    }
}

/**
 * @brief Render STELLARIA Auto Mode detail page (interactive)
 */
static void hmi_render_stellaria_auto_page(void)
{
    stellaria_snapshot_t data;
    telemetry_get_stellaria_data(&data);

    // Navigation symbols
    hmi_draw_back_symbol();

    hmi_fb_draw_string(26, 2, "Auto Mode", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items with current state
    const char *items[2];
    char auto_text[32];

    items[0] = "< Back";

    // Toggle auto mode item (shows current state)
    if (data.auto_mode_active) {
        snprintf(auto_text, sizeof(auto_text), "Auto: ON");
    } else {
        snprintf(auto_text, sizeof(auto_text), "Auto: OFF");
    }
    items[1] = auto_text;

    // Render menu items
    int y = 14;
    for (int i = 0; i < 2; i++) {
        bool selected = (i == hmi_status.selected_item);

        // Selection indicator
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }

        // Item text
        hmi_fb_draw_string(8, y, items[i], selected);

        y += 12;
    }

    // Status information (below menu)
    y += 2;  // Extra spacing
    char buf[32];

    snprintf(buf, sizeof(buf), "Light: %.2fV", data.last_light_reading);
    hmi_fb_draw_string(2, y, buf, false);

    y += 8;
    hmi_fb_draw_string(2, y, "Uses FLUCTUS", false);
    y += 8;
    hmi_fb_draw_string(2, y, "avg light", false);
}

// ########################## SYSTEM Pages ##########################

/**
 * @brief Render SYSTEM submenu (Info, Controls)
 */
static void hmi_render_system_menu(void)
{
    // Title
    hmi_fb_draw_string(30, 2, "SYSTEM", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items
    const char *items[] = {
        "< Back",
        "System Info",
        "Controls",
        "Global Intervals"
    };

    int y = 14;
    for (int i = 0; i < 4; i++) {
        bool selected = (i == hmi_status.selected_item);

        // Selection indicator
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }

        // Item text
        hmi_fb_draw_string(8, y, items[i], selected);

        y += 12;
    }
}

/**
 * @brief Render System Info page (WiFi RSSI, uptime, version)
 */
static void hmi_render_system_info_page(void)
{
    // Get WiFi data from TELEMETRY cache
    wifi_snapshot_t wifi_data;
    telemetry_get_wifi_data(&wifi_data);

    // Navigation symbols
    hmi_draw_back_symbol();

    // Title
    hmi_fb_draw_string(24, 2, "System Info", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // WiFi state
    const char *wifi_state_str[] = {
        "DISABLED", "INIT", "CONNECTING",
        "CONNECTED", "RECONNECT", "FAILED"
    };
    snprintf(buf, sizeof(buf), "WiFi: %s",
             wifi_state_str[wifi_data.state < 6 ? wifi_data.state : 0]);
    hmi_fb_draw_string(2, y, buf, false);
    y += 9;

    // RSSI (only if connected)
    if (wifi_data.state == WIFI_STATE_CONNECTED && wifi_data.has_ip) {
        snprintf(buf, sizeof(buf), "RSSI: %d dBm", wifi_data.rssi);
        hmi_fb_draw_string(2, y, buf, false);
    } else {
        hmi_fb_draw_string(2, y, "RSSI: N/A", false);
    }
    y += 9;

    // Reconnection count
    snprintf(buf, sizeof(buf), "Reconnects: %u", wifi_data.reconnect_count);
    hmi_fb_draw_string(2, y, buf, false);
    y += 9;

    // Power save mode
    snprintf(buf, sizeof(buf), "Power Save: %s",
             wifi_data.power_save_mode ? "ON" : "OFF");
    hmi_fb_draw_string(2, y, buf, false);
    y += 11;

    // Uptime (use esp_timer for time since boot, not Unix timestamp)
    uint64_t uptime_us = esp_timer_get_time();  // Microseconds since boot
    uint32_t uptime_sec = (uint32_t)(uptime_us / 1000000ULL);
    uint32_t hours = uptime_sec / 3600;
    uint32_t minutes = (uptime_sec % 3600) / 60;
    snprintf(buf, sizeof(buf), "Uptime: %luh %lum", hours, minutes);
    hmi_fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render System Controls page (Flush & Reset, WiFi Reconnect)
 */
static void hmi_render_system_controls_page(void)
{
    // Title
    hmi_fb_draw_string(26, 2, "Controls", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items
    const char *items[] = {
        "< Back",
        "Flush buffer & Reset",
        "WiFi Reconnect"
    };

    int y = 14;
    for (int i = 0; i < 3; i++) {
        bool selected = (i == hmi_status.selected_item);

        // Selection indicator
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }

        // Item text
        hmi_fb_draw_string(8, y, items[i], selected);

        y += 12;
    }

    // Help text at bottom
    hmi_fb_draw_string(2, 56, "Press to select", false);
}

/**
 * @brief Render SYSTEM Intervals (preset selection) page
 */
static void hmi_render_system_intervals_page(void)
{
    // Navigation symbols
    hmi_draw_back_symbol();

    // Title
    hmi_fb_draw_string(22, 2, "Intervals", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Get current preset
    interval_preset_t current_preset = interval_config_get_current_preset();
    const char *preset_name = interval_config_get_preset_name(current_preset);

    // Menu items
    const char *items[5];
    items[0] = "< Back";
    items[1] = "Aggressive";
    items[2] = "Balanced";
    items[3] = "Conservative";

    // Current preset status (item 4)
    char buf[32];
    snprintf(buf, sizeof(buf), "Now: %s", preset_name);
    items[4] = buf;

    // Render menu items
    int y = 14;
    for (int i = 0; i < 5; i++) {
        bool selected = (i == hmi_status.selected_item);

        // Selection indicator
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }

        // Item text (highlight current preset if not Custom)
        bool highlight = (i == 4) ||  // Always highlight status line
                         (i == 1 && current_preset == INTERVAL_PRESET_AGGRESSIVE) ||
                         (i == 2 && current_preset == INTERVAL_PRESET_BALANCED) ||
                         (i == 3 && current_preset == INTERVAL_PRESET_CONSERVATIVE);

        hmi_fb_draw_string(8, y, items[i], selected || highlight);

        y += 10;  // Tighter spacing for 5 items
    }
}

// ########################## Shared Pages ##########################

/**
 * @brief Render shared confirmation dialog (used by all components)
 */
static void hmi_render_confirm_page(void)
{
    // Title and message based on action (shared by all components)
    const char *title = "Confirm?";
    const char *message = "";

    switch (confirmation_action) {
        case CONFIRM_TEMPESTA_RESET_DAILY:
            title = "Reset Daily?";
            message = "Daily counters→0";
            break;
        case CONFIRM_TEMPESTA_RESET_WEEKLY:
            title = "Reset Weekly?";
            message = "Weekly counters→0";
            break;
        case CONFIRM_TEMPESTA_RESET_RAIN_TOTAL:
            title = "Reset Rain?";
            message = "Hardware reset!";
            break;
        case CONFIRM_TEMPESTA_RESET_TANK_TOTAL:
            title = "Reset Tank?";
            message = "Hardware reset!";
            break;
        case CONFIRM_TEMPESTA_RESET_ALL:
            title = "Reset ALL?";
            message = "ALL hardware reset!";
            break;
        case CONFIRM_RESET_ZONE_LEARNING:
            title = "Reset Learning?";
            message = "Zone data will reset";
            break;
        case CONFIRM_RESET_ALL_LEARNING:
            title = "Reset All?";
            message = "All zones reset";
            break;
        case CONFIRM_MANUAL_WATER:
            title = "Start Watering?";
            message = "Safety override!";
            break;
        case CONFIRM_SYSTEM_FLUSH_RESET:
            title = "Flush->Flash & Reset?";
            message = "Save & restart MCU";
            break;
        default:
            break;
    }

    hmi_fb_draw_string(20, 12, title, false);
    hmi_fb_draw_string(10, 22, message, false);
    hmi_fb_draw_hline(0, 30, 128);

    int y = 36;

    // Yes option
    bool selected = (hmi_status.selected_item == 0);
    if (selected) hmi_fb_draw_string(30, y, ">", false);
    hmi_fb_draw_string(36, y, "Yes", selected);
    y += 10;

    // No option
    selected = (hmi_status.selected_item == 1);
    if (selected) hmi_fb_draw_string(30, y, ">", false);
    hmi_fb_draw_string(36, y, "No", selected);
}

// ########################## Public Dispatcher ##########################

/**
 * @brief SYSTEM/STELLARIA/shared rendering dispatcher
 * Routes to appropriate page renderer based on current menu state
 */
void hmi_render_system_pages(void)
{
    switch (hmi_status.current_menu) {
        // Main menu
        case HMI_MENU_MAIN:
            hmi_render_main_menu();
            break;

        // STELLARIA pages
        case HMI_MENU_STELLARIA:
            hmi_render_stellaria_menu();
            break;
        case HMI_MENU_STELLARIA_STATUS:
            hmi_render_stellaria_status_page();
            break;
        case HMI_MENU_STELLARIA_CONTROL:
            hmi_render_stellaria_control_page();
            break;
        case HMI_MENU_STELLARIA_AUTO:
            hmi_render_stellaria_auto_page();
            break;

        // SYSTEM pages
        case HMI_MENU_SYSTEM:
            hmi_render_system_menu();
            break;
        case HMI_MENU_SYSTEM_INFO:
            hmi_render_system_info_page();
            break;
        case HMI_MENU_SYSTEM_CONTROLS:
            hmi_render_system_controls_page();
            break;
        case HMI_MENU_SYSTEM_INTERVALS:
            hmi_render_system_intervals_page();
            break;

        // Shared confirmation dialog
        case HMI_MENU_CONFIRM:
            hmi_render_confirm_page();
            break;

        default:
            break;
    }
}
