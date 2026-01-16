/**
 * @file hmi_render_tempesta.c
 * @brief HMI Rendering Module - TEMPESTA (Weather Station)
 *
 * Renders all TEMPESTA-related display pages:
 * - Menu (dispatcher)
 * - Environment: Temperature, humidity, pressure
 * - Wind: Speed (m/s, RPM) and direction
 * - Air Quality: PM2.5 and PM10 readings
 * - Rain Gauge: Hourly, daily, weekly rainfall accumulation
 * - Tank Intake: Hourly, daily, weekly intake volume
 * - Controls: Interactive system controls and resets
 * - Intervals: Configuration page for collection intervals
 */

#include "hmi_private.h"

#include <stdio.h>

// ########################## TEMPESTA Page Rendering ##########################

/**
 * @brief Render TEMPESTA main menu (dispatcher) with scrolling support
 * Menu: 5 items (Back, Sensors, Diagnostics, Controls, Intervals)
 */
static void hmi_render_tempesta_menu(void)
{
    const uint8_t TOTAL_ITEMS = 5;
    const char *items[] = {
        "< Back",
        "Sensors",      // Paginated: Environment, Wind, Rain+Tank, Air
        "Diagnostics",  // Realtime: Hall array, AS5600
        "Controls",
        "Intervals"
    };

    // Get data from TELEMETRY cache
    tempesta_snapshot_t data;
    telemetry_get_tempesta_data(&data);

    // Title
    hmi_fb_draw_string(28, 2, "TEMPESTA", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Draw scroll indicators (none needed for 4 items)
    hmi_draw_scroll_indicators(TOTAL_ITEMS);

    // Calculate visible item range
    uint8_t first_visible = hmi_status.scroll_offset;
    uint8_t last_visible = first_visible + HMI_MAX_VISIBLE_ITEMS;
    if (last_visible > TOTAL_ITEMS) {
        last_visible = TOTAL_ITEMS;
    }

    // Draw visible items
    int y = HMI_MENU_START_Y;
    for (uint8_t i = first_visible; i < last_visible; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }
        hmi_fb_draw_string(8, y, items[i], selected);
        y += HMI_MENU_ITEM_SPACING;
    }

    // Show state and quick status at bottom
    char buf[32];
    const char *state_str = "UNKNOWN";
    switch (data.state) {
        case TEMPESTA_STATE_DISABLED:   state_str = "OFF"; break;
        case TEMPESTA_STATE_IDLE:       state_str = data.power_save_mode ? "P-SAVE" : "IDLE"; break;
        case TEMPESTA_STATE_READING:    state_str = "READ"; break;
        case TEMPESTA_STATE_SHUTDOWN:   state_str = "SHTDWN"; break;
        case TEMPESTA_STATE_ERROR:      state_str = "ERROR!"; break;
    }
    snprintf(buf, sizeof(buf), "%s %.1fC %.0f%%", state_str, data.temperature, data.humidity);
    hmi_fb_draw_string(2, 56, buf, false);
}

/**
 * @brief Render TEMPESTA Sensors paginated page
 * Combines all sensor readings across 4 pages:
 * - Page 1/4: Environment (Temperature, Humidity, Pressure)
 * - Page 2/4: Wind (Speed, Direction)
 * - Page 3/4: Rain Gauge + Tank Intake
 * - Page 4/4: Air Quality (PM2.5, PM10)
 */
static void hmi_render_tempesta_sensors_page(void)
{
    tempesta_snapshot_t data;
    telemetry_get_tempesta_data(&data);

    // Set pagination state (4 pages total)
    hmi_status.total_pages = 4;

    // Navigation symbols
    hmi_draw_back_symbol();
    hmi_draw_pagination_symbol();

    // Title with page indicator
    char title[24];
    snprintf(title, sizeof(title), "Sensors %d/%d",
             hmi_status.current_page + 1, hmi_status.total_pages);
    hmi_fb_draw_string(20, 2, title, false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    switch (hmi_status.current_page) {
        case 0:  // PAGE 1: Environment
            hmi_fb_draw_string(2, y, "=== ENVIRONMENT ===", false);
            y += 10;

            snprintf(buf, sizeof(buf), "Temperature: %.1fC", data.temperature);
            hmi_fb_draw_string(2, y, buf, false);
            y += 10;

            snprintf(buf, sizeof(buf), "Humidity: %.0f%%", data.humidity);
            hmi_fb_draw_string(2, y, buf, false);
            y += 10;

            snprintf(buf, sizeof(buf), "Pressure: %.0fhPa", data.pressure);
            hmi_fb_draw_string(2, y, buf, false);
            break;

        case 1:  // PAGE 2: Wind
            hmi_fb_draw_string(2, y, "=== WIND ===", false);
            y += 10;

            snprintf(buf, sizeof(buf), "Speed: %.1fm/s", data.wind_speed_ms);
            hmi_fb_draw_string(2, y, buf, false);
            y += 10;

            snprintf(buf, sizeof(buf), "       %.0fRPM", data.wind_speed_rpm);
            hmi_fb_draw_string(2, y, buf, false);
            y += 10;

            snprintf(buf, sizeof(buf), "Dir: %.1fdeg / %s", data.wind_direction_deg, data.wind_direction_cardinal);
            hmi_fb_draw_string(2, y, buf, false);
            break;

        case 2:  // PAGE 3: Rain + Tank
            hmi_fb_draw_string(2, y, "=== RAIN/TANK ===", false);
            y += 10;

            // Rain (left column)
            snprintf(buf, sizeof(buf), "Rain now: %.1fmm", data.rainfall_current_hour_mm);
            hmi_fb_draw_string(2, y, buf, false);
            y += 9;

            snprintf(buf, sizeof(buf), "Day/Week: %.1f/%.0f", data.rainfall_daily_mm, data.rainfall_weekly_mm);
            hmi_fb_draw_string(2, y, buf, false);
            y += 11;

            // Tank (below rain)
            snprintf(buf, sizeof(buf), "Tank now: %.0fmL", data.tank_intake_current_hour_ml);
            hmi_fb_draw_string(2, y, buf, false);
            y += 9;

            snprintf(buf, sizeof(buf), "Day/Week: %.0f/%.0f", data.tank_intake_daily_ml, data.tank_intake_weekly_ml);
            hmi_fb_draw_string(2, y, buf, false);
            break;

        case 3:  // PAGE 4: Air Quality
            hmi_fb_draw_string(2, y, "=== AIR QUALITY ===", false);
            y += 10;

            snprintf(buf, sizeof(buf), "PM2.5: %.0fug/m3", data.air_quality_pm25);
            hmi_fb_draw_string(2, y, buf, false);
            y += 10;

            snprintf(buf, sizeof(buf), "PM10:  %.0fug/m3", data.air_quality_pm10);
            hmi_fb_draw_string(2, y, buf, false);
            break;
    }
}

/**
 * @brief Render TEMPESTA System Controls page with scrolling
 * Supports 8 items with 7-item visible window
 */
static void hmi_render_tempesta_controls_page(void)
{
    // Get current system state
    tempesta_snapshot_t data;
    telemetry_get_tempesta_data(&data);

    const uint8_t TOTAL_ITEMS = 8;

    // Prepare dynamic enable/disable item text
    char enable_text[32];
    const char *state_str = "OFF";
    if (data.state == TEMPESTA_STATE_SHUTDOWN) {
        state_str = "SHUTDOWN";  // Load shedding override
    } else if (data.state == TEMPESTA_STATE_ERROR) {
        state_str = "ERROR";  // Sensor errors
    } else if (data.state == TEMPESTA_STATE_IDLE || data.state == TEMPESTA_STATE_READING) {
        // System is operational (IDLE or READING)
        if (data.power_save_mode) {
            state_str = "P-SAVE";  // Power save mode active
        } else {
            state_str = "ON";  // Normal operation
        }
    } else {
        // System is DISABLED
        if (data.power_save_mode) {
            state_str = "P-SAVE (OFF)";  // Power save configured but disabled
        } else {
            state_str = "OFF";  // Disabled
        }
    }
    snprintf(enable_text, sizeof(enable_text), "Enable: %s", state_str);

    const char *items[] = {
        "< Back",
        enable_text,  // Dynamic state display
        "Force Collection",
        "Reset Daily",
        "Reset Weekly",
        "Reset Rain Total",
        "Reset Tank Total",
        "Reset ALL"
    };

    // Title
    hmi_fb_draw_string(4, 2, "System Controls", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Draw scroll indicators
    hmi_draw_scroll_indicators(TOTAL_ITEMS);

    // Calculate visible item range
    uint8_t first_visible = hmi_status.scroll_offset;
    uint8_t last_visible = first_visible + HMI_MAX_VISIBLE_ITEMS;
    if (last_visible > TOTAL_ITEMS) {
        last_visible = TOTAL_ITEMS;
    }

    // Draw visible items only
    int y = HMI_MENU_START_Y;
    for (uint8_t i = first_visible; i < last_visible; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }
        hmi_fb_draw_string(8, y, items[i], selected);
        y += HMI_MENU_ITEM_SPACING;
    }
}

/**
 * @brief Render TEMPESTA Intervals configuration page
 * Configure normal and power save collection intervals
 */
static void hmi_render_tempesta_intervals_page(void)
{
    // Navigation symbols
    hmi_draw_back_symbol();

    // Title
    hmi_fb_draw_string(18, 2, "TEMPESTA Int", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items with current values
    char buf[32];
    const char *items[3];
    items[0] = "< Back";

    // Normal interval (item 1)
    bool editing_normal = (interval_editing && hmi_status.selected_item == 1);
    if (editing_normal) {
        snprintf(buf, sizeof(buf), "Normal: %lum", editing_interval_value);
    } else {
        snprintf(buf, sizeof(buf), "Normal: %lum", g_interval_config.tempesta_normal_min);
    }
    items[1] = buf;

    // Power Save interval (item 2)
    char buf2[32];
    bool editing_pwr = (interval_editing && hmi_status.selected_item == 2);
    if (editing_pwr) {
        snprintf(buf2, sizeof(buf2), "Pwr Save: %lum", editing_interval_value);
    } else {
        snprintf(buf2, sizeof(buf2), "Pwr Save: %lum", g_interval_config.tempesta_power_save_min);
    }
    items[2] = buf2;

    // Render menu items
    int y = 14;
    for (int i = 0; i < 3; i++) {
        bool selected = (i == hmi_status.selected_item);
        bool is_editable = (i >= 1 && i <= 2);

        // Cursor logic: always blink on editable fields when selected
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
        if (interval_editing && selected && is_editable && hmi_status.blink_state) {
            hmi_fb_draw_string(104, y, "*", false);
        }

        y += 12;
    }
}

/**
 * @brief Render TEMPESTA Diagnostics paginated page (realtime 8Hz)
 * Live sensor debugging for hardware validation:
 * - Page 1/2: Hall Array (Wind Direction) - 4 voltages + calculated direction
 * - Page 2/2: AS5600 (Wind Speed) - Angle + rotation counter + magnet status
 *
 * Includes timeout warning at 60s (30s before auto-exit at 90s)
 */
static void hmi_render_tempesta_diag_page(void)
{
    // Set pagination state (2 pages total)
    hmi_status.total_pages = 2;

    // Navigation symbols
    hmi_draw_back_symbol();
    hmi_draw_pagination_symbol();

    // Check for timeout and display warning
    time_t current_time = time(NULL);
    time_t elapsed = current_time - tempesta_diag_start_time;
    bool show_timeout_warning = (elapsed >= 60); // Warn at 60s (30s before auto-exit)

    // Get diagnostic data from TEMPESTA (direct API call, not telemetry)
    tempesta_diag_snapshot_t diag;
    esp_err_t ret = tempesta_get_diagnostic_data(&diag);

    if (ret != ESP_OK) {
        // Diagnostic mode not active - show error
        hmi_fb_draw_string(10, 2, "Diagnostic Error", false);
        hmi_fb_draw_hline(0, 10, 128);
        hmi_fb_draw_string(2, 20, "Mode not active", false);
        hmi_fb_draw_string(2, 30, "Press to exit", false);
        return;
    }

    // Page 1/2: Hall Array (Wind Direction)
    if (hmi_status.current_page == 0) {
        // Title with page indicator
        char title[20];
        snprintf(title, sizeof(title), "Hall %d/2", hmi_status.current_page + 1);
        hmi_fb_draw_string(40, 2, title, false);
        hmi_fb_draw_hline(0, 10, 128);

        // Display raw voltages (2 columns)
        char buf[32];
        int y = 14;

        snprintf(buf, sizeof(buf), "N:%.2fV  E:%.2fV", diag.hall_voltage_north, diag.hall_voltage_east);
        hmi_fb_draw_string(2, y, buf, false);
        y += 10;

        snprintf(buf, sizeof(buf), "S:%.2fV  W:%.2fV", diag.hall_voltage_south, diag.hall_voltage_west);
        hmi_fb_draw_string(2, y, buf, false);
        y += 10;

        // Calculated direction
        snprintf(buf, sizeof(buf), "Dir: %.0f%c (%s)", diag.hall_direction_deg, 176, diag.hall_direction_cardinal); // 176 = degree symbol
        hmi_fb_draw_string(2, y, buf, false);
        y += 10;

        // Signal magnitude (strength)
        snprintf(buf, sizeof(buf), "Mag: %.2f", diag.hall_magnitude);
        hmi_fb_draw_string(2, y, buf, false);
        y += 10;

        // Timeout warning if applicable
        if (show_timeout_warning) {
            snprintf(buf, sizeof(buf), "Exit in %llu s", 90 - elapsed);
            hmi_fb_draw_string(2, 56, buf, true); // Inverted text
        }
    }
    // Page 2/2: AS5600 (Wind Speed Sensor)
    else if (hmi_status.current_page == 1) {
        // Title with page indicator
        char title[20];
        snprintf(title, sizeof(title), "AS5600 %d/2", hmi_status.current_page + 1);
        hmi_fb_draw_string(34, 2, title, false);
        hmi_fb_draw_hline(0, 10, 128);

        // Display angle
        char buf[32];
        int y = 14;

        snprintf(buf, sizeof(buf), "Angle: %u", diag.as5600_raw_angle);
        hmi_fb_draw_string(2, y, buf, false);
        y += 10;

        snprintf(buf, sizeof(buf), "       %.1f%c", diag.as5600_angle_deg, 176); // 176 = degree symbol
        hmi_fb_draw_string(2, y, buf, false);
        y += 10;

        // Rotation counter (display whole rotations)
        int32_t whole_rotations = diag.as5600_rotation_count / 1000;
        snprintf(buf, sizeof(buf), "Rotations: %ld", whole_rotations);
        hmi_fb_draw_string(2, y, buf, false);
        y += 12;

        // Magnet status
        const char *mag_status = "NONE";
        if (diag.as5600_magnet_detected) {
            if (diag.as5600_magnet_too_strong) {
                mag_status = "TOO STRONG!";
            } else if (diag.as5600_magnet_too_weak) {
                mag_status = "TOO WEAK";
            } else {
                mag_status = "GOOD";
            }
        }

        hmi_fb_draw_string(2, y, "Magnet:", false);
        hmi_fb_draw_string(48, y, mag_status, diag.as5600_magnet_detected);
        y += 10;

        // Timeout warning if applicable
        if (show_timeout_warning) {
            snprintf(buf, sizeof(buf), "Exit in %llus", 90 - elapsed);
            hmi_fb_draw_string(2, 56, buf, true); // Inverted text
        } else {
            // Instruction text (spin magnet)
            hmi_fb_draw_string(2, 56, "Spin magnet...", false);
        }
    }
}

// ########################## Public Dispatcher ##########################

/**
 * @brief TEMPESTA rendering dispatcher
 * Routes to appropriate TEMPESTA page renderer based on current menu state
 */
void hmi_render_tempesta_pages(void)
{
    switch (hmi_status.current_menu) {
        case HMI_MENU_TEMPESTA:
            hmi_render_tempesta_menu();
            break;
        case HMI_MENU_TEMPESTA_SENSORS:
            hmi_render_tempesta_sensors_page();
            break;
        case HMI_MENU_TEMPESTA_DIAG:
            hmi_render_tempesta_diag_page();

            // Check for 90-second timeout and auto-exit
            if (tempesta_diag_active) {
                time_t elapsed = time(NULL) - tempesta_diag_start_time;
                if (elapsed >= 90) {

                    // Exit diagnostic mode
                    tempesta_exit_diagnostic_mode();

                    // Release 3.3V bus power
                    if (tempesta_diag_bus_requested) {
                        fluctus_release_bus_power(POWER_BUS_3V3, "HMI_TEMPESTA_DIAG");
                        tempesta_diag_bus_requested = false;
                    }

                    tempesta_diag_active = false;

                    // Navigate back to TEMPESTA menu
                    hmi_status.current_menu = HMI_MENU_TEMPESTA;
                    hmi_status.selected_item = 0;
                    hmi_status.current_page = 0;
                    hmi_status.total_pages = 0;
                }
            }
            break;
        case HMI_MENU_TEMPESTA_CONTROLS:
            hmi_render_tempesta_controls_page();
            break;
        case HMI_MENU_TEMPESTA_INTERVALS:
            hmi_render_tempesta_intervals_page();
            break;
        default:
            break;
    }
}
