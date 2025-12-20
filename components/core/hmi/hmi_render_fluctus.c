/**
 * @file hmi_render_fluctus.c
 * @brief HMI Rendering Module - FLUCTUS (Power Management & Solar Tracking)
 *
 * Renders all FLUCTUS-related display pages:
 * - Menu (dispatcher)
 * - Overview: Power state, battery SOC, solar status, thermal
 * - Energy: Hourly and daily energy statistics
 * - Live Power: Real-time battery/solar readings (4Hz)
 * - Buses: Power bus states and consumer counts
 * - Tracking: Solar tracking position and state
 * - Solar Debug: Tracking errors, photoresistors, servo duties (4Hz)
 * - Controls: Interactive toggles and actions
 * - Intervals: Configuration page for check intervals
 */

#include "hmi_private.h"
#include <stdio.h>

// ########################## FLUCTUS Page Rendering ##########################

/**
 * @brief Render FLUCTUS main menu (dispatcher) with scrolling support
 * Supports 9 items with 7-item visible window (items 0-6 visible initially)
 */
static void hmi_render_fluctus_menu(void)
{
    const uint8_t TOTAL_ITEMS = 9;
    const char *items[] = {
        "< Back",
        "Overview",
        "Energy Balance",
        "Live Power",      // 4Hz REALTIME
        "Power Buses",
        "Solar tracker",  // 4Hz REALTIME, 3 pages (Tracking, Position, Sensors)
        "Servo Debug",    // 4Hz REALTIME (manual servo control)
        "Controls",
        "Intervals"
    };

    // Title
    hmi_fb_draw_string(30, 2, "FLUCTUS", false);
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

        // Add [LIVE] indicator for realtime pages (items 3, 5, and 6)
        if ((i == 3 || i == 5 || i == 6) && hmi_status.blink_state) {
            hmi_fb_draw_string(90, y, "[LV]", false);
        }

        y += HMI_MENU_ITEM_SPACING;
    }
}

/**
 * @brief Render FLUCTUS Overview page [1Hz]
 * Shows power state, battery SOC, solar status, thermal
 */
static void hmi_render_fluctus_overview_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    hmi_fb_draw_string(28, 2, "Overview", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Power state
    const char *state_str = "NORMAL";
    switch (data.power_state) {
        case FLUCTUS_POWER_STATE_NORMAL: state_str = "NORMAL"; break;
        case FLUCTUS_POWER_STATE_POWER_SAVING: state_str = "SAVE"; break;
        case FLUCTUS_POWER_STATE_LOW_POWER: state_str = "LOW"; break;
        case FLUCTUS_POWER_STATE_VERY_LOW: state_str = "VERY LOW"; break;
        case FLUCTUS_POWER_STATE_CRITICAL: state_str = "CRITICAL"; break;
        case FLUCTUS_POWER_STATE_SHUTDOWN: state_str = "SHUTDOWN"; break;
    }
    snprintf(buf, sizeof(buf), "State: %s", state_str);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Safety status
    if (data.safety_shutdown) {
        hmi_fb_draw_string(2, y, "SAFETY SHUTDOWN!", false);
        y += 10;
    }

    // Battery SOC (15-min average)
    snprintf(buf, sizeof(buf), "Bat SOC: %.0f%% (avg)", data.battery_soc_avg_15min);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Solar status
    snprintf(buf, sizeof(buf), "Solar: %s", data.solar_pv_active ? "ACTIVE" : "OFF");
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Thermal
    snprintf(buf, sizeof(buf), "Temp: %.1fC Fan:%d%%",
             data.case_temperature, data.fan_speed_percent);
    hmi_fb_draw_string(2, y, buf, false);

    hmi_fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render FLUCTUS Energy page [1Hz] with pagination
 * Shows hourly and daily energy statistics across 2 pages
 */
static void hmi_render_fluctus_energy_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    // Set pagination state (2 pages total)
    hmi_status.total_pages = 2;

    // Navigation symbols
    hmi_draw_back_symbol();
    hmi_draw_pagination_symbol();

    // Title with page indicator
    char title[24];
    snprintf(title, sizeof(title), "Energy %d/%d",
             hmi_status.current_page + 1, hmi_status.total_pages);
    hmi_fb_draw_string(26, 2, title, false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    if (hmi_status.current_page == 0) {
        // ========== PAGE 1: HOURLY STATS ==========
        hmi_fb_draw_string(2, y, "=== HOURLY ===", false);
        y += 10;

        // PV hourly energy
        snprintf(buf, sizeof(buf), "PV produced: %d[Wh]", (int)data.pv_produced_wh_hour);
        hmi_fb_draw_string(2, y, buf, false);
        y += 10;

        snprintf(buf, sizeof(buf), "PV Peak: %d[W]", (int)data.pv_peak_w_hour);
        hmi_fb_draw_string(2, y, buf, false);
        y += 10;

        // Battery hourly energy
        snprintf(buf, sizeof(buf), "Bat. Used: %d[Wh]", (int)data.battery_consumed_wh_hour);
        hmi_fb_draw_string(2, y, buf, false);
        y += 10;

        snprintf(buf, sizeof(buf), "Bat Peak: %d[W]", (int)data.battery_peak_w_hour);
        hmi_fb_draw_string(2, y, buf, false);

    } else {
        // ========== PAGE 2: DAILY STATS ==========
        hmi_fb_draw_string(2, y, "=== DAILY ===", false);
        y += 10;

        // PV daily energy
        snprintf(buf, sizeof(buf), "PV produced: %.1f[kWh]", data.pv_produced_wh_day / 1000.0f);
        hmi_fb_draw_string(2, y, buf, false);
        y += 10;

        snprintf(buf, sizeof(buf), "PV Peak: %d[W]", (int)data.pv_peak_w_day);
        hmi_fb_draw_string(2, y, buf, false);
        y += 10;

        // Battery daily consumption
        snprintf(buf, sizeof(buf), "Bat. Used: %.1f[kWh]", data.battery_consumed_wh_day / 1000.0f);
        hmi_fb_draw_string(2, y, buf, false);
        y += 10;

        snprintf(buf, sizeof(buf), "Bat. Peak: %d[W]", (int)data.battery_peak_w_day);
        hmi_fb_draw_string(2, y, buf, false);
    }
}

/**
 * @brief Render FLUCTUS Live Power page [4Hz REALTIME]
 * Shows both instantaneous and 15-min average power values
 */
static void hmi_render_fluctus_live_power_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    // Set pagination state (2 pages total)
    hmi_status.total_pages = 2;

    // Navigation symbols
    hmi_draw_back_symbol();
    hmi_draw_pagination_symbol();

    // Title with page indicator and [LIVE] indicator
    char title[24];
    snprintf(title, sizeof(title), "Power %d/%d",
             hmi_status.current_page + 1, hmi_status.total_pages);
    hmi_fb_draw_string(22, 2, title, false);

    if (hmi_status.blink_state) {
        hmi_fb_draw_string(90, 2, "[LV]", false);
    }
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    if (hmi_status.current_page == 0) {
        // ========== PAGE 1: BATTERY DETAILS ==========
        hmi_fb_draw_string(2, y, "=== BATTERY NOW ===", false);
        y += 10;

        // Voltage and current
        snprintf(buf, sizeof(buf), "%.2f[V]  %.2f[A]",
                 data.battery_voltage_inst, data.battery_current_inst);
        hmi_fb_draw_string(2, y, buf, false);
        y += 9;

        // Power instantaneous
        snprintf(buf, sizeof(buf), "Power: %.1f[W]", data.battery_power_inst);
        hmi_fb_draw_string(2, y, buf, false);
        y += 9;

        // 15-min average power
        snprintf(buf, sizeof(buf), "15m Pavg: %.1f[W]", data.battery_power_avg_15min);
        hmi_fb_draw_string(2, y, buf, false);
        y += 9;

        // SOC average
        snprintf(buf, sizeof(buf), "SOC: %.0f%%", data.battery_soc_avg_15min);
        hmi_fb_draw_string(2, y, buf, false);

    } else {
        // ========== PAGE 2: SOLAR DETAILS ==========
        hmi_fb_draw_string(2, y, "=== SOLAR NOW ===", false);
        y += 10;

        // Voltage and current
        snprintf(buf, sizeof(buf), "%.1f[V]  %.2f[A]",
                 data.solar_voltage_inst, data.solar_current_inst);
        hmi_fb_draw_string(2, y, buf, false);
        y += 9;

        // Power instantaneous
        snprintf(buf, sizeof(buf), "Power: %.1f[W]", data.solar_power_inst);
        hmi_fb_draw_string(2, y, buf, false);
        y += 9;

        // 15-min average power
        snprintf(buf, sizeof(buf), "15m Pavg: %.1f[W]", data.solar_power_avg_15min);
        hmi_fb_draw_string(2, y, buf, false);
        y += 9;

        // Active status
        const char *status = data.solar_pv_active ? "ACTIVE" : "INACTIVE";
        snprintf(buf, sizeof(buf), "Status: %s", status);
        hmi_fb_draw_string(2, y, buf, false);
    }
}

/**
 * @brief Render FLUCTUS Buses page [1Hz]
 * Shows power bus enable states and consumer counts
 */
static void hmi_render_fluctus_buses_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    hmi_fb_draw_string(36, 2, "Power Buses", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // 3.3V bus
    snprintf(buf, sizeof(buf), "3.3V: %s (%d)",
             data.bus_3v3_enabled ? "ON " : "OFF",
             data.bus_3v3_consumers);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // 5V bus
    snprintf(buf, sizeof(buf), "5V:   %s (%d)",
             data.bus_5v_enabled ? "ON " : "OFF",
             data.bus_5v_consumers);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // 6.6V bus
    snprintf(buf, sizeof(buf), "6.6V: %s (%d)",
             data.bus_6v6_enabled ? "ON " : "OFF",
             data.bus_6v6_consumers);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // 12V bus
    snprintf(buf, sizeof(buf), "12V:  %s (%d)",
             data.bus_12v_enabled ? "ON " : "OFF",
             data.bus_12v_consumers);
    hmi_fb_draw_string(2, y, buf, false);

    hmi_fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render FLUCTUS Tracking/Debug merged page [4Hz REALTIME]
 * Combines solar tracking state and debug info across 4 pages:
 * - Page 1/4: Tracking state + Yaw/Pitch positions
 * - Page 2/4: Position errors (Yaw/Pitch)
 * - Page 3/4: Servo duties + Photoresistor readings
 * - Page 4/4: Debug Mode toggle (interactive control)
 */
static void hmi_render_fluctus_solar_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    // Set pagination state (4 pages total)
    hmi_status.total_pages = 4;

    // Navigation symbols
    hmi_draw_back_symbol();
    hmi_draw_pagination_symbol();

    // Title with page indicator and [LIVE] indicator
    char title[24];
    snprintf(title, sizeof(title), "Solar %d/%d",
             hmi_status.current_page + 1, hmi_status.total_pages);
    hmi_fb_draw_string(14, 2, title, false);

    if (hmi_status.blink_state) {
        hmi_fb_draw_string(90, 2, "[LV]", false);
    }
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    switch (hmi_status.current_page) {
        case 0:  // PAGE 1: Tracking State + Positions
            {
                // State
                const char *state_str = "DISABLED";
                switch (data.tracking_state) {
                    case SOLAR_TRACKING_DISABLED: state_str = "DISABLED"; break;
                    case SOLAR_TRACKING_SLEEPING: state_str = "SLEEPING"; break;
                    case SOLAR_TRACKING_STANDBY: state_str = "STANDBY"; break;
                    case SOLAR_TRACKING_CORRECTING: state_str = "CORRECT"; break;
                    case SOLAR_TRACKING_PARKING: state_str = "PARKING"; break;
                    case SOLAR_TRACKING_ERROR: state_str = "ERROR"; break;
                }
                snprintf(buf, sizeof(buf), "State: %s", state_str);
                hmi_fb_draw_string(2, y, buf, false);
                y += 12;

                // Positions
                snprintf(buf, sizeof(buf), "Yaw:   %d%%", data.yaw_position_percent);
                hmi_fb_draw_string(2, y, buf, false);
                y += 9;
                hmi_fb_draw_string(2, y, "       (azimuth)", false);
                y += 11;

                snprintf(buf, sizeof(buf), "Pitch: %d%%", data.pitch_position_percent);
                hmi_fb_draw_string(2, y, buf, false);
                y += 9;
                hmi_fb_draw_string(2, y, "       (elevation)", false);
                y += 11;

                // Status message
                if (data.tracking_state == SOLAR_TRACKING_STANDBY) {
                    snprintf(buf, sizeof(buf), "STDBY, INTRVL: %lu min", (unsigned long)g_interval_config.fluctus_solar_correction_min);
                    hmi_fb_draw_string(2, y, buf, false);
                } else if (data.tracking_state == SOLAR_TRACKING_SLEEPING) {
                    hmi_fb_draw_string(2, y, "Awaiting sunrise", false);
                }
            }
            break;

        case 1:  // PAGE 2: Position Errors
            hmi_fb_draw_string(2, y, "=== ERRORS ===", false);
            y += 10;

            // Yaw position and error
            snprintf(buf, sizeof(buf), "Yaw: %d%%", data.yaw_position_percent);
            hmi_fb_draw_string(2, y, buf, false);
            y += 10;

            snprintf(buf, sizeof(buf), "Err: %+.2f[V]", data.yaw_error);
            hmi_fb_draw_string(2, y, buf, false);
            y += 12;

            // Pitch position and error
            snprintf(buf, sizeof(buf), "Pitch: %d%%", data.pitch_position_percent);
            hmi_fb_draw_string(2, y, buf, false);
            y += 10;

            snprintf(buf, sizeof(buf), "Err: %+.2f[V]", data.pitch_error);
            hmi_fb_draw_string(2, y, buf, false);
            break;

        case 2:  // PAGE 3: Photoresistor Readings
            hmi_fb_draw_string(2, y, "=== SENSORS ===", false);
            y += 10;

            // Servo duties
            snprintf(buf, sizeof(buf), "Duty Y=%lu P=%lu",
                     data.current_yaw_duty, data.current_pitch_duty);
            hmi_fb_draw_string(2, y, buf, false);
            y += 12;

            // Photoresistors header
            hmi_fb_draw_string(2, y, "Photoresistors:", false);
            y += 10;

            // Top row (NE, SE)
            snprintf(buf, sizeof(buf), "NW:%.2f[V]", data.photoresistor_readings[0]);
            hmi_fb_draw_string(2, y, buf, false);
            snprintf(buf, sizeof(buf), "NE:%.2f[V]", data.photoresistor_readings[1]);
            hmi_fb_draw_string(66, y, buf, false);
            y += 10;

            // Bottom row (NW, SW)
            snprintf(buf, sizeof(buf), "SW:%.2f[V]", data.photoresistor_readings[2]);
            hmi_fb_draw_string(2, y, buf, false);
            snprintf(buf, sizeof(buf), "SE:%.2f[V]", data.photoresistor_readings[3]);
            hmi_fb_draw_string(66, y, buf, false);
            break;

        case 3:  // PAGE 4: Debug Mode Control (interactive)
            {
                // Force selector to item 1 (Debug toggle) on this page
                // This allows button press to toggle debug mode
                // User must rotate to other pages to go back
                if (hmi_status.selected_item == 0) {
                    hmi_status.selected_item = 1;
                }

                hmi_fb_draw_string(2, y, "=== DEBUG CONTROL ===", false);
                y += 12;

                // Check if debug mode is active
                bool debug_active = fluctus_is_solar_debug_mode_active();

                // Menu items
                char debug_text[32];
                snprintf(debug_text, sizeof(debug_text), "Debug: %s", debug_active ? "ON" : "OFF");

                // Always render item 1 as selected (we force it above)
                hmi_fb_draw_string(2, y, ">", false);
                hmi_fb_draw_string(8, y, debug_text, true);
                y += 14;

                // Info text
                if (debug_active) {
                    hmi_fb_draw_string(2, y, "Active! (90s max)", false);
                } else {
                    hmi_fb_draw_string(2, y, "Force tracking", false);
                    y += 8;
                    hmi_fb_draw_string(2, y, "bypass daylight", false);
                }

                y += 10;
                hmi_fb_draw_string(2, y, "Rotate to go back", false);
            }
            break;
    }
}

/**
 * @brief Render FLUCTUS Controls page (interactive actions)
 */
static void hmi_render_fluctus_controls_page(void)
{
    // Get current system state
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    // Title
    hmi_fb_draw_string(30, 2, "Controls", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items with current state indicators
    const char *items[3];
    items[0] = "< Back";

    // Toggle solar tracking item (shows current state)
    if (data.tracking_state == SOLAR_TRACKING_DISABLED) {
        items[1] = "Enable Tracking";
    } else if (data.tracking_state == SOLAR_TRACKING_SLEEPING) {
        items[1] = "Disable (Sleeping now)";
    } else {
        items[1] = "Disable Tracking";
    }

    // Manual safety reset
    if (data.safety_shutdown) {
        items[2] = "Safety Reset!";  // Urgent indicator
    } else {
        items[2] = "Safety Reset";
    }

    // Render menu items
    int y = 14;
    for (int i = 0; i < 3; i++) {
        bool selected = (i == hmi_status.selected_item);

        // Selection indicator
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }

        // Item text
        hmi_fb_draw_string(8, y, items[i], selected);

        y += 14;  // Larger spacing for better visibility
    }

    hmi_fb_draw_string(2, 56, "Press to execute", false);
}

/**
 * @brief Render FLUCTUS Intervals configuration page
 */
static void hmi_render_fluctus_intervals_page(void)
{
    // Navigation symbols
    hmi_draw_back_symbol();

    // Title
    hmi_fb_draw_string(20, 2, "FLCTS Interval", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items with current values
    char buf[32];
    const char *items[4];
    items[0] = "< Back";

    // Power Day interval (item 1)
    bool editing_day = (interval_editing && hmi_status.selected_item == 1);
    if (editing_day) {
        snprintf(buf, sizeof(buf), "Day: %lum", editing_interval_value);
    } else {
        snprintf(buf, sizeof(buf), "Day: %lum", g_interval_config.fluctus_power_day_min);
    }
    items[1] = buf;

    // Power Night interval (item 2)
    char buf2[32];
    bool editing_night = (interval_editing && hmi_status.selected_item == 2);
    if (editing_night) {
        snprintf(buf2, sizeof(buf2), "Night: %lum", editing_interval_value);
    } else {
        snprintf(buf2, sizeof(buf2), "Night: %lum", g_interval_config.fluctus_power_night_min);
    }
    items[2] = buf2;

    // Solar Correction interval (item 3)
    char buf3[32];
    bool editing_solar = (interval_editing && hmi_status.selected_item == 3);
    if (editing_solar) {
        snprintf(buf3, sizeof(buf3), "Solar: %lum", editing_interval_value);
    } else {
        snprintf(buf3, sizeof(buf3), "Solar: %lum", g_interval_config.fluctus_solar_correction_min);
    }
    items[3] = buf3;

    // Render menu items
    int y = 14;
    for (int i = 0; i < 4; i++) {
        bool selected = (i == hmi_status.selected_item);
        bool is_editable = (i >= 1 && i <= 3);

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
 * @brief Render FLUCTUS Servo Debug selection page
 * Simple menu to select which servo to control (yaw or pitch)
 */
static void hmi_render_fluctus_servo_debug_page(void)
{
    // Title
    hmi_fb_draw_string(20, 2, "Servo Debug", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items
    const char *items[] = {
        "< Back",
        "Control Yaw",
        "Control Pitch"
    };

    // Render menu items
    int y = 14;
    for (int i = 0; i < 3; i++) {
        bool selected = (i == hmi_status.selected_item);

        // Selection indicator
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }

        // Item text
        hmi_fb_draw_string(8, y, items[i], selected);

        y += 14;  // Larger spacing for better visibility
    }

    hmi_fb_draw_string(2, 56, "Press to select", false);
}

/**
 * @brief Render FLUCTUS Servo Control page for Yaw servo [4Hz REALTIME]
 * Displays current servo position and instructions for encoder control
 */
static void hmi_render_fluctus_servo_control_yaw_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    // Title with [LIVE] indicator
    hmi_fb_draw_string(10, 2, "Yaw Servo", false);
    if (hmi_status.blink_state) {
        hmi_fb_draw_string(90, 2, "[LV]", false);
    }
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Servo position (percentage)
    snprintf(buf, sizeof(buf), "Position: %d%%", data.yaw_position_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Raw duty cycle value
    snprintf(buf, sizeof(buf), "Duty: %lu", (unsigned long)data.current_yaw_duty);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Range info
    snprintf(buf, sizeof(buf), "Range: %d-%d", FLUCTUS_SERVO_MIN_DUTY, FLUCTUS_SERVO_MAX_DUTY);
    hmi_fb_draw_string(2, y, buf, false);
    y += 12;

    // Instructions
    hmi_fb_draw_string(2, y, "Rotate: Move servo", false);
    y += 9;
    hmi_fb_draw_string(2, y, "Step: 50 units", false);

    hmi_fb_draw_string(2, 56, "Press to exit", false);
}

/**
 * @brief Render FLUCTUS Servo Control page for Pitch servo [4Hz REALTIME]
 * Displays current servo position and instructions for encoder control
 */
static void hmi_render_fluctus_servo_control_pitch_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    // Title with [LIVE] indicator
    hmi_fb_draw_string(10, 2, "Pitch Servo", false);
    if (hmi_status.blink_state) {
        hmi_fb_draw_string(90, 2, "[LV]", false);
    }
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Servo position (percentage)
    snprintf(buf, sizeof(buf), "Position: %d%%", data.pitch_position_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Raw duty cycle value
    snprintf(buf, sizeof(buf), "Duty: %lu", (unsigned long)data.current_pitch_duty);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Range info
    snprintf(buf, sizeof(buf), "Range: %d-%d", FLUCTUS_SERVO_MIN_DUTY, FLUCTUS_SERVO_MAX_DUTY);
    hmi_fb_draw_string(2, y, buf, false);
    y += 12;

    // Instructions
    hmi_fb_draw_string(2, y, "Rotate: Move servo", false);
    y += 9;
    hmi_fb_draw_string(2, y, "Step: 50 units", false);

    hmi_fb_draw_string(2, 56, "Press to exit", false);
}

// ########################## Public Dispatcher ##########################

/**
 * @brief FLUCTUS rendering dispatcher
 * Routes to appropriate FLUCTUS page renderer based on current menu state
 */
void hmi_render_fluctus_pages(void)
{
    switch (hmi_status.current_menu) {
        case HMI_MENU_FLUCTUS:
            hmi_render_fluctus_menu();
            break;
        case HMI_MENU_FLUCTUS_OVERVIEW:
            hmi_render_fluctus_overview_page();
            break;
        case HMI_MENU_FLUCTUS_ENERGY:
            hmi_render_fluctus_energy_page();
            break;
        case HMI_MENU_FLUCTUS_LIVE_POWER:
            hmi_render_fluctus_live_power_page();
            break;
        case HMI_MENU_FLUCTUS_BUSES:
            hmi_render_fluctus_buses_page();
            break;
        case HMI_MENU_FLUCTUS_SOLAR:
            hmi_render_fluctus_solar_page();
            break;
        case HMI_MENU_FLUCTUS_SERVO_DEBUG:
            hmi_render_fluctus_servo_debug_page();
            break;
        case HMI_MENU_FLUCTUS_SERVO_CONTROL_YAW:
            hmi_render_fluctus_servo_control_yaw_page();
            break;
        case HMI_MENU_FLUCTUS_SERVO_CONTROL_PITCH:
            hmi_render_fluctus_servo_control_pitch_page();
            break;
        case HMI_MENU_FLUCTUS_CONTROLS:
            hmi_render_fluctus_controls_page();
            break;
        case HMI_MENU_FLUCTUS_INTERVALS:
            hmi_render_fluctus_intervals_page();
            break;
        default:
            break;
    }
}
