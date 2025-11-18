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
 * @brief Render FLUCTUS main menu (dispatcher)
 */
static void hmi_render_fluctus_menu(void)
{
    // Get data from TELEMETRY cache
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    // Title
    hmi_fb_draw_string(30, 2, "FLUCTUS", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items (9 total: Back + 8 pages)
    const char *items[] = {
        "< Back",
        "Overview",
        "Energy",
        "Live Power",   // 4Hz REALTIME
        "Buses",
        "Tracking",
        "Solar Debug",  // 4Hz REALTIME
        "Controls",
        "Intervals"
    };

    int y = 14;
    for (int i = 0; i < 9; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }
        hmi_fb_draw_string(8, y, items[i], selected);

        // Add [LIVE] indicator for realtime pages (items 3 and 6)
        if (i == 3 || i == 6) {
            if (hmi_status.blink_state) {
                hmi_fb_draw_string(75, y, "[LIVE]", false);
            }
        }
        y += 7;  // Tight spacing to fit 9 items (9*7=63 pixels total)
    }

    // No bottom status bar - not enough space with 9 items
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
 * @brief Render FLUCTUS Energy page [1Hz]
 * Shows hourly and daily energy statistics
 */
static void hmi_render_fluctus_energy_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    hmi_fb_draw_string(32, 2, "Energy", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Hourly stats
    hmi_fb_draw_string(2, y, "=== HOURLY ===", false);
    y += 9;

    snprintf(buf, sizeof(buf), "PV: %dWh Pk:%dW",
             (int)data.pv_energy_wh_hour, (int)data.pv_peak_w_hour);
    hmi_fb_draw_string(2, y, buf, false);
    y += 9;

    snprintf(buf, sizeof(buf), "Bat:%dWh Pk:%dW",
             (int)data.battery_energy_wh_hour, (int)data.battery_peak_w_hour);
    hmi_fb_draw_string(2, y, buf, false);
    y += 11;

    // Daily stats
    hmi_fb_draw_string(2, y, "=== DAILY ===", false);
    y += 9;

    snprintf(buf, sizeof(buf), "PV: %.1fkWh Pk:%dW",
             data.pv_energy_wh_day / 1000.0f, (int)data.pv_peak_w_day);
    hmi_fb_draw_string(2, y, buf, false);
    y += 9;

    snprintf(buf, sizeof(buf), "Bat:%.1fkWh Pk:%dW",
             data.battery_consumed_wh_day / 1000.0f, (int)data.battery_peak_w_day);
    hmi_fb_draw_string(2, y, buf, false);

    hmi_fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render FLUCTUS Live Power page [4Hz REALTIME]
 * Shows both instantaneous and 15-min average power values
 */
static void hmi_render_fluctus_live_power_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    // Title with [LIVE] indicator
    hmi_fb_draw_string(22, 2, "Live Power", false);
    if (hmi_status.blink_state) {
        hmi_fb_draw_string(82, 2, "[LIVE]", false);
    }
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Battery - instantaneous
    hmi_fb_draw_string(2, y, "=== BATTERY NOW ===", false);
    y += 9;
    snprintf(buf, sizeof(buf), "%.2fV %.2fA %.1fW",
             data.battery_voltage_inst, data.battery_current_inst, data.battery_power_inst);
    hmi_fb_draw_string(2, y, buf, false);
    y += 9;

    // Battery - 15min average
    snprintf(buf, sizeof(buf), "15m: %.1fW %.0f%%",
             data.battery_power_avg_15min, data.battery_soc_avg_15min);
    hmi_fb_draw_string(2, y, buf, false);
    y += 11;

    // Solar - instantaneous
    hmi_fb_draw_string(2, y, "=== SOLAR NOW ===", false);
    y += 9;
    snprintf(buf, sizeof(buf), "%.1fV %.2fA %.1fW",
             data.solar_voltage_inst, data.solar_current_inst, data.solar_power_inst);
    hmi_fb_draw_string(2, y, buf, false);
    y += 9;

    // Solar - 15min average
    snprintf(buf, sizeof(buf), "15m: %.1fW", data.solar_power_avg_15min);
    hmi_fb_draw_string(2, y, buf, false);

    hmi_fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render FLUCTUS Buses page [1Hz]
 * Shows power bus enable states and consumer counts
 */
static void hmi_render_fluctus_buses_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    hmi_fb_draw_string(36, 2, "Buses", false);
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
 * @brief Render FLUCTUS Tracking page [1Hz]
 * Shows solar tracking position and state
 */
static void hmi_render_fluctus_tracking_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    hmi_fb_draw_string(28, 2, "Tracking", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

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
    snprintf(buf, sizeof(buf), "Yaw:   %d%% (azimuth)", data.yaw_position_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pitch: %d%% (elev)", data.pitch_position_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 12;

    // Status message
    if (data.tracking_state == SOLAR_TRACKING_STANDBY) {
        hmi_fb_draw_string(2, y, "Next: 12 min", false);
    } else if (data.tracking_state == SOLAR_TRACKING_SLEEPING) {
        hmi_fb_draw_string(2, y, "Awaiting sunrise", false);
    }

    hmi_fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render FLUCTUS Solar Debug page [4Hz REALTIME]
 * Shows tracking errors, photoresistors, and servo duties
 */
static void hmi_render_fluctus_solar_debug_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    // Title with [LIVE] indicator
    hmi_fb_draw_string(16, 2, "Solar Debug", false);
    if (hmi_status.blink_state) {
        hmi_fb_draw_string(80, 2, "[LIVE]", false);
    }
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Position with errors
    snprintf(buf, sizeof(buf), "Yaw:%d%% Err:%+.2fV",
             data.yaw_position_percent, data.yaw_error);
    hmi_fb_draw_string(2, y, buf, false);
    y += 9;

    snprintf(buf, sizeof(buf), "Pit:%d%% Err:%+.2fV",
             data.pitch_position_percent, data.pitch_error);
    hmi_fb_draw_string(2, y, buf, false);
    y += 9;

    // Servo duties
    snprintf(buf, sizeof(buf), "Duty: Y=%lu P=%lu",
             data.current_yaw_duty, data.current_pitch_duty);
    hmi_fb_draw_string(2, y, buf, false);
    y += 11;

    // Photoresistors (2x2 grid)
    hmi_fb_draw_string(2, y, "Photoresistors:", false);
    y += 9;
    snprintf(buf, sizeof(buf), "NE:%d SE:%d",
             (int)data.photoresistor_readings[0], (int)data.photoresistor_readings[1]);
    hmi_fb_draw_string(2, y, buf, false);
    y += 9;
    snprintf(buf, sizeof(buf), "NW:%d SW:%d",
             (int)data.photoresistor_readings[2], (int)data.photoresistor_readings[3]);
    hmi_fb_draw_string(2, y, buf, false);

    hmi_fb_draw_string(2, 56, "< Press to go back", false);
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
        items[1] = "Disable (SLEEP)";
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
    // Title
    hmi_fb_draw_string(20, 2, "FLUCTUS Int", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items with current values
    char buf[32];
    const char *items[4];
    items[0] = "< Back";

    // Power Day interval (item 1)
    if (interval_editing && hmi_status.selected_item == 1) {
        snprintf(buf, sizeof(buf), "Day: %lum*", editing_interval_value);
    } else {
        snprintf(buf, sizeof(buf), "Day: %lum", g_interval_config.fluctus_power_day_min);
    }
    items[1] = buf;

    // Power Night interval (item 2)
    char buf2[32];
    if (interval_editing && hmi_status.selected_item == 2) {
        snprintf(buf2, sizeof(buf2), "Night: %lum*", editing_interval_value);
    } else {
        snprintf(buf2, sizeof(buf2), "Night: %lum", g_interval_config.fluctus_power_night_min);
    }
    items[2] = buf2;

    // Solar Correction interval (item 3)
    char buf3[32];
    if (interval_editing && hmi_status.selected_item == 3) {
        snprintf(buf3, sizeof(buf3), "Solar: %lum*", editing_interval_value);
    } else {
        snprintf(buf3, sizeof(buf3), "Solar: %lum", g_interval_config.fluctus_solar_correction_min);
    }
    items[3] = buf3;

    // Render menu items
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

    // Help text at bottom
    if (interval_editing) {
        hmi_fb_draw_string(2, 56, "Rotate to edit", false);
    } else {
        hmi_fb_draw_string(2, 56, "Press to edit", false);
    }
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
        case HMI_MENU_FLUCTUS_TRACKING:
            hmi_render_fluctus_tracking_page();
            break;
        case HMI_MENU_FLUCTUS_SOLAR_DEBUG:
            hmi_render_fluctus_solar_debug_page();
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
