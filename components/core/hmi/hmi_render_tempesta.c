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
 * @brief Render TEMPESTA main menu (dispatcher)
 */
static void hmi_render_tempesta_menu(void)
{
    // Get data from TELEMETRY cache
    tempesta_snapshot_t data;
    telemetry_get_tempesta_data(&data);

    // Title
    hmi_fb_draw_string(28, 2, "TEMPESTA", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items (8 total: Back + 7 pages)
    const char *items[] = {
        "< Back",
        "Environment",
        "Wind",
        "Rain Gauge",
        "Tank Intake",
        "Air Quality",
        "Controls",
        "Intervals"
    };

    int y = 14;
    for (int i = 0; i < 8; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }
        hmi_fb_draw_string(8, y, items[i], selected);
        y += 6;  // Tight spacing to fit 8 items
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
 * @brief Render TEMPESTA Environment detail page
 * Shows temperature, humidity, and pressure
 */
static void hmi_render_tempesta_env_page(void)
{
    tempesta_snapshot_t data;
    telemetry_get_tempesta_data(&data);

    hmi_fb_draw_string(24, 2, "Environment", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Temp:     %.1fC", data.temperature);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Humidity: %.0f%%", data.humidity);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pressure: %.0fhPa", data.pressure);
    hmi_fb_draw_string(2, y, buf, false);

    hmi_fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render TEMPESTA Wind detail page
 * Shows wind speed (m/s, RPM) and direction
 */
static void hmi_render_tempesta_wind_page(void)
{
    tempesta_snapshot_t data;
    telemetry_get_tempesta_data(&data);

    hmi_fb_draw_string(34, 2, "Wind", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Speed: %.1fm/s", data.wind_speed_ms);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "       %.0fRPM", data.wind_speed_rpm);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Dir:   %.0fdeg", data.wind_direction_deg);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "       %s", data.wind_direction_cardinal);
    hmi_fb_draw_string(2, y, buf, false);

    hmi_fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render TEMPESTA Air Quality detail page
 * Shows PM2.5 and PM10 readings
 */
static void hmi_render_tempesta_air_page(void)
{
    tempesta_snapshot_t data;
    telemetry_get_tempesta_data(&data);

    hmi_fb_draw_string(20, 2, "Air Quality", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "PM2.5: %.0fug/m3", data.air_quality_pm25);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "PM10:  %.0fug/m3", data.air_quality_pm10);
    hmi_fb_draw_string(2, y, buf, false);

    hmi_fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render TEMPESTA Rain Gauge detail page
 * Shows hourly, daily, and weekly rainfall accumulation
 */
static void hmi_render_tempesta_rain_page(void)
{
    tempesta_snapshot_t data;
    telemetry_get_tempesta_data(&data);

    hmi_fb_draw_string(22, 2, "Rain Gauge", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Last:  %.1fmm", data.rainfall_last_hour_mm);
    hmi_fb_draw_string(2, y, buf, false);
    y += 9;

    snprintf(buf, sizeof(buf), "Now:   %.1fmm", data.rainfall_current_hour_mm);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Daily: %.1fmm", data.rainfall_daily_mm);
    hmi_fb_draw_string(2, y, buf, false);
    y += 9;

    snprintf(buf, sizeof(buf), "Week:  %.1fmm", data.rainfall_weekly_mm);
    hmi_fb_draw_string(2, y, buf, false);

    hmi_fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render TEMPESTA Tank Intake detail page
 * Shows hourly, daily, and weekly intake volume
 */
static void hmi_render_tempesta_tank_page(void)
{
    tempesta_snapshot_t data;
    telemetry_get_tempesta_data(&data);

    hmi_fb_draw_string(16, 2, "Tank Intake", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Last:  %.0fmL", data.tank_intake_last_hour_ml);
    hmi_fb_draw_string(2, y, buf, false);
    y += 9;

    snprintf(buf, sizeof(buf), "Now:   %.0fmL", data.tank_intake_current_hour_ml);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Daily: %.0fmL", data.tank_intake_daily_ml);
    hmi_fb_draw_string(2, y, buf, false);
    y += 9;

    snprintf(buf, sizeof(buf), "Week:  %.0fmL", data.tank_intake_weekly_ml);
    hmi_fb_draw_string(2, y, buf, false);

    hmi_fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render TEMPESTA System Controls page
 * Interactive controls for system toggles and resets
 */
static void hmi_render_tempesta_controls_page(void)
{
    // Title
    hmi_fb_draw_string(4, 2, "System Controls", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items
    const char *items[] = {
        "< Back",
        "Enable/Disable",
        "Force Collection",
        "Reset Daily",
        "Reset Weekly",
        "Reset Rain Total",
        "Reset Tank Total",
        "Reset ALL"
    };

    int y = 14;
    for (int i = 0; i < 8; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }
        hmi_fb_draw_string(8, y, items[i], selected);
        y += 8;  // Tighter spacing for 8 items
    }
}

/**
 * @brief Render TEMPESTA Intervals configuration page
 * Configure normal and power save collection intervals
 */
static void hmi_render_tempesta_intervals_page(void)
{
    // Title
    hmi_fb_draw_string(18, 2, "TEMPESTA Int", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items with current values
    char buf[32];
    const char *items[3];
    items[0] = "< Back";

    // Normal interval (item 1)
    if (interval_editing && hmi_status.selected_item == 1) {
        snprintf(buf, sizeof(buf), "Normal: %lum*", editing_interval_value);
    } else {
        snprintf(buf, sizeof(buf), "Normal: %lum", g_interval_config.tempesta_normal_min);
    }
    items[1] = buf;

    // Power Save interval (item 2)
    char buf2[32];
    if (interval_editing && hmi_status.selected_item == 2) {
        snprintf(buf2, sizeof(buf2), "Pwr Save: %lum*", editing_interval_value);
    } else {
        snprintf(buf2, sizeof(buf2), "Pwr Save: %lum", g_interval_config.tempesta_power_save_min);
    }
    items[2] = buf2;

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
 * @brief TEMPESTA rendering dispatcher
 * Routes to appropriate TEMPESTA page renderer based on current menu state
 */
void hmi_render_tempesta_pages(void)
{
    switch (hmi_status.current_menu) {
        case HMI_MENU_TEMPESTA:
            hmi_render_tempesta_menu();
            break;
        case HMI_MENU_TEMPESTA_ENV:
            hmi_render_tempesta_env_page();
            break;
        case HMI_MENU_TEMPESTA_WIND:
            hmi_render_tempesta_wind_page();
            break;
        case HMI_MENU_TEMPESTA_RAIN:
            hmi_render_tempesta_rain_page();
            break;
        case HMI_MENU_TEMPESTA_TANK:
            hmi_render_tempesta_tank_page();
            break;
        case HMI_MENU_TEMPESTA_AIR:
            hmi_render_tempesta_air_page();
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
