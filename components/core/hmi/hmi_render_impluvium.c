/**
 * @file hmi_render_impluvium.c
 * @brief HMI Rendering Module - IMPLUVIUM (Irrigation System)
 *
 * Renders all IMPLUVIUM-related display pages (23 total):
 * - Menu (dispatcher)
 * - Overview: System state, water level, daily stats
 * - Statistics: Per-zone hourly/daily water usage
 * - Zones submenu + 6 pages (all zones summary + 5 individual zone details)
 * - Learning submenu + 6 pages (all zones summary + 5 individual learning stats)
 * - Monitor: Real-time pressure, flow, pump during watering (4Hz)
 * - Controls: Interactive system controls
 * - Zone Config: Zone selection for editing
 * - Zone Edit: Edit zone parameters
 * - Manual Water: Manual watering time input
 * - Intervals: Configuration page for moisture check intervals
 */

#include "hmi_private.h"
#include <stdio.h>

// ########################## IMPLUVIUM Page Rendering ##########################

/**
 * @brief Render IMPLUVIUM main menu (dispatcher)
 */
static void hmi_render_impluvium_menu(void)
{
    // Get data from TELEMETRY cache
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    // Title
    hmi_fb_draw_string(24, 2, "IMPLUVIUM", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items (9 total: Back + 8 pages)
    const char *items[] = {
        "< Back",
        "Overview",
        "Statistics",
        "Zones",        // Submenu
        "Learning",     // Submenu
        "Monitor",      // 4Hz REALTIME
        "Controls",
        "Zone Config",
        "Intervals"
    };

    int y = 14;
    for (int i = 0; i < 9; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }
        hmi_fb_draw_string(8, y, items[i], selected);

        // Add [LIVE] indicator for Monitor (item 5)
        if (i == 5) {
            if (data.state == IMPLUVIUM_WATERING || data.state == IMPLUVIUM_MEASURING) {
                if (hmi_status.blink_state) {
                    hmi_fb_draw_string(70, y, "[LIVE]", false);
                }
            }
        }
        y += 6;  // Tight spacing to fit 9 items
    }

    // No bottom status bar - not enough space with 9 items
}

/**
 * @brief Render IMPLUVIUM OVERVIEW page [1Hz]
 */
static void hmi_render_impluvium_overview_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(24, 2, "Overview", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // System state
    const char *state_str = "STANDBY";
    switch (data.state) {
        case IMPLUVIUM_STANDBY: state_str = "STANDBY"; break;
        case IMPLUVIUM_MEASURING: state_str = "MEASURING"; break;
        case IMPLUVIUM_WATERING: state_str = "WATERING"; break;
        case IMPLUVIUM_STOPPING: state_str = "STOPPING"; break;
        case IMPLUVIUM_MAINTENANCE: state_str = "MAINT!"; break;
        case IMPLUVIUM_DISABLED: state_str = "DISABLED"; break;
    }
    snprintf(buf, sizeof(buf), "State: %s", state_str);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Water level
    snprintf(buf, sizeof(buf), "Water: %.0f%%", data.water_level_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Total water used today
    snprintf(buf, sizeof(buf), "Today: %.1fL", data.total_water_used_day_ml / 1000.0f);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Watering events today
    snprintf(buf, sizeof(buf), "Events: %d", data.watering_events_day);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Active zone (if any)
    if (data.active_zone != NO_ACTIVE_ZONE_ID) {
        snprintf(buf, sizeof(buf), "Active: Z%d", data.active_zone + 1);
        hmi_fb_draw_string(2, y, buf, false);
    }
}

/**
 * @brief Render IMPLUVIUM STATISTICS page [1Hz]
 */
static void hmi_render_impluvium_statistics_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(20, 2, "Statistics", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Header: Hr = current hour, Avg = average/hour, Day = total today
    hmi_fb_draw_string(2, y, "Hour (l) Avg (l/h) Day (l)", false);
    y += 10;

    // Zone statistics in 2-column layout
    // Left column: Z1, Z2, Z3
    // Right column: Z4, Z5
    int left_x = 2;
    int right_x = 66;
    y = 24; // Start data rows

    for (int i = 0; i < 3; i++) {
        // Left column (Z1-Z3)
        float vol_hour_l = data.zones[i].volume_used_hour_ml / 1000.0f;
        float avg_hour_l = data.zones[i].avg_hourly_consumption_ml / 1000.0f;
        float vol_day_l = data.zones[i].volume_used_today_ml / 1000.0f;

        snprintf(buf, sizeof(buf), "Z%d:%.1f %.1f %.1f", i + 1, vol_hour_l, avg_hour_l, vol_day_l);
        hmi_fb_draw_string(left_x, y, buf, false);

        // Right column (Z4-Z5, only first 2 iterations)
        if (i < 2) {
            int zone_idx = i + 3; // Z4=3, Z5=4
            vol_hour_l = data.zones[zone_idx].volume_used_hour_ml / 1000.0f;
            avg_hour_l = data.zones[zone_idx].avg_hourly_consumption_ml / 1000.0f;
            vol_day_l = data.zones[zone_idx].volume_used_today_ml / 1000.0f;

            snprintf(buf, sizeof(buf), "Z%d:%.1f %.1f %.1f", zone_idx + 1, vol_hour_l, avg_hour_l, vol_day_l);
            hmi_fb_draw_string(right_x, y, buf, false);
        }

        y += 10;
    }
}

/**
 * @brief Render IMPLUVIUM Zones submenu selector
 */
static void hmi_render_impluvium_zones_menu(void)
{
    hmi_fb_draw_string(28, 2, "Zones", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items (7 total: Back + All Summary + 5 zones)
    const char *items[] = {
        "< Back",
        "All Zones",
        "Zone 1",
        "Zone 2",
        "Zone 3",
        "Zone 4",
        "Zone 5"
    };

    int y = 14;
    for (int i = 0; i < 7; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }
        hmi_fb_draw_string(8, y, items[i], selected);
        y += 10;
    }
}

/**
 * @brief Render IMPLUVIUM All Zones Summary page
 */
static void hmi_render_impluvium_zones_all_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(16, 2, "All Zones", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Show all 5 zones with enable status and current moisture
    for (int i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        const char *enabled_str = data.zones[i].watering_enabled ? "EN" : "DS";
        snprintf(buf, sizeof(buf), "Z%d: %s %.0f%%/%.0f%%",
                 i + 1,
                 enabled_str,
                 data.zones[i].current_moisture_percent,
                 data.zones[i].target_moisture_percent);
        hmi_fb_draw_string(2, y, buf, false);
        y += 10;
    }
}

/**
 * @brief Render IMPLUVIUM Zone 1 detail page
 */
static void hmi_render_impluvium_zone_1_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(40, 2, "Zone 1", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Enabled status
    const char *status = data.zones[0].watering_enabled ? "Enabled" : "Disabled";
    snprintf(buf, sizeof(buf), "Status: %s", status);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Current moisture
    snprintf(buf, sizeof(buf), "Moisture: %.0f%%", data.zones[0].current_moisture_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Target
    snprintf(buf, sizeof(buf), "Target: %.0f%%", data.zones[0].target_moisture_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Deadband
    snprintf(buf, sizeof(buf), "Deadband: %.0f%%", data.zones[0].moisture_deadband_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Volume used today
    snprintf(buf, sizeof(buf), "Today: %.1fL", data.zones[0].volume_used_today_ml / 1000.0f);
    hmi_fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 2 detail page
 */
static void hmi_render_impluvium_zone_2_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(40, 2, "Zone 2", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    const char *status = data.zones[1].watering_enabled ? "Enabled" : "Disabled";
    snprintf(buf, sizeof(buf), "Status: %s", status);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Moisture: %.0f%%", data.zones[1].current_moisture_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Target: %.0f%%", data.zones[1].target_moisture_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Deadband: %.0f%%", data.zones[1].moisture_deadband_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Today: %.1fL", data.zones[1].volume_used_today_ml / 1000.0f);
    hmi_fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 3 detail page
 */
static void hmi_render_impluvium_zone_3_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(40, 2, "Zone 3", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    const char *status = data.zones[2].watering_enabled ? "Enabled" : "Disabled";
    snprintf(buf, sizeof(buf), "Status: %s", status);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Moisture: %.0f%%", data.zones[2].current_moisture_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Target: %.0f%%", data.zones[2].target_moisture_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Deadband: %.0f%%", data.zones[2].moisture_deadband_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Today: %.1fL", data.zones[2].volume_used_today_ml / 1000.0f);
    hmi_fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 4 detail page
 */
static void hmi_render_impluvium_zone_4_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(40, 2, "Zone 4", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    const char *status = data.zones[3].watering_enabled ? "Enabled" : "Disabled";
    snprintf(buf, sizeof(buf), "Status: %s", status);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Moisture: %.0f%%", data.zones[3].current_moisture_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Target: %.0f%%", data.zones[3].target_moisture_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Deadband: %.0f%%", data.zones[3].moisture_deadband_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Today: %.1fL", data.zones[3].volume_used_today_ml / 1000.0f);
    hmi_fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 5 detail page
 */
static void hmi_render_impluvium_zone_5_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(40, 2, "Zone 5", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    const char *status = data.zones[4].watering_enabled ? "Enabled" : "Disabled";
    snprintf(buf, sizeof(buf), "Status: %s", status);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Moisture: %.0f%%", data.zones[4].current_moisture_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Target: %.0f%%", data.zones[4].target_moisture_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Deadband: %.0f%%", data.zones[4].moisture_deadband_percent);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Today: %.1fL", data.zones[4].volume_used_today_ml / 1000.0f);
    hmi_fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Learning submenu selector
 */
static void hmi_render_impluvium_learning_menu(void)
{
    hmi_fb_draw_string(24, 2, "Learning", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items (7 total: Back + All Summary + 5 zones)
    const char *items[] = {
        "< Back",
        "All Zones",
        "Zone 1",
        "Zone 2",
        "Zone 3",
        "Zone 4",
        "Zone 5"
    };

    int y = 14;
    for (int i = 0; i < 7; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }
        hmi_fb_draw_string(8, y, items[i], selected);
        y += 10;
    }
}

/**
 * @brief Render IMPLUVIUM Learning Summary (all zones)
 */
static void hmi_render_impluvium_learning_all_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(10, 2, "Learn Summary", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Show all 5 zones with confidence and PPMP ratio
    for (int i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        snprintf(buf, sizeof(buf), "Z%d: %.0f%% %.1f",
                 i + 1,
                 data.zones[i].confidence_level * 100.0f,
                 data.zones[i].calculated_ppmp_ratio);
        hmi_fb_draw_string(2, y, buf, false);
        y += 10;
    }
}

/**
 * @brief Render IMPLUVIUM Zone 1 Learning detail
 */
static void hmi_render_impluvium_learning_1_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(24, 2, "Z1 Learning", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Confidence
    snprintf(buf, sizeof(buf), "Conf: %.0f%%", data.zones[0].confidence_level * 100.0f);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // PPMP ratio (use full name as requested)
    snprintf(buf, sizeof(buf), "PulsesPerMoisture%%: %.1f", data.zones[0].calculated_ppmp_ratio);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Pump duty
    snprintf(buf, sizeof(buf), "Pump: %lu", data.zones[0].calculated_pump_duty);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Predictions
    snprintf(buf, sizeof(buf), "Pred: %lu/%lu",
             data.zones[0].successful_predictions,
             data.zones[0].total_predictions);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // History entries
    snprintf(buf, sizeof(buf), "History: %d/15", data.zones[0].history_entry_count);
    hmi_fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 2 Learning detail
 */
static void hmi_render_impluvium_learning_2_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(24, 2, "Z2 Learning", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Conf: %.0f%%", data.zones[1].confidence_level * 100.0f);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "PulsesPerMoisture%%: %.1f", data.zones[1].calculated_ppmp_ratio);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pump: %lu", data.zones[1].calculated_pump_duty);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pred: %lu/%lu",
             data.zones[1].successful_predictions,
             data.zones[1].total_predictions);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "History: %d/15", data.zones[1].history_entry_count);
    hmi_fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 3 Learning detail
 */
static void hmi_render_impluvium_learning_3_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(24, 2, "Z3 Learning", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Conf: %.0f%%", data.zones[2].confidence_level * 100.0f);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "PulsesPerMoisture%%: %.1f", data.zones[2].calculated_ppmp_ratio);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pump: %lu", data.zones[2].calculated_pump_duty);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pred: %lu/%lu",
             data.zones[2].successful_predictions,
             data.zones[2].total_predictions);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "History: %d/15", data.zones[2].history_entry_count);
    hmi_fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 4 Learning detail
 */
static void hmi_render_impluvium_learning_4_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(24, 2, "Z4 Learning", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Conf: %.0f%%", data.zones[3].confidence_level * 100.0f);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "PulsesPerMoisture%%: %.1f", data.zones[3].calculated_ppmp_ratio);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pump: %lu", data.zones[3].calculated_pump_duty);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pred: %lu/%lu",
             data.zones[3].successful_predictions,
             data.zones[3].total_predictions);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "History: %d/15", data.zones[3].history_entry_count);
    hmi_fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 5 Learning detail
 */
static void hmi_render_impluvium_learning_5_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(24, 2, "Z5 Learning", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Conf: %.0f%%", data.zones[4].confidence_level * 100.0f);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "PulsesPerMoisture%%: %.1f", data.zones[4].calculated_ppmp_ratio);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pump: %lu", data.zones[4].calculated_pump_duty);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pred: %lu/%lu",
             data.zones[4].successful_predictions,
             data.zones[4].total_predictions);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "History: %d/15", data.zones[4].history_entry_count);
    hmi_fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM System Monitor detail page [LIVE]
 */
static void hmi_render_impluvium_monitor_page(void)
{
    impluvium_snapshot_t data;
    impluvium_snapshot_rt_t realtime;
    telemetry_get_impluvium_data(&data);
    telemetry_get_impluvium_realtime_data(&realtime);

    // Title with blink indicator
    hmi_fb_draw_string(20, 2, "Sys Monitor", false);
    if (hmi_status.blink_state) {
        hmi_fb_draw_string(80, 2, "*", false);
    }
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // State
    const char *state_str = "STANDBY";
    switch (data.state) {
        case IMPLUVIUM_STANDBY: state_str = "STANDBY"; break;
        case IMPLUVIUM_MEASURING: state_str = "MEASURING"; break;
        case IMPLUVIUM_WATERING: state_str = "WATERING"; break;
        case IMPLUVIUM_STOPPING: state_str = "STOPPING"; break;
        case IMPLUVIUM_MAINTENANCE: state_str = "MAINT!"; break;
        case IMPLUVIUM_DISABLED: state_str = "DISABLED"; break;
    }
    snprintf(buf, sizeof(buf), "%s", state_str);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Pressure & Flow
    snprintf(buf, sizeof(buf), "P: %.1fbar", realtime.outlet_pressure_bar);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "F: %.0fL/h", realtime.current_flow_rate_lh);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Pump
    snprintf(buf, sizeof(buf), "Pump: %lu%%",
             (realtime.pump_pwm_duty * 100) / 1023);
    hmi_fb_draw_string(2, y, buf, false);

    hmi_fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render IMPLUVIUM System Controls page (interactive actions)
 */
static void hmi_render_impluvium_controls_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    // Title
    hmi_fb_draw_string(4, 2, "System Controls", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items
    const char *items[] = {
        "< Back",
        data.load_shed_shutdown ? "Enable System" : "Disable System",
        "Force Check Now",
        data.emergency_stop ? "Emerg Reset!" : "Emerg Reset",
        "Clear Diag",
        "Reset Learning"
    };

    int y = 14;
    for (int i = 0; i < 6; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }
        hmi_fb_draw_string(8, y, items[i], selected);
        y += 10;
    }
}

/**
 * @brief Render IMPLUVIUM Zone Config page (zone list)
 */
static void hmi_render_impluvium_zone_config_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    // Title
    hmi_fb_draw_string(12, 2, "Zone Config", false);
    hmi_fb_draw_hline(0, 10, 128);

    int y = 14;

    // Item 0: Back
    bool selected = (hmi_status.selected_item == 0);
    if (selected) hmi_fb_draw_string(2, y, ">", false);
    hmi_fb_draw_string(8, y, "< Back", selected);
    y += 10;

    // Items 1-5: Zones
    for (uint8_t i = 0; i < 5; i++) {
        selected = (hmi_status.selected_item == i + 1);
        if (selected) hmi_fb_draw_string(2, y, ">", false);

        char buf[32];
        const char *en_marker = data.zones[i].watering_enabled ? "[*]" : "[ ]";
        snprintf(buf, sizeof(buf), "Z%d:%s %.0f%% +/-%.0f%%",
                 i + 1,
                 en_marker,
                 data.zones[i].target_moisture_percent,
                 data.zones[i].moisture_deadband_percent);
        hmi_fb_draw_string(8, y, buf, selected);
        y += 10;
    }
}

/**
 * @brief Render IMPLUVIUM Zone Edit page
 */
static void hmi_render_impluvium_zone_edit_page(void)
{
    // Title
    char title[32];
    snprintf(title, sizeof(title), "Zone %d Config%s", editing_zone_id + 1, zone_editing ? "*" : "");
    hmi_fb_draw_string(12, 2, title, false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Field 0: Cancel
    bool selected = (hmi_status.selected_item == 0);
    if (selected) hmi_fb_draw_string(2, y, ">", false);
    hmi_fb_draw_string(8, y, "< Cancel", selected);
    y += 8;

    // Field 1: Enabled
    selected = (hmi_status.selected_item == 1);
    if (selected) hmi_fb_draw_string(2, y, ">", false);
    const char *en_marker = editing_zone_enabled ? "[*]" : "[ ]";
    snprintf(buf, sizeof(buf), "Enabled: %s", en_marker);
    hmi_fb_draw_string(8, y, buf, selected);
    y += 8;

    // Field 2: Target moisture
    selected = (hmi_status.selected_item == 2);
    if (selected) hmi_fb_draw_string(2, y, ">", false);
    snprintf(buf, sizeof(buf), "Target:  %d%%", (int)editing_zone_target);
    hmi_fb_draw_string(8, y, buf, selected);
    y += 8;

    // Field 3: Deadband
    selected = (hmi_status.selected_item == 3);
    if (selected) hmi_fb_draw_string(2, y, ">", false);
    snprintf(buf, sizeof(buf), "Deadband: %d%%", (int)editing_zone_deadband);
    hmi_fb_draw_string(8, y, buf, selected);
    y += 8;

    // Field 4: Reset Learning
    selected = (hmi_status.selected_item == 4);
    if (selected) hmi_fb_draw_string(2, y, ">", false);
    hmi_fb_draw_string(8, y, "Reset Learning", selected);
    y += 8;

    // Field 5: Manual Water
    selected = (hmi_status.selected_item == 5);
    if (selected) hmi_fb_draw_string(2, y, ">", false);
    hmi_fb_draw_string(8, y, "Manual Water", selected);
    y += 8;

    // Field 6: Save
    selected = (hmi_status.selected_item == 6);
    if (selected) hmi_fb_draw_string(2, y, ">", false);
    hmi_fb_draw_string(8, y, "< Save", selected);
}

/**
 * @brief Render IMPLUVIUM Manual Water Time Input page
 */
static void hmi_render_impluvium_manual_water_page(void)
{
    // Title
    char title[32];
    snprintf(title, sizeof(title), "Manual Water Z%d*", manual_water_zone + 1);
    hmi_fb_draw_string(4, 2, title, false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 20;

    // Field 0: Duration
    bool selected = (hmi_status.selected_item == 0);
    if (selected) hmi_fb_draw_string(2, y, ">", false);
    snprintf(buf, sizeof(buf), "Duration: %ds", manual_water_duration);
    hmi_fb_draw_string(8, y, buf, selected);
    y += 12;

    // Field 1: Start
    selected = (hmi_status.selected_item == 1);
    if (selected) hmi_fb_draw_string(2, y, ">", false);
    hmi_fb_draw_string(8, y, "Start", selected);
    y += 12;

    // Field 2: Cancel
    selected = (hmi_status.selected_item == 2);
    if (selected) hmi_fb_draw_string(2, y, ">", false);
    hmi_fb_draw_string(8, y, "< Cancel", selected);

    // Footer instruction
    hmi_fb_draw_string(2, 56, "Rotate:adjust", false);
}

/**
 * @brief Render IMPLUVIUM Intervals configuration page
 */
static void hmi_render_impluvium_intervals_page(void)
{
    // Title
    hmi_fb_draw_string(16, 2, "IMPLUVIUM Int", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items with current values
    char buf[32];
    const char *items[5];
    items[0] = "< Back";

    // Optimal interval (item 1)
    if (interval_editing && hmi_status.selected_item == 1) {
        snprintf(buf, sizeof(buf), "Optimal: %lum*", editing_interval_value);
    } else {
        snprintf(buf, sizeof(buf), "Optimal: %lum", g_interval_config.impluvium_optimal_min);
    }
    items[1] = buf;

    // Cool interval (item 2)
    char buf2[32];
    if (interval_editing && hmi_status.selected_item == 2) {
        snprintf(buf2, sizeof(buf2), "Cool: %lum*", editing_interval_value);
    } else {
        snprintf(buf2, sizeof(buf2), "Cool: %lum", g_interval_config.impluvium_cool_min);
    }
    items[2] = buf2;

    // Power Save interval (item 3)
    char buf3[32];
    if (interval_editing && hmi_status.selected_item == 3) {
        snprintf(buf3, sizeof(buf3), "Pwr Save: %lum*", editing_interval_value);
    } else {
        snprintf(buf3, sizeof(buf3), "Pwr Save: %lum", g_interval_config.impluvium_power_save_min);
    }
    items[3] = buf3;

    // Night Min interval (item 4)
    char buf4[32];
    if (interval_editing && hmi_status.selected_item == 4) {
        snprintf(buf4, sizeof(buf4), "Night: %luh*", editing_interval_value);
    } else {
        snprintf(buf4, sizeof(buf4), "Night: %luh", g_interval_config.impluvium_night_min_hours);
    }
    items[4] = buf4;

    // Render menu items
    int y = 14;
    for (int i = 0; i < 5; i++) {
        bool selected = (i == hmi_status.selected_item);

        // Selection indicator
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }

        // Item text
        hmi_fb_draw_string(8, y, items[i], selected);

        y += 10;  // Tighter spacing for 5 items
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
 * @brief IMPLUVIUM rendering dispatcher
 * Routes to appropriate IMPLUVIUM page renderer based on current menu state
 */
void hmi_render_impluvium_pages(void)
{
    switch (hmi_status.current_menu) {
        case HMI_MENU_IMPLUVIUM:
            hmi_render_impluvium_menu();
            break;
        case HMI_MENU_IMPLUVIUM_OVERVIEW:
            hmi_render_impluvium_overview_page();
            break;
        case HMI_MENU_IMPLUVIUM_STATISTICS:
            hmi_render_impluvium_statistics_page();
            break;
        case HMI_MENU_IMPLUVIUM_ZONES:
            hmi_render_impluvium_zones_menu();
            break;
        case HMI_MENU_IMPLUVIUM_ZONES_ALL:
            hmi_render_impluvium_zones_all_page();
            break;
        case HMI_MENU_IMPLUVIUM_ZONE_1:
            hmi_render_impluvium_zone_1_page();
            break;
        case HMI_MENU_IMPLUVIUM_ZONE_2:
            hmi_render_impluvium_zone_2_page();
            break;
        case HMI_MENU_IMPLUVIUM_ZONE_3:
            hmi_render_impluvium_zone_3_page();
            break;
        case HMI_MENU_IMPLUVIUM_ZONE_4:
            hmi_render_impluvium_zone_4_page();
            break;
        case HMI_MENU_IMPLUVIUM_ZONE_5:
            hmi_render_impluvium_zone_5_page();
            break;
        case HMI_MENU_IMPLUVIUM_LEARNING:
            hmi_render_impluvium_learning_menu();
            break;
        case HMI_MENU_IMPLUVIUM_LEARNING_ALL:
            hmi_render_impluvium_learning_all_page();
            break;
        case HMI_MENU_IMPLUVIUM_LEARNING_1:
            hmi_render_impluvium_learning_1_page();
            break;
        case HMI_MENU_IMPLUVIUM_LEARNING_2:
            hmi_render_impluvium_learning_2_page();
            break;
        case HMI_MENU_IMPLUVIUM_LEARNING_3:
            hmi_render_impluvium_learning_3_page();
            break;
        case HMI_MENU_IMPLUVIUM_LEARNING_4:
            hmi_render_impluvium_learning_4_page();
            break;
        case HMI_MENU_IMPLUVIUM_LEARNING_5:
            hmi_render_impluvium_learning_5_page();
            break;
        case HMI_MENU_IMPLUVIUM_MONITOR:
            hmi_render_impluvium_monitor_page();
            break;
        case HMI_MENU_IMPLUVIUM_CONTROLS:
            hmi_render_impluvium_controls_page();
            break;
        case HMI_MENU_IMPLUVIUM_ZONE_CONFIG:
            hmi_render_impluvium_zone_config_page();
            break;
        case HMI_MENU_IMPLUVIUM_ZONE_EDIT:
            hmi_render_impluvium_zone_edit_page();
            break;
        case HMI_MENU_IMPLUVIUM_MANUAL_WATER:
            hmi_render_impluvium_manual_water_page();
            break;
        case HMI_MENU_IMPLUVIUM_INTERVALS:
            hmi_render_impluvium_intervals_page();
            break;
        default:
            break;
    }
}
