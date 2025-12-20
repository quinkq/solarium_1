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
 * @brief Render IMPLUVIUM main menu (dispatcher) with scrolling support
 * Supports 9 items with 7-item visible window
 */
static void hmi_render_impluvium_menu(void)
{
    const uint8_t TOTAL_ITEMS = 8;
    const char *items[] = {
        "< Back",
        "Overview/Stats",  // Paginated: Overview, Stats Z1-3, Stats Z4-5
        "Sys Monitor",     // 4Hz REALTIME (moved up)
        "Zones",           // Submenu
        "Zone Config",     // (moved from #8)
        "Learning",        // Submenu
        "System Controls",
        "Intervals"
    };

    // Get data from TELEMETRY cache (for [LIVE] indicator)
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    // Title
    hmi_fb_draw_string(24, 2, "IMPLUVIUM", false);
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

        // Add [LIVE] indicator for Sys Monitor (item 2)
        if (i == 2) {
            if ((data.state == IMPLUVIUM_WATERING || data.state == IMPLUVIUM_MEASURING) &&
                hmi_status.blink_state) {
                hmi_fb_draw_string(70, y, "[LIVE]", false);
            }
        }

        y += HMI_MENU_ITEM_SPACING;
    }
}

/**
 * @brief Render IMPLUVIUM Overview/Stats merged page [1Hz] with pagination
 * Combines Overview and Statistics across 3 pages:
 * - Page 1/3: Overview (State, Tank, Water Used, Events, Active Zone)
 * - Page 2/3: Statistics Zones 1-3
 * - Page 3/3: Statistics Zones 4-5 + Totals
 */
static void hmi_render_impluvium_overview_stats_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    // Set pagination state (3 pages total)
    hmi_status.total_pages = 3;

    // Navigation symbols
    hmi_draw_back_symbol();
    hmi_draw_pagination_symbol();

    // Title with page indicator
    char title[24];
    snprintf(title, sizeof(title), "Overview %d/%d",
             hmi_status.current_page + 1, hmi_status.total_pages);
    hmi_fb_draw_string(18, 2, title, false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    switch (hmi_status.current_page) {
        case 0:  // PAGE 1: Overview
            {
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
                snprintf(buf, sizeof(buf), "Tank lvl: %.0f%%", data.water_level_percent);
                hmi_fb_draw_string(2, y, buf, false);
                y += 10;

                // Total water used today
                snprintf(buf, sizeof(buf), "H20 used: %.1fL", data.total_water_used_day_ml / 1000.0f);
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
            break;

        case 1:  // PAGE 2: Stats Zones 1-3
            // Header
            hmi_fb_draw_string(2, y, "Hr(l) Avg(l/h) Day(l)", false);
            y += 10;

            // Zones 1-3
            for (int i = 0; i < 3; i++) {
                float vol_hour_l = data.zones[i].volume_used_hour_ml / 1000.0f;
                float avg_hour_l = data.zones[i].avg_hourly_consumption_ml / 1000.0f;
                float vol_day_l = data.zones[i].volume_used_today_ml / 1000.0f;

                snprintf(buf, sizeof(buf), "Z%d: %.1f %.1f %.1f",
                         i + 1, vol_hour_l, avg_hour_l, vol_day_l);
                hmi_fb_draw_string(2, y, buf, false);
                y += 10;
            }
            break;

        case 2:  // PAGE 3: Stats Zones 4-5 + Totals
            // Header
            hmi_fb_draw_string(2, y, "Hr(l) Avg(l/h) Day(l)", false);
            y += 10;

            // Zones 4-5
            for (int i = 3; i < 5; i++) {
                float vol_hour_l = data.zones[i].volume_used_hour_ml / 1000.0f;
                float avg_hour_l = data.zones[i].avg_hourly_consumption_ml / 1000.0f;
                float vol_day_l = data.zones[i].volume_used_today_ml / 1000.0f;

                snprintf(buf, sizeof(buf), "Z%d: %.1f %.1f %.1f",
                         i + 1, vol_hour_l, avg_hour_l, vol_day_l);
                hmi_fb_draw_string(2, y, buf, false);
                y += 10;
            }

            y += 2; // Extra spacing before totals

            // System totals
            float total_hour = 0.0f;
            float total_day = 0.0f;
            for (int i = 0; i < 5; i++) {
                total_hour += data.zones[i].volume_used_hour_ml / 1000.0f;
                total_day += data.zones[i].volume_used_today_ml / 1000.0f;
            }

            snprintf(buf, sizeof(buf), "Total: %.1f --- %.1f", total_hour, total_day);
            hmi_fb_draw_string(2, y, buf, false);
            break;
    }
}

/**
 * @brief Render IMPLUVIUM Zones submenu selector with scrolling support
 * Supports 7 items - exactly fits in 7-item window, but uses scrolling for consistency
 */
static void hmi_render_impluvium_zones_menu(void)
{
    const uint8_t TOTAL_ITEMS = 7;
    const char *items[] = {
        "< Back",
        "All Zones",
        "Zone 0",
        "Zone 1",
        "Zone 2",
        "Zone 3",
        "Zone 4"
    };

    hmi_fb_draw_string(28, 2, "Zones", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Draw scroll indicators (none will show since 7 items exactly fit)
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
        const char *enabled_str = data.zones[i].watering_enabled ? "En" : "Dis";
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
 * @brief Render IMPLUVIUM Zone 0 detail page
 */
static void hmi_render_impluvium_zone_1_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(40, 2, "Zone 0", false);
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
 * @brief Render IMPLUVIUM Zone 1 detail page
 */
static void hmi_render_impluvium_zone_2_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(40, 2, "Zone 1", false);
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
 * @brief Render IMPLUVIUM Zone 2 detail page
 */
static void hmi_render_impluvium_zone_3_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(40, 2, "Zone 2", false);
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
 * @brief Render IMPLUVIUM Zone 3 detail page
 */
static void hmi_render_impluvium_zone_4_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(40, 2, "Zone 3", false);
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
 * @brief Render IMPLUVIUM Zone 4 detail page
 */
static void hmi_render_impluvium_zone_5_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(40, 2, "Zone 4", false);
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
 * @brief Render IMPLUVIUM Learning submenu selector with scrolling support
 * Supports 7 items - exactly fits in 7-item window, but uses scrolling for consistency
 */
static void hmi_render_impluvium_learning_menu(void)
{
    const uint8_t TOTAL_ITEMS = 7;
    const char *items[] = {
        "< Back",
        "All Zones",
        "Zone 0",
        "Zone 1",
        "Zone 2",
        "Zone 3",
        "Zone 4"
    };

    hmi_fb_draw_string(24, 2, "Learning", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Draw scroll indicators (none will show since 7 items exactly fit)
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
 * @brief Render IMPLUVIUM Learning Summary (all zones)
 */
static void hmi_render_impluvium_learning_all_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(10, 2, "Summary (confid/ppm%%)", false);
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
 * @brief Render IMPLUVIUM Zone 0 Learning detail
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
    snprintf(buf, sizeof(buf), "Confidence: %.0f%%", data.zones[0].confidence_level * 100.0f);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // PPMP ratio (use full name as requested)
    snprintf(buf, sizeof(buf), "PPM%%: %.1f", data.zones[0].calculated_ppmp_ratio);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Pump duty
    snprintf(buf, sizeof(buf), "Pump duty: %lu", data.zones[0].calculated_pump_duty);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Predictions
    snprintf(buf, sizeof(buf), "Predictions: %lu/%lu",
             data.zones[0].successful_predictions,
             data.zones[0].total_predictions);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // History entries
    snprintf(buf, sizeof(buf), "History: %d/15", data.zones[0].history_entry_count);
    hmi_fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 1 Learning detail
 */
static void hmi_render_impluvium_learning_2_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(24, 2, "Z2 Learning", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Confidence: %.0f%%", data.zones[1].confidence_level * 100.0f);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "PPM%%: %.1f", data.zones[1].calculated_ppmp_ratio);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pump duty: %lu", data.zones[1].calculated_pump_duty);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Predictions: %lu/%lu",
             data.zones[1].successful_predictions,
             data.zones[1].total_predictions);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "History: %d/15", data.zones[1].history_entry_count);
    hmi_fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 2 Learning detail
 */
static void hmi_render_impluvium_learning_3_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(24, 2, "Z3 Learning", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Confidence: %.0f%%", data.zones[2].confidence_level * 100.0f);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "PPM%%: %.1f", data.zones[2].calculated_ppmp_ratio);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pump duty: %lu", data.zones[2].calculated_pump_duty);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Predictions: %lu/%lu",
             data.zones[2].successful_predictions,
             data.zones[2].total_predictions);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "History: %d/15", data.zones[2].history_entry_count);
    hmi_fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 3 Learning detail
 */
static void hmi_render_impluvium_learning_4_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(24, 2, "Z4 Learning", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Confidence: %.0f%%", data.zones[3].confidence_level * 100.0f);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "PPM%%: %.1f", data.zones[3].calculated_ppmp_ratio);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pump duty: %lu", data.zones[3].calculated_pump_duty);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Predictions: %lu/%lu",
             data.zones[3].successful_predictions,
             data.zones[3].total_predictions);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "History: %d/15", data.zones[3].history_entry_count);
    hmi_fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 4 Learning detail
 */
static void hmi_render_impluvium_learning_5_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    hmi_fb_draw_string(24, 2, "Z5 Learning", false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Confidence: %.0f%%", data.zones[4].confidence_level * 100.0f);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "PPM%%: %.1f", data.zones[4].calculated_ppmp_ratio);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pump duty: %lu", data.zones[4].calculated_pump_duty);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Predictions: %lu/%lu",
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
        hmi_fb_draw_string(100, 2, "*", false);
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
    snprintf(buf, sizeof(buf), "Pressure: %.1fbar", realtime.outlet_pressure_bar);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Flow: %.0fL/h", realtime.current_flow_rate_lh);
    hmi_fb_draw_string(2, y, buf, false);
    y += 10;

    // Pump
    snprintf(buf, sizeof(buf), "Pump%%: %lu%%",
             (realtime.pump_pwm_duty * 100) / 1023);
    hmi_fb_draw_string(2, y, buf, false);

    hmi_fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render IMPLUVIUM System Controls page (interactive actions) with scrolling
 * Supports 6 items - fits comfortably in 7-item window
 */
static void hmi_render_impluvium_controls_page(void)
{
    const uint8_t TOTAL_ITEMS = 6;
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    // Get current state directly (not from cache) for accurate toggle display
    impluvium_state_t current_state = impluvium_get_state();
    bool is_disabled = (current_state == IMPLUVIUM_DISABLED);

    // Build dynamic menu items
    const char *items[TOTAL_ITEMS];
    items[0] = "< Back";
    items[1] = is_disabled ? "Enable System" : "Disable System";
    items[2] = "Force Check Now";
    items[3] = data.emergency_stop ? "Emerg Reset!" : "Emerg Reset";
    items[4] = "Clear Diag";
    items[5] = "Reset Learning";

    // Title
    hmi_fb_draw_string(4, 2, "System Controls", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Draw scroll indicators (none will show since 6 items fit easily)
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
 * @brief Render IMPLUVIUM Zone Config page (zone list) with scrolling
 * Supports 7 items - exactly fits in 7-item window
 */
static void hmi_render_impluvium_zone_config_page(void)
{
    const uint8_t TOTAL_ITEMS = 7;
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    // Title
    hmi_fb_draw_string(12, 2, "Zone Config", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Draw scroll indicators (none will show since 7 items fit exactly)
    hmi_draw_scroll_indicators(TOTAL_ITEMS);

    // Calculate visible item range
    uint8_t first_visible = hmi_status.scroll_offset;
    uint8_t last_visible = first_visible + HMI_MAX_VISIBLE_ITEMS;
    if (last_visible > TOTAL_ITEMS) {
        last_visible = TOTAL_ITEMS;
    }

    // Count enabled zones for "All" toggle display
    uint8_t enabled_count = 0;
    for (int i = 0; i < 5; i++) {
        if (data.zones[i].watering_enabled) enabled_count++;
    }
    bool all_enabled = (enabled_count == 5);
    bool any_enabled = (enabled_count > 0);

    // Draw visible items only
    int y = HMI_MENU_START_Y;
    for (uint8_t i = first_visible; i < last_visible; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }

        if (i == 0) {
            // Item 0: Back
            hmi_fb_draw_string(8, y, "< Back", selected);
        } else if (i >= 1 && i <= 5) {
            // Items 1-5: Individual Zones
            uint8_t zone_idx = i - 1;
            char buf[32];
            const char *en_marker = data.zones[zone_idx].watering_enabled ? "[*]" : "[ ]";
            snprintf(buf, sizeof(buf), "Z%d:%s %.0f%% +/-%.0f%%",
                     zone_idx + 1,
                     en_marker,
                     data.zones[zone_idx].target_moisture_percent,
                     data.zones[zone_idx].moisture_deadband_percent);
            hmi_fb_draw_string(8, y, buf, selected);
        } else if (i == 6) {
            // Item 6: Enable/Disable All Zones
            const char *toggle_text = all_enabled ? "Disable All" :
                                     (any_enabled ? "Enable All" : "Enable All");
            hmi_fb_draw_string(8, y, toggle_text, selected);
        }

        y += HMI_MENU_ITEM_SPACING;
    }
}

/**
 * @brief Render IMPLUVIUM Zone Edit page with scrolling
 * Supports 7 items - exactly fits in 7-item window
 */
static void hmi_render_impluvium_zone_edit_page(void)
{
    const uint8_t TOTAL_ITEMS = 7;

    // Title
    char title[32];
    snprintf(title, sizeof(title), "Zone %d Config%s", editing_zone_id + 1, zone_editing ? "*" : "");
    hmi_fb_draw_string(12, 2, title, false);
    hmi_fb_draw_hline(0, 10, 128);

    // Draw scroll indicators (none will show since 7 items exactly fit)
    hmi_draw_scroll_indicators(TOTAL_ITEMS);

    // Calculate visible item range
    uint8_t first_visible = hmi_status.scroll_offset;
    uint8_t last_visible = first_visible + HMI_MAX_VISIBLE_ITEMS;
    if (last_visible > TOTAL_ITEMS) {
        last_visible = TOTAL_ITEMS;
    }

    // Draw visible items only
    int y = HMI_MENU_START_Y;
    char buf[32];
    for (uint8_t i = first_visible; i < last_visible; i++) {
        bool selected = (i == hmi_status.selected_item);
        bool is_editable = (i == 2 || i == 3);  // Target and Deadband are editable

        // Cursor logic: always blink on editable fields when selected
        if (selected && is_editable) {
            if (hmi_status.blink_state) {
                hmi_fb_draw_string(2, y, ">", false);
            }
        } else if (selected) {
            hmi_fb_draw_string(2, y, ">", false);
        }

        switch (i) {
            case 0:  // Cancel
                hmi_fb_draw_string(8, y, "< Cancel", selected);
                break;
            case 1:  // Enabled
                {
                    const char *en_marker = editing_zone_enabled ? "[*]" : "[ ]";
                    snprintf(buf, sizeof(buf), "Enabled: %s", en_marker);
                    hmi_fb_draw_string(8, y, buf, selected);
                }
                break;
            case 2:  // Target moisture
                {
                    bool editing_this = (zone_editing && i == hmi_status.selected_item);
                    snprintf(buf, sizeof(buf), "Target:  %d%%", (int)editing_zone_target);
                    hmi_fb_draw_string(8, y, buf, selected);
                    // Draw blinking "*" separately (not inverted)
                    if (editing_this && hmi_status.blink_state) {
                        hmi_fb_draw_string(104, y, "*", false);
                    }
                }
                break;
            case 3:  // Deadband
                {
                    bool editing_this = (zone_editing && i == hmi_status.selected_item);
                    snprintf(buf, sizeof(buf), "Deadband: %d%%", (int)editing_zone_deadband);
                    hmi_fb_draw_string(8, y, buf, selected);
                    // Draw blinking "*" separately (not inverted)
                    if (editing_this && hmi_status.blink_state) {
                        hmi_fb_draw_string(104, y, "*", false);
                    }
                }
                break;
            case 4:  // Reset Learning
                hmi_fb_draw_string(8, y, "Reset Learning", selected);
                break;
            case 5:  // Manual Water
                hmi_fb_draw_string(8, y, "Manual Water", selected);
                break;
            case 6:  // Save
                hmi_fb_draw_string(8, y, "< Save", selected);
                break;
        }

        y += HMI_MENU_ITEM_SPACING;
    }
}

/**
 * @brief Render IMPLUVIUM Manual Water Time Input page
 */
static void hmi_render_impluvium_manual_water_page(void)
{
    // Navigation symbols
    hmi_draw_back_symbol();

    // Title
    char title[32];
    snprintf(title, sizeof(title), "Manual Water Z%d*", manual_water_zone + 1);
    hmi_fb_draw_string(4, 2, title, false);
    hmi_fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 20;

    // Field 0: Duration (editable)
    bool selected = (hmi_status.selected_item == 0);
    if (selected && hmi_status.blink_state) {
        hmi_fb_draw_string(2, y, ">", false);
    }
    if (manual_water_input) {
        snprintf(buf, sizeof(buf), "Duration: %ds", manual_water_duration);
    } else {
        snprintf(buf, sizeof(buf), "Duration: %ds", manual_water_duration);
    }
    hmi_fb_draw_string(8, y, buf, selected);
    // Draw blinking "*" separately (not inverted)
    if (manual_water_input && hmi_status.blink_state) {
        hmi_fb_draw_string(104, y, "*", false);
    }
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
}

/**
 * @brief Render IMPLUVIUM Intervals configuration page
 */
static void hmi_render_impluvium_intervals_page(void)
{
    // Navigation symbols
    hmi_draw_back_symbol();

    // Title
    hmi_fb_draw_string(16, 2, "IMPLUVIUM Int", false);
    hmi_fb_draw_hline(0, 10, 128);

    // Menu items with current values
    char buf[32];
    const char *items[5];
    items[0] = "< Back";

    // Optimal interval (item 1)
    bool editing_optimal = (interval_editing && hmi_status.selected_item == 1);
    if (editing_optimal) {
        snprintf(buf, sizeof(buf), "Optimal: %lum", editing_interval_value);
    } else {
        snprintf(buf, sizeof(buf), "Optimal: %lum", g_interval_config.impluvium_optimal_min);
    }
    items[1] = buf;

    // Cool interval (item 2)
    char buf2[32];
    bool editing_cool = (interval_editing && hmi_status.selected_item == 2);
    if (editing_cool) {
        snprintf(buf2, sizeof(buf2), "Cool: %lum", editing_interval_value);
    } else {
        snprintf(buf2, sizeof(buf2), "Cool: %lum", g_interval_config.impluvium_cool_min);
    }
    items[2] = buf2;

    // Power Save interval (item 3)
    char buf3[32];
    bool editing_pwr = (interval_editing && hmi_status.selected_item == 3);
    if (editing_pwr) {
        snprintf(buf3, sizeof(buf3), "Pwr Save: %lum", editing_interval_value);
    } else {
        snprintf(buf3, sizeof(buf3), "Pwr Save: %lum", g_interval_config.impluvium_power_save_min);
    }
    items[3] = buf3;

    // Night Min interval (item 4)
    char buf4[32];
    bool editing_night = (interval_editing && hmi_status.selected_item == 4);
    if (editing_night) {
        snprintf(buf4, sizeof(buf4), "Night: %luh", editing_interval_value);
    } else {
        snprintf(buf4, sizeof(buf4), "Night: %luh", g_interval_config.impluvium_night_min_hours);
    }
    items[4] = buf4;

    // Render menu items
    int y = 14;
    for (int i = 0; i < 5; i++) {
        bool selected = (i == hmi_status.selected_item);
        bool is_editable = (i >= 1 && i <= 4);

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

        y += 10;  // Tighter spacing for 5 items
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
        case HMI_MENU_IMPLUVIUM_OVERVIEW_STATS:
            hmi_render_impluvium_overview_stats_page();
            break;
        case HMI_MENU_IMPLUVIUM_MONITOR:
            hmi_render_impluvium_monitor_page();
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
