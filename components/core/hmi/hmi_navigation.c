/**
 * @file hmi_navigation.c
 * @brief HMI Component - Table-Driven State Machine Implementation
 *
 * This file implements the navigation system using declarative transition tables
 * instead of a massive switch statement. This improves maintainability, clarity,
 * and makes it easy to add/modify menu navigation.
 *
 * Structure:
 * - Action handler function declarations (~50 functions)
 * - Transition tables (one per menu)
 * - Master metadata table (maps menu state → metadata)
 * - Navigation engine (lookup + dispatch)
 */

#include "hmi_private.h"
#include "hmi_navigation.h"

#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "HMI_NAV";

// ########################## Forward Declarations - Action Handlers ##########################

// FLUCTUS Actions
static void hmi_action_toggle_solar_tracking(void);
static void hmi_action_manual_safety_reset(void);
static void hmi_action_fluctus_intervals_back(void);
static void hmi_action_edit_fluctus_power_day(void);
static void hmi_action_edit_fluctus_power_night(void);
static void hmi_action_edit_fluctus_solar_correction(void);
static void hmi_action_toggle_solar_debug_mode(void);
static void hmi_action_enter_servo_control_yaw(void);
static void hmi_action_enter_servo_control_pitch(void);
static void hmi_action_exit_servo_control(void);

// TEMPESTA Actions
static void hmi_action_toggle_tempesta(void);
static void hmi_action_force_tempesta_collection(void);
static void hmi_action_enter_tempesta_diag(void);
static void hmi_action_exit_tempesta_diag(void);
static void hmi_action_confirm_tempesta_reset_daily(void);
static void hmi_action_confirm_tempesta_reset_weekly(void);
static void hmi_action_confirm_tempesta_reset_rain(void);
static void hmi_action_confirm_tempesta_reset_tank(void);
static void hmi_action_confirm_tempesta_reset_all(void);
static void hmi_action_tempesta_intervals_back(void);
static void hmi_action_edit_tempesta_normal(void);
static void hmi_action_edit_tempesta_power_save(void);

// IMPLUVIUM Actions
static void hmi_action_toggle_impluvium(void);
static void hmi_action_force_moisture_check(void);
static void hmi_action_emergency_stop_reset(void);
static void hmi_action_clear_diagnostics(void);
static void hmi_action_confirm_reset_all_learning(void);
static void hmi_action_enter_zone_edit(void);  // Generic for zones 1-5
static void hmi_action_toggle_all_zones(void);  // Enable/Disable All Zones
static void hmi_action_zone_edit_cancel(void);
static void hmi_action_zone_edit_toggle_enabled(void);
static void hmi_action_zone_edit_toggle_target(void);
static void hmi_action_zone_edit_toggle_deadband(void);
static void hmi_action_confirm_zone_reset_learning(void);
static void hmi_action_zone_edit_manual_water(void);
static void hmi_action_zone_edit_save(void);
static void hmi_action_manual_water_toggle_duration(void);
static void hmi_action_confirm_manual_water(void);
static void hmi_action_manual_water_cancel(void);
static void hmi_action_impluvium_intervals_back(void);
static void hmi_action_edit_impluvium_optimal(void);
static void hmi_action_edit_impluvium_cool(void);
static void hmi_action_edit_impluvium_power_save(void);
static void hmi_action_edit_impluvium_night_min(void);

// STELLARIA Actions
static void hmi_action_toggle_stellaria(void);
static void hmi_action_toggle_stellaria_intensity(void);
static void hmi_action_toggle_auto_mode(void);

// SYSTEM Actions
static void hmi_action_confirm_flush_reset(void);
static void hmi_action_wifi_reconnect(void);
static void hmi_action_apply_aggressive_preset(void);
static void hmi_action_apply_balanced_preset(void);
static void hmi_action_apply_conservative_preset(void);

// Confirmation Actions
static void hmi_action_confirm_yes(void);
static void hmi_action_confirm_no(void);

// ########################## Transition Tables ##########################

// Main Menu
static const nav_transition_t main_menu_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_FLUCTUS,   .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_TEMPESTA,  .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 2, .next_menu = HMI_MENU_IMPLUVIUM, .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 3, .next_menu = HMI_MENU_STELLARIA, .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 4, .next_menu = HMI_MENU_SYSTEM,    .action = NAV_ACTION_NAVIGATE, .handler = NULL },
};

// FLUCTUS Menus
static const nav_transition_t fluctus_menu_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_MAIN,                  .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_FLUCTUS_OVERVIEW,      .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 2, .next_menu = HMI_MENU_FLUCTUS_ENERGY,        .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 3, .next_menu = HMI_MENU_FLUCTUS_LIVE_POWER,    .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 4, .next_menu = HMI_MENU_FLUCTUS_BUSES,         .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 5, .next_menu = HMI_MENU_FLUCTUS_SOLAR,         .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 6, .next_menu = HMI_MENU_FLUCTUS_SERVO_DEBUG,   .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 7, .next_menu = HMI_MENU_FLUCTUS_CONTROLS,      .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 8, .next_menu = HMI_MENU_FLUCTUS_INTERVALS,     .action = NAV_ACTION_NAVIGATE, .handler = NULL },
};

static const nav_transition_t fluctus_detail_page_transitions[] = {
    // All detail pages return to parent on any button press
    { .item_index = 0xFF, .next_menu = HMI_MENU_FLUCTUS, .action = NAV_ACTION_NAVIGATE, .handler = NULL },
};

static const nav_transition_t fluctus_solar_transitions[] = {
    // Page 4 has interactive controls
    // Item 0 (< Back) navigates back, Item 1 (Debug toggle) triggers action
    { .item_index = 0, .next_menu = HMI_MENU_FLUCTUS,       .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_FLUCTUS_SOLAR, .action = NAV_ACTION_TOGGLE,   .handler = hmi_action_toggle_solar_debug_mode },
};

static const nav_transition_t fluctus_controls_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_FLUCTUS,         .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_FLUCTUS_CONTROLS,.action = NAV_ACTION_TOGGLE,   .handler = hmi_action_toggle_solar_tracking },
    { .item_index = 2, .next_menu = HMI_MENU_FLUCTUS_CONTROLS,.action = NAV_ACTION_EXECUTE,  .handler = hmi_action_manual_safety_reset },
};

static const nav_transition_t fluctus_intervals_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_FLUCTUS,          .action = NAV_ACTION_EXECUTE,     .handler = hmi_action_fluctus_intervals_back },
    { .item_index = 1, .next_menu = HMI_MENU_FLUCTUS_INTERVALS,.action = NAV_ACTION_EDIT_START,  .handler = hmi_action_edit_fluctus_power_day },
    { .item_index = 2, .next_menu = HMI_MENU_FLUCTUS_INTERVALS,.action = NAV_ACTION_EDIT_START,  .handler = hmi_action_edit_fluctus_power_night },
    { .item_index = 3, .next_menu = HMI_MENU_FLUCTUS_INTERVALS,.action = NAV_ACTION_EDIT_START,  .handler = hmi_action_edit_fluctus_solar_correction },
};

static const nav_transition_t fluctus_servo_debug_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_FLUCTUS,                     .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_FLUCTUS_SERVO_CONTROL_YAW,   .action = NAV_ACTION_EXECUTE,  .handler = hmi_action_enter_servo_control_yaw },
    { .item_index = 2, .next_menu = HMI_MENU_FLUCTUS_SERVO_CONTROL_PITCH, .action = NAV_ACTION_EXECUTE,  .handler = hmi_action_enter_servo_control_pitch },
};

static const nav_transition_t fluctus_servo_control_transitions[] = {
    // Button press exits servo control mode (handler cleans up power, returns to servo debug menu)
    { .item_index = 0xFF, .next_menu = HMI_MENU_FLUCTUS_SERVO_DEBUG, .action = NAV_ACTION_EXECUTE, .handler = hmi_action_exit_servo_control },
};

// TEMPESTA Menus
static const nav_transition_t tempesta_menu_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_MAIN,              .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_TEMPESTA_SENSORS,  .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 2, .next_menu = HMI_MENU_TEMPESTA_DIAG,     .action = NAV_ACTION_EXECUTE,  .handler = hmi_action_enter_tempesta_diag },
    { .item_index = 3, .next_menu = HMI_MENU_TEMPESTA_CONTROLS, .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 4, .next_menu = HMI_MENU_TEMPESTA_INTERVALS,.action = NAV_ACTION_NAVIGATE, .handler = NULL },
};

static const nav_transition_t tempesta_detail_page_transitions[] = {
    { .item_index = 0xFF, .next_menu = HMI_MENU_TEMPESTA, .action = NAV_ACTION_NAVIGATE, .handler = NULL },
};

static const nav_transition_t tempesta_diag_page_transitions[] = {
    { .item_index = 0xFF, .next_menu = HMI_MENU_TEMPESTA, .action = NAV_ACTION_EXECUTE, .handler = hmi_action_exit_tempesta_diag },
};

static const nav_transition_t tempesta_controls_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_TEMPESTA,         .action = NAV_ACTION_NAVIGATE,      .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_TEMPESTA_CONTROLS,.action = NAV_ACTION_TOGGLE,        .handler = hmi_action_toggle_tempesta },
    { .item_index = 2, .next_menu = HMI_MENU_TEMPESTA_CONTROLS,.action = NAV_ACTION_EXECUTE,       .handler = hmi_action_force_tempesta_collection },
    { .item_index = 3, .next_menu = HMI_MENU_CONFIRM,          .action = NAV_ACTION_CONFIRM_DIALOG,.handler = hmi_action_confirm_tempesta_reset_daily },
    { .item_index = 4, .next_menu = HMI_MENU_CONFIRM,          .action = NAV_ACTION_CONFIRM_DIALOG,.handler = hmi_action_confirm_tempesta_reset_weekly },
    { .item_index = 5, .next_menu = HMI_MENU_CONFIRM,          .action = NAV_ACTION_CONFIRM_DIALOG,.handler = hmi_action_confirm_tempesta_reset_rain },
    { .item_index = 6, .next_menu = HMI_MENU_CONFIRM,          .action = NAV_ACTION_CONFIRM_DIALOG,.handler = hmi_action_confirm_tempesta_reset_tank },
    { .item_index = 7, .next_menu = HMI_MENU_CONFIRM,          .action = NAV_ACTION_CONFIRM_DIALOG,.handler = hmi_action_confirm_tempesta_reset_all },
};

static const nav_transition_t tempesta_intervals_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_TEMPESTA,          .action = NAV_ACTION_EXECUTE,    .handler = hmi_action_tempesta_intervals_back },
    { .item_index = 1, .next_menu = HMI_MENU_TEMPESTA_INTERVALS,.action = NAV_ACTION_EDIT_START, .handler = hmi_action_edit_tempesta_normal },
    { .item_index = 2, .next_menu = HMI_MENU_TEMPESTA_INTERVALS,.action = NAV_ACTION_EDIT_START, .handler = hmi_action_edit_tempesta_power_save },
};

// IMPLUVIUM Menus
static const nav_transition_t impluvium_menu_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_MAIN,                     .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_IMPLUVIUM_OVERVIEW_STATS, .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 2, .next_menu = HMI_MENU_IMPLUVIUM_MONITOR,        .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 3, .next_menu = HMI_MENU_IMPLUVIUM_ZONES,          .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 4, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_CONFIG,    .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 5, .next_menu = HMI_MENU_IMPLUVIUM_LEARNING,       .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 6, .next_menu = HMI_MENU_IMPLUVIUM_CONTROLS,       .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 7, .next_menu = HMI_MENU_IMPLUVIUM_INTERVALS,      .action = NAV_ACTION_NAVIGATE, .handler = NULL },
};

static const nav_transition_t impluvium_detail_page_transitions[] = {
    { .item_index = 0xFF, .next_menu = HMI_MENU_IMPLUVIUM, .action = NAV_ACTION_NAVIGATE, .handler = NULL },
};

static const nav_transition_t impluvium_zones_menu_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_IMPLUVIUM,         .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_IMPLUVIUM_ZONES_ALL,.action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 2, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_1,  .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 3, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_2,  .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 4, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_3,  .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 5, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_4,  .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 6, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_5,  .action = NAV_ACTION_NAVIGATE, .handler = NULL },
};

static const nav_transition_t impluvium_zone_detail_transitions[] = {
    { .item_index = 0xFF, .next_menu = HMI_MENU_IMPLUVIUM_ZONES, .action = NAV_ACTION_NAVIGATE, .handler = NULL },
};

static const nav_transition_t impluvium_learning_menu_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_IMPLUVIUM,              .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_IMPLUVIUM_LEARNING_ALL, .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 2, .next_menu = HMI_MENU_IMPLUVIUM_LEARNING_1,   .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 3, .next_menu = HMI_MENU_IMPLUVIUM_LEARNING_2,   .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 4, .next_menu = HMI_MENU_IMPLUVIUM_LEARNING_3,   .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 5, .next_menu = HMI_MENU_IMPLUVIUM_LEARNING_4,   .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 6, .next_menu = HMI_MENU_IMPLUVIUM_LEARNING_5,   .action = NAV_ACTION_NAVIGATE, .handler = NULL },
};

static const nav_transition_t impluvium_learning_detail_transitions[] = {
    { .item_index = 0xFF, .next_menu = HMI_MENU_IMPLUVIUM_LEARNING, .action = NAV_ACTION_NAVIGATE, .handler = NULL },
};

static const nav_transition_t impluvium_controls_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_IMPLUVIUM,          .action = NAV_ACTION_NAVIGATE,      .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_IMPLUVIUM_CONTROLS, .action = NAV_ACTION_TOGGLE,        .handler = hmi_action_toggle_impluvium },
    { .item_index = 2, .next_menu = HMI_MENU_IMPLUVIUM_CONTROLS, .action = NAV_ACTION_EXECUTE,       .handler = hmi_action_force_moisture_check },
    { .item_index = 3, .next_menu = HMI_MENU_IMPLUVIUM_CONTROLS, .action = NAV_ACTION_EXECUTE,       .handler = hmi_action_emergency_stop_reset },
    { .item_index = 4, .next_menu = HMI_MENU_IMPLUVIUM_CONTROLS, .action = NAV_ACTION_EXECUTE,       .handler = hmi_action_clear_diagnostics },
    { .item_index = 5, .next_menu = HMI_MENU_CONFIRM,            .action = NAV_ACTION_CONFIRM_DIALOG,.handler = hmi_action_confirm_reset_all_learning },
};

static const nav_transition_t impluvium_zone_config_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_IMPLUVIUM,         .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_EDIT,.action = NAV_ACTION_EXECUTE, .handler = hmi_action_enter_zone_edit },
    { .item_index = 2, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_EDIT,.action = NAV_ACTION_EXECUTE, .handler = hmi_action_enter_zone_edit },
    { .item_index = 3, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_EDIT,.action = NAV_ACTION_EXECUTE, .handler = hmi_action_enter_zone_edit },
    { .item_index = 4, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_EDIT,.action = NAV_ACTION_EXECUTE, .handler = hmi_action_enter_zone_edit },
    { .item_index = 5, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_EDIT,.action = NAV_ACTION_EXECUTE, .handler = hmi_action_enter_zone_edit },
    { .item_index = 6, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_CONFIG,.action = NAV_ACTION_TOGGLE, .handler = hmi_action_toggle_all_zones },
};

static const nav_transition_t impluvium_zone_edit_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_CONFIG,.action = NAV_ACTION_EXECUTE,       .handler = hmi_action_zone_edit_cancel },
    { .item_index = 1, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_EDIT,  .action = NAV_ACTION_TOGGLE,        .handler = hmi_action_zone_edit_toggle_enabled },
    { .item_index = 2, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_EDIT,  .action = NAV_ACTION_EDIT_START,    .handler = hmi_action_zone_edit_toggle_target },
    { .item_index = 3, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_EDIT,  .action = NAV_ACTION_EDIT_START,    .handler = hmi_action_zone_edit_toggle_deadband },
    { .item_index = 4, .next_menu = HMI_MENU_CONFIRM,              .action = NAV_ACTION_CONFIRM_DIALOG,.handler = hmi_action_confirm_zone_reset_learning },
    { .item_index = 5, .next_menu = HMI_MENU_IMPLUVIUM_MANUAL_WATER,.action = NAV_ACTION_NAVIGATE,     .handler = hmi_action_zone_edit_manual_water },
    { .item_index = 6, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_CONFIG,.action = NAV_ACTION_EXECUTE,       .handler = hmi_action_zone_edit_save },
};

static const nav_transition_t impluvium_manual_water_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_IMPLUVIUM_MANUAL_WATER,.action = NAV_ACTION_EDIT_START,    .handler = hmi_action_manual_water_toggle_duration },
    { .item_index = 1, .next_menu = HMI_MENU_CONFIRM,               .action = NAV_ACTION_CONFIRM_DIALOG,.handler = hmi_action_confirm_manual_water },
    { .item_index = 2, .next_menu = HMI_MENU_IMPLUVIUM_ZONE_EDIT,   .action = NAV_ACTION_NAVIGATE,      .handler = hmi_action_manual_water_cancel },
};

static const nav_transition_t impluvium_intervals_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_IMPLUVIUM,           .action = NAV_ACTION_EXECUTE,    .handler = hmi_action_impluvium_intervals_back },
    { .item_index = 1, .next_menu = HMI_MENU_IMPLUVIUM_INTERVALS, .action = NAV_ACTION_EDIT_START, .handler = hmi_action_edit_impluvium_optimal },
    { .item_index = 2, .next_menu = HMI_MENU_IMPLUVIUM_INTERVALS, .action = NAV_ACTION_EDIT_START, .handler = hmi_action_edit_impluvium_cool },
    { .item_index = 3, .next_menu = HMI_MENU_IMPLUVIUM_INTERVALS, .action = NAV_ACTION_EDIT_START, .handler = hmi_action_edit_impluvium_power_save },
    { .item_index = 4, .next_menu = HMI_MENU_IMPLUVIUM_INTERVALS, .action = NAV_ACTION_EDIT_START, .handler = hmi_action_edit_impluvium_night_min },
};

// STELLARIA Menus
static const nav_transition_t stellaria_menu_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_MAIN,             .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_STELLARIA_STATUS, .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 2, .next_menu = HMI_MENU_STELLARIA_CONTROL,.action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 3, .next_menu = HMI_MENU_STELLARIA_AUTO,   .action = NAV_ACTION_NAVIGATE, .handler = NULL },
};

static const nav_transition_t stellaria_status_transitions[] = {
    { .item_index = 0xFF, .next_menu = HMI_MENU_STELLARIA, .action = NAV_ACTION_NAVIGATE, .handler = NULL },
};

static const nav_transition_t stellaria_control_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_STELLARIA,        .action = NAV_ACTION_NAVIGATE,  .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_STELLARIA_CONTROL,.action = NAV_ACTION_TOGGLE,    .handler = hmi_action_toggle_stellaria },
    { .item_index = 2, .next_menu = HMI_MENU_STELLARIA_CONTROL,.action = NAV_ACTION_EDIT_START,.handler = hmi_action_toggle_stellaria_intensity },
};

static const nav_transition_t stellaria_auto_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_STELLARIA,     .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_STELLARIA_AUTO,.action = NAV_ACTION_TOGGLE,   .handler = hmi_action_toggle_auto_mode },
};

// SYSTEM Menus
static const nav_transition_t system_menu_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_MAIN,            .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_SYSTEM_INFO,     .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 2, .next_menu = HMI_MENU_SYSTEM_CONTROLS, .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 3, .next_menu = HMI_MENU_SYSTEM_INTERVALS,.action = NAV_ACTION_NAVIGATE, .handler = NULL },
};

static const nav_transition_t system_info_transitions[] = {
    { .item_index = 0xFF, .next_menu = HMI_MENU_SYSTEM, .action = NAV_ACTION_NAVIGATE, .handler = NULL },
};

static const nav_transition_t system_controls_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_SYSTEM,         .action = NAV_ACTION_NAVIGATE,      .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_CONFIRM,        .action = NAV_ACTION_CONFIRM_DIALOG,.handler = hmi_action_confirm_flush_reset },
    { .item_index = 2, .next_menu = HMI_MENU_SYSTEM_CONTROLS,.action = NAV_ACTION_EXECUTE,       .handler = hmi_action_wifi_reconnect },
};

static const nav_transition_t system_intervals_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_SYSTEM,          .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_SYSTEM_INTERVALS,.action = NAV_ACTION_EXECUTE,  .handler = hmi_action_apply_aggressive_preset },
    { .item_index = 2, .next_menu = HMI_MENU_SYSTEM_INTERVALS,.action = NAV_ACTION_EXECUTE,  .handler = hmi_action_apply_balanced_preset },
    { .item_index = 3, .next_menu = HMI_MENU_SYSTEM_INTERVALS,.action = NAV_ACTION_EXECUTE,  .handler = hmi_action_apply_conservative_preset },
    { .item_index = 4, .next_menu = HMI_MENU_SYSTEM_INTERVALS,.action = NAV_ACTION_NONE,     .handler = NULL },  // Status display only
};

// Confirmation Dialog
static const nav_transition_t confirm_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_CONFIRM, .action = NAV_ACTION_EXECUTE, .handler = hmi_action_confirm_yes },
    { .item_index = 1, .next_menu = HMI_MENU_CONFIRM, .action = NAV_ACTION_EXECUTE, .handler = hmi_action_confirm_no },
};

// ########################## Master Metadata Table ##########################

static const nav_menu_metadata_t menu_metadata_table[] = {
    // Main Menu
    { .state = HMI_MENU_MAIN, .item_count = 5, .transitions = main_menu_transitions, .transition_count = 5, .is_realtime = false },

    // FLUCTUS Menus
    { .state = HMI_MENU_FLUCTUS,                .item_count = 9, .transitions = fluctus_menu_transitions,        .transition_count = 9, .is_realtime = false },
    { .state = HMI_MENU_FLUCTUS_OVERVIEW,       .item_count = 1, .transitions = fluctus_detail_page_transitions, .transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_FLUCTUS_ENERGY,         .item_count = 1, .transitions = fluctus_detail_page_transitions, .transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_FLUCTUS_LIVE_POWER,     .item_count = 1, .transitions = fluctus_detail_page_transitions, .transition_count = 1, .is_realtime = true },
    { .state = HMI_MENU_FLUCTUS_BUSES,          .item_count = 1, .transitions = fluctus_detail_page_transitions, .transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_FLUCTUS_SOLAR,          .item_count = 2, .transitions = fluctus_solar_transitions,       .transition_count = 2, .is_realtime = true },
    { .state = HMI_MENU_FLUCTUS_SERVO_DEBUG,    .item_count = 3, .transitions = fluctus_servo_debug_transitions, .transition_count = 3, .is_realtime = false },
    { .state = HMI_MENU_FLUCTUS_SERVO_CONTROL_YAW,   .item_count = 1, .transitions = fluctus_servo_control_transitions, .transition_count = 1, .is_realtime = true },
    { .state = HMI_MENU_FLUCTUS_SERVO_CONTROL_PITCH, .item_count = 1, .transitions = fluctus_servo_control_transitions, .transition_count = 1, .is_realtime = true },
    { .state = HMI_MENU_FLUCTUS_CONTROLS,       .item_count = 3, .transitions = fluctus_controls_transitions,    .transition_count = 3, .is_realtime = false },
    { .state = HMI_MENU_FLUCTUS_INTERVALS,      .item_count = 4, .transitions = fluctus_intervals_transitions,   .transition_count = 4, .is_realtime = false },

    // TEMPESTA Menus
    { .state = HMI_MENU_TEMPESTA,          .item_count = 5, .transitions = tempesta_menu_transitions,        .transition_count = 5, .is_realtime = false },
    { .state = HMI_MENU_TEMPESTA_SENSORS,  .item_count = 1, .transitions = tempesta_detail_page_transitions, .transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_TEMPESTA_DIAG,     .item_count = 1, .transitions = tempesta_diag_page_transitions,   .transition_count = 1, .is_realtime = true },
    { .state = HMI_MENU_TEMPESTA_CONTROLS, .item_count = 8, .transitions = tempesta_controls_transitions,    .transition_count = 8, .is_realtime = false },
    { .state = HMI_MENU_TEMPESTA_INTERVALS,.item_count = 3, .transitions = tempesta_intervals_transitions,   .transition_count = 3, .is_realtime = false },

    // IMPLUVIUM Menus
    { .state = HMI_MENU_IMPLUVIUM,              .item_count = 8, .transitions = impluvium_menu_transitions,           .transition_count = 8, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_OVERVIEW_STATS,.item_count = 1, .transitions = impluvium_detail_page_transitions,    .transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_ZONES,        .item_count = 7, .transitions = impluvium_zones_menu_transitions,     .transition_count = 7, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_ZONES_ALL,    .item_count = 1, .transitions = impluvium_zone_detail_transitions,    .transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_ZONE_1,       .item_count = 1, .transitions = impluvium_zone_detail_transitions,    .transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_ZONE_2,       .item_count = 1, .transitions = impluvium_zone_detail_transitions,    .transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_ZONE_3,       .item_count = 1, .transitions = impluvium_zone_detail_transitions,    .transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_ZONE_4,       .item_count = 1, .transitions = impluvium_zone_detail_transitions,    .transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_ZONE_5,       .item_count = 1, .transitions = impluvium_zone_detail_transitions,    .transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_LEARNING,     .item_count = 7, .transitions = impluvium_learning_menu_transitions,  .transition_count = 7, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_LEARNING_ALL, .item_count = 1, .transitions = impluvium_learning_detail_transitions,.transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_LEARNING_1,   .item_count = 1, .transitions = impluvium_learning_detail_transitions,.transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_LEARNING_2,   .item_count = 1, .transitions = impluvium_learning_detail_transitions,.transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_LEARNING_3,   .item_count = 1, .transitions = impluvium_learning_detail_transitions,.transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_LEARNING_4,   .item_count = 1, .transitions = impluvium_learning_detail_transitions,.transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_LEARNING_5,   .item_count = 1, .transitions = impluvium_learning_detail_transitions,.transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_MONITOR,      .item_count = 1, .transitions = impluvium_detail_page_transitions,    .transition_count = 1, .is_realtime = true },
    { .state = HMI_MENU_IMPLUVIUM_CONTROLS,     .item_count = 6, .transitions = impluvium_controls_transitions,       .transition_count = 6, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_ZONE_CONFIG,  .item_count = 7, .transitions = impluvium_zone_config_transitions,    .transition_count = 7, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_ZONE_EDIT,    .item_count = 7, .transitions = impluvium_zone_edit_transitions,      .transition_count = 7, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_MANUAL_WATER, .item_count = 3, .transitions = impluvium_manual_water_transitions,   .transition_count = 3, .is_realtime = false },
    { .state = HMI_MENU_IMPLUVIUM_INTERVALS,    .item_count = 5, .transitions = impluvium_intervals_transitions,      .transition_count = 5, .is_realtime = false },

    // STELLARIA Menus
    { .state = HMI_MENU_STELLARIA,        .item_count = 4, .transitions = stellaria_menu_transitions,    .transition_count = 4, .is_realtime = false },
    { .state = HMI_MENU_STELLARIA_STATUS, .item_count = 1, .transitions = stellaria_status_transitions,  .transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_STELLARIA_CONTROL,.item_count = 3, .transitions = stellaria_control_transitions, .transition_count = 3, .is_realtime = false },
    { .state = HMI_MENU_STELLARIA_AUTO,   .item_count = 2, .transitions = stellaria_auto_transitions,    .transition_count = 2, .is_realtime = false },

    // SYSTEM Menus
    { .state = HMI_MENU_SYSTEM,          .item_count = 4, .transitions = system_menu_transitions,      .transition_count = 4, .is_realtime = false },
    { .state = HMI_MENU_SYSTEM_INFO,     .item_count = 1, .transitions = system_info_transitions,      .transition_count = 1, .is_realtime = false },
    { .state = HMI_MENU_SYSTEM_CONTROLS, .item_count = 3, .transitions = system_controls_transitions,  .transition_count = 3, .is_realtime = false },
    { .state = HMI_MENU_SYSTEM_INTERVALS,.item_count = 5, .transitions = system_intervals_transitions, .transition_count = 5, .is_realtime = false },

    // Confirmation Dialog
    { .state = HMI_MENU_CONFIRM, .item_count = 2, .transitions = confirm_transitions, .transition_count = 2, .is_realtime = false },
};

#define MENU_METADATA_COUNT (sizeof(menu_metadata_table) / sizeof(nav_menu_metadata_t))

// ########################## Navigation Engine ##########################

const nav_menu_metadata_t* hmi_nav_get_menu_metadata(hmi_menu_state_t menu)
{
    for (size_t i = 0; i < MENU_METADATA_COUNT; i++) {
        if (menu_metadata_table[i].state == menu) {
            return &menu_metadata_table[i];
        }
    }

    ESP_LOGW(TAG, "Unknown menu state: %d", menu);
    return NULL;
}

const nav_transition_t* hmi_nav_find_transition(const nav_menu_metadata_t *meta, uint8_t selected_item)
{
    if (!meta) return NULL;

    for (size_t i = 0; i < meta->transition_count; i++) {
        if (meta->transitions[i].item_index == selected_item ||
            meta->transitions[i].item_index == 0xFF) {  // 0xFF = wildcard
            return &meta->transitions[i];
        }
    }

    ESP_LOGW(TAG, "No transition found for menu %d, item %d", meta->state, selected_item);
    return NULL;
}

// ########################## Scrolling Helper Functions ##########################

/**
 * @brief Update scroll window to ensure selected item is visible
 * @param total_items Total number of items in current menu
 */
static void hmi_update_scroll_window(uint8_t total_items)
{
    if (total_items <= HMI_MAX_VISIBLE_ITEMS) {
        // No scrolling needed
        hmi_status.scroll_offset = 0;
        return;
    }

    // Ensure selected_item is within visible window
    uint8_t first_visible = hmi_status.scroll_offset;
    uint8_t last_visible = first_visible + HMI_MAX_VISIBLE_ITEMS - 1;

    if (hmi_status.selected_item < first_visible) {
        // Scrolled above window - adjust offset up
        hmi_status.scroll_offset = hmi_status.selected_item;
    } else if (hmi_status.selected_item > last_visible) {
        // Scrolled below window - adjust offset down
        hmi_status.scroll_offset = hmi_status.selected_item - HMI_MAX_VISIBLE_ITEMS + 1;
    }

    // Clamp offset to valid range
    uint8_t max_offset = total_items - HMI_MAX_VISIBLE_ITEMS;
    if (hmi_status.scroll_offset > max_offset) {
        hmi_status.scroll_offset = max_offset;
    }
}

/**
 * @brief Save current scroll position and selected item to per-menu memory
 */
static void hmi_save_scroll_position(void)
{
    if (hmi_status.current_menu < 47) {  // Safety check (array bounds)
        hmi_status.menu_scroll_memory[hmi_status.current_menu] = hmi_status.scroll_offset;
        hmi_status.menu_selected_item_memory[hmi_status.current_menu] = hmi_status.selected_item;
    }
}

/**
 * @brief Restore scroll position and selected item from per-menu memory
 * @param menu Menu state to restore position for
 */
static void hmi_restore_scroll_position(hmi_menu_state_t menu)
{
    if (menu < 47) {  // Safety check (array bounds)
        hmi_status.scroll_offset = hmi_status.menu_scroll_memory[menu];
        hmi_status.selected_item = hmi_status.menu_selected_item_memory[menu];
    }
}

/**
 * @brief Get parent menu for a given detail page (for pagination back navigation)
 * @param current Current menu state
 * @return Parent menu state, or HMI_MENU_MAIN if no parent found
 */
static hmi_menu_state_t hmi_get_parent_menu(hmi_menu_state_t current)
{
    // Map detail pages to their parent menus
    if (current >= HMI_MENU_FLUCTUS && current <= HMI_MENU_FLUCTUS_INTERVALS) {
        return HMI_MENU_FLUCTUS;
    }
    if (current >= HMI_MENU_TEMPESTA && current <= HMI_MENU_TEMPESTA_INTERVALS) {
        return HMI_MENU_TEMPESTA;
    }
    if (current >= HMI_MENU_IMPLUVIUM && current <= HMI_MENU_IMPLUVIUM_INTERVALS) {
        return HMI_MENU_IMPLUVIUM;
    }
    if (current >= HMI_MENU_STELLARIA && current <= HMI_MENU_STELLARIA_AUTO) {
        return HMI_MENU_STELLARIA;
    }
    if (current >= HMI_MENU_SYSTEM && current <= HMI_MENU_SYSTEM_INTERVALS) {
        return HMI_MENU_SYSTEM;
    }

    // Default fallback
    return HMI_MENU_MAIN;
}

// ########################## Navigation Functions ##########################

void hmi_handle_button_press(void)
{
    hmi_update_activity();

    // Check if we're on a paginated detail page
    if (hmi_status.total_pages > 1) {
        // Special handling for FLUCTUS Tracking/Debug page 4 (interactive controls)
        if (hmi_status.current_menu == HMI_MENU_FLUCTUS_SOLAR && hmi_status.current_page == 3) {
            // Page 4 has interactive controls - use normal menu navigation
            // Fall through to the menu navigation logic below
        } else {
            // Button press on pagination = go back to parent menu
            hmi_menu_state_t parent = hmi_get_parent_menu(hmi_status.current_menu);

            // Reset pagination state
            hmi_status.current_page = 0;
            hmi_status.total_pages = 0;

            // Navigate to parent and restore scroll position + selected item
            hmi_status.current_menu = parent;
            hmi_restore_scroll_position(parent);
            return;
        }
    }

    const nav_menu_metadata_t *meta = hmi_nav_get_menu_metadata(hmi_status.current_menu);
    if (!meta) {
        ESP_LOGE(TAG, "Failed to get metadata for menu %d", hmi_status.current_menu);
        return;
    }

    const nav_transition_t *trans = hmi_nav_find_transition(meta, hmi_status.selected_item);
    if (!trans) {
        ESP_LOGE(TAG, "No valid transition for menu %d, item %d", hmi_status.current_menu, hmi_status.selected_item);
        return;
    }

    // Execute action based on type
    switch (trans->action) {
        case NAV_ACTION_NAVIGATE:
            // Save scroll position before leaving current menu
            hmi_save_scroll_position();

            // Reset pagination state when leaving detail pages
            hmi_status.current_page = 0;
            hmi_status.total_pages = 0;

            // Navigate to new menu and restore scroll + selection
            hmi_status.current_menu = trans->next_menu;
            hmi_restore_scroll_position(trans->next_menu);
            break;

        case NAV_ACTION_TOGGLE:
            // Toggle action (stays on same page)
            if (trans->handler) {
                trans->handler();
            }
            break;

        case NAV_ACTION_EDIT_START:
            // Enter/exit editing mode
            if (trans->handler) {
                trans->handler();
            }
            break;

        case NAV_ACTION_EDIT_CONFIRM:
            // Confirm edited value
            if (trans->handler) {
                trans->handler();
            }
            break;

        case NAV_ACTION_EXECUTE:
            // Execute command
            if (trans->handler) {
                trans->handler();
            }
            // May or may not navigate (handler decides)
            break;

        case NAV_ACTION_CONFIRM_DIALOG:
            // Set up confirmation dialog and navigate
            if (trans->handler) {
                trans->handler();  // Sets up confirmation context
            }
            hmi_status.current_menu = trans->next_menu;  // Should be HMI_MENU_CONFIRM
            hmi_status.selected_item = 0;
            break;

        case NAV_ACTION_NONE:
        default:
            // No action
            break;
    }
}

void hmi_handle_encoder_change(int delta)
{
    hmi_update_activity();

    // Check if we're on a paginated detail page
    if (hmi_status.total_pages > 1) {
        // Handle page navigation
        if (delta > 0) {
            hmi_status.current_page++;
            if (hmi_status.current_page >= hmi_status.total_pages) {
                hmi_status.current_page = 0; // Wrap to first page
            }
        } else {
            if (hmi_status.current_page == 0) {
                hmi_status.current_page = hmi_status.total_pages - 1; // Wrap to last page
            } else {
                hmi_status.current_page--;
            }
        }
        return; // Don't process as menu navigation
    }

    const nav_menu_metadata_t *meta = hmi_nav_get_menu_metadata(hmi_status.current_menu);
    if (!meta) return;

    // Handle editing modes
    if (stellaria_intensity_editing) {
        // Stellaria intensity: 5% steps
        int16_t new_value = stellaria_intensity_percent + (delta * 5);
        if (new_value < 0) new_value = 0;
        if (new_value > 100) new_value = 100;
        stellaria_intensity_percent = (uint8_t)new_value;
        return;
    }

    if (zone_editing && hmi_status.current_menu == HMI_MENU_IMPLUVIUM_ZONE_EDIT) {
        // Zone parameter editing
        if (hmi_status.selected_item == 2) {
            // Target moisture: 20-80%, 1% steps
            int16_t new_value = editing_zone_target + delta;
            if (new_value < 20) new_value = 20;
            if (new_value > 80) new_value = 80;
            editing_zone_target = (uint8_t)new_value;
        } else if (hmi_status.selected_item == 3) {
            // Deadband: 1-20%, 1% steps
            int16_t new_value = editing_zone_deadband + delta;
            if (new_value < 1) new_value = 1;
            if (new_value > 20) new_value = 20;
            editing_zone_deadband = (uint8_t)new_value;
        }
        return;
    }

    if (manual_water_input && hmi_status.current_menu == HMI_MENU_IMPLUVIUM_MANUAL_WATER) {
        // Manual water duration: 5-300s, 5s increments
        if (hmi_status.selected_item == 0) {
            int16_t new_value = manual_water_duration + (delta * 5);
            if (new_value < 5) new_value = 5;
            if (new_value > 300) new_value = 300;
            manual_water_duration = (uint16_t)new_value;
        }
        return;
    }

    if (interval_editing) {
        // Interval editing: 5-minute steps (or 1-hour for night min)
        int32_t step = (hmi_status.current_menu == HMI_MENU_IMPLUVIUM_INTERVALS && hmi_status.selected_item == 4) ? 1 : 5;
        int32_t new_value = (int32_t)editing_interval_value + (delta * step);

        // Apply range limits (simplified - actual limits checked in handlers)
        if (new_value < 1) new_value = 1;
        if (new_value > 240) new_value = 240;  // Max 240 minutes (4 hours) or hours

        editing_interval_value = (uint32_t)new_value;
        return;
    }

    // Servo control mode (direct servo position adjustment)
    if (servo_debug_active) {
        const int32_t SERVO_STEP = 50;  // 50 duty units per encoder step

        if (hmi_status.current_menu == HMI_MENU_FLUCTUS_SERVO_CONTROL_YAW) {
            // Control yaw servo using locally tracked position
            int32_t new_duty = (int32_t)servo_debug_current_duty + (delta * SERVO_STEP);

            // Clamp to servo limits (safety, though API also clamps)
            if (new_duty < FLUCTUS_SERVO_MIN_DUTY) new_duty = FLUCTUS_SERVO_MIN_DUTY;
            if (new_duty > FLUCTUS_SERVO_MAX_DUTY) new_duty = FLUCTUS_SERVO_MAX_DUTY;

            // Set servo position using public API (updates both hardware and system state)
            esp_err_t ret = fluctus_servo_debug_set_position(FLUCTUS_SERVO_YAW_CHANNEL, (uint32_t)new_duty);
            if (ret == ESP_OK) {
                // Update local tracking variable
                servo_debug_current_duty = (uint32_t)new_duty;
                ESP_LOGD(TAG, "Yaw servo: %ld (delta %d)", new_duty, delta);
            } else {
                ESP_LOGW(TAG, "Failed to set yaw servo position: %s", esp_err_to_name(ret));
            }

            // Update activity timestamp (reset timeout on user activity)
            servo_debug_start_time = time(NULL);

        } else if (hmi_status.current_menu == HMI_MENU_FLUCTUS_SERVO_CONTROL_PITCH) {
            // Control pitch servo using locally tracked position
            int32_t new_duty = (int32_t)servo_debug_current_duty + (delta * SERVO_STEP);

            // Clamp to servo limits (safety, though API also clamps)
            if (new_duty < FLUCTUS_SERVO_MIN_DUTY) new_duty = FLUCTUS_SERVO_MIN_DUTY;
            if (new_duty > FLUCTUS_SERVO_MAX_DUTY) new_duty = FLUCTUS_SERVO_MAX_DUTY;

            // Set servo position using public API (updates both hardware and system state)
            esp_err_t ret = fluctus_servo_debug_set_position(FLUCTUS_SERVO_PITCH_CHANNEL, (uint32_t)new_duty);
            if (ret == ESP_OK) {
                // Update local tracking variable
                servo_debug_current_duty = (uint32_t)new_duty;
            } else {
                ESP_LOGW(TAG, "Failed to set pitch servo position: %s", esp_err_to_name(ret));
            }

            // Update activity timestamp (reset timeout on user activity)
            servo_debug_start_time = time(NULL);
        }
        return;
    }

    // Normal menu navigation
    if (delta > 0) {
        hmi_status.selected_item++;
        if (hmi_status.selected_item >= meta->item_count) {
            hmi_status.selected_item = 0;
        }
    } else {
        if (hmi_status.selected_item == 0) {
            hmi_status.selected_item = meta->item_count - 1;
        } else {
            hmi_status.selected_item--;
        }
    }

    // Update scroll window to keep selected item visible
    hmi_update_scroll_window(meta->item_count);
}

uint8_t hmi_get_menu_item_count(hmi_menu_state_t menu)
{
    const nav_menu_metadata_t *meta = hmi_nav_get_menu_metadata(menu);
    return meta ? meta->item_count : 0;
}

bool hmi_is_realtime_page(void)
{
    const nav_menu_metadata_t *meta = hmi_nav_get_menu_metadata(hmi_status.current_menu);
    return meta ? meta->is_realtime : false;
}

void hmi_render_menu(void)
{
    // Dispatch to component-specific renderers based on menu state ranges
    if (hmi_status.current_menu >= HMI_MENU_FLUCTUS &&
        hmi_status.current_menu <= HMI_MENU_FLUCTUS_INTERVALS) {
        hmi_render_fluctus_pages();
    }
    else if (hmi_status.current_menu >= HMI_MENU_TEMPESTA &&
             hmi_status.current_menu <= HMI_MENU_TEMPESTA_INTERVALS) {
        hmi_render_tempesta_pages();
    }
    else if (hmi_status.current_menu >= HMI_MENU_IMPLUVIUM &&
             hmi_status.current_menu <= HMI_MENU_IMPLUVIUM_INTERVALS) {
        hmi_render_impluvium_pages();
    }
    else if (hmi_status.current_menu == HMI_MENU_MAIN ||
             hmi_status.current_menu == HMI_MENU_CONFIRM ||
             (hmi_status.current_menu >= HMI_MENU_STELLARIA &&
              hmi_status.current_menu <= HMI_MENU_SYSTEM_INTERVALS)) {
        hmi_render_system_pages();
    }
    else {
        ESP_LOGW(TAG, "Unknown menu state for rendering: %d", hmi_status.current_menu);
    }
}

// ########################## Action Handler Implementations ##########################

// NOTE: Due to file size limits, I'll implement key action handlers here.
// The rest will follow the same pattern and can be added incrementally.

// FLUCTUS Actions
static void hmi_action_toggle_solar_tracking(void)
{
    solar_tracking_state_t state = fluctus_get_solar_tracking_state();
    if (state == SOLAR_TRACKING_DISABLED) {
        fluctus_enable_solar_tracking();
        ESP_LOGI(TAG, "Solar tracking enabled");
    } else {
        fluctus_disable_solar_tracking();
        ESP_LOGI(TAG, "Solar tracking disabled");
    }

    // Force telemetry cache update so HMI sees new values immediately
    telemetry_fetch_snapshot(TELEMETRY_SRC_FLUCTUS);
}

static void hmi_action_manual_safety_reset(void)
{
    fluctus_manual_safety_reset();
    ESP_LOGI(TAG, "Manual safety reset executed");
}

static void hmi_action_fluctus_intervals_back(void)
{
    // Exit editing mode if active
    interval_editing = false;
    hmi_status.current_menu = HMI_MENU_FLUCTUS;
    hmi_status.selected_item = 0;
}

static void hmi_action_edit_fluctus_power_day(void)
{
    if (interval_editing) {
        // Apply change - use combined setter
        fluctus_set_power_intervals(editing_interval_value, g_interval_config.fluctus_power_night_min);
        interval_editing = false;
        ESP_LOGI(TAG, "FLUCTUS Power Day interval set to %lu min", editing_interval_value);
    } else {
        // Enter edit mode
        editing_interval_value = g_interval_config.fluctus_power_day_min;
        interval_editing = true;
    }
}

static void hmi_action_edit_fluctus_power_night(void)
{
    if (interval_editing) {
        // Apply change - use combined setter
        fluctus_set_power_intervals(g_interval_config.fluctus_power_day_min, editing_interval_value);
        interval_editing = false;
        ESP_LOGI(TAG, "FLUCTUS Power Night interval set to %lu min", editing_interval_value);
    } else {
        editing_interval_value = g_interval_config.fluctus_power_night_min;
        interval_editing = true;
    }
}

static void hmi_action_edit_fluctus_solar_correction(void)
{
    if (interval_editing) {
        // Apply change
        fluctus_set_solar_interval(editing_interval_value);
        interval_editing = false;
        ESP_LOGI(TAG, "FLUCTUS Solar Correction interval set to %lu min", editing_interval_value);
    } else {
        editing_interval_value = g_interval_config.fluctus_solar_correction_min;
        interval_editing = true;
    }
}

static void hmi_action_toggle_solar_debug_mode(void)
{
    bool debug_active = fluctus_is_solar_debug_mode_active();
    if (debug_active) {
        fluctus_disable_solar_debug_mode();
        ESP_LOGI(TAG, "Solar debug mode disabled");
    } else {
        esp_err_t ret = fluctus_enable_solar_debug_mode();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Solar debug mode enabled (90s timeout)");
        } else {
            ESP_LOGW(TAG, "Failed to enable debug mode - solar tracking may be disabled");
        }
    }

    // Force telemetry cache update so HMI sees new values immediately
    telemetry_fetch_snapshot(TELEMETRY_SRC_FLUCTUS);
}

/**
 * @brief Enter servo control mode for Yaw servo
 * Requests 6.6V bus power and initializes timeout tracking
 */
static void hmi_action_enter_servo_control_yaw(void)
{
    ESP_LOGI(TAG, "Entering Yaw servo control mode");

    // Read current yaw position from telemetry (one-time snapshot)
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);
    servo_debug_current_duty = data.current_yaw_duty;
    ESP_LOGI(TAG, "Initial yaw position: %lu (duty cycle)", (unsigned long)servo_debug_current_duty);

    // Request 6.6V bus power for servos
    esp_err_t ret = fluctus_request_bus_power(POWER_BUS_6V6, "HMI_SERVO_DEBUG");
    if (ret == ESP_OK) {
        servo_debug_bus_requested = true;
        servo_debug_active = true;
        servo_debug_start_time = time(NULL);
        ESP_LOGI(TAG, "6.6V bus powered, servo control active");
    } else {
        ESP_LOGW(TAG, "Failed to power 6.6V bus: %s", esp_err_to_name(ret));
        // Still allow control mode (maybe bus already powered by tracking system)
        servo_debug_active = true;
        servo_debug_start_time = time(NULL);
    }

    // Navigate to control page
    hmi_status.current_menu = HMI_MENU_FLUCTUS_SERVO_CONTROL_YAW;
    hmi_status.selected_item = 0;
}

/**
 * @brief Enter servo control mode for Pitch servo
 * Requests 6.6V bus power and initializes timeout tracking
 */
static void hmi_action_enter_servo_control_pitch(void)
{
    ESP_LOGI(TAG, "Entering Pitch servo control mode");

    // Read current pitch position from telemetry (one-time snapshot)
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);
    servo_debug_current_duty = data.current_pitch_duty;
    ESP_LOGI(TAG, "Initial pitch position: %lu (duty cycle)", (unsigned long)servo_debug_current_duty);

    // Request 6.6V bus power for servos
    esp_err_t ret = fluctus_request_bus_power(POWER_BUS_6V6, "HMI_SERVO_DEBUG");
    if (ret == ESP_OK) {
        servo_debug_bus_requested = true;
        servo_debug_active = true;
        servo_debug_start_time = time(NULL);
        ESP_LOGI(TAG, "6.6V bus powered, servo control active");
    } else {
        ESP_LOGW(TAG, "Failed to power 6.6V bus: %s", esp_err_to_name(ret));
        // Still allow control mode (maybe bus already powered by tracking system)
        servo_debug_active = true;
        servo_debug_start_time = time(NULL);
    }

    // Navigate to control page
    hmi_status.current_menu = HMI_MENU_FLUCTUS_SERVO_CONTROL_PITCH;
    hmi_status.selected_item = 0;
}

/**
 * @brief Exit servo control mode
 * Releases 6.6V bus power and clears state
 */
static void hmi_action_exit_servo_control(void)
{
    ESP_LOGI(TAG, "Exiting servo control mode");

    // Release 6.6V bus power if we requested it
    if (servo_debug_bus_requested) {
        fluctus_release_bus_power(POWER_BUS_6V6, "HMI_SERVO_DEBUG");
        servo_debug_bus_requested = false;
    }

    servo_debug_active = false;
    servo_debug_start_time = 0;
    servo_debug_current_duty = 0;

    // Navigate back to servo debug menu
    hmi_status.current_menu = HMI_MENU_FLUCTUS_SERVO_DEBUG;
    hmi_status.selected_item = 0;
}

// TEMPESTA Actions
static void hmi_action_toggle_tempesta(void)
{
    tempesta_state_t state = tempesta_get_state();
    if (state == TEMPESTA_STATE_DISABLED) {
        tempesta_set_system_enabled(true);
        ESP_LOGI(TAG, "TEMPESTA enabled");
    } else {
        tempesta_set_system_enabled(false);
        ESP_LOGI(TAG, "TEMPESTA disabled");
    }

    // Force telemetry cache update so HMI sees new values immediately
    telemetry_fetch_snapshot(TELEMETRY_SRC_TEMPESTA);
}

static void hmi_action_force_tempesta_collection(void)
{
    tempesta_force_collection();
    ESP_LOGI(TAG, "TEMPESTA force collection triggered");
}

static void hmi_action_confirm_tempesta_reset_daily(void)
{
    confirmation_action = CONFIRM_TEMPESTA_RESET_DAILY;
    confirmation_return_menu = HMI_MENU_TEMPESTA_CONTROLS;
}

static void hmi_action_confirm_tempesta_reset_weekly(void)
{
    confirmation_action = CONFIRM_TEMPESTA_RESET_WEEKLY;
    confirmation_return_menu = HMI_MENU_TEMPESTA_CONTROLS;
}

static void hmi_action_confirm_tempesta_reset_rain(void)
{
    confirmation_action = CONFIRM_TEMPESTA_RESET_RAIN_TOTAL;
    confirmation_return_menu = HMI_MENU_TEMPESTA_CONTROLS;
}

static void hmi_action_confirm_tempesta_reset_tank(void)
{
    confirmation_action = CONFIRM_TEMPESTA_RESET_TANK_TOTAL;
    confirmation_return_menu = HMI_MENU_TEMPESTA_CONTROLS;
}

static void hmi_action_confirm_tempesta_reset_all(void)
{
    confirmation_action = CONFIRM_TEMPESTA_RESET_ALL;
    confirmation_return_menu = HMI_MENU_TEMPESTA_CONTROLS;
}

static void hmi_action_tempesta_intervals_back(void)
{
    interval_editing = false;
    hmi_status.current_menu = HMI_MENU_TEMPESTA;
    hmi_status.selected_item = 0;
}

static void hmi_action_edit_tempesta_normal(void)
{
    if (interval_editing) {
        // Apply change - use correct API name
        tempesta_set_collection_intervals(editing_interval_value, g_interval_config.tempesta_power_save_min);
        interval_editing = false;
        ESP_LOGI(TAG, "TEMPESTA Normal interval set to %lu min", editing_interval_value);
    } else {
        editing_interval_value = g_interval_config.tempesta_normal_min;
        interval_editing = true;
    }
}

static void hmi_action_edit_tempesta_power_save(void)
{
    if (interval_editing) {
        // Apply change - use correct API name
        tempesta_set_collection_intervals(g_interval_config.tempesta_normal_min, editing_interval_value);
        interval_editing = false;
        ESP_LOGI(TAG, "TEMPESTA Power Save interval set to %lu min", editing_interval_value);
    } else {
        editing_interval_value = g_interval_config.tempesta_power_save_min;
        interval_editing = true;
    }
}

/**
 * @brief Enter TEMPESTA diagnostic mode
 * Requests 3.3V bus power, enters diagnostic mode, navigates to diagnostic page
 */
static void hmi_action_enter_tempesta_diag(void)
{
    ESP_LOGI(TAG, "Entering TEMPESTA diagnostic mode");

    // Request 3.3V bus power for sensors (I2C Bus B)
    esp_err_t ret = fluctus_request_bus_power(POWER_BUS_3V3, "HMI_TEMPESTA_DIAG");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to power 3.3V bus: %s", esp_err_to_name(ret));
        // Don't proceed if we can't power the sensors
        return;
    }
    tempesta_diag_bus_requested = true;

    // Enter TEMPESTA diagnostic mode (powers hall array + AS5600 NORMAL)
    ret = tempesta_enter_diagnostic_mode();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to enter diagnostic mode: %s", esp_err_to_name(ret));

        // Clean up - release bus power
        fluctus_release_bus_power(POWER_BUS_3V3, "HMI_TEMPESTA_DIAG");
        tempesta_diag_bus_requested = false;
        return;
    }

    // Mark diagnostic mode active and start timeout timer
    tempesta_diag_active = true;
    tempesta_diag_start_time = time(NULL);

    // Navigate to diagnostic page
    hmi_status.current_menu = HMI_MENU_TEMPESTA_DIAG;
    hmi_status.selected_item = 0;
    hmi_status.current_page = 0;  // Start on page 1 (Hall array)

    ESP_LOGI(TAG, "Diagnostic mode ACTIVE (90s timeout)");
}

/**
 * @brief Exit TEMPESTA diagnostic mode
 * Cleans up power, exits diagnostic mode, returns to TEMPESTA menu
 */
static void hmi_action_exit_tempesta_diag(void)
{
    ESP_LOGI(TAG, "Exiting TEMPESTA diagnostic mode");

    // Exit TEMPESTA diagnostic mode (disables hall array + AS5600 LPM3)
    tempesta_exit_diagnostic_mode();

    // Release 3.3V bus power
    if (tempesta_diag_bus_requested) {
        fluctus_release_bus_power(POWER_BUS_3V3, "HMI_TEMPESTA_DIAG");
        tempesta_diag_bus_requested = false;
    }

    // Clear diagnostic state
    tempesta_diag_active = false;

    // Navigate back to TEMPESTA menu
    hmi_status.current_menu = HMI_MENU_TEMPESTA;
    hmi_status.selected_item = 0;
    hmi_status.current_page = 0;
    hmi_status.total_pages = 0;

    ESP_LOGI(TAG, "Diagnostic mode EXITED");
}

// IMPLUVIUM Actions
static void hmi_action_toggle_impluvium(void)
{
    impluvium_state_t state = impluvium_get_state();
    if (state == IMPLUVIUM_DISABLED) {
        impluvium_set_system_enabled(true);
        ESP_LOGI(TAG, "IMPLUVIUM enabled");
    } else {
        impluvium_set_system_enabled(false);
        ESP_LOGI(TAG, "IMPLUVIUM disabled");
    }

    // Force telemetry cache update so HMI sees new values immediately
    telemetry_fetch_snapshot(TELEMETRY_SRC_IMPLUVIUM);
}

static void hmi_action_force_moisture_check(void)
{
    impluvium_force_moisture_check();
    ESP_LOGI(TAG, "IMPLUVIUM force moisture check triggered");
}

static void hmi_action_emergency_stop_reset(void)
{
    impluvium_clear_emergency_stop();
    ESP_LOGI(TAG, "IMPLUVIUM emergency stop reset");
}

static void hmi_action_clear_diagnostics(void)
{
    impluvium_clear_diagnostics();
    ESP_LOGI(TAG, "IMPLUVIUM diagnostics cleared");
}

static void hmi_action_confirm_reset_all_learning(void)
{
    confirmation_action = CONFIRM_RESET_ALL_LEARNING;
    confirmation_return_menu = HMI_MENU_IMPLUVIUM_CONTROLS;
}

static void hmi_action_enter_zone_edit(void)
{
    // Load zone config for editing (zone ID = selected_item - 1)
    editing_zone_id = hmi_status.selected_item - 1;

    // Get current zone configuration from telemetry snapshot
    impluvium_snapshot_t snapshot;
    if (telemetry_get_impluvium_data(&snapshot) == ESP_OK && editing_zone_id < IRRIGATION_ZONE_COUNT) {
        editing_zone_enabled = snapshot.zones[editing_zone_id].watering_enabled;
        editing_zone_target = snapshot.zones[editing_zone_id].target_moisture_percent;
        editing_zone_deadband = snapshot.zones[editing_zone_id].moisture_deadband_percent;

        // Use defaults if config is uninitialized (both target and deadband are 0.0)
        if (editing_zone_target == 0.0f && editing_zone_deadband == 0.0f) {
            editing_zone_target = 40.0f;      // Default 40% target
            editing_zone_deadband = 5.0f;     // Default 5% deadband
            ESP_LOGW(TAG, "Zone %d config uninitialized, using defaults: target=%.1f%%, deadband=%.1f%%",
                     editing_zone_id, editing_zone_target, editing_zone_deadband);
        }
    }

    zone_editing = false;  // Not in edit mode yet
    hmi_status.current_menu = HMI_MENU_IMPLUVIUM_ZONE_EDIT;
    hmi_status.selected_item = 0;
}

static void hmi_action_toggle_all_zones(void)
{
    // Refresh telemetry cache to ensure we have current zone configurations
    telemetry_fetch_snapshot(TELEMETRY_SRC_IMPLUVIUM);

    // Get current zone configurations from telemetry snapshot
    impluvium_snapshot_t snapshot;
    if (telemetry_get_impluvium_data(&snapshot) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get impluvium data for toggle all zones");
        return;
    }

    // Count enabled zones to determine action (if all enabled -> disable all, else -> enable all)
    uint8_t enabled_count = 0;
    for (int i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        if (snapshot.zones[i].watering_enabled) {
            enabled_count++;
        }
    }
    bool enable_all = (enabled_count < IRRIGATION_ZONE_COUNT);

    // Apply to all zones (preserve existing target/deadband from refreshed snapshot)
    for (int i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        impluvium_update_zone_config(i,
                                      snapshot.zones[i].target_moisture_percent,
                                      snapshot.zones[i].moisture_deadband_percent,
                                      enable_all);
    }

    ESP_LOGI(TAG, "All zones %s", enable_all ? "enabled" : "disabled");

    // Force telemetry cache update so HMI sees new values immediately
    telemetry_fetch_snapshot(TELEMETRY_SRC_IMPLUVIUM);
}

static void hmi_action_zone_edit_cancel(void)
{
    zone_editing = false;
    hmi_status.current_menu = HMI_MENU_IMPLUVIUM_ZONE_CONFIG;
    hmi_status.selected_item = 0;
}

static void hmi_action_zone_edit_toggle_enabled(void)
{
    editing_zone_enabled = !editing_zone_enabled;
}

static void hmi_action_zone_edit_toggle_target(void)
{
    zone_editing = !zone_editing;
}

static void hmi_action_zone_edit_toggle_deadband(void)
{
    zone_editing = !zone_editing;
}

static void hmi_action_confirm_zone_reset_learning(void)
{
    confirmation_action = CONFIRM_RESET_ZONE_LEARNING;
    confirmation_param = editing_zone_id;
    confirmation_return_menu = HMI_MENU_IMPLUVIUM_ZONE_EDIT;
}

static void hmi_action_zone_edit_manual_water(void)
{
    manual_water_zone = editing_zone_id;
    manual_water_duration = 30;  // Default 30 seconds
    manual_water_input = false;
}

static void hmi_action_zone_edit_save(void)
{
    // Save zone configuration using individual parameters
    impluvium_update_zone_config(editing_zone_id,
                                  editing_zone_target,
                                  editing_zone_deadband,
                                  editing_zone_enabled);

    ESP_LOGI(TAG, "Zone %d config saved: enabled=%d, target=%.1f%%, deadband=%.1f%%",
             editing_zone_id, editing_zone_enabled, editing_zone_target, editing_zone_deadband);

    // Force telemetry cache update so HMI sees new values immediately
    telemetry_fetch_snapshot(TELEMETRY_SRC_IMPLUVIUM);

    zone_editing = false;
    hmi_status.current_menu = HMI_MENU_IMPLUVIUM_ZONE_CONFIG;
    hmi_status.selected_item = 0;
}

static void hmi_action_manual_water_toggle_duration(void)
{
    manual_water_input = !manual_water_input;
}

static void hmi_action_confirm_manual_water(void)
{
    confirmation_action = CONFIRM_MANUAL_WATER;
    confirmation_param = manual_water_zone;
    confirmation_return_menu = HMI_MENU_IMPLUVIUM_MANUAL_WATER;
}

static void hmi_action_manual_water_cancel(void)
{
    manual_water_input = false;
    hmi_status.current_menu = HMI_MENU_IMPLUVIUM_ZONE_EDIT;
    hmi_status.selected_item = 0;
}

static void hmi_action_impluvium_intervals_back(void)
{
    interval_editing = false;
    hmi_status.current_menu = HMI_MENU_IMPLUVIUM;
    hmi_status.selected_item = 0;
}

static void hmi_action_edit_impluvium_optimal(void)
{
    if (interval_editing) {
        // Apply change - use correct API signature
        impluvium_set_check_intervals(editing_interval_value, g_interval_config.impluvium_cool_min,
                                      g_interval_config.impluvium_power_save_min, g_interval_config.impluvium_night_min_hours);
        interval_editing = false;
        ESP_LOGI(TAG, "IMPLUVIUM Optimal interval set to %lu min", editing_interval_value);
    } else {
        editing_interval_value = g_interval_config.impluvium_optimal_min;
        interval_editing = true;
    }
}

static void hmi_action_edit_impluvium_cool(void)
{
    if (interval_editing) {
        // Apply change - use correct API signature
        impluvium_set_check_intervals(g_interval_config.impluvium_optimal_min, editing_interval_value,
                                      g_interval_config.impluvium_power_save_min, g_interval_config.impluvium_night_min_hours);
        interval_editing = false;
        ESP_LOGI(TAG, "IMPLUVIUM Cool interval set to %lu min", editing_interval_value);
    } else {
        editing_interval_value = g_interval_config.impluvium_cool_min;
        interval_editing = true;
    }
}

static void hmi_action_edit_impluvium_power_save(void)
{
    if (interval_editing) {
        // Apply change - use correct API signature
        impluvium_set_check_intervals(g_interval_config.impluvium_optimal_min, g_interval_config.impluvium_cool_min,
                                      editing_interval_value, g_interval_config.impluvium_night_min_hours);
        interval_editing = false;
        ESP_LOGI(TAG, "IMPLUVIUM Power Save interval set to %lu min", editing_interval_value);
    } else {
        editing_interval_value = g_interval_config.impluvium_power_save_min;
        interval_editing = true;
    }
}

static void hmi_action_edit_impluvium_night_min(void)
{
    if (interval_editing) {
        // Apply change - use correct API signature
        impluvium_set_check_intervals(g_interval_config.impluvium_optimal_min, g_interval_config.impluvium_cool_min,
                                      g_interval_config.impluvium_power_save_min, editing_interval_value);
        interval_editing = false;
        ESP_LOGI(TAG, "IMPLUVIUM Night Min interval set to %lu hours", editing_interval_value);
    } else {
        editing_interval_value = g_interval_config.impluvium_night_min_hours;
        interval_editing = true;
    }
}

// STELLARIA Actions
static void hmi_action_toggle_stellaria(void)
{
    stellaria_state_t state = stellaria_get_state();
    if (state == STELLARIA_STATE_DISABLED) {
        stellaria_enable();
        ESP_LOGI(TAG, "STELLARIA enabled");
    } else {
        stellaria_disable();
        ESP_LOGI(TAG, "STELLARIA disabled");
    }

    // Force telemetry cache update so HMI sees new values immediately
    telemetry_fetch_snapshot(TELEMETRY_SRC_STELLARIA);
}

static void hmi_action_toggle_stellaria_intensity(void)
{
    if (stellaria_intensity_editing) {
        // Apply intensity
        uint16_t pwm_value = (stellaria_intensity_percent * 1023) / 100;
        stellaria_set_intensity(pwm_value);
        stellaria_intensity_editing = false;
        ESP_LOGI(TAG, "STELLARIA intensity set to %d%%", stellaria_intensity_percent);

        // Force telemetry cache update so HMI sees new values immediately
        telemetry_fetch_snapshot(TELEMETRY_SRC_STELLARIA);
    } else {
        // Enter edit mode - load current intensity from telemetry snapshot
        stellaria_snapshot_t snapshot;
        if (telemetry_get_stellaria_data(&snapshot) == ESP_OK) {
            stellaria_intensity_percent = (snapshot.current_intensity * 100) / 1023;
        }
        stellaria_intensity_editing = true;
    }
}

static void hmi_action_toggle_auto_mode(void)
{
    // Get current auto mode state from telemetry snapshot
    stellaria_snapshot_t snapshot;
    bool auto_mode = false;
    if (telemetry_get_stellaria_data(&snapshot) == ESP_OK) {
        auto_mode = snapshot.auto_mode_active;
    }
    stellaria_set_auto_mode(!auto_mode);
    ESP_LOGI(TAG, "STELLARIA auto mode %s", auto_mode ? "disabled" : "enabled");

    // Force telemetry cache update so HMI sees new values immediately
    telemetry_fetch_snapshot(TELEMETRY_SRC_STELLARIA);
}

// SYSTEM Actions
static void hmi_action_confirm_flush_reset(void)
{
    confirmation_action = CONFIRM_SYSTEM_FLUSH_RESET;
    confirmation_return_menu = HMI_MENU_SYSTEM_CONTROLS;
}

static void hmi_action_wifi_reconnect(void)
{
    wifi_helper_force_reconnect();
    ESP_LOGI(TAG, "WiFi reconnect triggered");
}

static void hmi_action_apply_aggressive_preset(void)
{
    interval_config_apply_preset(INTERVAL_PRESET_AGGRESSIVE);

    // Update all components with new intervals from global config
    fluctus_set_power_intervals(g_interval_config.fluctus_power_day_min, g_interval_config.fluctus_power_night_min);
    fluctus_set_solar_interval(g_interval_config.fluctus_solar_correction_min);
    tempesta_set_collection_intervals(g_interval_config.tempesta_normal_min, g_interval_config.tempesta_power_save_min);
    impluvium_set_check_intervals(g_interval_config.impluvium_optimal_min, g_interval_config.impluvium_cool_min,
                                  g_interval_config.impluvium_power_save_min, g_interval_config.impluvium_night_min_hours);

    ESP_LOGI(TAG, "Applied Aggressive preset");
}

static void hmi_action_apply_balanced_preset(void)
{
    interval_config_apply_preset(INTERVAL_PRESET_BALANCED);

    // Update all components with new intervals from global config
    fluctus_set_power_intervals(g_interval_config.fluctus_power_day_min, g_interval_config.fluctus_power_night_min);
    fluctus_set_solar_interval(g_interval_config.fluctus_solar_correction_min);
    tempesta_set_collection_intervals(g_interval_config.tempesta_normal_min, g_interval_config.tempesta_power_save_min);
    impluvium_set_check_intervals(g_interval_config.impluvium_optimal_min, g_interval_config.impluvium_cool_min,
                                  g_interval_config.impluvium_power_save_min, g_interval_config.impluvium_night_min_hours);

    ESP_LOGI(TAG, "Applied Balanced preset");
}

static void hmi_action_apply_conservative_preset(void)
{
    interval_config_apply_preset(INTERVAL_PRESET_CONSERVATIVE);

    // Update all components with new intervals from global config
    fluctus_set_power_intervals(g_interval_config.fluctus_power_day_min, g_interval_config.fluctus_power_night_min);
    fluctus_set_solar_interval(g_interval_config.fluctus_solar_correction_min);
    tempesta_set_collection_intervals(g_interval_config.tempesta_normal_min, g_interval_config.tempesta_power_save_min);
    impluvium_set_check_intervals(g_interval_config.impluvium_optimal_min, g_interval_config.impluvium_cool_min,
                                  g_interval_config.impluvium_power_save_min, g_interval_config.impluvium_night_min_hours);

    ESP_LOGI(TAG, "Applied Conservative preset");
}

// Confirmation Actions
static void hmi_action_confirm_yes(void)
{
    // Execute confirmed action based on confirmation_action
    switch (confirmation_action) {
        case CONFIRM_TEMPESTA_RESET_DAILY:
            tempesta_reset_daily_counters();
            ESP_LOGI(TAG, "TEMPESTA daily counters reset");
            break;

        case CONFIRM_TEMPESTA_RESET_WEEKLY:
            tempesta_reset_weekly_counters();
            ESP_LOGI(TAG, "TEMPESTA weekly counters reset");
            break;

        case CONFIRM_TEMPESTA_RESET_RAIN_TOTAL:
            tempesta_reset_rain_gauge_total();
            ESP_LOGI(TAG, "Rain gauge total reset");
            break;

        case CONFIRM_TEMPESTA_RESET_TANK_TOTAL:
            tempesta_reset_tank_intake_total();
            ESP_LOGI(TAG, "Tank intake total reset");
            break;

        case CONFIRM_TEMPESTA_RESET_ALL:
            tempesta_reset_all_counters();
            ESP_LOGI(TAG, "All TEMPESTA counters reset");
            break;

        case CONFIRM_RESET_ZONE_LEARNING:
            impluvium_reset_zone_learning(confirmation_param);
            ESP_LOGI(TAG, "Zone %d learning reset", confirmation_param);
            break;

        case CONFIRM_RESET_ALL_LEARNING:
            impluvium_reset_all_learning();
            ESP_LOGI(TAG, "All zone learning reset");
            break;

        case CONFIRM_MANUAL_WATER:
            impluvium_force_water_zone(manual_water_zone, manual_water_duration);
            ESP_LOGI(TAG, "Manual water: zone %d, %d seconds", manual_water_zone, manual_water_duration);
            break;

        case CONFIRM_SYSTEM_FLUSH_RESET:
            {
                uint16_t flushed_count = 0;
                telemetry_manual_flush_to_flash(&flushed_count);
                ESP_LOGI(TAG, "MQTT buffer flushed to FLASH (%d messages), restarting...", flushed_count);
                vTaskDelay(pdMS_TO_TICKS(1000));  // Allow time for flush
                esp_restart();
            }
            break;

        default:
            ESP_LOGW(TAG, "Unknown confirmation action: %d", confirmation_action);
            break;
    }

    // Return to menu
    hmi_status.current_menu = confirmation_return_menu;
    hmi_status.selected_item = 0;
    confirmation_action = CONFIRM_NONE;
}

static void hmi_action_confirm_no(void)
{
    // Cancel - return to previous menu
    hmi_status.current_menu = confirmation_return_menu;
    hmi_status.selected_item = 0;
    confirmation_action = CONFIRM_NONE;
    ESP_LOGI(TAG, "Confirmation cancelled");
}
