/**
 * @file hmi_navigation.h
 * @brief HMI Component - Table-Driven State Machine Types and API
 *
 * Defines the data structures and API for the table-driven navigation system.
 * This allows menus to be defined as declarative tables rather than nested
 * if-else statements, improving maintainability and clarity.
 */

#ifndef HMI_NAVIGATION_H
#define HMI_NAVIGATION_H

#include <stdint.h>
#include <stdbool.h>
#include "hmi.h"

// ########################## Navigation Action Types ##########################

/**
 * @brief Action types for state transitions
 *
 * These define what happens when a button is pressed on a menu item:
 * - NAVIGATE: Move to a different menu state
 * - EDIT_START: Begin editing a value (enter edit mode)
 * - EDIT_CONFIRM: Confirm edited value and exit edit mode
 * - EDIT_CANCEL: Cancel editing without saving
 * - TOGGLE: Toggle a boolean state without navigation
 * - EXECUTE: Execute a command/action
 * - CONFIRM_DIALOG: Show confirmation dialog before action
 */
typedef enum {
    NAV_ACTION_NONE = 0,
    NAV_ACTION_NAVIGATE,        // Simple menu navigation
    NAV_ACTION_EDIT_START,      // Begin editing mode
    NAV_ACTION_EDIT_CONFIRM,    // Commit edited value
    NAV_ACTION_EDIT_CANCEL,     // Cancel editing
    NAV_ACTION_TOGGLE,          // Toggle boolean state
    NAV_ACTION_EXECUTE,         // Execute command
    NAV_ACTION_CONFIRM_DIALOG,  // Show confirmation dialog
} nav_action_type_t;

// ########################## State Transition Types ##########################

/**
 * @brief Single state transition definition
 *
 * Describes what happens when a button is pressed on a specific menu item.
 * These are organized into arrays per menu state.
 */
typedef struct {
    uint8_t item_index;                 // Selected item index (0xFF = wildcard/default)
    hmi_menu_state_t next_menu;         // Target menu state
    nav_action_type_t action;           // Action to perform
    void (*handler)(void);              // Optional action handler function
} nav_transition_t;

/**
 * @brief Menu metadata for table-driven navigation
 *
 * Describes all navigation properties for a single menu state.
 * Includes the state, item count, and all possible transitions.
 */
typedef struct {
    hmi_menu_state_t state;             // Menu state ID
    uint8_t item_count;                 // Number of selectable items
    const nav_transition_t *transitions; // Array of possible transitions
    uint8_t transition_count;           // Number of transitions in array
    bool is_realtime;                   // True if page needs 4Hz refresh (realtime data)
} nav_menu_metadata_t;

// ########################## Navigation Engine API ##########################

/**
 * @brief Lookup menu metadata by state
 *
 * Searches the menu metadata table to find the configuration for a given
 * menu state. Used by the navigation engine to determine valid transitions.
 *
 * @param menu Menu state to lookup
 * @return Pointer to metadata if found, NULL if menu state is invalid
 */
const nav_menu_metadata_t* hmi_nav_get_menu_metadata(hmi_menu_state_t menu);

/**
 * @brief Find matching transition for a menu + item index
 *
 * Given a menu's metadata and a selected item index, finds the transition
 * that should occur. Supports wildcard matching (0xFF = any item).
 *
 * @param meta Current menu metadata (from hmi_nav_get_menu_metadata)
 * @param selected_item Currently selected item index
 * @return Pointer to matching transition, or NULL if no match found
 */
const nav_transition_t* hmi_nav_find_transition(const nav_menu_metadata_t *meta, uint8_t selected_item);

#endif // HMI_NAVIGATION_H
