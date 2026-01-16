# HMI - Human-Machine Interface

**Author:** Piotr P. <quinkq@gmail.com>
**Part of Solarium** - ESP32-S3 smart irrigation and solar power management system
**Lines of Code:** ~5907 (7 modules + 3 headers)

## Overview

HMI provides complete system monitoring and control through a custom-built graphical interface. Built from scratch without external UI libraries, it demonstrates efficient embedded graphics rendering and event-driven navigation for resource-constrained devices.

**Core Capabilities:**
- 47-state hierarchical menu system with declarative transitions
- Custom 5x7 bitmap font rendering (475 bytes, zero dependencies)
- Variable refresh rates (4Hz realtime monitoring, 1Hz static menus)
- Power-aware operation with auto-sleep and wake-on-input
- Table-driven navigation engine (maintainable, no nested if-else blocks)
- Scrolling support for long menus (>7 items visible)
- Multi-page pagination for complex detail views
- Live editing with confirmation dialogs

**Hardware:**
- SH1106 OLED display (128×64 monochrome, SPI2 shared with ABP sensor)
- EC11 rotary encoder (via MCP23008 I2C GPIO expander)
- 1024-byte framebuffer in SRAM
- 3.3V bus power gating for display/encoder

## Architecture

### Module Structure

```
hmi/
├── hmi.c                      (345 lines)  - Core orchestration, encoder input, FreeRTOS task
├── hmi_display.c              (509 lines)  - SH1106 hardware abstraction, framebuffer, font rendering
├── hmi_navigation.c          (1541 lines)  - Table-driven state machine, ~50 action handlers
├── hmi_render_fluctus.c       (780 lines)  - Power & solar tracking pages (9 pages)
├── hmi_render_tempesta.c      (491 lines)  - Weather sensor pages (8 pages)
├── hmi_render_impluvium.c    (1162 lines)  - Irrigation pages (24 pages: zones, learning, controls)
├── hmi_render_system.c        (582 lines)  - Main menu, STELLARIA (4 pages), SYSTEM (4 pages), confirmation
├── hmi.h                      (150 lines)  - Public API (3 functions)
├── hmi_navigation.h            (96 lines)  - State machine types, transition structures
└── hmi_private.h              (251 lines)  - Shared internal declarations
```

### State Machine (47 states)

**Menu Hierarchy:**
```
MAIN (5 items)
├─ FLUCTUS (9 pages)
│  ├─ Overview (state, temp, SOC)
│  ├─ Energy (hourly/daily stats)
│  ├─ Live Power [REALTIME 4Hz]
│  ├─ Buses (4-bus status)
│  ├─ Solar (3 pages paginated: Tracking/Errors/Sensors) [REALTIME 4Hz]
│  ├─ Servo Debug (manual control selector)
│  │  ├─ Yaw Control [REALTIME 4Hz, 60s timeout]
│  │  └─ Pitch Control [REALTIME 4Hz, 60s timeout]
│  ├─ Controls (solar tracking toggle, safety reset)
│  └─ Intervals (power day/night, solar correction)
│
├─ TEMPESTA (8 pages)
│  ├─ Sensors (4 pages paginated: Environment/Wind/Rain+Tank/Air)
│  ├─ Diagnostics [REALTIME 4Hz, 90s timeout]
│  ├─ Controls (enable/disable, force collection, resets)
│  └─ Intervals (normal, power save)
│
├─ IMPLUVIUM (24 pages)
│  ├─ Overview & Stats (3 pages paginated: Overview/Z1-3/Z4-5)
│  ├─ Monitor [REALTIME 4Hz] (state, pressure, flow, pump)
│  ├─ Zones (7 pages: All Zones + Zone 1-5 detail)
│  ├─ Learning (7 pages: All Zones + Zone 1-5 detail)
│  ├─ Controls (enable/disable, force check, resets)
│  ├─ Zone Config (5 zones selector)
│  │  ├─ Zone Edit (enable/target/deadband/manual water)
│  │  └─ Manual Water Input (5-300s)
│  └─ Intervals (optimal/cool/power save/night min)
│
├─ STELLARIA (4 pages)
│  ├─ Status (state, intensity, flags)
│  ├─ Control (manual intensity adjustment)
│  └─ Auto (light sensor thresholds)
│
└─ SYSTEM (4 pages)
   ├─ Info (WiFi RSSI, uptime, version)
   ├─ Controls (flush & reset, WiFi reconnect)
   └─ Intervals (global presets: Aggressive/Balanced/Conservative)

CONFIRM (shared dialog)
   └─ Yes/No confirmation for destructive actions
```

**State Transitions:**
- Declarative transition tables (one per menu)
- 6 action types: NAVIGATE / EDIT_START / EDIT_CONFIRM / EDIT_CANCEL / TOGGLE / EXECUTE / CONFIRM_DIALOG
- Wildcard matching (0xFF = any item triggers same action)
- Example: All detail pages have single transition (any button → back to parent)

### Task Communication

**FreeRTOS Task:**
- `hmi_display_task` (Med-5): Display refresh, encoder polling, timeout management
- Priority rationale: Medium-5 for responsive UI (encoder must feel instant, display updates need to be smooth)

**Synchronization:**
- `xHmiMutex`: Protects `hmi_status` and editing state variables
- 100ms timeout standard (prevents deadlock, acceptable for UI latency)
- Task notifications: Button press events from MCP23008 ISR callback

**Encoder Input Flow:**
1. User rotates encoder → MCP23008 interrupt fires
2. `mcp23008_helper` ISR reads GP0/GP1, updates quadrature state
3. `hmi_display_task` polls encoder count (no ISR for rotation, only button)
4. Delta calculated → `hmi_handle_encoder_change(delta)` dispatched
5. Action handler function executes (navigation, editing, etc.)

**Button Input Flow:**
1. User presses button → MCP23008 interrupt fires
2. `mcp23008_helper` invokes registered callback → sends task notification
3. `hmi_display_task` wakes from notification
4. `hmi_handle_button_press()` dispatched via navigation engine
5. Action handler function executes based on current menu + selected item

### Initialization & Startup

**Default States:**
- Display: **Powered off** (deployment safety - saves power until user interaction)
- Menu state: **MAIN** (starts at top level)
- Selected item: **0** (first item)
- Encoder count: **0** (will be synced from MCP23008 on first read)

**Initialization Order Dependency:**
- MCP23008_HELPER must initialize first (provides encoder/button hardware)
- FLUCTUS must initialize before HMI (provides 3.3V bus power infrastructure)
- TELEMETRY must be ready (HMI pulls cached snapshots for rendering)
- All component modules must initialize before HMI menus work (HMI controls call component APIs)

**First Boot Behavior:**
- Display remains off (saves power)
- Encoder rotation or button press wakes display
- Powers 3.3V bus via FLUCTUS reference counting
- Renders MAIN menu with blink indicator
- Starts 30-second inactivity timeout

## Hardware Configuration

### GPIO Allocation

| GPIO | Function | Type | Notes |
|------|----------|------|-------|
| GPIO13 | SH1106 CS | SPI2 CS | Chip select (shared SPI2 with ABP) |
| GPIO14 | SH1106 DC | Push-pull 3.3V | Data/Command select |
| GPIO15 | SPI2 MOSI | SPI2 MOSI | Shared with ABP sensor |
| GPIO16 | SPI2 SCK | SPI2 SCK | Shared with ABP sensor |

### MCP23008 I2C GPIO Expander @ 0x20

**Encoder Signals:**
- GP0: EC11 Phase A (quadrature)
- GP1: EC11 Phase B (quadrature)
- GP2: EC11 Button (active low, internal pull-up)

**Interrupt:**
- MCP23008 INT → GPIO8 (falling edge)
- Encoder rotations processed via polling (no per-rotation ISR)
- Button presses trigger ISR callback → task notification

### Display Specifications

- Model: SH1106 OLED (not SSD1306)
- Resolution: 128×64 monochrome
- Interface: SPI2 (4-wire: SCK, MOSI, CS, DC)
- Rotation: 180° (configurable via `HMI_DISPLAY_ROTATION`)
- Framebuffer: 1024 bytes (128×64 ÷ 8 bits/pixel)
- Refresh: Full 1KB transfer per frame (~8ms @ 10MHz SPI)

## Key Features

### 1. Table-Driven Navigation Engine

**Declarative Transition Tables:**
```c
// Example: FLUCTUS submenu transitions
static const nav_transition_t fluctus_menu_transitions[] = {
    { .item_index = 0, .next_menu = HMI_MENU_MAIN,               .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 1, .next_menu = HMI_MENU_FLUCTUS_OVERVIEW,   .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    { .item_index = 2, .next_menu = HMI_MENU_FLUCTUS_ENERGY,     .action = NAV_ACTION_NAVIGATE, .handler = NULL },
    // ... 9 items total
};
```

**Menu Metadata:**
```c
typedef struct {
    hmi_menu_state_t state;          // Menu state ID
    uint8_t item_count;              // Number of selectable items
    const nav_transition_t *transitions; // Array of possible transitions
    uint8_t transition_count;        // Number of transitions
    bool is_realtime;                // True if page needs 4Hz refresh
} nav_menu_metadata_t;
```

**Navigation Dispatch:**
1. User presses button → `hmi_handle_button_press()`
2. Lookup current menu metadata: `hmi_nav_get_menu_metadata(current_menu)`
3. Find matching transition: `hmi_nav_find_transition(meta, selected_item)`
4. Execute action type:
   - `NAV_ACTION_NAVIGATE`: Change menu state, reset selection
   - `NAV_ACTION_EDIT_START`: Enter edit mode, call handler to load value
   - `NAV_ACTION_TOGGLE`: Call handler, stay on same page
   - `NAV_ACTION_EXECUTE`: Call handler, navigate to next_menu
   - `NAV_ACTION_CONFIRM_DIALOG`: Store action + params, show confirmation

**Benefit:** Adding new menu requires 3 steps (no 500-line switch statement):
1. Add enum to `hmi_menu_state_t`
2. Create transition table
3. Add entry to master metadata table

### 2. Custom Font Rendering

**Font Data Structure:**
```c
// 5×7 bitmap font, 95 characters (ASCII 32-126)
// Each character: 5 bytes (5 columns × 7 rows packed into bits)
static const uint8_t font_5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space (32)
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    // ... 95 characters × 5 bytes = 475 bytes total
};
```

**Rendering Pipeline:**
```c
void hmi_fb_draw_char(int x, int y, char c, bool inverted);
void hmi_fb_draw_string(int x, int y, const char *str, bool inverted);
```
- Column-major unpacking (each byte = 1 column × 7 rows)
- Character spacing: 6 pixels (5px glyph + 1px gap)
- Line height: 8 pixels (7px font + 1px gap)
- Inverted mode: Black text on white background (for selection highlighting)

**Design Rationale:**
- U8g2 library: 50KB+ flash, heavyweight API for simple text
- LVGL: 200KB+ flash, overkill for monochrome 128×64
- Custom font: 475 bytes, full control, zero dependencies

### 3. Variable Refresh Rates

**Two Refresh Modes:**
- **Static (1Hz):** Normal menus, settings pages, selectors
  - Refresh interval: 1000ms (every 1 second)
  - Timeout: 30 seconds of inactivity
  - Power: ~25mA continuous (display + encoder)

- **Realtime (4Hz):** Live data monitoring, servo control, diagnostics
  - Refresh interval: 250ms (every 0.25 seconds)
  - Timeout: 60 seconds of inactivity (longer for observation)
  - Power: Same (~25mA continuous, refresh rate doesn't change power)

**Realtime Pages (15 total):**
```c
// FLUCTUS (3)
HMI_MENU_FLUCTUS_LIVE_POWER          // Battery/solar inst + avg
HMI_MENU_FLUCTUS_SOLAR               // Tracking state, servo positions, photoresistors
HMI_MENU_FLUCTUS_SERVO_CONTROL_YAW   // Manual yaw servo (60s timeout)
HMI_MENU_FLUCTUS_SERVO_CONTROL_PITCH // Manual pitch servo (60s timeout)

// TEMPESTA (1)
HMI_MENU_TEMPESTA_DIAG               // Wind sensors, photoresistors (90s timeout)

// IMPLUVIUM (1)
HMI_MENU_IMPLUVIUM_MONITOR           // Watering state, flow, pressure, pump duty
```

**Blink Indicator:**
- Top-left corner: `[LIVE]` text blinks at 2Hz (500ms cycle)
- Implemented via counter: `blink_counter++` every 250ms → toggle `blink_state` every 2 ticks
- Visual feedback that page is actively refreshing

### 4. Scrolling & Pagination

**Scrolling (Vertical, >7 items):**
- Max visible items: 7 (fits in 54px content area at 7px spacing)
- Algorithm: Window with wraparound
  - If `selected_item < scroll_offset`: Scroll up (show earlier items)
  - If `selected_item >= scroll_offset + 7`: Scroll down (show later items)
- Indicators: ▲ (top-right) and ▼ (bottom-right) when scrollable
- Per-menu memory: `menu_scroll_memory[47]` remembers position when navigating away

**Pagination (Horizontal, multi-page detail views):**
- Used for: FLUCTUS Solar (3 pages), TEMPESTA Sensors (4 pages), IMPLUVIUM Overview (3 pages)
- Navigation: Encoder rotation cycles through pages (wraps at ends)
- Indicator: ↔ symbol (top-right corner)
- Shared state: `hmi_status.current_page` and `total_pages`

**Design Rationale:**
- Scrolling: For selectable lists (user needs to choose an item)
- Pagination: For read-only views (user browsing grouped data)
- Both: Separate visual indicators (▲▼ vs ↔) prevent confusion

### 5. Live Editing with Confirmation

**Edit Mode Pattern:**
```c
// State variables (example: STELLARIA intensity)
bool stellaria_intensity_editing = false;
uint8_t stellaria_intensity_percent = 50;  // Editing buffer

// Action handler sequence:
1. NAV_ACTION_EDIT_START: Load value, set flag, enter edit mode
2. Encoder rotation: Modify value (±1 step, constrained to range)
3. NAV_ACTION_EDIT_CONFIRM: Apply value via component API, clear flag
4. NAV_ACTION_EDIT_CANCEL: Discard changes, clear flag
```

**Editable Values:**
- FLUCTUS intervals (power day/night, solar correction)
- TEMPESTA intervals (normal, power save)
- IMPLUVIUM intervals (optimal/cool/power save/night min)
- IMPLUVIUM zone config (target moisture, deadband, enable/disable)
- IMPLUVIUM manual water duration (5-300s)
- STELLARIA intensity (0-100%, 5% steps)
- SYSTEM interval presets (Aggressive/Balanced/Conservative)

**Confirmation Dialog:**
```c
typedef enum {
    CONFIRM_NONE = 0,
    CONFIRM_RESET_ZONE_LEARNING,
    CONFIRM_RESET_ALL_LEARNING,
    CONFIRM_MANUAL_WATER,
    CONFIRM_TEMPESTA_RESET_DAILY,
    CONFIRM_TEMPESTA_RESET_WEEKLY,
    // ... 9 total actions
} confirm_action_t;
```
- Shared `HMI_MENU_CONFIRM` state for all destructive actions
- Stores: Action type, parameter (e.g., zone ID), return menu
- Renders: "Confirm action? Yes / No"
- Yes → Execute action, return to origin menu
- No → Cancel, return to origin menu

### 6. Power Management

**Power Gating:**
```c
// hmi.c - Display power on/off
esp_err_t hmi_display_power_on(void) {
    fluctus_request_bus_power(POWER_BUS_3V3, "HMI");
    // ... SPI init, display init ...
    hmi_status.display_active = true;
}

esp_err_t hmi_display_power_off(void) {
    // ... display sleep command ...
    fluctus_release_bus_power(POWER_BUS_3V3, "HMI");
    hmi_status.display_active = false;
}
```

**Timeout Management:**
- 30 seconds: Normal menus (static data, user finished reading)
- 60 seconds: Realtime pages (live monitoring, user observing trends)
- 60 seconds: Servo control (safety timeout for manual actuation)
- 90 seconds: TEMPESTA diagnostics (sensor debugging, longer observation)

**Wake-on-Input:**
- Encoder rotation or button press wakes display from off state
- MCP23008 button callback sends task notification
- Display powers on, updates activity timestamp, resumes at last menu

**Special Timeouts:**
```c
// Servo control: 60-second safety timeout
if (servo_debug_active) {
    time_t now = time(NULL);
    if ((now - servo_debug_start_time) >= 60) {
        // Auto-exit, release 6.6V bus power
        hmi_action_exit_servo_control();
    }
}
```

### 7. Component Integration Controls

**FLUCTUS Controls:**
- Solar tracking: Enable/Disable toggle (sends `fluctus_enable_solar_tracking()`)
- Solar debug: 90-second continuous correction mode
- Servo manual control: Direct PWM duty adjustment (410-2048 range, ±50 steps)
- Safety reset: Clear overcurrent lockout via `fluctus_manual_safety_reset()`
- Intervals: Power monitoring (day/night), solar correction (5-60 min)

**TEMPESTA Controls:**
- System enable/disable: Toggle via `tempesta_set_system_enabled()`
- Force collection: Immediate sensor read via `tempesta_force_collection()`
- Diagnostics: 90-second mode with hall array enable (14mA draw)
- Resets: Daily, weekly, rain total, tank total, all accumulators
- Intervals: Normal (5-60 min), power save (15-120 min)

**IMPLUVIUM Controls:**
- System enable/disable: Toggle via `impluvium_set_system_enabled()`
- Force moisture check: Trigger immediate scan via `impluvium_force_moisture_check()`
- Zone config: Enable/disable, target moisture (20-80%), deadband (1-20%)
- Manual water: 5-300 seconds (5s increments), per-zone
- Learning reset: Per-zone or all zones via `impluvium_reset_zone_learning()`
- Emergency recovery: Clear emergency stop, clear diagnostics bitmask
- Intervals: Optimal/cool/power save (5-120 min), night min (1-6 hours)

**STELLARIA Controls:**
- System enable/disable: Toggle via `stellaria_set_system_enabled()`
- Intensity: 0-100% (5% steps) via `stellaria_set_manual_intensity()`
- Auto mode: Toggle via `stellaria_set_auto_mode()` (light sensor control)

**SYSTEM Controls:**
- MQTT flush & reset: `telemetry_flush_and_reset_mqtt_buffer()`
- WiFi reconnect: `wifi_helper_force_reconnect()`
- Interval presets: Aggressive / Balanced / Conservative (applies to all components)

## Public API

### Initialization
```c
esp_err_t hmi_init(void);
// Initializes display hardware, encoder input, creates FreeRTOS task
// Prerequisites: MCP23008_HELPER, FLUCTUS, TELEMETRY must be initialized first
```

### Status Query
```c
esp_err_t hmi_get_status(hmi_status_t *status);
// Thread-safe status snapshot (100ms mutex timeout)
// Returns: Current menu, selected item, display active, encoder count, etc.
```

### Display Control
```c
esp_err_t hmi_wake_display(void);
// Manually wakes display (resets timeout)
// Use case: External event wants to show alert (e.g., irrigation started)
```

## Data Structures

### `hmi_status_t`

Core HMI state (120 bytes):
```c
typedef struct {
    bool initialized;                 // Component initialization status
    bool display_active;              // Display powered on
    hmi_menu_state_t current_menu;    // Current menu state (0-46)
    uint8_t selected_item;            // Currently selected menu item (0-based)
    time_t last_activity_time;        // Last user interaction timestamp (for timeout)
    int16_t encoder_count;            // Current encoder count (detent-based)
    bool blink_state;                 // Blinking indicator state for [LIVE] pages
    uint32_t blink_counter;           // Counter for blink timing (250ms toggles)

    // Scrolling state
    uint8_t scroll_offset;            // First visible item index (0-based)
    uint8_t menu_scroll_memory[47];   // Per-menu scroll position memory
    uint8_t menu_selected_item_memory[47]; // Per-menu selected item memory

    // Pagination state
    uint8_t current_page;             // Current page index (0-based)
    uint8_t total_pages;              // Total pages for current view
} hmi_status_t;
```

### `nav_transition_t`

Single state transition definition:
```c
typedef struct {
    uint8_t item_index;               // Selected item index (0xFF = wildcard)
    hmi_menu_state_t next_menu;       // Target menu state
    nav_action_type_t action;         // Action to perform (6 types)
    void (*handler)(void);            // Optional action handler function
} nav_transition_t;
```

### `nav_menu_metadata_t`

Menu metadata for table-driven navigation:
```c
typedef struct {
    hmi_menu_state_t state;           // Menu state ID
    uint8_t item_count;               // Number of selectable items
    const nav_transition_t *transitions; // Array of possible transitions
    uint8_t transition_count;         // Number of transitions in array
    bool is_realtime;                 // True if page needs 4Hz refresh
} nav_menu_metadata_t;
```

## Integration

### Component Dependencies (CMakeLists.txt)
```c
REQUIRES
    fluctus          // Power bus management (3.3V gating)
    telemetry        // Cached data reads for all components
    tempesta         // Weather data, control API
    impluvium        // Irrigation data, control API
    stellaria        // Lighting data, control API
    wifi_helper      // WiFi status, control API
    interval_config  // Centralized interval configuration
    mcp23008_helper  // Encoder/button input
```

### Component Interactions

**TELEMETRY (data reads):**
```c
// Fast memcpy snapshots (~1-2μs), no mutex contention
fluctus_snapshot_t fluctus_data;
telemetry_get_fluctus_data(&fluctus_data);

tempesta_snapshot_t tempesta_data;
telemetry_get_tempesta_data(&tempesta_data);

impluvium_snapshot_t impluvium_data;
telemetry_get_impluvium_data(&impluvium_data);
```

**FLUCTUS (power + control):**
```c
// Power gating
fluctus_request_bus_power(POWER_BUS_3V3, "HMI");
fluctus_release_bus_power(POWER_BUS_3V3, "HMI");

// Solar tracking control
fluctus_enable_solar_tracking();
fluctus_disable_solar_tracking();
fluctus_enable_solar_debug_mode();  // 90s continuous

// Servo manual control (debug mode)
fluctus_request_bus_power(POWER_BUS_6V6, "HMI_SERVO_DEBUG");
fluctus_servo_debug_set_position(LEDC_CHANNEL_YAW, duty);
fluctus_release_bus_power(POWER_BUS_6V6, "HMI_SERVO_DEBUG");
```

**TEMPESTA (control):**
```c
tempesta_set_system_enabled(bool enable);
tempesta_force_collection();
tempesta_enable_debug_mode();  // 90s hall array + photoresistor debug
```

**IMPLUVIUM (control):**
```c
impluvium_set_system_enabled(bool enable);
impluvium_force_moisture_check();
impluvium_force_water_zone(zone_id, duration_sec);
impluvium_update_zone_config(zone_id, target, deadband, enabled);
impluvium_reset_zone_learning(zone_id);
impluvium_clear_emergency_stop();
```

**STELLARIA (control):**
```c
stellaria_set_system_enabled(bool enable);
stellaria_set_manual_intensity(percent);
stellaria_set_auto_mode(bool enable);
```

**MCP23008_HELPER (input):**
```c
// Encoder reading (polling, not ISR)
int32_t count = mcp23008_helper_get_encoder_count();

// Button callback registration (ISR-based)
mcp23008_helper_register_button_callback(hmi_button_callback);
```

**INTERVAL_CONFIG (settings):**
```c
// Read intervals for display
interval_config_get(INTERVAL_FLUCTUS_POWER_DAY, &value_ms);

// Update intervals from HMI edits
fluctus_set_power_intervals(day_min, night_min);
tempesta_set_intervals(normal_min, power_save_min);
impluvium_set_check_intervals(optimal, cool, power_save, night_hours);
```

## Performance

**Memory:**
- Framebuffer: 1024 bytes (128×64 ÷ 8 bits/pixel)
- Font data: 475 bytes (5×7 bitmap, 95 characters)
- Status struct: ~120 bytes (menu state, scroll memory, editing state)
- Transition tables: ~3 KB (47 menus × ~50 bytes avg metadata)
- Total SRAM: ~5 KB (excluding task stacks)

**Task Stacks:**
- `hmi_display_task`: 4096 bytes

**Timing:**
- Encoder read: <1μs (I2C already polled by mcp23008_helper)
- Telemetry fetch: 1-2μs (fast memcpy from cache)
- Framebuffer render: 5-20ms (depends on complexity: text vs full redraw)
- SPI transfer: ~8ms (1024 bytes @ 10MHz SPI2)
- Total frame time: 10-30ms (33-100 FPS capable, throttled to 1-4Hz)

**Refresh Rates:**
- Static menus: 1Hz (1000ms refresh interval)
- Realtime pages: 4Hz (250ms refresh interval)
- Blink indicator: 2Hz (500ms toggle cycle)

**Power Consumption:**
- Display active: ~25mA @ 3.3V (~82mW)
- Encoder idle: <1mA (MCP23008 with pull-ups)
- Display off: ~100μA (MCP23008 quiescent + GPIO leakage)
- Duty cycle: Typically 2-5% (30-60s active per 30-minute cycle)

## Configuration

Key defines in `hmi.h`:

```c
// Display hardware
#define HMI_DISPLAY_CS_GPIO     GPIO_NUM_13
#define HMI_DISPLAY_DC_GPIO     GPIO_NUM_14
#define HMI_DISPLAY_SPI_HOST    SPI2_HOST

// Display specifications
#define HMI_DISPLAY_WIDTH       128
#define HMI_DISPLAY_HEIGHT      64
#define HMI_DISPLAY_ROTATION    180  // 0, 90, 180, or 270 degrees

// Timing configuration
#define HMI_DISPLAY_TIMEOUT_NORMAL_MS   (30 * 1000)  // 30 seconds
#define HMI_DISPLAY_TIMEOUT_REALTIME_MS (60 * 1000)  // 60 seconds
#define HMI_REFRESH_RATE_NORMAL_MS      1000         // 1Hz
#define HMI_REFRESH_RATE_REALTIME_MS    250          // 4Hz

// Scrolling
#define HMI_MAX_VISIBLE_ITEMS   7        // Fits in 54px content area
#define HMI_MENU_START_Y        14       // Y-coord after title + divider
#define HMI_MENU_ITEM_SPACING   7        // Vertical spacing between items
```

## Design Patterns

**Table-Driven State Machine**: Navigation uses declarative transition tables instead of a 500-line switch statement. Each menu has a table of `{item_index, next_menu, action, handler}` tuples. The navigation engine looks up the current menu's metadata, finds the matching transition for the selected item, and dispatches the action. Adding a new menu requires 3 edits (enum, transition table, metadata entry) instead of touching dozens of case statements.

**Module-Based Rendering Dispatch**: The 5907-line codebase is split into 4 rendering modules (fluctus/tempesta/impluvium/system), each handling 8-24 pages. The main dispatcher calls `hmi_render_fluctus_pages()` / `hmi_render_tempesta_pages()` / etc. based on `current_menu`. This prevents one giant rendering function and makes the codebase navigable - you can understand FLUCTUS pages without reading IMPLUVIUM code.

**Framebuffer Encapsulation**: All rendering goes through framebuffer functions (`hmi_fb_clear`, `hmi_fb_draw_string`, `hmi_fb_flush`). Hardware-specific code (SPI transactions, SH1106 commands) lives only in `hmi_display.c`. Rendering modules call `hmi_fb_*` functions and never touch SPI directly. This makes porting to a different display (e.g., SSD1306) a single-file change.

**Variable Refresh Rate by Metadata**: Each menu has an `is_realtime` flag in its metadata. The display task checks `hmi_is_realtime_page()` to decide 250ms vs 1000ms refresh interval and 60s vs 30s timeout. This avoids hardcoding page IDs in multiple places - adding a new realtime page just requires setting `is_realtime: true` in the metadata table.

**Per-Menu Navigation Memory**: Scrolling and selection state is preserved in `menu_scroll_memory[47]` and `menu_selected_item_memory[47]`. When you navigate from FLUCTUS Overview → Main → TEMPESTA → back to FLUCTUS, it remembers you were on "Overview" (item 1) and restores the selection. This matches user expectations for hierarchical menu systems.

**Confirmation Dialog with Context**: Destructive actions (reset learning, manual water, flush MQTT buffer) show a shared `HMI_MENU_CONFIRM` dialog. The action type, parameter (e.g., zone ID), and return menu are stored in global state. "Yes" executes the action and returns to origin. This avoids duplicating confirmation UI logic across 9 different action types.

**Event-Driven Wakeup**: Button presses use MCP23008 ISR callback → task notification → display power-on. Encoder rotations are polled (no per-rotation ISR). This hybrid approach minimizes ISR overhead (only 1 ISR per button press) while keeping encoder response instant (250ms polling is imperceptible for human rotation speeds).

---

**Related Files:**
- Integration: `main/main.c`
- Encoder/button input: `components/helpers/mcp23008_helper/mcp23008_helper.c`
- Telemetry data fetch: `components/core/telemetry/telemetry_cache.c`
- Power management: `components/core/fluctus/fluctus_power_bus.c`
