# INTERVAL_CONFIG - Centralized Check Interval Management

**Author:** Piotr P. <quinkq@gmail.com>
**Part of Solarium** - ESP32-S3 smart irrigation and solar power management system
**Lines of Code:** ~604 (2 modules)

## Overview

Interval_config manages user-adjustable check intervals for all core components (FLUCTUS, TEMPESTA, IMPLUVIUM) with CRC32-protected LittleFS persistence. It provides a global configuration singleton with 3 predefined presets and runtime adjustment API, eliminating per-component configuration duplication.

**Core capabilities:**
- Global interval storage with automatic LittleFS persistence
- Three presets: Aggressive (5min), Balanced (15min), Conservative (30min)
- Range validation per component (5-120 minutes depending on interval type)
- CRC32 integrity checking on load/save
- Runtime adjustment API with immediate persistence
- Preset detection (identifies which preset is active or if values are custom)

## Architecture

### Module Structure

```
interval_config/
├── interval_config.h    (215 lines)  - Public API, range definitions, data structures
└── interval_config.c    (389 lines)  - Implementation, persistence, validation
```

### Initialization & Startup

**Default State:**
- Balanced preset (15min day, 60min night intervals)
- No file → creates `/data/intervals.dat` with defaults
- Corrupted file → logs error, restores defaults, overwrites file

**Initialization Order Dependency:**
- Must initialize **before** core components (FLUCTUS/TEMPESTA/IMPLUVIUM)
- Components read `g_interval_config` global during their own initialization
- Changes after init require component-specific setter calls (e.g., `fluctus_set_power_intervals()`)

### Data Flow

```
Boot
  └─→ interval_config_init()
      ├─→ Load /data/intervals.dat (40 bytes + CRC32)
      ├─→ Validate CRC32 + ranges
      └─→ Populate g_interval_config global

Component Init (FLUCTUS/TEMPESTA/IMPLUVIUM)
  └─→ Read g_interval_config.<field>
  └─→ Convert minutes → milliseconds for timers

Runtime Adjustment (HMI or future MQTT)
  └─→ interval_config_set_*()
      ├─→ Validate ranges
      ├─→ Update g_interval_config
      ├─→ Calculate CRC32
      ├─→ Write to LittleFS (atomic)
      └─→ Caller must notify component tasks

Component Task Notification
  └─→ FLUCTUS: xTaskNotify(FLUCTUS_NOTIFY_CONFIG_UPDATE) for power monitoring
  └─→ TEMPESTA: xTimerChangePeriod() for collection timer
  └─→ IMPLUVIUM: Dynamic recalculation on next moisture check
```

**Storage Safety:**
- 40-byte structure + 4-byte CRC32
- Atomic write (fwrite → fclose)
- Invalid values → refuses save, returns ESP_ERR_INVALID_ARG
- Corrupted load → auto-restore defaults, overwrites file

## Hardware Configuration

No hardware dependencies. Pure software configuration layer.

## Key Features

### 1. Range-Validated Intervals

**FLUCTUS Power Monitoring:**
- Day: 5-60 minutes (default: 15)
- Night: 15-120 minutes (default: 60)
- Solar correction: 5-60 minutes (default: 15)

**TEMPESTA Weather Collection:**
- Normal mode: 5-60 minutes (default: 15)
- Power save mode: 15-120 minutes (default: 60)

**IMPLUVIUM Moisture Checks:**
- Optimal temp (≥20°C): 5-60 minutes (default: 15)
- Cool temp (10-20°C): 10-90 minutes (default: 30)
- Power save mode: 30-120 minutes (default: 60)
- Nighttime minimum: 1-6 hours (default: 3)

**Validation:**
```c
bool interval_config_validate(const interval_config_t *config);
```
- Checks all 9 fields against min/max constants
- Logs specific field violations with ESP_LOGW
- Called automatically before every save
- Returns false → save operation aborts with ESP_ERR_INVALID_ARG

### 2. CRC32 Integrity Protection

**Algorithm:**
- `esp_crc32_le()` over first 36 bytes (9 × uint32_t fields, excluding CRC itself)
- Calculated on save, validated on load
- Mismatch → treats file as corrupted, restores defaults

**Failure Recovery:**
- Size mismatch: File too small/large → defaults
- CRC mismatch: Corrupted data → defaults
- Range violation: Valid CRC but out-of-bounds values → defaults
- All failures: Log error, restore defaults, **overwrite file** (self-healing)

### 3. Three Predefined Presets

**Aggressive (High power, fast response):**
- FLUCTUS: 5min / 30min / 5min
- TEMPESTA: 5min / 30min
- IMPLUVIUM: 5min / 10min / 30min / 1h

**Balanced (Default, recommended):**
- FLUCTUS: 15min / 60min / 15min
- TEMPESTA: 15min / 60min
- IMPLUVIUM: 15min / 30min / 60min / 3h

**Conservative (Battery saver):**
- FLUCTUS: 30min / 120min / 30min
- TEMPESTA: 30min / 120min
- IMPLUVIUM: 30min / 60min / 120min / 6h

**Preset Detection:**
```c
interval_preset_t interval_config_get_current_preset(void);
```
- Compares `g_interval_config` against all presets
- Returns INTERVAL_PRESET_CUSTOM if no match
- Used by HMI to display "Aggressive/Balanced/Conservative/Custom"

### 4. Atomic Persistence

**File Path:** `/data/intervals.dat` (LittleFS partition)

**Write Operation:**
```c
esp_err_t interval_config_save(void)
{
    if (!interval_config_validate(&g_interval_config)) {
        return ESP_ERR_INVALID_ARG;  // Refuse invalid data
    }
    g_interval_config.crc32 = calculate_crc32(&g_interval_config);
    // ... fopen/fwrite/fclose ...
}
```
- Pre-validates before touching filesystem
- Atomic write (single fwrite call)
- Returns ESP_FAIL if write incomplete
- Automatically called by all setter functions

### 5. Global Singleton Pattern

**Declaration:**
```c
extern interval_config_t g_interval_config;
```

**Access Pattern:**
- Components read directly: `uint32_t interval = g_interval_config.fluctus_power_day_min;`
- No getter functions (zero overhead)
- Setter functions handle validation + persistence
- Thread-safe: Only modified at init and during HMI/MQTT commands (low frequency)

## Public API

### Initialization

```c
esp_err_t interval_config_init(void);
```
Loads configuration from LittleFS. Must be called before core components initialize.

### Preset Management

```c
esp_err_t interval_config_apply_preset(interval_preset_t preset);
interval_preset_t interval_config_get_current_preset(void);
const char* interval_config_get_preset_name(interval_preset_t preset);
```
Apply or detect presets. Returns "Aggressive", "Balanced", "Conservative", or "Custom".

### Component-Specific Setters

```c
esp_err_t interval_config_set_fluctus_power(uint32_t day_min, uint32_t night_min);
esp_err_t interval_config_set_fluctus_solar(uint32_t correction_min);
esp_err_t interval_config_set_tempesta(uint32_t normal_min, uint32_t power_save_min);
esp_err_t interval_config_set_impluvium(uint32_t optimal_min, uint32_t cool_min,
                                        uint32_t power_save_min, uint32_t night_hours);
```
All setters validate ranges, update global, save to LittleFS. Return ESP_ERR_INVALID_ARG if out of range.

### Validation & Persistence

```c
bool interval_config_validate(const interval_config_t *config);
esp_err_t interval_config_save(void);
void interval_config_reset_defaults(void);
```
Manual validation, save (rarely needed), or factory reset.

## Data Structures

### `interval_config_t`

```c
typedef struct {
    // FLUCTUS Power Monitoring (2 intervals)
    uint32_t fluctus_power_day_min;          // 5-60 min
    uint32_t fluctus_power_night_min;        // 15-120 min

    // FLUCTUS Solar Tracking (1 interval)
    uint32_t fluctus_solar_correction_min;   // 5-60 min

    // TEMPESTA Weather Collection (2 intervals)
    uint32_t tempesta_normal_min;            // 5-60 min
    uint32_t tempesta_power_save_min;        // 15-120 min

    // IMPLUVIUM Moisture Checks (4 intervals)
    uint32_t impluvium_optimal_min;          // 5-60 min (temp ≥20°C)
    uint32_t impluvium_cool_min;             // 10-90 min (10-20°C)
    uint32_t impluvium_power_save_min;       // 30-120 min
    uint32_t impluvium_night_min_hours;      // 1-6 hours

    uint32_t crc32;                          // Integrity check
} interval_config_t;  // 40 bytes total
```

**Field Units:**
- Minutes for all sub-hourly intervals (reduces user confusion)
- Hours for nighttime minimum only (human-readable, 1-6h vs 60-360min)
- Components convert to milliseconds at runtime

**Memory Footprint:**
- Structure: 40 bytes (9 × uint32_t + 1 × uint32_t CRC)
- Stored on filesystem, loaded into global at boot
- No dynamic allocation

## Integration

### Component Usage

**FLUCTUS (Power & Solar):**
- Reads: `g_interval_config.fluctus_power_day_min`, `fluctus_power_night_min`, `fluctus_solar_correction_min`
- Converts: Minutes → milliseconds for FreeRTOS timers
- Notification-based: `fluctus_set_power_intervals()` calls `xTaskNotify()` for immediate update

**TEMPESTA (Weather):**
- Reads: `g_interval_config.tempesta_normal_min`, `tempesta_power_save_min`
- Converts: Minutes → milliseconds for FreeRTOS timer
- Timer-based: `tempesta_set_collection_intervals()` calls `xTimerChangePeriod()`

**IMPLUVIUM (Irrigation):**
- Reads: `g_interval_config.impluvium_optimal_min`, `cool_min`, `power_save_min`, `night_min_hours`
- Dynamic calculation: Combines temp/time/power state to select active interval
- No explicit notification: Next moisture check applies new configuration

**HMI (User Interface):**
- Displays current preset name via `interval_config_get_current_preset()`
- Provides 5-minute step adjustment for all intervals
- Shows "Custom" label if values don't match any preset
- Calls setter functions on user confirmation

### Future MQTT Integration

**Documented Protocol:**
- `solarium/config/intervals/set` - Individual interval commands (MessagePack)
- `solarium/config/intervals/preset` - Preset application (MessagePack)
- `solarium/config/intervals/ack` - Acknowledgment with current values (MessagePack)

**Command Flow:**
```c
// Pseudo-code (not yet implemented)
mqtt_handle_interval_command(payload, len)
  ├─→ Deserialize MessagePack
  ├─→ Extract component + intervals
  ├─→ Call interval_config_set_*()
  ├─→ Call component setter (fluctus_set_power_intervals, etc.)
  └─→ Publish ACK with status + new values
```

See `MQTT_INTERVAL_COMMANDS.md` for full specification.

## Performance

**Initialization:**
- File read: ~5-10ms (40 bytes from LittleFS)
- CRC32 calculation: ~20μs (36 bytes)
- Validation: ~100μs (9 range checks)

**Runtime Update:**
- Setter call: ~30ms total
  - Validation: ~100μs
  - CRC32: ~20μs
  - File write: ~20-30ms (LittleFS flush)
- Low frequency: Triggered by HMI user action or MQTT command (seconds/minutes between calls)

**Memory:**
- Global structure: 40 bytes (persistent in SRAM)
- File storage: 40 bytes on LittleFS
- No dynamic allocation

## Configuration

### Compile-Time Constants

Range limits defined in `interval_config.h`:

```c
// FLUCTUS
#define INTERVAL_FLUCTUS_POWER_DAY_MIN       5
#define INTERVAL_FLUCTUS_POWER_DAY_MAX       60
#define INTERVAL_FLUCTUS_POWER_NIGHT_MIN     15
#define INTERVAL_FLUCTUS_POWER_NIGHT_MAX     120
#define INTERVAL_FLUCTUS_SOLAR_MIN           5
#define INTERVAL_FLUCTUS_SOLAR_MAX           60

// TEMPESTA
#define INTERVAL_TEMPESTA_NORMAL_MIN         5
#define INTERVAL_TEMPESTA_NORMAL_MAX         60
#define INTERVAL_TEMPESTA_POWER_SAVE_MIN     15
#define INTERVAL_TEMPESTA_POWER_SAVE_MAX     120

// IMPLUVIUM
#define INTERVAL_IMPLUVIUM_OPTIMAL_MIN       5
#define INTERVAL_IMPLUVIUM_OPTIMAL_MAX       60
#define INTERVAL_IMPLUVIUM_COOL_MIN          10
#define INTERVAL_IMPLUVIUM_COOL_MAX          90
#define INTERVAL_IMPLUVIUM_POWER_SAVE_MIN    30
#define INTERVAL_IMPLUVIUM_POWER_SAVE_MAX    120
#define INTERVAL_IMPLUVIUM_NIGHT_MIN         1
#define INTERVAL_IMPLUVIUM_NIGHT_MAX         6
```

### Runtime Configuration

- File path: `/data/intervals.dat` (LittleFS)
- Default preset: Balanced (15min day, 60min night)
- Persistence: Automatic on every setter call
- Factory reset: `interval_config_reset_defaults()` → restores Balanced preset

## Design Patterns

**Global Singleton for System-Wide Configuration**: Uses a single global `g_interval_config` instance instead of per-component configuration files. This eliminates filesystem fragmentation (9 intervals in 1 file vs 3 separate files) and provides atomic updates across component boundaries. Components access fields directly with zero overhead, while setter functions ensure validation and persistence.

**CRC32 Self-Healing Storage**: Detects corrupted configuration on boot (size mismatch, CRC failure, range violations) and automatically restores defaults with file overwrite. This prevents boot loops from malformed data while maintaining user expectations (bad config → safe defaults, not refusal to start). Three-tier validation (size, CRC, ranges) catches all corruption types.

**Preset Detection via Value Comparison**: Compares current configuration against three static preset structures to determine active preset. Returns INTERVAL_PRESET_CUSTOM if values don't match, enabling HMI to show "Custom" label. This approach avoids storing a separate "active_preset" field that could desynchronize from actual values.

**Stateless API with Automatic Persistence**: All setter functions immediately save to LittleFS after validation. No explicit "apply" or "commit" step required. This prevents lost changes from unexpected resets and matches user mental model (set → saved). Validation-before-write ensures filesystem always contains valid data.

**Range Validation with Specific Limits**: Each interval has tailored min/max constants matching component capabilities. For example, IMPLUVIUM nighttime minimum uses 1-6 hours (not minutes) because sub-hourly nighttime checks waste power. FLUCTUS power monitoring minimum is 5min (not 1min) because more frequent readings provide no additional battery state resolution. These domain-specific ranges prevent misconfiguration.

---

**Related Files:**
- Integration: `main/main.c` (initialization order)
- Component setters: `components/core/fluctus/fluctus.h`, `components/core/tempesta/tempesta.h`, `components/core/impluvium/impluvium.h`
- HMI editing: `components/core/hmi/hmi_navigation.c` (interval adjustment actions)
- Future MQTT: `components/helpers/interval_config/MQTT_INTERVAL_COMMANDS.md` (protocol specification)
