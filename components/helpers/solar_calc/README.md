# SOLAR_CALC - Astronomical Time & Event Notification

**Author:** Piotr P. <quinkq@gmail.com>
**Part of Solarium** - ESP32-S3 smart irrigation and solar power management system
**Lines of Code:** ~541 (1 module)

## Overview

SOLAR_CALC provides NOAA-based sunrise/sunset calculations and event-driven time notifications for the Solarium system. It enables components to react to astronomical events (sunrise, midnight) without polling, supporting day/night operational mode switching and daily data rollover routines.

**Core Capabilities:**
- NOAA solar position algorithm with ±5-10 minute accuracy
- Hemisphere-agnostic calculations (handles Northern/Southern hemispheres)
- Event-driven sunrise and midnight callbacks (no polling required)
- Configurable operational buffers (30-minute sunrise/sunset margins)
- Automatic daily recalculation via esp_timer

**Hardware:** None (pure software component)

## Architecture

### Module Structure

```
solar_calc/
├── solar_calc.c       (410 lines)  - NOAA algorithm, event broadcasting, timer management
├── solar_calc.h       (131 lines)  - Public API, callback types, configuration
└── CMakeLists.txt     (8 lines)    - Component build definition
```

### Initialization & Startup

**Default State:**
- Solar times calculated immediately at init (no cached values)
- Midnight timer armed for next local midnight (one-shot esp_timer)
- Sunrise trigger reset (prevents duplicate sunrise events on boot)

**Initialization Order Dependency:**
- Must initialize after SNTP time sync for accurate calculations
- Components can register callbacks anytime after `solar_calc_init()`, but should do so before the first midnight/sunrise event they want to catch

### Task Communication

**No dedicated task** - operates via esp_timer callback:
- `midnight` timer (esp_timer): One-shot timer scheduled at local midnight
- Timer callback broadcasts midnight event → recalculates solar times → broadcasts sunrise if threshold crossed
- All callbacks execute in esp_timer task context (system task, high priority)

**Thread Safety:**
- Solar times cached in static struct (read-only after update)
- Callback arrays managed statically (write-once during init, read-many at runtime)
- No mutexes required (single-writer, multiple-reader pattern with atomic updates)

## Hardware Configuration

None - this is a pure software component. Configuration is compile-time via `#define` in header:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `SOLAR_CALC_LATITUDE` | 54.3521°N | Gdansk, Poland |
| `SOLAR_CALC_LONGITUDE` | 18.6464°E | Gdansk, Poland |
| `SOLAR_CALC_TIMEZONE_OFFSET` | UTC+1 | CET (adjust for DST manually) |
| `SOLAR_CALC_SUNRISE_BUFFER_MINUTES` | 30 min | Pre-sunrise operational window |
| `SOLAR_CALC_SUNSET_BUFFER_MINUTES` | 30 min | Post-sunset operational window |

## Key Features

### 1. NOAA Solar Position Algorithm

**Calculation Method:**
- Julian day → Julian century → Sun's mean anomaly → Equation of center → True longitude → Declination
- Hour angle calculation with atmospheric refraction correction (-0.833° offset)
- Equation of Time (Fourier series) for solar noon correction
- Handles polar day/night conditions (cos_hour_angle out of [-1, 1] range)

**Accuracy:**
- ±5-10 minutes for typical latitudes (sufficient for power management)
- Verified against NOAA Solar Calculator reference implementation

**Hemisphere Support:**
```c
// Latitude sign naturally handles hemisphere in trigonometric calculations
// Northern: positive latitude (0° to +90°)
// Southern: negative latitude (0° to -90°)
double latitude_rad = DEG_TO_RAD(latitude);
double cos_hour_angle = (sin(DEG_TO_RAD(-0.833)) - sin(latitude_rad) * sin(declination)) /
                        (cos(latitude_rad) * cos(declination));
```

**Polar Conditions:**
- Polar night: `sunrise_hour = 0.0, sunset_hour = 0.0` (sun never rises)
- Polar day: `sunrise_hour = 0.0, sunset_hour = 24.0` (sun never sets)

### 2. Buffered Daytime Detection

**Two Query Functions:**

**Exact astronomical times:**
```c
bool solar_calc_is_daytime(void);  // True between sunrise and sunset
```

**Operational buffers applied:**
```c
bool solar_calc_is_daytime_buffered(void);  // True from (sunrise - 30min) to (sunset + 30min)
```

**Buffer Rationale:**
- Solar tracking needs to wake before sunrise to reach correction position
- Component monitoring intervals switch from night to day mode gradually
- Prevents edge-case races where sunrise timestamp equals current time

**Usage Pattern:**
- FLUCTUS solar tracking: Uses buffered check to enable STANDBY → CORRECTING transition
- IMPLUVIUM/TEMPESTA: Use buffered check for interval switching (long night intervals, shorter day intervals)

### 3. Sunrise Notification System

**Callback Registration:**
```c
esp_err_t solar_calc_register_sunrise_callback(solar_calc_sunrise_callback_t callback);
```
- Max 4 sunrise callbacks (small array, no dynamic allocation)
- Callbacks triggered when `time(NULL)` crosses buffered sunrise threshold
- One trigger per day (reset at midnight)

**Triggering Logic:**
1. Midnight timer recalculates solar times
2. `solar_calc_update()` checks: `now >= (sunrise_time - 30min) && !sunrise_triggered_today`
3. If true: broadcast to all subscribers, set `sunrise_triggered_today = true`
4. Flag reset at next midnight

**Use Cases:**
- FLUCTUS: Wakes SLEEPING solar tracking to STANDBY state
- Future: Any component needing day/night mode transitions

### 4. Midnight Notification System

**Callback Registration:**
```c
esp_err_t solar_calc_register_midnight_callback(solar_calc_midnight_callback_t callback);
```
- Max 8 midnight callbacks (supports system-wide daily routines)
- Triggered by one-shot esp_timer at local midnight (00:00:00)

**Event Sequence (every midnight):**
1. Timer fires at 00:00:00 local time
2. Reset `sunrise_triggered_today = false`
3. Call `solar_calc_update()` (recalculates solar times for new day)
4. Broadcast sunrise callbacks if threshold already crossed (edge case: init after sunrise)
5. Broadcast midnight callbacks to all subscribers
6. Reschedule next midnight timer (one-shot, +24 hours)

**Use Cases:**
- FLUCTUS: Daily energy accumulator rollover
- IMPLUVIUM: Daily learning history advancement
- Future: Log rotation, NVS backup

### 5. Fast Time Conversion

**Avoids mktime() Bugs:**
```c
// Direct arithmetic prevents timezone/DST issues
time_t day_start = (base_time / 86400) * 86400;  // Truncate to UTC midnight
return day_start + (time_t)(hours * 3600.0);      // Add decimal hours
```

**Problem Solved:**
- `mktime()` applies timezone twice (once in struct tm, once in conversion)
- DST transitions cause off-by-one-hour errors
- Direct arithmetic guarantees UTC consistency

## Public API

### Initialization

```c
void solar_calc_init(void);
```
- Calculates initial solar times
- Creates midnight esp_timer
- Schedules first midnight event

### Time Queries

```c
const solar_times_t* solar_calc_get_times(void);
bool solar_calc_is_daytime(void);               // Exact astronomical
bool solar_calc_is_daytime_buffered(void);      // With 30-min margins
void solar_calc_update(void);                    // Force recalculation
```

### Event Notifications

```c
// Sunrise callbacks (max 4)
esp_err_t solar_calc_register_sunrise_callback(solar_calc_sunrise_callback_t callback);
void solar_calc_broadcast_sunrise(void);  // Internal use

// Midnight callbacks (max 8)
esp_err_t solar_calc_register_midnight_callback(solar_calc_midnight_callback_t callback);
void solar_calc_broadcast_midnight(void);  // Internal use
```

**Callback Context:**
- Execute in esp_timer task (high priority)
- Should be fast, non-blocking
- Avoid long I2C/SPI operations in callback

## Data Structures

### `solar_times_t`

```c
typedef struct {
    time_t sunrise_time;    // Unix timestamp (seconds since epoch, UTC)
    time_t sunset_time;     // Unix timestamp (seconds since epoch, UTC)
    bool is_daytime;        // Cached daytime status (updated by solar_calc_update())
    time_t last_update;     // Timestamp of last recalculation
} solar_times_t;
```

**Update Frequency:** Once per day at midnight (or manual call to `solar_calc_update()`)
**Memory:** 20 bytes (4 × time_t + 1 × bool + 3 padding)

### Callback Types

```c
typedef void (*solar_calc_sunrise_callback_t)(void);   // No parameters
typedef void (*solar_calc_midnight_callback_t)(void);  // No parameters
```

**Constraints:**
- Max 4 sunrise callbacks (`SOLAR_CALC_MAX_SUNRISE_CALLBACKS`)
- Max 8 midnight callbacks (`SOLAR_CALC_MAX_MIDNIGHT_CALLBACKS`)
- Registered callbacks persist for lifetime of system (no unregister API)

## Design Patterns

**Event-Driven Time Notifications:** Components register callbacks instead of polling `time(NULL)` and comparing against sunrise/sunset thresholds. This eliminates redundant calculations and enables precise one-time-per-day event triggering without state machines in consumers.

**Single Source of Truth:** The midnight timer callback is the only place that resets `sunrise_triggered_today`, ensuring exactly one sunrise broadcast per calendar day regardless of how many times `solar_calc_update()` is called.

**Hemisphere-Agnostic Trigonometry:** Uses standard geographic coordinate conventions (positive North/East, negative South/West) in a single algorithm that works globally. The latitude sign naturally propagates through trigonometric functions to produce correct results for both hemispheres.

**Failsafe Timer Scheduling:** One-shot timer rescheduled after each midnight event (not periodic timer), preventing drift from missed events. If timer fires slightly late (system overload), next midnight is calculated from current time, not from expected time.

**Zero-Copy Time Queries:** Returns const pointer to cached `solar_times_t` structure instead of copying. Consumers read directly from static memory (safe because updates are atomic writes).

## Integration

**FLUCTUS (Power & Solar Tracking):**
- Registers sunrise callback to wake SLEEPING solar tracking
- Registers midnight callback for daily energy accumulator reset
- Uses `solar_calc_is_daytime_buffered()` for tracking state machine transitions

**IMPLUVIUM (Irrigation):**
- Uses `solar_calc_is_daytime_buffered()` for interval switching (shorter day intervals, longer night intervals)
- Future: Midnight callback for daily learning history advancement

**TEMPESTA (Weather Station):**
- Uses `solar_calc_is_daytime_buffered()` for power save mode (longer night intervals)
- Midnight callback resets daily rain accumulator

**TELEMETRY:**
- Future: Midnight callback for daily MQTT summary reports

## Performance

**Memory Footprint:**
- Static data: 20 bytes (`solar_times_t`)
- Callback arrays: 32 bytes (4 sunrise + 8 midnight function pointers)
- esp_timer: ~100 bytes (ESP-IDF managed)
- Total SRAM: ~152 bytes

**Timing:**
- NOAA calculation: 200-500 μs (floating-point math, ~100 operations)
- Midnight timer callback: 1-2 ms (includes broadcasting + recalculation)
- Sunrise callback broadcast: <100 μs (4 function calls)
- `solar_calc_is_daytime_buffered()`: 2-5 μs (cached comparison)

**Accuracy:**
- Sunrise/sunset: ±5-10 minutes (NOAA simplified algorithm)
- Midnight timer: ±10 ms (esp_timer precision)

**Power Consumption:**
- Negligible (single timer, daily calculation)
- No periodic tasks or polling

## Configuration

Compile-time constants in `solar_calc.h`:

```c
// Geographic location (Gdansk, Poland)
#define SOLAR_CALC_LATITUDE     54.3521   // Degrees North
#define SOLAR_CALC_LONGITUDE    18.6464   // Degrees East
#define SOLAR_CALC_TIMEZONE_OFFSET  1     // UTC+1 (CET)

// Operational buffers
#define SOLAR_CALC_SUNRISE_BUFFER_MINUTES  30  // Pre-sunrise window
#define SOLAR_CALC_SUNSET_BUFFER_MINUTES   30  // Post-sunset window

// Callback limits (internal)
#define SOLAR_CALC_MAX_SUNRISE_CALLBACKS 4
#define SOLAR_CALC_MAX_MIDNIGHT_CALLBACKS 8
```

**Adjusting for DST:**
- Manual adjustment required (change `SOLAR_CALC_TIMEZONE_OFFSET` to UTC+2 in summer)
- Future: Automatic DST handling via TZ environment variable

**Changing Location:**
- Modify `SOLAR_CALC_LATITUDE` and `SOLAR_CALC_LONGITUDE`
- Algorithm works for any location (-90° to +90° latitude, -180° to +180° longitude)

## Known Limitations

- Manual DST adjustment required (no automatic timezone database)
- Callback arrays fixed size (no dynamic registration)
- No unregister API (callbacks persist for system lifetime)
- Midnight timer uses local time (assumes SNTP sets system timezone correctly)
- No validation for polar regions (algorithm handles it, but buffered sunrise may behave unexpectedly during polar night/day)

---

**Related Files:**
- Integration: `main/main.c` (initialization order)
- Primary consumer: `components/core/fluctus/fluctus_solar_tracking.c` (sunrise callback)
- Secondary consumers: `components/core/fluctus/fluctus_accumulator.c`, `components/core/tempesta/tempesta_accumulator.c` (midnight callbacks)
