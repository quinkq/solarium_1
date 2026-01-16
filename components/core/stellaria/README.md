# STELLARIA - Adaptive LED Lighting Control

**Author:** Piotr P. <quinkq@gmail.com>
**Part of Solarium** - ESP32-S3 smart irrigation and solar power management system
**Lines of Code:** ~760 (single-file)

Intelligent ambient lighting system for solar-powered garden automation. Controls LED drivers with smooth ramping, automatic light sensing, irrigation coordination, and battery-aware power management.

---

## Overview

STELLARIA manages ambient LED lighting through a Meanwell LDD-600L constant current driver, offering:

- **Manual Control**: Direct intensity setting (0-100% with smooth transitions)
- **Auto Mode**: Automatic ON/OFF based on FLUCTUS photoresistor readings
- **Power Save**: Battery protection with 10% intensity limit
- **Irrigation Dimming**: Automatic dimming during watering to reduce 12V bus load
- **Load Shedding**: Participates in system-wide shutdown during critical battery conditions

### Key Characteristics

- **10%/second ramping**: Smooth intensity transitions (no harsh changes)
- **Event-driven**: Suspended task architecture (zero CPU overhead when idle)
- **Power-aware**: Reference-counted 12V bus management
- **Thread-safe**: Mutex-protected with 100ms timeout
- **Zero-copy telemetry**: Direct cache writes

---

## Architecture

### Component Structure

```
stellaria/
├── stellaria.h         # Public API and configuration
├── stellaria.c         # Implementation (~800 lines)
└── CMakeLists.txt      # Build configuration
```

Single-file design - all logic in one compilation unit for simplicity.

### State Machine

```
DISABLED ──────> ENABLED ──────> SHUTDOWN
   ▲                │                │
   └────────────────┴────────────────┘
```

- **DISABLED**: System off, no bus power, PWM at 0
- **ENABLED**: Active (manual or auto mode), bus power held
- **SHUTDOWN**: Load shedding override, saves state for restoration

All transitions use smooth 10%/s ramping.

### Threading Model

**Ramping Task** (`stellaria_ramp_task`, priority 5):
- Suspended when not ramping (zero overhead)
- Resumed on intensity changes
- Updates PWM every 100ms during transitions
- Self-suspends when target reached

### Data Flow

```
User/Auto Mode ──> user_raw_intensity
                         │
                         ▼
              Calculate effective target:
                - State clamping (DISABLED=0)
                - Power save limit (10%)
                - Irrigation override (MIN)
                         │
                         ▼
                  Start ramping task
                         │
                         ▼
                   PWM hardware
```

**Light sensor feedback** (auto mode):
```
FLUCTUS photoresistors ──> Hysteresis (0.3V/0.4V)
                                    │
                    ┌───────────────┴────────────┐
                    ▼                            ▼
              Dark (>0.4V)                 Bright (<0.3V)
                    │                            │
                Turn ON                      Turn OFF
```

### Task Priority & Synchronization

**Single Task Design:**
- `stellaria_ramp_task` (Low-3): Suspended when idle, resumed on intensity changes
- No inter-task communication (self-contained operation)

**Priority Rationale:**
- Low-3 priority: LED ramping is not time-critical (100ms updates acceptable)
- Lower than Med-5 sensors/monitoring (irrigation, power, weather have precedence)
- Smooth ramping tolerates occasional scheduling delays without visual artifacts

**Synchronization:**
- Single mutex (`xStellariaMutex`) protects all state
- 100ms timeout for quick operations
- Suspended task consumes zero CPU when idle (no polling)

### Initialization & Startup

**Default States:**
- State machine: **DISABLED** (PWM at 0, no bus power)
- User intensity: 512 (50% preference)
- Auto mode: Disabled
- Power save mode: Disabled

**Initialization Order Dependency:**
- FLUCTUS must initialize first (12V bus infrastructure)
- TELEMETRY must be ready for cache writes
- No dependency on other components (IMPLUVIUM/TEMPESTA call STELLARIA, not vice versa)

**First Boot Behavior:**
- LED remains off until user enables via HMI or auto mode triggers
- No automatic turn-on (deployment safety)

---

## Hardware Configuration

### PWM Output

| GPIO | Function | Configuration | Notes |
|------|----------|---------------|-------|
| GPIO45 | LED PWM | LEDC Timer 0, Ch 4, 1kHz, 10-bit | Meanwell LDD-600L input |

**External Circuit:**
- GPIO45 → Meanwell LDD-600L PWM input
- 100kΩ pull-down resistor on PWM input

**Intensity Mapping:**
- 0: Off (0%)
- 51: Minimum (5%, driver turn-on threshold)
- 1023: Maximum (100%)
- Clamping: 1-50 → 51, >1023 → 1023

**Power Save Limit:**
- `STELLARIA_POWER_SAVE_LIMIT = 102` (10%)
- One-way clamp during battery protection

---

## Key Features

### 1. Smooth Ramping
- **Rate**: 10% per second (configurable)
- **Update interval**: 100ms PWM updates
- **Benefits**: Professional appearance, reduces inrush current, visually comfortable

### 2. Automatic Light Sensing
Uses FLUCTUS photoresistors with hysteresis to prevent toggling:
- Turn ON: >0.4V (dark)
- Turn OFF: <0.3V (bright)
- Update frequency: ~5s (FLUCTUS tracking interval)
- Remembers last manual intensity for auto ON transitions

### 3. Irrigation Dimming
IMPLUVIUM can request temporary dimming during watering:
```c
stellaria_request_irrigation_dim(true);   // Dim to 5% (minimum)
// ... pump running ...
stellaria_request_irrigation_dim(false);  // Restore preference
```
Reduces 12V bus load when pump and lights run simultaneously.

### 4. Power Management
**Bus power**: Automatic reference counting via FLUCTUS
```c
stellaria_enable_output()  → fluctus_request_bus_power(POWER_BUS_12V)
stellaria_disable_output() → fluctus_release_bus_power(POWER_BUS_12V)
```

**Load shedding** (called by FLUCTUS):
- 40% SOC: `stellaria_set_power_save_mode(true)` - Clamp to 10%
- 0% SOC: `stellaria_set_shutdown(true)` - Full shutdown, save state

**Power save mode**: Permanently clamps user preference to 10% (one-way, battery protection).

### 5. Zero-Copy Telemetry
Direct cache write eliminates intermediate buffers:
```c
stellaria_snapshot_t *cache = telemetry_lock_cache(TELEMETRY_SRC_STELLARIA);
stellaria_write_to_telemetry_cache(cache);  // Single memcpy
telemetry_unlock_cache(TELEMETRY_SRC_STELLARIA);
```

---

## Public API

### Initialization
```c
esp_err_t stellaria_init(void);
```
Creates mutex, initializes PWM (GPIO45, 1kHz, 10-bit), creates ramping task.

### System Control
```c
esp_err_t stellaria_enable(void);
esp_err_t stellaria_disable(void);
```
Enable requests bus power and ramps to target. Disable ramps to 0 and releases power.

```c
esp_err_t stellaria_set_intensity(uint16_t intensity);
```
Sets intensity (0-1023). Stores raw preference, calculates effective target (applies state/power-save/irrigation limits), starts ramp.

### Mode Configuration
```c
esp_err_t stellaria_set_auto_mode(bool enable);
```
Enable/disable automatic light sensing using FLUCTUS photoresistors.

```c
esp_err_t stellaria_update_light_intensity(float averaged_light_voltage);
```
Called by FLUCTUS with sensor data (~5s intervals). Applies hysteresis logic.

```c
esp_err_t stellaria_set_power_save_mode(bool enable);
```
Enable: Permanently clamps user preference to 10% if above limit.
Disable: Clears limit but does NOT restore previous value (one-way protection).

### Power Management
```c
esp_err_t stellaria_set_shutdown(bool shutdown);
```
Shutdown: Saves state and ramps down. Restore: Recovers saved state and ramps up.

```c
esp_err_t stellaria_request_irrigation_dim(bool dim);
```
Called by IMPLUVIUM. Dims to 5% during watering, restores preference afterward.

### State Queries
```c
stellaria_state_t stellaria_get_state(void);
```
Lightweight state getter (~5μs) for HMI control logic. No telemetry fetch.

```c
esp_err_t stellaria_write_to_telemetry_cache(stellaria_snapshot_t *cache);
```
Zero-copy write to TELEMETRY cache. Returns full snapshot with all state fields.

---

## Integration

### Component Dependencies

**FLUCTUS (Power & Light Sensing):**
- Provides 12V bus power via reference counting
- Supplies averaged photoresistor readings for auto mode (~5s intervals)
- Coordinates load shedding (power save @ 40% SOC, shutdown @ 0% SOC)
- **Multi-use sensors**: FLUCTUS photoresistors serve solar tracking, nighttime detection, and STELLARIA auto mode

**IMPLUVIUM (Watering Coordination):**
- Requests temporary dimming to 5% during watering (reduces 12V bus load)
- Restores user preference after pump shutdown

**TELEMETRY (Data Hub):**
- Event-driven snapshot injection on ramp start
- HMI retrieves cached data at 1Hz (static) or 4Hz (debug)

**External Libraries:**
- `driver/ledc`: PWM generation
- `freertos/FreeRTOS`: Task suspend/resume, mutex

---

## Data Structures

### `stellaria_snapshot_t`

Complete state snapshot for telemetry cache (~64 bytes):

```c
typedef struct {
    stellaria_state_t state;               // DISABLED/ENABLED/SHUTDOWN
    uint16_t current_intensity;            // Current PWM duty (0-1023)
    uint16_t user_raw_intensity;           // User preference (0-1023)
    uint16_t effective_target_intensity;   // Calculated target after limits

    bool auto_mode_active;                 // Automatic light sensing enabled
    bool power_save_mode;                  // Battery protection active (10% clamp)
    bool irrigation_dimming_active;        // Temporary dim during watering

    bool is_ramping;                       // Ramp task active
    float ramp_progress_percent;           // 0.0-100.0%

    time_t snapshot_timestamp;             // Unix epoch
} stellaria_snapshot_t;
```

**Update Frequency:**
- Event-driven injection on ramp start
- HMI pulls at 1Hz (static) or 4Hz (realtime debug)

---

## Configuration

### Compile-Time Parameters

**Ramping Behavior:**
- `STELLARIA_RAMP_RATE_PERCENT_PER_SEC = 10.0f` - 10% intensity change per second
- `STELLARIA_RAMP_UPDATE_INTERVAL_MS = 100` - PWM update interval during ramp
- `STELLARIA_RAMP_STEP_SIZE = 10` - Duty increment per update (10/1023 ≈ 1%)

**Auto Light Sensing:**
- `STELLARIA_LIGHT_TURN_ON_THRESHOLD = 0.4f` - Dark detection voltage (turn ON)
- `STELLARIA_LIGHT_TURN_OFF_THRESHOLD = 0.3f` - Bright detection voltage (turn OFF)
- Hysteresis prevents rapid toggling (0.1V gap)

**Intensity Limits:**
- `STELLARIA_MIN_INTENSITY = 51` - 5% (driver turn-on threshold)
- `STELLARIA_MAX_INTENSITY = 1023` - 100%
- `STELLARIA_POWER_SAVE_LIMIT = 102` - 10% (battery protection)

---

## Design Patterns

**Suspended Task Architecture**: The ramping task suspends itself when target intensity is reached, consuming zero CPU during steady-state operation. This is more efficient than polling-based designs - no wake-ups, no context switches, no power consumption. The task only runs during the 1-10 second transition periods, updating PWM every 100ms.

**Orthogonal Feature Flags**: Auto mode, power save mode, and irrigation dimming are independent boolean flags that combine through priority logic (irrigation override > power save limit > auto mode > manual preference). This allows features to be enabled/disabled without complex state machine transitions - you can have auto mode + power save simultaneously.

**One-Way Power Save Clamping**: When power save mode activates, it permanently clamps the user's raw intensity preference to 10% if it exceeds the limit. This is intentional battery protection - the system doesn't automatically restore high intensity when SOC recovers, preventing surprise power drain. User must manually increase intensity post-recovery.

**Hysteresis-Based Auto Mode**: Light sensing uses a 0.1V dead band (0.3V/0.4V thresholds) to prevent toggling when ambient light hovers near the threshold. Combined with FLUCTUS's 5-second update interval, this creates stable behavior - lights don't flicker during dawn/dusk transitions or when shadows pass over sensors.

**Reference-Counted Power Management**: The component doesn't directly control the 12V bus - it requests/releases power through FLUCTUS's reference counting system. This allows multiple 12V consumers (pump, valves, lights) to coexist safely. The bus stays powered until all consumers release their references.

---

**Related Files:**
- Integration: `main/main.c`
- HMI rendering: `components/core/hmi/hmi_render_system.c`
- Telemetry encoding: `components/core/telemetry/telemetry_msgpack.c`
