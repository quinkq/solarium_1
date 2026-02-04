# IMPLUVIUM - Intelligent Irrigation System

**Author:** Piotr P. <quinkq@gmail.com>
**Part of Solarium** - ESP32-S3 smart irrigation and solar power management system
**Lines of Code:** ~3890 (7 modules)

IMPLUVIUM is a learning-based multi-zone irrigation controller for ESP32-S3. It autonomously manages 5 irrigation zones using adaptive algorithms, real-time safety monitoring, and persistent learning to optimize water usage based on historical performance and environmental conditions.

---

## Overview

**Core Capabilities:**
- **Learning Algorithm:** 15-event history per zone with confidence-based predictions
- **Adaptive Intervals:** Dynamic moisture checking (5-120min) based on temperature/power/day-night
- **Multi-Zone Queue:** Priority-sorted sequential watering
- **Real-Time Safety:** Pre-checks + continuous 500ms monitoring + emergency diagnostics
- **Power Integration:** FLUCTUS load shedding with 3-bus management (3.3V/5V/12V)
- **Persistence:** LittleFS config + RTC accumulator for usage tracking

**Hardware:**
- 5× Capacitive moisture sensors (ADS1115-0)
- ABP pressure sensor (SPI2, 0-2.5 bar)
- Flow meter (GPIO48, PCNT, 400 pulses/L, ≥15 L/h)
- Solenoid valves (GPIO38-42)
- Variable pump (GPIO46, PWM 1kHz 10-bit, 42-100%)

---

## Architecture

### Modular Design (7 modules + private header)

```
impluvium.c                (919 lines)  - Core orchestration, power, telemetry
impluvium_sensors.c        (179 lines)  - Moisture/pressure/level with retry
impluvium_actuators.c      (244 lines)  - GPIO/pump/valve, PWM ramp, adaptive control
impluvium_learning.c       (482 lines)  - Interval calc, temp correction, predictions
impluvium_state_machine.c  (508 lines)  - 6-state workflow handlers
impluvium_safety.c         (779 lines)  - Pre-checks, monitoring task, diagnostics
impluvium_storage.c        (533 lines)  - LittleFS persistence, RTC accumulator
impluvium_private.h                     - Shared declarations (no circular deps)
```

### State Machine (6 states)

**Normal Operation Flow:**
```
<STANDBY> ──timer──> <MEASURING>
    ^                   │
    │              ┌────┴──────┐
    │              │           │
    │         no_zones    queue_built
    │         need_water       │
    │              │           v
    │              │      <WATERING> ──zone_done──> <STOPPING>
    │              │           ^                        │
    │              │           │                   ┌────┴─────┐
    │              │           │                   │          │
    │              │           │              more_zones  all_zones
    │              │           │              in_queue      done
    │              │           │                   │          │
    │              │           └───────────────────┘          │
    │              │        (loop repeats 2-5×)               v
    │              │                                    <MAINTENANCE>
    │              │                                          │
    │              └──────────────────────────────────────────┤
    └─────────────────────────────────────────────────────────┘
                                               (publish telemetry)

                  <DISABLED> (load shedding - separate branch)
```

**Key Pattern**: Timer → scan all zones → water queue sequentially (WATERING↔STOPPING loop) → publish telemetry → return to idle. Sensors powered once per cycle, not per zone.

**State Details:**

| State | Function | Power Buses | Duration | Exit Transitions |
|-------|----------|-------------|----------|------------------|
| `STANDBY` | Wait for timer | Off | 5-120min | **→ MEASURING** (timer fired)<br>**→ DISABLED** (shutdown) |
| `MEASURING` | Scan zones, build queue | 3.3V/5V on | ~5s | **→ WATERING** (queue built, pre-checks pass)<br>**→ STANDBY** (no zones need water) |
| `WATERING` | Water one zone | 3.3V/5V/12V on | 5-60s/zone | **→ STOPPING** (zone done: timeout or target reached) |
| `STOPPING` | Ramp pump, close valve, update learning | 12V ramp-down | ~4s | **→ WATERING** (more zones in queue - loop)<br>**→ MAINTENANCE** (all zones done) |
| `MAINTENANCE` | Publish telemetry OR run diagnostics | Off | <1s (normal)<br>Variable (emergency) | **→ STANDBY** (normal path)<br>**→ MAINTENANCE** (emergency loop) |
| `DISABLED` | Load shedding idle | Off | Until re-enabled | **→ STANDBY** (system re-enabled) |

**MAINTENANCE State Dual Purpose:**

**Normal Path** (no emergency):
```c
// impluvium_state_maintenance() - Line 528
impluvium_change_state(IMPLUVIUM_STANDBY);
telemetry_fetch_snapshot(TELEMETRY_SRC_IMPLUVIUM);  // Publish workflow completion
```
- **Single responsibility**: Publish telemetry snapshot to MQTT
- **Duration**: <1 second
- **Why not in STOPPING?** Architectural separation: STOPPING handles hardware, MAINTENANCE handles software/data

**Emergency Path** (`emergency_stop` triggered):
```c
// 5-state diagnostic state machine within MAINTENANCE:
EMERGENCY_TRIGGERED    → Check moisture levels, identify testable zones
EMERGENCY_DIAGNOSING   → Test each zone sequentially (5s per zone)
EMERGENCY_USER_REQUIRED → Manual intervention needed (stays here until user reset)
EMERGENCY_RESOLVED     → Auto-recovery successful
```
- **Zone isolation testing**: Measures flow/pressure per zone to identify faulty hardware
- **Failed zones bitmask**: Tracks which zones failed diagnostic tests
- **Power conservation**: All buses off during USER_REQUIRED state
- **Trigger conditions**: 3 consecutive emergency stops, persistent hardware faults

**Task Notifications:**
```c
// irrigation_task receives:
IRRIGATION_TASK_NOTIFY_REQUEST_MOISTURE_CHECK  // Timer expired
IRRIGATION_TASK_NOTIFY_WATERING_CUTOFF         // Monitoring task emergency
IRRIGATION_TASK_NOTIFY_EMERGENCY_SHUTDOWN      // Hardware fault
IRRIGATION_TASK_NOTIFY_MIDNIGHT_RESET          // Daily rollover
IRRIGATION_TASK_NOTIFY_MANUAL_WATER            // HMI manual trigger

// irrigation_monitoring_task receives:
MONITORING_TASK_NOTIFY_START_MONITORING  // Begin 500ms safety checks during watering
MONITORING_TASK_NOTIFY_STOP_MONITORING   // Stop monitoring
```

### FreeRTOS Tasks

- `irrigation_task` (Med-5): State machine, moisture checks, watering orchestration
- `irrigation_monitoring_task` (Med-5): Safety monitoring (500ms), adaptive pump control, emergency cutoffs

**Synchronization:**
- `xIrrigationMutex`: Protects `irrigation_zones[]` + `irrigation_system` state
- `xMoistureCheckTimer`: Periodic timer (interval changes dynamically)
- Task notifications: Timer events, emergency cutoffs, manual watering triggers

**Priority Rationale:**
- Both tasks Med-5: Watering operations are time-sensitive (moisture readings, pump control, flow monitoring)
- Equal priority prevents one task from blocking the other during critical operations
- Main task handles state machine, monitoring task handles real-time safety (both equally important)

### Initialization & Startup

**Default States:**
- State machine: **STANDBY** (timer started, awaiting first cycle)
- Power buses: All unpowered (ref_count = 0)
- Emergency state: EMERGENCY_NONE
- Learning data: Loaded from LittleFS (or defaults if first boot)

**Initialization Order Dependency:**
- FLUCTUS must initialize first (power bus infrastructure)
- TEMPESTA must initialize before IMPLUVIUM (temperature data for interval calculation)
- TELEMETRY must be ready for cache writes
- Load shedding callbacks require IMPLUVIUM to be fully initialized before FLUCTUS monitoring detects battery states

**First Boot Behavior:**
- Loads zone config from LittleFS (or creates default: 60% target, 5% deadband, all enabled)
- Loads learning data from LittleFS (or starts with default PPMP ratios, zero confidence)
- Starts moisture check timer with optimal interval (temperature-dependent)
- Does NOT water immediately - waits for first timer cycle to measure zones

---

## Hardware Configuration

### GPIO Assignments

| GPIO | Function | Type | Notes |
|------|----------|------|-------|
| GPIO48 | Flow sensor | Input (PCNT) | 400 pulses/L, ≥15 L/h min |
| GPIO38 | Zone 0 valve | Push-pull 3.3V | Solenoid control |
| GPIO39 | Zone 1 valve | Push-pull 3.3V | Solenoid control |
| GPIO40 | Zone 2 valve | Push-pull 3.3V | Solenoid control |
| GPIO41 | Zone 3 valve | Push-pull 3.3V | Solenoid control |
| GPIO42 | Zone 4 valve | Push-pull 3.3V | Solenoid control |
| GPIO46 | Pump PWM | LEDC CH4, Timer 0 | 1kHz, 10-bit, 42-100% duty |

### I2C Sensors

**ADS1115 Device 0** @ 0x48 (Moisture Sensors):
- Channel 0 (AIN0_GND): Zone 0
- Channel 1 (AIN1_GND): Zone 1
- Channel 2 (AIN2_GND): Zone 2
- Channel 3 (AIN3_GND): Zone 3
- Zone 4: Check actual mapping in zone config init (channel reuse)

### SPI Sensors

**ABP Pressure Sensor** @ GPIO12 (CS):
- Model: ABPDJJT001PDSA3
- Physical range: -68.94 to +68.94 mbar
- Operating range: 15-60 mbar (water level measurement)
- Purpose: Delta-P for tank level sensing

---

## Key Features

### 1. Learning Algorithm (impluvium_learning.c)

**Learning Cycle:**
1. **Predict:** 2-tier confidence blending (linear 0→70%: `blend = learned × (conf/0.70) + default × (1 - conf/0.70)`, above 70%: 100% learned), temp correction ±1%/°C, dynamic moisture cutoff (accounts for soil redistribution)
2. **Execute:** Water with real-time monitoring, ratio-based pump duty adjustment (±15% max per cycle)
3. **Measure:** Store pre-computed PPMP ratio, record immediate moisture
4. **Update:** Update confidence (20% EMA + boost floor), adjust pump duty (ratio-based)
5. **Verify:** 5-min delayed reading → update soil redistribution factor (15% EMA)

**Learning Data (per zone):**
```c
typedef struct {
    float ppmp_ratio_history[15];           // Pre-computed PPMP ratios
    bool anomaly_flags[15];                 // Invalid cycles (rain, errors)
    uint8_t history_entry_count;            // Valid entries (0-15)
    uint8_t history_index;                  // Circular buffer position
    float last_temperature_correction;      // Most recent temp factor
    float calculated_ppmp_ratio;            // Weighted average PPMP
    uint32_t calculated_pump_duty_cycle;    // Optimal PWM (0-1023)
    float soil_redistribution_factor;       // Moisture redistribution (1.0-3.0)
    float measured_moisture_gain_rate;      // Latest gain rate (%/sec)
    float confidence_level;                 // EMA-based confidence (0.0-1.0)
    uint32_t successful_predictions;        // Success count (legacy)
    uint32_t total_predictions;             // Total count (legacy)
} zone_learning_t;
```

**Temperature Correction:** ±1%/°C from 20°C baseline (0.8-1.2× factor)

**Anomaly Detection:** Moisture spike >5%/sec (rain), flow variance ±20 L/h, temp <5°C or >45°C

**Learning Algorithm Tuning:**

**When to Reset Learning:**
- Zone target moisture changed (new baseline for predictions)
- Hardware changes (new sensor, pump, valve)
- Soil type changed (different absorption characteristics)
- 3+ consecutive anomalies (algorithm not converging)

**Confidence Interpretation:**
- `0.0-0.25`: Initial learning, boost floor applied after first good prediction (≤20% error)
- `0.25-0.70`: Transitioning, linear blend (conf=0.35 → 50% learned + 50% default)
- `0.70-1.0`: High confidence, 100% learned predictions (optimized)
- Low confidence (<0.5) after 15+ events: Check for anomalies or hardware issues

**Soil Redistribution Factor (RF) Interpretation:**
- `1.0-1.2`: Sandy soil (fast drainage, minimal redistribution)
- `1.3-1.7`: Loam (moderate redistribution, typical)
- `1.8-3.0`: Clay (slow drainage, significant upward redistribution)
- Default: 1.5 (conservative mid-range estimate)

**History Size Rationale:**
- 15 events = 2-3 weeks of daily watering
- Balances recent performance (last week) vs long-term patterns
- Anomaly flags prevent bad data from corrupting predictions
- Storage: 5 most recent saved to LittleFS (survives power loss)

### 2. Adaptive Intervals (impluvium_learning.c)

**Runtime-Configurable:**
```c
esp_err_t impluvium_set_check_intervals(
    uint32_t optimal_min,      // 5-60min (temp ≥20°C)
    uint32_t cool_min,         // 10-90min (10-20°C)
    uint32_t power_save_min,   // 30-120min (FLUCTUS @ SOC 40%)
    uint32_t night_hours       // 1-6 hours (nighttime minimum)
);
```

**Logic:**
1. **Power Save:** Unconditional override (FLUCTUS-triggered)
2. **Temperature:** Optimal (≥20°C) vs Cool (10-20°C)
3. **Nighttime:** Enforces minimum (NOAA sunrise/sunset)
4. **Safety:** Returns `UINT32_MAX` (skip) if temp <10°C

**Configuration Constraints (Rationale):**

| Parameter | Range | Reason |
|-----------|-------|--------|
| Optimal interval | 5-60 min | Balance: 5min = responsive to deficit, 60min = power saving. Below 5min: sensor noise, above 60min: deficit accumulation |
| Cool interval | 10-90 min | Slower evaporation at 10-20°C allows longer intervals |
| Power save interval | 30-120 min | Battery conservation, acceptable for slow-growing deficit |
| Night minimum | 1-6 hours | Minimal evaporation at night, avoid unnecessary sensor power cycles |
| Default PPMP | 15.0 pulses/% | Conservative starting efficiency estimate for unknown soil |
| PPMP bounds | 2.0-100.0 pulses/% | Hardware-safe clamping range (prevents sensor glitch damage) |
| Default target pulses | 100 (~250mL) | Conservative initial watering volume |
| Target pulses bounds | 20-600 (50-1500mL) | Safety min/max to prevent under/over-watering |
| Pump min duty | 42% | Centrifugal pump stall protection (manufacturer spec) |
| Pump max duty | 100% | Full power available, adaptive control adjusts down if needed |
| Pump duty adjustment | ±15% per cycle | Prevents oscillation from noisy gain rate measurements |
| Soil RF range | 1.0-3.0 | Sandy (instant) to clay (slow redistribution) |
| Max watering time | 60 sec | Safety timeout: prevents runoff, limits damage if valve stuck |
| Min flow rate | 15 L/h | Blockage detection threshold (below = clogged line/failed pump) |
| Max pressure | 2.5 bar | Hydraulic safety: prevents hose burst, solenoid damage |
| Min water level | 3% (15 mbar) | Pump cavitation protection, ensures minimum system pressure |

### 3. Multi-Zone Queue (impluvium_state_machine.c)

**Queue Workflow:**
```c
typedef struct {
    uint8_t zone_id;
    float moisture_deficit_percent;  // Sorting key
    uint16_t target_pulses;          // Predicted water
    bool watering_completed;
} watering_queue_item_t;
```

1. `MEASURING`: Scan all zones → build queue sorted by deficit
2. `WATERING`: Process sequentially (one zone at a time)
3. Auto-dim STELLARIA 50% during watering
4. Realtime telemetry 500ms via `impluvium_snapshot_rt_t`

### 4. Safety System (impluvium_safety.c)

**Three-Tier Protection:**

**Pre-Checks (before watering):**
- Temperature: 10-50°C (watering ≥10°C, global 0-50°C)
- Water level: ≥3% (≥15 mbar)
- Pressure: ≤2.5 bar

**Continuous Monitoring (500ms during watering):**
- Flow rate ≥15 L/h (blockage detection)
- Pressure ≤2.5 bar (hydraulic safety)
- Max duration 60 seconds (timeout)
- Adaptive pump: Maintains 0.5%/sec moisture gain

**Emergency Diagnostics:**
```c
typedef enum {
    EMERGENCY_NONE, EMERGENCY_TRIGGERED, EMERGENCY_DIAGNOSING,
    EMERGENCY_USER_REQUIRED, EMERGENCY_RESOLVED
} emergency_state_t;
```
- Zone isolation testing (5s cycles)
- Flow/pressure measurement per zone
- Failed zones bitmask
- Requires manual reset after 3 consecutive failures

**Emergency Recovery:**
- System remains in safe state indefinitely when EMERGENCY_USER_REQUIRED
- All power buses released, state machine blocked in MAINTENANCE
- Components can still query `impluvium_get_state()` (returns MAINTENANCE + emergency flags)
- Manual intervention: `impluvium_clear_emergency_stop()` after fixing hardware issues
- Failed zones remain disabled until `impluvium_clear_diagnostics()` called

### 5. Persistent Storage (impluvium_storage.c)

**LittleFS Files:**
```
/data/irrigation/config.dat   (50B, CRC16)
├─ Zone settings: target_moisture, deadband, enabled
└─ Trigger: MQTT/HMI config updates

/data/irrigation/learning.dat (~350B, CRC16)
├─ Learned params: PPMP ratio, pump duty, soil redistribution factor, confidence
├─ Recent history: 5 most recent valid PPMP ratios per zone (pre-computed)
└─ Trigger: Daily at midnight
```

**RTC RAM Accumulator:**
```c
typedef struct {
    time_t current_hour_start;
    float zone_water_used_hour_ml[5];
    uint8_t zone_events_hour[5];
    time_t current_day_start;
    float zone_water_used_day_ml[5];
    uint8_t zone_events_day[5];
} irrigation_accumulator_rtc_t;  // 180 bytes
```
- Survives resets/deep sleep but **not power loss** (critical: usage data lost if system powered down completely)
- Hourly reset automatic, daily midnight save to LittleFS + reset

### 6. Power Management (impluvium.c)

**Three-Bus Orchestration:**
```c
typedef enum {
    POWER_ONLY_SENSORS = 0,      // 3.3V (moisture, ADS1115)
    POWER_ONLY_PUMP = 1,         // 12V (pump)
    POWER_ALL_DEVICES = 2,       // Both
    POWER_PUMP_AND_SOLENOID = 3  // 12V (pump + valves)
} power_level_t;
```

**Load Shedding:**
- SOC 40%: `impluvium_set_power_save_mode(true)` → 60min interval
- SOC 0%: `impluvium_set_shutdown(true)` → DISABLED state

**Reference Counting:** FLUCTUS tracks consumers, prevents premature shutdown

### 7. Adaptive Pump Control (impluvium_actuators.c)

**PWM Adjustment:**
```c
void impluvium_pump_adaptive_control(float current_gain_rate, float target_gain_rate);
// Adjusts ±20 duty steps (2%) to maintain 0.5%/sec target
// Constraints: 42% min (stall protection), 100% max
```

**Ramp Control:**
- `RAMP_UP`: 5s linear (42% → learned duty) - reduces inrush
- `RAMP_DOWN`: 3s linear (current → 0%) - extends lifespan

---

## Public API

### Initialization
```c
esp_err_t impluvium_init(void);
// Loads LittleFS config, creates tasks, starts timer
```

### System Control
```c
esp_err_t impluvium_set_system_enabled(bool enable);
impluvium_state_t impluvium_get_state(void);  // Lightweight (~5μs)
esp_err_t impluvium_force_moisture_check(void);
```

### Power Management
```c
esp_err_t impluvium_set_power_save_mode(bool enable);
esp_err_t impluvium_set_shutdown(bool shutdown);
```

### Interval Configuration
```c
esp_err_t impluvium_set_check_intervals(
    uint32_t optimal_min, uint32_t cool_min,
    uint32_t power_save_min, uint32_t night_hours
);
```

### Zone Configuration
```c
esp_err_t impluvium_update_zone_config(
    uint8_t zone_id, float target_moisture_percent,
    float moisture_deadband_percent, bool enabled
);
```

### Manual Operations
```c
esp_err_t impluvium_force_water_zone(uint8_t zone_id, uint16_t duration_sec);
// 5-300 sec (5s increments), respects pressure only
```

### Learning
```c
esp_err_t impluvium_reset_zone_learning(uint8_t zone_id);
esp_err_t impluvium_reset_all_learning(void);
```

### Emergency Recovery
```c
esp_err_t impluvium_clear_emergency_stop(void);
esp_err_t impluvium_clear_diagnostics(void);
```

### Telemetry
```c
esp_err_t impluvium_write_to_telemetry_cache(impluvium_snapshot_t *cache);
// Full: ~100+ fields (zones, learning, diagnostics)
// Trigger: Workflow completion

esp_err_t impluvium_write_realtime_to_telemetry_cache(impluvium_snapshot_rt_t *cache);
// Realtime: 6 fields (pressure, flow, pump, queue)
// Trigger: 500ms during WATERING
```

---

## Telemetry Structures

### Full Snapshot (impluvium_snapshot_t)
```c
typedef struct {
    impluvium_state_t state;
    uint8_t active_zone;
    bool emergency_stop, power_save_mode, load_shed_shutdown;
    float water_level_percent;

    // System stats (RTC accumulator)
    time_t current_hour_start;
    float total_water_used_hour_ml, total_water_used_day_ml;
    uint8_t watering_events_hour, watering_events_day;

    // Per-zone (5 zones)
    struct {
        bool watering_enabled;
        float current_moisture_percent, target_moisture_percent;
        float volume_used_hour_ml, volume_used_today_ml;
        uint8_t events_hour, events_day;
        float avg_hourly_consumption_ml;
        time_t last_watered_time;

        // Learning
        float calculated_ppmp_ratio, confidence_level;
        uint32_t calculated_pump_duty_cycle;
        float soil_redistribution_factor;
        float measured_moisture_gain_rate, last_temperature_correction;
        uint32_t successful_predictions, total_predictions;
        uint8_t history_entry_count;
    } zones[5];

    emergency_state_t emergency_state;
    uint8_t emergency_failed_zones_mask;
    time_t snapshot_timestamp;
} impluvium_snapshot_t;
```

### Realtime Snapshot (impluvium_snapshot_rt_t)
```c
typedef struct {
    float water_level_percent, outlet_pressure_bar;
    float current_flow_rate_lh, current_moisture_gain_rate;
    uint8_t pump_duty_percent, active_zone;
    bool emergency_stop, pressure_alarm, flow_alarm;

    uint8_t watering_queue_size, queue_index;
    struct {
        uint8_t zone_id;
        float moisture_deficit_percent;
        uint16_t target_pulses;
        bool watering_completed;
    } queue[1];  // Current zone only

    time_t snapshot_timestamp;
} impluvium_snapshot_rt_t;
```

---

## Integration

### Dependencies (CMakeLists.txt)
```c
REQUIRES
    ads1115_helper   // Unified ADS1115 (4 devices, auto-retry)
    interval_config  // Centralized intervals
    abp              // ABP pressure sensor (SPI)
    tempesta         // Temperature for correction
    fluctus          // Power buses (3.3V/5V/12V)
    stellaria        // Auto-dim during watering
    telemetry        // Cache writing
```

### Component Interactions

**FLUCTUS (power):**
```c
fluctus_power_state_t state = fluctus_get_power_state();
FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_3V3, "IMPLUVIUM", { return ESP_FAIL; });
fluctus_release_bus_power(POWER_BUS_3V3, "IMPLUVIUM");
```

**TEMPESTA (environmental):**
```c
float temp = tempesta_get_temperature();
// Used: temp correction, interval calc, safety checks
```

**STELLARIA (lighting):**
```c
stellaria_set_irrigation_dimming(true);  // 50% during watering
```

**TELEMETRY (data):**
```c
telemetry_lock_cache(TELEMETRY_SOURCE_IMPLUVIUM, &cache);
impluvium_write_to_telemetry_cache(cache);
telemetry_unlock_cache(TELEMETRY_SOURCE_IMPLUVIUM);
```

**INTERVAL_CONFIG (centralized):**
```c
interval_config_get(INTERVAL_IMPLUVIUM_OPTIMAL, &impluvium_optimal_interval_ms);
impluvium_set_check_intervals(15, 30, 60, 2);  // Saves to LittleFS
```

---

## Configuration

### Sensor Calibration

All sensors use two-point or factory calibration with typical operating ranges:

**Moisture Sensors (Capacitive, ADS1115):**
- Two-point calibration: Dry voltage (air, ~2.2V) and wet voltage (water, ~0.85V)
- Linear conversion: `moisture_percent = ((DRY_V - voltage) / (DRY_V - WET_V)) * 100.0f`
- Typical range: 0.7-2.5V (sensor-dependent)

**ABP Pressure Sensor (SPI, Delta-P):**
- Physical range: -68.94 to +68.94 mbar (factory calibrated)
- Operating range: 15-60 mbar (0-100% water level)
- Linear conversion: `water_level_percent = ((mbar - MIN) / (MAX - MIN)) * 100.0f`
- Typical: 10-80 mbar for 1m water column

**Flow Meter (Pulse Counter, PCNT):**
- Factory calibration: 400 pulses/L (typical: 350-450 pulses/L)
- Rate calculation: `flow_lh = (pulse_delta / 400.0f) * (3600.0f / time_delta_sec)`
- Minimum detectable flow: 15 L/h

## Design Patterns

**Modular Functional Decomposition**: Seven modules each own specific responsibilities (sensors, actuators, learning, safety, storage, state machine, core orchestration), communicating through a shared private header. This prevents circular dependencies and makes the 3890-line codebase navigable - you can understand the safety system without reading the learning algorithm.

**State Machine with Handler Delegation**: Each of the 6 states (STANDBY/MEASURING/WATERING/STOPPING/MAINTENANCE/DISABLED) has a dedicated handler function. State transitions are explicit and traceable, making the workflow testable in isolation. The WATERING↔STOPPING loop processes zones sequentially without complex nested logic.

**Workflow-Driven Telemetry**: The component publishes snapshots at workflow completion (end of MAINTENANCE state), not on every state transition. This reduces telemetry traffic and ensures HMI/MQTT receive complete watering cycle data (all zones measured, queued, watered, and learned) rather than fragmented state updates.

**Confidence-Weighted Learning**: 2-tier blending system - below 70% confidence uses linear interpolation `target = learned × (conf/0.70) + default × (1 - conf/0.70)`, above 70% uses 100% learned predictions. Early learning (confidence <0.25) accelerates via boost floor after first good prediction. As confidence grows (0.25→0.70), the system gradually transitions from safe defaults to optimized learned behavior. This prevents premature optimization while building reliable historical data with 20% EMA weight for responsive adaptation to hardware changes.

**Dual Persistence Strategy**: Hot operational data (hourly/daily usage) lives in RTC RAM for fast access and survives resets. Cold configuration data (zone targets, learned PPMP ratios) persists to LittleFS with daily writes. This minimizes flash wear while maintaining data continuity through normal reboots - only complete power loss resets usage counters.

**Multi-Tier Safety Architecture**: Pre-checks gate watering initiation (temperature/pressure/level), continuous 500ms monitoring enforces limits during operation (flow/pressure/duration), and emergency diagnostics isolate faulty zones through systematic testing. Each tier operates independently - a pre-check failure doesn't trigger diagnostics, but repeated monitoring failures do.

---

## Performance

**Memory:**
- RAM: ~3KB state + learning
- RTC RAM: 180B accumulator
- LittleFS: 400B config + learning

**Update Frequencies:**
- Moisture checks: 5-120min (dynamic)
- Monitoring: 500ms (watering only)
- Learning save: Daily midnight
- Config save: On user update

---

**Documentation:**
- `LEARNING_ALGORITHM.md` - 4-phase cycle details
- `impluvium.h` - Public API (21 functions)
- `impluvium_private.h` - Internal interface

