# FLUCTUS - Power Management & Solar Tracking

**Author:** Piotr P. <quinkq@gmail.com>
**Part of Solarium** - ESP32-S3 smart irrigation and solar power management system
**Lines of Code:** ~2850 (6 modules + private header)

## Overview

FLUCTUS manages all electrical power distribution, battery monitoring, solar tracking, and load shedding for the Solarium system. The name "fluctus" (Latin: wave/flow) represents its role in managing power flow throughout the hardware.

## Architecture

### Module Structure

```
fluctus/
├── fluctus.c                  (773 lines)  - Init, orchestration, telemetry, public API
├── fluctus_power_bus.c        (446 lines)  - 4-bus GPIO control, reference counting
├── fluctus_power_monitor.c    (679 lines)  - INA219 sensors, overcurrent protection
├── fluctus_solar_tracking.c   (929 lines)  - Servo control, 6-state tracking machine
├── fluctus_thermal.c          (308 lines)  - DS18B20 temp sensor, PWM fan control
├── fluctus_accumulator.c      (228 lines)  - RTC energy tracking, rollover logic
├── fluctus_private.h          (191 lines)  - Shared internal declarations
└── fluctus.h                  (634 lines)  - Public API, config, data structures
```

### Per-Module Ownership

Each module owns its state and mutex to prevent high-frequency tasks (500ms monitoring) from blocking low-frequency operations (15min solar tracking).

| Module | State | Mutex | Task |
|--------|-------|-------|------|
| `fluctus_power_bus.c` | `system_status`, `hardware_bus_state[]` | `xPowerBusMutex` | - |
| `fluctus_power_monitor.c` | `monitoring_data`, `ina219_dev[]` | `xMonitoringMutex` | `fluctus_monitor` (Med-5) |
| `fluctus_solar_tracking.c` | `solar_data` (source of truth) | `xSolarMutex` | `fluctus_solar` (Med-5)<br>`fluctus_servo_control` (Med-5) |
| `fluctus_accumulator.c` | `rtc_fluctus_accumulator` | `xEnergyMutex` | - |
| `fluctus.c` | Orchestration | - | `fluctus_core_orq` (Low-3) |

**Design Benefit:** Concurrent operations across subsystems without mutex contention.

**Task Communication:**
- `fluctus_monitor` (Med-5): Detects battery state changes, sends `xTaskNotify()` to orchestration task
- `fluctus_core_orq` (Low-3): Receives notifications, coordinates component shutdowns (non-blocking)
- `fluctus_solar` (Med-5): State machine - daytime checks, power management, blocks during CORRECTING
- `fluctus_servo_control` (Med-5): 250ms control loop - sensor reads, servo corrections, convergence detection

**Solar Tracking Dual-Task Architecture:**

The solar tracking system uses two cooperating tasks for clean separation of concerns:

1. **State Machine Task** (`fluctus_solar`):
   - Handles 15-min interval timing
   - Performs ONE-TIME daytime/light check before correction
   - Manages 6.6V bus power (on/off)
   - **Blocks during CORRECTING** using `xTaskNotifyWait(30s timeout)` waiting for servo task
   - Receives notifications: ENABLE, DISABLE, SUNRISE, SERVO_CONVERGED
   - Forces servo task suspension on timeout or DISABLE

2. **Servo Control Task** (`fluctus_servo_control`):
   - Created SUSPENDED, resumed by state machine when CORRECTING starts
   - Runs 250ms loop: reads photoresistors, applies small corrections (max 15 duty units)
   - Detects convergence: 3 seconds continuous below 0.01V threshold
   - Notifies state machine on convergence, then suspends self
   - No timeout checking (handled by state machine's wait timeout)

**Benefits:**
- State machine doesn't poll during correction (blocks efficiently)
- Servo task focuses purely on corrections without exit condition complexity
- Single timeout authority (state machine's `xTaskNotifyWait`)
- Power management centralized in one place (state machine)

**Priority Rationale:**
- Med-5 for monitoring/solar: Time-sensitive sensor readings and servo control
- Low-3 for orchestration: Coordination task that can tolerate latency (shutdown decisions aren't microsecond-critical)

### Initialization & Startup

**Default States:**
- Solar tracking: **DISABLED** (deployment safety - prevents unintended servo movement on first boot)
- Power buses: **All unpowered** (ref_count = 0)
- Monitoring: Starts immediately in ACTIVE mode

**Initialization Order Dependency:**
- FLUCTUS must initialize before other components (provides power bus infrastructure)
- Components call `fluctus_request_bus_power()` during their own init routines
- Load shedding callbacks require components to be fully initialized before FLUCTUS monitoring starts detecting battery states

## Hardware Configuration

### GPIO Allocation

| GPIO | Function | Type | Logic |
|------|----------|------|-------|
| GPIO0 | SN74AHCT125 Level Shifter OE | Open-drain | LOW=Enable |
| GPIO3 | 12V Fan Enable | Push-pull | HIGH=ON |
| GPIO4 | 3.3V Bus Enable | Open-drain | LOW=ON (P-MOSFET) |
| GPIO5 | 5V Bus Enable | Push-pull | HIGH=ON |
| GPIO6 | 6.6V Bus Enable | Open-drain | LOW=ON (inverted buck) |
| GPIO7 | 12V Bus Enable | Push-pull | HIGH=ON |
| GPIO8 | Fan PWM | LEDC Timer 3, Ch 3 | 25kHz, 8-bit |
| GPIO17 | DS18B20 OneWire | OneWire protocol | Temperature |
| GPIO21 | Servo Pitch PWM | LEDC Timer 2, Ch 2 | 50Hz, 14-bit |
| GPIO47 | Servo Yaw PWM | LEDC Timer 2, Ch 1 | 50Hz, 14-bit |

### I2C Sensors (Bus A)

**INA219 Power Monitors:**
- **Solar PV** @ 0x40: 32V range, 0.5A gain, 0.1Ω shunt, 12-bit 8-sample avg
- **Battery** @ 0x41: 32V range, 0.125A gain, 0.01Ω shunt, 12-bit 8-sample avg

**ADS1115 Device 2** @ 0x4A (Photoresistor Array):
- Channel 0: Top-Left (TL)
- Channel 1: Top-Right (TR)
- Channel 2: Bottom-Left (BL)
- Channel 3: Bottom-Right (BR)
- **Multi-use sensors**: Solar tracking corrections, nighttime PV shutdown detection (<0.4V), STELLARIA auto mode (averaged light level)

### MCP23008 @ 0x20 (via mcp23008_helper)

- GP5: Hall array N-MOSFET enable (HIGH=ON, controls ADS1115-3)

### Servo Range

- Min duty: 410 (14-bit @ 50Hz)
- Center duty: 1229
- Max duty: 2048
- Max adjustment: ±15 duty units per 250ms iteration (smooth tracking)

## Key Features

### 1. Four-Bus Power Distribution

**Hardware:**
- **3.3V** (GPIO4): Sensor bus, P-MOSFET, open-drain (LOW=ON)
- **5V** (GPIO5): MOSFET drivers, push-pull (HIGH=ON)
- **6.6V** (GPIO6): Servo bus, inverted buck, open-drain (LOW=ON)
- **12V** (GPIO7): Pump/valves, push-pull (HIGH=ON)

**Reference Counting:**
```c
fluctus_request_bus_power(POWER_BUS_3V3, "consumer_name");
// ... use bus ...
fluctus_release_bus_power(POWER_BUS_3V3, "consumer_name");
```
- Bus stays powered while `ref_count > 0`
- 500ms debounce delay prevents rapid cycling
- Tracks up to 16 consumers per bus (string IDs for debugging)

**Level Shifter (SN74AHCT125):**
```c
fluctus_request_level_shifter("consumer");   // 3.3V→5V translation for 12V actuators
fluctus_release_level_shifter("consumer");
```
- Reference counted (max 8 consumers)
- 10ms stabilization delay on first enable
- Required for pump/valves/fan MOSFET control

### 2. Adaptive Power Metering

**Dual INA219 Sensors:**
- Solar PV (0x40): Panel voltage/current/power
- Battery (0x41): Battery output monitoring

**Adaptive Sampling:**
- **ACTIVE** (500ms): Continuous during bus activity/state changes
- **STEADY** (15s probe every 15min): Stable conditions
- **PV_SHUTDOWN**: Solar INA off at night (photoresistor < 0.4V)

**Overcurrent Protection:**
- 3.0A for 5s → Warning, graceful shutdown
- 4.0A immediate → Emergency shutdown, manual reset required

**Failure Recovery:**
- System remains in safe state indefinitely during overcurrent lockout
- All power buses shut down, consumers released
- Components can still query `fluctus_get_power_state()` (returns safety fault flag)
- Manual intervention required: `fluctus_manual_safety_reset()` after fixing hardware issue

### 3. Solar Tracking State Machine

**Six States:**

```
                                        User Enable/Ack
            DISABLED ◄─────────────────────────────────────────┐
              │ ▲                                              │
              │ │                                              │
              │ │  User Disable or                             │
              │ │  Critical Power (0% SOC)                     │
              │ │                                              │
              ▼ │                                              │
             STANDBY ◄──────────────┐                        ERROR
            │      │                │                          ▲
            │      │                │                          │
            │      │ <15min timer + │  << error <0.01V         │
            │      │ daytime check  │  (success)               │ timeout (30s)
            │      │                │                          │ or 5× errors
            │      │                │                          │
            │      ▼                │                          │
            │ CORRECTING ►──────────┴──────────────────────────┘
            │      │    ^^^ Internal 3s loop:
            │      │    - Read photoresistors (4× quadrants)
            │      │    - Check error margin (<0.01V threshold)
            │      │    - Apply servo corrections (max ±250 duty/cycle)
            │      │    
            │      │
            │  <<  │ << Sunset detected
            │      │   (astronomical nighttime + light <0.4V)
            │      │
            │      ▼
            └───► PARKING << Parks servos, releases 6.6V bus
                    │  ^^
                    │  Decision based on parking_reason enum:
                    │
                    ├─── SUNSET ──────────────────────► SLEEPING (night park: 10% yaw, 60% pitch)
                    │                                      │
                    │                                      │  Sunrise callback
                    │                                      │  (-30min buffer)
                    │                                      │
                    └─── USER_DISABLE ───────────┐         │
                         or CRITICAL_POWER       │         │
                                                 │         │
                                                 ▼         ▼
                                             DISABLED  or  STANDBY
                                             (center park: 50%/50%)

Power Gating:
- 6.6V servo bus: ON only during CORRECTING + PARKING (~3-30s per 15min cycle = 0.3-3% duty)
```

**State Descriptions:**

- **DISABLED**: Parked center, waits for user enable notification
- **STANDBY**: Idle, checks daytime every 15min, starts correction if OK
- **CORRECTING**: Blocking state - daytime check, power up 6.6V, resume servo task, wait for convergence/timeout (30s) or disable
- **PARKING**: Transient - parks servos, chooses next state based on reason (SUNSET/USER/CRITICAL)
- **SLEEPING**: Parked east/up, waits for sunrise callback (-30min buffer) to auto-resume
- **ERROR**: Progressive backoff (5s/10s/15s/20s delay), returns to STANDBY unless 5× failures → DISABLED (requires manual `fluctus_enable_solar_tracking()` to restart)

**Photoresistor Feedback:**
- 4× sensors (ADS1115-2): Top-Left/Top-Right/Bottom-Left/Bottom-Right
- Differential errors: `yaw = (TL+BL) - (TR+BR)`, `pitch = (TL+TR) - (BL+BR)`
- Proportional control: Adjustment scales with error (max 15 duty units per 250ms iteration)
- Error margin: 0.010V threshold (prevents jitter)
- Settling time: 3 seconds continuous below threshold required for convergence

**Parking Positions:**
- Night: Yaw 10% (east), Pitch 60% (up) - ready for sunrise
- Error: Yaw 50%, Pitch 50% (center) - safe fallback

**Power Gating:**
- 6.6V bus only ON during CORRECTING state (servo task active)
- Typical duty cycle: 3-10s active per 15min = 0.3-1.1%

### 4. Five-State Load Shedding

**Battery Thresholds (12V AGM, 50% SOC = 0%):**

| State | Voltage | SOC | Actions |
|-------|---------|-----|---------|
| NORMAL | >12.58V | >40% | All features on |
| POWER_SAVING | 12.48V | 40% | STELLARIA → 12.5% max |
| LOW_POWER | 12.33V | 25% | STELLARIA off, TEMPESTA 60min |
| VERY_LOW | 12.23V | 15% | IMPLUVIUM off, PMS5003 off |
| CRITICAL | 12.08V | 0% | TEMPESTA off, solar parked |

**Hysteresis:** +0.1V to re-enable (prevents flapping)
**Debouncing:** 5 consecutive readings required (2.5s @ 500ms)

**Event-Driven:**
1. Monitor task detects state change → sends notification
2. Core orchestration task wakes → calls component APIs:
   ```c
   component_set_power_save_mode(bool enable);
   component_set_shutdown(bool shutdown);
   ```

### 5. Thermal Management

**DS18B20 OneWire:**
- Non-blocking async conversion (750ms, 12-bit)
- Shared state machine with monitoring module

**PWM Fan (Intel 4-pin fan):**
- Turn-on: 32°C → 10% min speed
- Turn-off: 28°C (hysteresis)
- Full speed: 42°C → 100%
- 25kHz PWM on GPIO8 (LEDC Timer 3, 8-bit)

### 6. Energy Accumulation

**RTC RAM Persistence:**
- Survives resets/deep sleep but **not power loss** (critical: data lost if system powered down completely)
- 452 bytes in `power_accumulator_rtc_t`

**Tracking Windows:**
- **15-min averaging**: Voltage/current/power sums for MQTT
- **Hourly rollover**: Energy totals (Wh), peak power (W)
- **Daily rollover**: Energy totals, peak power, active hours

**Callbacks:**
- `fluctus_midnight_callback()` for daily reset
- Hourly checked during 15min telemetry writes

## Public API

### Initialization

```c
esp_err_t fluctus_init(void);
```

### Power Control

```c
// Bus management
esp_err_t fluctus_request_bus_power(power_bus_t bus, const char* consumer_id);
esp_err_t fluctus_release_bus_power(power_bus_t bus, const char* consumer_id);
bool fluctus_is_bus_powered(power_bus_t bus);

// Macro helper
FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_12V, "PUMP", { return ESP_FAIL; });

// Level shifter
esp_err_t fluctus_request_level_shifter(const char* consumer_id);
esp_err_t fluctus_release_level_shifter(const char* consumer_id);
```

### State Queries

```c
fluctus_power_state_t fluctus_get_power_state(void);
solar_tracking_state_t fluctus_get_solar_tracking_state(void);
// Lightweight, no telemetry side effects, 100ms mutex timeout
```

### Solar Tracking

```c
esp_err_t fluctus_enable_solar_tracking(void);
esp_err_t fluctus_disable_solar_tracking(void);
esp_err_t fluctus_enable_solar_debug_mode(void);    // 90s continuous
esp_err_t fluctus_servo_debug_set_position(ledc_channel_t channel, uint32_t duty);
```

### Configuration

```c
esp_err_t fluctus_set_power_intervals(uint32_t day_min, uint32_t night_min);
esp_err_t fluctus_set_solar_interval(uint32_t correction_min);
// Updates LittleFS config, notifies tasks immediately
```

### Safety

```c
esp_err_t fluctus_manual_safety_reset(void);        // Clear overcurrent
void fluctus_hall_array_enable(bool enable);        // TEMPESTA wind sensor
esp_err_t fluctus_set_cooling_fan_speed(uint8_t duty_percent);
```

### Telemetry

```c
esp_err_t fluctus_write_to_telemetry_cache(fluctus_snapshot_t *cache);
esp_err_t fluctus_write_realtime_to_telemetry_cache(fluctus_snapshot_t *cache);
// Direct cache injection (called within TELEMETRY lock/unlock)
```

## Data Structures

### `fluctus_snapshot_t`

Unified 55-field structure with layered writes:
- **26 RT fields** (500ms): Battery/solar instantaneous, servo positions, temp
- **29 Normal fields** (15-60min): Averages, energy stats, system state

**Categories:**
- Power buses (8): Enabled flags + consumer counts
- Battery instant (4): V/A/W/SOC
- Battery avg (4): 15min rolling averages
- Solar instant (4): V/A/W + INA state
- Solar avg (3): 15min rolling averages
- Solar tracking (3): State + yaw%/pitch%
- Solar debug (5): Errors, duty cycles, photoresistor array
- Thermal (3): Temp, validity, fan%
- Energy hourly (5): Wh totals, peak W, timestamp
- Energy daily (6): Wh totals, peak W, active hours, timestamp
- System state (4): Power state, safety flags
- Validity (2): Battery/solar data valid
- Metadata (1): Snapshot timestamp

### `power_accumulator_rtc_t`

RTC RAM structure (persistent across resets):
- 15min averaging buffers
- Hourly totals (energy, peak, start time)
- Daily totals (energy, peak, hours active, start time)

## Integration

### Component Dependencies

**IMPLUVIUM (Irrigation):**
- Requests: `POWER_BUS_3V3`, `POWER_BUS_5V`, `POWER_BUS_12V`, level shifter
- Receives: Load shedding at VERY_LOW (15% SOC)

**TEMPESTA (Weather):**
- Requests: `POWER_BUS_3V3`
- Calls: `fluctus_hall_array_enable()` for wind direction (14mA savings)
- Receives: Power save at LOW_POWER (60min), shutdown at CRITICAL

**STELLARIA (Lighting):**
- Requests: `POWER_BUS_12V`
- Receives: Power save at POWER_SAVING (12.5% max), shutdown at LOW_POWER
- Uses: Averaged photoresistor light for auto mode

**HMI:**
- Requests: `POWER_BUS_3V3` for display/encoder
- Queries: `fluctus_get_*_state()` for controls
- Fetches: `telemetry_get_fluctus_data()` for display

**TELEMETRY:**
- Two sources: `TELEMETRY_SRC_FLUCTUS` (normal, 15-60min), `TELEMETRY_SRC_FLUCTUS_RT` (realtime, 500ms)
- Zero-copy writes via lock/unlock API

### External Libraries

- **ina219**: I2C power sensors
- **ds18x20 + onewire**: Temperature sensor
- **ads1115_helper**: Photoresistor reading with auto-recovery
- **mcp23008_helper**: Hall array enable (MCP23008 GP5)
- **interval_config**: Runtime config with LittleFS persistence
- **solar_calc**: NOAA sunrise/sunset callbacks

## Performance

**Memory:**
- Snapshot: ~700 bytes (55 fields × 4-8 bytes)
- Module state: ~1200 bytes (6 modules × 200 bytes)
- Total SRAM: ~2 KB (excluding task stacks)

**Task Stacks:**
- `fluctus_monitor`: 4096 bytes
- `fluctus_solar`: 4096 bytes (state machine)
- `fluctus_servo_control`: 4096 bytes (servo corrections)
- `fluctus_core_orq`: 2048 bytes

**Timing:**
- Power bus switching: <10ms
- INA219 read: 5-15ms (I2C)
- Photoresistor array: 20-40ms (4× ADS1115)
- Servo update: 2ms (LEDC)
- Telemetry write: 1-2μs (memcpy + mutex)

**Power Consumption:**
- INA219 active: 2mA
- Servos idle: 0mA (bus gated)
- Servos active: 200-400mA @ 6.6V (3-30s bursts)
- Fan active: 150mA @ 12V (duty cycle dependent)

## Configuration

Key defines in `fluctus.h`:

```c
// Power metering
#define FLUCTUS_POWER_ACTIVE_MONITOR_INTERVAL_MS    500
#define FLUCTUS_POWER_STEADY_MONITOR_INTERVAL_MS    (15 * 60 * 1000)

// Solar tracking - state machine
#define FLUCTUS_TRACKING_CORRECTION_INTERVAL_MS         (15 * 60 * 1000)
#define FLUCTUS_TRACKING_ADJUSTMENT_CYCLE_TIMEOUT_MS    30000
#define FLUCTUS_PHOTORESISTOR_THRESHOLD                 0.010f

// Solar tracking - servo control task
#define FLUCTUS_SERVO_CONTROL_LOOP_MS              250
#define FLUCTUS_SERVO_MAX_STEP_PER_ITERATION       15
#define FLUCTUS_SERVO_SETTLING_TIME_MS             3000
#define FLUCTUS_SERVO_SETTLING_COUNT               12  // (3000 / 250)

// Thermal
#define FLUCTUS_FAN_TURN_ON_TEMP_THRESHOLD          32.0f
#define FLUCTUS_FAN_TURN_OFF_TEMP_THRESHOLD         28.0f

// Battery (12V AGM)
#define FLUCTUS_BATTERY_LEVEL_POWER_SAVING          12.48f
#define FLUCTUS_BATTERY_LEVEL_CRITICAL              12.08f
#define FLUCTUS_BATTERY_HYSTERESIS                  0.10f

// Overcurrent
#define FLUCTUS_OVERCURRENT_THRESHOLD_1             3.0f
#define FLUCTUS_OVERCURRENT_THRESHOLD_2             4.0f
```

## Design Patterns

**Per-Module Mutex Isolation**: Each subsystem (power bus, monitoring, solar tracking, energy accumulation) owns its state and mutex. This prevents high-frequency tasks (500ms power monitoring) from blocking low-frequency operations (15min solar corrections), enabling true concurrent operation across subsystems.

**Event-Driven Load Shedding**: Battery monitoring runs independently at 500ms, detecting state changes and notifying a separate orchestration task. The orchestration task coordinates component shutdowns without blocking critical metering, using a clean separation between detection and action.

**Reference Counting with Named Consumers**: Power buses track up to 16 consumers by string ID. This enables safe shared resource access (multiple components can use 3.3V simultaneously) and provides debugging visibility into who requested what. 500ms debounce prevents rapid power cycling.

**Adaptive Sampling**: Power metering adjusts its polling strategy based on system activity - 500ms during active operations, 15-second probes every 15 minutes when stable, and complete PV shutdown at night. This reduces power consumption while maintaining responsiveness.

**Power Gating**: The 6.6V servo bus only powers up during the CORRECTING state (~3-5 seconds per 15-minute cycle), achieving 0.3-0.6% duty cycle. Combined with sensor-level gating (hall array 10ms enable), this minimizes idle power draw in solar-powered operation.

**Layered Telemetry**: Single unified snapshot structure supports both realtime (500ms, 26 fields) and normal (15min, 29 fields) data injection. Components write only changed fields, and the cache layer handles routing to HMI (1-4Hz pulls) and MQTT (hourly pushes) without duplicating state.

---

**Related Files:**
- Integration: `main/main.c`
- HMI rendering: `components/core/hmi/hmi_render_fluctus.c`
- Telemetry encoding: `components/core/telemetry/telemetry_msgpack.c`
