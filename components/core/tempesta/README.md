# TEMPESTA - Weather Station Component

**Author:** Piotr P. <quinkq@gmail.com>
**Part of Solarium** - ESP32-S3 smart irrigation and solar power management system
**Lines of Code:** ~2827 (5 modules + core + wind task)

Multi-sensor environmental monitoring for ESP32-S3 with power optimization and adaptive polling.

---

## Overview

TEMPESTA orchestrates 8 sensors for comprehensive weather monitoring: temperature, humidity, pressure, air quality, wind (speed/direction), rainfall, and tank intake. Designed for solar-powered operation with sensor-level gating, adaptive intervals, and progressive load shedding.

**Core Capabilities:**
- **8-Sensor Array:** Environmental + wind + precipitation monitoring
- **Power Optimization:** Sensor gating (10ms hall array), UART sleep (PMS5003), low-power modes (AS5600 LPM3)
- **Adaptive Polling:** 15min normal / 60min power-save (configurable 5-120min)
- **Time Tracking:** Hourly/daily/weekly accumulation with midnight callbacks
- **Per-Sensor Status:** Individual health tracking (OK/UNAVAILABLE/ERROR/WARMING_UP)
- **Diagnostic Mode:** Real-time hardware validation for HMI debugging

**Hardware:**
- **I2C (Bus B, 3.3V):** SHT4x (temp/humidity), BMP280 (pressure), AS5600 (wind speed)
- **UART2:** PMS5003 air quality (PM2.5/PM10, 35s warmup, 5V)
- **MCP23008 (I2C @ 0x20):** GP3 rainfall counter, GP4 tank counter, GP5 hall enable
- **ADS1115-3 (I2C @ 0x4B):** 4-channel hall array (wind direction)

---

## Architecture

### Module Structure (7 files, 2827 lines)

```
tempesta.c                  (849)  - Core orchestration, state machine, power, API
tempesta_sensors.c          (708)  - Hardware init, I2C/UART reading, pulse counters
tempesta_processing.c       (265)  - Averaging algorithms, sensor fusion
tempesta_accumulator.c      (445)  - Hourly/daily/weekly tracking, unit conversion
tempesta_wind.c             (142)  - AS5600 sampling task (10Hz, 5s)
tempesta_private.h                 - Shared declarations
tempesta.h                         - Public API
```

### State Machine (5 states)

```
DISABLED ←→ IDLE ←→ READING
             ↓        ↓
          SHUTDOWN ───┘
             ↓
          ERROR (planned)
```

- **DISABLED:** No polling, timer stopped
- **IDLE:** Awaiting collection cycle
- **READING:** Active sensor polling (3.3V/5V buses powered)
- **SHUTDOWN:** Load shedding (FLUCTUS battery VERY_LOW)
- **ERROR:** Multiple sensor failures (not implemented - TODO in tempesta.c:342)

### Data Flow

```
Timer (15min/60min) → Main Task Notification → IDLE → READING

├─ Power: Request 3.3V + 5V (PMS5003 optional)
├─ I2C: SHT4x, BMP280 (1s stabilization)
├─ Wind: AS5600 task trigger (5s @ 10Hz, parallel)
├─ Hall: 4-channel direction read (10ms gate)
├─ Pulse: Rainfall/tank counters (MCP23008)
├─ Air: PMS5003 warmup + read (35s, if enabled)
├─ Process: 10-sample averaging, sensor fusion
├─ Accumulate: Hourly rollover, daily/weekly totals
├─ Telemetry: Direct cache write (zero-copy)
└─ Power Down: Release buses, AS5600 LPM3, hall disable

READING → IDLE
```

### FreeRTOS Tasks

**Main Task** (`tempesta_main_task`, Pri 5, 4KB):
- Event-driven collection on timer notification
- Orchestrates sensor sequence with power management
- State transitions, telemetry injection

**Wind Task** (`tempesta_as5600_sampling_task`, Pri 6, 4KB):
- Independent AS5600 sampling (10Hz, 5s)
- RPM calculation with wrap-around handling
- Auto LPM3 management

**Collection Timer** (`xTempestaCollectionTimer`):
- Dynamic period (normal/power-save mode)
- Stopped during DISABLED/SHUTDOWN

### Thread Safety

**Mutex:** `xTempestaDataMutex` protects `weather_data` + `calculation_data`
- Quick reads: 100ms timeout
- Updates: 1000ms timeout

### Task Priority & Synchronization

**Two-Task Design:**
- `tempesta_main_task` (Med-5): Collection orchestration, state machine
- `tempesta_as5600_sampling_task` (Med-6): Wind speed sampling (independent)

**Priority Rationale:**
- Med-5 main task: Time-sensitive sensor readings (I2C, SPI, UART) with strict timing requirements
- Med-6 wind task: Higher priority than main (10Hz sampling must not miss deadline during concurrent I2C operations)
- Independent operation: Wind task doesn't block main collection cycle

**Synchronization:**
- Main task triggers wind task via notification when entering READING state
- Wind task runs parallel 5s sampling window (50 readings @ 10Hz)
- No shared data during sampling (results written after completion)

### Initialization & Startup

**Default States:**
- State machine: **IDLE** (timer started, awaiting first collection)
- Power buses: All unpowered (ref_count = 0)
- Collection timer: Started with normal interval (default 15min)
- PMS5003: Enabled (can be disabled via load shedding)

**Initialization Order Dependency:**
- FLUCTUS must initialize first (power bus infrastructure)
- `interval_config` must load before TEMPESTA (collection intervals from LittleFS)
- TELEMETRY must be ready for cache writes
- `solar_calc` should register callbacks before TEMPESTA (midnight reset integration)

**First Boot Behavior:**
- Starts in IDLE state, timer fires after configured interval (default 15min)
- Does not collect immediately (waits for first timer cycle)
- RTC accumulator initializes to zero (first hourly/daily rollover occurs naturally)

---

## Hardware Configuration

### I2C Sensors (Bus B, 3.3V)

| Device | Address | Function | Notes |
|--------|---------|----------|-------|
| SHT4x | 0x44 | Temperature/Humidity | 10-sample averaging |
| BMP280 | 0x76 | Pressure | Direct reading (no averaging) |
| AS5600 | 0x36 | Wind speed (magnetic encoder) | LPM3 idle mode (5mA savings) |
| ADS1115-3 | 0x4B | Wind direction (4-ch hall array) | N-MOSFET gated via MCP23008 GP5 |
| MCP23008 | 0x20 | I/O expander | Rainfall/tank pulse counters, hall enable |

### UART Sensors

**PMS5003 Air Quality (UART2):**
- TX: GPIO43
- RX: GPIO44
- Power: 5V bus (gated via FLUCTUS)
- Warmup: 35 seconds
- UART sleep mode between cycles (100mA savings)

### MCP23008 GPIO Assignments

| Pin | Function | Type | Notes |
|-----|----------|------|-------|
| GP3 | Rainfall counter | Input (interrupt) | 100mL per pulse (50mL per tip × 2 tips) |
| GP4 | Tank intake counter | Input (interrupt) | 50mL per pulse |
| GP5 | Hall array enable | Output (N-MOSFET) | HIGH=ON, 10ms gate per cycle (14mA savings) |

**Counter Details:**
- 16-bit signed counters (software tracking)
- Base-counting accumulation (weekly reset prevents overflow)
- Software counting via `mcp23008_helper` interrupt handler

### Sensor Calibration Constants

**Wind:**
- `WEATHER_WIND_RPM_TO_MS_FACTOR = 1.0f` (TODO: requires physical calibration)
- AS5600 resolution: 12-bit (0.088° precision)

**Wind Direction:**
- 4-channel hall array → 8 cardinal points (45° sectors: N/NE/E/SE/S/SW/W/NW)
- Threshold voltage: 0.2V minimum for valid reading

**Precipitation:**
- Rainfall: 100mL per pulse (2 tips × 50mL tipbucket)
- Tank intake: 50mL per pulse
- Weekly reset prevents MCP23008 16-bit overflow

**Temperature/Humidity:**
- 10-sample rolling average (150min @ 15min collection)
- Sensor fusion: SHT4x + BMP280 (temperature), SHT4x + BME280 if equipped (humidity)

---

## Key Features

### 1. Sensor Suite & Status Tracking

**Sensors (8):**
- Temperature: SHT4x + BMP280 fusion, 10-sample average
- Humidity: SHT4x + BME280 (if equipped), 10-sample average
- Pressure: BMP280 direct (no averaging)
- Air Quality: PMS5003 PM2.5/PM10 (35s warmup, UART sleep)
- Wind Speed: AS5600 magnetic encoder (RPM + m/s calibration)
- Wind Direction: 4-channel hall → 8 cardinal points (N/NE/E/SE/S/SW/W/NW)
- Rainfall: 100mL tipbucket (hourly/daily/weekly mm)
- Tank Intake: 50mL tipbucket (hourly/daily/weekly mL)

**Per-Sensor Status:**
```c
typedef enum {
    WEATHER_SENSOR_OK,           // Operational
    WEATHER_SENSOR_UNAVAILABLE,  // Init failed
    WEATHER_SENSOR_ERROR,        // Read failure
    WEATHER_SENSOR_WARMING_UP    // PMS5003 only
} weather_sensor_status_t;
```

### 2. Power Management

**Bus Reference Counting:**
```c
FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_3V3, "TEMPESTA", return ESP_FAIL);
if (pms5003_enabled) {
    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_5V, "TEMPESTA", {...});
}
```

**Sensor Gating:**
- Hall array: 10ms ON per cycle (14mA savings)
- AS5600: LPM3 idle (5mA savings)
- PMS5003: UART sleep between cycles (100mA savings)

**Load Shedding:**
- 40% SOC: Power-save mode (60min cycle)
- 15% SOC: Disable PMS5003 (5V bus)
- 0% SOC: Full shutdown

### 3. Time-Based Accumulation

**Hourly Tracking:** Monotonic time, separate "last hour" (stable) + "current hour" (debug)
**Daily Reset:** Midnight callback via solar_calc
**Weekly Reset:** Boot day (prevents MCP23008 16-bit overflow)

### 4. Diagnostic Mode (HMI)

```c
tempesta_enter_diagnostic_mode();   // Pauses collection, continuous sensing
tempesta_get_diagnostic_data(&diag); // 125ms refresh (8Hz)
tempesta_exit_diagnostic_mode();     // Restore normal operation
```

**Diagnostic Data:**
- Hall array: 4 voltages, direction (deg + cardinal), magnitude
- AS5600: Raw angle, degrees, rotation count, magnet status

---

## Public API

### Initialization & Control

```c
esp_err_t tempesta_station_init(void);
esp_err_t tempesta_set_system_enabled(bool enable);  // DISABLED ↔ IDLE
esp_err_t tempesta_force_collection(void);           // Trigger immediate read
```

### Data Retrieval

```c
esp_err_t tempesta_get_data_snapshot(tempesta_snapshot_t *data);
esp_err_t tempesta_write_to_telemetry_cache(tempesta_snapshot_t *cache);
float tempesta_get_temperature(void);  // IMPLUVIUM integration
tempesta_state_t tempesta_get_state(void);
```

### Load Shedding

```c
esp_err_t tempesta_set_power_save_mode(bool enable);      // 15min ↔ 60min
esp_err_t tempesta_set_shutdown(bool shutdown);           // Any ↔ SHUTDOWN
esp_err_t tempesta_set_pms5003_enabled(bool enable);      // Air quality toggle
esp_err_t tempesta_set_collection_intervals(uint32_t normal_min, uint32_t power_save_min);
```

### Counter Management

```c
esp_err_t tempesta_reset_daily_counters(void);
esp_err_t tempesta_reset_weekly_counters(void);
esp_err_t tempesta_reset_rain_gauge_total(void);    // WARNING: clears hardware
esp_err_t tempesta_reset_tank_intake_total(void);   // WARNING: clears hardware
```

### Diagnostic Mode

```c
esp_err_t tempesta_enter_diagnostic_mode(void);
void tempesta_exit_diagnostic_mode(void);
esp_err_t tempesta_get_diagnostic_data(tempesta_diag_snapshot_t *diag_data);
bool tempesta_is_diagnostic_mode_active(void);
```

---

## Integration

### Component Dependencies

**FLUCTUS (Power & Hall Gating):**
- Provides 3.3V/5V bus power via reference counting
- Supplies hall array enable control (14mA savings via 10ms gating)
- Coordinates progressive load shedding (power save @ 40% SOC, PMS5003 disable @ 15%, shutdown @ 0%)

**TELEMETRY (Data Hub):**
- Zero-copy snapshot injection at cycle completion
- HMI retrieves cached data at 1Hz (normal) or 8Hz (diagnostic mode)

**IMPLUVIUM (Temperature Sharing):**
- TEMPESTA provides `tempesta_get_temperature()` for irrigation interval calculation
- Temperature data affects moisture evaporation rate (±1%/°C correction factor)

**External Libraries:**
- `solar_calc`: Midnight callback registration for daily counter reset
- `interval_config`: Persistent collection interval storage (LittleFS)
- `mcp23008_helper`: Software pulse counting for rainfall/tank intake
- `ads1115_helper`: Hall array ADC with auto-recovery

---

## Data Structures

### `tempesta_snapshot_t`

Main telemetry snapshot (~400 bytes):

```c
typedef struct {
    // Environmental sensors
    float temperature;                      // Averaged (°C)
    float humidity;                         // Current (%)
    float pressure;                         // Current (hPa)
    float air_quality_pm25;                 // PM2.5 (μg/m³)
    float air_quality_pm10;                 // PM10 (μg/m³)

    // Wind measurements
    float wind_speed_rpm;                   // Rotations per minute
    float wind_speed_ms;                    // Meters per second
    float wind_direction_deg;               // 0-360° (0=North, clockwise)
    char wind_direction_cardinal[4];        // N/NE/E/SE/S/SW/W/NW

    // Rainfall accumulation
    float rainfall_last_hour_mm;            // Completed hour (stable)
    float rainfall_current_hour_mm;         // Hour in progress (debug)
    float rainfall_daily_mm;                // Today's total
    float rainfall_weekly_mm;               // Week total (boot day reset)

    // Tank intake accumulation
    float tank_intake_last_hour_ml;         // Completed hour (stable)
    float tank_intake_current_hour_ml;      // Hour in progress (debug)
    float tank_intake_daily_ml;             // Today's total
    float tank_intake_weekly_ml;            // Week total (boot day reset)

    // System state
    tempesta_state_t state;                 // DISABLED/IDLE/READING/SHUTDOWN/ERROR
    bool power_save_mode;                   // 15min vs 60min cycle

    // Per-sensor status (8 fields)
    weather_sensor_status_t temp_sensor_status;
    weather_sensor_status_t humidity_sensor_status;
    weather_sensor_status_t pressure_sensor_status;
    weather_sensor_status_t air_quality_status;
    weather_sensor_status_t wind_sensor_status;
    weather_sensor_status_t wind_direction_status;
    weather_sensor_status_t rain_gauge_status;
    weather_sensor_status_t tank_intake_status;

    time_t snapshot_timestamp;              // Unix epoch
} tempesta_snapshot_t;
```

### `tempesta_diag_snapshot_t`

Diagnostic data for HMI hardware validation:

```c
typedef struct {
    // Hall array (wind direction debugging)
    float hall_voltage_north;               // North sensor (V)
    float hall_voltage_east;                // East sensor (V)
    float hall_voltage_south;               // South sensor (V)
    float hall_voltage_west;                // West sensor (V)
    float hall_direction_deg;               // Calculated 0-360°
    char hall_direction_cardinal[4];        // N/NE/E/SE/S/SW/W/NW
    float hall_magnitude;                   // Signal strength 0.0-1.0

    // AS5600 (wind speed debugging)
    uint16_t as5600_raw_angle;              // Raw counts 0-4095
    float as5600_angle_deg;                 // Degrees 0-360°
    int32_t as5600_rotation_count;          // Total rotations since diag start
    bool as5600_magnet_detected;            // Magnet presence
    bool as5600_magnet_too_weak;            // Increase proximity
    bool as5600_magnet_too_strong;          // Decrease proximity

    time_t snapshot_timestamp;
} tempesta_diag_snapshot_t;
```

**Update Frequency:**
- Main snapshot: Cycle completion (every 15-60min depending on mode)
- Diagnostic snapshot: On-demand during diagnostic mode (8Hz capable, 125ms refresh)

---

## Configuration

### Runtime Parameters

**Collection Intervals:**
- Normal mode: 5-60min (default 15min) - configurable via `interval_config` + LittleFS persistence
- Power-save mode: 15-120min (default 60min) - triggered by FLUCTUS @ 40% SOC
- Adaptive polling reduces power consumption during low-battery conditions

---

## Design Patterns

**Parallel Task Architecture**: Wind speed sampling runs in a separate higher-priority task (Med-6 vs Med-5) to ensure precise 10Hz timing isn't disrupted by main task I2C/UART operations. The main task triggers wind sampling via notification, and both execute concurrently during the 5-second wind measurement window. This prevents I2C delays from corrupting RPM calculations.

**Sensor-Level Power Gating**: Instead of powering buses continuously, TEMPESTA gates individual sensors: hall array (10ms ON per cycle, 14mA savings), AS5600 LPM3 during idle (5mA savings), PMS5003 UART sleep (100mA savings). This fine-grained control reduces average power consumption from 130mA to <10mA while maintaining full 8-sensor functionality.

**Per-Sensor Health Tracking**: Each of the 8 sensors maintains independent status (OK/UNAVAILABLE/ERROR/WARMING_UP) rather than a global component state. This allows graceful degradation - if the PMS5003 fails, temperature/humidity/wind data remains valid. HMI can display partial data and highlight sensor faults without losing all weather information.

**Dual Accumulation Windows**: Rainfall and tank intake tracking uses separate "last hour" (stable, completed data) and "current hour" (debug, in-progress) fields. This prevents HMI from displaying fluctuating values during hourly rollover while still providing real-time debug visibility. The "last hour" value only updates at the 60-minute mark, giving consistent MQTT/HMI readings.

**Time-Based Weekly Reset**: MCP23008 pulse counters are 16-bit signed (-32768 to +32767). Instead of tracking absolute pulses indefinitely, the system resets weekly on boot day. At 100mL rainfall per pulse, this allows 3276 liters (3.2m³) per week before overflow - far exceeding typical rainfall patterns while preventing counter wrap-around corruption.

**Diagnostic Mode Parallelism**: When HMI enters diagnostic mode, normal collection pauses but sensors remain powered for continuous 8Hz readouts. This gives hardware validation without interfering with normal operation cycles. The diagnostic path reads raw sensor values (AS5600 angle, hall voltages) that aren't normally exposed, enabling field troubleshooting of mechanical alignment issues.

---

### Compile-Time Constants

**Timing:**
- `WEATHER_AS5600_SAMPLE_DURATION_MS = 5000` - Wind speed sampling duration (10Hz × 5s = 50 samples)
- `WEATHER_PMS5003_WARMUP_TIME_MS = 35000` - Air quality sensor warmup period

**Calibration:**
- `WEATHER_AVERAGING_SAMPLES = 10` - Temperature/humidity rolling average window (150min @ 15min)
- `WEATHER_RAIN_ML_PER_PULSE = 100.0f` - Rainfall tipbucket calibration (2 tips × 50mL)
- `WEATHER_TANK_INTAKE_ML_PER_PULSE = 50.0f` - Tank intake tipbucket
- `WEATHER_WIND_RPM_TO_MS_FACTOR = 1.0f` - Wind speed conversion (TODO: requires physical calibration)
- `WEATHER_WIND_DIR_THRESHOLD_VOLTAGE = 0.2f` - Hall sensor minimum valid reading

---

**Related Files:**
- Integration: `main/main.c`
- HMI rendering: `components/core/hmi/hmi_render_tempesta.c`
- Telemetry encoding: `components/core/telemetry/telemetry_msgpack.c`

