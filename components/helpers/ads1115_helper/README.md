# ADS1115_HELPER - Multi-Device ADC Management with Fault Recovery

**Author:** Piotr P. <quinkq@gmail.com>
**Part of Solarium** - ESP32-S3 smart irrigation and solar power management system
**Lines of Code:** ~740 (1 module + header)

## Overview

ADS1115_HELPER provides unified management for four ADS1115 16-bit I2C ADC devices across the Solarium system. The component abstracts device initialization, implements automatic fault recovery with two-tier retry strategies, and provides transparent read retry logic that eliminates the need for consumers to handle transient communication failures.

**Core Capabilities:**
- Four-device management (moisture sensors, mixed sensors, photoresistors, hall array)
- Transparent read retries (3 attempts × 50ms) - consumers see single-call success/failure
- Automatic device recovery with notification-based task (zero polling overhead when healthy)
- Two-tier retry strategy (fast 5s/10s/20s for transient issues, slow 1min/5min/10min for hardware faults)
- Power-aware operation (FLUCTUS 3.3V bus integration with reference counting)
- Diagnostic health monitoring (needs_slow_retries flag for faulty device detection)

**Hardware Summary:**
- 4× ADS1115 devices @ 0x48/0x49/0x4A/0x4B on I2C Bus B
- 16 total analog channels (4 per device)
- Configurable per-device gain (16-bit, 860 SPS max)
- Toggled 3.3V power bus via FLUCTUS

## Architecture

### Module Structure

```
ads1115_helper/
├── ads1115_helper.c         (~636 lines)  - Device management, retry logic, public API
└── ads1115_helper.h         (~115 lines)  - Configuration, types, function declarations
```

**Single-Module Design:** All functionality consolidated in one file with clear internal separation:
- Device initialization (per-device config, test reads)
- Read operations (transparent 3× retry, voltage calculation)
- Recovery task (notification-based, two-tier retry strategy)
- Public API (5 functions)

### Threading Model

| Task | Priority | Stack | Purpose | Wake Condition |
|------|----------|-------|---------|----------------|
| `ADS1115_Retry` | Med-5 | 1536B | Device recovery | Notification on device failure |

**Task Communication:**
- Consumer calls `ads1115_helper_read_channel()` → 3 read retries (synchronous, 50ms delays)
- All 3 retries fail → marks device failed → sends `xTaskNotify()` to retry task → returns error to consumer
- Retry task wakes → executes two-tier strategy → re-enables device on success
- Next consumer read → device ready again (transparent recovery)

**Priority Rationale:**
- Med-5 matches sensor reading tasks (FLUCTUS monitoring, TEMPESTA weather) - recovery shouldn't preempt active sensor operations but needs timely execution to restore functionality

### Data Flow

```
1. Consumer Read Request
   └─→ ads1115_helper_read_channel(device_id, channel, &raw, &voltage)
       ├─→ Check device initialized (return ESP_ERR_INVALID_STATE if not)
       ├─→ Request POWER_BUS_3V3 (10ms stabilization)
       ├─→ Retry Loop (3 attempts × 50ms):
       │   ├─→ Take mutex (100ms timeout)
       │   ├─→ Reconfigure device (gain, data rate) - required after power cycling
       │   ├─→ Set MUX channel (10ms settling)
       │   ├─→ Start conversion
       │   ├─→ Fixed delay (70ms @ 16 SPS, 15ms @ 128 SPS)
       │   ├─→ Read value
       │   ├─→ Calculate voltage (gain-aware)
       │   └─→ Release mutex
       ├─→ Success → Release power bus, return ESP_OK
       └─→ All retries failed → Mark device failed, notify retry task, return error

2. Device Failure Recovery (Notification-Based)
   └─→ Retry Task wakes on xTaskNotify
       ├─→ Phase 1: Fast Retries (3× exponential: 5s, 10s, 20s)
       │   └─→ retry_all_failed_devices() after each delay
       ├─→ Phase 2: Slow Retries (1min, 5min, 10min) + set needs_slow_retries flag
       │   └─→ retry_all_failed_devices() after each delay
       └─→ Phase 3: Continuous Monitoring (10min intervals until all recovered)
           └─→ Task suspends when no devices failed (wait for next notification)
```

### Initialization & Startup

**Default States:**
- All 4 devices: `initialized = false`, `needs_slow_retries = false`
- `retry_count = 0`, `next_retry_delay_ms = 5000ms`

**Startup Sequence:**
1. `ads1115_helper_init()` called (requires FLUCTUS initialized for 3.3V bus)
2. Request POWER_BUS_3V3 (100ms stabilization)
3. Create mutex for thread safety
4. Attempt initialization of all 4 devices:
   - Descriptor init (I2C Bus B, device-specific address)
   - Configure mode, data rate, gain (per-device settings in ads1115_devices[])
   - Test read (channel 0) to verify responsiveness
5. Create retry task (starts in suspended state, waits for notification)
6. Release POWER_BUS_3V3 (runtime ops request power as needed)
7. If any devices failed → notify retry task to start recovery
8. Return ESP_OK if at least 1 device initialized, ESP_FAIL if all failed

**Deployment Safety:**
- System remains functional with partial device failures (consumers check `ads1115_helper_is_device_ready()`)
- Failed devices automatically recover in background without user intervention
- Critical consumers (IMPLUVIUM irrigation) check device status before operations

**Dependency Requirements:**
- FLUCTUS must initialize before ads1115_helper (provides 3.3V bus infrastructure)
- I2C Bus B must be configured in main.h (I2C_BUS_B_PORT, I2C_BUS_B_SDA_PIN, I2C_BUS_B_SCL_PIN)

## Hardware Configuration

### I2C Device Allocation (Bus B)

| Device ID | I2C Address | Name | Usage | Config |
|-----------|-------------|------|-------|--------|
| 0 | 0x48 (GND) | Moisture_Sensors | Zones 0-4 soil moisture | 16 SPS, ±4.096V |
| 1 | 0x49 (VCC) | Mixed_Sensors | Zone 5 + pressure + 3.3V bus voltage | 16 SPS, ±4.096V |
| 2 | 0x4A (SCL) | Photoresistors | Solar tracking quadrants (TL/TR/BL/BR) | 128 SPS, ±4.096V |
| 3 | 0x4B (SDA) | Hall_array | Wind direction (8 cardinal sensors) | 128 SPS, ±4.096V |

**I2C Bus Configuration:**
- Bus: I2C_BUS_B (Port 1 typical)
- Pins: Defined in `main.h` (I2C_BUS_B_SDA_PIN, I2C_BUS_B_SCL_PIN)
- Speed: Standard 100kHz (I2C device stack default)
- Power: FLUCTUS 3.3V bus (toggled, reference counted)

**Data Rate Selection:**
- **16 SPS (62.5ms conversion)**: High accuracy for slow-changing signals (moisture, pressure)
- **128 SPS (7.8ms conversion)**: Faster sampling for real-time tracking (photoresistors, wind direction)

**Gain Configuration:**
- All devices: ±4.096V (ADS111X_GAIN_4V096)
- Resolution: 125μV/bit (16-bit ADC)
- Input range: 0-3.3V typical (resistive sensors, voltage dividers)

**Power Gating:**
- 3.3V bus enables all 4 devices simultaneously
- Consumers use FLUCTUS reference counting (multiple components share bus safely)
- Devices reset to default config (±2.048V gain, 128 SPS) after power cycle → helper reconfigures on every read

## Key Features

### 1. Transparent Read Retry Logic

I2C communication on a toggled power bus experiences transient failures from power stabilization, thermal drift, and EMI. Rather than forcing every consumer to implement retry loops, the helper absorbs this complexity with built-in 3-attempt retries (50ms delays). Consumers make single-call reads and only see failures after 150ms of persistent issues.

```c
// Consumer code - no retry handling needed
esp_err_t ret = ads1115_helper_read_channel(dev, ch, &raw, &voltage);
// Helper already tried 3 times if this fails
```

**Critical design decision:** Uses fixed conversion delays (70ms @ 16 SPS, 15ms @ 128 SPS) instead of polling the OS bit (conversion ready flag). Some ADS1115 modules have unreliable OS bit behavior, and fixed delays are more deterministic while avoiding I2C bus contention from repeated config register reads. The trade-off is conservative timing margins (12-92%) acceptable for sensor applications.

**Power cycling challenge:** ADS1115 devices reset to defaults (±2.048V gain, 128 SPS) when the 3.3V bus cycles. The helper reconfigures gain and data rate before every conversion, trading 20ms overhead for stateless robustness - correct operation regardless of when the bus cycled, no synchronization needed with FLUCTUS power management.

### 2. Two-Tier Device Recovery Strategy

The recovery strategy distinguishes between transient failures (power glitches, thermal settling) and persistent hardware faults (loose connections, marginal components). Fast exponential backoff (5s → 10s → 20s, total 35s) handles 95% of failures. If these fail, slow retries (1min → 5min → 10min, total 16min) target hardware issues while setting a diagnostic flag (`needs_slow_retries = true`) for predictive maintenance alerts. A final continuous monitoring phase (10min intervals) ensures eventual recovery even from catastrophic failures.

**Why two tiers?** Most I2C failures on toggled power buses resolve within 30 seconds (power rail stabilization, thermal equilibrium). Aggressive fast retries minimize downtime for these common cases. Slow retries avoid log spam and I2C bus congestion for the rare persistent failures that need human intervention anyway. The diagnostic flag enables later MQTT alerts: "Device 0 recovered but needed slow retries - check connections."

Power-aware logic checks 3.3V bus availability before attempts, gracefully deferring recovery during load shedding (15% SOC) rather than spamming failed I2C transactions.

### 3. Notification-Based Task Activation

The retry task blocks indefinitely (`portMAX_DELAY`) when all devices are healthy, consuming zero CPU cycles. Read failures send notifications that wake the task immediately. This avoids both polling overhead (vs timer-based periodic checks) and delayed recovery (vs 10-minute intervals). Multiple simultaneous failures coalesce into a single notification, preventing retry spam while maintaining responsiveness. When recovery completes, the task returns to blocked state until the next failure.

### 4. Per-Device Configuration

Different sensor types have different speed/accuracy requirements. Moisture and pressure sensors (Devices 0-1) use 16 SPS (62.5ms conversion) for high accuracy on slow-changing signals used in watering decisions. Photoresistors and hall arrays (Devices 2-3) use 128 SPS (7.8ms conversion) for faster sampling of dynamic signals in solar tracking (3s correction cycles) and wind direction (5s samples). Each device declares its configuration in the `ads1115_devices[]` array (mode, data rate, gain, name).

### 5. Diagnostic Health Monitoring

Each device maintains a `needs_slow_retries` flag set when recovery requires the slow retry phase (>35s). This persistent indicator signals intermittent hardware faults (loose connections, marginal solder joints, insufficient 3.3V regulation) that resolve eventually but warrant preventive maintenance. Future MQTT integration can alert: "Device recovered but required slow retries - inspect hardware." The flag persists across successful recoveries, providing historical insight into system reliability.

## Public API

**Initialization:**
- `ads1115_helper_init()` - Initialize all devices, create retry task (requires FLUCTUS initialized for 3.3V bus)

**Reading Channels:**
- `ads1115_helper_read_channel(device_id, channel, raw, voltage)` - Single-shot conversion with transparent 3× retry, optional voltage calculation
- `ads1115_helper_is_device_ready(device_id)` - Fast availability check

**Diagnostics:**
- `ads1115_helper_get_device_info(device_id, device_info)` - Full device state including `needs_slow_retries` health flag

## Data Structures

### `ads1115_device_t`

Per-device state structure (4 instances in `ads1115_devices[]` array):

```c
typedef struct {
    // I2C Communication
    i2c_dev_t device;             // Low-level I2C device handle (from i2cdev library)

    // Runtime State (managed by helper)
    bool initialized;             // Device ready for reads
    uint32_t last_retry_time;     // Timestamp of last retry attempt (milliseconds)
    uint8_t retry_count;          // Current retry attempt number
    uint32_t next_retry_delay_ms; // Next retry delay (exponential backoff)
    bool needs_slow_retries;      // Diagnostic flag: device required slow retry phase

    // Device Configuration (set at compile-time in ads1115_devices[])
    const char *name;             // "Moisture_Sensors", "Photoresistors", etc.
    ads111x_mode_t mode;          // ADS111X_MODE_SINGLE_SHOT (all devices)
    ads111x_data_rate_t data_rate;// 16 SPS or 128 SPS (device-specific)
    ads111x_gain_t gain;          // ADS111X_GAIN_4V096 (±4.096V, all devices)
} ads1115_device_t;
```

**Memory:** 4 devices × ~80 bytes = ~320 bytes (includes i2c_dev_t handle)

**Access Patterns:**
- Initialization: Write-once setup of name/mode/data_rate/gain
- Runtime: Read/write initialized flag, retry state during recovery
- Diagnostics: Read-only access via `ads1115_helper_get_device_info()`

**Thread Safety:** Protected by `xADS1115Mutex` (100ms timeout standard)

### Device Array

```c
extern ads1115_device_t ads1115_devices[ADS1115_DEVICE_COUNT];
// Global array - 4 devices
// Device 0: 0x48 - Moisture sensors (Zones 0-4)
// Device 1: 0x49 - Mixed sensors (Zone 5, pressure, 3.3V bus voltage)
// Device 2: 0x4A - Photoresistors (solar tracking TL/TR/BL/BR)
// Device 3: 0x4B - Hall array (wind direction 8-point)
```

## Design Patterns

**Transparent Retry Abstraction**: Consumers make single-call reads without handling transient I2C failures. The helper silently retries up to 3 times with 50ms delays, only failing if the issue persists beyond 150ms. This eliminates repetitive error-handling boilerplate across all sensor drivers while maintaining deterministic timing (consumers know reads complete within 150ms worst-case).

**Two-Tier Failure Recovery**: Fast exponential backoff (5s/10s/20s) handles 95% of failures - power stabilization, thermal drift, bus contention. If those fail, slow retries (1min/5min/10min) target persistent hardware issues like loose connections or marginal components. The `needs_slow_retries` flag persists as a diagnostic indicator, enabling predictive maintenance alerts for intermittent faults that resolve eventually but signal hardware degradation.

**Notification-Based Task Activation**: The retry task blocks indefinitely when all devices healthy, consuming zero CPU cycles. Read failures send notifications that wake the task immediately, avoiding both polling overhead and delayed recovery. Multiple simultaneous failures coalesce into single notification, preventing retry spam while maintaining responsiveness.

**Power-Aware Reference Counting Integration**: Every read operation requests POWER_BUS_3V3 via FLUCTUS, correctly incrementing the reference count so the bus stays powered across overlapping operations from multiple components. Retry logic checks bus availability before attempts, gracefully deferring recovery during load shedding scenarios (15% SOC) rather than spamming failed I2C transactions.

**Stateless Per-Read Reconfiguration**: ADS1115 devices reset to default config (±2.048V gain, 128 SPS) when the 3.3V bus cycles. Instead of tracking power state, the helper reconfigures gain and data rate before every conversion. This trades 20ms overhead for robustness - correct operation regardless of when the bus cycled, no synchronization needed with FLUCTUS power management.

## Integration

### Component Dependencies

**FLUCTUS (Power Management):**
- Requests: `POWER_BUS_3V3` for all I2C operations (reference counted)
- Timing: 10ms stabilization on power-up, 100ms on system init
- Load shedding: Retry logic checks bus availability, defers attempts if unavailable (15% SOC)

**IMPLUVIUM (Irrigation):**
- Reads: Device 0 (Zones 0-4 moisture), Device 1 Ch0 (Zone 5 moisture)
- Reads: Device 1 Ch1 (ABP pressure sensor analog output)
- Safety: Checks `ads1115_helper_is_device_ready()` before watering cycles
- Voltage sharing: Moisture readings used for learning algorithm calibration

**FLUCTUS (Solar Tracking):**
- Reads: Device 2 all channels (photoresistor array TL/TR/BL/BR)
- Frequency: Every 3s during CORRECTING state (15min intervals)
- Data sharing: Stores photoresistor readings in FLUCTUS snapshot → telemetry cache → STELLARIA auto mode

**TEMPESTA (Weather Station):**
- Reads: Device 3 all channels (hall sensor array for wind direction)
- Power gating: Hall array enabled via MCP23008 GP5 (10ms pulse per read)
- Frequency: 5s samples during normal operation

**STELLARIA (Lighting):**
- Indirect usage: Receives averaged photoresistor values from FLUCTUS via telemetry cache
- No direct helper calls (data flow: helper → FLUCTUS → telemetry → STELLARIA)

### External Libraries

- **i2cdev** (`components/custom_drivers/i2cdev`): Low-level I2C device abstraction (descriptor management, mutex, error codes)
- **ads111x** (from esp-idf-lib): ADS1115/ADS1015 driver (config, conversion, register access)

## Performance

**Memory Footprint:**
- Device state: 4 × 80 bytes = 320 bytes (includes i2c_dev_t handles)
- Mutex: ~80 bytes (FreeRTOS semaphore)
- Task stack: 1536 bytes
- **Total: ~1940 bytes SRAM**

**Task Resources:**
- Priority: Med-5 (matches sensor reading tasks)
- Stack: 1536 bytes (`configMINIMAL_STACK_SIZE × 3`)
- CPU: 0% when all devices healthy (notification-blocked)
- CPU: <1% during recovery phases (I2C transactions + delays)

**Timing Characteristics:**

| Operation | Duration | Notes |
|-----------|----------|-------|
| Read (16 SPS) | 100-120ms | 70ms conversion + 30ms setup/mutex |
| Read (128 SPS) | 35-50ms | 15ms conversion + 20ms setup/mutex |
| Read with 3 retries | 300-360ms | Worst case (16 SPS, all retries) |
| Device init | 200-250ms | Descriptor + config + test read |
| Fast retry phase | 35s | 3 attempts: 5s + 10s + 20s |
| Slow retry phase | 16min | 3 attempts: 1min + 5min + 10min |

**Power Consumption:**
- ADS1115 active: 150μA per device (4 devices = 600μA total)
- ADS1115 idle: 0.5μA per device (toggled power bus)
- I2C bus active: ~5mA @ 3.3V during transactions (shared with other Bus B devices)
- System impact: Negligible (<2mW) due to toggled 3.3V bus and single-shot mode

**I2C Bus Contention:**
- Bus B shared with: MCP23008 (encoder/pulses), SHT4x (temp/humidity), BMP280 (pressure), AS5600 (wind speed)
- Read frequency: Variable per consumer (3s solar tracking, 5s wind, 15min normal)
- Mutex prevents simultaneous access to same device
- No bus-level locking (i2cdev handles per-transaction bus arbitration)

## Configuration

Key compile-time constants (defaults tuned for solar-powered operation):

```c
#define ADS1115_DEVICE_COUNT 4                     // 4 devices on Bus B
#define ADS1115_READ_RETRY_ATTEMPTS 3              // Transparent retries
#define ADS1115_READ_RETRY_DELAY_MS 50             // Inter-retry delay
#define ADS1115_INIT_FAST_RETRY_DELAY_MS 5000      // First fast retry: 5s (exponential: 5s→10s→20s)
#define ADS1115_INIT_SLOW_RETRY_DELAY_1_MS 60000   // Slow retries: 1min, 5min, 10min
```

Per-device addresses (0x48/0x49/0x4A/0x4B) and configurations (data rate, gain) set in `ads1115_devices[]` array. I2C Bus B pins defined in `main.h`.

## Known Limitations

- **No differential mode support**: API only exposes single-ended channels (ADS111X_MUX_0_GND to ADS111X_MUX_3_GND). Differential measurements (ADS111X_MUX_0_1, etc.) require direct ads111x driver access.

- **Fixed conversion delays**: Uses data rate-based fixed delays instead of OS bit polling. Conservative margins (12-92%) acceptable for sensor applications but wasteful if maximum throughput required.

- **Retry task never exits**: Continuous monitoring phase (Phase 3) runs indefinitely for unrecoverable hardware failures. Could accumulate many sleeping tasks if components repeatedly re-initialize helper.

- **Power bus dependency**: Initialization fails if FLUCTUS not initialized first or 3.3V bus unavailable. No fallback or queued initialization.

---

**Related Files:**
- Integration: `main/main.c` (initialization order), `components/core/fluctus/fluctus_power_bus.c` (3.3V bus)
- Consumers: `components/core/impluvium/impluvium_sensors.c`, `components/core/fluctus/fluctus_solar_tracking.c`, `components/core/tempesta/tempesta_sensors.c`
- Dependency: `components/custom_drivers/i2cdev/`, `esp-idf-lib ads111x driver`
