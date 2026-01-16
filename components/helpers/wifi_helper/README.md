# WIFI_HELPER - Power-Aware WiFi Management

**Author:** Piotr P. <quinkq@gmail.com>
**Part of Solarium** - ESP32-S3 smart irrigation and solar power management system
**Lines of Code:** ~750 (2 modules)

## Overview

WIFI_HELPER provides power-optimized WiFi connectivity with intelligent reconnection strategies and load shedding integration. Designed for battery-powered operation, it balances network reliability with energy conservation through adaptive power modes and exponential backoff algorithms.

**Core Capabilities:**
- Automatic reconnection with exponential backoff (1s → 60s, max 20 retries)
- Three power states: Enabled (~100mA) / Power-Save (~20mA) / Shutdown (0mA)
- RSSI monitoring with threshold-based telemetry injection (>5dBm delta)
- SNTP time synchronization for solar calculations (sunrise/sunset)
- Anti-bounce delayed restart (30s) to prevent rapid cycling
- Detailed disconnect diagnostics and connection time tracking

**Hardware:**
- ESP32-S3 integrated WiFi (802.11 b/g/n, 2.4GHz STA mode)
- No external hardware

## Architecture

### Module Structure

```
wifi_helper/
├── wifi_helper.c       (638 lines)  - Init, event handling, power management
├── wifi_helper.h       (108 lines)  - Public API, state machine, data structures
└── README.md           (this file)
```

### State Machine

Six operational states with event-driven transitions:

```
┌──────────────────────────────────────────────────────────────────────┐
│ NORMAL OPERATION CYCLE                                               │
└──────────────────────────────────────────────────────────────────────┘

    wifi_helper_init()
           │
           └──► [INIT]
                  │
                  │ esp_wifi_start() triggers WIFI_EVENT_STA_START
                  │
                  ▼
              CONNECTING ◄───────────────────────────────────┐
                  │                                          │
                  │ IP_EVENT_STA_GOT_IP                      │
                  │                                          │
                  ▼                                          │
              CONNECTED                                      │
                  │                                          │
           ┌──────┴──────────┐                               │
           │                 │                               │
    RSSI monitoring   WIFI_EVENT_STA_                        │
    (15min cycle)     DISCONNECTED                           │
                              │                              │
                              ▼                              │
                        RECONNECTING                         │
                              │                              │
                   ┌──────────┴─────────┐                    │
                   │                    │                    │
             retry < 30           retry >= 30                │
         exponential backoff            │                    │
            1s→2s→...→10m→60min         │                    │
                   │                    │                    │
          esp_wifi_connect()            │                    │
          (back to CONNECTING)          ▼                    │
              retry_count++;         FAILED                  │
                   │                    │                    │
                   │                    │ << wifi_helper_    │
                   │                    │ force_reconnect()  │
                   │                    │ (manual reset)     │
                   │                    │                    │
                   └────────────────────┴────────────────────┘

┌──────────────────────────────────────────────────────────────────────┐
│ SHUTDOWN / RESTART FLOW (FLUCTUS Load Shedding)                      │
└──────────────────────────────────────────────────────────────────────┘

    Any State
       │
       │ wifi_helper_set_shutdown(true)
       │ [CRITICAL battery: 0% SOC]
       │
       ▼
    DISABLED
       │
       │ wifi_helper_set_shutdown(false)
       │ [Battery recovered: >15% SOC]
       │
       ▼
    30-second anti-bounce timer
       │
       │ (prevents rapid cycling during voltage sag)
       │
       ▼
    esp_wifi_start()
       │
       └──► CONNECTING (resumes normal cycle)
```

**State Descriptions:**

- **INIT**: Initial state during component initialization
- **CONNECTING**: First connection attempt in progress (before any disconnects)
- **CONNECTED**: Active connection with IP address assigned, RSSI monitoring active
- **RECONNECTING**: Lost connection, exponential backoff timer running before next attempt
- **FAILED**: Max retry limit exceeded (30 attempts), requires manual `wifi_helper_force_reconnect()`
- **DISABLED**: WiFi subsystem powered down (CRITICAL battery state or user shutdown)

### Threading Model

**Tasks:**
- `wifi_rssi` (Med-5, 4KB stack): RSSI monitoring, SNTP initialization, connection time tracking

**FreeRTOS Timers:**
- `wifi_reconnect` (one-shot): Delayed reconnection with exponential backoff
- `wifi_restart_delay` (one-shot, 30s): Anti-bounce delayed restart

**Priority Rationale:**
- Med-5: Network monitoring is not time-critical but should respond reasonably to connection events

### Data Flow

```
WiFi Events ──────────┬──► State Machine ───┬──► TELEMETRY Cache
                      │                     │    (state changes)
                      │                     │
RSSI Monitor Task ────┴──► Periodic Checks ─┴──► TELEMETRY Cache
(15min intervals)                                  (>5dBm delta)


External Control ────────► Power Save Mode ────► esp_wifi_set_ps()
(FLUCTUS)                  or Shutdown            esp_wifi_stop()
```

### Initialization & Startup

**Default State:**
- Starts in **INIT** state
- Power save mode **ENABLED** by default (`WIFI_PS_MAX_MODEM`, ~20mA)
- Automatic connection attempt on `esp_wifi_start()`

**Initialization Order Dependency:**
- ESP-NETIF and event loop must be initialized first (done internally)
- TELEMETRY must be initialized before WiFi events occur (components should init TELEMETRY early)

**Startup Sequence:**
1. `wifi_helper_init()` → registers event handlers, creates timers/task
2. `esp_wifi_start()` → triggers `WIFI_EVENT_STA_START`
3. State transitions: INIT → CONNECTING → CONNECTED
4. Post-connection: RSSI task initializes SNTP, reads initial signal strength

### Task Communication

**Event-Driven Architecture:**
- **WiFi event handler** (`WIFI_EVENT_STA_START`, `WIFI_EVENT_STA_DISCONNECTED`): Updates state, starts reconnect timers
- **IP event handler** (`IP_EVENT_STA_GOT_IP`): Notifies RSSI task via `xTaskNotifyGive()`
- **RSSI monitor task**: Waits for notification (post-connection init) or timeout (15min periodic check)

**Synchronization:**
- Single mutex (`xWiFiMutex`, 100ms timeout) protects `current_snapshot` structure
- State transitions inject to TELEMETRY atomically (mutex held during state update + injection trigger)

## Hardware Configuration

### WiFi Radio Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| Mode | STA (Station) | Client mode only |
| Frequency | 2.4GHz | 802.11 b/g/n |
| Power Save | WIFI_PS_MAX_MODEM | Default enabled (~20mA vs ~100mA) |
| Authentication | WPA2-PSK | Credentials from `main/wifi_credentials.h` |

### Power States

| State | Mode | Current Draw | Latency | Use Case |
|-------|------|--------------|---------|----------|
| Enabled | WIFI_PS_NONE | ~100mA | <10ms | High-priority operations (firmware updates) |
| Power-Save | WIFI_PS_MAX_MODEM | ~20mA | ~100ms | Default - maintains MQTT connectivity |
| Shutdown | Radio off | 0mA | 30s restart | CRITICAL battery (0% SOC) |

**Note:** Power-save mode introduces ~100ms latency for packet delivery but maintains full TCP/IP functionality. Suitable for MQTT publishing (QoS 1) where timing is not critical.

## Key Features

### 1. Exponential Backoff Reconnection

Prevents network congestion and AP overload by progressively increasing retry delays:

```c
static const uint32_t BACKOFF_DELAYS_MS[] = {
    1000,    // 1 second   (attempts 1-n)
    2000,    // 2 seconds
    5000,    // 5 seconds
    10000,   // 10 seconds
    30000,   // 30 seconds
    60000    // 60 seconds 
    600000,  // 10 minutes
    3600000, // 60 minutes (max, stays here until success or max retries)
};
```

**Behavior:**
- **On disconnect**: Increment `retry_count`, select backoff level, start timer
- **On success**: Reset `retry_count = 0`, `backoff_level = 0`
- **On max retries** (30): Enter FAILED state, stop reconnection attempts
- **Total time to FAILED**: ~10 minutes (1s + 2s + 5s + 10s + 30s + 60s + 600s + 3600s×13 ≈ 23h)

**Recovery from FAILED:**
```c
wifi_helper_force_reconnect();  // Reset counters, immediate connect
```

### 2. Three-Tier Power Management

Integrates with FLUCTUS load shedding for battery-aware operation:

**Power-Save Mode** (40% SOC):
```c
wifi_helper_set_power_save_mode(true);   // WIFI_PS_MAX_MODEM (~20mA)
```
- Modem sleeps between beacon intervals
- Slight latency increase (~100ms)
- MQTT publishing unaffected (QoS 1 handles delays)

**Shutdown Mode** (0% SOC):
```c
wifi_helper_set_shutdown(true);          // esp_wifi_stop() (0mA)
```
- Complete radio shutdown
- State transitions: CONNECTED → DISABLED
- Stops all reconnection attempts

**Restart with Anti-Bounce** (return to >15% SOC):
```c
wifi_helper_set_shutdown(false);         // Starts 30s delay timer
```
- Prevents rapid cycling during voltage bounces (e.g., IMPLUVIUM pump startup causing battery voltage sag)
- If shutdown requested again during 30s delay → abort restart
- After 30s → `esp_wifi_start()`, re-enable power-save mode

### 3. RSSI Monitoring & Telemetry Injection

Tracks signal strength with threshold-based updates to minimize telemetry traffic:

**Periodic Checks** (15min interval):
```c
wifi_ap_record_t ap_info;
esp_wifi_sta_get_ap_info(&ap_info);
update_rssi(ap_info.rssi);  // Compares against last_reported_rssi
```

**Threshold Injection** (>5dBm delta):
```c
if (abs(new_rssi - last_reported_rssi) >= 5) {
    last_reported_rssi = new_rssi;
    telemetry_fetch_snapshot(TELEMETRY_SRC_WIFI);  // Inject only on significant change
}
```

**Why 5dBm threshold:**
- RSSI naturally fluctuates ±2-3dBm due to interference/multipath
- 5dBm represents meaningful signal quality change
- Reduces telemetry cache writes by ~80% vs every reading

### 4. SNTP Time Synchronization

Initializes NTP client after first successful connection:

```c
// Post-connection init (triggered by xTaskNotifyGive from IP event handler)
if (!sntp_initialized) {
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    sntp_initialized = true;  // Idempotent on reconnections
}
```

**Why time sync matters:**
- **FLUCTUS solar tracking**: NOAA sunrise/sunset calculations require accurate UTC time
- **TELEMETRY timestamps**: Consistent message ordering for MQTT
- **Energy accumulation**: Hourly/daily rollovers depend on correct system clock

**Implementation notes:**
- Runs in RSSI monitor task context (4KB stack) to avoid event handler stack overflow
- One-time initialization per boot (survives reconnections)
- ESP-IDF SNTP operates independently after init (automatic updates every hour)

### 5. Disconnect Diagnostics

Tracks detailed failure reasons for debugging:

```c
typedef struct {
    uint32_t last_disconnect_reason;  // esp_wifi_reason_t enum
    uint16_t reconnect_count;         // Total reconnections this session
    uint32_t connected_time_sec;      // Cumulative connection time
} wifi_snapshot_t;
```

**Common disconnect reasons:**
- `WIFI_REASON_AUTH_FAIL` (15): Wrong password, AP rejected
- `WIFI_REASON_NO_AP_FOUND` (201): SSID not broadcasting or out of range
- `WIFI_REASON_BEACON_TIMEOUT` (200): Lost signal (moved too far from AP)
- `WIFI_REASON_ASSOC_LEAVE` (8): AP kicked us (congestion, max clients)

**Connection time tracking:**
```c
// Accumulates total connected time across disconnects/reconnects
connect_time = time(NULL);               // Set on IP_EVENT_STA_GOT_IP
update_connected_time();                 // Called on disconnect
total_connected_sec += (now - connect_time);
```

## Public API

### Initialization

```c
esp_err_t wifi_helper_init(const char *ssid, const char *password);
// - Registers event handlers, creates timers/task
// - Starts WiFi in STA mode, enables power-save by default
// - SSID/password max lengths: 32/64 chars (802.11 spec)
```

### State Query

```c
esp_err_t wifi_helper_get_snapshot(wifi_snapshot_t *snapshot);
// - Thread-safe memcpy of current WiFi state
// - No side effects (no telemetry injection, no state changes)
// - Used by HMI for status display, diagnostics
```

### Power Management

```c
esp_err_t wifi_helper_set_power_save_mode(bool enable);
// - Called by FLUCTUS at 40% SOC (POWER_SAVING state)
// - true: WIFI_PS_MAX_MODEM (~20mA), false: WIFI_PS_NONE (~100mA)
// - Injects to TELEMETRY on mode change

esp_err_t wifi_helper_set_shutdown(bool shutdown);
// - Called by FLUCTUS at 0% SOC (CRITICAL state)
// - true: Immediate stop, false: Delayed restart (30s anti-bounce)
// - Cancels pending restart timer on immediate shutdown
```

### Manual Control

```c
esp_err_t wifi_helper_force_reconnect(void);
// - Resets retry counters (retry_count = 0, backoff_level = 0)
// - Disconnects and reconnects immediately
// - Use case: HMI "Reconnect WiFi" button, recover from FAILED state
// - Returns ESP_FAIL if shutdown_requested flag set
```

### Telemetry Integration

```c
esp_err_t wifi_helper_write_to_telemetry_cache(wifi_snapshot_t *cache);
// - Called by TELEMETRY subsystem (not components)
// - Zero-copy write to TELEMETRY's cache buffer
// - Components use telemetry_fetch_snapshot(TELEMETRY_SRC_WIFI) to trigger injection
```

## Data Structures

### `wifi_snapshot_t`

Compact 24-byte structure for event-driven telemetry:

```c
typedef struct {
    wifi_state_t state;              // 4 bytes - Current state machine state
    int8_t rssi;                     // 1 byte  - Signal strength (-100 to 0 dBm)
                                     //           0 = not connected
    uint16_t reconnect_count;        // 2 bytes - Total reconnections this session
    uint32_t connected_time_sec;     // 4 bytes - Cumulative connection time (seconds)
    uint32_t last_disconnect_reason; // 4 bytes - Last esp_wifi_reason_t (0 = never disconnected)
    bool has_ip;                     // 1 byte  - IP address assigned flag
    bool power_save_mode;            // 1 byte  - WIFI_PS_MAX_MODEM enabled flag
    time_t snapshot_timestamp;       // 8 bytes - POSIX timestamp of last update
} wifi_snapshot_t;
```

**Field Update Frequencies:**
- `state`: On every state transition (events: connect/disconnect/shutdown)
- `rssi`: Every 15 minutes, or on >5dBm change
- `reconnect_count`: On each disconnect event
- `connected_time_sec`: Updated on disconnect and every 15min while connected
- `last_disconnect_reason`: On disconnect events only
- `has_ip`, `power_save_mode`: On state change
- `snapshot_timestamp`: Set by TELEMETRY on cache write

**Memory Optimization:**
- No buffering (single snapshot, not time-series)
- Event-driven updates only (not polled)
- Threshold-based RSSI injection reduces cache writes by 80%

## Integration

### Component Dependencies

**TELEMETRY:**
- WiFi injects on: State changes, RSSI delta >5dBm
- Cache source: `TELEMETRY_SRC_WIFI`
- Encoding: `msgpack_encode_wifi()` in `telemetry_msgpack.c`

**FLUCTUS (Load Shedding):**
- Calls `wifi_helper_set_power_save_mode(true)` at POWER_SAVING (40% SOC)
- Calls `wifi_helper_set_shutdown(true)` at CRITICAL (0% SOC)
- No direct dependency (WiFi does not query FLUCTUS state)

**HMI:**
- Fetches `telemetry_get_wifi_data()` for status display (RSSI bar, state icon)
- Calls `wifi_helper_force_reconnect()` on "Reconnect WiFi" button press
- Pattern: Lightweight state getter for button logic, snapshot fetch for display

**MQTT Client (within TELEMETRY):**
- Depends on WiFi connectivity for publishing
- Handles disconnects gracefully via internal buffering
- No direct WiFi API calls (uses ESP-IDF's `esp_mqtt_client`)

### External Libraries

- **esp_wifi**: ESP-IDF WiFi driver (STA mode, power save, event handling)
- **esp_netif**: Network interface abstraction (IP stack integration)
- **esp_event**: Event loop for WiFi/IP events
- **esp_sntp**: SNTP client for time synchronization

## Performance

### Memory Footprint

**SRAM:**
- `current_snapshot`: 24 bytes
- `wifi_ssid`, `wifi_password`: 98 bytes (33 + 65)
- State variables: ~20 bytes (retry counters, flags, handles)
- **Total**: ~150 bytes (excluding task stack)

**Task Stack:**
- `wifi_rssi`: 4096 bytes (handles SNTP init + RSSI operations)

**DRAM (uninitialized):**
- Event handlers: ~200 bytes (function pointers, event base registration)

### Timing Characteristics

**Operations:**
- State transition: <1ms (mutex lock + memcpy + inject trigger)
- RSSI read: 5-10ms (esp_wifi_sta_get_ap_info via SPI)
- Connection attempt: 2-5 seconds (DHCP negotiation)
- SNTP sync: 1-3 seconds (first sync), then automatic hourly updates

**Intervals:**
- RSSI monitoring: 15 minutes (configurable via `WIFI_HELPER_RSSI_CHECK_INTERVAL`)
- Exponential backoff: 1s → 60s (6 levels)
- Restart delay: 30 seconds (anti-bounce)

### Power Consumption

| Mode | Current Draw | Notes |
|------|--------------|-------|
| WIFI_PS_NONE | ~100mA | Max performance, always-on modem |
| WIFI_PS_MAX_MODEM | ~20mA | Default, modem sleeps between beacons |
| WIFI_PS_MIN_MODEM | ~50mA | Light sleep, lower latency than MAX |
| Shutdown | 0mA | Radio off, 30s restart delay |

**Daily Energy (Power-Save Mode):**
- Average: 20mA × 3.3V × 24h = 1.58Wh/day
- vs Normal: 100mA × 3.3V × 24h = 7.92Wh/day
- **Savings: 80%** (critical for solar-powered operation)

## Configuration

### Compile-Time Constants

```c
// wifi_helper.h
#define WIFI_HELPER_MAX_RETRIES           20    // Max reconnect attempts before FAILED state
#define WIFI_HELPER_RSSI_UPDATE_THRESHOLD 5     // dBm change to trigger telemetry injection
#define WIFI_HELPER_RSSI_CHECK_INTERVAL   900000 // 15 minutes (milliseconds)

// wifi_helper.c
#define WIFI_RESTART_DELAY_MS             30000  // Anti-bounce delay (30 seconds)
```

### Runtime Configuration

**Credentials** (static, set at init):
- SSID: Max 32 characters (802.11 spec)
- Password: Max 64 characters (WPA2-PSK spec)
- Source: `main/wifi_credentials.h` (not version-controlled)

**Power modes** (dynamic, controlled by FLUCTUS):
- `wifi_helper_set_power_save_mode(bool)`: Toggle WIFI_PS_MAX_MODEM
- `wifi_helper_set_shutdown(bool)`: Complete radio shutdown

**Backoff delays** (static array):
- Edit `BACKOFF_DELAYS_MS[]` in `wifi_helper.c` to customize retry strategy
- Current: 1s/2s/5s/10s/30s/60s/10m/60m (conservative, AP-friendly)

## Known Limitations

- **PSRAM interference**: WiFi enabled during PSRAM access can cause cache coherency issues on ESP32-S3. Workaround: SPIRAM_FETCH_INSTRUCTIONS and SPIRAM_RODATA disabled in sdkconfig.
- **Single AP only**: No roaming support. Disconnect/reconnect required to switch APs (same SSID).
- **SNTP server hardcoded**: `pool.ntp.org` only. No fallback servers configured.
- **Reconnect counter overflow**: `uint16_t reconnect_count` wraps at 65,535 (acceptable for diagnostic counter, not critical).
- **No AP mode**: STA (client) mode only. No support for hosting WiFi network.

---

**Related Files:**
- Integration: `main/main.c`
- Credentials: `main/wifi_credentials.h`
- Telemetry encoding: `components/core/telemetry/telemetry_msgpack.c`
- HMI display: `components/core/hmi/hmi_render_system.c` (WiFi status page)
