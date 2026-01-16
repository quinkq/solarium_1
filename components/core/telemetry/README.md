# TELEMETRY - Central Data Hub & MQTT Gateway

**Author:** Piotr P. <quinkq@gmail.com>
**Part of Solarium** - ESP32-S3 smart irrigation and solar power management system
**Lines of Code:** ~3149 (6 modules + 2 headers)

## Overview

TELEMETRY is the system's data coordination center, managing all component telemetry through a unified caching layer and two-tier MQTT buffering system. It provides zero-copy cache access for components to inject data, fast snapshot retrieval for HMI display, and resilient MQTT publishing with persistent backup.

**Core Capabilities:**
- Unified cache system with per-source mutexes (7 telemetry sources)
- Two-tier MQTT buffering: 512KB PSRAM ring (1,820 slots) + 1.1MB FLASH backup (4,096 slots)
- MessagePack binary serialization (80-250 bytes per message)
- QoS 1 MQTT publishing with automatic dequeue on PUBACK
- Remote configuration via MQTT commands (interval presets, realtime mode control)
- Power-aware publishing control (buffering-only mode at low SOC)

**Hardware:**
- PSRAM: 512KB allocated for volatile ring buffer (requires SPIRAM enabled in menuconfig)
- LittleFS: 2MB partition at `/data` for FLASH backup persistence
- No sensors - pure data orchestration component

## Architecture

### Module Structure

```
telemetry/
├── telemetry.c                  (398 lines)  - Init, LittleFS mount, control API
├── telemetry_cache.c            (211 lines)  - Zero-copy cache, snapshot retrieval
├── telemetry_mqtt_buffer.c      (421 lines)  - PSRAM ring + FLASH backup
├── telemetry_msgpack.c          (793 lines)  - 7 MessagePack encoders
├── telemetry_mqtt_client.c      (515 lines)  - MQTT lifecycle, publish task
├── telemetry_commands.c         (404 lines)  - MQTT command handlers
├── telemetry_private.h          (158 lines)  - Shared internal declarations
└── telemetry.h                  (249 lines)  - Public API, data structures
```

### State Machine

TELEMETRY is stateless - no lifecycle states. Components inject data asynchronously via lock/unlock API. MQTT client maintains connection state (CONNECTED/DISCONNECTED), but this doesn't affect cache operations.

### Threading Model

**MQTT Publish Task** (`mqtt_publish`, priority 5, 4KB stack):
- Dual responsibility: Data ingestion (when notified) + continuous publishing loop
- Notified by components via `xTaskNotify()` with source bit flags
- Encoding: Lock cache → Msgpack encode → Unlock cache → Enqueue to PSRAM
- Publishing: Peek buffer → Publish QoS 1 → Track in-flight → Wait PUBACK → Dequeue

**No dedicated tasks** - All operations triggered by:
1. Component calls to `telemetry_fetch_snapshot()` → notify MQTT task
2. MQTT task continuous publish loop (waits on buffer count)
3. MQTT event handler callbacks (PUBACK, commands, connection events)

### Data Flow

```
Components (variable cycles)
    ↓ telemetry_fetch_snapshot(src)
    ↓
Lock cache → Write via writer_function → Unlock cache
    ↓
Notify MQTT task (source bit flag)
    ↓
MQTT task wakes → Lock cache → Encode msgpack → Unlock → Enqueue PSRAM
    ↓
Publishing loop: Peek → Publish QoS1 → Track in-flight
    ↓
PUBACK event → Dequeue from PSRAM
    ↓
(95% full) → Batch flush to FLASH backup

HMI (1Hz/4Hz): telemetry_get_*_data() → Fast memcpy from cache (~1-2μs)
```

### Initialization & Startup

**Default States:**
- All caches: Zeroed (no data)
- Realtime mode: **ENABLED** (user preference stored separately)
- Publishing mode: **ENABLED** (normal publishing)
- MQTT: **DISCONNECTED** (awaiting WiFi)

**Startup Sequence:**
1. Mount LittleFS (`/data` partition)
2. Initialize PSRAM ring buffer (allocate 512KB via `MALLOC_CAP_SPIRAM`)
3. Load FLASH backup (if exists) → Restore to PSRAM → Delete file
4. Create new FLASH backup file for current session
5. Initialize MQTT client + Start publish task
6. Register component writer functions in stream table

**Dependency:** Requires WiFi connection before MQTT can publish, but cache operations work offline.

### Task Communication

**Component → MQTT Task:**
- Notification-based: `xTaskNotify(mqtt_task_handle, (1 << src), eSetBits)`
- Each source gets a bit flag (7 sources = 7 bits)
- Non-blocking: Components inject and return immediately

**MQTT Event Handler → Publish Task:**
- PUBACK dequeue: Direct buffer manipulation via `telemetry_buffer_dequeue()`
- Commands: Direct API calls to `telemetry_set_realtime_mode()`, interval config updates
- No queues: Event handler operates on shared state with mutex protection

**Priority Rationale:**
- Priority 5 (Med-5): MQTT publishing is important but tolerates latency (messages buffered)
- Same priority as component tasks (irrigation, weather, solar) for fair scheduling

## Hardware Configuration

TELEMETRY is a software-only component with no GPIO or sensor hardware. Storage hardware:

| Resource | Configuration | Capacity | Usage |
|----------|---------------|----------|-------|
| PSRAM | `MALLOC_CAP_SPIRAM` | 512 KB | Volatile ring buffer (1,820 slots × 288B) |
| LittleFS | Partition `littlefs` | 2 MB | `/data/mqtt_backup.dat` (1.1 MB max) |
| NVS Flash | ESP-IDF managed | - | WiFi PHY calibration (not used by TELEMETRY) |

**CRITICAL:** PSRAM must be enabled in `menuconfig` (`CONFIG_SPIRAM_SUPPORT=y`) or system will fail to allocate ring buffer.

## Key Features

### 1. Unified Cache System

**Seven Telemetry Sources:**

| Source | Cache Structure | Update Frequency | Writer Function |
|--------|----------------|------------------|-----------------|
| `FLUCTUS` | `fluctus_snapshot_t` (unified) | 15-60 min | `fluctus_write_to_telemetry_cache()` |
| `FLUCTUS_RT` | `fluctus_snapshot_t` (shared) | 500 ms | `fluctus_write_realtime_to_telemetry_cache()` |
| `TEMPESTA` | `tempesta_snapshot_t` | 5-60 min | `tempesta_write_to_telemetry_cache()` |
| `IMPLUVIUM` | `impluvium_snapshot_t` | Workflow completion | `impluvium_write_to_telemetry_cache()` |
| `IMPLUVIUM_RT` | `impluvium_snapshot_rt_t` | 500 ms (watering) | `impluvium_write_realtime_to_telemetry_cache()` |
| `STELLARIA` | `stellaria_snapshot_t` | Ramp start | `stellaria_write_to_telemetry_cache()` |
| `WIFI` | `wifi_snapshot_t` | State change | `wifi_helper_write_to_telemetry_cache()` |

**Per-Source Mutexes:**
```c
SemaphoreHandle_t xTelemetryMutexes[TELEMETRY_SRC_COUNT];  // Lazy init
```
- 100ms timeout standard
- Minimal contention: FLUCTUS normal (15min) vs FLUCTUS_RT (500ms) don't block each other
- Unified FLUCTUS structure uses same cache, different mutexes based on source enum

**Zero-Copy Write API:**
```c
void *cache = NULL;
telemetry_lock_cache(TELEMETRY_SRC_FLUCTUS, &cache);
// Component writes directly to cache pointer
fluctus_write_to_telemetry_cache(cache);
telemetry_unlock_cache(TELEMETRY_SRC_FLUCTUS);
```

**Fast Snapshot Retrieval (HMI):**
```c
fluctus_snapshot_t data;
telemetry_get_fluctus_data(&data);  // ~1-2μs memcpy
```

### 2. Two-Tier MQTT Buffering

**Tier 1: PSRAM Ring Buffer (Primary, Volatile)**
- **Capacity:** 1,820 slots × 288 bytes = 512 KB
- **Allocation:** `heap_caps_malloc(MALLOC_CAP_SPIRAM)`
- **Structure:** Circular buffer with head/tail pointers
- **Thread Safety:** `xBufferMutex` with 100ms timeout
- **Auto-Dequeue:** On MQTT PUBACK (QoS 1 acknowledgment)

**Slot Structure (`buffer_slot_t`, 288 bytes):**
```c
typedef struct {
    uint32_t global_seq;           // Monotonic sequence number
    uint32_t component_seq;        // Per-component sequence (unused)
    time_t timestamp_utc;          // Enqueue time
    uint8_t component_id;          // Telemetry source (0-6)
    uint8_t message_type;          // Message type identifier
    uint16_t payload_len;          // Actual payload bytes (≤256)
    bool occupied;                 // Slot in use flag
    int msg_id;                    // MQTT message ID (in-flight tracking)
    uint8_t retry_count;           // Publish retry counter
    uint8_t _padding[2];           // Alignment
    uint32_t last_publish_time;    // Retry timing
    uint8_t payload[256];          // MessagePack binary data
    uint32_t crc32;                // Checksum (currently unused)
} buffer_slot_t;
```

**Tier 2: FLASH Backup (Secondary, Persistent)**
- **Capacity:** 4,096 slots × 288 bytes = 1,179,648 bytes (~1.1 MB)
- **File:** `/data/mqtt_backup.dat`
- **Trigger:** Automatic flush at 95% PSRAM capacity (1,729 slots)
- **Manual Flush:** HMI "Save & Reset" calls `telemetry_manual_flush_to_flash()`
- **Boot Recovery:** Load all slots → Sort by `global_seq` → Restore to PSRAM → Delete file

**FLASH Backup File Structure:**
```
Header (12 bytes):
  - uint32_t magic (0xDEADBEEF)
  - uint16_t version (1)
  - uint16_t capacity (4096)
  - uint16_t head (circular write position)

Data (4096 slots × 288 bytes):
  - Circular buffer layout
  - Slots marked occupied=true are valid
  - Chronological order reconstructed via qsort(global_seq)
```

**Failure Recovery:**
- FLASH backup survives power loss (data preserved across reboots)
- PSRAM buffer lost on power cycle (volatile RAM)
- Boot sequence: PSRAM allocation → FLASH restore → Delete old file → Create new file
- Global sequence counter restored from `max(global_seq) + 1`

### 3. MessagePack Binary Serialization

**Seven Encoders:**

| Encoder | Fields | Typical Size | Source |
|---------|--------|--------------|--------|
| `fluctus` | 21 (15min avg, energy, state) | ~180 bytes | Normal mode data |
| `fluctus_rt` | 28 (instant power, buses, debug) | ~200 bytes | Realtime monitoring |
| `tempesta` | 25 (8 sensors + status) | ~220 bytes | Weather cycle |
| `impluvium_system` | 15 (state, learning, errors) | ~100 bytes | Irrigation system |
| `impluvium_zone` | 12 (moisture, config, timing) | ~100 bytes | Per-zone (×5) |
| `impluvium_rt` | 10 (sensors during watering) | ~140 bytes | Watering realtime |
| `stellaria` | 8 (lighting state) | ~60 bytes | Lighting events |
| `wifi` | 10 (connection, RSSI, IP) | ~80 bytes | WiFi state changes |

**Encoding Pattern:**
```c
msgpack_sbuffer sbuf;
msgpack_sbuffer_init(&sbuf);
msgpack_packer pk;
msgpack_packer_init(&pk, &sbuf, msgpack_sbuffer_write);

msgpack_pack_map(&pk, field_count);
// Pack key-value pairs...

if (sbuf.size <= max_len) {
    memcpy(output, sbuf.data, sbuf.size);
    *output_len = sbuf.size;
}
msgpack_sbuffer_destroy(&sbuf);
```

**IMPLUVIUM Split Encoding:**
- 1 system message (overall state, learning params)
- 5 zone messages (moisture, config, timing per zone)
- Total: 6 MQTT publishes per irrigation cycle (~600-700 bytes combined)

### 4. QoS 1 MQTT Publishing

**Publishing Loop (in `mqtt_publish` task):**
```
while (mqtt_connected) {
    buffer_slot_t slot;
    uint16_t tail_index;

    if (telemetry_buffer_peek(&slot, &tail_index) == ESP_OK) {
        // Check publishing flags
        if (message_type == control || telemetry_publishing_enabled) {
            int msg_id = esp_mqtt_client_publish(..., QoS=1);

            // Track in-flight for PUBACK matching
            in_flight_msg_id = msg_id;
            in_flight_seq = slot.global_seq;
            in_flight_tail_index = tail_index;

            // Wait for PUBACK with timeout
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));  // Buffering only
        }
    }
}
```

**PUBACK Handling (in event handler):**
```c
case MQTT_EVENT_PUBLISHED:
    if (event->msg_id == in_flight_msg_id) {
        telemetry_buffer_dequeue();  // Remove from buffer
        in_flight_msg_id = -1;       // Clear tracking
    }
```

**Retry Logic:**
- 5 second PUBACK timeout per message
- 3 retries per message (tracked via `slot.retry_count`)
- After 3 retries → Flush to FLASH backup
- On disconnect: Clear `in_flight_msg_id` (retry on reconnect)

### 5. Power-Aware Publishing Control

**Three Control Flags:**

| Flag | Purpose | Controlled By | Behavior |
|------|---------|---------------|----------|
| `realtime_mode_enabled` | Current RT state | FLUCTUS (SOC), User/Server | RT sources enqueued only if true |
| `realtime_mode_user_preference` | User's preference | User/MQTT command | Restored when SOC recovers |
| `telemetry_publishing_enabled` | Normal publishing | FLUCTUS (SOC) | False = buffering only (control msgs always publish) |

**Power State Integration:**

```c
// FLUCTUS calls based on battery SOC:
void fluctus_power_state_change(fluctus_power_state_t state) {
    switch (state) {
        case POWER_STATE_NORMAL:
        case POWER_STATE_POWER_SAVING:
            telemetry_force_realtime_monitoring_disable(false);  // Restore preference
            telemetry_enable_telemetry_publishing(true);         // Publish normally
            break;

        case POWER_STATE_LOW_POWER:  // 25% SOC
            telemetry_force_realtime_monitoring_disable(true);   // Force RT off
            telemetry_enable_telemetry_publishing(true);         // Still publish
            break;

        case POWER_STATE_VERY_LOW:   // 15% SOC
        case POWER_STATE_CRITICAL:   // 0% SOC
            telemetry_force_realtime_monitoring_disable(true);   // Force RT off
            telemetry_enable_telemetry_publishing(false);        // Buffer only
            break;
    }
}
```

**Buffering-Only Mode:**
- Normal messages (type 0-3): Enqueued but **not published**
- Control messages (type 4): **Always published** regardless of flag
- PSRAM fills to 95% → Automatic FLASH backup
- On SOC recovery: Resume publishing from buffer (FIFO order preserved)

### 6. Remote Configuration via MQTT

**Command Topics:**
- `solarium/config/intervals/set` - Set individual component intervals
- `solarium/config/intervals/preset` - Apply preset profile (Aggressive/Balanced/Conservative)
- `solarium/cmd/realtime` - Enable/disable realtime mode

**MessagePack Command Format (SET):**
```json
{
  "component": "FLUCTUS",
  "intervals": {
    "power_day": 15,
    "power_night": 60,
    "solar_correction": 15
  }
}
```

**Command Handling Flow:**
1. MQTT event handler receives DATA event
2. Parse MessagePack payload → Validate ranges
3. Apply to `interval_config` helper component
4. Notify component tasks (xTaskNotify or xTimerChangePeriod)
5. Persist to LittleFS (`/data/intervals.dat`)
6. Publish ACK to `solarium/config/intervals/ack` with current config

**Acknowledgment Format:**
```json
{
  "status": "ok",
  "fluctus": { "power_day": 15, "power_night": 60, "solar": 15 },
  "tempesta": { "normal": 15, "power_save": 60 },
  "impluvium": { "optimal": 15, "cool": 30, "power_save": 60, "night_min": 120 }
}
```

## Public API

### Initialization

```c
esp_err_t telemetry_init(void);
// Mounts LittleFS, allocates PSRAM buffer, loads FLASH backup,
// initializes MQTT client, starts publish task
```

### Cache Access (Components)

```c
// Zero-copy write (called by component writer functions)
esp_err_t telemetry_lock_cache(telemetry_source_t src, void **cache_ptr);
void telemetry_unlock_cache(telemetry_source_t src);

// Fetch snapshot (triggers encoding + publishing)
esp_err_t telemetry_fetch_snapshot(telemetry_source_t src);
// Internal flow: lock → writer_function → unlock → notify MQTT task
```

### Snapshot Retrieval (HMI)

```c
// Fast memcpy from cache (~1-2μs, 100ms mutex timeout)
esp_err_t telemetry_get_stellaria_data(stellaria_snapshot_t *data);
esp_err_t telemetry_get_fluctus_data(fluctus_snapshot_t *data);
esp_err_t telemetry_get_tempesta_data(tempesta_snapshot_t *data);
esp_err_t telemetry_get_impluvium_data(impluvium_snapshot_t *data);
esp_err_t telemetry_get_impluvium_realtime_data(impluvium_snapshot_rt_t *data);
esp_err_t telemetry_get_wifi_data(wifi_snapshot_t *data);
```

### Control & Status

```c
// Power-aware publishing control
void telemetry_set_realtime_mode(bool enabled);                    // User/server preference
void telemetry_force_realtime_monitoring_disable(bool force);      // FLUCTUS power override
void telemetry_enable_telemetry_publishing(bool mode);             // FLUCTUS SOC control

// State queries
bool telemetry_is_realtime_enabled(void);
bool telemetry_is_telemetry_publishing_enabled(void);

// Buffer management
esp_err_t telemetry_get_buffer_status(uint16_t *buffered_count, uint16_t *buffer_capacity);
esp_err_t telemetry_manual_flush_to_flash(uint16_t *flushed_count);  // HMI save & reset
```

### MQTT Command Handlers (Internal)

```c
// Called by MQTT event handler on DATA events
esp_err_t telemetry_handle_interval_command(const char *topic, const uint8_t *data, size_t data_len);
// Parses msgpack, validates, applies intervals, publishes ACK
```

## Data Structures

### Stream Configuration

```c
typedef esp_err_t (*telemetry_writer_fn_t)(void *cache);

typedef struct {
    void *cache;                          // Pointer to component snapshot structure
    telemetry_writer_fn_t writer_function;  // Component's cache writer function
} telemetry_stream_t;

// Stream table (populated during telemetry_init)
telemetry_stream_t streams[TELEMETRY_SRC_COUNT];
```

### Buffer Slot (288 bytes)

```c
typedef struct {
    uint32_t global_seq;       // Monotonic sequence (never resets)
    uint32_t component_seq;    // Per-component sequence (unused)
    time_t timestamp_utc;      // Enqueue timestamp
    uint8_t component_id;      // 0-6 (telemetry_source_t)
    uint8_t message_type;      // Message type identifier
    uint16_t payload_len;      // Actual bytes in payload (≤256)
    bool occupied;             // Slot validity flag
    int msg_id;                // MQTT msg_id for PUBACK matching
    uint8_t retry_count;       // Publish retry counter (max 3)
    uint8_t _padding[2];       // Struct alignment
    uint32_t last_publish_time;  // Retry timing (FreeRTOS ticks)
    uint8_t payload[256];      // MessagePack binary payload
    uint32_t crc32;            // Checksum (reserved, unused)
} buffer_slot_t;
```

**Memory Layout:**
- Metadata: 32 bytes (sequences, timestamps, counters, flags)
- Payload: 256 bytes (MessagePack binary)
- Total: 288 bytes per slot (aligned)

**Global Sequence:**
- Monotonic counter (never resets, wraps at UINT32_MAX)
- Restored from FLASH backup on boot: `max(global_seq) + 1`
- Used for chronological sorting during backup recovery

### Cache Memory (~900 bytes internal SRAM)

```c
// Static storage in telemetry.c
stellaria_snapshot_t cached_stellaria_data;        // ~60 bytes
fluctus_snapshot_t cached_fluctus_data;            // ~700 bytes (55 fields)
tempesta_snapshot_t cached_tempesta_data;          // ~200 bytes
impluvium_snapshot_t cached_impluvium_data;        // ~400 bytes
impluvium_snapshot_rt_t cached_impluvium_realtime_data;  // ~100 bytes
wifi_snapshot_t cached_wifi_data;                  // ~80 bytes
```

**Rationale:** Internal SRAM for cache-coherent, low-latency access. PSRAM buffer only for volatile MQTT queue.

## Integration

### Component Dependencies

**FLUCTUS (Power):**
- Injects: `TELEMETRY_SRC_FLUCTUS` (15-60min), `TELEMETRY_SRC_FLUCTUS_RT` (500ms)
- Controls: Power-aware publishing flags via `telemetry_force_realtime_monitoring_disable()`, `telemetry_enable_telemetry_publishing()`
- Writer: `fluctus_write_to_telemetry_cache()`, `fluctus_write_realtime_to_telemetry_cache()`

**TEMPESTA (Weather):**
- Injects: `TELEMETRY_SRC_TEMPESTA` (cycle completion)
- Writer: `tempesta_write_to_telemetry_cache()`

**IMPLUVIUM (Irrigation):**
- Injects: `TELEMETRY_SRC_IMPLUVIUM` (workflow complete), `TELEMETRY_SRC_IMPLUVIUM_RT` (500ms watering)
- Writer: `impluvium_write_to_telemetry_cache()`, `impluvium_write_realtime_to_telemetry_cache()`

**STELLARIA (Lighting):**
- Injects: `TELEMETRY_SRC_STELLARIA` (ramp start)
- Writer: `stellaria_write_to_telemetry_cache()`

**WIFI_HELPER:**
- Injects: `TELEMETRY_SRC_WIFI` (state change, RSSI delta >5dBm)
- Writer: `wifi_helper_write_to_telemetry_cache()`
- Provides: WiFi connection required for MQTT publishing

**HMI:**
- Retrieves: All `telemetry_get_*_data()` functions (1Hz static, 4Hz realtime)
- Controls: Manual FLASH flush, realtime mode toggle (via HMI controls)

### External Libraries

- **msgpack-c**: Binary serialization (components/third_party/msgpack/msgpack-c)
- **esp_mqtt**: ESP-IDF MQTT client (QoS 0/1/2 support)
- **esp_littlefs**: LittleFS VFS integration (2MB partition)
- **interval_config**: Centralized interval management (helper component)

## Performance

**Memory Footprint:**
- PSRAM buffer: 512 KB (1,820 slots × 288 bytes)
- Cache storage: ~900 bytes (internal SRAM)
- FLASH backup: 1.1 MB (LittleFS file)
- Task stack: 4 KB (MQTT publish task)
- **Total RAM:** ~517 KB (514 KB PSRAM + 5 KB internal)

**Timing Characteristics:**
- Cache lock/unlock: <50μs (mutex acquire + release)
- Snapshot retrieval: 1-2μs (memcpy ~200-700 bytes)
- MessagePack encode: 500μs-2ms (depends on encoder complexity)
- PSRAM enqueue: 200-400μs (memcpy 288 bytes + mutex)
- FLASH write: 10-30ms (LittleFS write + fsync)
- MQTT publish: 5-50ms (network latency, QoS 1)

**Buffer Capacity:**
- PSRAM: 1,820 messages before 95% threshold
- FLASH: 4,096 messages (total system capacity)
- Typical usage: 10-50 messages/hour (normal mode) or 100-200/hour (realtime mode)
- Days to fill PSRAM: ~2-7 days offline (depending on realtime mode)

**Power Consumption:**
- TELEMETRY software: <1mA (task switching + PSRAM access)
- MQTT WiFi: ~100mA active, ~20mA power-save (WIFI_PS_MAX_MODEM)
- LittleFS writes: ~30mA bursts (95% flush or manual flush)

## Configuration

### Compile-Time (`telemetry.h`)

```c
// PSRAM ring buffer
#define BUFFER_SLOT_SIZE 288
#define BUFFER_CAPACITY 1820
#define BUFFER_PAYLOAD_SIZE 256

// FLASH backup
#define FLASH_BACKUP_FILE "/data/mqtt_backup.dat"
#define FLASH_BACKUP_CAPACITY 4096
#define FLASH_BACKUP_THRESHOLD 1729  // 95% of PSRAM
#define FLASH_BATCH_SIZE 100         // Write batch size

// MQTT QoS 1
#define MQTT_PUBACK_TIMEOUT_SEC 5
#define MQTT_MAX_RETRIES 3
```

### Runtime (Kconfig, `menuconfig`)

```
CONFIG_TELEMETRY_MQTT_ENABLE=y                   # Enable MQTT client
CONFIG_TELEMETRY_MQTT_BROKER_URI="mqtt://..."   # Broker address
CONFIG_TELEMETRY_MQTT_USERNAME="solarium"
CONFIG_TELEMETRY_MQTT_PASSWORD="..."
CONFIG_TELEMETRY_MQTT_CLIENT_ID="solarium_esp32"
```

### MQTT Topics

```
Publish:
  - solarium/power/fluctus         (FLUCTUS normal)
  - solarium/power/fluctus_rt      (FLUCTUS realtime)
  - solarium/weather/tempesta      (TEMPESTA)
  - solarium/irrigation/system     (IMPLUVIUM system)
  - solarium/irrigation/zone_0-4   (IMPLUVIUM zones)
  - solarium/irrigation/realtime   (IMPLUVIUM RT)
  - solarium/lighting/stellaria    (STELLARIA)
  - solarium/status/wifi           (WiFi helper)
  - solarium/status                (LWT: online/offline)
  - solarium/config/intervals/ack  (Command acknowledgments)

Subscribe:
  - solarium/cmd/realtime                 (true/false)
  - solarium/config/intervals/set         (MessagePack)
  - solarium/config/intervals/preset      (MessagePack)
```

## Design Patterns

**Per-Source Mutex Isolation**: Each telemetry source owns its cache mutex. This prevents high-frequency sources (FLUCTUS_RT 500ms, IMPLUVIUM_RT 500ms) from blocking low-frequency sources (FLUCTUS normal 15-60min). HMI reads at 1-4Hz proceed independently, achieving true concurrent operation without priority inversion.

**Zero-Copy Cache Access**: Components receive direct pointers to cache structures via `telemetry_lock_cache()`, eliminating memcpy overhead during writes. Only HMI reads perform memcpy (1-2μs), optimizing for the write-heavy data flow pattern (7 sources writing vs 1 HMI reading).

**Two-Tier "In-Flight Database"**: PSRAM ring buffer provides fast enqueue/dequeue for active publishing, while FLASH backup acts as persistent overflow storage. Automatic 95% threshold flush prevents PSRAM overflow, manual flush before reset preserves unsent messages. Boot recovery with qsort chronological ordering ensures FIFO integrity across power cycles.

**Event-Driven Publishing**: Components call `telemetry_fetch_snapshot()` which notifies the MQTT task via task notification bits (7 sources = 7 bits). The MQTT task wakes, encodes MessagePack, and enqueues to buffer. This decouples component timing from MQTT publishing, allowing offline operation and resilient buffering.

**Power-Aware Flag Layering**: Three orthogonal flags control publishing behavior: `realtime_mode_user_preference` (user's choice), `realtime_mode_enabled` (active state after power override), `telemetry_publishing_enabled` (normal mode buffer/publish). FLUCTUS power state controls these flags without modifying component code, and user preference restoration after SOC recovery happens automatically.

**QoS 1 with In-Flight Tracking**: Single in-flight message tracking (`in_flight_msg_id`, `in_flight_seq`, `in_flight_tail_index`) ensures correct PUBACK matching and buffer dequeue. Peek-before-dequeue pattern prevents message loss during publish failures, and retry counter in each slot enables hybrid retry logic (per-message lifetime + session failure detection).

**Unified FLUCTUS Cache**: FLUCTUS normal and realtime sources share the same cache structure (`fluctus_snapshot_t`) but use different source enums and mutexes. This prevents duplicate storage (55 fields × 2 = unnecessary 1.4KB overhead) while allowing concurrent writes via separate locks.

---

**Related Files:**
- Integration: `main/main.c`
- Component writers: `components/core/*/telemetry_msgpack.c` (injected via function pointers)
- HMI rendering: `components/core/hmi/hmi_render_*.c` (uses `telemetry_get_*_data()`)
- Interval config: `components/helpers/interval_config/interval_config.c`
