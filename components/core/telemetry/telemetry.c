/**
 * @file telemetry.c
 * @brief Central data hub with zero-copy caching and two-tier MQTT buffering
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Original implementation of TELEMETRY - the system's data coordination center.
 *
 * Key features:
 * - Zero-copy data caching with push-based injection from 7 sources
 * - Per-cache mutexes for minimal contention
 * - Two-tier "in-flight database" (PSRAM ring buffer + flash backup)
 * - 1,820 slots in PSRAM (512KB), 4,096 slots in flash (1.1MB)
 * - Automatic flush at 95% capacity, recovery on boot
 * - QoS 1 MQTT integration with auto-dequeue on PUBACK
 * - Msgpack binary serialization for efficient transmission
 * - NOAA solar calculations with sunrise/sunset callbacks
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "telemetry.h"
#include "solar_calc.h"
#include "esp_log.h"
#include "esp_littlefs.h"
#include "esp_timer.h"
#include "mqtt_client.h"
#include "msgpack.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>
#include <unistd.h>

// ########################## Constants and Variables ##########################

static const char *TAG = "TELEMETRY";

// MQTT QoS 1 retry configuration
#define MQTT_PUBACK_TIMEOUT_SEC 5
#define MQTT_MAX_RETRIES 3

// FLASH backup configuration
#define FLASH_BACKUP_FILE "/data/mqtt_backup.dat"
#define FLASH_BACKUP_CAPACITY 4096          // ~1 MB / 288 bytes per slot (256 payload + 32 metadata)
#define FLASH_BACKUP_THRESHOLD 1729         // 95% of PSRAM (1820 * 0.95)
#define FLASH_BACKUP_MAGIC 0xDEADBEEF
#define FLASH_BATCH_SIZE 100                // Write 100 slots at a time

// Mutex for thread safety
static SemaphoreHandle_t xTelemetryMutex = NULL;

// Initialization flags
static bool telemetry_initialized = false;
static bool littlefs_mounted = false;

// ################ Component Data Hub (Central Storage) ################

// Static storage for component data caches
static stellaria_snapshot_t cached_stellaria_data = {0};
static fluctus_snapshot_t cached_fluctus_data = {0};  // Unified: shared by both FLUCTUS and FLUCTUS_RT
static tempesta_snapshot_t cached_tempesta_data = {0};
static impluvium_snapshot_t cached_impluvium_data = {0};
static impluvium_snapshot_rt_t cached_impluvium_realtime_data = {0};
static wifi_snapshot_t cached_wifi_data = {0};

// Unified cache infrastructure
static SemaphoreHandle_t xTelemetryMutexes[TELEMETRY_SRC_COUNT] = {NULL};
static TaskHandle_t mqtt_task_handle = NULL;

// MQTT infrastructure
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;

// In-flight message tracking (for QoS 1 PUBACK handling and hybrid retry logic)
static volatile int in_flight_msg_id = -1;         // MQTT msg_id awaiting PUBACK
static volatile uint32_t in_flight_seq = 0;        // Sequence number (for logging)
static volatile uint16_t in_flight_tail_index = 0; // Buffer tail index (for retry_count tracking)

// Realtime mode and publishing control flags
static bool realtime_mode_enabled = false;          // Current active state
static bool realtime_mode_user_preference = false;  // Last user/server preference
static bool telemetry_publishing_enabled = true;    // Normal mode publishing control

// PSRAM ring buffer (512 KB, 1,820 slots of 288 bytes each)
#define BUFFER_SLOT_SIZE 288
#define BUFFER_CAPACITY 1820
#define BUFFER_PAYLOAD_SIZE 256

typedef struct {
    uint32_t global_seq;
    uint32_t component_seq;
    time_t timestamp_utc;
    uint8_t component_id;
    uint8_t message_type;
    uint16_t payload_len;
    bool occupied;
    int msg_id;
    uint8_t retry_count;
    uint8_t _padding[2];
    uint32_t last_publish_time;  // Seconds since boot for timeout tracking
    uint8_t payload[BUFFER_PAYLOAD_SIZE];
    uint32_t crc32;
} buffer_slot_t;

static buffer_slot_t *psram_buffer = NULL;
static uint16_t buffer_head = 0;
static uint16_t buffer_tail = 0;
static uint16_t buffer_count = 0;
static SemaphoreHandle_t xBufferMutex = NULL;
static uint32_t global_sequence = 0;

// FLASH backup state
static FILE *flash_backup_file = NULL;
static uint16_t flash_head = 0;
static SemaphoreHandle_t xFlashMutex = NULL;

// Component writer function type
typedef esp_err_t (*telemetry_writer_fn_t)(void *cache);

// Telemetry stream structure
typedef struct {
    void *cache;
    telemetry_writer_fn_t writer_function;
} telemetry_stream_t;

// Stream mapping table (will be populated in telemetry_init)
static telemetry_stream_t streams[TELEMETRY_SRC_COUNT];

// ########################## Private Function Declarations ##########################

static esp_err_t telemetry_mount_littlefs(void);
static esp_err_t flash_backup_init(void);
static esp_err_t flash_backup_write_slot(const buffer_slot_t *slot);
static esp_err_t flash_backup_load_on_boot(void);

static esp_err_t buffer_init(void);

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data);
static void mqtt_publish_task(void *parameters);

// ########################## Private Functions ##########################

/**
 * @brief Mount LittleFS filesystem
 */
static esp_err_t telemetry_mount_littlefs(void)
{
    ESP_LOGI(TAG, "Mounting LittleFS...");

    esp_vfs_littlefs_conf_t conf = {
        .base_path = TELEMETRY_LITTLEFS_MOUNT_POINT,
        .partition_label = "littlefs",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    esp_err_t ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find LittleFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize LittleFS: %s", esp_err_to_name(ret));
        }
        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info("littlefs", &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "LittleFS: %d KB total, %d KB used", total / 1024, used / 1024);
    }

    littlefs_mounted = true;
    return ESP_OK;
}

// ########################## Public API Functions ##########################

esp_err_t telemetry_init(void)
{
    ESP_LOGI(TAG, "Initializing telemetry subsystem...");

    if (telemetry_initialized) {
        ESP_LOGW(TAG, "Telemetry already initialized");
        return ESP_OK;
    }

    xTelemetryMutex = xSemaphoreCreateMutex();
    if (xTelemetryMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create telemetry mutex");
        return ESP_FAIL;
    }

    esp_err_t ret = telemetry_mount_littlefs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount LittleFS");
        return ret;
    }

    ret = buffer_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PSRAM ring buffer");
        return ret;
    }

    // 1. Load any existing backup from previous boot
    //    This function opens, reads, loads to PSRAM, and DELETES the backup file.
    ret = flash_backup_load_on_boot();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Loaded buffered messages from FLASH backup");
    }

    // 2. Initialize FLASH backup system for THIS session
    //    Since the old file is gone, this creates a new, empty one.
    //    The 'flash_backup_file' handle will now be valid for the entire runtime.
    ret = flash_backup_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "FLASH backup init failed (continuing without backup)");
    }

    streams[TELEMETRY_SRC_FLUCTUS] = (telemetry_stream_t){
        .cache = &cached_fluctus_data,
        .writer_function = (telemetry_writer_fn_t)fluctus_write_to_telemetry_cache
    };
    streams[TELEMETRY_SRC_FLUCTUS_RT] = (telemetry_stream_t){
        .cache = &cached_fluctus_data,  // Unified: same cache, different write function
        .writer_function = (telemetry_writer_fn_t)fluctus_write_realtime_to_telemetry_cache
    };
    streams[TELEMETRY_SRC_TEMPESTA] = (telemetry_stream_t){
        .cache = &cached_tempesta_data,
        .writer_function = (telemetry_writer_fn_t)tempesta_write_to_telemetry_cache
    };
    streams[TELEMETRY_SRC_IMPLUVIUM] = (telemetry_stream_t){
        .cache = &cached_impluvium_data,
        .writer_function = (telemetry_writer_fn_t)impluvium_write_to_telemetry_cache
    };
    streams[TELEMETRY_SRC_IMPLUVIUM_RT] = (telemetry_stream_t){
        .cache = &cached_impluvium_realtime_data,
        .writer_function = (telemetry_writer_fn_t)impluvium_write_realtime_to_telemetry_cache
    };
    streams[TELEMETRY_SRC_STELLARIA] = (telemetry_stream_t){
        .cache = &cached_stellaria_data,
        .writer_function = (telemetry_writer_fn_t)stellaria_write_to_telemetry_cache
    };
    streams[TELEMETRY_SRC_WIFI] = (telemetry_stream_t){
        .cache = &cached_wifi_data,
        .writer_function = (telemetry_writer_fn_t)wifi_helper_write_to_telemetry_cache
    };

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = TELEMETRY_MQTT_BROKER_URI,
        .credentials.username = TELEMETRY_MQTT_USERNAME,
        .credentials.authentication.password = TELEMETRY_MQTT_PASSWORD,
        .credentials.client_id = TELEMETRY_MQTT_CLIENT_ID,
        .session.last_will.topic = "solarium/status",
        .session.last_will.msg = "offline",
        .session.last_will.msg_len = 7,
        .session.last_will.qos = 1,
        .session.last_will.retain = 1
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }

    ret = esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MQTT event handler");
        esp_mqtt_client_destroy(mqtt_client);
        return ret;
    }

    ret = esp_mqtt_client_start(mqtt_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client");
        esp_mqtt_client_destroy(mqtt_client);
        return ret;
    }

    BaseType_t task_ret = xTaskCreate(mqtt_publish_task, "mqtt_publish", 4096, NULL, 5, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create MQTT publishing task");
        esp_mqtt_client_stop(mqtt_client);
        esp_mqtt_client_destroy(mqtt_client);
        return ESP_FAIL;
    }

    telemetry_initialized = true;
    ESP_LOGI(TAG, "Telemetry initialization complete (MQTT enabled)");
    return ESP_OK;
}

// ################ Unified Cache Implementation ################

static inline SemaphoreHandle_t telemetry_get_mutex(telemetry_source_t src)
{
    if (src >= TELEMETRY_SRC_COUNT) {
        return NULL;
    }
    if (!xTelemetryMutexes[src]) {
        xTelemetryMutexes[src] = xSemaphoreCreateMutex();
    }
    return xTelemetryMutexes[src];
}

esp_err_t telemetry_lock_cache(telemetry_source_t src, void **cache_ptr)
{
    if (src >= TELEMETRY_SRC_COUNT || cache_ptr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    SemaphoreHandle_t m = telemetry_get_mutex(src);
    if (!m || xSemaphoreTake(m, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Cache lock timeout for source %d", src);
        return ESP_FAIL;
    }

    *cache_ptr = streams[src].cache;
    return ESP_OK;
}

void telemetry_unlock_cache(telemetry_source_t src)
{
    if (src >= TELEMETRY_SRC_COUNT) {
        return;
    }

    // Timestamp update removed: Writer functions are now responsible for setting
    // snapshot_timestamp ONLY on successful cache population. This prevents
    // stale data from receiving fresh timestamps when writer functions fail.

    SemaphoreHandle_t m = telemetry_get_mutex(src);
    if (m) {
        xSemaphoreGive(m);
    }
}

esp_err_t telemetry_fetch_snapshot(telemetry_source_t src)
{
    if (!telemetry_initialized || src >= TELEMETRY_SRC_COUNT) {
        return ESP_ERR_INVALID_STATE;
    }

    void *cache = NULL;
    esp_err_t ret = telemetry_lock_cache(src, &cache);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to lock cache for source %d", src);
        return ret;
    }

    ret = streams[src].writer_function(cache);
    telemetry_unlock_cache(src);

    if (ret == ESP_OK && mqtt_task_handle) {
        xTaskNotify(mqtt_task_handle, (1 << src), eSetBits);
    }

    return ret;
}

// ################ PSRAM Ring Buffer ################

static esp_err_t buffer_init(void)
{
    psram_buffer = (buffer_slot_t *)heap_caps_malloc(
        BUFFER_CAPACITY * sizeof(buffer_slot_t),
        MALLOC_CAP_SPIRAM
    );

    if (psram_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate PSRAM ring buffer (%zu bytes)",
                 BUFFER_CAPACITY * sizeof(buffer_slot_t));
        return ESP_ERR_NO_MEM;
    }

    memset(psram_buffer, 0, BUFFER_CAPACITY * sizeof(buffer_slot_t));

    xBufferMutex = xSemaphoreCreateMutex();
    if (xBufferMutex == NULL) {
        heap_caps_free(psram_buffer);
        psram_buffer = NULL;
        ESP_LOGE(TAG, "Failed to create buffer mutex");
        return ESP_FAIL;
    }

    buffer_head = 0;
    buffer_tail = 0;
    buffer_count = 0;
    global_sequence = 0;

    ESP_LOGI(TAG, "PSRAM ring buffer initialized: %d slots, %zu KB total",
             BUFFER_CAPACITY, (BUFFER_CAPACITY * sizeof(buffer_slot_t)) / 1024);

    return ESP_OK;
}

static esp_err_t buffer_enqueue(const uint8_t *payload, uint16_t payload_len,
                                telemetry_source_t src, uint8_t message_type)
{
    if (!psram_buffer || !payload || payload_len > BUFFER_PAYLOAD_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xBufferMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    if (buffer_count >= BUFFER_CAPACITY) {
        xSemaphoreGive(xBufferMutex);
        ESP_LOGW(TAG, "Ring buffer full (%d slots), dropping new message", BUFFER_CAPACITY);
        return ESP_ERR_NO_MEM;
    }

    buffer_slot_t *slot = &psram_buffer[buffer_head];
    slot->global_seq = global_sequence++;
    slot->component_seq = 0;
    slot->timestamp_utc = time(NULL);
    slot->component_id = (uint8_t)src;
    slot->message_type = message_type;
    slot->payload_len = payload_len;
    slot->occupied = true;
    slot->msg_id = -1;
    slot->retry_count = 0;
    slot->last_publish_time = 0;
    memcpy(slot->payload, payload, payload_len);
    slot->crc32 = 0;

    buffer_head = (buffer_head + 1) % BUFFER_CAPACITY;
    buffer_count++;

    xSemaphoreGive(xBufferMutex);

    ESP_LOGD(TAG, "Enqueued message: seq=%lu, src=%d, type=%d, len=%d, count=%d",
             slot->global_seq, src, message_type, payload_len, buffer_count);

    return ESP_OK;
}

static esp_err_t buffer_peek(buffer_slot_t *out_slot, uint16_t *out_tail_index)
{
    if (!psram_buffer || !out_slot) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xBufferMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    if (buffer_count == 0) {
        xSemaphoreGive(xBufferMutex);
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(out_slot, &psram_buffer[buffer_tail], sizeof(buffer_slot_t));

    // Optionally return tail index for hybrid retry logic
    if (out_tail_index != NULL) {
        *out_tail_index = buffer_tail;
    }

    xSemaphoreGive(xBufferMutex);

    return ESP_OK;
}

static esp_err_t buffer_dequeue(void)
{
    if (!psram_buffer) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xBufferMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    if (buffer_count == 0) {
        xSemaphoreGive(xBufferMutex);
        return ESP_ERR_NOT_FOUND;
    }

    psram_buffer[buffer_tail].occupied = false;
    buffer_tail = (buffer_tail + 1) % BUFFER_CAPACITY;
    buffer_count--;

    xSemaphoreGive(xBufferMutex);

    ESP_LOGD(TAG, "Dequeued message, remaining count=%d", buffer_count);

    return ESP_OK;
}

static uint16_t buffer_get_count(void)
{
    if (xSemaphoreTake(xBufferMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return 0;
    }

    uint16_t count = buffer_count;
    xSemaphoreGive(xBufferMutex);

    return count;
}

// ################ FLASH Backup System ################

/**
 * @brief Initialize FLASH backup system for current session
 *
 * This function is called AFTER flash_backup_load_on_boot() has completed.
 * The old backup file (if any) has been deleted by the load function.
 * This function creates a fresh, empty backup file for this session.
 *
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t flash_backup_init(void)
{
    xFlashMutex = xSemaphoreCreateMutex();
    if (xFlashMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create FLASH mutex");
        return ESP_FAIL;
    }

    // Create new backup file (old file deleted by load function)
    ESP_LOGI(TAG, "Creating new FLASH backup file for current session");
    flash_backup_file = fopen(FLASH_BACKUP_FILE, "w+b");

    if (flash_backup_file == NULL) {
        ESP_LOGE(TAG, "Failed to create FLASH backup file");
        return ESP_FAIL;
    }

    // Write fresh header (flash_head = 0 for new file)
    uint32_t magic = FLASH_BACKUP_MAGIC;
    uint16_t version = 1;
    uint16_t capacity = FLASH_BACKUP_CAPACITY;
    flash_head = 0;  // Reset for new session

    fwrite(&magic, sizeof(magic), 1, flash_backup_file);
    fwrite(&version, sizeof(version), 1, flash_backup_file);
    fwrite(&capacity, sizeof(capacity), 1, flash_backup_file);
    fwrite(&flash_head, sizeof(flash_head), 1, flash_backup_file);
    fflush(flash_backup_file);

    ESP_LOGI(TAG, "FLASH backup file created successfully");

    return ESP_OK;
}

/**
 * @brief Write single slot to FLASH backup (circular buffer)
 */
static esp_err_t flash_backup_write_slot(const buffer_slot_t *slot)
{
    if (!flash_backup_file || !slot) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Calculate file position: header (12 bytes) + (slot_index * slot_size)
    long position = 12 + (flash_head * BUFFER_SLOT_SIZE);
    fseek(flash_backup_file, position, SEEK_SET);

    // Write slot data
    size_t written = fwrite(slot, sizeof(buffer_slot_t), 1, flash_backup_file);

    if (written != 1) {
        ESP_LOGE(TAG, "Failed to write FLASH slot at index %d", flash_head);
        xSemaphoreGive(xFlashMutex);
        return ESP_FAIL;
    }

    // Update head pointer (circular) - ONLY in RAM, not in file header
    // The file header is NOT updated here to prevent extreme flash wear.
    // On boot, flash_backup_load_on_boot() reconstructs order using global_seq.
    uint16_t written_index = flash_head;
    flash_head = (flash_head + 1) % FLASH_BACKUP_CAPACITY;

    xSemaphoreGive(xFlashMutex);

    ESP_LOGD(TAG, "Wrote slot seq=%lu to FLASH at index %d", slot->global_seq, written_index);

    return ESP_OK;
}

/**
 * @brief Helper structure for sorting backup entries by sequence number
 */
typedef struct {
    uint32_t global_seq;
    uint16_t file_index;
} backup_entry_t;

/**
 * @brief qsort comparison function for backup entries
 */
static int compare_backup_entries(const void *a, const void *b)
{
    backup_entry_t *entryA = (backup_entry_t *)a;
    backup_entry_t *entryB = (backup_entry_t *)b;
    if (entryA->global_seq < entryB->global_seq) return -1;
    if (entryA->global_seq > entryB->global_seq) return 1;
    return 0;
}

/**
 * @brief Load FLASH backup into PSRAM on boot
 *
 * This function is completely self-contained:
 * - Opens its own file handle
 * - Reads and sorts all occupied slots by global_seq (handles wraparound)
 * - Loads them into PSRAM in chronological order
 * - Restores the global sequence counter
 * - Deletes the backup file
 * - Closes its file handle
 *
 * @return ESP_OK if data loaded, ESP_ERR_NOT_FOUND if no backup, ESP_FAIL on error
 */
static esp_err_t flash_backup_load_on_boot(void)
{
    FILE *f = fopen(FLASH_BACKUP_FILE, "rb");
    if (f == NULL) {
        ESP_LOGI(TAG, "No FLASH backup found (first boot or no prior backup)");
        return ESP_ERR_NOT_FOUND;
    }

    // Read and verify header
    uint32_t magic;
    uint16_t version, capacity, head;

    fread(&magic, sizeof(magic), 1, f);
    fread(&version, sizeof(version), 1, f);
    fread(&capacity, sizeof(capacity), 1, f);
    fread(&head, sizeof(head), 1, f);

    if (magic != FLASH_BACKUP_MAGIC) {
        ESP_LOGE(TAG, "Invalid FLASH backup magic: 0x%08lx", magic);
        fclose(f);
        unlink(FLASH_BACKUP_FILE);  // Delete corrupt file
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Loading FLASH backup: capacity=%d, head=%d", capacity, head);

    // Allocate array to store sequence/index pairs for sorting
    backup_entry_t *entries = malloc(capacity * sizeof(backup_entry_t));
    if (entries == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for backup entry sorting");
        fclose(f);
        return ESP_FAIL;
    }

    // Scan all slots and build sorted index
    uint16_t occupied_count = 0;
    uint32_t max_seq = 0;
    buffer_slot_t temp_slot;

    for (uint16_t i = 0; i < capacity; i++) {
        long position = 12 + (i * BUFFER_SLOT_SIZE);
        fseek(f, position, SEEK_SET);

        if (fread(&temp_slot, sizeof(buffer_slot_t), 1, f) != 1) {
            break;
        }

        if (temp_slot.occupied) {
            entries[occupied_count].global_seq = temp_slot.global_seq;
            entries[occupied_count].file_index = i;
            occupied_count++;

            if (temp_slot.global_seq > max_seq) {
                max_seq = temp_slot.global_seq;
            }
        }
    }

    if (occupied_count == 0) {
        ESP_LOGI(TAG, "FLASH backup is empty");
        free(entries);
        fclose(f);
        unlink(FLASH_BACKUP_FILE);
        return ESP_ERR_NOT_FOUND;
    }

    // Sort entries by sequence number (chronological order)
    qsort(entries, occupied_count, sizeof(backup_entry_t), compare_backup_entries);

    ESP_LOGI(TAG, "Found %d occupied slots, loading in chronological order...", occupied_count);

    // Load into PSRAM in correct order
    uint16_t loaded = 0;
    for (uint16_t i = 0; i < occupied_count; i++) {
        uint16_t file_index = entries[i].file_index;

        // Read the full slot data from backup
        long position = 12 + (file_index * BUFFER_SLOT_SIZE);
        fseek(f, position, SEEK_SET);
        fread(&temp_slot, sizeof(buffer_slot_t), 1, f);

        // Enqueue into PSRAM
        if (xSemaphoreTake(xBufferMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (buffer_count < BUFFER_CAPACITY) {
                memcpy(&psram_buffer[buffer_head], &temp_slot, sizeof(buffer_slot_t));
                buffer_head = (buffer_head + 1) % BUFFER_CAPACITY;
                buffer_count++;
                loaded++;
            }
            xSemaphoreGive(xBufferMutex);
        }
    }

    free(entries);

    // CRITICAL: Restore the global sequence counter
    global_sequence = max_seq + 1;
    ESP_LOGI(TAG, "Restored global sequence to %lu", global_sequence);

    ESP_LOGI(TAG, "Loaded %d messages from FLASH backup into PSRAM", loaded);

    // Clean up: close file and delete it
    fclose(f);

    if (unlink(FLASH_BACKUP_FILE) == 0) {
        ESP_LOGI(TAG, "FLASH backup file deleted after successful load");
    } else {
        ESP_LOGW(TAG, "Failed to delete FLASH backup file");
    }

    return (loaded > 0) ? ESP_OK : ESP_FAIL;
}

// ################ MessagePack Encoders ################

static esp_err_t msgpack_encode_fluctus(const fluctus_snapshot_t *data,
                                        uint8_t *output, uint16_t *output_len,
                                        uint16_t max_len)
{
    if (!data || !output || !output_len) {
        return ESP_ERR_INVALID_ARG;
    }

    msgpack_sbuffer sbuf;
    msgpack_sbuffer_init(&sbuf);

    msgpack_packer pk;
    msgpack_packer_init(&pk, &sbuf, msgpack_sbuffer_write);

    // Normal snapshot: 15-min averages + energy statistics + system state
    msgpack_pack_map(&pk, 21);  // Increased field count

    // Timestamp
    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "ts", 2);
    msgpack_pack_uint64(&pk, data->snapshot_timestamp);

    // Battery - 15-minute averages
    msgpack_pack_str(&pk, 7); msgpack_pack_str_body(&pk, "bat_v15", 7);
    msgpack_pack_float(&pk, data->battery_voltage_avg_15min);

    msgpack_pack_str(&pk, 7); msgpack_pack_str_body(&pk, "bat_a15", 7);
    msgpack_pack_float(&pk, data->battery_current_avg_15min);

    msgpack_pack_str(&pk, 7); msgpack_pack_str_body(&pk, "bat_w15", 7);
    msgpack_pack_float(&pk, data->battery_power_avg_15min);

    msgpack_pack_str(&pk, 7); msgpack_pack_str_body(&pk, "bat_s15", 7);
    msgpack_pack_float(&pk, data->battery_soc_avg_15min);

    // Solar - 15-minute averages
    msgpack_pack_str(&pk, 6); msgpack_pack_str_body(&pk, "pv_v15", 6);
    msgpack_pack_float(&pk, data->solar_voltage_avg_15min);

    msgpack_pack_str(&pk, 6); msgpack_pack_str_body(&pk, "pv_a15", 6);
    msgpack_pack_float(&pk, data->solar_current_avg_15min);

    msgpack_pack_str(&pk, 6); msgpack_pack_str_body(&pk, "pv_w15", 6);
    msgpack_pack_float(&pk, data->solar_power_avg_15min);

    msgpack_pack_str(&pk, 6); msgpack_pack_str_body(&pk, "pv_act", 6);
    msgpack_pack_bin(&pk, data->solar_pv_active);

    // Solar tracking state (minimal)
    msgpack_pack_str(&pk, 5); msgpack_pack_str_body(&pk, "trk_s", 5);
    msgpack_pack_uint8(&pk, data->tracking_state);

    msgpack_pack_str(&pk, 5); msgpack_pack_str_body(&pk, "trk_y", 5);
    msgpack_pack_uint8(&pk, data->yaw_position_percent);

    msgpack_pack_str(&pk, 5); msgpack_pack_str_body(&pk, "trk_p", 5);
    msgpack_pack_uint8(&pk, data->pitch_position_percent);

    // Thermal
    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "temp", 4);
    msgpack_pack_float(&pk, data->case_temperature);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "fan", 3);
    msgpack_pack_uint8(&pk, data->fan_speed_percent);

    // Energy statistics - hourly
    msgpack_pack_str(&pk, 6); msgpack_pack_str_body(&pk, "pv_wh_h", 6);
    msgpack_pack_float(&pk, data->pv_energy_wh_hour);

    msgpack_pack_str(&pk, 7); msgpack_pack_str_body(&pk, "bat_wh_h", 7);
    msgpack_pack_float(&pk, data->battery_energy_wh_hour);

    msgpack_pack_str(&pk, 7); msgpack_pack_str_body(&pk, "pv_pk_h", 7);
    msgpack_pack_float(&pk, data->pv_peak_w_hour);

    msgpack_pack_str(&pk, 8); msgpack_pack_str_body(&pk, "bat_pk_h", 8);
    msgpack_pack_float(&pk, data->battery_peak_w_hour);

    // Energy statistics - daily
    msgpack_pack_str(&pk, 6); msgpack_pack_str_body(&pk, "pv_wh_d", 6);
    msgpack_pack_float(&pk, data->pv_energy_wh_day);

    msgpack_pack_str(&pk, 7); msgpack_pack_str_body(&pk, "bat_wh_d", 7);
    msgpack_pack_float(&pk, data->battery_consumed_wh_day);

    msgpack_pack_str(&pk, 7); msgpack_pack_str_body(&pk, "pv_pk_d", 7);
    msgpack_pack_float(&pk, data->pv_peak_w_day);

    msgpack_pack_str(&pk, 8); msgpack_pack_str_body(&pk, "bat_pk_d", 8);
    msgpack_pack_float(&pk, data->battery_peak_w_day);

    // System state
    msgpack_pack_str(&pk, 5); msgpack_pack_str_body(&pk, "state", 5);
    msgpack_pack_uint8(&pk, data->power_state);

    if (sbuf.size > max_len) {
        msgpack_sbuffer_destroy(&sbuf);
        ESP_LOGE(TAG, "MessagePack buffer overflow: %zu > %d", sbuf.size, max_len);
        return ESP_ERR_NO_MEM;
    }

    memcpy(output, sbuf.data, sbuf.size);
    *output_len = sbuf.size;

    msgpack_sbuffer_destroy(&sbuf);

    ESP_LOGD(TAG, "Encoded FLUCTUS normal: %d bytes", *output_len);
    return ESP_OK;
}

/**
 * @brief Encode FLUCTUS realtime snapshot to MessagePack format
 * Realtime snapshot: instantaneous readings at 500ms intervals
 * Includes debug fields (photoresistors, duty cycles, SOC)
 */
static esp_err_t msgpack_encode_fluctus_rt(const fluctus_snapshot_t *data,
                                            uint8_t *output, uint16_t *output_len,
                                            uint16_t max_len)
{
    if (!data || !output || !output_len) {
        return ESP_ERR_INVALID_ARG;
    }

    msgpack_sbuffer sbuf;
    msgpack_sbuffer_init(&sbuf);

    msgpack_packer pk;
    msgpack_packer_init(&pk, &sbuf, msgpack_sbuffer_write);

    // Realtime snapshot: instantaneous readings + debug data
    msgpack_pack_map(&pk, 22);  // Rich version with all debug fields

    // Timestamp
    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "ts", 2);
    msgpack_pack_uint64(&pk, data->snapshot_timestamp);

    // Power buses (catches transient activity)
    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "b3e", 3);
    msgpack_pack_bin(&pk, data->bus_3v3_enabled);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "b5e", 3);
    msgpack_pack_bin(&pk, data->bus_5v_enabled);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "b66e", 4);
    msgpack_pack_bin(&pk, data->bus_6v6_enabled);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "b12e", 4);
    msgpack_pack_bin(&pk, data->bus_12v_enabled);

    // Battery - instantaneous
    msgpack_pack_str(&pk, 5); msgpack_pack_str_body(&pk, "bat_v", 5);
    msgpack_pack_float(&pk, data->battery_voltage_inst);

    msgpack_pack_str(&pk, 5); msgpack_pack_str_body(&pk, "bat_a", 5);
    msgpack_pack_float(&pk, data->battery_current_inst);

    msgpack_pack_str(&pk, 5); msgpack_pack_str_body(&pk, "bat_w", 5);
    msgpack_pack_float(&pk, data->battery_power_inst);

    msgpack_pack_str(&pk, 5); msgpack_pack_str_body(&pk, "bat_s", 5);  // DEBUG
    msgpack_pack_float(&pk, data->battery_soc_inst);

    // Solar - instantaneous
    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "pv_v", 4);
    msgpack_pack_float(&pk, data->solar_voltage_inst);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "pv_a", 4);
    msgpack_pack_float(&pk, data->solar_current_inst);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "pv_w", 4);
    msgpack_pack_float(&pk, data->solar_power_inst);

    msgpack_pack_str(&pk, 6); msgpack_pack_str_body(&pk, "pv_act", 6);
    msgpack_pack_bin(&pk, data->solar_pv_active);

    // Solar tracking - dynamic state
    msgpack_pack_str(&pk, 5); msgpack_pack_str_body(&pk, "trk_s", 5);
    msgpack_pack_uint8(&pk, data->tracking_state);

    msgpack_pack_str(&pk, 5); msgpack_pack_str_body(&pk, "trk_y", 5);
    msgpack_pack_uint8(&pk, data->yaw_position_percent);

    msgpack_pack_str(&pk, 5); msgpack_pack_str_body(&pk, "trk_p", 5);
    msgpack_pack_uint8(&pk, data->pitch_position_percent);

    msgpack_pack_str(&pk, 6); msgpack_pack_str_body(&pk, "trk_ye", 6);
    msgpack_pack_float(&pk, data->yaw_error);

    msgpack_pack_str(&pk, 6); msgpack_pack_str_body(&pk, "trk_pe", 6);
    msgpack_pack_float(&pk, data->pitch_error);

    msgpack_pack_str(&pk, 6); msgpack_pack_str_body(&pk, "trk_yd", 6);  // DEBUG: duty cycle
    msgpack_pack_uint32(&pk, data->current_yaw_duty);

    msgpack_pack_str(&pk, 6); msgpack_pack_str_body(&pk, "trk_pd", 6);  // DEBUG: duty cycle
    msgpack_pack_uint32(&pk, data->current_pitch_duty);

    // DEBUG: Photoresistor array [TL, TR, BL, BR]
    msgpack_pack_str(&pk, 5); msgpack_pack_str_body(&pk, "photo", 5);
    msgpack_pack_array(&pk, 4);
    msgpack_pack_float(&pk, data->photoresistor_readings[0]);
    msgpack_pack_float(&pk, data->photoresistor_readings[1]);
    msgpack_pack_float(&pk, data->photoresistor_readings[2]);
    msgpack_pack_float(&pk, data->photoresistor_readings[3]);

    // Thermal - instantaneous
    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "temp", 4);
    msgpack_pack_float(&pk, data->case_temperature);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "fan", 3);
    msgpack_pack_uint8(&pk, data->fan_speed_percent);

    if (sbuf.size > max_len) {
        msgpack_sbuffer_destroy(&sbuf);
        ESP_LOGE(TAG, "MessagePack buffer overflow: %zu > %d", sbuf.size, max_len);
        return ESP_ERR_NO_MEM;
    }

    memcpy(output, sbuf.data, sbuf.size);
    *output_len = sbuf.size;

    msgpack_sbuffer_destroy(&sbuf);

    ESP_LOGD(TAG, "Encoded FLUCTUS realtime: %d bytes", *output_len);
    return ESP_OK;
}

static esp_err_t msgpack_encode_stellaria(const stellaria_snapshot_t *data,
                                          uint8_t *output, uint16_t *output_len,
                                          uint16_t max_len)
{
    if (!data || !output || !output_len) {
        return ESP_ERR_INVALID_ARG;
    }

    msgpack_sbuffer sbuf;
    msgpack_sbuffer_init(&sbuf);
    msgpack_packer pk;
    msgpack_packer_init(&pk, &sbuf, msgpack_sbuffer_write);

    msgpack_pack_map(&pk, 4);

    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "ts", 2);
    msgpack_pack_uint64(&pk, data->snapshot_timestamp);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "int", 3);
    msgpack_pack_uint16(&pk, data->current_intensity);

    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "en", 2);
    msgpack_pack_uint8(&pk, data->driver_enabled ? 1 : 0);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "mode", 4);
    msgpack_pack_uint8(&pk, data->auto_mode_active);

    if (sbuf.size > max_len) {
        msgpack_sbuffer_destroy(&sbuf);
        return ESP_ERR_NO_MEM;
    }

    memcpy(output, sbuf.data, sbuf.size);
    *output_len = sbuf.size;
    msgpack_sbuffer_destroy(&sbuf);

    ESP_LOGD(TAG, "Encoded STELLARIA: %d bytes", *output_len);
    return ESP_OK;
}

static esp_err_t msgpack_encode_tempesta(const tempesta_snapshot_t *data,
                                         uint8_t *output, uint16_t *output_len,
                                         uint16_t max_len)
{
    if (!data || !output || !output_len) {
        return ESP_ERR_INVALID_ARG;
    }

    msgpack_sbuffer sbuf;
    msgpack_sbuffer_init(&sbuf);
    msgpack_packer pk;
    msgpack_packer_init(&pk, &sbuf, msgpack_sbuffer_write);

    // 25 fields total: 5 basic sensors + 4 wind + 3 rainfall + 3 tank + 8 status + 1 state + 1 timestamp
    msgpack_pack_map(&pk, 25);

    // Timestamp
    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "ts", 2);
    msgpack_pack_uint64(&pk, data->snapshot_timestamp);

    // Basic sensors (5)
    msgpack_pack_str(&pk, 1); msgpack_pack_str_body(&pk, "t", 1);
    msgpack_pack_float(&pk, data->temperature);

    msgpack_pack_str(&pk, 1); msgpack_pack_str_body(&pk, "h", 1);
    msgpack_pack_float(&pk, data->humidity);

    msgpack_pack_str(&pk, 1); msgpack_pack_str_body(&pk, "p", 1);
    msgpack_pack_float(&pk, data->pressure);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "pm2", 3);
    msgpack_pack_float(&pk, data->air_quality_pm25);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "pm10", 4);
    msgpack_pack_float(&pk, data->air_quality_pm10);

    // Wind measurements (4)
    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "wrpm", 4);
    msgpack_pack_float(&pk, data->wind_speed_rpm);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "wms", 3);
    msgpack_pack_float(&pk, data->wind_speed_ms);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "wdir", 4);
    msgpack_pack_float(&pk, data->wind_direction_deg);

    msgpack_pack_str(&pk, 5); msgpack_pack_str_body(&pk, "wcard", 5);
    if (data->wind_direction_cardinal) {
        msgpack_pack_str(&pk, strlen(data->wind_direction_cardinal));
        msgpack_pack_str_body(&pk, data->wind_direction_cardinal, strlen(data->wind_direction_cardinal));
    } else {
        msgpack_pack_nil(&pk);
    }

    // Rainfall measurements (3)
    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "rhr", 3);
    msgpack_pack_float(&pk, data->rainfall_last_hour_mm);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "rday", 4);
    msgpack_pack_float(&pk, data->rainfall_daily_mm);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "rwek", 4);
    msgpack_pack_float(&pk, data->rainfall_weekly_mm);

    // Tank intake measurements (3)
    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "thr", 3);
    msgpack_pack_float(&pk, data->tank_intake_last_hour_ml);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "tday", 4);
    msgpack_pack_float(&pk, data->tank_intake_daily_ml);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "twek", 4);
    msgpack_pack_float(&pk, data->tank_intake_weekly_ml);

    // Sensor status flags (8)
    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "st", 3);
    msgpack_pack_uint8(&pk, data->temp_sensor_status);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "sh", 3);
    msgpack_pack_uint8(&pk, data->humidity_sensor_status);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "sp", 3);
    msgpack_pack_uint8(&pk, data->pressure_sensor_status);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "saq", 3);
    msgpack_pack_uint8(&pk, data->air_quality_status);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "sw", 3);
    msgpack_pack_uint8(&pk, data->wind_sensor_status);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "swd", 3);
    msgpack_pack_uint8(&pk, data->wind_direction_status);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "sr", 3);
    msgpack_pack_uint8(&pk, data->rain_gauge_status);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "sti", 3);
    msgpack_pack_uint8(&pk, data->tank_intake_status);

    // System state
    msgpack_pack_str(&pk, 5); msgpack_pack_str_body(&pk, "state", 5);
    msgpack_pack_uint8(&pk, data->state);

    if (sbuf.size > max_len) {
        msgpack_sbuffer_destroy(&sbuf);
        return ESP_ERR_NO_MEM;
    }

    memcpy(output, sbuf.data, sbuf.size);
    *output_len = sbuf.size;
    msgpack_sbuffer_destroy(&sbuf);

    ESP_LOGD(TAG, "Encoded TEMPESTA: %d bytes", *output_len);
    return ESP_OK;
}

/**
 * @brief Encode IMPLUVIUM system-level data to MessagePack (split message 1/6)
 * Contains: system state, water level, hourly/daily stats, emergency, anomaly
 * Size: ~100-120 bytes (fits in 256-byte slot)
 */
static esp_err_t msgpack_encode_impluvium_system(const impluvium_snapshot_t *data,
                                                  uint8_t *output, uint16_t *output_len,
                                                  uint16_t max_len)
{
    if (!data || !output || !output_len) {
        return ESP_ERR_INVALID_ARG;
    }

    msgpack_sbuffer sbuf;
    msgpack_sbuffer_init(&sbuf);
    msgpack_packer pk;
    msgpack_packer_init(&pk, &sbuf, msgpack_sbuffer_write);

    // System state (5) + water (1) + stats (5) + emergency (5) + anomaly (2) + ts (1) = 19 fields
    msgpack_pack_map(&pk, 19);

    // Timestamp
    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "ts", 2);
    msgpack_pack_uint64(&pk, data->snapshot_timestamp);

    // System state (5)
    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "sta", 3);
    msgpack_pack_uint8(&pk, data->state);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "act", 3);
    msgpack_pack_uint8(&pk, data->active_zone);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "eme", 3);
    msgpack_pack_bin(&pk, data->emergency_stop);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "psv", 3);
    msgpack_pack_bin(&pk, data->power_save_mode);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "lsd", 3);
    msgpack_pack_bin(&pk, data->load_shed_shutdown);

    // Physical sensors (1)
    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "wlv", 3);
    msgpack_pack_float(&pk, data->water_level_percent);

    // Hourly/daily statistics (5)
    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "hrs", 3);
    msgpack_pack_uint64(&pk, data->current_hour_start);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "whr", 3);
    msgpack_pack_float(&pk, data->total_water_used_hour_ml);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "wday", 4);
    msgpack_pack_float(&pk, data->total_water_used_day_ml);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "ehr", 3);
    msgpack_pack_uint8(&pk, data->watering_events_hour);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "eday", 4);
    msgpack_pack_uint8(&pk, data->watering_events_day);

    // Emergency diagnostics (5)
    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "esta", 4);
    msgpack_pack_uint8(&pk, data->emergency_state);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "etz", 3);
    msgpack_pack_uint8(&pk, data->emergency_test_zone);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "efzm", 4);
    msgpack_pack_uint8(&pk, data->emergency_failed_zones_mask);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "ecf", 4);
    msgpack_pack_uint8(&pk, data->consecutive_failures);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "efr", 4);
    if (data->emergency_failure_reason) {
        msgpack_pack_str(&pk, strlen(data->emergency_failure_reason));
        msgpack_pack_str_body(&pk, data->emergency_failure_reason, strlen(data->emergency_failure_reason));
    } else {
        msgpack_pack_nil(&pk);
    }

    // Anomaly tracking (2)
    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "anom", 4);
    msgpack_pack_uint8(&pk, data->current_anomaly_type);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "ats", 3);
    msgpack_pack_uint64(&pk, data->anomaly_timestamp);

    if (sbuf.size > max_len) {
        msgpack_sbuffer_destroy(&sbuf);
        return ESP_ERR_NO_MEM;
    }

    memcpy(output, sbuf.data, sbuf.size);
    *output_len = sbuf.size;
    msgpack_sbuffer_destroy(&sbuf);

    ESP_LOGD(TAG, "Encoded IMPLUVIUM_SYSTEM: %d bytes", *output_len);
    return ESP_OK;
}

/**
 * @brief Encode IMPLUVIUM single zone data to MessagePack (split messages 2-6/6)
 * Contains: per-zone configuration, stats, and learning data
 * Size: ~80-100 bytes per zone (fits in 256-byte slot)
 *
 * @param data Full IMPLUVIUM snapshot
 * @param zone_id Zone to encode (0-4)
 * @param output Output buffer
 * @param output_len Output length
 * @param max_len Maximum output length
 */
static esp_err_t msgpack_encode_impluvium_zone(const impluvium_snapshot_t *data,
                                               uint8_t zone_id,
                                               uint8_t *output, uint16_t *output_len,
                                               uint16_t max_len)
{
    if (!data || !output || !output_len || zone_id >= 5) {
        return ESP_ERR_INVALID_ARG;
    }

    msgpack_sbuffer sbuf;
    msgpack_sbuffer_init(&sbuf);
    msgpack_packer pk;
    msgpack_packer_init(&pk, &sbuf, msgpack_sbuffer_write);

    // Zone data: zid (1) + config (5) + stats (4) + learning (8) + ts (1) = 19 fields
    msgpack_pack_map(&pk, 19);

    // Timestamp
    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "ts", 2);
    msgpack_pack_uint64(&pk, data->snapshot_timestamp);

    // Zone ID
    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "zid", 3);
    msgpack_pack_uint8(&pk, zone_id);

    // Configuration (5)
    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "en", 2);
    msgpack_pack_bin(&pk, data->zones[zone_id].watering_enabled);

    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "mc", 2);
    msgpack_pack_float(&pk, data->zones[zone_id].current_moisture_percent);

    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "mt", 2);
    msgpack_pack_float(&pk, data->zones[zone_id].target_moisture_percent);

    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "md", 2);
    msgpack_pack_float(&pk, data->zones[zone_id].moisture_deadband_percent);

    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "lw", 2);
    msgpack_pack_uint64(&pk, data->zones[zone_id].last_watered_time);

    // Hourly/daily statistics (4)
    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "vhr", 3);
    msgpack_pack_float(&pk, data->zones[zone_id].volume_used_hour_ml);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "vday", 4);
    msgpack_pack_float(&pk, data->zones[zone_id].volume_used_today_ml);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "ehr", 3);
    msgpack_pack_uint8(&pk, data->zones[zone_id].events_hour);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "eday", 4);
    msgpack_pack_uint8(&pk, data->zones[zone_id].events_day);

    // Learning algorithm (8)
    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "ppmp", 4);
    msgpack_pack_float(&pk, data->zones[zone_id].calculated_ppmp_ratio);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "pdu", 3);
    msgpack_pack_uint32(&pk, data->zones[zone_id].calculated_pump_duty);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "mgr", 3);
    msgpack_pack_float(&pk, data->zones[zone_id].target_moisture_gain_rate);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "conf", 4);
    msgpack_pack_float(&pk, data->zones[zone_id].confidence_level);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "tcor", 4);
    msgpack_pack_float(&pk, data->zones[zone_id].last_temperature_correction);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "psuc", 4);
    msgpack_pack_uint32(&pk, data->zones[zone_id].successful_predictions);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "ptot", 4);
    msgpack_pack_uint32(&pk, data->zones[zone_id].total_predictions);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "hist", 4);
    msgpack_pack_uint8(&pk, data->zones[zone_id].history_entry_count);

    if (sbuf.size > max_len) {
        msgpack_sbuffer_destroy(&sbuf);
        return ESP_ERR_NO_MEM;
    }

    memcpy(output, sbuf.data, sbuf.size);
    *output_len = sbuf.size;
    msgpack_sbuffer_destroy(&sbuf);

    ESP_LOGD(TAG, "Encoded IMPLUVIUM_ZONE_%d: %d bytes", zone_id, *output_len);
    return ESP_OK;
}

/**
 * @brief Encode IMPLUVIUM realtime snapshot to MessagePack format
 * Realtime snapshot: fast-changing sensors at 500ms intervals during watering
 * Includes pressure/flow alarms and active queue item
 */
static esp_err_t msgpack_encode_impluvium_rt(const impluvium_snapshot_rt_t *data,
                                             uint8_t *output, uint16_t *output_len,
                                             uint16_t max_len)
{
    if (!data || !output || !output_len) {
        return ESP_ERR_INVALID_ARG;
    }

    msgpack_sbuffer sbuf;
    msgpack_sbuffer_init(&sbuf);
    msgpack_packer pk;
    msgpack_packer_init(&pk, &sbuf, msgpack_sbuffer_write);

    // Realtime: sensors (4) + operation (3) + flags (4) + queue (3 fields + array) + ts (1) = 15 fields
    msgpack_pack_map(&pk, 15);

    // Timestamp
    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "ts", 2);
    msgpack_pack_uint64(&pk, data->snapshot_timestamp);

    // Fast-changing sensors (4)
    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "wlv", 3);
    msgpack_pack_float(&pk, data->water_level_percent);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "pres", 4);
    msgpack_pack_float(&pk, data->outlet_pressure_bar);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "flow", 4);
    msgpack_pack_float(&pk, data->current_flow_rate_lh);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "mgr", 3);
    msgpack_pack_float(&pk, data->current_moisture_gain_rate);

    // Current operation status (3)
    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "pdu", 3);
    msgpack_pack_uint8(&pk, data->pump_duty_percent);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "ppwm", 4);
    msgpack_pack_uint32(&pk, data->pump_pwm_duty);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "act", 3);
    msgpack_pack_uint8(&pk, data->active_zone);

    // System status flags (4)
    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "spw", 3);
    msgpack_pack_bin(&pk, data->sensors_powered);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "svl", 3);
    msgpack_pack_bin(&pk, data->sensor_data_valid);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "palm", 4);
    msgpack_pack_bin(&pk, data->pressure_alarm);

    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "falm", 4);
    msgpack_pack_bin(&pk, data->flow_alarm);

    // Watering queue (3 + array)
    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "qsz", 3);
    msgpack_pack_uint8(&pk, data->watering_queue_size);

    msgpack_pack_str(&pk, 3); msgpack_pack_str_body(&pk, "qix", 3);
    msgpack_pack_uint8(&pk, data->queue_index);

    msgpack_pack_str(&pk, 1); msgpack_pack_str_body(&pk, "q", 1);
    msgpack_pack_array(&pk, 1);  // Only 1 queue item in realtime

    // Encode queue item
    msgpack_pack_map(&pk, 5);

    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "zid", 2);
    msgpack_pack_uint8(&pk, data->queue[0].zone_id);

    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "mm", 2);
    msgpack_pack_float(&pk, data->queue[0].measured_moisture_percent);

    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "def", 2);
    msgpack_pack_float(&pk, data->queue[0].moisture_deficit_percent);

    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "tp", 2);
    msgpack_pack_uint16(&pk, data->queue[0].target_pulses);

    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "wc", 2);
    msgpack_pack_bin(&pk, data->queue[0].watering_completed);

    if (sbuf.size > max_len) {
        msgpack_sbuffer_destroy(&sbuf);
        return ESP_ERR_NO_MEM;
    }

    memcpy(output, sbuf.data, sbuf.size);
    *output_len = sbuf.size;
    msgpack_sbuffer_destroy(&sbuf);

    ESP_LOGD(TAG, "Encoded IMPLUVIUM_RT: %d bytes", *output_len);
    return ESP_OK;
}

static esp_err_t msgpack_encode_wifi(const wifi_snapshot_t *data,
                                      uint8_t *output, uint16_t *output_len,
                                      uint16_t max_len)
{
    if (!data || !output || !output_len) {
        return ESP_ERR_INVALID_ARG;
    }

    msgpack_sbuffer sbuf;
    msgpack_sbuffer_init(&sbuf);
    msgpack_packer pk;
    msgpack_packer_init(&pk, &sbuf, msgpack_sbuffer_write);

    msgpack_pack_map(&pk, 8);

    // Timestamp
    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "ts", 2);
    msgpack_pack_uint64(&pk, data->snapshot_timestamp);

    // WiFi state (0=DISABLED, 1=INIT, 2=CONNECTING, 3=CONNECTED, 4=RECONNECTING, 5=FAILED)
    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "st", 2);
    msgpack_pack_uint8(&pk, data->state);

    // RSSI (signal strength in dBm, -100 to 0)
    msgpack_pack_str(&pk, 4); msgpack_pack_str_body(&pk, "rssi", 4);
    msgpack_pack_int8(&pk, data->rssi);

    // Reconnection count
    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "rc", 2);
    msgpack_pack_uint16(&pk, data->reconnect_count);

    // Connected time (seconds)
    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "ct", 2);
    msgpack_pack_uint32(&pk, data->connected_time_sec);

    // Last disconnect reason
    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "dr", 2);
    msgpack_pack_uint32(&pk, data->last_disconnect_reason);

    // Has IP address
    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "ip", 2);
    msgpack_pack_uint8(&pk, data->has_ip ? 1 : 0);

    // Power save mode enabled
    msgpack_pack_str(&pk, 2); msgpack_pack_str_body(&pk, "ps", 2);
    msgpack_pack_uint8(&pk, data->power_save_mode ? 1 : 0);

    if (sbuf.size > max_len) {
        msgpack_sbuffer_destroy(&sbuf);
        return ESP_ERR_NO_MEM;
    }

    memcpy(output, sbuf.data, sbuf.size);
    *output_len = sbuf.size;
    msgpack_sbuffer_destroy(&sbuf);

    ESP_LOGD(TAG, "Encoded WiFi: %d bytes (state=%d, rssi=%d dBm)", *output_len, data->state, data->rssi);
    return ESP_OK;
}

// ################ MQTT Infrastructure ################

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            mqtt_connected = true;

            esp_mqtt_client_publish(mqtt_client, "solarium/status", "online", 6, 1, 1);
            esp_mqtt_client_subscribe(mqtt_client, "solarium/cmd/realtime", 1);

            ESP_LOGI(TAG, "MQTT ready");
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            mqtt_connected = false;
            in_flight_msg_id = -1;  // Clear in-flight flag on disconnect
            in_flight_seq = 0;
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(TAG, "MQTT message published (PUBACK): msg_id=%d", event->msg_id);

            // Handle QoS 1 PUBACK by dequeuing the matching message from ring buffer
            // NOTE: With hybrid retry logic, this check should match in_flight_msg_id instead
            if (event->msg_id == in_flight_msg_id) {
                buffer_dequeue();
                ESP_LOGI(TAG, "Dequeued QoS 1 message after PUBACK: seq=%lu, msg_id=%d",
                         in_flight_seq, event->msg_id);
                in_flight_msg_id = -1;  // Clear in-flight tracking
                in_flight_seq = 0;
            } else {
                ESP_LOGW(TAG, "PUBACK msg_id=%d doesn't match in-flight msg_id=%d",
                         event->msg_id, in_flight_msg_id);
            }
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGD(TAG, "MQTT data received: topic=%.*s, data=%.*s",
                     event->topic_len, event->topic,
                     event->data_len, event->data);

            // Handle realtime mode control command
            if (event->topic_len == 22 &&
                strncmp(event->topic, "solarium/cmd/realtime", 22) == 0) {

                if (event->data_len >= 4 && strncmp(event->data, "true", 4) == 0) {
                    telemetry_set_realtime_mode(true);
                    ESP_LOGI(TAG, "Realtime mode ENABLED by server command");
                } else if (event->data_len >= 5 && strncmp(event->data, "false", 5) == 0) {
                    telemetry_set_realtime_mode(false);
                    ESP_LOGI(TAG, "Realtime mode DISABLED by server command");
                } else {
                    ESP_LOGW(TAG, "Invalid realtime command payload: %.*s",
                             event->data_len, event->data);
                }
            }
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error occurred");
            break;

        default:
            ESP_LOGD(TAG, "MQTT event: %d", event->event_id);
            break;
    }
}

static const char* get_topic_name(telemetry_source_t src)
{
    // Check for IMPLUVIUM zone messages (component_id 100-104)
    if (src >= 100 && src <= 104) {
        static char zone_topic[32];
        uint8_t zone = src - 100;
        snprintf(zone_topic, sizeof(zone_topic), "solarium/impluvium/zone%d", zone);
        return zone_topic;
    }

    switch (src) {
        case TELEMETRY_SRC_FLUCTUS: return "solarium/fluctus";
        case TELEMETRY_SRC_FLUCTUS_RT: return "solarium/fluctus_rt";
        case TELEMETRY_SRC_TEMPESTA: return "solarium/tempesta";
        case TELEMETRY_SRC_IMPLUVIUM: return "solarium/impluvium/system";
        case TELEMETRY_SRC_IMPLUVIUM_RT: return "solarium/impluvium_rt";
        case TELEMETRY_SRC_STELLARIA: return "solarium/stellaria";
        case TELEMETRY_SRC_WIFI: return "solarium/wifi";
        default: return "solarium/unknown";
    }
}

static void mqtt_publish_task(void *parameters)
{
    mqtt_task_handle = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "MQTT publishing task started");

    uint8_t msgpack_buffer[BUFFER_PAYLOAD_SIZE];

    // Hybrid retry logic: Track session failures separately from message failures
    static uint8_t session_failure_count = 0;
    static TickType_t in_flight_start_time = 0;

    while (1) {
        uint32_t notified_sources = 0;
        xTaskNotifyWait(0, 0xFFFFFFFF, &notified_sources, portMAX_DELAY);

        if (notified_sources == 0) {
            continue;
        }

        ESP_LOGD(TAG, "Received notification from sources: 0x%08lX", notified_sources);

        for (telemetry_source_t src = 0; src < TELEMETRY_SRC_COUNT; src++) {
            if (!(notified_sources & (1 << src))) {
                continue;
            }

            void *cache_ptr = NULL;
            if (telemetry_lock_cache(src, &cache_ptr) != ESP_OK) {
                ESP_LOGW(TAG, "Failed to lock cache for source %d", src);
                continue;
            }

            uint16_t payload_len = 0;
            esp_err_t encode_ret = ESP_FAIL;

            switch (src) {
                case TELEMETRY_SRC_FLUCTUS:
                    encode_ret = msgpack_encode_fluctus((fluctus_snapshot_t*)cache_ptr,
                                                       msgpack_buffer, &payload_len,
                                                       BUFFER_PAYLOAD_SIZE);
                    break;
                case TELEMETRY_SRC_FLUCTUS_RT:
                    // Encodes RT subset from unified structure
                    encode_ret = msgpack_encode_fluctus_rt((fluctus_snapshot_t*)cache_ptr,
                                                          msgpack_buffer, &payload_len,
                                                          BUFFER_PAYLOAD_SIZE);
                    break;
                case TELEMETRY_SRC_TEMPESTA:
                    encode_ret = msgpack_encode_tempesta((tempesta_snapshot_t*)cache_ptr,
                                                        msgpack_buffer, &payload_len,
                                                        BUFFER_PAYLOAD_SIZE);
                    break;
                case TELEMETRY_SRC_IMPLUVIUM:
                    // IMPLUVIUM split encoding: 1 system + 5 zone messages
                    // Encode system message first
                    encode_ret = msgpack_encode_impluvium_system((impluvium_snapshot_t*)cache_ptr,
                                                                msgpack_buffer, &payload_len,
                                                                BUFFER_PAYLOAD_SIZE);

                    if (encode_ret == ESP_OK) {
                        // Enqueue system message (component_id = TELEMETRY_SRC_IMPLUVIUM = 3)
                        buffer_enqueue(msgpack_buffer, payload_len, TELEMETRY_SRC_IMPLUVIUM, 0);

                        // Encode and enqueue 5 zone messages
                        // Encode zone in component_id: (IMPLUVIUM_BASE + zone_id)
                        // Zone 0-4 = component_id 100-104 (decoded in get_topic_name)
                        for (uint8_t zone = 0; zone < 5; zone++) {
                            payload_len = 0;
                            encode_ret = msgpack_encode_impluvium_zone((impluvium_snapshot_t*)cache_ptr,
                                                                      zone,
                                                                      msgpack_buffer, &payload_len,
                                                                      BUFFER_PAYLOAD_SIZE);
                            if (encode_ret == ESP_OK) {
                                // Encode zone in component_id: 100 + zone (100-104)
                                uint8_t zone_component_id = 100 + zone;
                                buffer_enqueue(msgpack_buffer, payload_len,
                                             (telemetry_source_t)zone_component_id, 0);
                            } else {
                                ESP_LOGE(TAG, "Failed to encode IMPLUVIUM zone %d", zone);
                            }
                        }
                    }

                    // Unlock and skip normal enqueue logic (already enqueued above)
                    telemetry_unlock_cache(src);
                    continue;  // Skip to next source
                case TELEMETRY_SRC_IMPLUVIUM_RT:
                    encode_ret = msgpack_encode_impluvium_rt((impluvium_snapshot_rt_t*)cache_ptr,
                                                            msgpack_buffer, &payload_len,
                                                            BUFFER_PAYLOAD_SIZE);
                    break;
                case TELEMETRY_SRC_STELLARIA:
                    encode_ret = msgpack_encode_stellaria((stellaria_snapshot_t*)cache_ptr,
                                                         msgpack_buffer, &payload_len,
                                                         BUFFER_PAYLOAD_SIZE);
                    break;
                case TELEMETRY_SRC_WIFI:
                    encode_ret = msgpack_encode_wifi((wifi_snapshot_t*)cache_ptr,
                                                    msgpack_buffer, &payload_len,
                                                    BUFFER_PAYLOAD_SIZE);
                    break;
                default:
                    ESP_LOGW(TAG, "Unknown source %d", src);
                    break;
            }

            telemetry_unlock_cache(src);

            if (encode_ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to encode source %d", src);
                continue;
            }

            uint8_t message_type = (src == TELEMETRY_SRC_FLUCTUS_RT ||
                                   src == TELEMETRY_SRC_IMPLUVIUM_RT) ? 1 : 0;

            // Realtime mode filtering: Skip enqueuing RT messages when disabled
            bool is_realtime = (message_type == 1);

            if (is_realtime && !realtime_mode_enabled) {
                // Realtime mode disabled - do NOT enqueue, skip entirely
                // This saves PSRAM buffer space when realtime is off
                ESP_LOGD(TAG, "Realtime disabled, skipping enqueue: %s", get_topic_name(src));
                continue;  // Skip to next source
            }

            if (buffer_enqueue(msgpack_buffer, payload_len, src, message_type) == ESP_OK) {
                ESP_LOGI(TAG, "Enqueued %s: %d bytes (buffer: %d/%d)",
                        get_topic_name(src), payload_len,
                        buffer_get_count(), BUFFER_CAPACITY);
            }
        }

        // FLASH backup: When buffer critical, flush to FLASH
        // Triggers on EITHER disconnection OR stalled connection (buffer near full)
        uint16_t count = buffer_get_count();
        if (count >= FLASH_BACKUP_THRESHOLD && flash_backup_file != NULL) {
            if (!mqtt_connected) {
                ESP_LOGW(TAG, "MQTT disconnected, PSRAM buffer critical (%d/%d), flushing to FLASH",
                        count, BUFFER_CAPACITY);
            } else {
                ESP_LOGW(TAG, "PSRAM buffer critical (%d/%d), MQTT stalled? Flushing to FLASH",
                        count, BUFFER_CAPACITY);
            }

            // Flush batch of messages to FLASH (incremental, non-blocking)
            buffer_slot_t flush_slot;
            uint16_t flushed = 0;

            for (uint16_t i = 0; i < FLASH_BATCH_SIZE && buffer_peek(&flush_slot, NULL) == ESP_OK; i++) {
                if (flash_backup_write_slot(&flush_slot) == ESP_OK) {
                    buffer_dequeue();
                    flushed++;
                } else {
                    ESP_LOGE(TAG, "FLASH write failed, stopping flush");
                    break;
                }
            }

            if (flushed > 0) {
                ESP_LOGI(TAG, "Flushed %d messages to FLASH (buffer: %d/%d)",
                        flushed, buffer_get_count(), BUFFER_CAPACITY);
            }
        }

        buffer_slot_t slot;
        uint16_t current_tail_index;

        while (buffer_peek(&slot, &current_tail_index) == ESP_OK) {
            if (!mqtt_connected) {
                ESP_LOGD(TAG, "MQTT not connected, buffering messages (%d queued)",
                        buffer_get_count());
                break;
            }

            // Power state filtering: Check if normal mode publishing is disabled
            // message_type: 0=normal, 1=realtime, 4=control
            // Control messages (type 4) ALWAYS bypass power state restrictions
            bool is_control = (slot.message_type == 4);

            if (!is_control && !telemetry_publishing_enabled) {
                // Normal/realtime publishing disabled (SOC <15%, VERY_LOW state)
                // Control messages always published regardless of power state
                // Keep non-control messages in buffer (will retry when SOC recovers)
                ESP_LOGD(TAG, "Normal mode buffering only (publishing disabled), buffered: seq=%lu, count=%d",
                         slot.global_seq, buffer_get_count());
                break;  // Stop processing queue, messages stay buffered
            }

            const char *topic = get_topic_name((telemetry_source_t)slot.component_id);
            uint8_t qos = (slot.message_type == 1) ? 0 : 1;

            // *** QoS 1 Hybrid Retry Logic: Only one message in-flight at a time ***
            if (in_flight_msg_id != -1) {
                // Check if PUBACK timed out (5 seconds)
                TickType_t now = xTaskGetTickCount();
                TickType_t elapsed = now - in_flight_start_time;

                if (elapsed > pdMS_TO_TICKS(5000)) {
                    ESP_LOGW(TAG, "PUBACK timeout (5s) for msg_id=%d, seq=%lu",
                             in_flight_msg_id, in_flight_seq);

                    // --- START HYBRID RETRY LOGIC ---

                    // 1. Increment per-message lifetime retry_count in the actual buffer slot
                    uint8_t current_retries = 0;
                    if (xSemaphoreTake(xBufferMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        // Access the *actual* slot in the buffer, not the copy
                        psram_buffer[in_flight_tail_index].retry_count++;
                        current_retries = psram_buffer[in_flight_tail_index].retry_count;
                        xSemaphoreGive(xBufferMutex);
                    }

                    // 2. Increment session failure count
                    session_failure_count++;

                    // 3. Check for POISON MESSAGE (10 total lifetime failures)
                    const uint8_t MAX_LIFETIME_RETRIES = 10;
                    if (current_retries >= MAX_LIFETIME_RETRIES) {
                        ESP_LOGE(TAG, "Dropping POISON message seq=%lu after %d lifetime failures",
                                 in_flight_seq, current_retries);

                        buffer_dequeue(); // Drop the message by advancing the tail

                        // Reset all counters and proceed to next message
                        in_flight_msg_id = -1;
                        in_flight_seq = 0;
                        session_failure_count = 0;
                        continue; // Unblocks the queue
                    }

                    // 4. Check for BAD SESSION (5 consecutive failures)
                    const uint8_t MAX_SESSION_FAILURES = 5;
                    if (session_failure_count >= MAX_SESSION_FAILURES) {
                        ESP_LOGE(TAG, "Session failure limit reached (%d timeouts). Assuming bad connection.",
                                 session_failure_count);
                        ESP_LOGW(TAG, "Forcing disconnect to trigger exponential backoff.");

                        // Reset for next session, but NOT the message
                        session_failure_count = 0;
                        in_flight_msg_id = -1;

                        // Force a disconnect. The event handler will set mqtt_connected=false,
                        // and the built-in MQTT client will handle exponential backoff and reconnect.
                        // The message is NOT dropped and will be retried with a fresh connection.
                        esp_mqtt_client_disconnect(mqtt_client);
                        continue;
                    }

                    // 5. Just a normal timeout, retry the same message
                    ESP_LOGW(TAG, "Retrying message seq=%lu (Attempt %d/%d, Session Fail %d/%d)",
                             in_flight_seq, current_retries, MAX_LIFETIME_RETRIES,
                             session_failure_count, MAX_SESSION_FAILURES);

                    in_flight_msg_id = -1; // Clear to allow retry on next loop

                    // --- END HYBRID RETRY LOGIC ---
                    in_flight_start_time = 0;
                    // Do NOT dequeue - message stays in buffer for retry
                    continue;
                } else {
                    // Still waiting for PUBACK, timeout not reached
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
            }

            // Publish message
            int msg_id = esp_mqtt_client_publish(mqtt_client, topic,
                                                 (const char*)slot.payload,
                                                 slot.payload_len, qos, 0);

            if (msg_id >= 0) {
                ESP_LOGI(TAG, "Published %s: seq=%lu, qos=%d, msg_id=%d%s",
                        topic, slot.global_seq, qos, msg_id,
                        (qos == 1 && slot.retry_count > 0) ? " (retry)" : "");

                if (qos == 0) {
                    // QoS 0: Fire-and-forget, dequeue immediately
                    buffer_dequeue();
                    ESP_LOGD(TAG, "Published QoS 0: seq=%lu (dequeued)", slot.global_seq);
                } else {
                    // QoS 1: Store msg_id, tail index, and wait for PUBACK in event handler
                    in_flight_msg_id = msg_id;
                    in_flight_seq = slot.global_seq;
                    in_flight_tail_index = current_tail_index;  // CRITICAL: Track buffer index for retry_count
                    in_flight_start_time = xTaskGetTickCount();

                    // Reset session failure count on successful publish (new transmission cycle)
                    session_failure_count = 0;

                    ESP_LOGD(TAG, "Published QoS 1: seq=%lu, msg_id=%d (awaiting PUBACK)",
                             in_flight_seq, in_flight_msg_id);
                    // Message stays in buffer until event handler confirms delivery
                    break;  // Wait for PUBACK before processing next message
                }
            } else {
                ESP_LOGW(TAG, "Publish failed for %s (msg_id=%d)", topic, msg_id);
                vTaskDelay(pdMS_TO_TICKS(1000));  // Wait before retry
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ################ Realtime Mode & Power Control ################

/**
 * @brief Get current realtime mode state
 */
bool telemetry_is_realtime_enabled(void)
{
    return realtime_mode_enabled;
}

/**
 * @brief Get current normal telemetry publishing state
 */
bool telemetry_is_telemetry_publishing_enabled(void)
{
    return telemetry_publishing_enabled;
}

/**
 * @brief Set realtime mode enabled/disabled
 * Stores user preference for restoration after power state recovery
 */
void telemetry_set_realtime_mode(bool enabled)
{
    realtime_mode_user_preference = enabled;
    realtime_mode_enabled = enabled;

    ESP_LOGI(TAG, "Realtime mode %s (user preference)", enabled ? "ENABLED" : "DISABLED");
}

/**
 * @brief Enable or disable normal telemetry publishing
 * When disabled, normal mode messages are enqueued but NOT published (buffering only)
 * Control messages (message_type == 4) are ALWAYS published regardless
 * Called by FLUCTUS at VERY_LOW/CRITICAL states (SOC <15%)
 */
void telemetry_enable_telemetry_publishing(bool mode)
{
    telemetry_publishing_enabled = mode;
    ESP_LOGI(TAG, "Normal mode publishing %s", mode ? "ENABLED" : "BUFFERING ONLY");
}

/**
 * @brief Force disable/enable realtime mode (power state override)
 * Called by FLUCTUS to override realtime mode based on battery SOC
 * Does NOT update user preference - allows restoration when SOC recovers
 *
 * @param disable true to force disable, false to restore user preference
 */
void telemetry_force_realtime_monitoring_disable(bool disable)
{
    if (disable) {
        realtime_mode_enabled = false;
        ESP_LOGI(TAG, "Realtime mode FORCE DISABLED (power state override)");
    } else {
        realtime_mode_enabled = realtime_mode_user_preference;
        ESP_LOGI(TAG, "Realtime mode restored to user preference: %s",
                 realtime_mode_enabled ? "ENABLED" : "DISABLED");
    }
}

/**
 * @brief Get current MQTT buffer status
 */
esp_err_t telemetry_get_buffer_status(uint16_t *buffered_count, uint16_t *buffer_capacity)
{
    if (!telemetry_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (buffered_count != NULL) {
        *buffered_count = buffer_get_count();
    }

    if (buffer_capacity != NULL) {
        *buffer_capacity = BUFFER_CAPACITY;
    }

    return ESP_OK;
}

/**
 * @brief Manually flush all buffered MQTT messages from PSRAM to FLASH
 * Useful before user-triggered MCU reset to preserve unsent messages
 */
esp_err_t telemetry_manual_flush_to_flash(uint16_t *flushed_count)
{
    if (!telemetry_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (flash_backup_file == NULL) {
        ESP_LOGW(TAG, "FLASH backup not available for manual flush");
        if (flushed_count != NULL) {
            *flushed_count = 0;
        }
        return ESP_FAIL;
    }

    uint16_t count = buffer_get_count();
    if (count == 0) {
        ESP_LOGI(TAG, "No buffered messages to flush");
        if (flushed_count != NULL) {
            *flushed_count = 0;
        }
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Manual flush requested: %d messages in buffer", count);

    uint16_t flushed = 0;
    buffer_slot_t flush_slot;

    // Flush all messages from PSRAM to FLASH
    while (buffer_peek(&flush_slot, NULL) == ESP_OK) {
        if (flash_backup_write_slot(&flush_slot) == ESP_OK) {
            buffer_dequeue();
            flushed++;
        } else {
            ESP_LOGE(TAG, "FLASH write failed during manual flush at message %d", flushed);
            break;
        }
    }

    // Clean shutdown optimization: Write the final head pointer to file header
    // This allows fast recovery on next boot without expensive qsort scan
    if (flushed > 0 && flash_backup_file != NULL && xFlashMutex != NULL) {
        if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_LOGI(TAG, "Writing final head pointer (%d) for clean shutdown", flash_head);

            fseek(flash_backup_file, 10, SEEK_SET);  // Offset to head field in header
            fwrite(&flash_head, sizeof(flash_head), 1, flash_backup_file);
            fflush(flash_backup_file);

            xSemaphoreGive(xFlashMutex);
        }
    }

    ESP_LOGI(TAG, "Manual flush completed: %d/%d messages saved to FLASH", flushed, count);

    if (flushed_count != NULL) {
        *flushed_count = flushed;
    }

    return (flushed > 0) ? ESP_OK : ESP_FAIL;
}

// ################ Component Get Functions (HMI Retrieval) ################

/**
 * @brief Get cached STELLARIA data for HMI display
 */
esp_err_t telemetry_get_stellaria_data(stellaria_snapshot_t *data)
{
    if (!telemetry_initialized || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    SemaphoreHandle_t m = telemetry_get_mutex(TELEMETRY_SRC_STELLARIA);
    if (m == NULL || xSemaphoreTake(m, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    memcpy(data, &cached_stellaria_data, sizeof(stellaria_snapshot_t));

    xSemaphoreGive(m);

    return ESP_OK;
}

/**
 * @brief Get cached FLUCTUS data for HMI display (full snapshot)
 */
esp_err_t telemetry_get_fluctus_data(fluctus_snapshot_t *data)
{
    if (!telemetry_initialized || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    SemaphoreHandle_t m = telemetry_get_mutex(TELEMETRY_SRC_FLUCTUS);
    if (m == NULL || xSemaphoreTake(m, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    memcpy(data, &cached_fluctus_data, sizeof(fluctus_snapshot_t));

    xSemaphoreGive(m);

    return ESP_OK;
}

// Removed: telemetry_get_fluctus_realtime_data() - unified structure now uses single get function

/**
 * @brief Get cached TEMPESTA data for HMI display
 */
esp_err_t telemetry_get_tempesta_data(tempesta_snapshot_t *data)
{
    if (!telemetry_initialized || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    SemaphoreHandle_t m = telemetry_get_mutex(TELEMETRY_SRC_TEMPESTA);
    if (m == NULL || xSemaphoreTake(m, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    memcpy(data, &cached_tempesta_data, sizeof(tempesta_snapshot_t));

    xSemaphoreGive(m);

    return ESP_OK;
}

/**
 * @brief Get cached IMPLUVIUM data for HMI display
 */
esp_err_t telemetry_get_impluvium_data(impluvium_snapshot_t *data)
{
    if (!telemetry_initialized || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    SemaphoreHandle_t m = telemetry_get_mutex(TELEMETRY_SRC_IMPLUVIUM);
    if (m == NULL || xSemaphoreTake(m, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    memcpy(data, &cached_impluvium_data, sizeof(impluvium_snapshot_t));

    xSemaphoreGive(m);

    return ESP_OK;
}

/**
 * @brief Get cached IMPLUVIUM realtime data for HMI display
 */
esp_err_t telemetry_get_impluvium_realtime_data(impluvium_snapshot_rt_t *data)
{
    if (!telemetry_initialized || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    SemaphoreHandle_t m = telemetry_get_mutex(TELEMETRY_SRC_IMPLUVIUM_RT);
    if (m == NULL || xSemaphoreTake(m, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    memcpy(data, &cached_impluvium_realtime_data, sizeof(impluvium_snapshot_rt_t));

    xSemaphoreGive(m);

    return ESP_OK;
}

/**
 * @brief Get cached WiFi data for HMI display
 */
esp_err_t telemetry_get_wifi_data(wifi_snapshot_t *data)
{
    if (!telemetry_initialized || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    SemaphoreHandle_t m = telemetry_get_mutex(TELEMETRY_SRC_WIFI);
    if (m == NULL || xSemaphoreTake(m, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    memcpy(data, &cached_wifi_data, sizeof(wifi_snapshot_t));

    xSemaphoreGive(m);

    return ESP_OK;
}