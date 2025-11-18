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
#include "telemetry_private.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_littlefs.h"

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
SemaphoreHandle_t xTelemetryMutex = NULL;

// Initialization flags
bool telemetry_initialized = false;
bool littlefs_mounted = false;

// ################ Component Data Hub (Central Storage) ################

// Static storage for component data caches
stellaria_snapshot_t cached_stellaria_data = {0};
fluctus_snapshot_t cached_fluctus_data = {0};  // Unified: shared by both FLUCTUS and FLUCTUS_RT
tempesta_snapshot_t cached_tempesta_data = {0};
impluvium_snapshot_t cached_impluvium_data = {0};
impluvium_snapshot_rt_t cached_impluvium_realtime_data = {0};
wifi_snapshot_t cached_wifi_data = {0};

// Unified cache infrastructure
SemaphoreHandle_t xTelemetryMutexes[TELEMETRY_SRC_COUNT] = {NULL};

#ifdef CONFIG_TELEMETRY_MQTT_ENABLE
TaskHandle_t mqtt_task_handle = NULL;

// MQTT infrastructure
esp_mqtt_client_handle_t mqtt_client = NULL;
bool mqtt_connected = false;

// In-flight message tracking (for QoS 1 PUBACK handling and hybrid retry logic)
volatile int in_flight_msg_id = -1;         // MQTT msg_id awaiting PUBACK
volatile uint32_t in_flight_seq = 0;        // Sequence number (for logging)
volatile uint16_t in_flight_tail_index = 0; // Buffer tail index (for retry_count tracking)
#endif  // CONFIG_TELEMETRY_MQTT_ENABLE

// Realtime mode and publishing control flags
bool realtime_mode_enabled = false;          // Current active state
bool realtime_mode_user_preference = false;  // Last user/server preference
bool telemetry_publishing_enabled = true;    // Normal mode publishing control

buffer_slot_t *psram_buffer = NULL;
uint16_t buffer_head = 0;
uint16_t buffer_tail = 0;
uint16_t buffer_count = 0;
SemaphoreHandle_t xBufferMutex = NULL;
uint32_t global_sequence = 0;

// FLASH backup state
FILE *flash_backup_file = NULL;
uint16_t flash_head = 0;
SemaphoreHandle_t xFlashMutex = NULL;

telemetry_stream_t streams[TELEMETRY_SRC_COUNT] = {0};

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

    ret = telemetry_buffer_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PSRAM ring buffer");
        return ret;
    }

    // 1. Load any existing backup from previous boot
    //    This function opens, reads, loads to PSRAM, and DELETES the backup file.
    ret = telemetry_flash_backup_load_on_boot();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Loaded buffered messages from FLASH backup");
    }

    // 2. Initialize FLASH backup system for THIS session
    //    Since the old file is gone, this creates a new, empty one.
    //    The 'flash_backup_file' handle will now be valid for the entire runtime.
    ret = telemetry_flash_backup_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "FLASH backup init failed (continuing without backup)");
    }

#ifdef CONFIG_TELEMETRY_MQTT_ENABLE
    ret = telemetry_mqtt_client_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client (inner)");
    }
#else
    ESP_LOGI(TAG, "MQTT client disabled via Kconfig (CONFIG_TELEMETRY_MQTT_ENABLE=n)");
#endif

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

    telemetry_initialized = true;
    ESP_LOGI(TAG, "Telemetry initialization complete (MQTT enabled)");
    return ESP_OK;
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
        *buffered_count = telemetry_buffer_get_count();
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

    uint16_t count = telemetry_buffer_get_count();
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
    while (telemetry_buffer_peek(&flush_slot, NULL) == ESP_OK) {
        if (telemetry_flash_backup_write_slot(&flush_slot) == ESP_OK) {
            telemetry_buffer_dequeue();
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