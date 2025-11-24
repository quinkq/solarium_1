/**
 * @file telemetry_mqtt_buffer.c
 * @brief TELEMETRY MQTT buffering - Two-tier storage system
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Two-tier MQTT buffering system for TELEMETRY component:
 * - PSRAM ring buffer: 1820 slots × 288 bytes = 512KB (primary, volatile)
 * - FLASH backup: 4096 slots × 288 bytes = ~1.1MB (secondary, persistent)
 *
 * PSRAM buffer features:
 * - Circular buffer with head/tail pointers
 * - Auto-dequeue on MQTT PUBACK (QoS 1)
 * - Peek/dequeue operations for publish task
 *
 * FLASH backup features:
 * - Automatic flush at 95% PSRAM capacity (1729 slots)
 * - Manual flush via HMI before MCU reset
 * - Boot recovery: restore to PSRAM, then delete file
 * - Chronological order maintained via qsort by global_seq
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "telemetry.h"
#include "telemetry_private.h"

#include <unistd.h>


static const char *TAG = "TELEMETRY_MQTT_BUFFER";


// ################ PSRAM Ring Buffer ################

/**
 * @brief Initialize PSRAM ring buffer for MQTT messages
 * Allocates 512KB in SPIRAM (1820 slots × 288 bytes)
 */
esp_err_t telemetry_buffer_init(void)
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

/**
 * @brief Enqueue message to PSRAM ring buffer
 * Called after msgpack encoding, before MQTT publishing
 */
esp_err_t telemetry_buffer_enqueue(const uint8_t *payload, uint16_t payload_len,
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

/**
 * @brief Peek at next message in buffer without removing it
 * Used by publish task to read message before MQTT publish
 */
esp_err_t telemetry_buffer_peek(buffer_slot_t *out_slot, uint16_t *out_tail_index)
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

/**
 * @brief Remove message from buffer after successful MQTT publish
 * Called in event handler upon QoS 1 PUBACK
 */
esp_err_t telemetry_buffer_dequeue(void)
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

/**
 * @brief Get current number of messages in PSRAM buffer
 * Thread-safe read of buffer_count
 */
uint16_t telemetry_buffer_get_count(void)
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
 * This function is called AFTER telemetry_flash_backup_load_on_boot() has completed.
 * The old backup file (if any) has been deleted by the load function.
 * This function creates a fresh, empty backup file for this session.
 *
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t telemetry_flash_backup_init(void)
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
esp_err_t telemetry_flash_backup_write_slot(const buffer_slot_t *slot)
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
    // On boot, telemetry_flash_backup_load_on_boot() reconstructs order using global_seq.
    uint16_t written_index = flash_head;
    flash_head = (flash_head + 1) % FLASH_BACKUP_CAPACITY;

    xSemaphoreGive(xFlashMutex);

    ESP_LOGD(TAG, "Wrote slot seq=%lu to FLASH at index %d", slot->global_seq, written_index);

    return ESP_OK;
}

/**
 * @brief Helper structure for sorting backup entries by sequence number
 */

/**
 * @brief qsort comparison function for backup entries
 */
int telemetry_compare_backup_entries(const void *a, const void *b)
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
esp_err_t telemetry_flash_backup_load_on_boot(void)
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
    qsort(entries, occupied_count, sizeof(backup_entry_t), telemetry_compare_backup_entries);

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
