#ifndef IMPLUVIUM_PRIVATE_H
#define IMPLUVIUM_PRIVATE_H

#include "telemetry.h"
#include "mqtt_client.h"

#include "esp_log.h"



// ########################## Constants and Variables ##########################

// Initialization
extern bool telemetry_initialized;
extern bool littlefs_mounted;

// Mutexes
extern SemaphoreHandle_t xTelemetryMutex;
extern SemaphoreHandle_t xTelemetryMutexes[TELEMETRY_SRC_COUNT];
extern SemaphoreHandle_t xBufferMutex;
extern SemaphoreHandle_t xFlashMutex;

// MQTT infrastructure
extern esp_mqtt_client_handle_t mqtt_client;
extern bool mqtt_connected;
extern TaskHandle_t mqtt_task_handle;

// In-flight tracking (QoS 1)
extern volatile int in_flight_msg_id;
extern volatile uint32_t in_flight_seq;
extern volatile uint16_t in_flight_tail_index;

// Control flags
extern bool realtime_mode_enabled;
extern bool realtime_mode_user_preference;
extern bool telemetry_publishing_enabled;

// Cached data (central storage)
extern stellaria_snapshot_t cached_stellaria_data;
extern fluctus_snapshot_t cached_fluctus_data;
extern tempesta_snapshot_t cached_tempesta_data;
extern impluvium_snapshot_t cached_impluvium_data;
extern impluvium_snapshot_rt_t cached_impluvium_realtime_data;
extern wifi_snapshot_t cached_wifi_data;

// Buffer slot (PSRAM + FLASH)
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
    uint32_t last_publish_time;
    uint8_t payload[BUFFER_PAYLOAD_SIZE];
    uint32_t crc32;
} buffer_slot_t;

// PSRAM ring buffer
extern buffer_slot_t *psram_buffer;
extern uint16_t buffer_head;
extern uint16_t buffer_tail;
extern uint16_t buffer_count;
extern uint32_t global_sequence;

// FLASH backup
extern FILE *flash_backup_file;
extern uint16_t flash_head;

// Stream configuration
typedef esp_err_t (*telemetry_writer_fn_t)(void *cache);

typedef struct {
    void *cache;
    telemetry_writer_fn_t writer_function;
} telemetry_stream_t;

// Stream table
extern telemetry_stream_t streams[TELEMETRY_SRC_COUNT];

// Flash backup sorting helper
typedef struct {
    uint32_t global_seq;
    uint16_t file_index;
} backup_entry_t;

// ########################## Private Function Declarations ##########################

// Cache (telemetry_cache.c)
SemaphoreHandle_t telemetry_get_mutex(telemetry_source_t src);

// MQTT Buffer (telemetry_mqtt_buffer.c)
esp_err_t telemetry_buffer_init(void);
esp_err_t telemetry_buffer_enqueue(const uint8_t *payload, uint16_t payload_len,
                         telemetry_source_t src, uint8_t message_type);
esp_err_t telemetry_buffer_peek(buffer_slot_t *out_slot, uint16_t *out_tail_index);
esp_err_t telemetry_buffer_dequeue(void);
uint16_t telemetry_buffer_get_count(void);
esp_err_t telemetry_buffer_increment_retry(uint16_t tail_index);
esp_err_t telemetry_flash_backup_init(void);
esp_err_t telemetry_flash_backup_write_slot(const buffer_slot_t *slot);
esp_err_t telemetry_flash_backup_load_on_boot(void);
int telemetry_compare_backup_entries(const void *a, const void *b);

// MessagePack (telemetry_msgpack.c)
esp_err_t telemetry_msgpack_encode_fluctus(const fluctus_snapshot_t *data,
                                 uint8_t *output, uint16_t *output_len,
                                 uint16_t max_len);
esp_err_t telemetry_msgpack_encode_fluctus_rt(const fluctus_snapshot_t *data,
                                    uint8_t *output, uint16_t *output_len,
                                    uint16_t max_len);
esp_err_t telemetry_msgpack_encode_stellaria(const stellaria_snapshot_t *data,
                                   uint8_t *output, uint16_t *output_len,
                                   uint16_t max_len);
esp_err_t telemetry_msgpack_encode_tempesta(const tempesta_snapshot_t *data,
                                  uint8_t *output, uint16_t *output_len,
                                  uint16_t max_len);
esp_err_t telemetry_msgpack_encode_impluvium_system(const impluvium_snapshot_t *data,
                                         uint8_t *output, uint16_t *output_len,
                                         uint16_t max_len);
esp_err_t telemetry_msgpack_encode_impluvium_zone(const impluvium_snapshot_t *data, 
                                       uint8_t zone_id, uint8_t *output,
                                       uint16_t *output_len, uint16_t max_len);
esp_err_t telemetry_msgpack_encode_impluvium_rt(const impluvium_snapshot_rt_t *data,
                                     uint8_t *output, uint16_t *output_len,
                                     uint16_t max_len);
esp_err_t telemetry_msgpack_encode_wifi(const wifi_snapshot_t *data,
                             uint8_t *output, uint16_t *output_len,
                             uint16_t max_len);

// MQTT Client (telemetry_mqtt_client.c)
esp_err_t telemetry_mqtt_client_init(void);
void telemetry_mqtt_event_handler(void *handler_args, esp_event_base_t base,
                       int32_t event_id, void *event_data);
void telemetry_mqtt_publish_task(void *parameters);

// Public API, but internal to orchestrator
esp_err_t telemetry_fetch_snapshot(telemetry_source_t src);  

#endif // IMPLUVIUM_PRIVATE_H