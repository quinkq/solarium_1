/**
 * @file telemetry_mqtt_client.c
 * @brief TELEMETRY MQTT client - Publishing orchestrator
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Central orchestrator for complete TELEMETRY pipeline:
 * - MQTT client lifecycle (init, connection, events)
 * - Component data ingestion (telemetry_fetch_snapshot)
 * - Main publish task (encode/enqueue + dequeue/publish loops)
 * - Event handler (CONNECTED, DATA, PUBLISHED, ERROR)
 *
 * Publishing task has TWO responsibilities:
 * A. Data Ingestion (when notified by components):
 *    - Lock cache → Encode msgpack → Unlock cache
 *    - Check realtime/publishing flags → Enqueue to PSRAM
 *    - Check for FLASH backup needed (95% threshold)
 *
 * B. MQTT Publishing (continuous loop):
 *    - Peek message from buffer → Publish (QoS 1)
 *    - Track in-flight message → Wait for PUBACK
 *    - Dequeue on success → Retry/FLASH backup on failure
 *
 * Hybrid retry logic: Per-message lifetime (10 retries) + session failures (5 consecutive)
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "telemetry.h"
#include "telemetry_private.h"


static const char *TAG = "TELEMETRY_MQTT_CLIENT";

// ################ MQTT Infrastructure ################

/**
 * @brief Initialize MQTT client and start publishing task
 * Creates client, registers event handler, starts connection
 */
esp_err_t telemetry_mqtt_client_init(void) 
{
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
        ESP_LOGE(TAG, "Failed to initialize MQTT client (inner)");
        return ESP_FAIL;
    }

    esp_err_t ret = esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, telemetry_mqtt_event_handler, NULL);
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

    BaseType_t task_ret = xTaskCreate(telemetry_mqtt_publish_task, "mqtt_publish", 4096, NULL, 5, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create MQTT publishing task");
        esp_mqtt_client_stop(mqtt_client);
        esp_mqtt_client_destroy(mqtt_client);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief MQTT event handler - Handles connection, disconnection, PUBACK, commands
 * Processes QoS 1 acknowledgments, interval commands, realtime mode control
 */
void telemetry_mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            mqtt_connected = true;

            esp_mqtt_client_publish(mqtt_client, "solarium/status", "online", 6, 1, 1);
            esp_mqtt_client_subscribe(mqtt_client, "solarium/cmd/realtime", 1);
            esp_mqtt_client_subscribe(mqtt_client, TELEMETRY_MQTT_TOPIC_INTERVAL_SET, 1);
            esp_mqtt_client_subscribe(mqtt_client, TELEMETRY_MQTT_TOPIC_INTERVAL_PRESET, 1);
            esp_mqtt_client_subscribe(mqtt_client, TELEMETRY_MQTT_TOPIC_ZONE_SET, 1);

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
                telemetry_buffer_dequeue();
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
            // Handle interval configuration commands
            else if (strncmp(event->topic, TELEMETRY_MQTT_TOPIC_INTERVAL_SET,
                            strlen(TELEMETRY_MQTT_TOPIC_INTERVAL_SET)) == 0 ||
                     strncmp(event->topic, TELEMETRY_MQTT_TOPIC_INTERVAL_PRESET,
                            strlen(TELEMETRY_MQTT_TOPIC_INTERVAL_PRESET)) == 0) {

                esp_err_t ret = telemetry_handle_interval_command(event->topic,
                                                                   (const uint8_t*)event->data,
                                                                   event->data_len);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to process interval command from topic: %.*s",
                             event->topic_len, event->topic);
                }
            }
            // Handle zone configuration commands
            else if (strncmp(event->topic, TELEMETRY_MQTT_TOPIC_ZONE_SET,
                            strlen(TELEMETRY_MQTT_TOPIC_ZONE_SET)) == 0) {

                esp_err_t ret = telemetry_handle_zone_config_command((const uint8_t*)event->data,
                                                                      event->data_len);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to process zone config command from topic: %.*s",
                             event->topic_len, event->topic);
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

/**
 * @brief Convert telemetry source ID to MQTT topic name
 * Handles special case for IMPLUVIUM zone messages (component_id 100-104)
 */
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

/**
 * @brief Main MQTT publishing task - Dual responsibility orchestrator
 * A. Data ingestion: Wait for component notifications → Lock cache → Encode → Enqueue
 * B. Publishing loop: Peek buffer → Publish (QoS 1) → Wait PUBACK → Dequeue
 * Implements hybrid retry logic: per-message lifetime (10) + session failures (5)
 */
void telemetry_mqtt_publish_task(void *parameters)
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
                    encode_ret = telemetry_msgpack_encode_fluctus((fluctus_snapshot_t*)cache_ptr,
                                                       msgpack_buffer, &payload_len,
                                                       BUFFER_PAYLOAD_SIZE);
                    break;
                case TELEMETRY_SRC_FLUCTUS_RT:
                    // Encodes RT subset from unified structure
                    encode_ret = telemetry_msgpack_encode_fluctus_rt((fluctus_snapshot_t*)cache_ptr,
                                                          msgpack_buffer, &payload_len,
                                                          BUFFER_PAYLOAD_SIZE);
                    break;
                case TELEMETRY_SRC_TEMPESTA:
                    encode_ret = telemetry_msgpack_encode_tempesta((tempesta_snapshot_t*)cache_ptr,
                                                        msgpack_buffer, &payload_len,
                                                        BUFFER_PAYLOAD_SIZE);
                    break;
                case TELEMETRY_SRC_IMPLUVIUM:
                    // IMPLUVIUM split encoding: 1 system + 5 zone messages
                    // Encode system message first
                    encode_ret = telemetry_msgpack_encode_impluvium_system((impluvium_snapshot_t*)cache_ptr,
                                                                msgpack_buffer, &payload_len,
                                                                BUFFER_PAYLOAD_SIZE);

                    if (encode_ret == ESP_OK) {
                        // Enqueue system message (component_id = TELEMETRY_SRC_IMPLUVIUM = 3)
                        telemetry_buffer_enqueue(msgpack_buffer, payload_len, TELEMETRY_SRC_IMPLUVIUM, 0);

                        // Encode and enqueue 5 zone messages
                        // Encode zone in component_id: (IMPLUVIUM_BASE + zone_id)
                        // Zone 0-4 = component_id 100-104 (decoded in get_topic_name)
                        for (uint8_t zone = 0; zone < 5; zone++) {
                            payload_len = 0;
                            encode_ret = telemetry_msgpack_encode_impluvium_zone((impluvium_snapshot_t*)cache_ptr,
                                                                      zone,
                                                                      msgpack_buffer, &payload_len,
                                                                      BUFFER_PAYLOAD_SIZE);
                            if (encode_ret == ESP_OK) {
                                // Encode zone in component_id: 100 + zone (100-104)
                                uint8_t zone_component_id = 100 + zone;
                                telemetry_buffer_enqueue(msgpack_buffer, payload_len,
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
                    encode_ret = telemetry_msgpack_encode_impluvium_rt((impluvium_snapshot_rt_t*)cache_ptr,
                                                            msgpack_buffer, &payload_len,
                                                            BUFFER_PAYLOAD_SIZE);
                    break;
                case TELEMETRY_SRC_STELLARIA:
                    encode_ret = telemetry_msgpack_encode_stellaria((stellaria_snapshot_t*)cache_ptr,
                                                         msgpack_buffer, &payload_len,
                                                         BUFFER_PAYLOAD_SIZE);
                    break;
                case TELEMETRY_SRC_WIFI:
                    encode_ret = telemetry_msgpack_encode_wifi((wifi_snapshot_t*)cache_ptr,
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

            if (telemetry_buffer_enqueue(msgpack_buffer, payload_len, src, message_type) == ESP_OK) {
                ESP_LOGI(TAG, "Enqueued %s: %d bytes (buffer: %d/%d)",
                        get_topic_name(src), payload_len,
                        telemetry_buffer_get_count(), BUFFER_CAPACITY);
            }
        }

        // FLASH backup: When buffer critical, flush to FLASH
        // Triggers on EITHER disconnection OR stalled connection (buffer near full)
        uint16_t count = telemetry_buffer_get_count();
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

            for (uint16_t i = 0; i < FLASH_BATCH_SIZE && telemetry_buffer_peek(&flush_slot, NULL) == ESP_OK; i++) {
                if (telemetry_flash_backup_write_slot(&flush_slot) == ESP_OK) {
                    telemetry_buffer_dequeue();
                    flushed++;
                } else {
                    ESP_LOGE(TAG, "FLASH write failed, stopping flush");
                    break;
                }
            }

            if (flushed > 0) {
                ESP_LOGI(TAG, "Flushed %d messages to FLASH (buffer: %d/%d)",
                        flushed, telemetry_buffer_get_count(), BUFFER_CAPACITY);
            }
        }

        buffer_slot_t slot;
        uint16_t current_tail_index;

        while (telemetry_buffer_peek(&slot, &current_tail_index) == ESP_OK) {
            if (!mqtt_connected) {
                ESP_LOGD(TAG, "MQTT not connected, buffering messages (%d queued)",
                        telemetry_buffer_get_count());
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
                         slot.global_seq, telemetry_buffer_get_count());
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

                        telemetry_buffer_dequeue(); // Drop the message by advancing the tail

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
                    telemetry_buffer_dequeue();
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
