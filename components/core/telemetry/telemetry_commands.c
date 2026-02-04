/**
 * @file telemetry_commands.c
 * @brief TELEMETRY MQTT command handling - Remote configuration
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * MQTT command handling for TELEMETRY component:
 * - Parse MQTT interval commands (MessagePack format)
 * - Validate and apply configuration changes
 * - Publish acknowledgments with current config
 *
 * Command types:
 * 1. PRESET - Apply preset profile (Aggressive/Balanced/Conservative)
 *    Topic: solarium/config/intervals/preset
 *    Payload: {"preset": "Aggressive"|"Balanced"|"Conservative"}
 *
 * 2. SET - Set individual component intervals
 *    Topic: solarium/config/intervals/set
 *    Payload: {"component": "FLUCTUS"|"TEMPESTA"|"IMPLUVIUM", "intervals": {...}}
 *
 * All commands publish ACK to solarium/config/intervals/ack with full current config.
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "telemetry.h"
#include "telemetry_private.h"

#include "interval_config.h"
#include "msgpack.h"


static const char *TAG = "TELEMETRY_COMMANDS";


// ################ MQTT Command Handlers ################

/**
 * @brief Handle incoming MQTT interval configuration command
 *
 * Parses MessagePack payload, validates ranges, applies changes to interval_config
 * and components, and publishes acknowledgment with current configuration.
 */
esp_err_t telemetry_handle_interval_command(const char *topic, const uint8_t *data, size_t data_len)
{
    if (topic == NULL || data == NULL || data_len == 0) {
        ESP_LOGW(TAG, "Invalid interval command parameters");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;
    bool is_preset_cmd = (strncmp(topic, TELEMETRY_MQTT_TOPIC_INTERVAL_PRESET,
                                  strlen(TELEMETRY_MQTT_TOPIC_INTERVAL_PRESET)) == 0);
    bool is_set_cmd = (strncmp(topic, TELEMETRY_MQTT_TOPIC_INTERVAL_SET,
                               strlen(TELEMETRY_MQTT_TOPIC_INTERVAL_SET)) == 0);

    if (!is_preset_cmd && !is_set_cmd) {
        ESP_LOGW(TAG, "Unknown interval command topic: %s", topic);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Received interval command: %s (%zu bytes)",
             is_preset_cmd ? "PRESET" : "SET", data_len);

    // #################### Parse MessagePack Payload ####################

    msgpack_unpacked unpacked;
    msgpack_unpacked_init(&unpacked);

    size_t offset = 0;
    msgpack_unpack_return unpack_ret = msgpack_unpack_next(&unpacked, (const char*)data, data_len, &offset);

    if (unpack_ret != MSGPACK_UNPACK_SUCCESS) {
        ESP_LOGE(TAG, "Failed to unpack MessagePack payload (ret=%d)", unpack_ret);
        msgpack_unpacked_destroy(&unpacked);
        return ESP_FAIL;
    }

    msgpack_object root = unpacked.data;

    // #################### Handle PRESET Command ####################

    if (is_preset_cmd) {
        // Expect: {"preset": "Aggressive"|"Balanced"|"Conservative"}
        if (root.type != MSGPACK_OBJECT_MAP) {
            ESP_LOGE(TAG, "PRESET command: Expected map, got type %d", root.type);
            msgpack_unpacked_destroy(&unpacked);
            return ESP_FAIL;
        }

        const char *preset_str = NULL;
        msgpack_object_kv *kv = root.via.map.ptr;
        for (uint32_t i = 0; i < root.via.map.size; i++) {
            if (kv[i].key.type == MSGPACK_OBJECT_STR &&
                strncmp(kv[i].key.via.str.ptr, "preset", kv[i].key.via.str.size) == 0) {

                if (kv[i].val.type == MSGPACK_OBJECT_STR) {
                    preset_str = kv[i].val.via.str.ptr;
                    size_t preset_len = kv[i].val.via.str.size;

                    interval_preset_t preset = INTERVAL_PRESET_CUSTOM;
                    if (strncmp(preset_str, "Aggressive", preset_len) == 0) {
                        preset = INTERVAL_PRESET_AGGRESSIVE;
                    } else if (strncmp(preset_str, "Balanced", preset_len) == 0) {
                        preset = INTERVAL_PRESET_BALANCED;
                    } else if (strncmp(preset_str, "Conservative", preset_len) == 0) {
                        preset = INTERVAL_PRESET_CONSERVATIVE;
                    } else {
                        ESP_LOGE(TAG, "Unknown preset: %.*s", (int)preset_len, preset_str);
                        msgpack_unpacked_destroy(&unpacked);
                        return ESP_FAIL;
                    }

                    // Apply preset
                    ret = interval_config_apply_preset(preset);
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to apply preset %.*s", (int)preset_len, preset_str);
                        msgpack_unpacked_destroy(&unpacked);
                        return ESP_FAIL;
                    }

                    ESP_LOGI(TAG, "Applied preset: %s", interval_config_get_preset_name(preset));

                    // Update all components with new intervals
                    fluctus_set_power_intervals(g_interval_config.fluctus_power_day_min,
                                               g_interval_config.fluctus_power_night_min);
                    fluctus_set_solar_interval(g_interval_config.fluctus_solar_correction_min);
                    tempesta_set_collection_intervals(g_interval_config.tempesta_normal_min,
                                                     g_interval_config.tempesta_power_save_min);
                    impluvium_set_check_intervals(g_interval_config.impluvium_optimal_min,
                                                 g_interval_config.impluvium_cool_min,
                                                 g_interval_config.impluvium_power_save_min,
                                                 g_interval_config.impluvium_night_min_hours);

                    break;
                }
            }
        }

        if (preset_str == NULL) {
            ESP_LOGE(TAG, "PRESET command: Missing 'preset' field");
            msgpack_unpacked_destroy(&unpacked);
            return ESP_FAIL;
        }
    }

    // #################### Handle SET Command ####################

    else if (is_set_cmd) {
        // Expect: {"component": "FLUCTUS"|"TEMPESTA"|"IMPLUVIUM", "intervals": {...}}
        if (root.type != MSGPACK_OBJECT_MAP) {
            ESP_LOGE(TAG, "SET command: Expected map, got type %d", root.type);
            msgpack_unpacked_destroy(&unpacked);
            return ESP_FAIL;
        }

        const char *component = NULL;
        size_t component_len = 0;
        msgpack_object intervals_obj = {0};
        bool found_component = false;
        bool found_intervals = false;

        msgpack_object_kv *kv = root.via.map.ptr;
        for (uint32_t i = 0; i < root.via.map.size; i++) {
            if (kv[i].key.type == MSGPACK_OBJECT_STR) {
                if (strncmp(kv[i].key.via.str.ptr, "component", kv[i].key.via.str.size) == 0) {
                    if (kv[i].val.type == MSGPACK_OBJECT_STR) {
                        component = kv[i].val.via.str.ptr;
                        component_len = kv[i].val.via.str.size;
                        found_component = true;
                    }
                } else if (strncmp(kv[i].key.via.str.ptr, "intervals", kv[i].key.via.str.size) == 0) {
                    if (kv[i].val.type == MSGPACK_OBJECT_MAP) {
                        intervals_obj = kv[i].val;
                        found_intervals = true;
                    }
                }
            }
        }

        if (!found_component || !found_intervals) {
            ESP_LOGE(TAG, "SET command: Missing 'component' or 'intervals' field");
            msgpack_unpacked_destroy(&unpacked);
            return ESP_FAIL;
        }

        // ########## FLUCTUS ##########
        if (strncmp(component, "FLUCTUS", component_len) == 0) {
            uint32_t day_min = 0, night_min = 0, solar_min = 0;
            bool has_day = false, has_night = false, has_solar = false;

            msgpack_object_kv *int_kv = intervals_obj.via.map.ptr;
            for (uint32_t i = 0; i < intervals_obj.via.map.size; i++) {
                if (int_kv[i].key.type == MSGPACK_OBJECT_STR && int_kv[i].val.type == MSGPACK_OBJECT_POSITIVE_INTEGER) {
                    if (strncmp(int_kv[i].key.via.str.ptr, "power_day_min", int_kv[i].key.via.str.size) == 0) {
                        day_min = int_kv[i].val.via.u64;
                        has_day = true;
                    } else if (strncmp(int_kv[i].key.via.str.ptr, "power_night_min", int_kv[i].key.via.str.size) == 0) {
                        night_min = int_kv[i].val.via.u64;
                        has_night = true;
                    } else if (strncmp(int_kv[i].key.via.str.ptr, "solar_correction_min", int_kv[i].key.via.str.size) == 0) {
                        solar_min = int_kv[i].val.via.u64;
                        has_solar = true;
                    }
                }
            }

            if (has_day && has_night) {
                ret = interval_config_set_fluctus_power(day_min, night_min);
                if (ret == ESP_OK) {
                    fluctus_set_power_intervals(day_min, night_min);
                    ESP_LOGI(TAG, "FLUCTUS power intervals updated: day=%lu, night=%lu", day_min, night_min);
                } else {
                    ESP_LOGE(TAG, "FLUCTUS power interval validation failed");
                }
            }

            if (has_solar) {
                ret = interval_config_set_fluctus_solar(solar_min);
                if (ret == ESP_OK) {
                    fluctus_set_solar_interval(solar_min);
                    ESP_LOGI(TAG, "FLUCTUS solar interval updated: %lu min", solar_min);
                } else {
                    ESP_LOGE(TAG, "FLUCTUS solar interval validation failed");
                }
            }
        }

        // ########## TEMPESTA ##########
        else if (strncmp(component, "TEMPESTA", component_len) == 0) {
            uint32_t normal_min = 0, power_save_min = 0;
            bool has_normal = false, has_power_save = false;

            msgpack_object_kv *int_kv = intervals_obj.via.map.ptr;
            for (uint32_t i = 0; i < intervals_obj.via.map.size; i++) {
                if (int_kv[i].key.type == MSGPACK_OBJECT_STR && int_kv[i].val.type == MSGPACK_OBJECT_POSITIVE_INTEGER) {
                    if (strncmp(int_kv[i].key.via.str.ptr, "normal_min", int_kv[i].key.via.str.size) == 0) {
                        normal_min = int_kv[i].val.via.u64;
                        has_normal = true;
                    } else if (strncmp(int_kv[i].key.via.str.ptr, "power_save_min", int_kv[i].key.via.str.size) == 0) {
                        power_save_min = int_kv[i].val.via.u64;
                        has_power_save = true;
                    }
                }
            }

            if (has_normal && has_power_save) {
                ret = interval_config_set_tempesta(normal_min, power_save_min);
                if (ret == ESP_OK) {
                    tempesta_set_collection_intervals(normal_min, power_save_min);
                    ESP_LOGI(TAG, "TEMPESTA intervals updated: normal=%lu, power_save=%lu", normal_min, power_save_min);
                } else {
                    ESP_LOGE(TAG, "TEMPESTA interval validation failed");
                }
            } else {
                ESP_LOGE(TAG, "TEMPESTA: Missing normal_min or power_save_min");
                ret = ESP_FAIL;
            }
        }

        // ########## IMPLUVIUM ##########
        else if (strncmp(component, "IMPLUVIUM", component_len) == 0) {
            uint32_t optimal_min = 0, cool_min = 0, power_save_min = 0, night_hours = 0;
            bool has_optimal = false, has_cool = false, has_power_save = false, has_night = false;

            msgpack_object_kv *int_kv = intervals_obj.via.map.ptr;
            for (uint32_t i = 0; i < intervals_obj.via.map.size; i++) {
                if (int_kv[i].key.type == MSGPACK_OBJECT_STR && int_kv[i].val.type == MSGPACK_OBJECT_POSITIVE_INTEGER) {
                    if (strncmp(int_kv[i].key.via.str.ptr, "optimal_min", int_kv[i].key.via.str.size) == 0) {
                        optimal_min = int_kv[i].val.via.u64;
                        has_optimal = true;
                    } else if (strncmp(int_kv[i].key.via.str.ptr, "cool_min", int_kv[i].key.via.str.size) == 0) {
                        cool_min = int_kv[i].val.via.u64;
                        has_cool = true;
                    } else if (strncmp(int_kv[i].key.via.str.ptr, "power_save_min", int_kv[i].key.via.str.size) == 0) {
                        power_save_min = int_kv[i].val.via.u64;
                        has_power_save = true;
                    } else if (strncmp(int_kv[i].key.via.str.ptr, "night_min_hours", int_kv[i].key.via.str.size) == 0) {
                        night_hours = int_kv[i].val.via.u64;
                        has_night = true;
                    }
                }
            }

            if (has_optimal && has_cool && has_power_save && has_night) {
                ret = interval_config_set_impluvium(optimal_min, cool_min, power_save_min, night_hours);
                if (ret == ESP_OK) {
                    impluvium_set_check_intervals(optimal_min, cool_min, power_save_min, night_hours);
                    ESP_LOGI(TAG, "IMPLUVIUM intervals updated: optimal=%lu, cool=%lu, power_save=%lu, night=%luh",
                             optimal_min, cool_min, power_save_min, night_hours);
                } else {
                    ESP_LOGE(TAG, "IMPLUVIUM interval validation failed");
                }
            } else {
                ESP_LOGE(TAG, "IMPLUVIUM: Missing required interval fields");
                ret = ESP_FAIL;
            }
        }

        else {
            ESP_LOGE(TAG, "Unknown component: %.*s", (int)component_len, component);
            ret = ESP_FAIL;
        }
    }

    msgpack_unpacked_destroy(&unpacked);

    // #################### Publish Acknowledgment ####################

    if (mqtt_client == NULL) {
        ESP_LOGW(TAG, "MQTT client not initialized, skipping ACK");
        return ret;
    }

    // Pack acknowledgment with current configuration
    msgpack_sbuffer sbuf;
    msgpack_packer pk;
    msgpack_sbuffer_init(&sbuf);
    msgpack_packer_init(&pk, &sbuf, msgpack_sbuffer_write);

    // Build map: {status, message, preset, intervals}
    msgpack_pack_map(&pk, 4);

    // status: "ok" or "error"
    msgpack_pack_str(&pk, 6);
    msgpack_pack_str_body(&pk, "status", 6);
    const char *status = (ret == ESP_OK) ? "ok" : "error";
    msgpack_pack_str(&pk, strlen(status));
    msgpack_pack_str_body(&pk, status, strlen(status));

    // message: descriptive text
    msgpack_pack_str(&pk, 7);
    msgpack_pack_str_body(&pk, "message", 7);
    const char *msg = (ret == ESP_OK) ? "Configuration updated" : "Validation failed";
    msgpack_pack_str(&pk, strlen(msg));
    msgpack_pack_str_body(&pk, msg, strlen(msg));

    // preset: current preset name
    msgpack_pack_str(&pk, 6);
    msgpack_pack_str_body(&pk, "preset", 6);
    const char *preset_name = interval_config_get_preset_name(interval_config_get_current_preset());
    msgpack_pack_str(&pk, strlen(preset_name));
    msgpack_pack_str_body(&pk, preset_name, strlen(preset_name));

    // intervals: nested map with all current values
    msgpack_pack_str(&pk, 9);
    msgpack_pack_str_body(&pk, "intervals", 9);
    msgpack_pack_map(&pk, 3);  // FLUCTUS, TEMPESTA, IMPLUVIUM

    // FLUCTUS
    msgpack_pack_str(&pk, 7);
    msgpack_pack_str_body(&pk, "FLUCTUS", 7);
    msgpack_pack_map(&pk, 3);
    msgpack_pack_str(&pk, 13);
    msgpack_pack_str_body(&pk, "power_day_min", 13);
    msgpack_pack_uint32(&pk, g_interval_config.fluctus_power_day_min);
    msgpack_pack_str(&pk, 15);
    msgpack_pack_str_body(&pk, "power_night_min", 15);
    msgpack_pack_uint32(&pk, g_interval_config.fluctus_power_night_min);
    msgpack_pack_str(&pk, 20);
    msgpack_pack_str_body(&pk, "solar_correction_min", 20);
    msgpack_pack_uint32(&pk, g_interval_config.fluctus_solar_correction_min);

    // TEMPESTA
    msgpack_pack_str(&pk, 8);
    msgpack_pack_str_body(&pk, "TEMPESTA", 8);
    msgpack_pack_map(&pk, 2);
    msgpack_pack_str(&pk, 10);
    msgpack_pack_str_body(&pk, "normal_min", 10);
    msgpack_pack_uint32(&pk, g_interval_config.tempesta_normal_min);
    msgpack_pack_str(&pk, 14);
    msgpack_pack_str_body(&pk, "power_save_min", 14);
    msgpack_pack_uint32(&pk, g_interval_config.tempesta_power_save_min);

    // IMPLUVIUM
    msgpack_pack_str(&pk, 9);
    msgpack_pack_str_body(&pk, "IMPLUVIUM", 9);
    msgpack_pack_map(&pk, 4);
    msgpack_pack_str(&pk, 11);
    msgpack_pack_str_body(&pk, "optimal_min", 11);
    msgpack_pack_uint32(&pk, g_interval_config.impluvium_optimal_min);
    msgpack_pack_str(&pk, 8);
    msgpack_pack_str_body(&pk, "cool_min", 8);
    msgpack_pack_uint32(&pk, g_interval_config.impluvium_cool_min);
    msgpack_pack_str(&pk, 14);
    msgpack_pack_str_body(&pk, "power_save_min", 14);
    msgpack_pack_uint32(&pk, g_interval_config.impluvium_power_save_min);
    msgpack_pack_str(&pk, 15);
    msgpack_pack_str_body(&pk, "night_min_hours", 15);
    msgpack_pack_uint32(&pk, g_interval_config.impluvium_night_min_hours);

    // Publish ACK (QoS 0 - fire-and-forget for acknowledgments)
    int msg_id = esp_mqtt_client_publish(mqtt_client, TELEMETRY_MQTT_TOPIC_INTERVAL_ACK,
                                         sbuf.data, sbuf.size, 0, 0);

    if (msg_id >= 0) {
        ESP_LOGI(TAG, "Published interval ACK: status=%s, preset=%s", status, preset_name);
    } else {
        ESP_LOGW(TAG, "Failed to publish interval ACK (msg_id=%d)", msg_id);
    }

    msgpack_sbuffer_destroy(&sbuf);

    return ret;
}

/**
 * @brief Handle zone configuration command from MQTT
 *
 * Expected MessagePack format:
 * {
 *   "zone_id": <uint8>,                        // 0-4
 *   "enabled": <bool>,
 *   "target_moisture": <float>,                // 20.0-80.0 %
 *   "deadband": <float>,                       // 1.0-20.0 %
 *   "target_moisture_gain_rate": <float>       // 0.1-3.0 %/sec
 * }
 *
 * @param[in] data Raw MessagePack binary data
 * @param[in] data_len Length of data in bytes
 * @return ESP_OK on success, ESP_FAIL on parse/validation error
 */
esp_err_t telemetry_handle_zone_config_command(const uint8_t *data, size_t data_len)
{
    esp_err_t ret = ESP_OK;

    // Parse MessagePack
    msgpack_unpacked unpacked;
    msgpack_unpacked_init(&unpacked);

    size_t offset = 0;
    msgpack_unpack_return unpack_ret = msgpack_unpack_next(&unpacked, (const char*)data, data_len, &offset);

    if (unpack_ret != MSGPACK_UNPACK_SUCCESS) {
        ESP_LOGE(TAG, "Failed to unpack zone config MessagePack payload (ret=%d)", unpack_ret);
        msgpack_unpacked_destroy(&unpacked);
        return ESP_FAIL;
    }

    msgpack_object root = unpacked.data;

    if (root.type != MSGPACK_OBJECT_MAP) {
        ESP_LOGE(TAG, "Expected map, got type %d", root.type);
        msgpack_unpacked_destroy(&unpacked);
        return ESP_FAIL;
    }

    // Extract fields
    uint8_t zone_id = 0;
    bool enabled = false;
    float target_moisture = 0.0f;
    float deadband = 0.0f;
    float target_moisture_gain_rate = 1.0f;

    bool zone_id_found = false;
    bool enabled_found = false;
    bool target_found = false;
    bool deadband_found = false;
    bool gain_rate_found = false;

    msgpack_object_kv *kv = root.via.map.ptr;
    for (uint32_t i = 0; i < root.via.map.size; i++) {
        if (kv[i].key.type != MSGPACK_OBJECT_STR) continue;

        const char *key = kv[i].key.via.str.ptr;
        size_t key_len = kv[i].key.via.str.size;

        if (strncmp(key, "zone_id", key_len) == 0 && kv[i].val.type == MSGPACK_OBJECT_POSITIVE_INTEGER) {
            zone_id = (uint8_t)kv[i].val.via.u64;
            zone_id_found = true;
        }
        else if (strncmp(key, "enabled", key_len) == 0 && kv[i].val.type == MSGPACK_OBJECT_BOOLEAN) {
            enabled = kv[i].val.via.boolean;
            enabled_found = true;
        }
        else if (strncmp(key, "target_moisture", key_len) == 0) {
            if (kv[i].val.type == MSGPACK_OBJECT_FLOAT32 || kv[i].val.type == MSGPACK_OBJECT_FLOAT64) {
                target_moisture = (float)kv[i].val.via.f64;
                target_found = true;
            }
            else if (kv[i].val.type == MSGPACK_OBJECT_POSITIVE_INTEGER) {
                target_moisture = (float)kv[i].val.via.u64;
                target_found = true;
            }
        }
        else if (strncmp(key, "deadband", key_len) == 0) {
            if (kv[i].val.type == MSGPACK_OBJECT_FLOAT32 || kv[i].val.type == MSGPACK_OBJECT_FLOAT64) {
                deadband = (float)kv[i].val.via.f64;
                deadband_found = true;
            }
            else if (kv[i].val.type == MSGPACK_OBJECT_POSITIVE_INTEGER) {
                deadband = (float)kv[i].val.via.u64;
                deadband_found = true;
            }
        }
        else if (strncmp(key, "target_moisture_gain_rate", key_len) == 0) {
            if (kv[i].val.type == MSGPACK_OBJECT_FLOAT32 || kv[i].val.type == MSGPACK_OBJECT_FLOAT64) {
                target_moisture_gain_rate = (float)kv[i].val.via.f64;
                gain_rate_found = true;
            }
            else if (kv[i].val.type == MSGPACK_OBJECT_POSITIVE_INTEGER) {
                target_moisture_gain_rate = (float)kv[i].val.via.u64;
                gain_rate_found = true;
            }
        }
    }

    msgpack_unpacked_destroy(&unpacked);

    // Validate required fields
    if (!zone_id_found || !enabled_found || !target_found || !deadband_found || !gain_rate_found) {
        ESP_LOGE(TAG, "Missing required fields (zone_id=%d, enabled=%d, target=%d, deadband=%d, gain_rate=%d)",
                 zone_id_found, enabled_found, target_found, deadband_found, gain_rate_found);
        ret = ESP_FAIL;
    }

    // Validate ranges
    if (zone_id >= IRRIGATION_ZONE_COUNT) {
        ESP_LOGE(TAG, "Invalid zone_id %d (must be 0-%d)", zone_id, IRRIGATION_ZONE_COUNT - 1);
        ret = ESP_FAIL;
    }

    if (target_moisture < 20.0f || target_moisture > 80.0f) {
        ESP_LOGE(TAG, "Invalid target_moisture %.1f%% (must be 20-80)", target_moisture);
        ret = ESP_FAIL;
    }

    if (deadband < 1.0f || deadband > 20.0f) {
        ESP_LOGE(TAG, "Invalid deadband %.1f%% (must be 1-20)", deadband);
        ret = ESP_FAIL;
    }

    if (target_moisture_gain_rate < 0.1f || target_moisture_gain_rate > 3.0f) {
        ESP_LOGE(TAG, "Invalid target_moisture_gain_rate %.2f%%/s (must be 0.1-3.0)", target_moisture_gain_rate);
        ret = ESP_FAIL;
    }

    // Apply configuration if validation passed
    if (ret == ESP_OK) {
        ret = impluvium_update_zone_config(zone_id, target_moisture, deadband, target_moisture_gain_rate, enabled);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Zone %d config updated: enabled=%d, target=%.1f%%, deadband=%.1f%%, gain_rate=%.2f%%/s",
                     zone_id, enabled, target_moisture, deadband, target_moisture_gain_rate);

            // Force telemetry cache update
            telemetry_fetch_snapshot(TELEMETRY_SRC_IMPLUVIUM);
        } else {
            ESP_LOGE(TAG, "Failed to apply zone %d config", zone_id);
        }
    }

    // Publish ACK
    extern esp_mqtt_client_handle_t mqtt_client;
    if (!mqtt_client) {
        ESP_LOGW(TAG, "MQTT client not initialized, cannot publish zone ACK");
        return ret;
    }

    // Build ACK message
    msgpack_sbuffer sbuf;
    msgpack_sbuffer_init(&sbuf);

    msgpack_packer pk;
    msgpack_packer_init(&pk, &sbuf, msgpack_sbuffer_write);

    // Root map with 3 fields: status, zone_id, config
    msgpack_pack_map(&pk, 3);

    // Field 1: status
    msgpack_pack_str(&pk, 6);
    msgpack_pack_str_body(&pk, "status", 6);
    const char *status = (ret == ESP_OK) ? "ok" : "error";
    msgpack_pack_str(&pk, strlen(status));
    msgpack_pack_str_body(&pk, status, strlen(status));

    // Field 2: zone_id
    msgpack_pack_str(&pk, 7);
    msgpack_pack_str_body(&pk, "zone_id", 7);
    msgpack_pack_uint8(&pk, zone_id);

    // Field 3: config (nested map)
    msgpack_pack_str(&pk, 6);
    msgpack_pack_str_body(&pk, "config", 6);
    msgpack_pack_map(&pk, 5);  // 5 config fields

    // config.enabled
    msgpack_pack_str(&pk, 7);
    msgpack_pack_str_body(&pk, "enabled", 7);
    if (enabled) {
        msgpack_pack_true(&pk);
    } else {
        msgpack_pack_false(&pk);
    }

    // config.target_moisture
    msgpack_pack_str(&pk, 15);
    msgpack_pack_str_body(&pk, "target_moisture", 15);
    msgpack_pack_float(&pk, target_moisture);

    // config.deadband
    msgpack_pack_str(&pk, 8);
    msgpack_pack_str_body(&pk, "deadband", 8);
    msgpack_pack_float(&pk, deadband);

    // config.target_moisture_gain_rate
    msgpack_pack_str(&pk, 25);
    msgpack_pack_str_body(&pk, "target_moisture_gain_rate", 25);
    msgpack_pack_float(&pk, target_moisture_gain_rate);

    // config.zone_id (redundant, but useful for clients)
    msgpack_pack_str(&pk, 7);
    msgpack_pack_str_body(&pk, "zone_id", 7);
    msgpack_pack_uint8(&pk, zone_id);

    // Publish ACK (QoS 0)
    int msg_id = esp_mqtt_client_publish(mqtt_client, TELEMETRY_MQTT_TOPIC_ZONE_ACK,
                                         sbuf.data, sbuf.size, 0, 0);

    if (msg_id >= 0) {
        ESP_LOGI(TAG, "Published zone ACK: status=%s, zone_id=%d", status, zone_id);
    } else {
        ESP_LOGW(TAG, "Failed to publish zone ACK (msg_id=%d)", msg_id);
    }

    msgpack_sbuffer_destroy(&sbuf);

    return ret;
}