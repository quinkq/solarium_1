/**
 * @file telemetry_msgpack.c
 * @brief TELEMETRY MessagePack encoders - Binary serialization
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * MessagePack binary serialization for all TELEMETRY sources:
 * - 7 encoder functions (FLUCTUS, FLUCTUS_RT, TEMPESTA, IMPLUVIUM system/zone, IMPLUVIUM_RT, STELLARIA, WiFi)
 * - All follow same pattern: snapshot struct → msgpack binary
 * - Compact encoding: ~80-250 bytes per message (fits in 256-byte slot)
 *
 * Encoder breakdown:
 * - FLUCTUS: 15-min averages + energy stats + system state (~180 bytes)
 * - FLUCTUS_RT: Instantaneous power + bus states + debug fields (~200 bytes)
 * - TEMPESTA: 8 sensors + status flags (~220 bytes)
 * - IMPLUVIUM: Split into 1 system + 5 zone messages (~100-120 bytes each)
 * - IMPLUVIUM_RT: Fast-changing sensors during watering (~140 bytes)
 * - STELLARIA: Lighting state (~60 bytes)
 * - WiFi: Connection status (~80 bytes)
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "telemetry.h"
#include "telemetry_private.h"

#include "msgpack.h"


static const char *TAG = "TELEMETRY_MSGPACK";


// ################ MessagePack Encoders ################

/**
 * @brief Encode FLUCTUS normal snapshot to MessagePack format
 * Normal snapshot: 15-min averages + energy statistics + system state
 */
esp_err_t telemetry_msgpack_encode_fluctus(const fluctus_snapshot_t *data,
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
esp_err_t telemetry_msgpack_encode_fluctus_rt(const fluctus_snapshot_t *data,
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

/**
 * @brief Encode STELLARIA snapshot to MessagePack format
 * Lighting state: intensity, driver enabled, auto mode
 */
esp_err_t telemetry_msgpack_encode_stellaria(const stellaria_snapshot_t *data,
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

/**
 * @brief Encode TEMPESTA snapshot to MessagePack format
 * Weather data: 8 sensors (temp/humidity/pressure/air quality/wind/rain/tank) + status flags
 */
esp_err_t telemetry_msgpack_encode_tempesta(const tempesta_snapshot_t *data,
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
esp_err_t telemetry_msgpack_encode_impluvium_system(const impluvium_snapshot_t *data,
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
esp_err_t telemetry_msgpack_encode_impluvium_zone(const impluvium_snapshot_t *data,
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
esp_err_t telemetry_msgpack_encode_impluvium_rt(const impluvium_snapshot_rt_t *data,
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

/**
 * @brief Encode WiFi snapshot to MessagePack format
 * Connection status: state, RSSI, reconnect count, IP address, power save mode
 */
esp_err_t telemetry_msgpack_encode_wifi(const wifi_snapshot_t *data,
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