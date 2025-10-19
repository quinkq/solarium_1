#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <time.h>
#include <stdbool.h>
#include "esp_err.h"

// ########################## Configuration ##########################

// Storage configuration
#define TELEMETRY_ROLLING_BUFFER_HOURS    72      // 3 days of hourly data
#define TELEMETRY_LITTLEFS_MOUNT_POINT    "/data"
#define TELEMETRY_POWER_DATA_FILE          "/data/power.dat"

// MQTT configuration (placeholders)
#define TELEMETRY_MQTT_BROKER_URI     "mqtt://192.168.1.100:1883"
#define TELEMETRY_MQTT_USERNAME       "solarium"
#define TELEMETRY_MQTT_PASSWORD       "password"
#define TELEMETRY_MQTT_CLIENT_ID      "solarium_esp32"
#define TELEMETRY_MQTT_TOPIC_HOURLY   "solarium/power/hourly"
#define TELEMETRY_MQTT_TOPIC_DAILY    "solarium/power/daily"

// Power metering configuration
#define TELEMETRY_ACTIVE_SAMPLE_INTERVAL_MS    500    // Active monitoring: 500ms
#define TELEMETRY_STEADY_SAMPLE_INTERVAL_MS    60000  // Steady state: 60s
#define TELEMETRY_STEADY_PROBE_DURATION_MS     3000   // Probe for 3 seconds

// ########################## Data Structures ##########################

/**
 * @brief Hourly power telemetry data
 * Size: 68 bytes per hour, 4896 bytes for 72 hours (3 days)
 */
typedef struct {
    time_t timestamp_utc;              // Hour start time (UTC) - 4 bytes

    // PV Panel metrics (28 bytes)
    float pv_energy_wh;                // Total energy generated this hour
    float pv_peak_power_w;             // Peak power
    float pv_avg_power_w;              // Average power
    float pv_max_voltage_v;            // Maximum voltage
    float pv_avg_voltage_v;            // Average voltage
    float pv_max_current_a;            // Maximum current
    float pv_avg_current_a;            // Average current

    // Battery consumption metrics (32 bytes)
    float battery_consumed_wh;         // Total energy consumed this hour
    float battery_peak_power_w;        // Peak consumption power
    float battery_avg_power_w;         // Average consumption power
    float battery_voltage_v;           // Latest voltage reading (end of hour)
    float battery_min_voltage_v;       // Minimum voltage
    float battery_max_voltage_v;       // Maximum voltage
    float battery_max_current_a;       // Maximum current
    float battery_avg_current_a;       // Average current

    // Metadata (4 bytes)
    uint16_t pv_sample_count;          // Number of PV samples taken
    uint16_t battery_sample_count;     // Number of battery samples taken
} power_telemetry_hour_t;

/**
 * @brief Daily power summary
 */
typedef struct {
    time_t day_start_utc;              // Day start timestamp (UTC)
    float pv_total_energy_wh;          // Total PV energy for the day
    float battery_total_consumed_wh;   // Total battery consumption for the day
    float pv_peak_power_w;             // Peak PV power for the day
    float battery_peak_power_w;        // Peak battery consumption for the day
    uint16_t hours_collected;          // Number of hours with data
} power_telemetry_day_t;

/**
 * @brief Power metering state
 */
typedef enum {
    POWER_METER_STATE_ACTIVE = 0,      // Active monitoring (500ms sampling)
    POWER_METER_STATE_STEADY,          // Steady state (60s sampling with 3s probes)
    POWER_METER_STATE_PV_SHUTDOWN,     // PV INA in power-down mode (nighttime)
    POWER_METER_STATE_BATTERY_SHUTDOWN // Battery INA in power-down mode
} power_meter_state_t;

/**
 * @brief Live power accumulator (stored in RTC RAM)
 * This structure survives deep sleep but not power loss
 */
typedef struct {
    time_t current_hour_start;         // Start time of current hour
    float pv_energy_wh_accumulator;    // Accumulated PV energy (Wh)
    float battery_energy_wh_accumulator; // Accumulated battery energy (Wh)
    time_t last_sample_time;           // Last sample timestamp
    bool initialized;                  // Accumulator initialization flag
} power_accumulator_rtc_t;

// ########################## Public API Functions ##########################

/**
 * @brief Initialize telemetry subsystem
 * Initializes LittleFS, RTC RAM accumulators, and MQTT (placeholder)
 * @return ESP_OK on success
 */
esp_err_t telemetry_init(void);

/**
 * @brief Update power metering accumulation
 * Called periodically by FLUCTUS monitoring task
 *
 * @param pv_voltage PV panel voltage (V)
 * @param pv_current PV panel current (A)
 * @param pv_power PV panel power (W)
 * @param battery_voltage Battery voltage (V)
 * @param battery_current Battery current (A)
 * @param battery_power Battery power (W)
 * @param is_pv_active Is PV INA active (not in shutdown)?
 * @param is_battery_active Is Battery INA active (not in shutdown)?
 * @return ESP_OK on success
 */
esp_err_t telemetry_update_power_sample(
    float pv_voltage, float pv_current, float pv_power,
    float battery_voltage, float battery_current, float battery_power,
    bool is_pv_active, bool is_battery_active
);

/**
 * @brief Check if PV INA should be active (daytime detection)
 * Uses solar calculations + photoresistor verification
 *
 * @param avg_light_level Average photoresistor reading (V)
 * @return true if PV should be active (daytime)
 */
bool telemetry_should_pv_be_active(float avg_light_level);

/**
 * @brief Get recommended power meter state based on system activity
 *
 * @param any_bus_active Is any power bus active?
 * @param stellaria_only Is only Stellaria active (12V bus)?
 * @return Recommended power_meter_state_t
 */
power_meter_state_t telemetry_get_recommended_state(bool any_bus_active, bool stellaria_only);

/**
 * @brief Get current hourly telemetry data
 * @param[out] data Pointer to power_telemetry_hour_t to fill
 * @return ESP_OK on success
 */
esp_err_t telemetry_get_current_hour_data(power_telemetry_hour_t *data);

/**
 * @brief Get daily summary
 * @param[out] summary Pointer to power_telemetry_day_t to fill
 * @return ESP_OK on success
 */
esp_err_t telemetry_get_daily_summary(power_telemetry_day_t *summary);

/**
 * @brief Get historical hourly data
 * @param hours_ago How many hours ago (0 = current hour, 1 = last hour, etc.)
 * @param[out] data Pointer to power_telemetry_hour_t to fill
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if data not available
 */
esp_err_t telemetry_get_historical_hour(uint16_t hours_ago, power_telemetry_hour_t *data);

/**
 * @brief Export current day's data via MQTT
 * @note This is a placeholder - MQTT functionality to be implemented
 * @return ESP_OK on success
 */
esp_err_t telemetry_mqtt_publish_daily(void);

/**
 * @brief Export latest hourly data via MQTT
 * @note This is a placeholder - MQTT functionality to be implemented
 * @return ESP_OK on success
 */
esp_err_t telemetry_mqtt_publish_hourly(void);

/**
 * @brief Print current power metrics to serial console
 */
void telemetry_print_power_status(void);

/**
 * @brief Print daily summary to serial console
 */
void telemetry_print_daily_summary(void);

// ################ Component Data Hub (Central Storage) ################

/**
 * @brief STELLARIA data structure for TELEMETRY
 * Cached data from STELLARIA component for HMI/MQTT distribution
 */
typedef struct {
    // State
    int state;                      // stellaria_state_t (avoid circular dependency)
    uint16_t current_intensity;     // Current PWM intensity (0-1023)
    uint16_t target_intensity;      // Target intensity requested
    bool driver_enabled;            // Driver enable state
    bool initialized;               // Component initialization status
    bool auto_mode_active;          // Auto light sensing mode active
    float last_light_reading;       // Last averaged light reading (V)

    // Metadata
    time_t last_update_time;        // Last update timestamp
    bool data_valid;                // Data validity flag
} telemetry_stellaria_t;

/**
 * @brief FLUCTUS data structure for TELEMETRY
 * Cached data from FLUCTUS component for HMI/MQTT distribution
 */
typedef struct {
    // Power buses state
    bool bus_3v3_enabled;
    bool bus_5v_enabled;
    bool bus_6v6_enabled;
    bool bus_12v_enabled;
    uint8_t bus_3v3_consumers;
    uint8_t bus_5v_consumers;
    uint8_t bus_6v6_consumers;
    uint8_t bus_12v_consumers;

    // Battery monitoring
    float battery_voltage;
    float battery_current;
    float battery_soc_percent;      // Calculated SOC based on voltage
    float battery_power_w;

    // Solar monitoring
    float solar_voltage;
    float solar_current;
    float solar_power_w;
    bool solar_pv_active;

    // Solar tracking
    int tracking_state;             // solar_tracking_state_t (avoid circular dependency)
    uint8_t yaw_position_percent;   // 0-100%
    uint8_t pitch_position_percent; // 0-100%
    float yaw_error;                // Tracking error in volts
    float pitch_error;

    // Thermal management
    float case_temperature;
    bool temperature_valid;
    uint8_t fan_speed_percent;

    // System state
    int power_state;                // fluctus_power_state_t (avoid circular dependency)
    bool safety_shutdown;
    bool manual_reset_required;
    time_t last_activity_time;

    // Readings validity
    bool battery_data_valid;
    bool solar_data_valid;

    // Metadata
    time_t last_update_time;        // Last update timestamp
    bool data_valid;                // Data validity flag
} telemetry_fluctus_t;

/**
 * @brief TEMPESTA data structure for TELEMETRY
 * Cached data from TEMPESTA (weather_station) component for HMI/MQTT distribution
 */
typedef struct {
    // Environmental readings
    float temperature;              // Averaged temperature (°C)
    float humidity;                 // Current humidity (%)
    float pressure;                 // Current pressure (hPa)

    // Air quality
    float air_quality_pm25;         // PM2.5 (μg/m³)
    float air_quality_pm10;         // PM10 (μg/m³)

    // Wind and rain
    float wind_speed_rpm;           // Wind speed (RPM)
    float wind_speed_ms;            // Wind speed (m/s)
    float rainfall_mm;              // Accumulated rainfall (mm)

    // Sensor status (int to avoid circular dependency)
    int temp_sensor_status;
    int humidity_sensor_status;
    int pressure_sensor_status;
    int air_quality_status;
    int wind_sensor_status;
    int rain_sensor_status;

    // Metadata
    time_t last_update_time;        // Last update timestamp
    bool data_valid;                // Data validity flag
} telemetry_tempesta_t;

/**
 * @brief IMPLUVIUM data structure for TELEMETRY
 * Cached data from IMPLUVIUM (irrigation) component for HMI/MQTT distribution
 * Simplified from full impluvium_snapshot_t for display purposes
 */
typedef struct {
    // System state
    int state;                      // irrigation_state_t (avoid circular dependency)
    bool system_initialized;
    bool monitoring_active;

    // Water tank
    float water_level_percent;
    float tank_pressure_kpa;

    // System status
    uint8_t active_zone_id;
    uint16_t pump_pwm_duty;
    uint16_t current_flow_rate_mlpm;

    // Daily summary
    float total_volume_used_today_ml;

    // Per-zone essentials (simplified - just current moisture)
    struct {
        float current_moisture_percent;
        bool watering_enabled;
        time_t last_watered_time;
    } zones[4];  // IRRIGATION_ZONE_COUNT

    // Anomaly state
    int current_anomaly_type;       // anomaly_type_t (avoid circular dependency)

    // Metadata
    time_t last_update_time;        // Last update timestamp
    bool data_valid;                // Data validity flag
} telemetry_impluvium_t;

// ################ Component Update Functions (Injection Points) ################

/**
 * @brief Update STELLARIA data in TELEMETRY cache
 * Called by STELLARIA component when state changes (in ramp task)
 * TELEMETRY internally calls stellaria_get_data_snapshot()
 *
 * @return ESP_OK on success
 */
esp_err_t telemetry_update_stellaria(void);

/**
 * @brief Update FLUCTUS state data in TELEMETRY cache
 * Called by FLUCTUS component at periodic intervals (15min when idle)
 * TELEMETRY internally calls fluctus_get_data_snapshot()
 *
 * @return ESP_OK on success
 */
esp_err_t telemetry_update_fluctus_state(void);

/**
 * @brief Update TEMPESTA data in TELEMETRY cache
 * Called by TEMPESTA component at end of measurement cycle (15min/60min variable)
 * TELEMETRY internally calls tempesta_get_data_snapshot()
 *
 * @return ESP_OK on success
 */
esp_err_t telemetry_update_tempesta(void);

/**
 * @brief Update IMPLUVIUM data in TELEMETRY cache
 * Called by IMPLUVIUM component at state changes and after measurements
 * TELEMETRY internally calls impluvium_get_data_snapshot()
 *
 * @return ESP_OK on success
 */
esp_err_t telemetry_update_impluvium(void);

// ################ Component Get Functions (HMI/MQTT Retrieval) ################

/**
 * @brief Get cached STELLARIA data for HMI display
 * Thread-safe retrieval from TELEMETRY cache
 *
 * @param[out] data Pointer to telemetry_stellaria_t structure to fill
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t telemetry_get_stellaria_data(telemetry_stellaria_t *data);

/**
 * @brief Get cached FLUCTUS data for HMI display
 * Thread-safe retrieval from TELEMETRY cache
 *
 * @param[out] data Pointer to telemetry_fluctus_t structure to fill
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t telemetry_get_fluctus_data(telemetry_fluctus_t *data);

/**
 * @brief Get cached TEMPESTA data for HMI display
 * Thread-safe retrieval from TELEMETRY cache
 *
 * @param[out] data Pointer to telemetry_tempesta_t structure to fill
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t telemetry_get_tempesta_data(telemetry_tempesta_t *data);

/**
 * @brief Get cached IMPLUVIUM data for HMI display
 * Thread-safe retrieval from TELEMETRY cache
 *
 * @param[out] data Pointer to telemetry_impluvium_t structure to fill
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null pointer
 */
esp_err_t telemetry_get_impluvium_data(telemetry_impluvium_t *data);

#endif // TELEMETRY_H
