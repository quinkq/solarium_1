// - Direct-write cache API (lock/unlock only)
// - Snapshot retrieval (all get_*_data functions for HMI)
// - Per-cache mutex management

#include "telemetry.h"
#include "telemetry_private.h"


static const char *TAG = "TELEMETRY_CACHE";


// ################ Unified Cache Implementation ################

inline SemaphoreHandle_t telemetry_get_mutex(telemetry_source_t src)
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
