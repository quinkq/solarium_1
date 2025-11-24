/**
 * @file telemetry_cache.c
 * @brief TELEMETRY cache management - Pure data hub
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Passive data storage module for TELEMETRY component:
 * - Direct-write cache API (lock/unlock only) - zero-copy access for components
 * - Snapshot retrieval (all get_*_data functions for HMI display)
 * - Per-cache mutex management (one mutex per source, minimal contention)
 * - No MQTT dependencies - pure data hub (circular dependency avoided)
 *
 * Components write directly to cache via lock/unlock, HMI reads via memcpy snapshots.
 * Cache memory: ~900B in internal SRAM for low-latency access.
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "telemetry.h"
#include "telemetry_private.h"


static const char *TAG = "TELEMETRY_CACHE";


// ################ Unified Cache Implementation ################

/**
 * @brief Get or create mutex for a specific cache source
 * Lazy initialization of per-cache mutexes
 */
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

/**
 * @brief Lock cache for direct write access - Zero-copy API
 * Returns pointer to cache for component to write directly
 */
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

/**
 * @brief Unlock cache after write - Timestamp handled by writer functions
 * Components must call this after completing cache write
 */
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
