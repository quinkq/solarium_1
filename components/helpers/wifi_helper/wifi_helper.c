/**
 * @file wifi_helper.c
 * @brief Power-aware WiFi management with exponential backoff
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Original implementation of WiFi Helper - power-optimized connectivity.
 *
 * Key features:
 * - Automatic reconnection with exponential backoff (1s → 60s, max 20 retries)
 * - Three power states: Enabled / Power-Save (~20mA) / Shutdown
 * - TELEMETRY integration with event-driven snapshot injection
 * - RSSI monitoring with configurable update threshold (>5dBm delta)
 * - Detailed disconnect reason tracking and diagnostics
 * - SNTP time synchronization for solar calculations
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "wifi_helper.h"
#include "telemetry.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_sntp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include <string.h>

// ########################## Constants ##########################

#define TAG "WIFI_HELPER"

// Exponential backoff delays (milliseconds)
static const uint32_t BACKOFF_DELAYS_MS[] = {
    1000,    // 1 second
    2000,    // 2 seconds
    5000,    // 5 seconds
    10000,   // 10 seconds
    30000,   // 30 seconds
    60000,   // 60 seconds
    600000,  // 10 minutes
    3600000, // 60 minutes (max)
};
#define BACKOFF_LEVELS (sizeof(BACKOFF_DELAYS_MS) / sizeof(BACKOFF_DELAYS_MS[0]))

// ########################## Internal State ##########################

static bool wifi_initialized = false;
static SemaphoreHandle_t xWiFiMutex = NULL;

// WiFi configuration (stored on init)
static char wifi_ssid[33] = {0};
static char wifi_password[65] = {0};

// Current WiFi state
static wifi_snapshot_t current_snapshot = {
    .state = WIFI_STATE_INIT,
    .rssi = 0,
    .reconnect_count = 0,
    .connected_time_sec = 0,
    .last_disconnect_reason = 0,
    .has_ip = false,
    .power_save_mode = false
};

// Reconnection state
static uint8_t retry_count = 0;
static uint8_t backoff_level = 0;
static TimerHandle_t reconnect_timer = NULL;
static bool shutdown_requested = false;

// WiFi restart delay (anti-bounce for rapid power state changes)
static TimerHandle_t restart_delay_timer = NULL;
#define WIFI_RESTART_DELAY_MS 30000  // 30 seconds

// RSSI monitoring
static TaskHandle_t rssi_monitor_task_handle = NULL;
static int8_t last_reported_rssi = 0;

// SNTP initialization state
static bool sntp_initialized = false;

// Connection timing
static time_t connect_time = 0;
static time_t total_connected_sec = 0;

// ########################## Forward Declarations ##########################

static void reconnect_timer_callback(TimerHandle_t xTimer);
static void restart_delay_timer_callback(TimerHandle_t xTimer);
static void rssi_monitor_task(void *pvParameters);
static void update_connected_time(void);

// ########################## Helper Functions ##########################

/**
 * @brief Get exponential backoff delay for current retry level
 */
static uint32_t get_backoff_delay_ms(void)
{
    uint8_t level = (backoff_level < BACKOFF_LEVELS) ? backoff_level : (BACKOFF_LEVELS - 1);
    return BACKOFF_DELAYS_MS[level];
}

/**
 * @brief Update WiFi state and inject to TELEMETRY
 */
static void set_wifi_state(wifi_state_t new_state)
{
    if (xSemaphoreTake(xWiFiMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    if (current_snapshot.state != new_state) {
        current_snapshot.state = new_state;
        xSemaphoreGive(xWiFiMutex);

        // Inject to TELEMETRY on state change
        telemetry_fetch_snapshot(TELEMETRY_SRC_WIFI);

        const char *state_str[] = {
            "DISABLED", "INIT", "CONNECTING", "CONNECTED", "RECONNECTING", "FAILED"
        };
        ESP_LOGI(TAG, "WiFi state: %s", state_str[new_state]);
    } else {
        xSemaphoreGive(xWiFiMutex);
    }
}

/**
 * @brief Update RSSI and inject to TELEMETRY if changed significantly
 */
static void update_rssi(int8_t new_rssi)
{
    if (xSemaphoreTake(xWiFiMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    current_snapshot.rssi = new_rssi;

    // Check if RSSI changed significantly (threshold: 5 dBm)
    int8_t delta = abs(new_rssi - last_reported_rssi);

    xSemaphoreGive(xWiFiMutex);

    if (delta >= WIFI_HELPER_RSSI_UPDATE_THRESHOLD) {
        last_reported_rssi = new_rssi;
        telemetry_fetch_snapshot(TELEMETRY_SRC_WIFI);
        ESP_LOGD(TAG, "RSSI updated: %d dBm", new_rssi);
    }
}

/**
 * @brief Update total connected time counter
 */
static void update_connected_time(void)
{
    if (connect_time > 0) {
        time_t now = time(NULL);
        total_connected_sec += (now - connect_time);
        connect_time = now;  // Reset for next interval
    }
}

/**
 * @brief Initialize SNTP for time synchronization
 */
static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
}

// ########################## WiFi Event Handlers ##########################

/**
 * @brief WiFi event handler
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base != WIFI_EVENT) {
        return;
    }

    switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi started, connecting to %s...", wifi_ssid);
            set_wifi_state(WIFI_STATE_CONNECTING);
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;

            // Update disconnect reason
            if (xSemaphoreTake(xWiFiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                current_snapshot.last_disconnect_reason = event->reason;
                current_snapshot.has_ip = false;
                current_snapshot.rssi = 0;
                update_connected_time();  // Save connected time
                xSemaphoreGive(xWiFiMutex);
            }

            ESP_LOGW(TAG, "WiFi disconnected (reason: %d)", event->reason);

            // Don't reconnect if shutdown requested
            if (shutdown_requested) {
                ESP_LOGI(TAG, "Shutdown requested, not reconnecting");
                set_wifi_state(WIFI_STATE_DISABLED);
                break;
            }

            // Check if max retries exceeded
            if (retry_count >= WIFI_HELPER_MAX_RETRIES) {
                ESP_LOGE(TAG, "Max retries exceeded (%d), entering FAILED state", retry_count);
                set_wifi_state(WIFI_STATE_FAILED);
                break;
            }

            // Start reconnection with backoff
            retry_count++;
            uint32_t delay_ms = get_backoff_delay_ms();

            ESP_LOGI(TAG, "Reconnecting in %lu ms (attempt %d/%d, level %d)",
                     delay_ms, retry_count, WIFI_HELPER_MAX_RETRIES, backoff_level);

            set_wifi_state(WIFI_STATE_RECONNECTING);

            // Start timer for delayed reconnection
            if (reconnect_timer != NULL) {
                xTimerChangePeriod(reconnect_timer, pdMS_TO_TICKS(delay_ms), 100);
                xTimerStart(reconnect_timer, 100);
            }

            // Increase backoff level for next retry
            if (backoff_level < BACKOFF_LEVELS - 1) {
                backoff_level++;
            }

            // Update reconnect counter
            if (xSemaphoreTake(xWiFiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                current_snapshot.reconnect_count++;
                xSemaphoreGive(xWiFiMutex);
            }

            telemetry_fetch_snapshot(TELEMETRY_SRC_WIFI);
            break;
        }

        default:
            break;
    }
}

/**
 * @brief IP event handler
 */
static void ip_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_base != IP_EVENT) {
        return;
    }

    if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));

        // Reset retry counters on successful connection
        retry_count = 0;
        backoff_level = 0;
        connect_time = time(NULL);

        // Update state
        if (xSemaphoreTake(xWiFiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            current_snapshot.has_ip = true;
            xSemaphoreGive(xWiFiMutex);
        }

        set_wifi_state(WIFI_STATE_CONNECTED);

        // Notify rssi_monitor_task to perform post-connection initialization
        // (SNTP init + initial RSSI reading) in a safe context with adequate stack
        if (rssi_monitor_task_handle != NULL) {
            xTaskNotifyGive(rssi_monitor_task_handle);
        }
    }
}

/**
 * @brief Timer callback for delayed reconnection
 */
static void reconnect_timer_callback(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Reconnect timer expired, attempting connection...");
    esp_wifi_connect();
}

/**
 * @brief Timer callback for delayed WiFi restart (anti-bounce)
 * Waits 30s after power state returns to NORMAL before restarting WiFi
 * Prevents rapid cycling during voltage bounces (e.g., IMPLUVIUM pump tests)
 */
static void restart_delay_timer_callback(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "WiFi restart delay expired, starting WiFi...");

    // Check if shutdown was requested again during delay
    if (shutdown_requested) {
        ESP_LOGI(TAG, "Shutdown requested during delay, aborting WiFi restart");
        return;
    }

    // Start WiFi
    esp_wifi_start();
    retry_count = 0;
    backoff_level = 0;

    // Re-enable power save mode after restart (it's reset by esp_wifi_start())
    // Note: This will be applied once WiFi connects, setting it early is safe
    vTaskDelay(pdMS_TO_TICKS(100));  // Small delay to let WiFi start
    esp_err_t ps_ret = esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    if (ps_ret == ESP_OK) {
        if (xSemaphoreTake(xWiFiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            current_snapshot.power_save_mode = true;
            xSemaphoreGive(xWiFiMutex);
        }
        ESP_LOGI(TAG, "WiFi power save mode re-enabled after restart");
    } else {
        ESP_LOGW(TAG, "Failed to re-enable power save mode: %s", esp_err_to_name(ps_ret));
    }
}

/**
 * @brief RSSI monitoring task (runs when connected)
 * Handles post-connection initialization (SNTP + initial RSSI) via task notification
 * Performs periodic RSSI checks every 15 minutes
 */
static void rssi_monitor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "RSSI monitor task started");

    while (1) {
        // Wait for notification (post-connection init) or timeout (periodic check)
        uint32_t notification = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(WIFI_HELPER_RSSI_CHECK_INTERVAL));

        // Monitor stack usage (debug - watermark shows minimum free stack ever reached)
        UBaseType_t stack_high_water = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGI(TAG, "[STACK] High water mark: %u bytes free (min ever)", stack_high_water * sizeof(StackType_t));

        // Handle post-connection initialization notification
        if (notification > 0) {
            ESP_LOGI(TAG, "Post-connection initialization triggered");

            // Initialize SNTP (only once per boot, idempotent on reconnections)
            if (!sntp_initialized) {
                initialize_sntp();
                sntp_initialized = true;
            }

            // Get initial RSSI after connection
            if (current_snapshot.state == WIFI_STATE_CONNECTED && current_snapshot.has_ip) {
                wifi_ap_record_t ap_info;
                if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                    update_rssi(ap_info.rssi);
                    last_reported_rssi = ap_info.rssi;
                    ESP_LOGI(TAG, "Initial RSSI: %d dBm", ap_info.rssi);
                }
            }
        }

        // Periodic RSSI check (timeout or after notification handling)
        if (current_snapshot.state == WIFI_STATE_CONNECTED && current_snapshot.has_ip) {
            wifi_ap_record_t ap_info;
            if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                update_rssi(ap_info.rssi);
            }

            // Update connected time counter
            if (xSemaphoreTake(xWiFiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                update_connected_time();
                current_snapshot.connected_time_sec = total_connected_sec;
                xSemaphoreGive(xWiFiMutex);
            }
        }
    }
}

// ########################## Public API Implementation ##########################

esp_err_t wifi_helper_init(const char *ssid, const char *password)
{
    if (ssid == NULL || password == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (wifi_initialized) {
        ESP_LOGW(TAG, "WiFi helper already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing WiFi helper...");

    // Create mutex
    xWiFiMutex = xSemaphoreCreateMutex();
    if (xWiFiMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    // Store credentials
    strncpy(wifi_ssid, ssid, sizeof(wifi_ssid) - 1);
    strncpy(wifi_password, password, sizeof(wifi_password) - 1);

    // Initialize ESP-NETIF
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL, NULL));

    // Configure WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
        },
    };
    strncpy((char *)wifi_config.sta.ssid, wifi_ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, wifi_password, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // Create reconnect timer (one-shot)
    reconnect_timer = xTimerCreate("wifi_reconnect", pdMS_TO_TICKS(1000),
                                    pdFALSE, NULL, reconnect_timer_callback);
    if (reconnect_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create reconnect timer");
        return ESP_FAIL;
    }

    // Create restart delay timer (one-shot, anti-bounce)
    restart_delay_timer = xTimerCreate("wifi_restart_delay", pdMS_TO_TICKS(WIFI_RESTART_DELAY_MS),
                                        pdFALSE, NULL, restart_delay_timer_callback);
    if (restart_delay_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create restart delay timer");
        return ESP_FAIL;
    }

    // Start RSSI monitoring task
    BaseType_t task_created = xTaskCreate(
        rssi_monitor_task,
        "wifi_rssi",
        4096,  // Increased to handle SNTP init + RSSI operations
        NULL,
        5,
        &rssi_monitor_task_handle
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RSSI monitor task");
        return ESP_FAIL;
    }

    // Start WiFi
    ESP_ERROR_CHECK(esp_wifi_start());

    // Enable power save mode by default (WIFI_PS_MAX_MODEM: ~20mA vs WIFI_PS_NONE: ~100mA)
    // This reduces power consumption while maintaining connectivity and MQTT functionality
    esp_err_t ps_ret = esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    if (ps_ret == ESP_OK) {
        current_snapshot.power_save_mode = true;
        ESP_LOGI(TAG, "WiFi power save mode enabled by default (WIFI_PS_MAX_MODEM)");
    } else {
        ESP_LOGW(TAG, "Failed to enable default power save mode: %s", esp_err_to_name(ps_ret));
    }

    wifi_initialized = true;
    ESP_LOGI(TAG, "WiFi helper initialized successfully");

    return ESP_OK;
}

esp_err_t wifi_helper_get_snapshot(wifi_snapshot_t *snapshot)
{
    if (!wifi_initialized || snapshot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(xWiFiMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    memcpy(snapshot, &current_snapshot, sizeof(wifi_snapshot_t));

    xSemaphoreGive(xWiFiMutex);

    return ESP_OK;
}

esp_err_t wifi_helper_set_power_save_mode(bool enable)
{
    if (!wifi_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    wifi_ps_type_t ps_mode = enable ? WIFI_PS_MAX_MODEM : WIFI_PS_NONE;
    esp_err_t ret = esp_wifi_set_ps(ps_mode);

    if (ret == ESP_OK) {
        if (xSemaphoreTake(xWiFiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            current_snapshot.power_save_mode = enable;
            xSemaphoreGive(xWiFiMutex);
        }

        ESP_LOGI(TAG, "WiFi power save mode %s", enable ? "ENABLED" : "DISABLED");
        telemetry_fetch_snapshot(TELEMETRY_SRC_WIFI);
    } else {
        ESP_LOGE(TAG, "Failed to set power save mode: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t wifi_helper_set_shutdown(bool shutdown)
{
    if (!wifi_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    shutdown_requested = shutdown;

    if (shutdown) {
        ESP_LOGI(TAG, "Shutting down WiFi...");

        // Cancel any pending restart timer (immediate shutdown takes priority)
        if (xTimerIsTimerActive(restart_delay_timer)) {
            xTimerStop(restart_delay_timer, 100);
            ESP_LOGI(TAG, "Cancelled pending WiFi restart");
        }

        // Update connected time before shutdown
        update_connected_time();

        esp_wifi_stop();
        set_wifi_state(WIFI_STATE_DISABLED);

        // Reset connection state
        if (xSemaphoreTake(xWiFiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            current_snapshot.has_ip = false;
            current_snapshot.rssi = 0;
            xSemaphoreGive(xWiFiMutex);
        }
    } else {
        // Delayed restart to prevent rapid cycling during voltage bounces
        ESP_LOGI(TAG, "WiFi restart requested, delaying %d seconds to prevent rapid cycling...",
                 WIFI_RESTART_DELAY_MS / 1000);

        // Start delay timer (will call esp_wifi_start() after 30s)
        xTimerStart(restart_delay_timer, 100);
    }

    return ESP_OK;
}

esp_err_t wifi_helper_force_reconnect(void)
{
    if (!wifi_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (shutdown_requested) {
        ESP_LOGW(TAG, "Cannot reconnect while shutdown requested");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Force reconnect requested, resetting retry counters");

    // Reset retry state
    retry_count = 0;
    backoff_level = 0;

    // Disconnect and reconnect
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_wifi_connect();

    set_wifi_state(WIFI_STATE_CONNECTING);

    return ESP_OK;
}

/**
 * @brief Write WiFi snapshot to TELEMETRY cache (called by TELEMETRY)
 * This is the cache writer function registered in TELEMETRY streams table
 *
 * @param cache Pointer to TELEMETRY's wifi_snapshot_t buffer
 * @return ESP_OK on success, ESP_FAIL if mutex timeout
 */
esp_err_t wifi_helper_write_to_telemetry_cache(wifi_snapshot_t *cache)
{
    if (cache == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!wifi_initialized) {
        memset(cache, 0, sizeof(wifi_snapshot_t));
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xWiFiMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    // Copy current WiFi state to TELEMETRY cache
    memcpy(cache, &current_snapshot, sizeof(wifi_snapshot_t));

    xSemaphoreGive(xWiFiMutex);

    return ESP_OK;
}
