#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "ads1115_helper.h"
#include "mcp23008_helper.h"
#include "impluvium.h"
#include "tempesta.h"
#include "fluctus.h"
#include "stellaria.h"
#include "telemetry.h"
#include "solar_calc.h"
#include "hmi.h"
#include "wifi_helper.h"
#include "interval_config.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/spi_master.h"

#include "esp_log.h"
#include "esp_err.h"

#include "main.h"
#include "wifi_credentials.h"


// ########################## Global Variable Definitions ##########################
#define TAG "SOLARIUM_1"

// ########################## FUNCTION DECLARATIONS ################################


// ################################ FUNCTIONS ######################################

// ------- SPI Bus Initialization -------

/**
 * @brief Initialize SPI2 bus for shared use by ABP sensor and HMI display
 */
esp_err_t spi_bus_init(void)
{
    ESP_LOGI(TAG, "Initializing SPI2 bus (MOSI=%d, MISO=%d, SCLK=%d)...",
             SPI2_MOSI_PIN, SPI2_MISO_PIN, SPI2_SCLK_PIN);

    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI2_MOSI_PIN,
        .miso_io_num = SPI2_MISO_PIN,
        .sclk_io_num = SPI2_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI2 bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SPI2 bus initialized successfully");
    return ESP_OK;
}

// -----------------##########################################-----------------
// -----------------################### MAIN #################-----------------
// -----------------##########################################-----------------

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    // Suppress noisy SPI debug messages (keep global DEBUG level for other components)
    esp_log_level_set("spi_master", ESP_LOG_INFO);

    ESP_LOGI(TAG, "Starting ESP...");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }
    printf("%" PRIu32 "MB %s flash\n",
           flash_size / (uint32_t) (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // Configure timezone for Poland (CET/CEST with automatic DST transitions)
    // CET = UTC+1 (winter), CEST = UTC+2 (summer)
    // DST: Last Sunday of March 02:00 -> Last Sunday of October 03:00
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
    ESP_LOGI(TAG, "Timezone configured: CET-1CEST (Poland)");

    // Initialize Wi-Fi helper
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Initializing WiFi helper...");
    ESP_ERROR_CHECK(wifi_helper_init(WIFI_SSID, WIFI_PASSWORD));

    // Initialize I2C subsystem
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize SPI2 bus (shared by ABP sensor and HMI display)
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Initializing SPI2 bus...");
    ESP_ERROR_CHECK(spi_bus_init());

    // Initialize telemetry subsystem
    // Must run BEFORE interval_config_init (mounts LittleFS at /data)
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Initializing telemetry subsystem...");
    esp_err_t telemetry_ret = telemetry_init();
    if (telemetry_ret != ESP_OK) {
        ESP_LOGE(TAG, "Telemetry initialization failed: %s", esp_err_to_name(telemetry_ret));
        ESP_LOGW(TAG, "System will continue without telemetry/data logging functionality");
    } else {
        ESP_LOGI(TAG, "Telemetry subsystem initialized successfully");
    }

    // Initialize interval configuration (Must run AFTER telemetry, BEFORE core component init)
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Loading interval configuration...");
    esp_err_t interval_ret = interval_config_init();
    if (interval_ret != ESP_OK) {
        ESP_LOGW(TAG, "Interval config load failed, using defaults");
    } else {
        ESP_LOGI(TAG, "Interval configuration loaded successfully");
    }

    // Initialize solar calculator (Must run BEFORE fluctus_init for midnight callback registration)
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Initializing solar calculator...");
    solar_calc_init();

    //---------- Core system TASKS ----------

    ESP_LOGI(TAG, "Creating core system tasks...");

    // Test solar calculator
    const solar_times_t *times = solar_calc_get_times();
    ESP_LOGI(TAG, "Today's sunrise: %s", ctime(&times->sunrise_time));
    ESP_LOGI(TAG, "Today's sunset: %s", ctime(&times->sunset_time));
    ESP_LOGI(TAG, "Is daytime: %s", solar_calc_is_daytime() ? "YES" : "NO");

    // Initialize FLUCTUS power management and solar tracking system
    // IMPORTANT: Must be initialized BEFORE ads1115_helper (ADS1115 needs 3.3V bus power)
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Initializing FLUCTUS power management...");
    esp_err_t fluctus_ret = fluctus_init();
    if (fluctus_ret != ESP_OK) {
        ESP_LOGE(TAG, "FLUCTUS initialization failed: %s", esp_err_to_name(fluctus_ret));
        // Continue anyway - system can still operate with limited functionality
    } else {
        ESP_LOGI(TAG, "FLUCTUS initialized successfully");
    }

    // Initialize ADS1115 helper system (requires FLUCTUS for 3.3V bus power)
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Initializing ADS1115 helper...");
    esp_err_t ads_init_result = ads1115_helper_init();
    if (ads_init_result == ESP_FAIL) {
        ESP_LOGW(TAG,
                 "No ADS1115 devices available at startup - system will continue with limited "
                 "functionality");
        ESP_LOGW(TAG, "The ADS1115 helper retry task will attempt to recover failed devices automatically");
    } else {
        ESP_LOGI(TAG, "ADS1115 helper system initialized successfully");
    }

    // Initialize MCP23008 I/O expander (encoder, pulse counters, hall array enable)
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Initializing MCP23008 helper...");
    esp_err_t mcp_ret = mcp23008_helper_init();
    if (mcp_ret != ESP_OK) {
        ESP_LOGE(TAG, "MCP23008 helper initialization failed: %s", esp_err_to_name(mcp_ret));
        ESP_LOGW(TAG, "System will continue without encoder/pulse counter functionality");
    } else {
        ESP_LOGI(TAG, "MCP23008 helper initialized successfully");
    }

    // Initialize weather station (SHT4x, BME280, AS5600, PMS5003)
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Initializing weather station...");
    esp_err_t weather_ret = tempesta_station_init();
    if (weather_ret != ESP_OK) {
        ESP_LOGE(TAG, "Weather station initialization failed: %s", esp_err_to_name(weather_ret));
        ESP_LOGW(TAG, "System will continue with limited weather functionality");
    } else {
        ESP_LOGI(TAG, "Weather station initialized successfully");
    }

    // Initialize IMPLUVIUM irrigation system
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Initializing IMPLUVIUM irrigation system...");
    esp_err_t impluvium_ret = impluvium_init();
    if (impluvium_ret != ESP_OK) {
        ESP_LOGE(TAG, "IMPLUVIUM initialization failed: %s", esp_err_to_name(impluvium_ret));
    } else {
        ESP_LOGI(TAG, "IMPLUVIUM initialized successfully");
    }
     
    // Initialize STELLARIA ambient lighting system
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Initializing STELLARIA ambient lighting...");
    esp_err_t stellaria_ret = stellaria_init();
    if (stellaria_ret != ESP_OK) {
        ESP_LOGE(TAG, "STELLARIA initialization failed: %s", esp_err_to_name(stellaria_ret));
        // Continue anyway - ambient lighting is optional
    } else {
        ESP_LOGI(TAG, "STELLARIA initialized successfully");

        // Enable automatic light toggle mode (uses FLUCTUS photoresistor readings)
        esp_err_t auto_ret = stellaria_set_auto_mode(true);
        if (auto_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to enable STELLARIA auto mode: %s", esp_err_to_name(auto_ret));
        } else {
            ESP_LOGI(TAG, "STELLARIA auto mode enabled");
        }
    }

    // Initialize HMI system (OLED display + rotary encoder)
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Initializing HMI system...");
    esp_err_t hmi_ret = hmi_init();
    if (hmi_ret != ESP_OK) {
        ESP_LOGE(TAG, "HMI initialization failed: %s", esp_err_to_name(hmi_ret));
        ESP_LOGW(TAG, "System will continue without display/user interface");
    } else {
        ESP_LOGI(TAG, "HMI initialized successfully");
    }

    ESP_LOGI(TAG, "Tasks created!!!");

    // Detailed heap stats after all initialization complete
    ESP_LOGI(TAG, "=== Heap Memory Status (Post-Init) ===");
    ESP_LOGI(TAG, "Internal RAM free:    %6d bytes (DMA-capable, fast)", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "Internal RAM largest: %6d bytes (biggest block)", heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "PSRAM free:           %6d bytes (slower, buffers)", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "PSRAM largest:        %6d bytes (biggest block)", heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "Total free heap:      %6d bytes (all combined)", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    ESP_LOGI(TAG, "Min ever free heap:   %lu bytes (lowest point)", esp_get_minimum_free_heap_size());
    ESP_LOGI(TAG, "======================================");
}