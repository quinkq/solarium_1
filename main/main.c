#include <stdio.h>
#include <string.h>
#include <math.h>

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
#include "irrigation.h"
#include "weather_station.h"
#include "fluctus.h"
#include "stellaria.h"
#include "telemetry.h"
#include "solar_calc.h" // For debug printouts only - remvoe it later
#include "hmi.h" 

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

#include "esp_log.h"
#include "esp_err.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_sntp.h"
#include <time.h>

#include "driver/spi_master.h"

#include "main.h"
#include "wifi_credentials.h"


// ########################## Global Variable Definitions ##########################
#define TAG "SOLARIUM_1"

// Mutex for protecting shared display data
SemaphoreHandle_t xDisplayDataMutex = NULL;

// ADS1115 voltages are now managed by ads1115_helper component

// INA219 data now retrieved directly from FLUCTUS component

// Debuging display variables
float latest_sht_temp = -999.9;
float latest_sht_hum = -999.9;
float latest_bmp_temp = -999.9;
float latest_bmp_press = -999.9;
float latest_bmp_hum = -999.9;
float latest_as5600_angle = -999.9;
uint16_t latest_as5600_raw = 0;

// Flag to indicate if time has been synchronized
static bool g_time_synced = false;


// ########################## FUNCTION DECLARATIONS ################################


// ################################ FUNCTIONS ######################################
// ------- WiFI and Simple Network Time Protocol Synchronization -------

// Wi-Fi credentials are in wifi_credentials.h

static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Time synchronized");
    g_time_synced = true;
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        initialize_sntp();
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta =
            {
                .ssid = WIFI_SSID,
                .password = WIFI_PASSWORD,
            },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

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

// -----------------#################################-----------------
// -----------------############# TASKS #############-----------------
// -----------------#################################-----------------

void serial_debug_display_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Serial Display Task started.");
    vTaskDelay(pdMS_TO_TICKS(1000));
    // ANSI escape codes
    const char *ANSI_CLEAR_SCREEN = "\033[2J";
    const char *ANSI_CURSOR_HOME = "\033[H";

    char buffer[100]; // Buffer for formatting lines

    // Declare local variables for sensor readings outside the loop
    float sht_t, sht_h, bmp_t, bmp_p, bmp_h, as_a;
    uint16_t as_r;
    float ads_voltages[ADS1115_DEVICE_COUNT][4];
    fluctus_monitoring_data_t fluctus_data;

    while (1) {
        // --- Read Global Variables Safely ---
        if (xSemaphoreTake(xDisplayDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // SHT4x
            sht_t = latest_sht_temp;
            sht_h = latest_sht_hum;
            // BMP280
            bmp_t = latest_bmp_temp;
            bmp_p = latest_bmp_press;
            bmp_h = latest_bmp_hum;
            // Get ADS1115 voltages from helper component (no longer from global arrays)
            // AS5600
            as_a = latest_as5600_angle;
            as_r = latest_as5600_raw;
            // INA219 values will be read from FLUCTUS separately

            xSemaphoreGive(xDisplayDataMutex);
        } else {
            ESP_LOGW(TAG, "Serial display task could not get mutex");
            // Optionally print an error message to serial?
            printf("%s%s!!! Failed to get mutex !!!\n", ANSI_CLEAR_SCREEN, ANSI_CURSOR_HOME);
            vTaskDelay(pdMS_TO_TICKS(SERIAL_DISPLAY_INTERVAL_MS));
            continue;
        }
        
        // Get ADS1115 voltages from helper component
        esp_err_t ads_ret = ads1115_get_latest_voltages(ads_voltages);
        if (ads_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to get ADS1115 voltages: %s", esp_err_to_name(ads_ret));
            // Initialize with error values
            for (int dev = 0; dev < ADS1115_DEVICE_COUNT; dev++) {
                for (int ch = 0; ch < 4; ch++) {
                    ads_voltages[dev][ch] = -999.9;
                }
            }
        }
        
        // Get INA219/power monitoring data from FLUCTUS
        esp_err_t fluctus_ret = fluctus_get_monitoring_data(&fluctus_data);
        if (fluctus_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to get FLUCTUS monitoring data: %s", esp_err_to_name(fluctus_ret));
            // Initialize with error values
            fluctus_data.solar_pv.voltage = fluctus_data.solar_pv.current = fluctus_data.solar_pv.power = -999.9;
            fluctus_data.battery_out.voltage = fluctus_data.battery_out.current = fluctus_data.battery_out.power = -999.9;
        }
        // --- End Read ---

        // --- Format and Print to Serial Monitor ---
        // Clear screen and move cursor to top-left

        // printf("%s%s", ANSI_CLEAR_SCREEN, ANSI_CURSOR_HOME);

        // Print header
        printf("--- Sensor Readings ---\n");

        // Print sensor data (chat says using snprintf for safety is good practice)
        snprintf(buffer, sizeof(buffer), "SHT4x: Temp=%.1f C, Hum=%.1f %%\n", sht_t, sht_h);
        printf("%s", buffer);

        snprintf(buffer,
                 sizeof(buffer),
                 "BMP280: Temp=%.1f C, Press=%.0f hPa, Hum=%.1f %%\n",
                 bmp_t,
                 bmp_p / 100,
                 bmp_h); // Assuming BME & Pa -> hPa
        printf("%s", buffer);

        // Display ADS1115 readings organized by function
        snprintf(buffer,
                 sizeof(buffer),
                 "ADS0 (Photo): LT=%.3f RT=%.3f LB=%.3f RB=%.3f\n",
                 ads_voltages[0][0],
                 ads_voltages[0][1],
                 ads_voltages[0][2],
                 ads_voltages[0][3]);
        printf("%s", buffer);

        snprintf(buffer,
                 sizeof(buffer),
                 "ADS1 (Moist): Z1=%.3f Z2=%.3f Z3=%.3f Z4=%.3f\n",
                 ads_voltages[1][0],
                 ads_voltages[1][1],
                 ads_voltages[1][2],
                 ads_voltages[1][3]);
        printf("%s", buffer);

        snprintf(buffer,
                 sizeof(buffer),
                 "ADS2 (Mixed): Z5=%.3f Press=%.3f Ch2=%.3f Ch3=%.3f\n",
                 ads_voltages[2][0],
                 ads_voltages[2][1],
                 ads_voltages[2][2],
                 ads_voltages[2][3]);
        printf("%s", buffer);

        snprintf(buffer, sizeof(buffer), "AS5600: Angle=%.1f deg, Raw=%u\n", as_a, as_r);
        printf("%s", buffer);

        // Add INA219 display
        snprintf(buffer,
                 sizeof(buffer),
                 "INA219_Solar: V=%.2f V, I=%.2f mA, P=%.2f mW\n",
                 fluctus_data.solar_pv.voltage,
                 fluctus_data.solar_pv.current * 1000,
                 fluctus_data.solar_pv.power * 1000);
        printf("%s", buffer);
        
        snprintf(buffer,
                 sizeof(buffer),
                 "INA219_Battery: V=%.2f V, I=%.2f mA, P=%.2f mW\n",
                 fluctus_data.battery_out.voltage,
                 fluctus_data.battery_out.current * 1000,
                 fluctus_data.battery_out.power * 1000);
        printf("%s", buffer);

        printf("-----------------------\n");
        // Flush stdout buffer to ensure data is sent immediately
        fflush(stdout);

        static uint32_t last_print = 0;
            if ((xTaskGetTickCount() - last_print) > pdMS_TO_TICKS(600000)) {
                telemetry_print_power_status();
                telemetry_print_daily_summary();
                last_print = xTaskGetTickCount();
            }

        // --- End Format and Print ---

        vTaskDelay(pdMS_TO_TICKS(SERIAL_DISPLAY_INTERVAL_MS));
    }
}

// -----------------##########################################-----------------
// -----------------################### MAIN #################-----------------
// -----------------##########################################-----------------

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Starting ESP...");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


    /* Display Mutex creation */
    xDisplayDataMutex = xSemaphoreCreateMutex();
    if (xDisplayDataMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create display data mutex!");
        // Handle error appropriately, maybe halt or return an error code
    } else {
        ESP_LOGI(TAG, "Display data mutex created successfully.");
    }

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

    // Initialize Wi-Fi and SNTP
    wifi_init_sta();

    // Initialize I2C subsystem
    ESP_LOGI(TAG, "Initializing I2C...");
    ESP_ERROR_CHECK(i2cdev_init());

    // Initialize SPI2 bus (shared by ABP sensor and HMI display)
    ESP_LOGI(TAG, "Initializing SPI2 bus...");
    ESP_ERROR_CHECK(spi_bus_init());

    // Initialize ADS1115 helper system
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


    //---------- Core system TASKS ----------

    // Initialize telemetry subsystem (power metering, weather data collection, LittleFS storage)
    ESP_LOGI(TAG, "Initializing telemetry subsystem...");
    esp_err_t telemetry_ret = telemetry_init();
    if (telemetry_ret != ESP_OK) {
        ESP_LOGE(TAG, "Telemetry initialization failed: %s", esp_err_to_name(telemetry_ret));
        ESP_LOGW(TAG, "System will continue without telemetry/data logging functionality");
    } else {
        ESP_LOGI(TAG, "Telemetry subsystem initialized successfully");
    }

    // Test solar calculator
  const solar_times_t *times = solar_calc_get_times();
  ESP_LOGI(TAG, "Today's sunrise: %s", ctime(&times->sunrise_time));
  ESP_LOGI(TAG, "Today's sunset: %s", ctime(&times->sunset_time));
  ESP_LOGI(TAG, "Is daytime: %s", solar_calc_is_daytime() ? "YES" : "NO");

    ESP_LOGI(TAG, "Creating tasks...");

    // Initialize FLUCTUS power management and solar tracking system
    ESP_LOGI(TAG, "Initializing FLUCTUS power management...");
    esp_err_t fluctus_ret = fluctus_init();
    if (fluctus_ret != ESP_OK) {
        ESP_LOGE(TAG, "FLUCTUS initialization failed: %s", esp_err_to_name(fluctus_ret));
        // Continue anyway - system can still operate with limited functionality
    } else {
        ESP_LOGI(TAG, "FLUCTUS initialized successfully");
    }

    // Initialize weather station (SHT4x, BME280, AS5600, PMS5003)
    ESP_LOGI(TAG, "Initializing weather station...");
    esp_err_t weather_ret = weather_station_init();
    if (weather_ret != ESP_OK) {
        ESP_LOGE(TAG, "Weather station initialization failed: %s", esp_err_to_name(weather_ret));
        ESP_LOGW(TAG, "System will continue with limited weather functionality");
    } else {
        ESP_LOGI(TAG, "Weather station initialized successfully");
    }

    // Initialize IMPLUVIUM irrigation system
    ESP_LOGI(TAG, "Initializing IMPLUVIUM irrigation system...");
    esp_err_t impluvium_ret = impluvium_init();
    if (impluvium_ret != ESP_OK) {
        ESP_LOGE(TAG, "IMPLUVIUM initialization failed: %s", esp_err_to_name(impluvium_ret));
    } else {
        ESP_LOGI(TAG, "IMPLUVIUM initialized successfully");
    }
     
    // Initialize STELLARIA ambient lighting system
    ESP_LOGI(TAG, "Initializing STELLARIA ambient lighting...");
    esp_err_t stellaria_ret = stellaria_init();
    if (stellaria_ret != ESP_OK) {
        ESP_LOGE(TAG, "STELLARIA initialization failed: %s", esp_err_to_name(stellaria_ret));
        // Continue anyway - ambient lighting is optional
    } else {
        ESP_LOGI(TAG, "STELLARIA initialized successfully");

        // Enable automatic light toggle mode (uses FLUCTUS photoresistor readings)
        esp_err_t auto_ret = stellaria_enable_auto_toggle();
        if (auto_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to enable STELLARIA auto toggle mode: %s", esp_err_to_name(auto_ret));
        } else {
            ESP_LOGI(TAG, "STELLARIA auto toggle mode enabled");
        }
    }

    // Initialize HMI system (OLED display + rotary encoder)
    ESP_LOGI(TAG, "Initializing HMI system...");
    esp_err_t hmi_ret = hmi_init();
    if (hmi_ret != ESP_OK) {
        ESP_LOGE(TAG, "HMI initialization failed: %s", esp_err_to_name(hmi_ret));
        ESP_LOGW(TAG, "System will continue without display/user interface");
    } else {
        ESP_LOGI(TAG, "HMI initialized successfully");
    }

    // Misc tasks / debug tools - ADS1115 retry task now handled by ads1115_helper component
    xTaskCreate(serial_debug_display_task, "serial_debug_display_task", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

    ESP_LOGI(TAG, "Tasks created.");
}
