/**
 * @file hmi.c
 * @brief HMI - Human-Machine Interface with custom rendering system
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * Original implementation of HMI - complete UI system built from scratch.
 *
 * Key features:
 * - Custom 5x8 bitmap font (475 bytes, no external dependencies)
 * - 95-state hierarchical menu system
 * - SH1106 128x64 OLED with framebuffer rendering
 * - EC11 rotary encoder input (PCNT-based)
 * - Variable refresh rates (1Hz static, 4Hz realtime)
 * - Power-aware with auto sleep and wake-on-input
 * - Complete system control and monitoring
 *
 * Part of the Solarium project - Solar-powered garden automation system
 */

#include "hmi.h"
#include "fluctus.h"
#include "impluvium.h"
#include "tempesta.h"
#include "stellaria.h"
#include "telemetry.h"
#include "wifi_helper.h"

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

// ########################## Constants and Variables ##########################

static const char *TAG = "HMI";

// Framebuffer for 128x64 monochrome display (1 bit per pixel)
#define FB_WIDTH 128
#define FB_HEIGHT 64
#define FB_SIZE ((FB_WIDTH * FB_HEIGHT) / 8)
static uint8_t framebuffer[FB_SIZE];

// Simple 5x7 bitmap font (ASCII 32-126)
static const uint8_t font_5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space (32)
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // backslash
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // _
    {0x00, 0x01, 0x02, 0x04, 0x00}, // `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // f
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // i
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // l
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // p
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // x
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // z
    {0x00, 0x08, 0x36, 0x41, 0x00}, // {
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // |
    {0x00, 0x41, 0x36, 0x08, 0x00}, // }
    {0x08, 0x04, 0x08, 0x10, 0x08}, // ~
};

// Global status structure
static hmi_status_t hmi_status = {
    .initialized = false,
    .display_active = false,
    .current_menu = HMI_MENU_MAIN,
    .selected_item = 0,
    .last_activity_time = 0,
    .encoder_count = 0,
    .blink_state = false,
    .blink_counter = 0
};

// Mutex for thread-safe operations
static SemaphoreHandle_t xHmiMutex = NULL;

// Task handles
static TaskHandle_t xHmiDisplayTask = NULL;

// Stellaria control page editing state
static bool stellaria_intensity_editing = false;
static uint8_t stellaria_intensity_percent = 50;  // 0-100%

// IMPLUVIUM zone editing state
static bool zone_editing = false;
static uint8_t editing_zone_id = 0;                    // 0-4
static bool editing_zone_enabled = true;
static uint8_t editing_zone_target = 45;              // 20-80%
static uint8_t editing_zone_deadband = 5;             // 1-20%

// IMPLUVIUM manual water input state
static bool manual_water_input = false;
static uint8_t manual_water_zone = 0;
static uint16_t manual_water_duration = 30;           // 5-300 seconds

// Confirmation dialog state (shared by IMPLUVIUM, TEMPESTA, and SYSTEM)
typedef enum {
    CONFIRM_NONE = 0,
    CONFIRM_RESET_ZONE_LEARNING,
    CONFIRM_RESET_ALL_LEARNING,
    CONFIRM_MANUAL_WATER,
    CONFIRM_TEMPESTA_RESET_DAILY,
    CONFIRM_TEMPESTA_RESET_WEEKLY,
    CONFIRM_TEMPESTA_RESET_RAIN_TOTAL,
    CONFIRM_TEMPESTA_RESET_TANK_TOTAL,
    CONFIRM_TEMPESTA_RESET_ALL,
    CONFIRM_SYSTEM_FLUSH_RESET
} confirm_action_t;
static confirm_action_t confirmation_action = CONFIRM_NONE;
static uint8_t confirmation_param = 0;  // For zone ID or other params
static hmi_menu_state_t confirmation_return_menu = HMI_MENU_MAIN;  // Menu to return to after confirmation

// Hardware handles
static pcnt_unit_handle_t encoder_pcnt_unit = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;

// ########################## Private Function Declarations ##########################

// Encoder functions
static esp_err_t hmi_encoder_init(void);
static void hmi_encoder_button_isr(void *arg);

// Display functions
static esp_err_t hmi_display_init(void);
static esp_err_t hmi_display_power_on(void);
static esp_err_t hmi_display_power_off(void);

// Framebuffer drawing functions
static void fb_clear(void);
static void fb_set_pixel(int x, int y, bool on);
static void fb_draw_char(int x, int y, char c, bool inverted);
static void fb_draw_string(int x, int y, const char *str, bool inverted);
static void fb_draw_hline(int x, int y, int width);
static void fb_flush(void);

// Menu navigation functions
static void hmi_handle_encoder_change(int delta);
static void hmi_handle_button_press(void);
static uint8_t hmi_get_menu_item_count(hmi_menu_state_t menu);

// Menu rendering functions
static void hmi_render_menu(void);
static void hmi_render_main_menu(void);
static void hmi_render_fluctus_menu(void);
static void hmi_render_fluctus_overview_page(void);
static void hmi_render_fluctus_energy_page(void);
static void hmi_render_fluctus_live_power_page(void);
static void hmi_render_fluctus_buses_page(void);
static void hmi_render_fluctus_tracking_page(void);
static void hmi_render_fluctus_solar_debug_page(void);
static void hmi_render_fluctus_controls_page(void);
static void hmi_render_tempesta_menu(void);
static void hmi_render_tempesta_env_page(void);
static void hmi_render_tempesta_wind_page(void);
static void hmi_render_tempesta_rain_page(void);
static void hmi_render_tempesta_tank_page(void);
static void hmi_render_tempesta_air_page(void);
static void hmi_render_tempesta_controls_page(void);
static void hmi_render_impluvium_menu(void);
static void hmi_render_impluvium_overview_page(void);
static void hmi_render_impluvium_statistics_page(void);
static void hmi_render_impluvium_zones_menu(void);
static void hmi_render_impluvium_zones_all_page(void);
static void hmi_render_impluvium_zone_1_page(void);
static void hmi_render_impluvium_zone_2_page(void);
static void hmi_render_impluvium_zone_3_page(void);
static void hmi_render_impluvium_zone_4_page(void);
static void hmi_render_impluvium_zone_5_page(void);
static void hmi_render_impluvium_learning_menu(void);
static void hmi_render_impluvium_learning_all_page(void);
static void hmi_render_impluvium_learning_1_page(void);
static void hmi_render_impluvium_learning_2_page(void);
static void hmi_render_impluvium_learning_3_page(void);
static void hmi_render_impluvium_learning_4_page(void);
static void hmi_render_impluvium_learning_5_page(void);
static void hmi_render_impluvium_monitor_page(void);
static void hmi_render_impluvium_controls_page(void);
static void hmi_render_impluvium_zone_config_page(void);
static void hmi_render_impluvium_zone_edit_page(void);
static void hmi_render_impluvium_manual_water_page(void);
static void hmi_render_stellaria_menu(void);
static void hmi_render_stellaria_status_page(void);
static void hmi_render_stellaria_control_page(void);
static void hmi_render_stellaria_auto_page(void);
static void hmi_render_system_menu(void);
static void hmi_render_system_info_page(void);
static void hmi_render_system_controls_page(void);
static void hmi_render_confirm_page(void);

// Task functions
static void hmi_display_task(void *pvParameters);

// Activity tracking
static void hmi_update_activity(void);
static bool hmi_check_timeout(void);

// ########################## Encoder Functions ##########################

/**
 * @brief Initialize EC11 rotary encoder with PCNT
 */
static esp_err_t hmi_encoder_init(void)
{
    ESP_LOGI(TAG, "Initializing EC11 rotary encoder (PCNT)...");

    // Create PCNT unit for quadrature decoding
    // Note: One PCNT unit can handle both encoder channels (A and B)
    pcnt_unit_config_t unit_config = {
        .high_limit = HMI_PCNT_HIGH_LIMIT,
        .low_limit = HMI_PCNT_LOW_LIMIT,
    };

    esp_err_t ret = pcnt_new_unit(&unit_config, &encoder_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT unit: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure channel A (primary encoder signal)
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = HMI_ENCODER_A_GPIO,
        .level_gpio_num = HMI_ENCODER_B_GPIO,  // Use B as control signal for quadrature
    };

    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ret = pcnt_new_channel(encoder_pcnt_unit, &chan_a_config, &pcnt_chan_a);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT channel A: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure channel B (secondary encoder signal)
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = HMI_ENCODER_B_GPIO,
        .level_gpio_num = HMI_ENCODER_A_GPIO,  // Use A as control signal for quadrature
    };

    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ret = pcnt_new_channel(encoder_pcnt_unit, &chan_b_config, &pcnt_chan_b);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT channel B: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set edge and level actions for quadrature decoding
    // Channel A: increment on positive edge when B is low, decrement when B is high
    pcnt_channel_set_edge_action(pcnt_chan_a,
                                  PCNT_CHANNEL_EDGE_ACTION_INCREASE,  // Pos edge
                                  PCNT_CHANNEL_EDGE_ACTION_DECREASE); // Neg edge
    pcnt_channel_set_level_action(pcnt_chan_a,
                                   PCNT_CHANNEL_LEVEL_ACTION_KEEP,    // B high
                                   PCNT_CHANNEL_LEVEL_ACTION_INVERSE); // B low

    // Channel B: increment on positive edge when A is high, decrement when A is low
    pcnt_channel_set_edge_action(pcnt_chan_b,
                                  PCNT_CHANNEL_EDGE_ACTION_INCREASE,  // Pos edge
                                  PCNT_CHANNEL_EDGE_ACTION_DECREASE); // Neg edge
    pcnt_channel_set_level_action(pcnt_chan_b,
                                   PCNT_CHANNEL_LEVEL_ACTION_INVERSE, // A high
                                   PCNT_CHANNEL_LEVEL_ACTION_KEEP);   // A low

    // Enable and start the PCNT unit
    ret = pcnt_unit_enable(encoder_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable PCNT unit: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_clear_count(encoder_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear PCNT count: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_start(encoder_pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start PCNT unit: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure button GPIO (external pull-up already present)
    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << HMI_ENCODER_BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,    // External pull-up present
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,       // Trigger on both press and release
    };

    ret = gpio_config(&btn_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure button GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    // Install GPIO ISR service and attach handler
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means ISR service already installed (by another component)
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_isr_handler_add(HMI_ENCODER_BTN_GPIO, hmi_encoder_button_isr, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add button ISR handler: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Encoder initialized - PCNT unit handles both channels (A=%d, B=%d, BTN=%d)",
             HMI_ENCODER_A_GPIO, HMI_ENCODER_B_GPIO, HMI_ENCODER_BTN_GPIO);

    return ESP_OK;
}

/**
 * @brief Encoder button ISR handler
 */
static void IRAM_ATTR hmi_encoder_button_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Read button state
    int level = gpio_get_level(HMI_ENCODER_BTN_GPIO);

    if (level == 0) {
        // Button pressed (active low with external pull-up)
        // Notify display task to wake and handle button event
        if (xHmiDisplayTask != NULL) {
            xTaskNotifyFromISR(xHmiDisplayTask, 1, eSetBits, &xHigherPriorityTaskWoken);
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ########################## Display Functions ##########################

/**
 * @brief Initialize SH1106 OLED display via SPI + esp_lcd
 */
static esp_err_t hmi_display_init(void)
{
    ESP_LOGI(TAG, "Initializing SH1106 OLED display (SPI)...");

    // Configure SPI panel IO (shares SPI2_HOST with ABP sensor - bus initialized in main.c)
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = HMI_DISPLAY_DC_GPIO,
        .cs_gpio_num = HMI_DISPLAY_CS_GPIO,
        .pclk_hz = 10 * 1000 * 1000,  // 10 MHz SPI clock
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };

    esp_err_t ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)HMI_DISPLAY_SPI_HOST,
                                              &io_config,
                                              &io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create panel IO: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure SH1106 panel
    // Note: SH1106 is similar to SSD1306 but uses different column addressing
    // If esp_lcd doesn't have native SH1106 support, we may need to use SSD1306 driver with adjustments
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,  // No hardware reset pin
        .color_space = ESP_LCD_COLOR_SPACE_MONOCHROME,
        .bits_per_pixel = 1,
    };

    // Try SSD1306 driver first (SH1106 is compatible with minor differences)
    ret = esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create SSD1306/SH1106 panel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize panel
    ret = esp_lcd_panel_reset(panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset panel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_lcd_panel_init(panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize panel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure display orientation
    esp_lcd_panel_invert_color(panel_handle, false);
    esp_lcd_panel_mirror(panel_handle, false, false);

    // Start with display off (will be powered on by button press)
    esp_lcd_panel_disp_on_off(panel_handle, false);

    ESP_LOGI(TAG, "Display initialized (%dx%d, CS=%d, DC=%d)",
             HMI_DISPLAY_WIDTH, HMI_DISPLAY_HEIGHT,
             HMI_DISPLAY_CS_GPIO, HMI_DISPLAY_DC_GPIO);

    return ESP_OK;
}

/**
 * @brief Power on display (request 3.3V bus)
 */
static esp_err_t hmi_display_power_on(void)
{
    if (hmi_status.display_active) {
        return ESP_OK;  // Already on
    }

    // Request 3.3V bus from FLUCTUS (toggled for display)
    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_3V3, "HMI", return ESP_FAIL);

    // Turn on display
    esp_err_t ret = esp_lcd_panel_disp_on_off(panel_handle, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to turn on display: %s", esp_err_to_name(ret));
        fluctus_release_bus_power(POWER_BUS_3V3, "HMI");
        return ret;
    }
    if (xSemaphoreTake(xHmiMutex, pdMS_TO_TICKS(100)) == pdTRUE){
    hmi_status.display_active = true;
    hmi_update_activity();
    xSemaphoreGive(xHmiMutex);
    }

    ESP_LOGI(TAG, "Display powered ON (3.3V bus requested)");
    return ESP_OK;
}

/**
 * @brief Power off display (release 3.3V bus)
 */
static esp_err_t hmi_display_power_off(void)
{
    if (!hmi_status.display_active) {
        return ESP_OK;  // Already off
    }

    // Turn off display
    esp_lcd_panel_disp_on_off(panel_handle, false);

    // Release 3.3V bus
    fluctus_release_bus_power(POWER_BUS_3V3, "HMI");

    hmi_status.display_active = false;

    ESP_LOGI(TAG, "Display powered OFF (3.3V bus released)");
    return ESP_OK;
}

// ########################## Framebuffer Drawing Functions ##########################

/**
 * @brief Clear framebuffer (all pixels off)
 */
static void fb_clear(void)
{
    memset(framebuffer, 0x00, FB_SIZE);
}

/**
 * @brief Set a single pixel in the framebuffer
 * @param x X coordinate (0-127)
 * @param y Y coordinate (0-63)
 * @param on true = white pixel, false = black pixel
 */
static void fb_set_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= FB_WIDTH || y < 0 || y >= FB_HEIGHT) {
        return;  // Out of bounds
    }

    // Framebuffer layout: 8 pages (rows of 8 pixels), 128 columns
    // Each byte represents 8 vertical pixels
    int page = y / 8;
    int bit = y % 8;
    int index = page * FB_WIDTH + x;

    if (on) {
        framebuffer[index] |= (1 << bit);
    } else {
        framebuffer[index] &= ~(1 << bit);
    }
}

/**
 * @brief Draw a single character (5x7 font)
 * @param x X coordinate (left edge)
 * @param y Y coordinate (top edge)
 * @param c Character to draw (ASCII 32-126)
 * @param inverted true = black on white, false = white on black
 */
static void fb_draw_char(int x, int y, char c, bool inverted)
{
    if (c < 32 || c > 126) {
        c = ' ';  // Replace invalid characters with space
    }

    const uint8_t *glyph = font_5x7[c - 32];

    for (int col = 0; col < 5; col++) {
        uint8_t column_data = glyph[col];
        for (int row = 0; row < 7; row++) {
            bool pixel_on = (column_data & (1 << row)) != 0;
            if (inverted) {
                pixel_on = !pixel_on;
            }
            fb_set_pixel(x + col, y + row, pixel_on);
        }
    }

    // Add 1 pixel spacing after character
    for (int row = 0; row < 7; row++) {
        fb_set_pixel(x + 5, y + row, inverted);
    }
}

/**
 * @brief Draw a string (5x7 font, 6 pixels per char with spacing)
 * @param x X coordinate (left edge)
 * @param y Y coordinate (top edge)
 * @param str Null-terminated string
 * @param inverted true = black on white, false = white on black
 */
static void fb_draw_string(int x, int y, const char *str, bool inverted)
{
    int cur_x = x;
    while (*str) {
        fb_draw_char(cur_x, y, *str, inverted);
        cur_x += 6;  // 5 pixels + 1 spacing
        str++;
    }
}

/**
 * @brief Draw horizontal line
 * @param x X coordinate (left edge)
 * @param y Y coordinate
 * @param width Line width in pixels
 */
static void fb_draw_hline(int x, int y, int width)
{
    for (int i = 0; i < width; i++) {
        fb_set_pixel(x + i, y, true);
    }
}

/**
 * @brief Flush framebuffer to display
 */
static void fb_flush(void)
{
    // Send framebuffer to display using esp_lcd
    // For SSD1306/SH1106, draw the entire framebuffer (128x64 = 1024 bytes)
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, FB_WIDTH, FB_HEIGHT, framebuffer);
}

// ########################## Menu Navigation Functions ##########################

/**
 * @brief Get number of menu items for current menu
 */
static uint8_t hmi_get_menu_item_count(hmi_menu_state_t menu)
{
    switch (menu) {
        case HMI_MENU_MAIN:
            return 5;  // FLUCTUS, TEMPESTA, IMPLUVIUM, STELLARIA, SYSTEM
        case HMI_MENU_FLUCTUS:
            return 8;  // Back, Overview, Energy, Live Power, Buses, Tracking, Solar Debug, Controls
        case HMI_MENU_FLUCTUS_CONTROLS:
            return 3;  // Back, Toggle Tracking, Manual Safety Reset
        case HMI_MENU_TEMPESTA:
            return 7;  // Back, Environment, Wind, Rain Gauge, Tank Intake, Air Quality, Controls
        case HMI_MENU_TEMPESTA_CONTROLS:
            return 8;  // Back, Enable/Disable, Force Collection, Reset Daily, Reset Weekly, Reset Rain, Reset Tank, Reset All
        case HMI_MENU_IMPLUVIUM:
            return 8;  // Back, Overview, Statistics, Zones, Learning, Monitor, Controls, Zone Config
        case HMI_MENU_IMPLUVIUM_ZONES:
            return 7;  // Back, All Zones Summary, Zone 1-5
        case HMI_MENU_IMPLUVIUM_LEARNING:
            return 7;  // Back, Learning Summary, Zone 1-5 Learning
        case HMI_MENU_IMPLUVIUM_CONTROLS:
            return 6;  // Back, Enable/Disable, Force Check, Emergency Reset, Clear Diagnostics, Reset All Learning
        case HMI_MENU_IMPLUVIUM_ZONE_CONFIG:
            return 6;  // Back + 5 zones
        case HMI_MENU_IMPLUVIUM_ZONE_EDIT:
            return 7;  // Cancel, Enabled, Target, Deadband, Reset Learning, Manual Water, Save
        case HMI_MENU_IMPLUVIUM_MANUAL_WATER:
            return 3;  // Duration, Start, Cancel
        case HMI_MENU_STELLARIA:
            return 4;  // Back, Status, Manual Control, Auto Mode
        case HMI_MENU_STELLARIA_CONTROL:
            return 3;  // Back, Enable/Disable, Intensity
        case HMI_MENU_STELLARIA_AUTO:
            return 2;  // Back, Toggle Auto Mode
        case HMI_MENU_SYSTEM:
            return 3;  // Back, System Info, Controls
        case HMI_MENU_SYSTEM_CONTROLS:
            return 3;  // Back, Flush & Reset, WiFi Reconnect
        case HMI_MENU_CONFIRM:
            return 2;  // Yes, No (shared confirmation dialog)
        default:
            return 1;  // Just "Back" for detail pages
    }
}

/**
 * @brief Handle encoder rotation (menu navigation or intensity editing)
 * @param delta Encoder delta (+/- from PCNT)
 */
static void hmi_handle_encoder_change(int delta)
{
    if (delta == 0) return;

    // Special handling for stellaria intensity editing mode
    if (stellaria_intensity_editing) {
        // Adjust intensity value (5% steps)
        int16_t new_value = stellaria_intensity_percent + (delta * 5);
        if (new_value < 0) new_value = 0;
        if (new_value > 100) new_value = 100;
        stellaria_intensity_percent = (uint8_t)new_value;

        hmi_update_activity();
        return;
    }

    // Special handling for IMPLUVIUM zone editing mode
    if (zone_editing && hmi_status.current_menu == HMI_MENU_IMPLUVIUM_ZONE_EDIT) {
        if (hmi_status.selected_item == 2) {
            // Editing target moisture (20-80%, 1% steps)
            int16_t new_value = editing_zone_target + delta;
            if (new_value < 20) new_value = 20;
            if (new_value > 80) new_value = 80;
            editing_zone_target = (uint8_t)new_value;
        } else if (hmi_status.selected_item == 3) {
            // Editing deadband (1-20%, 1% steps)
            int16_t new_value = editing_zone_deadband + delta;
            if (new_value < 1) new_value = 1;
            if (new_value > 20) new_value = 20;
            editing_zone_deadband = (uint8_t)new_value;
        }

        hmi_update_activity();
        return;
    }

    // Special handling for IMPLUVIUM manual water duration input
    if (manual_water_input && hmi_status.current_menu == HMI_MENU_IMPLUVIUM_MANUAL_WATER) {
        if (hmi_status.selected_item == 0) {
            // Editing duration (5-300s, 5s increments)
            int16_t new_value = manual_water_duration + (delta * 5);
            if (new_value < 5) new_value = 5;
            if (new_value > 300) new_value = 300;
            manual_water_duration = (uint16_t)new_value;
        }

        hmi_update_activity();
        return;
    }

    // Normal menu navigation
    uint8_t item_count = hmi_get_menu_item_count(hmi_status.current_menu);

    if (delta > 0) {
        // Rotate clockwise (down in menu)
        hmi_status.selected_item++;
        if (hmi_status.selected_item >= item_count) {
            hmi_status.selected_item = 0;  // Wrap to top
        }
    } else {
        // Rotate counter-clockwise (up in menu)
        if (hmi_status.selected_item == 0) {
            hmi_status.selected_item = item_count - 1;  // Wrap to bottom
        } else {
            hmi_status.selected_item--;
        }
    }

    hmi_update_activity();
}

/**
 * @brief Handle button press (enter submenu or go back)
 */
static void hmi_handle_button_press(void)
{
    // Navigate based on current menu and selected item
    switch (hmi_status.current_menu) {
        case HMI_MENU_MAIN:
            // Enter component submenu
            switch (hmi_status.selected_item) {
                case 0: hmi_status.current_menu = HMI_MENU_FLUCTUS; break;
                case 1: hmi_status.current_menu = HMI_MENU_TEMPESTA; break;
                case 2: hmi_status.current_menu = HMI_MENU_IMPLUVIUM; break;
                case 3: hmi_status.current_menu = HMI_MENU_STELLARIA; break;
                case 4: hmi_status.current_menu = HMI_MENU_SYSTEM; break;
            }
            hmi_status.selected_item = 0;
            break;

        case HMI_MENU_FLUCTUS:
            if (hmi_status.selected_item == 0) {
                hmi_status.current_menu = HMI_MENU_MAIN;
            } else if (hmi_status.selected_item == 1) {
                hmi_status.current_menu = HMI_MENU_FLUCTUS_OVERVIEW;
            } else if (hmi_status.selected_item == 2) {
                hmi_status.current_menu = HMI_MENU_FLUCTUS_ENERGY;
            } else if (hmi_status.selected_item == 3) {
                hmi_status.current_menu = HMI_MENU_FLUCTUS_LIVE_POWER;
            } else if (hmi_status.selected_item == 4) {
                hmi_status.current_menu = HMI_MENU_FLUCTUS_BUSES;
            } else if (hmi_status.selected_item == 5) {
                hmi_status.current_menu = HMI_MENU_FLUCTUS_TRACKING;
            } else if (hmi_status.selected_item == 6) {
                hmi_status.current_menu = HMI_MENU_FLUCTUS_SOLAR_DEBUG;
            } else if (hmi_status.selected_item == 7) {
                hmi_status.current_menu = HMI_MENU_FLUCTUS_CONTROLS;
            }
            hmi_status.selected_item = 0;
            break;

        case HMI_MENU_TEMPESTA:
            if (hmi_status.selected_item == 0) {
                hmi_status.current_menu = HMI_MENU_MAIN;
            } else if (hmi_status.selected_item == 1) {
                hmi_status.current_menu = HMI_MENU_TEMPESTA_ENV;
            } else if (hmi_status.selected_item == 2) {
                hmi_status.current_menu = HMI_MENU_TEMPESTA_WIND;
            } else if (hmi_status.selected_item == 3) {
                hmi_status.current_menu = HMI_MENU_TEMPESTA_RAIN;
            } else if (hmi_status.selected_item == 4) {
                hmi_status.current_menu = HMI_MENU_TEMPESTA_TANK;
            } else if (hmi_status.selected_item == 5) {
                hmi_status.current_menu = HMI_MENU_TEMPESTA_AIR;
            } else if (hmi_status.selected_item == 6) {
                hmi_status.current_menu = HMI_MENU_TEMPESTA_CONTROLS;
            }
            hmi_status.selected_item = 0;
            break;

        case HMI_MENU_IMPLUVIUM:
            if (hmi_status.selected_item == 0) {
                hmi_status.current_menu = HMI_MENU_MAIN;
            } else if (hmi_status.selected_item == 1) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_OVERVIEW;
            } else if (hmi_status.selected_item == 2) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_STATISTICS;
            } else if (hmi_status.selected_item == 3) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_ZONES;  // Goes to submenu
            } else if (hmi_status.selected_item == 4) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_LEARNING;  // Goes to submenu
            } else if (hmi_status.selected_item == 5) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_MONITOR;
            } else if (hmi_status.selected_item == 6) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_CONTROLS;
            } else if (hmi_status.selected_item == 7) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_ZONE_CONFIG;
            }
            hmi_status.selected_item = 0;
            break;

        // IMPLUVIUM Zones submenu navigation
        case HMI_MENU_IMPLUVIUM_ZONES:
            if (hmi_status.selected_item == 0) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM;
            } else if (hmi_status.selected_item == 1) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_ZONES_ALL;
            } else if (hmi_status.selected_item == 2) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_ZONE_1;
            } else if (hmi_status.selected_item == 3) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_ZONE_2;
            } else if (hmi_status.selected_item == 4) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_ZONE_3;
            } else if (hmi_status.selected_item == 5) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_ZONE_4;
            } else if (hmi_status.selected_item == 6) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_ZONE_5;
            }
            hmi_status.selected_item = 0;
            break;

        // IMPLUVIUM Learning submenu navigation
        case HMI_MENU_IMPLUVIUM_LEARNING:
            if (hmi_status.selected_item == 0) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM;
            } else if (hmi_status.selected_item == 1) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_LEARNING_ALL;
            } else if (hmi_status.selected_item == 2) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_LEARNING_1;
            } else if (hmi_status.selected_item == 3) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_LEARNING_2;
            } else if (hmi_status.selected_item == 4) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_LEARNING_3;
            } else if (hmi_status.selected_item == 5) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_LEARNING_4;
            } else if (hmi_status.selected_item == 6) {
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_LEARNING_5;
            }
            hmi_status.selected_item = 0;
            break;

        case HMI_MENU_STELLARIA:
            if (hmi_status.selected_item == 0) {
                hmi_status.current_menu = HMI_MENU_MAIN;
            } else if (hmi_status.selected_item == 1) {
                hmi_status.current_menu = HMI_MENU_STELLARIA_STATUS;
            } else if (hmi_status.selected_item == 2) {
                hmi_status.current_menu = HMI_MENU_STELLARIA_CONTROL;
            } else if (hmi_status.selected_item == 3) {
                hmi_status.current_menu = HMI_MENU_STELLARIA_AUTO;
            }
            hmi_status.selected_item = 0;
            break;

        // FLUCTUS detail pages - go back to parent submenu (button press handler)
        case HMI_MENU_FLUCTUS_OVERVIEW:
        case HMI_MENU_FLUCTUS_ENERGY:
        case HMI_MENU_FLUCTUS_LIVE_POWER:
        case HMI_MENU_FLUCTUS_BUSES:
        case HMI_MENU_FLUCTUS_TRACKING:
        case HMI_MENU_FLUCTUS_SOLAR_DEBUG:
            hmi_status.current_menu = HMI_MENU_FLUCTUS;
            hmi_status.selected_item = 0;
            break;

        // Interactive controls page - execute actions
        case HMI_MENU_FLUCTUS_CONTROLS:
            if (hmi_status.selected_item == 0) {
                // Back
                hmi_status.current_menu = HMI_MENU_FLUCTUS;
                hmi_status.selected_item = 0;
            } else if (hmi_status.selected_item == 1) {
                // Toggle Solar Tracking
                fluctus_snapshot_t data;
                telemetry_get_fluctus_data(&data);

                if (data.tracking_state == SOLAR_TRACKING_DISABLED) {
                    ESP_LOGI(TAG, "HMI: Enabling solar tracking");
                    fluctus_enable_solar_tracking();
                } else {
                    ESP_LOGI(TAG, "HMI: Disabling solar tracking");
                    fluctus_disable_solar_tracking();
                }
            } else if (hmi_status.selected_item == 2) {
                // Manual Safety Reset
                ESP_LOGI(TAG, "HMI: Manual safety reset requested");
                fluctus_manual_safety_reset();
            }
            break;

        case HMI_MENU_TEMPESTA_ENV:
        case HMI_MENU_TEMPESTA_WIND:
        case HMI_MENU_TEMPESTA_RAIN:
        case HMI_MENU_TEMPESTA_TANK:
        case HMI_MENU_TEMPESTA_AIR:
            hmi_status.current_menu = HMI_MENU_TEMPESTA;
            hmi_status.selected_item = 0;
            break;

        // TEMPESTA System Controls page - execute actions
        case HMI_MENU_TEMPESTA_CONTROLS:
            if (hmi_status.selected_item == 0) {
                // Back
                hmi_status.current_menu = HMI_MENU_TEMPESTA;
                hmi_status.selected_item = 0;
            } else if (hmi_status.selected_item == 1) {
                // Toggle System Enable/Disable
                // Note: TEMPESTA doesn't expose current state, so we toggle
                // TODO: Add tempesta_get_system_enabled() to check state first
                ESP_LOGI(TAG, "HMI: Toggling TEMPESTA system enable/disable");
                // For now, assuming enabled - calling with false to disable
                // User will need to press twice if already disabled
                static bool tempesta_enabled = true;
                tempesta_enabled = !tempesta_enabled;
                tempesta_set_system_enabled(tempesta_enabled);
            } else if (hmi_status.selected_item == 2) {
                // Force Collection Now
                ESP_LOGI(TAG, "HMI: Forcing immediate data collection");
                esp_err_t ret = tempesta_force_collection();
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "HMI: Force collection failed (system busy or disabled)");
                }
            } else if (hmi_status.selected_item == 3) {
                // Reset Daily - show confirmation
                ESP_LOGI(TAG, "HMI: Requesting confirmation to reset daily counters");
                confirmation_action = CONFIRM_TEMPESTA_RESET_DAILY;
                confirmation_param = 0;
                confirmation_return_menu = HMI_MENU_TEMPESTA_CONTROLS;
                hmi_status.current_menu = HMI_MENU_CONFIRM;
                hmi_status.selected_item = 0;
            } else if (hmi_status.selected_item == 4) {
                // Reset Weekly - show confirmation
                ESP_LOGI(TAG, "HMI: Requesting confirmation to reset weekly counters");
                confirmation_action = CONFIRM_TEMPESTA_RESET_WEEKLY;
                confirmation_param = 0;
                confirmation_return_menu = HMI_MENU_TEMPESTA_CONTROLS;
                hmi_status.current_menu = HMI_MENU_CONFIRM;
                hmi_status.selected_item = 0;
            } else if (hmi_status.selected_item == 5) {
                // Reset Rain Gauge Total - show confirmation
                ESP_LOGI(TAG, "HMI: Requesting confirmation to reset rain gauge (hardware)");
                confirmation_action = CONFIRM_TEMPESTA_RESET_RAIN_TOTAL;
                confirmation_param = 0;
                confirmation_return_menu = HMI_MENU_TEMPESTA_CONTROLS;
                hmi_status.current_menu = HMI_MENU_CONFIRM;
                hmi_status.selected_item = 0;
            } else if (hmi_status.selected_item == 6) {
                // Reset Tank Intake Total - show confirmation
                ESP_LOGI(TAG, "HMI: Requesting confirmation to reset tank intake (hardware)");
                confirmation_action = CONFIRM_TEMPESTA_RESET_TANK_TOTAL;
                confirmation_param = 0;
                confirmation_return_menu = HMI_MENU_TEMPESTA_CONTROLS;
                hmi_status.current_menu = HMI_MENU_CONFIRM;
                hmi_status.selected_item = 0;
            } else if (hmi_status.selected_item == 7) {
                // Reset ALL - show confirmation
                ESP_LOGI(TAG, "HMI: Requesting confirmation to reset ALL counters (hardware)");
                confirmation_action = CONFIRM_TEMPESTA_RESET_ALL;
                confirmation_param = 0;
                confirmation_return_menu = HMI_MENU_TEMPESTA_CONTROLS;
                hmi_status.current_menu = HMI_MENU_CONFIRM;
                hmi_status.selected_item = 0;
            }
            break;

        // Shared confirmation dialog - execute or cancel
        case HMI_MENU_CONFIRM:
            if (hmi_status.selected_item == 0) {
                // Yes - execute action
                switch (confirmation_action) {
                    case CONFIRM_TEMPESTA_RESET_DAILY:
                        ESP_LOGI(TAG, "HMI: Resetting daily counters");
                        tempesta_reset_daily_counters();
                        break;

                    case CONFIRM_TEMPESTA_RESET_WEEKLY:
                        ESP_LOGI(TAG, "HMI: Resetting weekly counters");
                        tempesta_reset_weekly_counters();
                        break;

                    case CONFIRM_TEMPESTA_RESET_RAIN_TOTAL:
                        ESP_LOGI(TAG, "HMI: Resetting rain gauge hardware counter");
                        tempesta_reset_rain_gauge_total();
                        break;

                    case CONFIRM_TEMPESTA_RESET_TANK_TOTAL:
                        ESP_LOGI(TAG, "HMI: Resetting tank intake hardware counter");
                        tempesta_reset_tank_intake_total();
                        break;

                    case CONFIRM_TEMPESTA_RESET_ALL:
                        ESP_LOGI(TAG, "HMI: Resetting ALL hardware counters");
                        tempesta_reset_all_counters();
                        break;

                    case CONFIRM_RESET_ZONE_LEARNING:
                        ESP_LOGI(TAG, "HMI: Resetting learning for zone %d", confirmation_param + 1);
                        impluvium_reset_zone_learning(confirmation_param);
                        break;

                    case CONFIRM_RESET_ALL_LEARNING:
                        ESP_LOGI(TAG, "HMI: Resetting learning for all zones");
                        impluvium_reset_all_learning();
                        break;

                    case CONFIRM_MANUAL_WATER:
                        ESP_LOGI(TAG, "HMI: Starting manual water for zone %d, duration %ds",
                                 confirmation_param + 1, manual_water_duration);
                        esp_err_t ret = impluvium_force_water_zone(confirmation_param, manual_water_duration);
                        if (ret != ESP_OK) {
                            ESP_LOGW(TAG, "HMI: Manual water failed (system busy or not implemented)");
                        }
                        break;

                    case CONFIRM_SYSTEM_FLUSH_RESET:
                        ESP_LOGI(TAG, "HMI: Flushing MQTT buffer to FLASH and restarting MCU");
                        uint16_t flushed = 0;
                        esp_err_t flush_ret = telemetry_manual_flush_to_flash(&flushed);
                        if (flush_ret == ESP_OK) {
                            ESP_LOGI(TAG, "Successfully flushed %d messages to FLASH", flushed);
                        } else {
                            ESP_LOGW(TAG, "Failed to flush messages: %s", esp_err_to_name(flush_ret));
                        }
                        // Small delay to allow log output
                        vTaskDelay(pdMS_TO_TICKS(500));
                        esp_restart();
                        break;

                    default:
                        break;
                }
                // Return to the menu that triggered the confirmation
                hmi_status.current_menu = confirmation_return_menu;
                confirmation_action = CONFIRM_NONE;
                hmi_status.selected_item = 0;
            } else if (hmi_status.selected_item == 1) {
                // No - cancel action
                ESP_LOGI(TAG, "HMI: Confirmation cancelled");
                hmi_status.current_menu = confirmation_return_menu;
                confirmation_action = CONFIRM_NONE;
                hmi_status.selected_item = 0;
            }
            break;

        // IMPLUVIUM System Controls page - execute actions
        case HMI_MENU_IMPLUVIUM_CONTROLS:
            if (hmi_status.selected_item == 0) {
                // Back
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM;
                hmi_status.selected_item = 0;
            } else if (hmi_status.selected_item == 1) {
                // Toggle System Enable/Disable
                impluvium_snapshot_t data;
                telemetry_get_impluvium_data(&data);

                if (data.load_shed_shutdown) {
                    ESP_LOGI(TAG, "HMI: Enabling IMPLUVIUM system");
                    impluvium_set_system_enabled(true);
                } else {
                    ESP_LOGI(TAG, "HMI: Disabling IMPLUVIUM system");
                    impluvium_set_system_enabled(false);
                }
            } else if (hmi_status.selected_item == 2) {
                // Force Moisture Check Now
                ESP_LOGI(TAG, "HMI: Forcing immediate moisture check");
                esp_err_t ret = impluvium_force_moisture_check();
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "HMI: Force check failed (system busy or disabled)");
                }
            } else if (hmi_status.selected_item == 3) {
                // Emergency Stop Reset
                ESP_LOGI(TAG, "HMI: Clearing emergency stop flag");
                impluvium_clear_emergency_stop();
            } else if (hmi_status.selected_item == 4) {
                // Clear Diagnostics
                ESP_LOGI(TAG, "HMI: Clearing diagnostic state");
                impluvium_clear_diagnostics();
            } else if (hmi_status.selected_item == 5) {
                // Reset All Learning - show confirmation
                ESP_LOGI(TAG, "HMI: Requesting confirmation to reset all learning data");
                confirmation_action = CONFIRM_RESET_ALL_LEARNING;
                confirmation_param = 0;
                confirmation_return_menu = HMI_MENU_IMPLUVIUM_CONTROLS;
                hmi_status.current_menu = HMI_MENU_CONFIRM;
                hmi_status.selected_item = 0;
            }
            break;

        // IMPLUVIUM detail pages - return to IMPLUVIUM main menu
        case HMI_MENU_IMPLUVIUM_OVERVIEW:
        case HMI_MENU_IMPLUVIUM_STATISTICS:
        case HMI_MENU_IMPLUVIUM_MONITOR:
            hmi_status.current_menu = HMI_MENU_IMPLUVIUM;
            hmi_status.selected_item = 0;
            break;

        // IMPLUVIUM Zone detail pages - return to Zones submenu
        case HMI_MENU_IMPLUVIUM_ZONES_ALL:
        case HMI_MENU_IMPLUVIUM_ZONE_1:
        case HMI_MENU_IMPLUVIUM_ZONE_2:
        case HMI_MENU_IMPLUVIUM_ZONE_3:
        case HMI_MENU_IMPLUVIUM_ZONE_4:
        case HMI_MENU_IMPLUVIUM_ZONE_5:
            hmi_status.current_menu = HMI_MENU_IMPLUVIUM_ZONES;
            hmi_status.selected_item = 0;
            break;

        // IMPLUVIUM Learning detail pages - return to Learning submenu
        case HMI_MENU_IMPLUVIUM_LEARNING_ALL:
        case HMI_MENU_IMPLUVIUM_LEARNING_1:
        case HMI_MENU_IMPLUVIUM_LEARNING_2:
        case HMI_MENU_IMPLUVIUM_LEARNING_3:
        case HMI_MENU_IMPLUVIUM_LEARNING_4:
        case HMI_MENU_IMPLUVIUM_LEARNING_5:
            hmi_status.current_menu = HMI_MENU_IMPLUVIUM_LEARNING;
            hmi_status.selected_item = 0;
            break;

        // IMPLUVIUM Zone Config page - enter zone edit mode
        case HMI_MENU_IMPLUVIUM_ZONE_CONFIG:
            if (hmi_status.selected_item == 0) {
                // Back
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM;
                hmi_status.selected_item = 0;
            } else {
                // Enter zone edit for selected zone (items 1-5 = zones 0-4)
                editing_zone_id = hmi_status.selected_item - 1;

                // Load current zone config from TELEMETRY
                impluvium_snapshot_t data;
                telemetry_get_impluvium_data(&data);
                editing_zone_enabled = data.zones[editing_zone_id].watering_enabled;
                editing_zone_target = (uint8_t)data.zones[editing_zone_id].target_moisture_percent;
                editing_zone_deadband = (uint8_t)data.zones[editing_zone_id].moisture_deadband_percent;

                zone_editing = false;  // Start in navigation mode
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_ZONE_EDIT;
                hmi_status.selected_item = 0;
            }
            break;

        // IMPLUVIUM Zone Edit page - field actions
        case HMI_MENU_IMPLUVIUM_ZONE_EDIT:
            if (hmi_status.selected_item == 0) {
                // Cancel - go back to zone config
                zone_editing = false;
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_ZONE_CONFIG;
                hmi_status.selected_item = 0;
            } else if (hmi_status.selected_item == 1) {
                // Toggle Enabled
                editing_zone_enabled = !editing_zone_enabled;
                ESP_LOGI(TAG, "HMI: Zone %d enabled toggled to %d", editing_zone_id + 1, editing_zone_enabled);
            } else if (hmi_status.selected_item == 2) {
                // Target moisture - toggle editing mode
                zone_editing = !zone_editing;
                ESP_LOGI(TAG, "HMI: Zone %d target editing mode: %d", editing_zone_id + 1, zone_editing);
            } else if (hmi_status.selected_item == 3) {
                // Deadband - toggle editing mode
                zone_editing = !zone_editing;
                ESP_LOGI(TAG, "HMI: Zone %d deadband editing mode: %d", editing_zone_id + 1, zone_editing);
            } else if (hmi_status.selected_item == 4) {
                // Reset Learning - show confirmation
                confirmation_action = CONFIRM_RESET_ZONE_LEARNING;
                confirmation_param = editing_zone_id;
                confirmation_return_menu = HMI_MENU_IMPLUVIUM_ZONE_EDIT;
                hmi_status.current_menu = HMI_MENU_CONFIRM;
                hmi_status.selected_item = 0;
            } else if (hmi_status.selected_item == 5) {
                // Manual Water - go to manual water input page
                manual_water_zone = editing_zone_id;
                manual_water_duration = 30;  // Reset to default
                manual_water_input = false;
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_MANUAL_WATER;
                hmi_status.selected_item = 0;
            } else if (hmi_status.selected_item == 6) {
                // Save - apply changes
                ESP_LOGI(TAG, "HMI: Saving zone %d config: enabled=%d, target=%d%%, deadband=%d%%",
                         editing_zone_id + 1, editing_zone_enabled, editing_zone_target, editing_zone_deadband);

                impluvium_update_zone_config(editing_zone_id,
                                            (float)editing_zone_target,
                                            (float)editing_zone_deadband,
                                            editing_zone_enabled);

                zone_editing = false;
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_ZONE_CONFIG;
                hmi_status.selected_item = 0;
            }
            break;

        // IMPLUVIUM Manual Water page - duration input and start
        case HMI_MENU_IMPLUVIUM_MANUAL_WATER:
            if (hmi_status.selected_item == 0) {
                // Duration - toggle editing mode
                manual_water_input = !manual_water_input;
                ESP_LOGI(TAG, "HMI: Manual water duration editing mode: %d", manual_water_input);
            } else if (hmi_status.selected_item == 1) {
                // Start - show confirmation
                confirmation_action = CONFIRM_MANUAL_WATER;
                confirmation_param = manual_water_zone;
                confirmation_return_menu = HMI_MENU_IMPLUVIUM_ZONE_EDIT;
                hmi_status.current_menu = HMI_MENU_CONFIRM;
                hmi_status.selected_item = 0;
            } else if (hmi_status.selected_item == 2) {
                // Cancel - go back to zone edit
                manual_water_input = false;
                hmi_status.current_menu = HMI_MENU_IMPLUVIUM_ZONE_EDIT;
                hmi_status.selected_item = 0;
            }
            break;

        // STELLARIA Control page - interactive controls
        case HMI_MENU_STELLARIA_CONTROL:
            if (hmi_status.selected_item == 0) {
                // Back - exit editing mode if active
                stellaria_intensity_editing = false;
                hmi_status.current_menu = HMI_MENU_STELLARIA;
                hmi_status.selected_item = 0;
            } else if (hmi_status.selected_item == 1) {
                // Toggle Enable/Disable
                stellaria_snapshot_t data;
                telemetry_get_stellaria_data(&data);

                if (data.state == STELLARIA_STATE_DISABLED || data.state == STELLARIA_STATE_SHUTDOWN) {
                    ESP_LOGI(TAG, "HMI: Enabling STELLARIA");
                    stellaria_enable();
                } else {
                    ESP_LOGI(TAG, "HMI: Disabling STELLARIA");
                    stellaria_disable();
                }
            } else if (hmi_status.selected_item == 2) {
                // Toggle intensity editing mode
                if (stellaria_intensity_editing) {
                    // Exit edit mode and apply the value
                    stellaria_intensity_editing = false;

                    // Convert percentage to PWM value (0-100% → 0-1023)
                    uint16_t pwm_value = (uint16_t)((stellaria_intensity_percent * 1023) / 100);

                    ESP_LOGI(TAG, "HMI: Setting STELLARIA intensity to %d%% (PWM: %d)",
                             stellaria_intensity_percent, pwm_value);
                    stellaria_set_intensity(pwm_value);
                } else {
                    // Enter edit mode - load current intensity from telemetry
                    stellaria_snapshot_t data;
                    telemetry_get_stellaria_data(&data);

                    // Convert current PWM to percentage (0-1023 → 0-100%)
                    stellaria_intensity_percent = (uint8_t)((data.target_intensity * 100) / 1023);

                    stellaria_intensity_editing = true;
                    ESP_LOGI(TAG, "HMI: Entering intensity edit mode (current: %d%%)",
                             stellaria_intensity_percent);
                }
            }
            break;

        case HMI_MENU_STELLARIA_STATUS:
            hmi_status.current_menu = HMI_MENU_STELLARIA;
            hmi_status.selected_item = 0;
            break;

        // STELLARIA Auto Mode page - interactive toggle
        case HMI_MENU_STELLARIA_AUTO:
            if (hmi_status.selected_item == 0) {
                // Back
                hmi_status.current_menu = HMI_MENU_STELLARIA;
                hmi_status.selected_item = 0;
            } else if (hmi_status.selected_item == 1) {
                // Toggle Auto Mode
                stellaria_snapshot_t data;
                telemetry_get_stellaria_data(&data);

                if (data.auto_mode_active) {
                    ESP_LOGI(TAG, "HMI: Disabling STELLARIA auto mode");
                    stellaria_set_auto_mode(false);
                } else {
                    ESP_LOGI(TAG, "HMI: Enabling STELLARIA auto mode");
                    stellaria_set_auto_mode(true);
                }
            }
            break;

        // SYSTEM menu - navigate to Info or Controls
        case HMI_MENU_SYSTEM:
            if (hmi_status.selected_item == 0) {
                // Back
                hmi_status.current_menu = HMI_MENU_MAIN;
            } else if (hmi_status.selected_item == 1) {
                // System Info
                hmi_status.current_menu = HMI_MENU_SYSTEM_INFO;
            } else if (hmi_status.selected_item == 2) {
                // Controls
                hmi_status.current_menu = HMI_MENU_SYSTEM_CONTROLS;
            }
            hmi_status.selected_item = 0;
            break;

        // SYSTEM Info page - just back button
        case HMI_MENU_SYSTEM_INFO:
            hmi_status.current_menu = HMI_MENU_SYSTEM;
            hmi_status.selected_item = 0;
            break;

        // SYSTEM Controls page - Flush & Reset, WiFi Reconnect
        case HMI_MENU_SYSTEM_CONTROLS:
            if (hmi_status.selected_item == 0) {
                // Back
                hmi_status.current_menu = HMI_MENU_SYSTEM;
                hmi_status.selected_item = 0;
            } else if (hmi_status.selected_item == 1) {
                // Flush & Reset - show confirmation
                ESP_LOGI(TAG, "HMI: Requesting confirmation to flush buffer and restart MCU");
                confirmation_action = CONFIRM_SYSTEM_FLUSH_RESET;
                confirmation_param = 0;
                confirmation_return_menu = HMI_MENU_SYSTEM_CONTROLS;
                hmi_status.current_menu = HMI_MENU_CONFIRM;
                hmi_status.selected_item = 0;
            } else if (hmi_status.selected_item == 2) {
                // WiFi Reconnect - immediate action
                ESP_LOGI(TAG, "HMI: Forcing WiFi reconnection");
                wifi_helper_force_reconnect();
                // Stay on this page to see the effect
            }
            break;

        default:
            // Fallback: go to main menu
            hmi_status.current_menu = HMI_MENU_MAIN;
            hmi_status.selected_item = 0;
            break;
    }

    hmi_update_activity();
}

// ########################## Menu Rendering Functions ##########################

/**
 * @brief Check if current menu is a realtime page requiring fast refresh
 */
static bool hmi_is_realtime_page(void)
{
    return (hmi_status.current_menu == HMI_MENU_FLUCTUS_LIVE_POWER ||
            hmi_status.current_menu == HMI_MENU_FLUCTUS_SOLAR_DEBUG ||
            hmi_status.current_menu == HMI_MENU_IMPLUVIUM_MONITOR);
}

/**
 * @brief Render current menu to display
 */
static void hmi_render_menu(void)
{
    fb_clear();

    switch (hmi_status.current_menu) {
        case HMI_MENU_MAIN:
            hmi_render_main_menu();
            break;

        case HMI_MENU_FLUCTUS:
            hmi_render_fluctus_menu();
            break;
        case HMI_MENU_FLUCTUS_OVERVIEW:
            hmi_render_fluctus_overview_page();
            break;
        case HMI_MENU_FLUCTUS_ENERGY:
            hmi_render_fluctus_energy_page();
            break;
        case HMI_MENU_FLUCTUS_LIVE_POWER:
            hmi_render_fluctus_live_power_page();
            break;
        case HMI_MENU_FLUCTUS_BUSES:
            hmi_render_fluctus_buses_page();
            break;
        case HMI_MENU_FLUCTUS_TRACKING:
            hmi_render_fluctus_tracking_page();
            break;
        case HMI_MENU_FLUCTUS_SOLAR_DEBUG:
            hmi_render_fluctus_solar_debug_page();
            break;
        case HMI_MENU_FLUCTUS_CONTROLS:
            hmi_render_fluctus_controls_page();
            break;

        case HMI_MENU_TEMPESTA:
            hmi_render_tempesta_menu();
            break;
        case HMI_MENU_TEMPESTA_ENV:
            hmi_render_tempesta_env_page();
            break;
        case HMI_MENU_TEMPESTA_WIND:
            hmi_render_tempesta_wind_page();
            break;
        case HMI_MENU_TEMPESTA_RAIN:
            hmi_render_tempesta_rain_page();
            break;
        case HMI_MENU_TEMPESTA_TANK:
            hmi_render_tempesta_tank_page();
            break;
        case HMI_MENU_TEMPESTA_AIR:
            hmi_render_tempesta_air_page();
            break;
        case HMI_MENU_TEMPESTA_CONTROLS:
            hmi_render_tempesta_controls_page();
            break;

        case HMI_MENU_IMPLUVIUM:
            hmi_render_impluvium_menu();
            break;
        case HMI_MENU_IMPLUVIUM_OVERVIEW:
            hmi_render_impluvium_overview_page();
            break;
        case HMI_MENU_IMPLUVIUM_STATISTICS:
            hmi_render_impluvium_statistics_page();
            break;
        case HMI_MENU_IMPLUVIUM_ZONES:
            hmi_render_impluvium_zones_menu();
            break;
        case HMI_MENU_IMPLUVIUM_ZONES_ALL:
            hmi_render_impluvium_zones_all_page();
            break;
        case HMI_MENU_IMPLUVIUM_ZONE_1:
            hmi_render_impluvium_zone_1_page();
            break;
        case HMI_MENU_IMPLUVIUM_ZONE_2:
            hmi_render_impluvium_zone_2_page();
            break;
        case HMI_MENU_IMPLUVIUM_ZONE_3:
            hmi_render_impluvium_zone_3_page();
            break;
        case HMI_MENU_IMPLUVIUM_ZONE_4:
            hmi_render_impluvium_zone_4_page();
            break;
        case HMI_MENU_IMPLUVIUM_ZONE_5:
            hmi_render_impluvium_zone_5_page();
            break;
        case HMI_MENU_IMPLUVIUM_LEARNING:
            hmi_render_impluvium_learning_menu();
            break;
        case HMI_MENU_IMPLUVIUM_LEARNING_ALL:
            hmi_render_impluvium_learning_all_page();
            break;
        case HMI_MENU_IMPLUVIUM_LEARNING_1:
            hmi_render_impluvium_learning_1_page();
            break;
        case HMI_MENU_IMPLUVIUM_LEARNING_2:
            hmi_render_impluvium_learning_2_page();
            break;
        case HMI_MENU_IMPLUVIUM_LEARNING_3:
            hmi_render_impluvium_learning_3_page();
            break;
        case HMI_MENU_IMPLUVIUM_LEARNING_4:
            hmi_render_impluvium_learning_4_page();
            break;
        case HMI_MENU_IMPLUVIUM_LEARNING_5:
            hmi_render_impluvium_learning_5_page();
            break;
        case HMI_MENU_IMPLUVIUM_MONITOR:
            hmi_render_impluvium_monitor_page();
            break;
        case HMI_MENU_IMPLUVIUM_CONTROLS:
            hmi_render_impluvium_controls_page();
            break;
        case HMI_MENU_IMPLUVIUM_ZONE_CONFIG:
            hmi_render_impluvium_zone_config_page();
            break;
        case HMI_MENU_IMPLUVIUM_ZONE_EDIT:
            hmi_render_impluvium_zone_edit_page();
            break;
        case HMI_MENU_IMPLUVIUM_MANUAL_WATER:
            hmi_render_impluvium_manual_water_page();
            break;

        case HMI_MENU_STELLARIA:
            hmi_render_stellaria_menu();
            break;
        case HMI_MENU_STELLARIA_STATUS:
            hmi_render_stellaria_status_page();
            break;
        case HMI_MENU_STELLARIA_CONTROL:
            hmi_render_stellaria_control_page();
            break;
        case HMI_MENU_STELLARIA_AUTO:
            hmi_render_stellaria_auto_page();
            break;

        case HMI_MENU_SYSTEM:
            hmi_render_system_menu();
            break;
        case HMI_MENU_SYSTEM_INFO:
            hmi_render_system_info_page();
            break;
        case HMI_MENU_SYSTEM_CONTROLS:
            hmi_render_system_controls_page();
            break;

        case HMI_MENU_CONFIRM:
            hmi_render_confirm_page();
            break;

        default:
            fb_draw_string(20, 28, "Not Implemented", false);
            break;
    }

    fb_flush();
}

/**
 * @brief Render main menu (component selection)
 */
static void hmi_render_main_menu(void)
{
    // Title
    fb_draw_string(20, 2, "SOLARIUM v1.0", false);
    fb_draw_hline(0, 10, 128);

    // Menu items
    const char *items[] = {
        "> FLUCTUS",
        "  TEMPESTA",
        "  IMPLUVIUM",
        "  STELLARIA",
        "  SYSTEM"
    };

    int y = 14;
    for (int i = 0; i < 5; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            // Draw selection indicator
            fb_draw_string(2, y, ">", false);
        }
        fb_draw_string(8, y, items[i] + 2, selected);  // Skip the "> " prefix
        y += 10;
    }
}

/**
 * @brief Render FLUCTUS submenu
 */
static void hmi_render_fluctus_menu(void)
{
    // Get data from TELEMETRY cache
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    // Title
    fb_draw_string(30, 2, "FLUCTUS", false);
    fb_draw_hline(0, 10, 128);

    // Menu items (8 total: Back + 7 pages)
    const char *items[] = {
        "< Back",
        "Overview",
        "Energy",
        "Live Power",   // 4Hz REALTIME
        "Buses",
        "Tracking",
        "Solar Debug",  // 4Hz REALTIME
        "Controls"
    };

    int y = 14;
    for (int i = 0; i < 8; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            fb_draw_string(2, y, ">", false);
        }
        fb_draw_string(8, y, items[i], selected);

        // Add [LIVE] indicator for realtime pages (items 3 and 6)
        if (i == 3 || i == 6) {
            if (hmi_status.blink_state) {
                fb_draw_string(75, y, "[LIVE]", false);
            }
        }
        y += 8;  // Tighter spacing to fit 8 items (8*8=64 pixels total)
    }

    // No bottom status bar - not enough space with 8 items
}

/**
 * @brief Render IMPLUVIUM OVERVIEW page [1Hz]
 */
static void hmi_render_impluvium_overview_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    fb_draw_string(24, 2, "Overview", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // System state
    const char *state_str = "STANDBY";
    switch (data.state) {
        case IRRIGATION_STANDBY: state_str = "STANDBY"; break;
        case IRRIGATION_MEASURING: state_str = "MEASURING"; break;
        case IRRIGATION_WATERING: state_str = "WATERING"; break;
        case IRRIGATION_STOPPING: state_str = "STOPPING"; break;
        case IRRIGATION_MAINTENANCE: state_str = "MAINT!"; break;
        case IRRIGATION_DISABLED: state_str = "DISABLED"; break;
    }
    snprintf(buf, sizeof(buf), "State: %s", state_str);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // Water level
    snprintf(buf, sizeof(buf), "Water: %.0f%%", data.water_level_percent);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // Total water used today
    snprintf(buf, sizeof(buf), "Today: %.1fL", data.total_water_used_day_ml / 1000.0f);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // Watering events today
    snprintf(buf, sizeof(buf), "Events: %d", data.watering_events_day);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // Active zone (if any)
    if (data.active_zone != NO_ACTIVE_ZONE_ID) {
        snprintf(buf, sizeof(buf), "Active: Z%d", data.active_zone + 1);
        fb_draw_string(2, y, buf, false);
    }
}

/**
 * @brief Render IMPLUVIUM STATISTICS page [1Hz]
 */
static void hmi_render_impluvium_statistics_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    fb_draw_string(20, 2, "Statistics", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Header: Hr = current hour, Avg = average/hour, Day = total today
    fb_draw_string(2, y, "Hour (l) Avg (l/h) Day (l)", false);
    y += 10;

    // Zone statistics in 2-column layout
    // Left column: Z1, Z2, Z3
    // Right column: Z4, Z5
    int left_x = 2;
    int right_x = 66;
    y = 24; // Start data rows

    for (int i = 0; i < 3; i++) {
        // Left column (Z1-Z3)
        float vol_hour_l = data.zones[i].volume_used_hour_ml / 1000.0f;
        float avg_hour_l = data.zones[i].avg_hourly_consumption_ml / 1000.0f;
        float vol_day_l = data.zones[i].volume_used_today_ml / 1000.0f;

        snprintf(buf, sizeof(buf), "Z%d:%.1f %.1f %.1f", i + 1, vol_hour_l, avg_hour_l, vol_day_l);
        fb_draw_string(left_x, y, buf, false);

        // Right column (Z4-Z5, only first 2 iterations)
        if (i < 2) {
            int zone_idx = i + 3; // Z4=3, Z5=4
            vol_hour_l = data.zones[zone_idx].volume_used_hour_ml / 1000.0f;
            avg_hour_l = data.zones[zone_idx].avg_hourly_consumption_ml / 1000.0f;
            vol_day_l = data.zones[zone_idx].volume_used_today_ml / 1000.0f;

            snprintf(buf, sizeof(buf), "Z%d:%.1f %.1f %.1f", zone_idx + 1, vol_hour_l, avg_hour_l, vol_day_l);
            fb_draw_string(right_x, y, buf, false);
        }

        y += 10;
    }
}

/**
 * @brief Render IMPLUVIUM Zones submenu selector
 */
static void hmi_render_impluvium_zones_menu(void)
{
    fb_draw_string(28, 2, "Zones", false);
    fb_draw_hline(0, 10, 128);

    // Menu items (7 total: Back + All Summary + 5 zones)
    const char *items[] = {
        "< Back",
        "All Zones",
        "Zone 1",
        "Zone 2",
        "Zone 3",
        "Zone 4",
        "Zone 5"
    };

    int y = 14;
    for (int i = 0; i < 7; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            fb_draw_string(2, y, ">", false);
        }
        fb_draw_string(8, y, items[i], selected);
        y += 10;
    }
}

/**
 * @brief Render IMPLUVIUM All Zones Summary page
 */
static void hmi_render_impluvium_zones_all_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    fb_draw_string(16, 2, "All Zones", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Show all 5 zones with enable status and current moisture
    for (int i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        const char *enabled_str = data.zones[i].watering_enabled ? "EN" : "DS";
        snprintf(buf, sizeof(buf), "Z%d: %s %.0f%%/%.0f%%",
                 i + 1,
                 enabled_str,
                 data.zones[i].current_moisture_percent,
                 data.zones[i].target_moisture_percent);
        fb_draw_string(2, y, buf, false);
        y += 10;
    }
}

/**
 * @brief Render IMPLUVIUM Zone 1 detail page
 */
static void hmi_render_impluvium_zone_1_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    fb_draw_string(40, 2, "Zone 1", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Enabled status
    const char *status = data.zones[0].watering_enabled ? "Enabled" : "Disabled";
    snprintf(buf, sizeof(buf), "Status: %s", status);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // Current moisture
    snprintf(buf, sizeof(buf), "Moisture: %.0f%%", data.zones[0].current_moisture_percent);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // Target
    snprintf(buf, sizeof(buf), "Target: %.0f%%", data.zones[0].target_moisture_percent);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // Deadband
    snprintf(buf, sizeof(buf), "Deadband: %.0f%%", data.zones[0].moisture_deadband_percent);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // Volume used today
    snprintf(buf, sizeof(buf), "Today: %.1fL", data.zones[0].volume_used_today_ml / 1000.0f);
    fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 2 detail page
 */
static void hmi_render_impluvium_zone_2_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    fb_draw_string(40, 2, "Zone 2", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    const char *status = data.zones[1].watering_enabled ? "Enabled" : "Disabled";
    snprintf(buf, sizeof(buf), "Status: %s", status);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Moisture: %.0f%%", data.zones[1].current_moisture_percent);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Target: %.0f%%", data.zones[1].target_moisture_percent);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Deadband: %.0f%%", data.zones[1].moisture_deadband_percent);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Today: %.1fL", data.zones[1].volume_used_today_ml / 1000.0f);
    fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 3 detail page
 */
static void hmi_render_impluvium_zone_3_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    fb_draw_string(40, 2, "Zone 3", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    const char *status = data.zones[2].watering_enabled ? "Enabled" : "Disabled";
    snprintf(buf, sizeof(buf), "Status: %s", status);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Moisture: %.0f%%", data.zones[2].current_moisture_percent);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Target: %.0f%%", data.zones[2].target_moisture_percent);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Deadband: %.0f%%", data.zones[2].moisture_deadband_percent);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Today: %.1fL", data.zones[2].volume_used_today_ml / 1000.0f);
    fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 4 detail page
 */
static void hmi_render_impluvium_zone_4_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    fb_draw_string(40, 2, "Zone 4", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    const char *status = data.zones[3].watering_enabled ? "Enabled" : "Disabled";
    snprintf(buf, sizeof(buf), "Status: %s", status);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Moisture: %.0f%%", data.zones[3].current_moisture_percent);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Target: %.0f%%", data.zones[3].target_moisture_percent);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Deadband: %.0f%%", data.zones[3].moisture_deadband_percent);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Today: %.1fL", data.zones[3].volume_used_today_ml / 1000.0f);
    fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 5 detail page
 */
static void hmi_render_impluvium_zone_5_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    fb_draw_string(40, 2, "Zone 5", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    const char *status = data.zones[4].watering_enabled ? "Enabled" : "Disabled";
    snprintf(buf, sizeof(buf), "Status: %s", status);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Moisture: %.0f%%", data.zones[4].current_moisture_percent);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Target: %.0f%%", data.zones[4].target_moisture_percent);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Deadband: %.0f%%", data.zones[4].moisture_deadband_percent);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Today: %.1fL", data.zones[4].volume_used_today_ml / 1000.0f);
    fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Learning submenu selector
 */
static void hmi_render_impluvium_learning_menu(void)
{
    fb_draw_string(24, 2, "Learning", false);
    fb_draw_hline(0, 10, 128);

    // Menu items (7 total: Back + All Summary + 5 zones)
    const char *items[] = {
        "< Back",
        "All Zones",
        "Zone 1",
        "Zone 2",
        "Zone 3",
        "Zone 4",
        "Zone 5"
    };

    int y = 14;
    for (int i = 0; i < 7; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            fb_draw_string(2, y, ">", false);
        }
        fb_draw_string(8, y, items[i], selected);
        y += 10;
    }
}

/**
 * @brief Render IMPLUVIUM Learning Summary (all zones)
 */
static void hmi_render_impluvium_learning_all_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    fb_draw_string(10, 2, "Learn Summary", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Show all 5 zones with confidence and PPMP ratio
    for (int i = 0; i < IRRIGATION_ZONE_COUNT; i++) {
        snprintf(buf, sizeof(buf), "Z%d: %.0f%% %.1f",
                 i + 1,
                 data.zones[i].confidence_level * 100.0f,
                 data.zones[i].calculated_ppmp_ratio);
        fb_draw_string(2, y, buf, false);
        y += 10;
    }
}

/**
 * @brief Render IMPLUVIUM Zone 1 Learning detail
 */
static void hmi_render_impluvium_learning_1_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    fb_draw_string(24, 2, "Z1 Learning", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Confidence
    snprintf(buf, sizeof(buf), "Conf: %.0f%%", data.zones[0].confidence_level * 100.0f);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // PPMP ratio (use full name as requested)
    snprintf(buf, sizeof(buf), "PulsesPerMoisture%%: %.1f", data.zones[0].calculated_ppmp_ratio);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // Pump duty
    snprintf(buf, sizeof(buf), "Pump: %lu", data.zones[0].calculated_pump_duty);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // Predictions
    snprintf(buf, sizeof(buf), "Pred: %lu/%lu",
             data.zones[0].successful_predictions,
             data.zones[0].total_predictions);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // History entries
    snprintf(buf, sizeof(buf), "History: %d/15", data.zones[0].history_entry_count);
    fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 2 Learning detail
 */
static void hmi_render_impluvium_learning_2_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    fb_draw_string(24, 2, "Z2 Learning", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Conf: %.0f%%", data.zones[1].confidence_level * 100.0f);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "PulsesPerMoisture%%: %.1f", data.zones[1].calculated_ppmp_ratio);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pump: %lu", data.zones[1].calculated_pump_duty);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pred: %lu/%lu",
             data.zones[1].successful_predictions,
             data.zones[1].total_predictions);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "History: %d/15", data.zones[1].history_entry_count);
    fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 3 Learning detail
 */
static void hmi_render_impluvium_learning_3_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    fb_draw_string(24, 2, "Z3 Learning", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Conf: %.0f%%", data.zones[2].confidence_level * 100.0f);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "PulsesPerMoisture%%: %.1f", data.zones[2].calculated_ppmp_ratio);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pump: %lu", data.zones[2].calculated_pump_duty);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pred: %lu/%lu",
             data.zones[2].successful_predictions,
             data.zones[2].total_predictions);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "History: %d/15", data.zones[2].history_entry_count);
    fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 4 Learning detail
 */
static void hmi_render_impluvium_learning_4_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    fb_draw_string(24, 2, "Z4 Learning", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Conf: %.0f%%", data.zones[3].confidence_level * 100.0f);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "PulsesPerMoisture%%: %.1f", data.zones[3].calculated_ppmp_ratio);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pump: %lu", data.zones[3].calculated_pump_duty);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pred: %lu/%lu",
             data.zones[3].successful_predictions,
             data.zones[3].total_predictions);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "History: %d/15", data.zones[3].history_entry_count);
    fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render IMPLUVIUM Zone 5 Learning detail
 */
static void hmi_render_impluvium_learning_5_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    fb_draw_string(24, 2, "Z5 Learning", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Conf: %.0f%%", data.zones[4].confidence_level * 100.0f);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "PulsesPerMoisture%%: %.1f", data.zones[4].calculated_ppmp_ratio);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pump: %lu", data.zones[4].calculated_pump_duty);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pred: %lu/%lu",
             data.zones[4].successful_predictions,
             data.zones[4].total_predictions);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "History: %d/15", data.zones[4].history_entry_count);
    fb_draw_string(2, y, buf, false);
}

/**
 * @brief Render TEMPESTA submenu
 */
static void hmi_render_tempesta_menu(void)
{
    // Get data from TELEMETRY cache
    tempesta_snapshot_t data;
    telemetry_get_tempesta_data(&data);

    // Title
    fb_draw_string(28, 2, "TEMPESTA", false);
    fb_draw_hline(0, 10, 128);

    // Menu items
    const char *items[] = {
        "< Back",
        "Environment",
        "Wind",
        "Rain Gauge",
        "Tank Intake",
        "Air Quality",
        "Controls"
    };

    int y = 14;
    for (int i = 0; i < 7; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            fb_draw_string(2, y, ">", false);
        }
        fb_draw_string(8, y, items[i], selected);
        y += 10;
    }

    // Show state and quick status at bottom
    char buf[32];
    const char *state_str = "UNKNOWN";
    switch (data.state) {
        case TEMPESTA_STATE_DISABLED:   state_str = "OFF"; break;
        case TEMPESTA_STATE_IDLE:       state_str = "IDLE"; break;
        case TEMPESTA_STATE_READING:    state_str = "READ"; break;
        case TEMPESTA_STATE_POWER_SAVE: state_str = "P-SAVE"; break;
        case TEMPESTA_STATE_SHUTDOWN:   state_str = "SHTDWN"; break;
        case TEMPESTA_STATE_ERROR:      state_str = "ERROR!"; break;
    }
    snprintf(buf, sizeof(buf), "%s %.1fC %.0f%%", state_str, data.temperature, data.humidity);
    fb_draw_string(2, 56, buf, false);
}

/**
 * @brief Render IMPLUVIUM submenu
 */
static void hmi_render_impluvium_menu(void)
{
    // Get data from TELEMETRY cache
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    // Title
    fb_draw_string(24, 2, "IMPLUVIUM", false);
    fb_draw_hline(0, 10, 128);

    // Menu items (8 total: Back + 7 pages)
    const char *items[] = {
        "< Back",
        "Overview",
        "Statistics",
        "Zones",        // Submenu
        "Learning",     // Submenu
        "Monitor",      // 4Hz REALTIME
        "Controls",
        "Zone Config"
    };

    int y = 14;
    for (int i = 0; i < 8; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            fb_draw_string(2, y, ">", false);
        }
        fb_draw_string(8, y, items[i], selected);

        // Add [LIVE] indicator for Monitor (item 5)
        if (i == 5) {
            if (data.state == IRRIGATION_WATERING || data.state == IRRIGATION_MEASURING) {
                if (hmi_status.blink_state) {
                    fb_draw_string(70, y, "[LIVE]", false);
                }
            }
        }
        y += 8;  // Tighter spacing to fit 8 items
    }

    // No bottom status bar - not enough space with 8 items
}

/**
 * @brief Render STELLARIA submenu
 */
static void hmi_render_stellaria_menu(void)
{
    // Get data from TELEMETRY cache
    stellaria_snapshot_t data;
    telemetry_get_stellaria_data(&data);

    // Title
    fb_draw_string(26, 2, "STELLARIA", false);
    fb_draw_hline(0, 10, 128);

    // Menu items
    const char *items[] = {
        "< Back",
        "Status",
        "Manual Control",
        "Auto Mode"
    };

    int y = 14;
    for (int i = 0; i < 4; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            fb_draw_string(2, y, ">", false);
        }
        fb_draw_string(8, y, items[i], selected);
        y += 10;
    }

    // Show quick status at bottom
    char buf[32];
    snprintf(buf, sizeof(buf), "%s Int:%d%%",
             data.driver_enabled ? "ON " : "OFF",
             (data.current_intensity * 100) / 1023);
    fb_draw_string(2, 56, buf, false);
}

// ########################## Detail Page Renderers ##########################

/**
 * @brief Render FLUCTUS Overview page [1Hz]
 * Shows system state, thermal, and average SOC
 */
static void hmi_render_fluctus_overview_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    fb_draw_string(28, 2, "Overview", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Power state
    const char *state_str = "NORMAL";
    switch (data.power_state) {
        case FLUCTUS_POWER_STATE_NORMAL: state_str = "NORMAL"; break;
        case FLUCTUS_POWER_STATE_POWER_SAVING: state_str = "SAVE"; break;
        case FLUCTUS_POWER_STATE_LOW_POWER: state_str = "LOW"; break;
        case FLUCTUS_POWER_STATE_VERY_LOW: state_str = "VERY LOW"; break;
        case FLUCTUS_POWER_STATE_CRITICAL: state_str = "CRITICAL"; break;
        case FLUCTUS_POWER_STATE_SHUTDOWN: state_str = "SHUTDOWN"; break;
    }
    snprintf(buf, sizeof(buf), "State: %s", state_str);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // Safety status
    if (data.safety_shutdown) {
        fb_draw_string(2, y, "SAFETY SHUTDOWN!", false);
        y += 10;
    }

    // Battery SOC (15-min average)
    snprintf(buf, sizeof(buf), "Bat SOC: %.0f%% (avg)", data.battery_soc_avg_15min);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // Solar status
    snprintf(buf, sizeof(buf), "Solar: %s", data.solar_pv_active ? "ACTIVE" : "OFF");
    fb_draw_string(2, y, buf, false);
    y += 10;

    // Thermal
    snprintf(buf, sizeof(buf), "Temp: %.1fC Fan:%d%%",
             data.case_temperature, data.fan_speed_percent);
    fb_draw_string(2, y, buf, false);

    fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render FLUCTUS Energy page [1Hz]
 * Shows hourly and daily energy statistics
 */
static void hmi_render_fluctus_energy_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    fb_draw_string(32, 2, "Energy", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Hourly stats
    fb_draw_string(2, y, "=== HOURLY ===", false);
    y += 9;

    snprintf(buf, sizeof(buf), "PV: %dWh Pk:%dW",
             (int)data.pv_energy_wh_hour, (int)data.pv_peak_w_hour);
    fb_draw_string(2, y, buf, false);
    y += 9;

    snprintf(buf, sizeof(buf), "Bat:%dWh Pk:%dW",
             (int)data.battery_energy_wh_hour, (int)data.battery_peak_w_hour);
    fb_draw_string(2, y, buf, false);
    y += 11;

    // Daily stats
    fb_draw_string(2, y, "=== DAILY ===", false);
    y += 9;

    snprintf(buf, sizeof(buf), "PV: %.1fkWh Pk:%dW",
             data.pv_energy_wh_day / 1000.0f, (int)data.pv_peak_w_day);
    fb_draw_string(2, y, buf, false);
    y += 9;

    snprintf(buf, sizeof(buf), "Bat:%.1fkWh Pk:%dW",
             data.battery_consumed_wh_day / 1000.0f, (int)data.battery_peak_w_day);
    fb_draw_string(2, y, buf, false);

    fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render FLUCTUS Live Power page [4Hz REALTIME]
 * Shows both instantaneous and 15-min average power values
 */
static void hmi_render_fluctus_live_power_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    // Title with [LIVE] indicator
    fb_draw_string(22, 2, "Live Power", false);
    if (hmi_status.blink_state) {
        fb_draw_string(82, 2, "[LIVE]", false);
    }
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Battery - instantaneous
    fb_draw_string(2, y, "=== BATTERY NOW ===", false);
    y += 9;
    snprintf(buf, sizeof(buf), "%.2fV %.2fA %.1fW",
             data.battery_voltage_inst, data.battery_current_inst, data.battery_power_inst);
    fb_draw_string(2, y, buf, false);
    y += 9;

    // Battery - 15min average
    snprintf(buf, sizeof(buf), "15m: %.1fW %.0f%%",
             data.battery_power_avg_15min, data.battery_soc_avg_15min);
    fb_draw_string(2, y, buf, false);
    y += 11;

    // Solar - instantaneous
    fb_draw_string(2, y, "=== SOLAR NOW ===", false);
    y += 9;
    snprintf(buf, sizeof(buf), "%.1fV %.2fA %.1fW",
             data.solar_voltage_inst, data.solar_current_inst, data.solar_power_inst);
    fb_draw_string(2, y, buf, false);
    y += 9;

    // Solar - 15min average
    snprintf(buf, sizeof(buf), "15m: %.1fW", data.solar_power_avg_15min);
    fb_draw_string(2, y, buf, false);

    fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render FLUCTUS Buses page [1Hz]
 * Shows power bus enable states and consumer counts
 */
static void hmi_render_fluctus_buses_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    fb_draw_string(36, 2, "Buses", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // 3.3V bus
    snprintf(buf, sizeof(buf), "3.3V: %s (%d)",
             data.bus_3v3_enabled ? "ON " : "OFF",
             data.bus_3v3_consumers);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // 5V bus
    snprintf(buf, sizeof(buf), "5V:   %s (%d)",
             data.bus_5v_enabled ? "ON " : "OFF",
             data.bus_5v_consumers);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // 6.6V bus
    snprintf(buf, sizeof(buf), "6.6V: %s (%d)",
             data.bus_6v6_enabled ? "ON " : "OFF",
             data.bus_6v6_consumers);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // 12V bus
    snprintf(buf, sizeof(buf), "12V:  %s (%d)",
             data.bus_12v_enabled ? "ON " : "OFF",
             data.bus_12v_consumers);
    fb_draw_string(2, y, buf, false);

    fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render FLUCTUS Tracking page [1Hz]
 * Shows solar tracking position and state
 */
static void hmi_render_fluctus_tracking_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    fb_draw_string(28, 2, "Tracking", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // State
    const char *state_str = "DISABLED";
    switch (data.tracking_state) {
        case SOLAR_TRACKING_DISABLED: state_str = "DISABLED"; break;
        case SOLAR_TRACKING_SLEEPING: state_str = "SLEEPING"; break;
        case SOLAR_TRACKING_STANDBY: state_str = "STANDBY"; break;
        case SOLAR_TRACKING_CORRECTING: state_str = "CORRECT"; break;
        case SOLAR_TRACKING_PARKING: state_str = "PARKING"; break;
        case SOLAR_TRACKING_ERROR: state_str = "ERROR"; break;
    }
    snprintf(buf, sizeof(buf), "State: %s", state_str);
    fb_draw_string(2, y, buf, false);
    y += 12;

    // Positions
    snprintf(buf, sizeof(buf), "Yaw:   %d%% (azimuth)", data.yaw_position_percent);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pitch: %d%% (elev)", data.pitch_position_percent);
    fb_draw_string(2, y, buf, false);
    y += 12;

    // Status message
    if (data.tracking_state == SOLAR_TRACKING_STANDBY) {
        fb_draw_string(2, y, "Next: 12 min", false);
    } else if (data.tracking_state == SOLAR_TRACKING_SLEEPING) {
        fb_draw_string(2, y, "Awaiting sunrise", false);
    }

    fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render FLUCTUS Solar Debug page [4Hz REALTIME]
 * Shows tracking errors, photoresistors, and servo duties
 */
static void hmi_render_fluctus_solar_debug_page(void)
{
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    // Title with [LIVE] indicator
    fb_draw_string(16, 2, "Solar Debug", false);
    if (hmi_status.blink_state) {
        fb_draw_string(80, 2, "[LIVE]", false);
    }
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Position with errors
    snprintf(buf, sizeof(buf), "Yaw:%d%% Err:%+.2fV",
             data.yaw_position_percent, data.yaw_error);
    fb_draw_string(2, y, buf, false);
    y += 9;

    snprintf(buf, sizeof(buf), "Pit:%d%% Err:%+.2fV",
             data.pitch_position_percent, data.pitch_error);
    fb_draw_string(2, y, buf, false);
    y += 9;

    // Servo duties
    snprintf(buf, sizeof(buf), "Duty: Y=%lu P=%lu",
             data.current_yaw_duty, data.current_pitch_duty);
    fb_draw_string(2, y, buf, false);
    y += 11;

    // Photoresistors (2x2 grid)
    fb_draw_string(2, y, "Photoresistors:", false);
    y += 9;
    snprintf(buf, sizeof(buf), "NE:%d SE:%d",
             (int)data.photoresistor_readings[0], (int)data.photoresistor_readings[1]);
    fb_draw_string(2, y, buf, false);
    y += 9;
    snprintf(buf, sizeof(buf), "NW:%d SW:%d",
             (int)data.photoresistor_readings[2], (int)data.photoresistor_readings[3]);
    fb_draw_string(2, y, buf, false);

    fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render FLUCTUS Controls page (interactive actions)
 */
static void hmi_render_fluctus_controls_page(void)
{
    // Get current system state
    fluctus_snapshot_t data;
    telemetry_get_fluctus_data(&data);

    // Title
    fb_draw_string(30, 2, "Controls", false);
    fb_draw_hline(0, 10, 128);

    // Menu items with current state indicators
    const char *items[3];
    items[0] = "< Back";

    // Toggle solar tracking item (shows current state)
    if (data.tracking_state == SOLAR_TRACKING_DISABLED) {
        items[1] = "Enable Tracking";
    } else if (data.tracking_state == SOLAR_TRACKING_SLEEPING) {
        items[1] = "Disable (SLEEP)";
    } else {
        items[1] = "Disable Tracking";
    }

    // Manual safety reset
    if (data.safety_shutdown) {
        items[2] = "Safety Reset!";  // Urgent indicator
    } else {
        items[2] = "Safety Reset";
    }

    // Render menu items
    int y = 14;
    for (int i = 0; i < 3; i++) {
        bool selected = (i == hmi_status.selected_item);

        // Selection indicator
        if (selected) {
            fb_draw_string(2, y, ">", false);
        }

        // Item text
        fb_draw_string(8, y, items[i], selected);

        y += 14;  // Larger spacing for better visibility
    }

    fb_draw_string(2, 56, "Press to execute", false);
}

/**
 * @brief Render TEMPESTA Environment detail page
 */
static void hmi_render_tempesta_env_page(void)
{
    tempesta_snapshot_t data;
    telemetry_get_tempesta_data(&data);

    fb_draw_string(24, 2, "Environment", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Temp:     %.1fC", data.temperature);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Humidity: %.0f%%", data.humidity);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Pressure: %.0fhPa", data.pressure);
    fb_draw_string(2, y, buf, false);

    fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render TEMPESTA Wind detail page
 */
static void hmi_render_tempesta_wind_page(void)
{
    tempesta_snapshot_t data;
    telemetry_get_tempesta_data(&data);

    fb_draw_string(34, 2, "Wind", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Speed: %.1fm/s", data.wind_speed_ms);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "       %.0fRPM", data.wind_speed_rpm);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Dir:   %.0fdeg", data.wind_direction_deg);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "       %s", data.wind_direction_cardinal);
    fb_draw_string(2, y, buf, false);

    fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render TEMPESTA Air Quality detail page
 */
static void hmi_render_tempesta_air_page(void)
{
    tempesta_snapshot_t data;
    telemetry_get_tempesta_data(&data);

    fb_draw_string(20, 2, "Air Quality", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "PM2.5: %.0fug/m3", data.air_quality_pm25);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "PM10:  %.0fug/m3", data.air_quality_pm10);
    fb_draw_string(2, y, buf, false);

    fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render TEMPESTA Rain Gauge detail page
 */
static void hmi_render_tempesta_rain_page(void)
{
    tempesta_snapshot_t data;
    telemetry_get_tempesta_data(&data);

    fb_draw_string(22, 2, "Rain Gauge", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Last:  %.1fmm", data.rainfall_last_hour_mm);
    fb_draw_string(2, y, buf, false);
    y += 9;

    snprintf(buf, sizeof(buf), "Now:   %.1fmm", data.rainfall_current_hour_mm);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Daily: %.1fmm", data.rainfall_daily_mm);
    fb_draw_string(2, y, buf, false);
    y += 9;

    snprintf(buf, sizeof(buf), "Week:  %.1fmm", data.rainfall_weekly_mm);
    fb_draw_string(2, y, buf, false);

    fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render TEMPESTA Tank Intake detail page
 */
static void hmi_render_tempesta_tank_page(void)
{
    tempesta_snapshot_t data;
    telemetry_get_tempesta_data(&data);

    fb_draw_string(16, 2, "Tank Intake", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    snprintf(buf, sizeof(buf), "Last:  %.0fmL", data.tank_intake_last_hour_ml);
    fb_draw_string(2, y, buf, false);
    y += 9;

    snprintf(buf, sizeof(buf), "Now:   %.0fmL", data.tank_intake_current_hour_ml);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Daily: %.0fmL", data.tank_intake_daily_ml);
    fb_draw_string(2, y, buf, false);
    y += 9;

    snprintf(buf, sizeof(buf), "Week:  %.0fmL", data.tank_intake_weekly_ml);
    fb_draw_string(2, y, buf, false);

    fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render TEMPESTA System Controls page
 */
static void hmi_render_tempesta_controls_page(void)
{
    // Title
    fb_draw_string(4, 2, "System Controls", false);
    fb_draw_hline(0, 10, 128);

    // Menu items
    const char *items[] = {
        "< Back",
        "Enable/Disable",
        "Force Collection",
        "Reset Daily",
        "Reset Weekly",
        "Reset Rain Total",
        "Reset Tank Total",
        "Reset ALL"
    };

    int y = 14;
    for (int i = 0; i < 8; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            fb_draw_string(2, y, ">", false);
        }
        fb_draw_string(8, y, items[i], selected);
        y += 8;  // Tighter spacing for 8 items
    }
}

/**
 * @brief Render shared confirmation dialog (TEMPESTA, IMPLUVIUM, etc.)
 */
static void hmi_render_confirm_page(void)
{
    // Title and message based on action (shared by all components)
    const char *title = "Confirm?";
    const char *message = "";

    switch (confirmation_action) {
        case CONFIRM_TEMPESTA_RESET_DAILY:
            title = "Reset Daily?";
            message = "Daily counters→0";
            break;
        case CONFIRM_TEMPESTA_RESET_WEEKLY:
            title = "Reset Weekly?";
            message = "Weekly counters→0";
            break;
        case CONFIRM_TEMPESTA_RESET_RAIN_TOTAL:
            title = "Reset Rain?";
            message = "Hardware reset!";
            break;
        case CONFIRM_TEMPESTA_RESET_TANK_TOTAL:
            title = "Reset Tank?";
            message = "Hardware reset!";
            break;
        case CONFIRM_TEMPESTA_RESET_ALL:
            title = "Reset ALL?";
            message = "ALL hardware reset!";
            break;
        case CONFIRM_RESET_ZONE_LEARNING:
            title = "Reset Learning?";
            message = "Zone data will reset";
            break;
        case CONFIRM_RESET_ALL_LEARNING:
            title = "Reset All?";
            message = "All zones reset";
            break;
        case CONFIRM_MANUAL_WATER:
            title = "Start Watering?";
            message = "Safety override!";
            break;
        case CONFIRM_SYSTEM_FLUSH_RESET:
            title = "Flush & Reset?";
            message = "Save & restart MCU";
            break;
        default:
            break;
    }

    fb_draw_string(20, 12, title, false);
    fb_draw_string(10, 22, message, false);
    fb_draw_hline(0, 30, 128);

    int y = 36;

    // Yes option
    bool selected = (hmi_status.selected_item == 0);
    if (selected) fb_draw_string(30, y, ">", false);
    fb_draw_string(36, y, "Yes", selected);
    y += 10;

    // No option
    selected = (hmi_status.selected_item == 1);
    if (selected) fb_draw_string(30, y, ">", false);
    fb_draw_string(36, y, "No", selected);
}

/**
 * @brief Render IMPLUVIUM System Monitor detail page [LIVE]
 */
static void hmi_render_impluvium_monitor_page(void)
{
    impluvium_snapshot_t data;
    impluvium_snapshot_rt_t realtime;
    telemetry_get_impluvium_data(&data);
    telemetry_get_impluvium_realtime_data(&realtime);

    // Title with blink indicator
    fb_draw_string(20, 2, "Sys Monitor", false);
    if (hmi_status.blink_state) {
        fb_draw_string(80, 2, "*", false);
    }
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // State
    const char *state_str = "STANDBY";
    switch (data.state) {
        case IRRIGATION_STANDBY: state_str = "STANDBY"; break;
        case IRRIGATION_MEASURING: state_str = "MEASURING"; break;
        case IRRIGATION_WATERING: state_str = "WATERING"; break;
        case IRRIGATION_STOPPING: state_str = "STOPPING"; break;
        case IRRIGATION_MAINTENANCE: state_str = "MAINT!"; break;
        case IRRIGATION_DISABLED: state_str = "DISABLED"; break;
    }
    snprintf(buf, sizeof(buf), "%s", state_str);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // Pressure & Flow
    snprintf(buf, sizeof(buf), "P: %.1fbar", realtime.outlet_pressure_bar);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "F: %.0fL/h", realtime.current_flow_rate_lh);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // Pump
    snprintf(buf, sizeof(buf), "Pump: %lu%%",
             (realtime.pump_pwm_duty * 100) / 1023);
    fb_draw_string(2, y, buf, false);

    fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render IMPLUVIUM System Controls page (interactive actions)
 */
static void hmi_render_impluvium_controls_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    // Title
    fb_draw_string(4, 2, "System Controls", false);
    fb_draw_hline(0, 10, 128);

    // Menu items
    const char *items[] = {
        "< Back",
        data.load_shed_shutdown ? "Enable System" : "Disable System",
        "Force Check Now",
        data.emergency_stop ? "Emerg Reset!" : "Emerg Reset",
        "Clear Diag",
        "Reset Learning"
    };

    int y = 14;
    for (int i = 0; i < 6; i++) {
        bool selected = (i == hmi_status.selected_item);
        if (selected) {
            fb_draw_string(2, y, ">", false);
        }
        fb_draw_string(8, y, items[i], selected);
        y += 10;
    }
}

/**
 * @brief Render IMPLUVIUM Zone Config page (zone list)
 */
static void hmi_render_impluvium_zone_config_page(void)
{
    impluvium_snapshot_t data;
    telemetry_get_impluvium_data(&data);

    // Title
    fb_draw_string(12, 2, "Zone Config", false);
    fb_draw_hline(0, 10, 128);

    int y = 14;

    // Item 0: Back
    bool selected = (hmi_status.selected_item == 0);
    if (selected) fb_draw_string(2, y, ">", false);
    fb_draw_string(8, y, "< Back", selected);
    y += 10;

    // Items 1-5: Zones
    for (uint8_t i = 0; i < 5; i++) {
        selected = (hmi_status.selected_item == i + 1);
        if (selected) fb_draw_string(2, y, ">", false);

        char buf[32];
        const char *en_marker = data.zones[i].watering_enabled ? "[*]" : "[ ]";
        snprintf(buf, sizeof(buf), "Z%d:%s %.0f%% +/-%.0f%%",
                 i + 1,
                 en_marker,
                 data.zones[i].target_moisture_percent,
                 data.zones[i].moisture_deadband_percent);
        fb_draw_string(8, y, buf, selected);
        y += 10;
    }
}

/**
 * @brief Render IMPLUVIUM Zone Edit page
 */
static void hmi_render_impluvium_zone_edit_page(void)
{
    // Title
    char title[32];
    snprintf(title, sizeof(title), "Zone %d Config%s", editing_zone_id + 1, zone_editing ? "*" : "");
    fb_draw_string(12, 2, title, false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // Field 0: Cancel
    bool selected = (hmi_status.selected_item == 0);
    if (selected) fb_draw_string(2, y, ">", false);
    fb_draw_string(8, y, "< Cancel", selected);
    y += 8;

    // Field 1: Enabled
    selected = (hmi_status.selected_item == 1);
    if (selected) fb_draw_string(2, y, ">", false);
    const char *en_marker = editing_zone_enabled ? "[*]" : "[ ]";
    snprintf(buf, sizeof(buf), "Enabled: %s", en_marker);
    fb_draw_string(8, y, buf, selected);
    y += 8;

    // Field 2: Target moisture
    selected = (hmi_status.selected_item == 2);
    if (selected) fb_draw_string(2, y, ">", false);
    snprintf(buf, sizeof(buf), "Target:  %d%%", editing_zone_target);
    fb_draw_string(8, y, buf, selected);
    y += 8;

    // Field 3: Deadband
    selected = (hmi_status.selected_item == 3);
    if (selected) fb_draw_string(2, y, ">", false);
    snprintf(buf, sizeof(buf), "Deadband: %d%%", editing_zone_deadband);
    fb_draw_string(8, y, buf, selected);
    y += 8;

    // Field 4: Reset Learning
    selected = (hmi_status.selected_item == 4);
    if (selected) fb_draw_string(2, y, ">", false);
    fb_draw_string(8, y, "Reset Learning", selected);
    y += 8;

    // Field 5: Manual Water
    selected = (hmi_status.selected_item == 5);
    if (selected) fb_draw_string(2, y, ">", false);
    fb_draw_string(8, y, "Manual Water", selected);
    y += 8;

    // Field 6: Save
    selected = (hmi_status.selected_item == 6);
    if (selected) fb_draw_string(2, y, ">", false);
    fb_draw_string(8, y, "< Save", selected);
}

/**
 * @brief Render IMPLUVIUM Manual Water Time Input page
 */
static void hmi_render_impluvium_manual_water_page(void)
{
    // Title
    char title[32];
    snprintf(title, sizeof(title), "Manual Water Z%d*", manual_water_zone + 1);
    fb_draw_string(4, 2, title, false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 20;

    // Field 0: Duration
    bool selected = (hmi_status.selected_item == 0);
    if (selected) fb_draw_string(2, y, ">", false);
    snprintf(buf, sizeof(buf), "Duration: %ds", manual_water_duration);
    fb_draw_string(8, y, buf, selected);
    y += 12;

    // Field 1: Start
    selected = (hmi_status.selected_item == 1);
    if (selected) fb_draw_string(2, y, ">", false);
    fb_draw_string(8, y, "Start", selected);
    y += 12;

    // Field 2: Cancel
    selected = (hmi_status.selected_item == 2);
    if (selected) fb_draw_string(2, y, ">", false);
    fb_draw_string(8, y, "< Cancel", selected);

    // Footer instruction
    fb_draw_string(2, 56, "Rotate:adjust", false);
}

/**
 * @brief Render STELLARIA Status detail page
 */
static void hmi_render_stellaria_status_page(void)
{
    stellaria_snapshot_t data;
    telemetry_get_stellaria_data(&data);

    fb_draw_string(30, 2, "Status", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    const char *state_str = "DISABLED";
    switch (data.state) {
        case STELLARIA_STATE_DISABLED: state_str = "DISABLED"; break;
        case STELLARIA_STATE_ENABLED: state_str = "ENABLED"; break;
        case STELLARIA_STATE_POWER_SAVE: state_str = "PWR SAVE"; break;
        case STELLARIA_STATE_SHUTDOWN: state_str = "SHUTDOWN"; break;
        case STELLARIA_STATE_AUTO: state_str = "AUTO"; break;
    }

    snprintf(buf, sizeof(buf), "State: %s", state_str);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Current: %d%%",
             (data.current_intensity * 100) / 1023);
    fb_draw_string(2, y, buf, false);
    y += 10;

    snprintf(buf, sizeof(buf), "Target:  %d%%",
             (data.target_intensity * 100) / 1023);
    fb_draw_string(2, y, buf, false);

    fb_draw_string(2, 56, "< Press to go back", false);
}

/**
 * @brief Render STELLARIA Control detail page
 */
static void hmi_render_stellaria_control_page(void)
{
    // Get current system state
    stellaria_snapshot_t data;
    telemetry_get_stellaria_data(&data);

    // Title
    fb_draw_string(16, 2, "Manual Control", false);
    fb_draw_hline(0, 10, 128);

    // Menu items with current state indicators
    const char *items[3];
    char enable_text[32];
    char intensity_text[32];

    items[0] = "< Back";

    // Enable/Disable item (shows current state)
    const char *state_str = "OFF";
    if (data.state == STELLARIA_STATE_ENABLED || data.state == STELLARIA_STATE_POWER_SAVE) {
        state_str = "ON";
    } else if (data.state == STELLARIA_STATE_AUTO) {
        state_str = "AUTO";
    } else if (data.state == STELLARIA_STATE_SHUTDOWN) {
        state_str = "SHUTDOWN";
    }
    snprintf(enable_text, sizeof(enable_text), "Enable: %s", state_str);
    items[1] = enable_text;

    // Intensity item (shows current or editing value)
    if (stellaria_intensity_editing) {
        snprintf(intensity_text, sizeof(intensity_text), "Intensity: %d%% *", stellaria_intensity_percent);
    } else {
        uint8_t current_percent = (uint8_t)((data.target_intensity * 100) / 1023);
        snprintf(intensity_text, sizeof(intensity_text), "Intensity: %d%%", current_percent);
    }
    items[2] = intensity_text;

    // Render menu items
    int y = 14;
    for (int i = 0; i < 3; i++) {
        bool selected = (i == hmi_status.selected_item);

        // Selection indicator
        if (selected) {
            fb_draw_string(2, y, ">", false);
        }

        // Item text
        fb_draw_string(8, y, items[i], selected);

        y += 14;  // Larger spacing for better visibility
    }

    // Footer - context-sensitive help
    if (stellaria_intensity_editing) {
        fb_draw_string(2, 56, "Rotate:adjust Press:save", false);
    } else {
        fb_draw_string(2, 56, "Press to execute/edit", false);
    }
}

/**
 * @brief Render STELLARIA Auto Mode detail page (interactive)
 */
static void hmi_render_stellaria_auto_page(void)
{
    stellaria_snapshot_t data;
    telemetry_get_stellaria_data(&data);

    fb_draw_string(26, 2, "Auto Mode", false);
    fb_draw_hline(0, 10, 128);

    // Menu items with current state
    const char *items[2];
    char auto_text[32];

    items[0] = "< Back";

    // Toggle auto mode item (shows current state)
    if (data.auto_mode_active) {
        snprintf(auto_text, sizeof(auto_text), "Auto: ON");
    } else {
        snprintf(auto_text, sizeof(auto_text), "Auto: OFF");
    }
    items[1] = auto_text;

    // Render menu items
    int y = 14;
    for (int i = 0; i < 2; i++) {
        bool selected = (i == hmi_status.selected_item);

        // Selection indicator
        if (selected) {
            fb_draw_string(2, y, ">", false);
        }

        // Item text
        fb_draw_string(8, y, items[i], selected);

        y += 14;
    }

    // Status information (below menu)
    y += 4;  // Extra spacing
    char buf[32];

    snprintf(buf, sizeof(buf), "Light: %.2fV", data.last_light_reading);
    fb_draw_string(2, y, buf, false);

    fb_draw_string(2, 56, "Press to toggle", false);
}

// ########################## System Menu Functions ##########################

/**
 * @brief Render SYSTEM submenu (Info, Controls)
 */
static void hmi_render_system_menu(void)
{
    // Title
    fb_draw_string(30, 2, "SYSTEM", false);
    fb_draw_hline(0, 10, 128);

    // Menu items
    const char *items[] = {
        "< Back",
        "System Info",
        "Controls"
    };

    int y = 14;
    for (int i = 0; i < 3; i++) {
        bool selected = (i == hmi_status.selected_item);

        // Selection indicator
        if (selected) {
            fb_draw_string(2, y, ">", false);
        }

        // Item text
        fb_draw_string(8, y, items[i], selected);

        y += 12;
    }
}

/**
 * @brief Render System Info page (WiFi RSSI, uptime, version)
 */
static void hmi_render_system_info_page(void)
{
    // Get WiFi data from TELEMETRY cache
    wifi_snapshot_t wifi_data;
    telemetry_get_wifi_data(&wifi_data);

    // Title
    fb_draw_string(24, 2, "System Info", false);
    fb_draw_hline(0, 10, 128);

    char buf[32];
    int y = 14;

    // WiFi state
    const char *wifi_state_str[] = {
        "DISABLED", "INIT", "CONNECTING",
        "CONNECTED", "RECONNECT", "FAILED"
    };
    snprintf(buf, sizeof(buf), "WiFi: %s",
             wifi_state_str[wifi_data.state < 6 ? wifi_data.state : 0]);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // RSSI (only if connected)
    if (wifi_data.state == WIFI_STATE_CONNECTED && wifi_data.has_ip) {
        snprintf(buf, sizeof(buf), "RSSI: %d dBm", wifi_data.rssi);
        fb_draw_string(2, y, buf, false);
    } else {
        fb_draw_string(2, y, "RSSI: N/A", false);
    }
    y += 10;

    // Reconnection count
    snprintf(buf, sizeof(buf), "Reconnects: %u", wifi_data.reconnect_count);
    fb_draw_string(2, y, buf, false);
    y += 10;

    // Power save mode
    snprintf(buf, sizeof(buf), "Power Save: %s",
             wifi_data.power_save_mode ? "ON" : "OFF");
    fb_draw_string(2, y, buf, false);
    y += 14;

    // Uptime
    uint32_t uptime_sec = (uint32_t)time(NULL);
    uint32_t hours = uptime_sec / 3600;
    uint32_t minutes = (uptime_sec % 3600) / 60;
    snprintf(buf, sizeof(buf), "Uptime: %luh %lum", hours, minutes);
    fb_draw_string(2, y, buf, false);

    // Navigation hint
    fb_draw_string(2, 56, "Press to go back", false);
}

/**
 * @brief Render System Controls page (Flush & Reset, WiFi Reconnect)
 */
static void hmi_render_system_controls_page(void)
{
    // Title
    fb_draw_string(26, 2, "Controls", false);
    fb_draw_hline(0, 10, 128);

    // Menu items
    const char *items[] = {
        "< Back",
        "Flush & Reset",
        "WiFi Reconnect"
    };

    int y = 14;
    for (int i = 0; i < 3; i++) {
        bool selected = (i == hmi_status.selected_item);

        // Selection indicator
        if (selected) {
            fb_draw_string(2, y, ">", false);
        }

        // Item text
        fb_draw_string(8, y, items[i], selected);

        y += 12;
    }

    // Help text at bottom
    fb_draw_string(2, 56, "Press to select", false);
}

// ########################## Activity Tracking ##########################

/**
 * @brief Update last activity timestamp
 */
static void hmi_update_activity(void)
{
    hmi_status.last_activity_time = time(NULL);
}

/**
 * @brief Check if display timeout has been reached
 * Uses different timeouts for realtime pages (60s) vs normal pages (30s)
 */
static bool hmi_check_timeout(void)
{
    if (!hmi_status.display_active) {
        return false;  // Already off
    }

    time_t now = time(NULL);
    int64_t elapsed_ms = (now - hmi_status.last_activity_time) * 1000;

    // Use longer timeout for realtime pages
    int64_t timeout_ms = hmi_is_realtime_page() ?
                         HMI_DISPLAY_TIMEOUT_REALTIME_MS :
                         HMI_DISPLAY_TIMEOUT_NORMAL_MS;

    return (elapsed_ms >= timeout_ms);
}

// ########################## Task Functions ##########################

/**
 * @brief HMI display task - handles display refresh and timeout
 * Implements variable refresh rate: 250ms for realtime pages, 1000ms for normal pages
 */
static void hmi_display_task(void *pvParameters)
{
    ESP_LOGI(TAG, "HMI display task started");

    TickType_t last_wake_time = xTaskGetTickCount();
    int last_encoder_count = 0;
    uint32_t notification_value = 0;

    while (1) {
        // Check for button press notification (from ISR)
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &notification_value, 0) == pdTRUE) {
            ESP_LOGI(TAG, "Button press detected");

            // Wake display if off
            if (!hmi_status.display_active) {
                hmi_display_power_on();
            } else {
                // Handle button press (menu navigation)
                if (xSemaphoreTake(xHmiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    hmi_handle_button_press();
                    xSemaphoreGive(xHmiMutex);
                }
            }
        }

        // Read encoder count and check for changes
        if (hmi_status.display_active) {
            int current_count = 0;
            pcnt_unit_get_count(encoder_pcnt_unit, &current_count);

            int delta = current_count - last_encoder_count;

            // Only process significant changes (debounce noise)
            if (delta >= 4 || delta <= -4) {
                if (xSemaphoreTake(xHmiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    hmi_handle_encoder_change(delta / 4);  // Normalize to +/-1 per detent
                    xSemaphoreGive(xHmiMutex);
                }
                last_encoder_count = current_count;
                ESP_LOGD(TAG, "Encoder delta: %d (count: %d)", delta / 4, current_count);
            }
        }

        // Check for display timeout
        if (hmi_check_timeout()) {
            ESP_LOGI(TAG, "Display timeout - powering off");
            hmi_display_power_off();
        }

        // Update blink indicator and render menu if display is active
        if (hmi_status.display_active) {
            if (xSemaphoreTake(xHmiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                // Update blink state (toggle every 2 cycles = 500ms with 250ms refresh)
                if (hmi_is_realtime_page()) {
                    hmi_status.blink_counter++;
                    if (hmi_status.blink_counter >= 2) {
                        hmi_status.blink_state = !hmi_status.blink_state;
                        hmi_status.blink_counter = 0;
                    }
                }

                hmi_render_menu();
                xSemaphoreGive(xHmiMutex);
            }
        }

        // Variable refresh rate: fast for realtime pages, slow for normal pages
        TickType_t delay_ms = hmi_is_realtime_page() ?
                             HMI_REFRESH_RATE_REALTIME_MS :
                             HMI_REFRESH_RATE_NORMAL_MS;
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(delay_ms));
    }
}

// ########################## Public API Functions ##########################

/**
 * @brief Initialize HMI system
 */
esp_err_t hmi_init(void)
{
    ESP_LOGI(TAG, "Initializing HMI system...");

    // Create mutex
    xHmiMutex = xSemaphoreCreateMutex();
    if (xHmiMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create HMI mutex");
        return ESP_FAIL;
    }

    // Initialize encoder
    esp_err_t ret = hmi_encoder_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize encoder: %s", esp_err_to_name(ret));
        vSemaphoreDelete(xHmiMutex);
        return ret;
    }

    // Initialize display
    ret = hmi_display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize display: %s", esp_err_to_name(ret));
        vSemaphoreDelete(xHmiMutex);
        return ret;
    }

    // Create display task
    BaseType_t task_ret = xTaskCreate(
        hmi_display_task,
        "hmi_display",
        4096,  // Stack size
        NULL,
        5,     // Priority
        &xHmiDisplayTask
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display task");
        vSemaphoreDelete(xHmiMutex);
        return ESP_FAIL;
    }

    // Mark as initialized
    hmi_status.initialized = true;

    ESP_LOGI(TAG, "HMI system initialized successfully");
    return ESP_OK;
}

/**
 * @brief Get current HMI status
 */
esp_err_t hmi_get_status(hmi_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!hmi_status.initialized) {
        memset(status, 0, sizeof(hmi_status_t));
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xHmiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(status, &hmi_status, sizeof(hmi_status_t));
        xSemaphoreGive(xHmiMutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief Manually wake display
 */
esp_err_t hmi_wake_display(void)
{
    if (!hmi_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xHmiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        esp_err_t ret = hmi_display_power_on();
        xSemaphoreGive(xHmiMutex);
        return ret;
    }

    return ESP_FAIL;
}
