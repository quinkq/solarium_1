#include "hmi.h"
#include "fluctus.h"
#include "irrigation.h"
#include "weather_station.h"
#include "stellaria.h"
#include "telemetry.h"

#include <string.h>
#include <stdio.h>
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
    .encoder_count = 0
};

// Mutex for thread-safe operations
static SemaphoreHandle_t xHmiMutex = NULL;

// Task handles
static TaskHandle_t xHmiDisplayTask = NULL;

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
static void hmi_render_tempesta_menu(void);
static void hmi_render_impluvium_menu(void);
static void hmi_render_stellaria_menu(void);

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

    // Configure SPI panel IO (shares SPI2_HOST with ABP sensor - bus already initialized in main.c)
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

    hmi_status.display_active = true;
    hmi_update_activity();

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
            return 4;  // FLUCTUS, TEMPESTA, IMPLUVIUM, STELLARIA
        case HMI_MENU_FLUCTUS:
        case HMI_MENU_TEMPESTA:
        case HMI_MENU_IMPLUVIUM:
        case HMI_MENU_STELLARIA:
            return 4;  // Back + 3 submenu items (simplified for prototype)
        default:
            return 1;  // Just "Back" for detail pages
    }
}

/**
 * @brief Handle encoder rotation (menu navigation)
 * @param delta Encoder delta (+/- from PCNT)
 */
static void hmi_handle_encoder_change(int delta)
{
    if (delta == 0) return;

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
            }
            hmi_status.selected_item = 0;  // Reset selection
            break;

        case HMI_MENU_FLUCTUS:
        case HMI_MENU_TEMPESTA:
        case HMI_MENU_IMPLUVIUM:
        case HMI_MENU_STELLARIA:
            if (hmi_status.selected_item == 0) {
                // "Back" selected - return to main menu
                hmi_status.current_menu = HMI_MENU_MAIN;
                hmi_status.selected_item = 0;
            } else {
                // TODO: Enter detail pages (not implemented in prototype)
            }
            break;

        default:
            // For detail pages, go back to parent menu
            hmi_status.current_menu = HMI_MENU_MAIN;
            hmi_status.selected_item = 0;
            break;
    }

    hmi_update_activity();
}

// ########################## Menu Rendering Functions ##########################

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

        case HMI_MENU_TEMPESTA:
            hmi_render_tempesta_menu();
            break;

        case HMI_MENU_IMPLUVIUM:
            hmi_render_impluvium_menu();
            break;

        case HMI_MENU_STELLARIA:
            hmi_render_stellaria_menu();
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
        "  STELLARIA"
    };

    int y = 14;
    for (int i = 0; i < 4; i++) {
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
    telemetry_fluctus_t data;
    telemetry_get_fluctus_data(&data);

    // Title
    fb_draw_string(30, 2, "FLUCTUS", false);
    fb_draw_hline(0, 10, 128);

    // Menu items
    const char *items[] = {
        "< Back",
        "Power Buses",
        "Battery",
        "Solar Tracking"
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
    snprintf(buf, sizeof(buf), "Bat:%.2fV SOC:%f%%",
             data.battery_voltage, data.battery_soc_percent);
    fb_draw_string(2, 56, buf, false);
}

/**
 * @brief Render TEMPESTA submenu
 */
static void hmi_render_tempesta_menu(void)
{
    // Get data from TELEMETRY cache
    telemetry_tempesta_t data;
    telemetry_get_tempesta_data(&data);

    // Title
    fb_draw_string(28, 2, "TEMPESTA", false);
    fb_draw_hline(0, 10, 128);

    // Menu items
    const char *items[] = {
        "< Back",
        "Environment",
        "Wind & Rain",
        "Air Quality"
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
    snprintf(buf, sizeof(buf), "%.1fC %.0f%%RH",
             data.temperature, data.humidity);
    fb_draw_string(2, 56, buf, false);
}

/**
 * @brief Render IMPLUVIUM submenu
 */
static void hmi_render_impluvium_menu(void)
{
    // Get data from TELEMETRY cache
    telemetry_impluvium_t data;
    telemetry_get_impluvium_data(&data);

    // Title
    fb_draw_string(24, 2, "IMPLUVIUM", false);
    fb_draw_hline(0, 10, 128);

    // Menu items
    const char *items[] = {
        "< Back",
        "Zone Status",
        "Learning Data",
        "System Status"
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
    const char *state_str = "IDLE";
    switch (data.state) {
        case IRRIGATION_MEASURING: state_str = "MEAS"; break;
        case IRRIGATION_WATERING: state_str = "WATER"; break;
        case IRRIGATION_STOPPING: state_str = "STOP"; break;
        default: break;
    }
    snprintf(buf, sizeof(buf), "State:%s Lvl:%f%%",
             state_str, data.water_level_percent);
    fb_draw_string(2, 56, buf, false);
}

/**
 * @brief Render STELLARIA submenu
 */
static void hmi_render_stellaria_menu(void)
{
    // Get data from TELEMETRY cache
    telemetry_stellaria_t data;
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
 */
static bool hmi_check_timeout(void)
{
    if (!hmi_status.display_active) {
        return false;  // Already off
    }

    time_t now = time(NULL);
    int64_t elapsed_ms = (now - hmi_status.last_activity_time) * 1000;

    return (elapsed_ms >= HMI_DISPLAY_TIMEOUT_MS);
}

// ########################## Task Functions ##########################

/**
 * @brief HMI display task - handles display refresh and timeout
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

        // Render menu if display is active
        if (hmi_status.display_active) {
            if (xSemaphoreTake(xHmiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                hmi_render_menu();
                xSemaphoreGive(xHmiMutex);
            }
        }

        // Sleep until next refresh (10Hz)
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(HMI_REFRESH_RATE_MS));
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
