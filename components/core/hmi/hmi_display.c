/**
 * @file hmi_display.c
 * @brief HMI Component - Display Hardware Abstraction Layer
 *
 * Handles all hardware-specific display operations:
 * - SH1106 OLED display initialization and power management
 * - Framebuffer management (1024 bytes, monochrome 128x64)
 * - Custom font data (5x7 bitmap font)
 * - Drawing primitives (pixels, characters, strings, lines)
 */

#include "hmi_private.h"
#include "driver/spi_master.h"
#include <string.h>
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"



static const char *TAG = "HMI_DISPLAY";

// ########################## Display Constants ##########################

#define FB_WIDTH 128
#define FB_HEIGHT 64
#define FB_SIZE ((FB_WIDTH * FB_HEIGHT) / 8)  // 1024 bytes

// ########################## Font Data ##########################

/**
 * Simple 5x7 bitmap font (ASCII 32-126)
 * Each character is 5 bytes (5 columns × 7 rows packed)
 * Total: 95 characters × 5 bytes = 475 bytes
 */
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

// ########################## Framebuffer ##########################

/**
 * Framebuffer for 128x64 monochrome display
 * Layout: 8 pages (rows of 8 pixels), 128 columns
 * Total: 1024 bytes (128 × 8)
 * Each byte represents 8 vertical pixels (LSB = top)
 */
static uint8_t framebuffer[FB_SIZE];

// ########################## Display Hardware Handles ##########################

static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;

// ########################## Display Initialization ##########################

/**
 * @brief Initialize SH1106 OLED display via SPI + esp_lcd
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t hmi_display_init(void)
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
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,  // No hardware reset pin
        .color_space = ESP_LCD_COLOR_SPACE_MONOCHROME,
        .bits_per_pixel = 1,
    };

    // Try SSD1306 driver (SH1106 is compatible with minor differences)
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
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t hmi_display_power_on(void)
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

    if (xSemaphoreTake(xHmiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        hmi_status.display_active = true;
        // Note: hmi_update_activity() would be called here but it's in hmi.c
        // For now, just update the display_active flag
        xSemaphoreGive(xHmiMutex);
    }

    ESP_LOGI(TAG, "Display powered ON (3.3V bus requested)");
    return ESP_OK;
}

/**
 * @brief Power off display (release 3.3V bus)
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t hmi_display_power_off(void)
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
 * @brief Clear entire framebuffer (all pixels off)
 */
void hmi_fb_clear(void)
{
    memset(framebuffer, 0x00, FB_SIZE);
}

/**
 * @brief Set a single pixel in the framebuffer
 * @param x X coordinate (0-127)
 * @param y Y coordinate (0-63)
 * @param on Pixel state (true = on, false = off)
 */
void hmi_fb_set_pixel(int x, int y, bool on)
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
 * @brief Draw a single character using custom 5x7 font
 * @param x X coordinate (0-127)
 * @param y Y coordinate (0-63)
 * @param c ASCII character to draw (32-126)
 * @param inverted Invert colors (true = black text on white background)
 */
void hmi_fb_draw_char(int x, int y, char c, bool inverted)
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
            hmi_fb_set_pixel(x + col, y + row, pixel_on);
        }
    }

    // Add 1 pixel spacing after character
    for (int row = 0; row < 7; row++) {
        hmi_fb_set_pixel(x + 5, y + row, inverted);
    }
}

/**
 * @brief Draw a text string using custom font
 * @param x X coordinate (0-127)
 * @param y Y coordinate (0-63)
 * @param str Null-terminated string to draw
 * @param inverted Invert colors (true = black text on white background)
 */
void hmi_fb_draw_string(int x, int y, const char *str, bool inverted)
{
    int cur_x = x;
    while (*str) {
        hmi_fb_draw_char(cur_x, y, *str, inverted);
        cur_x += 6;  // 5 pixels + 1 spacing
        str++;
    }
}

/**
 * @brief Draw a horizontal line
 * @param x X start coordinate (0-127)
 * @param y Y coordinate (0-63)
 * @param width Line width in pixels
 */
void hmi_fb_draw_hline(int x, int y, int width)
{
    for (int i = 0; i < width; i++) {
        hmi_fb_set_pixel(x + i, y, true);
    }
}

/**
 * @brief Flush framebuffer to display via SPI
 * Sends the entire 1024-byte framebuffer to the display
 */
void hmi_fb_flush(void)
{
    // Send framebuffer to display using esp_lcd
    // For SSD1306/SH1106, draw the entire framebuffer (128x64 = 1024 bytes)
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, FB_WIDTH, FB_HEIGHT, framebuffer);
}
