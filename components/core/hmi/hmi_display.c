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
#include "mcp23008_helper.h"



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
    // Perform hardware reset via MCP23008 GP6
    ESP_LOGI(TAG, "Performing hardware reset via MCP23008 GP6...");
    esp_err_t ret = mcp23008_helper_oled_reset_pulse();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset display: %s", esp_err_to_name(ret));        
        return ret;
    }

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

    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)HMI_DISPLAY_SPI_HOST,
                                              &io_config,
                                              &io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create panel IO: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure SH1106 panel
    // Note: SH1106 is similar to SSD1306 but uses different column addressing
    // Hardware reset performed via MCP23008 GP6 above
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,  // Reset handled by MCP23008 GP6
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

    // Configure display settings
    // Note: Rotation handled in hmi_fb_flush() for SH1106 compatibility
    // esp_lcd orientation functions don't work with custom page-by-page writes
    esp_lcd_panel_invert_color(panel_handle, false);

    // Verify rotation setting at compile time
    #if HMI_DISPLAY_ROTATION != 0 && HMI_DISPLAY_ROTATION != 90 && \
        HMI_DISPLAY_ROTATION != 180 && HMI_DISPLAY_ROTATION != 270
        #error "HMI_DISPLAY_ROTATION must be 0, 90, 180, or 270"
    #endif

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

    // Drive RES high BEFORE powering on bus (ensures RES is valid when display gets power)
    esp_err_t ret = mcp23008_helper_set_oled_reset(true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to drive RES high: %s", esp_err_to_name(ret));
        return ret;
    }

    // Request 3.3V bus from FLUCTUS (toggled for display)
    // 50ms bus delay ensures RES is stable before display controller starts
    FLUCTUS_REQUEST_BUS_OR_FAIL(POWER_BUS_3V3, "HMI", return ESP_FAIL);

    // Turn on display
    ret = esp_lcd_panel_disp_on_off(panel_handle, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to turn on display: %s", esp_err_to_name(ret));
        fluctus_release_bus_power(POWER_BUS_3V3, "HMI");
        return ret;
    }

    if (xSemaphoreTake(xHmiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        hmi_status.display_active = true;
        hmi_update_activity();  // Reset timeout timer
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

    // Release GP6 to high-Z (power saving)
    mcp23008_helper_oled_reset_release();

    // Release 3.3V bus
    fluctus_release_bus_power(POWER_BUS_3V3, "HMI");

    hmi_status.display_active = false;

    ESP_LOGI(TAG, "Display powered OFF (3.3V bus released, GP6 high-Z)");
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
#if HMI_DISPLAY_ROTATION != 0
/**
 * @brief Bit reversal lookup table for vertical flip
 * Used for 180° and 270° rotations to flip bits within each byte
 */
static const uint8_t bit_reverse_table[256] = {
    0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
    0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
    0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
    0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
    0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
    0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
    0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
    0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
    0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
    0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
    0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
    0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
    0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
    0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
    0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
    0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};
#endif
/**
 * @brief Flush framebuffer to display via SPI
 * Sends the entire 1024-byte framebuffer to the display page-by-page
 *
 * SH1106 requires page-by-page writes with column offset commands.
 * Cannot use esp_lcd_panel_draw_bitmap() as it doesn't support SH1106's
 * 132-column addressing (displays middle 128 columns with +2 offset).
 *
 * Handles rotation at flush time since esp_lcd orientation functions
 * don't work with custom page-by-page writes.
 */
void hmi_fb_flush(void)
{
    static uint8_t line_buffer[FB_WIDTH];  // Temporary buffer for rotation

#if HMI_DISPLAY_ROTATION == 0
    // 0° - Normal orientation, direct write
    for (uint8_t page = 0; page < 8; page++) {
        esp_lcd_panel_io_tx_param(io_handle, 0xB0 + page, NULL, 0);
        esp_lcd_panel_io_tx_param(io_handle, 0x02, NULL, 0);
        esp_lcd_panel_io_tx_param(io_handle, 0x10, NULL, 0);
        esp_lcd_panel_io_tx_color(io_handle, -1, &framebuffer[page * FB_WIDTH], FB_WIDTH);
    }

#elif HMI_DISPLAY_ROTATION == 180
    // 180° - Upside down: reverse pages, reverse columns, flip bits
    for (int8_t page = 7; page >= 0; page--) {
        esp_lcd_panel_io_tx_param(io_handle, 0xB0 + (7 - page), NULL, 0);
        esp_lcd_panel_io_tx_param(io_handle, 0x02, NULL, 0);
        esp_lcd_panel_io_tx_param(io_handle, 0x10, NULL, 0);

        // Reverse columns and flip bits
        for (int16_t x = 0; x < FB_WIDTH; x++) {
            line_buffer[x] = bit_reverse_table[framebuffer[page * FB_WIDTH + (FB_WIDTH - 1 - x)]];
        }
        esp_lcd_panel_io_tx_color(io_handle, -1, line_buffer, FB_WIDTH);
    }

#elif HMI_DISPLAY_ROTATION == 90 || HMI_DISPLAY_ROTATION == 270
    #error "90° and 270° rotations not yet implemented for SH1106"

#endif
}

/**
 * @brief Draw scroll indicators (▲/▼) for menus with >7 items
 *
 * Shows visual feedback when there are hidden menu items:
 * - Up arrow (^) at top-right if scrolled down (offset > 0)
 * - Down arrow (v) at bottom-right if more items below
 *
 * @param total_items Total number of items in current menu
 */
void hmi_draw_scroll_indicators(uint8_t total_items)
{
    // No indicators needed if all items fit on screen
    if (total_items <= HMI_MAX_VISIBLE_ITEMS) {
        return;
    }

    // Draw up arrow if scrolled down (items above current view)
    if (hmi_status.scroll_offset > 0) {
        hmi_fb_draw_char(120, 2, '^', false);  // ASCII 94 (^) as up arrow
    }

    // Draw down arrow if more items below current view
    uint8_t last_visible = hmi_status.scroll_offset + HMI_MAX_VISIBLE_ITEMS - 1;
    if (last_visible < total_items - 1) {
        hmi_fb_draw_char(120, 56, 'v', false);  // ASCII 118 (v) as down arrow
    }
}

/**
 * @brief Draw navigation symbol in top-left corner (← for back)
 * Indicates "press button to go back"
 */
void hmi_draw_back_symbol(void)
{
    hmi_fb_draw_char(2, 2, '<', false);  // '<' represents back/left arrow
}

/**
 * @brief Draw navigation symbol in top-right corner (↔ for pagination)
 * Indicates "rotate to change pages"
 */
void hmi_draw_pagination_symbol(void)
{
    hmi_fb_draw_string(114, 2, "<>", false);  // '<>' represents bidirectional arrows
}
