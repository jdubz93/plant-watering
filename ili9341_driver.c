#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/systick.h>
#include "ili9341_driver.h"
#include <stdint.h>
#include <string.h>

volatile uint16_t LCD_HEIGHT = 320;
volatile uint16_t LCD_WIDTH = 240;

#define SWRESET (0x01)

#define DISPLAYOFF (0b00101000)

#define PIXELFORMAT_COMMAND (0b00111010)
#define PIXELFORMAT_16BIT_RGB565 (0b01010101)

#define FRAMERATE_COMMAND (0b10110001)
#define FRAMERATE_70HZ_DEFAULT (0b11000)
#define FRAMERATE_61HZ (0b11111)
#define FRAMERATE_119HZ (0b10000)

void 
delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < (ms * 8000); i++)
        __asm__("nop");
}

// must select/deselect before sending data
void 
ili9341_select(void) {
    gpio_clear(GPIOB, GPIO7);
}

void 
ili9341_deselect(void) {
    gpio_set(GPIOB, GPIO7);
}

 // DC low = command mode
void 
ili9341_dc_command(void) {
    gpio_clear(GPIOB, GPIO10);
}

// DC high = data mode
void 
ili9341_dc_data(void) {
    gpio_set(GPIOB, GPIO10);
}

// waits for spi TX to finish
void 
ili9341_tx_finished(void) {
    while (!(SPI_SR(SPI1) & SPI_SR_TXE));
}

// hardware reset display
void 
ili9341_reset(void) {
    gpio_clear(GPIOB, GPIO11);
    delay_ms(10);
    gpio_set(GPIOB, GPIO11);
    delay_ms(120);
}

void 
ili9341_write_command(uint8_t cmd) {
    ili9341_dc_command();
    spi_send(SPI1, cmd);
    ili9341_tx_finished();
}

void 
ili9341_write_data(uint8_t data) {
    ili9341_dc_data();
    spi_send(SPI1, data);
    ili9341_tx_finished();
}

void ili9341_init(void) {
    ili9341_reset();
    ili9341_select();

    ili9341_write_command(SWRESET); // sw reset
    delay_ms(100);

    ili9341_write_command(DISPLAYOFF); // display off

    ili9341_write_command(0xCF);
    ili9341_write_data(0x00);
    ili9341_write_data(0xC1);
    ili9341_write_data(0x30);

    ili9341_write_command(0xED);
    ili9341_write_data(0x64);
    ili9341_write_data(0x03);
    ili9341_write_data(0x12);
    ili9341_write_data(0x81);

    ili9341_write_command(0xE8);
    ili9341_write_data(0x85);
    ili9341_write_data(0x00);
    ili9341_write_data(0x78);

    // ili9341_write_command(0x36);
    // ili9341_write_data(0x88); // rotation
/*
rotations
0x48 = 0
0x28 = 90
0x88 = 180
0xE8 = 270
*/
    ili9341_write_command(PIXELFORMAT_COMMAND); // pixel format
    ili9341_write_data(PIXELFORMAT_16BIT_RGB565);   // 16 bit rgb565

    // ili9341_write_command(FRAMERATE_COMMAND);
    // ili9341_write_data(0x00);
    // ili9341_write_data(FRAMERATE_70HZ_DEFAULT);

    ili9341_write_command(0x26); // gamma
    ili9341_write_data(0x01);

    ili9341_write_command(0x11); // sleep OUT
    delay_ms(120);

    ili9341_write_command(0x29); // display ON
    ili9341_deselect();
}

/***********************
 *      DRAWING FUNCTIONS
 ***********************/
void 
ili9341_fill_screen(uint16_t color) {
    uint32_t total_pixels = LCD_WIDTH * LCD_HEIGHT;

    ili9341_write_command(0x2A); // sets column address
    ili9341_write_data(0x00);
    ili9341_write_data(0x00);
    ili9341_write_data((LCD_WIDTH - 1) >> 8); // end high byte
    ili9341_write_data((LCD_WIDTH - 1) & 0xFF); // end low byte

    ili9341_write_command(0x2B); // sets page address
    ili9341_write_data(0x00); // start high byte
    ili9341_write_data(0x00); // start low byte
    ili9341_write_data((LCD_HEIGHT - 1) >> 8); // end high byte
    ili9341_write_data((LCD_HEIGHT - 1) & 0xFF); // end low byte

    ili9341_write_command(0x2C);

    for (uint32_t i = 0; i < total_pixels; i++) {
        ili9341_write_data(color >> 8);
        ili9341_write_data(color & 0xFF);
    }
}

void ili9341_test_fill(void) {
    ili9341_write_command(0x2A);
    ili9341_write_data(0x00);
    ili9341_write_data(0x00);
    ili9341_write_data(0x00);
    ili9341_write_data(0x09);

    ili9341_write_command(0x2B);
    ili9341_write_data(0x00);
    ili9341_write_data(0x00);
    ili9341_write_data(0x00);
    ili9341_write_data(0x09);

    ili9341_write_command(0x2C);
    for (int i = 0; i < 100; i++) { // 10x10 = 100 pixels
        ili9341_write_data(0xF8);
        ili9341_write_data(0x00);
    }
}

void 
ili9341_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    // Set the drawing area
    ili9341_write_command(0x2A); // Column Address Set
    ili9341_write_data(x >> 8);
    ili9341_write_data(x & 0xFF);
    ili9341_write_data((x + w - 1) >> 8);
    ili9341_write_data((x + w - 1) & 0xFF);

    ili9341_write_command(0x2B); // Page Address Set
    ili9341_write_data(y >> 8);
    ili9341_write_data(y & 0xFF);
    ili9341_write_data((y + h - 1) >> 8);
    ili9341_write_data((y + h - 1) & 0xFF);

    // Fill the area
    ili9341_write_command(0x2C); // Memory Write
    for (uint32_t i = 0; i < w * h; i++) {
        ili9341_write_data(color >> 8);
        ili9341_write_data(color & 0xFF);
    }
}

void 
ili9341_set_pixel(uint16_t x, uint16_t y, uint16_t color) {
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) {
        return; // Ignore out-of-bound coordinates
    }

    // Set column address
    ili9341_write_command(0x2A);

    // start
    ili9341_write_data(x >> 8);
    ili9341_write_data(x & 0xFF);
    // end
    ili9341_write_data(x >> 8); // high byte
    ili9341_write_data(x & 0xFF); // low byte

    // Set row address
    ili9341_write_command(0x2B);

    // start
    ili9341_write_data(y >> 8);
    ili9341_write_data(y & 0xFF);
    // end
    ili9341_write_data(y >> 8);        // High byte (end row same as start)
    ili9341_write_data(y & 0xFF);      // Low byte

    // Write pixel color
    ili9341_write_command(0x2C);
    ili9341_write_data(color >> 8);
    ili9341_write_data(color & 0xFF);
}

void 
ili9341_draw_char(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg_color) {
    if (c < ' ' || c > '~') {
        return; // ignores characters not in the ascii range
    }

    uint16_t index = (c - ' ') * FONT1_WIDTH;

    for (int col = 0; col < FONT1_WIDTH; col++) {
        for (int row = 0; row < FONT1_HEIGHT; row++) {
            if (FONT1[index + col] & (1 << row)) {
                ili9341_set_pixel(x + col, y + row, color); // foreground color
            }
            else
            {
                ili9341_set_pixel(x + col, y + row, bg_color); // background color
            }
        }
    }

    for (uint8_t row = 0; row < FONT1_HEIGHT; row++) {
        ili9341_set_pixel(x + FONT1_WIDTH, y + row, bg_color);
    }
}

void 
ili9341_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg_color) {
    uint16_t len = strlen(str);
    for (uint16_t i = len; i > 0; i--) {
        ili9341_draw_char(x, y, str[i - 1], color, bg_color);
        x += 6; // move to the next character position (5x7 font + 1 pixel spacing)
    }
}

void 
ili9341_draw_char_scaled(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg_color, uint8_t scale) {
    uint16_t index = (c - ' ') * FONT1_WIDTH;

    for (int col = 0; col < FONT1_WIDTH; col++) {
        uint8_t column_data = FONT1[index + col];
        for (int row = 0; row < FONT1_HEIGHT; row++) {
            if (column_data & (1 << row)) {
                for (uint8_t sx = 0; sx < scale; sx++) {
                    for (uint8_t sy = 0; sy < scale; sy++) {
                        ili9341_set_pixel(x + col * scale + sx, y + row * scale + sy, color); // foreground
                    }
                }
            } else {
                for (uint8_t sx = 0; sx < scale; sx++) {
                    for (uint8_t sy = 0; sy < scale; sy++) {
                        ili9341_set_pixel(x + col * scale + sx, y + row * scale + sy, bg_color); // background
                    }
                }
            }
        }
    }

    // Add scaled spacing column for better readability
    for (uint8_t row = 0; row < FONT1_HEIGHT * scale; row++) {
        for (uint8_t sx = 0; sx < scale; sx++) {
            ili9341_set_pixel(x + FONT1_WIDTH * scale + sx, y + row, bg_color);
        }
    }
}

void 
ili9341_draw_string_scaled(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg_color, uint8_t scale) {
    uint16_t len = strlen(str);
    for (uint16_t i = len; i > 0; i--) {
        ili9341_draw_char_scaled(x, y, str[i - 1], color, bg_color, scale);
        x += (6 * scale) + scale; 
    }
}
