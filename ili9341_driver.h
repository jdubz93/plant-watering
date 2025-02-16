#ifndef _ILI9341_DRIVER_H_
#define _ILI9341_DRIVER_H_

#include "fonts.h"

void delay_ms(uint32_t ms);

void ili9341_select(void);
void ili9341_deselect(void);

void ili9341_dc_command(void);
void ili9341_dc_data(void);

void ili9341_write_command(uint8_t cmd);
void ili9341_write_data(uint8_t data);

void ili9341_reset(void);
void ili9341_init(void);

void ili9341_set_pixel(uint16_t x, uint16_t y, uint16_t color);

void ili9341_fill_screen(uint16_t color);
void ili9341_test_fill(void);
void ili9341_draw_corners(void);
void ili9341_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

void ili9341_draw_char(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg_color);
void ili9341_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg_color);
void ili9341_draw_char_scaled(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg_color, uint8_t scale);
void ili9341_draw_string_scaled(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg_color, uint8_t scale);

#endif