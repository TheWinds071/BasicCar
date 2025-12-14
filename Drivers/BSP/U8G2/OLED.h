#ifndef __oled_H
#define __oled_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include "u8g2.h"

#define u8         unsigned char  // ?unsigned char ????
#define MAX_LEN    128  //
#define OLED_ADDRESS  0x78 // oled??????
#define OLED_CMD   0x00  // ???
#define OLED_DATA  0x40  // ???

uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

void u8g2Init(u8g2_t *u8g2);

// 基本整数显示函数
void drawInt(u8g2_t *u8g2, uint8_t x, uint8_t y, int32_t value);

// 带宽度和填充选项的整数显示函数
void drawIntWidth(u8g2_t *u8g2, uint8_t x, uint8_t y, int32_t value, uint8_t width, uint8_t zero_pad);

// 浮点数显示函数
void drawFloat(u8g2_t *u8g2, uint8_t x, uint8_t y, float value);
void drawFloatPrec(u8g2_t *u8g2, uint8_t x, uint8_t y, float value, uint8_t prec);

// 十六进制显示函数
void drawHex16(u8g2_t *u8g2, uint8_t x, uint8_t y, uint16_t value);
void drawHex32(u8g2_t *u8g2, uint8_t x, uint8_t y, uint32_t value);

#ifdef __cplusplus
 }
#endif
#endif /*__ i2c_H */

