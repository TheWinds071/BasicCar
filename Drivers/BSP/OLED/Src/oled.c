/**
 * @file oled.c
 * @brief BSP-style API implementation for SSD1306 OLED display
 */

#include "oled.h"
#include "u8g2.h"
#include <string.h>

/* u8g2 callback prototypes */
extern uint8_t u8g2_hal_i2c_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
extern uint8_t u8g2_hal_gpio_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

/* u8g2 instance */
static u8g2_t u8g2;
static bool initialized = false;

/**
 * @brief Initialize the OLED display
 */
bool OLED_Init(void)
{
    /* Initialize u8g2 for SSD1306 128x64 with full buffer mode (_f) over I2C */
    /* Using NOROT (no rotation) variant */
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
        &u8g2,
        U8G2_R0,                    /* No rotation */
        u8g2_hal_i2c_stm32,         /* I2C byte communication callback */
        u8g2_hal_gpio_delay         /* GPIO and delay callback */
    );

    /* Set I2C address (7-bit address 0x3C, shifted left becomes 0x78) */
    u8g2_SetI2CAddress(&u8g2, 0x3C << 1);

    /* Initialize display */
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);  /* Wake up display */

    /* Clear buffer and display */
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);

    /* Set default font */
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);

    initialized = true;
    return true;
}

/**
 * @brief Periodic update task
 */
void OLED_Task(void)
{
    if (!initialized)
        return;

    /* Send buffer to display */
    u8g2_SendBuffer(&u8g2);
}

/**
 * @brief Clear the display buffer
 */
void OLED_Clear(void)
{
    if (!initialized)
        return;

    u8g2_ClearBuffer(&u8g2);
}

/**
 * @brief Draw text on the display
 */
void OLED_DrawText(uint8_t x, uint8_t y, const char *text)
{
    if (!initialized || text == NULL)
        return;

    u8g2_DrawStr(&u8g2, x, y, text);
}

/**
 * @brief Draw a pixel
 */
void OLED_DrawPixel(uint8_t x, uint8_t y)
{
    if (!initialized)
        return;

    u8g2_DrawPixel(&u8g2, x, y);
}

/**
 * @brief Draw a line
 */
void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    if (!initialized)
        return;

    u8g2_DrawLine(&u8g2, x0, y0, x1, y1);
}

/**
 * @brief Draw a box
 */
void OLED_DrawBox(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    if (!initialized)
        return;

    u8g2_DrawBox(&u8g2, x, y, w, h);
}
