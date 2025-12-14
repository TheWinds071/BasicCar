/**
 * @file u8g2_hal_i2c.c
 * @brief u8g2 HAL I2C communication callback for STM32H7
 * 
 * This file implements the low-level I2C communication functions
 * required by u8g2 library to interface with SSD1306 OLED display
 * over STM32 HAL I2C.
 */

#include "u8g2.h"
#include "i2c.h"

/**
 * @brief u8g2 HAL I2C byte send callback
 * @param u8x8 u8x8 structure pointer
 * @param msg Message type (start, byte, end)
 * @param arg_int Argument integer (data byte or address)
 * @param arg_ptr Argument pointer (data buffer)
 * @return Always returns 1 for success
 */
uint8_t u8g2_hal_i2c_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    static uint8_t buffer[256];
    static uint8_t buf_idx = 0;
    uint8_t *data;

    switch (msg)
    {
        case U8X8_MSG_BYTE_INIT:
            /* I2C initialization is done in main.c via MX_I2C4_Init() */
            break;

        case U8X8_MSG_BYTE_START_TRANSFER:
            /* Reset buffer index at the start of transfer */
            buf_idx = 0;
            /* Store I2C address (shifted left for HAL) */
            buffer[buf_idx++] = u8x8_GetI2CAddress(u8x8);
            break;

        case U8X8_MSG_BYTE_SEND:
            /* Add bytes to buffer */
            data = (uint8_t *)arg_ptr;
            while (arg_int > 0)
            {
                buffer[buf_idx++] = *data;
                data++;
                arg_int--;
            }
            break;

        case U8X8_MSG_BYTE_END_TRANSFER:
            /* Transmit the buffered data via I2C */
            if (buf_idx > 1)
            {
                /* buffer[0] is the address, rest is data */
                HAL_I2C_Master_Transmit(&hi2c4, buffer[0], &buffer[1], buf_idx - 1, HAL_MAX_DELAY);
            }
            break;

        case U8X8_MSG_BYTE_SET_DC:
            /* Not used in I2C mode */
            break;

        default:
            return 0;
    }

    return 1;
}

/**
 * @brief u8g2 GPIO and delay callback (minimal implementation)
 * @param u8x8 u8x8 structure pointer
 * @param msg Message type
 * @param arg_int Delay value in milliseconds
 * @param arg_ptr Unused
 * @return Always returns 1 for success
 */
uint8_t u8g2_hal_gpio_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg)
    {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            /* GPIO init not needed - no reset pin used */
            break;

        case U8X8_MSG_DELAY_MILLI:
            HAL_Delay(arg_int);
            break;

        case U8X8_MSG_DELAY_10MICRO:
            /* Approximate 10us delay */
            for (volatile int i = 0; i < arg_int * 100; i++)
                __NOP();
            break;

        case U8X8_MSG_DELAY_100NANO:
            /* Approximate 100ns delay */
            for (volatile int i = 0; i < arg_int; i++)
                __NOP();
            break;

        case U8X8_MSG_GPIO_RESET:
            /* No reset pin used */
            break;

        default:
            return 0;
    }

    return 1;
}
