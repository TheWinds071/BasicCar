#include <stdio.h>

#include "u8g2.h"
#include "OLED.h"

#include "i2c.h"
#include "main.h"



uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
    static uint8_t buffer[128];
    static uint8_t buf_idx;
    uint8_t *data;

    switch (msg)
    {
    case U8X8_MSG_BYTE_INIT:
    {
        /* add your custom code to init i2c subsystem */
        MX_I2C4_Init(); //I2C???
    }
    break;

    case U8X8_MSG_BYTE_START_TRANSFER:
    {
        buf_idx = 0;
    }
    break;

    case U8X8_MSG_BYTE_SEND:
    {
        data = (uint8_t *)arg_ptr;

        while (arg_int > 0)
        {
            buffer[buf_idx++] = *data;
            data++;
            arg_int--;
        }
    }
    break;

    case U8X8_MSG_BYTE_END_TRANSFER:
    {
        if (HAL_I2C_Master_Transmit(&hi2c4, (OLED_ADDRESS), buffer, buf_idx, 1000) != HAL_OK)
            return 0;
    }
    break;

    case U8X8_MSG_BYTE_SET_DC:
        break;

    default:
        return 0;
    }

    return 1;
}

void delay_us(uint32_t time)
{
    uint32_t i = 8 * time;
    while (i--)
        ;
}

uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
    switch(msg){

        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            break;

        case U8X8_MSG_DELAY_MILLI:
            HAL_Delay(arg_int);
            break;

        case U8X8_MSG_GPIO_I2C_CLOCK:
            break;

        case U8X8_MSG_GPIO_I2C_DATA:
            break;

        default:
            return 0;
    }
    return 1; // command processed successfully.
}

void u8g2Init(u8g2_t *u8g2)
{
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8g2_gpio_and_delay_stm32); // ??? u8g2 ???
	u8g2_InitDisplay(u8g2);                                                                       // ??????????????,??????,?????????
	u8g2_SetPowerSave(u8g2, 0);                                                                   // ?????
	u8g2_ClearBuffer(u8g2);
}


void drawInt(u8g2_t *u8g2, uint8_t x, uint8_t y, int32_t value) {
    char temp_buff[12]; // 足够容纳32位整数的字符串表示
    char *s = temp_buff;
    uint8_t is_negative = 0;

    // 处理0的特殊情况
    if (value == 0) {
        *s++ = '0';
        *s = '\0';
        u8g2_DrawStr(u8g2, x, y, temp_buff);
        return;
    }

    // 处理负数
    if (value < 0) {
        is_negative = 1;
        value = -value;
    }

    // 将数字转换为字符串（逆序）
    char temp_str[12];
    uint8_t len = 0;

    while (value > 0) {
        temp_str[len++] = (value % 10) + '0';
        value /= 10;
    }

    // 添加负号
    if (is_negative) {
        *s++ = '-';
    }

    // 反转数字字符串并复制到结果缓冲区
    for (int i = len - 1; i >= 0; i--) {
        *s++ = temp_str[i];
    }

    // 字符串结束
    *s = '\0';

    u8g2_DrawStr(u8g2, x, y, temp_buff);
}

// 带固定宽度和前导零填充的整数显示函数
void drawIntWidth(u8g2_t *u8g2, uint8_t x, uint8_t y, int32_t value, uint8_t width, uint8_t zero_pad) {
    char temp_buff[12];
    char *s = temp_buff;
    uint8_t is_negative = 0;
    uint8_t char_count = 0;

    // 处理0的特殊情况
    if (value == 0) {
        char_count = 1;
        if (zero_pad && width > 1) {
            // 填充前导零
            for (int i = 0; i < width - 1; i++) {
                *s++ = '0';
            }
        }
        *s++ = '0';
        *s = '\0';
        u8g2_DrawStr(u8g2, x, y, temp_buff);
        return;
    }

    // 处理负数
    if (value < 0) {
        is_negative = 1;
        value = -value;
        char_count++; // 计入负号
    }

    // 将数字转换为字符串（逆序）
    char temp_str[12];
    uint8_t len = 0;
    int32_t temp = value;

    while (temp > 0) {
        temp_str[len++] = (temp % 10) + '0';
        temp /= 10;
        char_count++;
    }

    // 添加负号
    if (is_negative) {
        *s++ = '-';
    }

    // 处理前导零填充
    if (zero_pad && width > char_count) {
        for (int i = 0; i < width - char_count; i++) {
            *s++ = '0';
        }
    }

    // 反转数字字符串并复制到结果缓冲区
    for (int i = len - 1; i >= 0; i--) {
        *s++ = temp_str[i];
    }

    // 字符串结束
    *s = '\0';

    u8g2_DrawStr(u8g2, x, y, temp_buff);
}

// 16位无符号整数的十六进制显示函数
void drawHex16(u8g2_t *u8g2, uint8_t x, uint8_t y, uint16_t value) {
    char temp_buff[5]; // 4个十六进制字符 + 结束符
    char hex_chars[] = "0123456789ABCDEF";

    temp_buff[0] = hex_chars[(value >> 12) & 0xF];
    temp_buff[1] = hex_chars[(value >> 8) & 0xF];
    temp_buff[2] = hex_chars[(value >> 4) & 0xF];
    temp_buff[3] = hex_chars[value & 0xF];
    temp_buff[4] = '\0';

    u8g2_DrawStr(u8g2, x, y, temp_buff);
}

// 32位无符号整数的十六进制显示函数
void drawHex32(u8g2_t *u8g2, uint8_t x, uint8_t y, uint32_t value) {
    char temp_buff[9]; // 8个十六进制字符 + 结束符
    char hex_chars[] = "0123456789ABCDEF";

    temp_buff[0] = hex_chars[(value >> 28) & 0xF];
    temp_buff[1] = hex_chars[(value >> 24) & 0xF];
    temp_buff[2] = hex_chars[(value >> 20) & 0xF];
    temp_buff[3] = hex_chars[(value >> 16) & 0xF];
    temp_buff[4] = hex_chars[(value >> 12) & 0xF];
    temp_buff[5] = hex_chars[(value >> 8) & 0xF];
    temp_buff[6] = hex_chars[(value >> 4) & 0xF];
    temp_buff[7] = hex_chars[value & 0xF];
    temp_buff[8] = '\0';

    u8g2_DrawStr(u8g2, x, y, temp_buff);
}

void drawFloat(u8g2_t *u8g2, uint8_t x, uint8_t y, float value) {
    char temp_buff[20];
    char *s = temp_buff;
    // 处理负数
    if (value < 0) {
        *s++ = '-';
        value = -value;
    }
    // 获取整数部分
    int integer_part = (int)value;
    // 获取小数部分（保留1位小数）
    int decimal_part = (int)((value - integer_part) * 10);
    // 处理整数部分为0的情况
    if (integer_part == 0 && temp_buff[0] != '-') {
        *s++ = '0';
    } else {
        // 转换整数部分
        char int_buff[10];
        int int_len = 0;
        int temp = integer_part;

        // 特殊处理整数部分为0的情况
        if (temp == 0) {
            int_buff[int_len++] = '0';
        } else {
            // 将整数转换为字符串（逆序）
            while (temp > 0) {
                int_buff[int_len++] = (temp % 10) + '0';
                temp /= 10;
            }
        }
        // 正序复制到结果缓冲区
        for (int i = int_len - 1; i >= 0; i--) {
            *s++ = int_buff[i];
        }
    }
    // 添加小数点和小数部分
    *s++ = '.';
    *s++ = (decimal_part % 10) + '0';
    // 字符串结束
    *s = '\0';
    u8g2_DrawStr(u8g2, x, y, temp_buff);
}
// 更灵活的浮点数显示函数，可以指定小数位数
void drawFloatPrec(u8g2_t *u8g2, uint8_t x, uint8_t y, float value, uint8_t prec) {
    char temp_buff[20];
    char *s = temp_buff;
    // 限制精度在合理范围内
    if (prec > 5) prec = 5;
    // 处理负数
    if (value < 0) {
        *s++ = '-';
        value = -value;
    }
    // 获取整数部分
    int integer_part = (int)value;
    // 获取小数部分
    float decimal_part = value - integer_part;
    // 处理整数部分为0的情况
    if (integer_part == 0 && temp_buff[0] != '-') {
        *s++ = '0';
    } else {
        // 转换整数部分
        char int_buff[10];
        int int_len = 0;
        int temp = integer_part;

        // 特殊处理整数部分为0的情况
        if (temp == 0) {
            int_buff[int_len++] = '0';
        } else {
            // 将整数转换为字符串（逆序）
            while (temp > 0) {
                int_buff[int_len++] = (temp % 10) + '0';
                temp /= 10;
            }
        }

        // 正序复制到结果缓冲区
        for (int i = int_len - 1; i >= 0; i--) {
            *s++ = int_buff[i];
        }
    }
    // 如果精度为0，则不显示小数部分
    if (prec > 0) {
        // 添加小数点
        *s++ = '.';
        // 处理小数部分
        for (int i = 0; i < prec; i++) {
            decimal_part *= 10;
            int digit = (int)decimal_part;
            *s++ = digit + '0';
            decimal_part -= digit;
        }
    }
    // 字符串结束
    *s = '\0';
    u8g2_DrawStr(u8g2, x, y, temp_buff);
}



