/**
 * @file oled.h
 * @brief BSP-style API for SSD1306 128x64 OLED display
 * 
 * This module provides a simple interface to initialize and control
 * an SSD1306 OLED display via I2C using the u8g2 library.
 * 
 * Hardware Configuration:
 *  - Display: SSD1306 128x64 OLED
 *  - Interface: I2C (hi2c4)
 *  - I2C Address: 0x3C (7-bit)
 *  - No RST pin used
 * 
 * Usage Example:
 * @code
 *   // In main or app initialization:
 *   OLED_Init();
 * 
 *   // In main loop:
 *   while(1) {
 *       OLED_DrawText(0, 10, "Hello World");
 *       OLED_Task();  // Updates display
 *       HAL_Delay(100);
 *   }
 * @endcode
 */

#ifndef OLED_H
#define OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the OLED display
 * 
 * This function initializes the u8g2 library and the SSD1306 display.
 * It should be called once during system initialization.
 * 
 * @return true if initialization was successful, false otherwise
 */
bool OLED_Init(void);

/**
 * @brief Periodic update task for OLED display
 * 
 * This function should be called periodically (e.g., in main loop)
 * to refresh the display content. It's non-blocking and lightweight.
 */
void OLED_Task(void);

/**
 * @brief Clear the display buffer
 * 
 * Clears the internal display buffer. Call OLED_Task() to apply changes.
 */
void OLED_Clear(void);

/**
 * @brief Draw text on the display
 * 
 * @param x X coordinate (0-127)
 * @param y Y coordinate (0-63)
 * @param text Null-terminated string to display
 * 
 * Note: Changes are buffered. Call OLED_Task() to update the display.
 */
void OLED_DrawText(uint8_t x, uint8_t y, const char *text);

/**
 * @brief Draw a pixel on the display
 * 
 * @param x X coordinate (0-127)
 * @param y Y coordinate (0-63)
 */
void OLED_DrawPixel(uint8_t x, uint8_t y);

/**
 * @brief Draw a line on the display
 * 
 * @param x0 Starting X coordinate
 * @param y0 Starting Y coordinate
 * @param x1 Ending X coordinate
 * @param y1 Ending Y coordinate
 */
void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);

/**
 * @brief Draw a box/rectangle on the display
 * 
 * @param x X coordinate of top-left corner
 * @param y Y coordinate of top-left corner
 * @param w Width
 * @param h Height
 */
void OLED_DrawBox(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

#ifdef __cplusplus
}
#endif

#endif /* OLED_H */
