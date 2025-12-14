# SSD1306 OLED Display Driver

This module provides support for SSD1306 128x64 OLED displays using the u8g2 library.

## Hardware Configuration

- **Display**: SSD1306 128x64 OLED
- **Interface**: I2C (Hardware I2C4)
- **I2C Address**: 0x3C (7-bit)
- **Reset Pin**: Not used (software-only initialization)

## Hardware Connections

| OLED Pin | STM32 Pin | Description |
|----------|-----------|-------------|
| VCC      | 3.3V      | Power supply |
| GND      | GND       | Ground |
| SCL      | I2C4_SCL  | I2C Clock |
| SDA      | I2C4_SDA  | I2C Data |

**Note**: No RST (reset) pin connection is required. The driver initializes the display via I2C commands only.

## Software Architecture

### Directory Structure

```
Drivers/BSP/OLED/
├── Inc/
│   └── oled.h              # BSP API header
├── Src/
│   ├── oled.c              # BSP API implementation
│   └── u8g2_hal_i2c.c      # HAL I2C callbacks for u8g2
└── u8g2/                   # Vendored u8g2 library (minimal subset)
    ├── u8g2.h
    ├── u8g2_setup.c
    ├── u8g2_d_memory.c
    ├── u8g2_d_setup.c
    ├── u8g2_buffer.c
    ├── u8g2_fonts.c
    └── ... (other u8g2 core files)
```

### API Functions

#### `bool OLED_Init(void)`
Initializes the OLED display. Must be called once during system startup after I2C is initialized.

**Returns**: `true` on success, `false` on failure

#### `void OLED_Task(void)`
Sends the display buffer to the OLED. Call this periodically in the main loop to update the display.

**Note**: This is a non-blocking function, but I2C transfer takes some time. Recommended to call every 100ms or slower.

#### `void OLED_Clear(void)`
Clears the display buffer. Changes take effect after calling `OLED_Task()`.

#### `void OLED_DrawText(uint8_t x, uint8_t y, const char *text)`
Draws text at the specified position.
- **x**: X coordinate (0-127)
- **y**: Y coordinate (0-63) - baseline of text
- **text**: Null-terminated string

#### `void OLED_DrawPixel(uint8_t x, uint8_t y)`
Draws a single pixel.

#### `void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)`
Draws a line between two points.

#### `void OLED_DrawBox(uint8_t x, uint8_t y, uint8_t w, uint8_t h)`
Draws a filled rectangle.

## Usage Example

```cpp
#include "oled.h"

void setup() {
    // Initialize I2C4 first (done in main.c via MX_I2C4_Init())
    
    // Initialize OLED
    if (OLED_Init()) {
        // Display startup message
        OLED_Clear();
        OLED_DrawText(0, 10, "System Ready");
        OLED_DrawText(0, 25, "Version 1.0");
        OLED_Task();
    }
}

void loop() {
    // Update display content
    OLED_Clear();
    OLED_DrawText(0, 10, "Hello World");
    
    // Send buffer to display (call every 100ms or so)
    OLED_Task();
    
    HAL_Delay(100);
}
```

## u8g2 Library

This module uses the **u8g2** graphics library in **full-buffer mode** (`_f`):
- **Mode**: Full framebuffer (1024 bytes for 128x64 display)
- **Advantages**: Fast updates, no flicker, complex graphics support
- **Disadvantages**: Uses 1KB of RAM

The vendored u8g2 sources include only the essential files needed for SSD1306 support to minimize flash usage. The full u8g2 library is available at: https://github.com/olikraus/u8g2

## Font Support

The minimal u8g2_fonts.c includes:
- `u8g2_font_6x10_tf` - 6x10 pixel font (full character set)
- `u8g2_font_6x10_tr` - 6x10 pixel font (reduced character set)

Additional fonts can be added from the u8g2 repository if needed.

## Memory Usage

- **RAM**: ~1100 bytes (framebuffer + u8g2 structure)
- **Flash**: ~25 KB (u8g2 core + fonts + BSP code)

## Integration Notes

### CMakeLists.txt
The OLED sources are already integrated into the build system:
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Drivers/BSP/OLED/Src/oled.c
    Drivers/BSP/OLED/Src/u8g2_hal_i2c.c
    Drivers/BSP/OLED/u8g2/u8g2_*.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Drivers/BSP/OLED/Inc
    Drivers/BSP/OLED/u8g2
)
```

### app_entry.cpp
The OLED is initialized in `App_Start()` and updated periodically in the main loop:
```cpp
void App_Start(void) {
    // Initialize OLED
    if (OLED_Init()) {
        OLED_Clear();
        OLED_DrawText(0, 10, "BasicCar Ready");
        OLED_Task();
    }
    
    // Main loop
    while (1) {
        // ... other tasks ...
        
        // Update OLED periodically
        if (update_counter++ >= 100) {
            update_counter = 0;
            OLED_Task();
        }
    }
}
```

## Troubleshooting

### Display not turning on
- Check I2C connections (SCL, SDA)
- Verify I2C address is 0x3C
- Ensure I2C4 is initialized before calling `OLED_Init()`
- Check power supply (3.3V)

### Garbled display
- Check I2C pull-up resistors (typically 4.7kΩ)
- Verify I2C clock speed is appropriate (100kHz or 400kHz)
- Check for I2C bus conflicts with other devices

### Display not updating
- Ensure `OLED_Task()` is called regularly
- Check that drawing commands are called before `OLED_Task()`
- Verify I2C is not blocked by other operations

## Performance Considerations

- **I2C Transfer Time**: ~15ms at 400kHz for full framebuffer update
- **Update Rate**: Recommended 10-20 Hz (every 50-100ms)
- **CPU Usage**: Minimal - most time spent in I2C transfer

## References

- u8g2 library: https://github.com/olikraus/u8g2
- SSD1306 datasheet: https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
- u8g2 documentation: https://github.com/olikraus/u8g2/wiki
