# OLED Display Driver (SSD1306)

## Overview
This directory contains the driver and wrapper code for the SSD1306 128x64 OLED display using the u8g2 library.

## Hardware Configuration
- **Display Controller**: SSD1306
- **Resolution**: 128x64 pixels
- **Interface**: I2C
- **I2C Bus**: hi2c4
- **I2C Address**: 0x3C (7-bit)
- **Reset Pin**: Not used

## Directory Structure
```
OLED/
├── Inc/
│   └── oled.h                 # BSP-style API header
├── Src/
│   ├── oled.c                 # BSP-style API implementation
│   └── u8g2_hal_i2c.c        # u8g2 HAL callbacks for STM32
└── u8g2/
    └── csrc/                  # u8g2 library source files
        ├── u8g2.h             # Main u8g2 header
        ├── u8x8.h             # u8x8 low-level header
        ├── u8g2_*.c           # u8g2 graphics functions
        └── u8x8_*.c           # u8x8 low-level functions
```

## API Usage

### Initialization
```c
#include "oled.h"

// Initialize the display
if (OLED_Init()) {
    // Display is ready
}
```

### Drawing
```c
// Clear the buffer
OLED_Clear();

// Draw text
OLED_DrawText(0, 10, "Hello World");

// Draw shapes
OLED_DrawLine(0, 0, 127, 63);
OLED_DrawBox(10, 10, 50, 20);
OLED_DrawPixel(64, 32);

// Update the display
OLED_Task();
```

### Periodic Update
Call `OLED_Task()` periodically (e.g., in the main loop) to refresh the display:
```c
while(1) {
    // Your code here
    OLED_Task();  // Updates display
    HAL_Delay(10);
}
```

## u8g2 Library
The u8g2 library is vendored in the `u8g2/csrc/` directory. The library is sourced from:
- **Repository**: https://github.com/olikraus/u8g2
- **License**: BSD 2-Clause License (see headers in source files)

### Included Components
- Full buffer mode graphics functions
- SSD1306 128x64 I2C driver
- Basic font rendering
- Shape drawing (lines, boxes, circles, etc.)
- Bitmap support

## Integration Notes
- The `u8g2_hal_i2c.c` file implements STM32 HAL-specific callbacks for I2C communication
- Full buffer mode (`_f`) is used for maximum flexibility
- No rotation is applied to the display (`U8G2_R0`)
- The display uses the default u8g2 font: `u8g2_font_6x10_tf`

## CMake Configuration
The u8g2 sources are included in the main `CMakeLists.txt`:
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Drivers/BSP/OLED/Src/oled.c
    Drivers/BSP/OLED/Src/u8g2_hal_i2c.c
    Drivers/BSP/OLED/u8g2/csrc/u8g2_*.c
    Drivers/BSP/OLED/u8g2/csrc/u8x8_*.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Drivers/BSP/OLED/Inc
    Drivers/BSP/OLED/u8g2/csrc
)
```
