cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME BasicCar)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Drivers/BSP/Inc/key_driver.h
        Core/Inc/app_entry.h
        Drivers/BSP/Inc/button.hpp
        Drivers/BSP/Inc/LineFollower.h
        Drivers/BSP/Src/LineFollower_Interface.cpp
        Drivers/BSP/Inc/Pid.hpp
        Drivers/BSP/Inc/LineFollower_Interface.h
        Drivers/BSP/Inc/W25Q64.hpp
        Drivers/BSP/Inc/PidStorage.hpp
        Drivers/BSP/Inc/App_PidConfig.h
        Drivers/BSP/Src/App_PidConfig.cpp
        Drivers/BSP/Inc/UartRingBuffer.hpp
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here
        Drivers/BSP/SEGGER-RTT/SEGGER_RTT.c
        Drivers/BSP/SEGGER-RTT/SEGGER_RTT_printf.c
        Core/Src/app_entry.cpp
        Drivers/BSP/Src/key_driver.c
        Drivers/BSP/Src/button.cpp
        Drivers/BSP/ICM45686/IMU.c
        Drivers/BSP/ICM45686/inv_imu_transport.c
        Drivers/BSP/ICM45686/inv_imu_driver.c
        Drivers/BSP/ICM45686/read_aux_data_mode.c
        # OLED display support
        Drivers/BSP/OLED/Src/oled.c
        Drivers/BSP/OLED/Src/u8g2_hal_i2c.c
        # u8g2 library sources
        Drivers/BSP/OLED/u8g2/csrc/u8g2_setup.c
        Drivers/BSP/OLED/u8g2/csrc/u8g2_d_memory.c
        Drivers/BSP/OLED/u8g2/csrc/u8g2_d_setup.c
        Drivers/BSP/OLED/u8g2/csrc/u8g2_buffer.c
        Drivers/BSP/OLED/u8g2/csrc/u8g2_hvline.c
        Drivers/BSP/OLED/u8g2/csrc/u8g2_ll_hvline.c
        Drivers/BSP/OLED/u8g2/csrc/u8g2_box.c
        Drivers/BSP/OLED/u8g2/csrc/u8g2_font.c
        Drivers/BSP/OLED/u8g2/csrc/u8g2_fonts.c
        Drivers/BSP/OLED/u8g2/csrc/u8g2_cleardisplay.c
        Drivers/BSP/OLED/u8g2/csrc/u8g2_line.c
        Drivers/BSP/OLED/u8g2/csrc/u8g2_bitmap.c
        Drivers/BSP/OLED/u8g2/csrc/u8g2_circle.c
        Drivers/BSP/OLED/u8g2/csrc/u8g2_polygon.c
        Drivers/BSP/OLED/u8g2/csrc/u8g2_intersection.c
        Drivers/BSP/OLED/u8g2/csrc/u8g2_kerning.c
        # u8x8 library sources
        Drivers/BSP/OLED/u8g2/csrc/u8x8_setup.c
        Drivers/BSP/OLED/u8g2/csrc/u8x8_display.c
        Drivers/BSP/OLED/u8g2/csrc/u8x8_byte.c
        Drivers/BSP/OLED/u8g2/csrc/u8x8_cad.c
        Drivers/BSP/OLED/u8g2/csrc/u8x8_gpio.c
        Drivers/BSP/OLED/u8g2/csrc/u8x8_d_ssd1306_128x64_noname.c
        Drivers/BSP/OLED/u8g2/csrc/u8x8_debounce.c
        Drivers/BSP/OLED/u8g2/csrc/u8x8_input_value.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Drivers/BSP/SEGGER-RTT
        Drivers/BSP/Inc
        Core/Inc
        Drivers/BSP/ICM45686
        Drivers/BSP/OLED/Inc
        Drivers/BSP/OLED/u8g2/csrc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
