cmake_minimum_required(VERSION 3.12)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{PICO_SDK_PATH}/tools/CMakeLists.txt)

project(freq-simple C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

pico_sdk_init()

add_executable(${PROJECT_NAME}
    freq-simple.c
    I2C_LCD/LiquidCrystal_I2C.c
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_pwm 
    hardware_i2c 
    hardware_flash 
    hardware_sync
)

target_compile_definitions(${PROJECT_NAME}
    PUBLIC PICO_DEFAULT_I2C_SDA_PIN=20
    PUBLIC PICO_DEFAULT_I2C_SCL_PIN=21
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})
