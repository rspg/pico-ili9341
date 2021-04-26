# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "C:/Users/yuki/Documents/Pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(test C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(test test.cpp )

pico_set_program_name(test "test")
pico_set_program_version(test "0.1")

pico_enable_stdio_uart(test 0)
pico_enable_stdio_usb(test 1)

# Add the standard library to the build
target_link_libraries(test pico_stdlib)

# Add any user requested libraries
target_link_libraries(test
        hardware_spi
        hardware_dma
        hardware_timer
        )

pico_add_extra_outputs(test)

