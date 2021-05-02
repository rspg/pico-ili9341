# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "C:/Users/yuki/Documents/Pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(SDFat C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_library(SDFat STATIC 
    common/FsCache.cpp
    common/FmtNumber.cpp
    common/FsDateTime.cpp
    common/FsStructs.cpp
    ExFatLib/ExFatVolume.cpp
    ExFatLib/ExFatPartition.cpp
    ExFatLib/ExFatFile.cpp
    ExFatLib/ExFatDbg.cpp
    ExFatLib/ExFatFilePrint.cpp
    ExFatLib/ExFatFileWrite.cpp
    ExFatLib/ExFatFormatter.cpp
    ExFatLib/upcase.cpp
    )

pico_set_program_name(SDFat "SDFat")
pico_set_program_version(SDFat "0.1")

pico_enable_stdio_uart(SDFat 0)
pico_enable_stdio_usb(SDFat 0)

# Add the standard library to the build
target_link_libraries(SDFat pico_stdlib)

add_definitions(-DRP2040=1)
include_directories(.)



