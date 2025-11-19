cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Include toolchain file for ARM cross-compilation
# This must be set BEFORE the project() command
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake)
# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME STM32_Class_Project)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        APP/scheduler.h
        APP/scheduler.c
        APP/bsp_system.h
        Core/Src/retarget.c
        APP/mq2_app.c
        APP/mq2_app.h
        APP/oled.c
        APP/oled.h
        APP/oledfont.h
        APP/oledfont.c
        APP/dht11_app.h
        APP/dht11_app.c
        APP/delay.h
        APP/mpu_app.h
        APP/mpu_app.c
        APP/eMPL_MPU/inv_mpu.c
        APP/eMPL_MPU/dmpKey.h
        APP/eMPL_MPU/dmpmap.h
        APP/eMPL_MPU/inv_mpu.h
        APP/eMPL_MPU/inv_mpu_dmp_motion_driver.c
        APP/eMPL_MPU/inv_mpu_dmp_motion_driver.h
        APP/max30102_app.c
        APP/max30102_app.h
        APP/gps_app.c
        APP/gps_app.h
        APP/ringbuffer/ringbuffer.c
        APP/ringbuffer/ringbuffer.h
        Core/Src/syscalls.c
        Core/Src/syscalls.c
        Core/Src/syscalls.c
        APP/esp32_comm.h
        APP/esp32_comm.c

)  # printf重定向

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/APP  # 添加APP目录到包含路径
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    MPU6050=1             # 定义MPU6050芯片
    EMPL_TARGET_STM32=1   # 定义STM32平台
    # Add user defined symbols
)

# 为eMPL库单独设置编译定义
set_source_files_properties(
    APP/eMPL/inv_mpu.c
    APP/eMPL/inv_mpu_dmp_motion_driver.c
    PROPERTIES
    COMPILE_DEFINITIONS "MPU6050=1;EMPL_TARGET_STM32=1"
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# 启用printf浮点数支持
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    LINKER:-u,_printf_float  # 支持printf输出浮点数
)
