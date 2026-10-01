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
# 注意：这个文件 CubeMX 只在第一次生成时写一次，之后 regenerate 不会覆盖，
# 所以从 hello_imu 复制过来的工程名要在这里手动改成 motor_control，
# 否则产物一直叫 hello_imu.elf / hello_imu.map。
# cmake/stm32cubemx/CMakeLists.txt 里全部用 ${CMAKE_PROJECT_NAME}，不用跟着改。
set(CMAKE_PROJECT_NAME motor_control)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# ---------------------------------------------------------------------------
# 链接脚本 / map 文件的名字
#
# 工具链文件 cmake/gcc-arm-none-eabi.cmake（CubeMX 生成，每次 Generate Code 都会重写）
# 里面写死了两项：-T STM32H723xG_flash.ld 和 -Wl,-Map=<工程名>.map。这里都摘掉：
#   - 链接脚本换成我们自己的 STM32H723xG_slots.ld，镜像基址用
#     -Wl,--defsym=APP_BASE=... 传进去（App-A / App-B / Bootloader 基址不同，
#     具体在 cmake/ota.cmake 里按目标设置）；
#   - map 名字按目标生成，不然后面加的两个 App 会往同一个文件里写。
# 用"事后修剪"而不是去改工具链文件，就是为了让 Generate Code 覆盖不掉。
# ---------------------------------------------------------------------------
string(REGEX REPLACE "-T +\"[^\"]*STM32H723xG_flash\\.ld\"" "" OTA_LINK_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
string(REGEX REPLACE "-Wl,-Map=[^ ]+" "" OTA_LINK_FLAGS "${OTA_LINK_FLAGS}")
string(STRIP "${OTA_LINK_FLAGS}" CMAKE_EXE_LINKER_FLAGS)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

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
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# User defined libraries.
# Kept in a separate file that STM32CubeMX does not generate/overwrite, so the
# driver configuration survives every "Generate Code" run.
include(cmake/bmi088.cmake)
include(cmake/ws2812.cmake)
include(cmake/uart_log.cmake)
include(cmake/motor.cmake)
include(cmake/motor_ctrl.cmake)
# 小车底盘驱动（速度环 + 里程差闭环 + 看门狗）；要链 motor_ctrl，所以放在它后面
include(cmake/motor_drive.cmake)
# 电源电压监测（PC4/ADC1_INP4）；ADC 本体由 CubeMX 生成的 Core/Src/adc.c 提供
include(cmake/power_mon.cmake)
# 无线 OTA：ota_core/ota_service + 三个镜像目标（App-A / App-B / Bootloader）
include(cmake/ota.cmake)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
