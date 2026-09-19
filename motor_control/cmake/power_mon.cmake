#
# User-owned CMake fragment: 电源电压监测（Device/Power/）
#
#   power_mon.c  - ADC1_INP4(PC4) 采样 + 分压换算 + 1 Hz 后台任务
#
# ADC 本体（时钟/通道/采样时间/引脚）是 **CubeMX 生成的** Core/Src/adc.c，
# 由 cmake/stm32cubemx/CMakeLists.txt 带进构建（Generate Code 会自己维护那两行），
# 所以这里只加"上层"这一个文件。
#
# 和 cmake/bmi088.cmake / motor.cmake 一样：这个文件 CubeMX 不会覆盖，
# 加完由根 CMakeLists.txt 里的一行 `include(cmake/power_mon.cmake)` 拉起来。
#

set(POWERMON_DIR ${CMAKE_CURRENT_LIST_DIR}/../Device/Power)

add_library(power_mon OBJECT
    ${POWERMON_DIR}/power_mon.c
)

target_include_directories(power_mon PUBLIC
    ${POWERMON_DIR}/inc
)

# 继承 CubeMX 那边的头文件路径和宏（adc.h / main.h / HAL / FreeRTOS）
target_link_libraries(power_mon PUBLIC stm32cubemx)

# 打日志要用 uart_log.h（UartLog_xxx 的符号由 uart_log 这个 OBJECT 库提供，
# 见 cmake/uart_log.cmake，最终都链进同一个可执行文件，这里不用显式 link）。
target_include_directories(power_mon PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../Device/UartLog/inc)

# 同 cmake/motor.cmake：OBJECT 库必须链进可执行目标，它的 .o 才会被算进去，
# 而且 PUBLIC 的头文件路径（Device/Power/inc）也是靠这一步传给 main.c / freertos.c 的。
target_link_libraries(${CMAKE_PROJECT_NAME} power_mon)
