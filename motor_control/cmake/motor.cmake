#
# User-owned CMake fragment: serial motor driver (protocol + transport).
#
#   motor.c     - 协议层：组帧 / CRC-8/MAXIM / 校验 / 各 Motor_xxx() 高层命令
#   motor_io.c  - 传输层：请求队列 + 独占 USART10 的收发任务（FreeRTOS）
#   motor_fmt.c - 格式化：模式名 / 故障码文本 / 角度换算
#
# 应用层（业务任务、主控制流程）在 cmake/motor_ctrl.cmake 里，见那个文件。
#
# Same idea as cmake/bmi088.cmake: STM32CubeMX rewrites
# cmake/stm32cubemx/CMakeLists.txt on every "Generate Code", but this file is not
# generated, so the driver stays in the build across regenerations.
#
# Pulled in by one `include(cmake/motor.cmake)` line in the root CMakeLists.txt
# (that file is only written once, on the very first generation).
#

set(MOTOR_DIR ${CMAKE_CURRENT_LIST_DIR}/../Device/Motor)

# Object library: the objects are always linked into the executable,
# just like when the sources were listed directly in the executable target.
add_library(motor OBJECT
    ${MOTOR_DIR}/motor.c
    ${MOTOR_DIR}/motor_io.c
    ${MOTOR_DIR}/motor_fmt.c
)

# Driver headers (the host application includes "motor.h" / "motor_io.h" / "motor_fmt.h")
target_include_directories(motor PUBLIC
    ${MOTOR_DIR}/inc
)

# Inherit the STM32CubeMX include paths and symbols (main.h, usart.h, FreeRTOS, HAL, CMSIS)
target_link_libraries(motor PUBLIC stm32cubemx)

# 传输层会打日志（MotorIo_Init 的启动信息 / "RX 一直是脏的" 警告），需要 uart_log 的头文件。
# 只有 motor_io.c 用得到，所以是 PRIVATE；UartLog_xxx 的符号由 uart_log 这个 OBJECT 库提供
# （见 cmake/uart_log.cmake），最终都链进同一个可执行文件，这里不用显式 link。
target_include_directories(motor PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../Device/UartLog/inc
)

# Attach to the project executable.
# NOTE: plain signature - the root CMakeLists.txt also uses the plain form,
# mixing plain and keyword signature for the same target is an error.
target_link_libraries(${CMAKE_PROJECT_NAME} motor)
