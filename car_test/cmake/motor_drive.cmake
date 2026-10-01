#
# User-owned CMake fragment: car drive loop (odometry trim + watchdog).
#
#   motor_drive.c - 小车底盘驱动：里程差闭环（走直线）+ 看门狗，自带一个 25 ms 的任务
#
# STM32CubeMX rewrites cmake/stm32cubemx/CMakeLists.txt on every "Generate Code",
# but this file is not generated, so it survives regenerations.
#
# Pulled in by one `include(cmake/motor_drive.cmake)` line in the root CMakeLists.txt
# （放在 motor_ctrl.cmake **后面**：下面要链 motor_ctrl 这个 target）。
#
# ⚠ 它和 motor_ctrl 是**互相调用**的（motor_ctrl.c 调 MotorDrive_*，motor_drive.c 调
#   MotorCtrl_SetSpeeds / PollPause），但这里**只声明单向依赖**：
#   两个都是 OBJECT 库、对象最后都并进同一个可执行文件，符号在链接那一步自然就齐了，
#   所以不需要（也不能）互相 target_link_libraries —— 那会构成 CMake 的依赖环。
#   motor_ctrl.c 能 #include "motor_drive.h" 是因为两边共用同一个头目录
#   Device/Motor/inc（motor_ctrl 自己的 include 目录已经覆盖了）。
#

set(MOTOR_DRIVE_DIR ${CMAKE_CURRENT_LIST_DIR}/../Device/Motor)

add_library(motor_drive OBJECT
    ${MOTOR_DRIVE_DIR}/motor_drive.c
)

target_include_directories(motor_drive PUBLIC
    ${MOTOR_DRIVE_DIR}/inc
)

target_link_libraries(motor_drive PUBLIC motor motor_ctrl uart_log stm32cubemx)

# Attach to the project executable.
# NOTE: plain signature - the root CMakeLists.txt also uses the plain form,
# mixing plain and keyword signature for the same target is an error.
target_link_libraries(${CMAKE_PROJECT_NAME} motor_drive)
