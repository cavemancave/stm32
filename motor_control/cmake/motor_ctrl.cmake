#
# User-owned CMake fragment: motor application layer (tasks + control flow).
#
# Split out from cmake/motor.cmake on purpose: motor.c / motor_io.c / motor_fmt.c
# are protocol + transport and must not depend on the LED, while motor_ctrl.c is
# the application layer that also drives the WS2812 indicator.
#
# STM32CubeMX rewrites cmake/stm32cubemx/CMakeLists.txt on every "Generate Code",
# but this file is not generated, so it survives regenerations.
#
# Pulled in by one `include(cmake/motor_ctrl.cmake)` line in the root CMakeLists.txt.
#

set(MOTOR_CTRL_DIR ${CMAKE_CURRENT_LIST_DIR}/../Device/Motor)

# Object library: the objects are always linked into the executable,
# just like when the sources were listed directly in the executable target.
add_library(motor_ctrl OBJECT
    ${MOTOR_CTRL_DIR}/motor_ctrl.c
)

# Application layer talks to the motor driver, the LED and the log UART.
target_include_directories(motor_ctrl PUBLIC
    ${MOTOR_CTRL_DIR}/inc
)

target_link_libraries(motor_ctrl PUBLIC motor ws2812 uart_log stm32cubemx)

# Attach to the project executable.
# NOTE: plain signature - the root CMakeLists.txt also uses the plain form,
# mixing plain and keyword signature for the same target is an error.
target_link_libraries(${CMAKE_PROJECT_NAME} motor_ctrl)
