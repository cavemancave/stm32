#
# User-owned CMake fragment: serial motor driver (version query).
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
)

# Driver headers (the host application includes "motor.h")
target_include_directories(motor PUBLIC
    ${MOTOR_DIR}/inc
)

# Inherit the STM32CubeMX include paths and symbols (main.h, usart.h, HAL, CMSIS)
target_link_libraries(motor PUBLIC stm32cubemx)

# Attach to the project executable.
# NOTE: plain signature - the root CMakeLists.txt also uses the plain form,
# mixing plain and keyword signature for the same target is an error.
target_link_libraries(${CMAKE_PROJECT_NAME} motor)
