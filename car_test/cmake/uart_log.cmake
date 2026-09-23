#
# User-owned CMake fragment: debug log over UART7.
#
# Same idea as cmake/bmi088.cmake: STM32CubeMX rewrites
# cmake/stm32cubemx/CMakeLists.txt on every "Generate Code", but this file is not
# generated, so it survives regenerations.
#
# Pulled in by one `include(cmake/uart_log.cmake)` line in the root CMakeLists.txt
# (that file is only written once, on the very first generation).
#

set(UART_LOG_DIR ${CMAKE_CURRENT_LIST_DIR}/../Device/UartLog)

# Object library: the objects are always linked into the executable,
# just like when the sources were listed directly in the executable target.
add_library(uart_log OBJECT
    ${UART_LOG_DIR}/uart_log.c
)

# Driver header (the host application includes "uart_log.h")
target_include_directories(uart_log PUBLIC
    ${UART_LOG_DIR}/inc
)

# Inherit the STM32CubeMX include paths and symbols (main.h, FreeRTOS, HAL, CMSIS)
target_link_libraries(uart_log PUBLIC stm32cubemx)

# Attach to the project executable.
# NOTE: plain signature - the root CMakeLists.txt also uses the plain form,
# mixing plain and keyword signature for the same target is an error.
target_link_libraries(${CMAKE_PROJECT_NAME} uart_log)
