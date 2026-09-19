#
# User-owned CMake fragment: 无线 OTA（Device/Ota/）+ 三个镜像目标
#
#   ota_core   OBJECT  组帧/Flash/元数据/会话/协议 —— Bootloader 和 App 共用（不依赖 RTOS）
#   ota_service OBJECT App 侧服务（RTOS 任务 + 控制命令 + 启动确认）
#
#   motor_control        App，链接到 Slot A(0x08020000) —— 主产物
#   motor_control_slotB  App，**同一份源码**、只换链接基址到 Slot B(0x08080000)
#   motor_boot           Bootloader，链接到 0x08000000（独立最小工程，不含 FreeRTOS/App 代码）
#
# 为什么要有两个 App 目标：镜像里的绝对地址（向量表、字面量池、跳转）都是链接期定死的，
# 所以"能在 A 槽跑"和"能在 B 槽跑"的固件必须是两份按不同基址链接出来的 .bin。
# 上位机 tools/ota.py 会根据设备上报的"当前活动槽"自动挑对应那份。
#
# 链接脚本统一用 STM32H723xG_slots.ld + --defsym APP_BASE/APP_SIZE（见根 CMakeLists.txt 里
# 把工具链里写死的 -T 摘掉的那几行）。
#
# ⚠ 本文件是用户文件，CubeMX "Generate Code" 不会覆盖。
#

set(OTA_DIR ${CMAKE_CURRENT_LIST_DIR}/../Device/Ota)

# ---------------------------------------------------------------------------
# ota_apply_slot(<target> <基址> <大小>)
#   给目标套上"跑在哪个槽"的链接参数，并在链接后生成 .bin / .hex（无线升级发 .bin）
# ---------------------------------------------------------------------------
function(ota_apply_slot target base size)
    target_link_options(${target} PRIVATE
        -T "${CMAKE_SOURCE_DIR}/STM32H723xG_slots.ld"
        -Wl,-Map=${CMAKE_BINARY_DIR}/${target}.map
        -Wl,--defsym=APP_BASE=${base}
        -Wl,--defsym=APP_SIZE=${size}
    )

    add_custom_command(TARGET ${target} POST_BUILD
        # ⚠ 这里**不要**加 `--gap-fill 0xFF`（试过，错的）：加了之后 objcopy 会把
        #   最低 section 到最高 section（包含了 .data 的 RAM 地址 0x2000_0000）
        #   整段范围都补出来，.bin 直接变成 400 MB。保持默认行为。
        #
        # 关于 .bin 的 CRC32 和"设备按 Flash 算的 CRC32"：
        #   链接脚本留下的对齐空洞（例如 0x080202CC 那 4 字节）在 .bin 里是 0x00，
        #   而 ST-Link 烧 .hex 时那些位置是**跳过**的、保持擦除态 0xFF，
        #   所以同一个镜像会算出两个 CRC（实测 0x27F6680E vs 0xABAA221F）。
        #   这不影响功能：设备侧永远按"Flash 内容"自己算、自己比，
        #   元数据里登记的也是它自己算的那个值；走 OTA 时空洞被写成 0x00，
        #   所以设备算出来的和上位机的 .bin CRC 又会对上（详见 README「关于 CRC32」）。
        COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:${target}>" "${CMAKE_BINARY_DIR}/${target}.bin"
        COMMAND ${CMAKE_OBJCOPY} -O ihex "$<TARGET_FILE:${target}>" "${CMAKE_BINARY_DIR}/${target}.hex"
        COMMAND ${CMAKE_SIZE} "$<TARGET_FILE:${target}>"
        COMMENT "==> ${target}: ${target}.bin / .hex 已生成（无线升级发 .bin）"
        VERBATIM
    )
endfunction()

# ---------------------------------------------------------------------------
# 公共层：Bootloader 和 App 都用（只用 HAL + 标准库，不碰 FreeRTOS）
# ---------------------------------------------------------------------------
add_library(ota_core OBJECT
    ${OTA_DIR}/src/ota_crc.c
    ${OTA_DIR}/src/ota_flash.c
    ${OTA_DIR}/src/ota_meta.c
    ${OTA_DIR}/src/ota_com.c
    ${OTA_DIR}/src/ota_link.c
    ${OTA_DIR}/src/ota_session.c
    ${OTA_DIR}/src/ota_host.c
    ${OTA_DIR}/src/ota_trace.c
)

target_include_directories(ota_core PUBLIC ${OTA_DIR}/inc)
target_link_libraries(ota_core PUBLIC stm32cubemx)

# ---------------------------------------------------------------------------
# App 侧服务：RTOS 任务 + 日志钩子 + 电机控制命令
# ---------------------------------------------------------------------------
add_library(ota_service OBJECT
    ${OTA_DIR}/src/ota_service.c
)

target_include_directories(ota_service PUBLIC ${OTA_DIR}/inc)
target_link_libraries(ota_service PUBLIC ota_core motor_ctrl uart_log stm32cubemx)

# motor_ctrl.c 里的"无线控制入口"（MotorCtrl_RemoteCmd）用的就是 OTA 协议的
# 命令码/状态码（ota_layout.h），而 motor_ctrl 目标在 ota.cmake 之前就建好了，
# 所以这里补一条 include 路径（只影响编译，不影响链接）。
target_include_directories(motor_ctrl PRIVATE ${OTA_DIR}/inc)

# ---------------------------------------------------------------------------
# App（Slot A）= 主产物 motor_control.bin
# ---------------------------------------------------------------------------
target_link_libraries(${CMAKE_PROJECT_NAME} ota_core ota_service)
ota_apply_slot(${CMAKE_PROJECT_NAME} 0x08020000 0x60000)

# ---------------------------------------------------------------------------
# App（Slot B）= motor_control_slotB.bin
# 源码和主产物完全一样（从主目标的 SOURCES 里拿），只有链接基址不同。
# ---------------------------------------------------------------------------
get_target_property(OTA_APP_SOURCES ${CMAKE_PROJECT_NAME} SOURCES)

add_executable(${CMAKE_PROJECT_NAME}_slotB ${OTA_APP_SOURCES})

target_link_libraries(${CMAKE_PROJECT_NAME}_slotB
    stm32cubemx
    STM32_Drivers
    FreeRTOS
    ota_core
    ota_service
    bmi088
    ws2812
    motor
    motor_ctrl
    uart_log
)

ota_apply_slot(${CMAKE_PROJECT_NAME}_slotB 0x08080000 0x60000)

# ---------------------------------------------------------------------------
# Bootloader = motor_boot.bin（0x08000000，128K 扇区 0）
# 只链接 startup + sysmem.c + ota_boot.c + HAL + ota_core：
#   - 没有 FreeRTOS（BL 不需要，越小越不容易坏）
#   - 没有 App 的任何业务代码
#   - 用 SysTick 当 HAL 时基（stm32h7xx_hal_timebase_tim.c 是 App 侧的，不链进来）
#   - sysmem.c 只为满足 newlib 的 _sbrk（snprintf 会把它牵进来），不是真要用堆
# ---------------------------------------------------------------------------
add_executable(motor_boot
    ${CMAKE_SOURCE_DIR}/startup_stm32h723xx.s
    ${CMAKE_SOURCE_DIR}/Core/Src/sysmem.c
    ${OTA_DIR}/src/ota_boot.c
)

target_link_libraries(motor_boot
    stm32cubemx
    STM32_Drivers
    ota_core
)

ota_apply_slot(motor_boot 0x08000000 0x20000)
