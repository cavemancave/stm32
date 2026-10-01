# 代码结构 / 任务 / CMake 片段

> 返回 [README](../README.md)。协议细节见 [`motor_protocol.md`](motor_protocol.md)，
> 构建与 CubeMX 注意事项见 [`build.md`](build.md)。

`main.c` 只留外设初始化 + 5V 上电时序 + 启动调度器，业务全在 FreeRTOS 任务里。

## 模块

CMake 片段按仓库惯例用 `include()` 从根 `CMakeLists.txt` 拉进来（CubeMX 不覆盖）：

| 文件 | 干什么 | cmake 片段 |
| --- | --- | --- |
| `Device/Motor/motor.c` `inc/motor.h` | 协议层：组帧 / CRC-8/MAXIM / 校验 / `Motor_Enable` `Motor_SetMode` `Motor_SetValue` `Motor_QueryStatus` `Motor_QueryVersion` | `cmake/motor.cmake` |
| `Device/Motor/motor_io.c` `inc/motor_io.h` | 传输层：请求队列 + 独占 USART10 的收发任务 `MotorIo_Task`；对上只暴露 `MotorIo_Exchange()` | 同上 |
| `Device/Motor/motor_fmt.c` `inc/motor_fmt.h` | 纯格式化：模式名 / 故障码文本 / 角度↔位置换算 | 同上 |
| `Device/Motor/motor_ctrl.c` `inc/motor_ctrl.h` | 业务层：`MotorCtrl_PollTask`（轮询）、速度环 `MotorCtrl_PrepareSpeedLoop / SetSpeed / SetSpeeds`、位置环 `MotorCtrl_PreparePositionLoop`、无线命令入口 `MotorCtrl_RemoteCmd` | `cmake/motor_ctrl.cmake` |
| `Device/Motor/motor_drive.c` `inc/motor_drive.h` | **小车底盘**：里程差闭环（走直线）+ 看门狗，25 ms 一拍，懒创建 `motorDrive` 任务 | `cmake/motor_drive.cmake` |
| `Device/Motor/motor_power.c` `inc/motor_power.h` | 电机电源开关（**PC14**，上电默认关断；`OnAndSettle` / `PowerCycle`） | `cmake/motor.cmake` |
| `Device/UartLog/uart_log.c` `inc/uart_log.h` | 调试日志：主口 = 无线口 USART1，UART7 镜像；内部带互斥，多任务可并发打，OTA 期间可静音 | `cmake/uart_log.cmake` |
| `Device/WS2812/` | 板载 WS2812（SPI6 MOSI 波形模拟单总线）；见 [`hardware.md`](hardware.md) | `cmake/ws2812.cmake` |
| `Device/Power/` | 电源电压监测（PC4/ADC1_INP4）；见 [`hardware.md`](hardware.md) | `cmake/power_mon.cmake` |
| `Device/Ota/` | 无线 OTA：`ota_com/ota_link/ota_host/ota_session/ota_flash/ota_meta/ota_service` + Bootloader；见 [`ota.md`](ota.md) | `cmake/ota.cmake` |

## 任务 / 对象

| 任务 / 对象 | 优先级 | 说明 |
| --- | --- | --- |
| `defaultTask` | Normal | 空闲任务（没有按键/台架流程；业务全由无线命令驱动） |
| `motor_task` → `MotorIo_Task` | Low | 独占 USART10 的收发（HAL 是轮询阻塞的，所以单独给一个低优先级任务） |
| `motorPoll` | BelowNormal | 被定时器唤醒做状态轮询（CubeMX 的线程列表里没有它，在 `MotorCtrl_Init()` 里建） |
| `motorPos`（`osTimerPeriodic`） | — | 200 ms 一次；回调**只释放信号量**（跑在 timer service task 里，阻塞会把所有软件定时器卡死） |
| `motorDrive` | Normal | 小车闭环（里程差 + 看门狗），懒创建：第一次 `MotorDrive_Set` 时建，跑着的时候替掉 200 ms 轮询 |
| `otaSvc` | Normal | 无线口收帧：日志/控制命令/OTA 三合一 |
| `powerMon` | Low | 1 Hz 采样电源电压，别的任务读缓存 |

> **收发的串行化**：`MotorIo_Exchange()` 先看当前是不是收发任务自己 —— 不是就把请求丢进队列
> 再等任务通知，是就直接收发（否则就是自己等自己）。所以 `Motor_xxx()` 那些高层函数
> 从**任何任务**里调都是线程安全的，上层不用关心队列。
> ⚠ 队列是**值拷贝**：请求 struct 里只放指针，结果用**任务通知的值**回传；
> tx/rx 指针靠调用方阻塞等待保证有效期。
> ⚠ 收发任务只跑一个，不要在别处再起第二个。

## 无线命令

`OTA_T_CTRL` 的子命令（`OTA_CTRL_*`）定义在 `Device/Ota/inc/ota_layout.h`：
失能/使能/切位置环/走位置/急停/静音/停轮询/电源/栈余量/切速度环/给转速/加速时间/小车 DRIVE/里程差闭环 TRIM。
上位机：`tools/ota.py`（升级）、`tools/motor.py`（电机）、`tools/car.py`（小车）；
三者共用协议层 `tools/proto.py`。
