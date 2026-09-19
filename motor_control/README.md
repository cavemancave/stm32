# motor_control

STM32H723VGT6（达妙 DM-MC-Board02 / CtrBoard-H7）上的**串口电机控制测试**固件，
从 `hello_imu` 工程复制而来。

- **当前主流程（FreeRTOS）**：`MotorCtrl_Task` 等按键（PA15 / USER_KEY）→ 使能（0xA0/0x08，
  **失败会重试**）→ 查版本号（0xFD）→ 切**位置环**（0xA0/0x03）→ **再查一次模式确认真的在 0x03**
  → **单次测试**：先回 0°，再分 **4 步（每步 90°）走满一圈**，每步打一行
  「命令的位置 → 实际停的位置 + 误差 + 里程差」
  → **再按一下键就失能**（0xA0/0x09）→ 回到等按键（按键是「使能 ↔ 失能」切换）；
  另有一个 `MotorCtrl_PollTask` 被 200 ms 的 osTimer 唤醒，查一轮里程/位置/故障码（0x74）。
  结果全部打印到 UART7（见「电机测试」一节），板载 WS2812 指示状态。
  ⚠ 原来的 0°（12 点）↔ 90°（3 点）来回走没删，用 `motor_ctrl.c` 里的 `MOTOR_SINGLE_MOVE_TEST` 开关
  （`1U` = 单次测试，`0U` = 恢复往返）切换。
- **BMI088 那套暂时关掉**：初始化、打包、100 Hz 发 20 字节二进制包三处都用 `#if 0`
  关着（`Core/Src/main.c`）。下面「上位机」「串口输出」两节留着备查，
  要跑 IMU 时把这三处 `#if 0` 改回 `#if 1` 即可。

## 上位机：网页版（BMI088 部分，暂时关掉）

![网页版上位机界面](docs/imu-web-viewer.png)

> 上图是网页自带 **Demo Mode**（模拟数据）的截图，只为展示界面长什么样；
> 接上单板后 3D 模型和 Roll / Pitch 会跟着板子一起动。

直接用 Chrome / Edge（需要 Web Serial API）打开 <https://imu.steppeschool.com/>，
接线后按下表设置，点 `Connect` 选串口即可：

| 控件 | 选什么 | 说明 |
| --- | --- | --- |
| Baud | **115200** | 必须和固件里的 UART7 波特率一致（固件只用到网站支持的这几档） |
| Filter | 随便 | 只是网页上的姿态解算方式，不影响串口数据 |
| Gyro | **±2000 dps** | 固件里陀螺仪量程是 `BMI088_GYRO_RANGE_2000`，选错转动速度会不对 |

- 加速度量程不用在网页上设置：网页只拿加速度算姿态方向，比例不影响结果。
- 本板没有磁力计，包里的磁力 3 轴固定填 0，所以 yaw 没有绝对方位参考，会漂（网页上
  `Mag Cal` 也用不上）。
- 网页会对 `ax / gy / gz` 取反（这是参考板的贴装方向）。本板 BMI088 的贴装和参考板不同，
  固件里做了一套轴向映射：**X = -传感器 Y、Y = 传感器 X、Z 不变**（加速度和陀螺同一套）。
  如果 3D 模型的转动方向还是相反/镜像，继续改 `Core/Src/main.c` 里打包那 6 个分量的
  符号/顺序即可。

- 红灯常亮 = BMI088 还没初始化成功（会一直重试，错误码打串口）
- 绿灯常亮 = 初始化成功，开始输出数据

## 电机测试（当前的主流程，BMI088 暂时关掉）

这一版先不跑 IMU（当前只挂 1 台电机，ID1），业务已经全部搬进 FreeRTOS 任务：

```
上电 → 按下 USER_KEY → 使能(重试到有应答) → 查版本号 → 切位置环(0xA0/0x03) → 再查一次模式(0x75)
     → 单次测试：回 0° → 0°→90°→180°→270°→360°（分 4 步），每步打位置/误差/里程
       （同时另一个任务每 200 ms 查一轮 0x74）
     → 再按一下 USER_KEY → 失能(0xA0/0x09) → 回到等按键（再按又使能）
```

**代码组织**（`main.c` 只留外设初始化 + 5V 上电时序 + 启动调度器，不再有业务逻辑）。
CMake 片段和驱动一样按仓库惯例用 `include()` 从根 `CMakeLists.txt` 拉进来：

| 文件 | 干什么 | cmake 片段 |
| --- | --- | --- |
| `Device/Motor/motor.c` `inc/motor.h` | 协议层：组帧 / CRC-8/MAXIM / 校验 / `Motor_Enable` `Motor_SetMode` `Motor_SetValue` `Motor_QueryStatus` `Motor_QueryVersion` | `cmake/motor.cmake` |
| `Device/Motor/motor_io.c` `inc/motor_io.h` | 传输层：请求队列 + 独占 USART10 的收发任务 `MotorIo_Task`；对上只暴露 `MotorIo_Exchange()` | 同上 |
| `Device/Motor/motor_fmt.c` `inc/motor_fmt.h` | 纯格式化：模式名 / 故障码文本 / 角度↔位置换算 | 同上 |
| `Device/Motor/motor_ctrl.c` `inc/motor_ctrl.h` | 业务层：`MotorCtrl_Task`（主流程）、`MotorCtrl_PollTask`（轮询）、`MotorCtrl_OnPollTimer`（定时器回调） | `cmake/motor_ctrl.cmake` |
| `Device/UartLog/uart_log.c` `inc/uart_log.h` | 调试日志：主口 = 无线口 USART1，UART7 镜像；内部带互斥，多任务可并发打，OTA 期间可静音 | `cmake/uart_log.cmake` |

| 任务 / 对象 | 优先级 | 说明 |
| --- | --- | --- |
| `defaultTask` → `MotorCtrl_Task` | Normal | 主控制流程 |
| `motor_task` → `MotorIo_Task` | Low | 独占 USART10 的收发（HAL 是轮询阻塞的，所以单独给一个低优先级任务） |
| `motorPoll` | BelowNormal | 被定时器唤醒做状态轮询（CubeMX 的线程列表里没有它，在 `MotorCtrl_Init()` 里建） |
| `motorPos`（`osTimerPeriodic`） | — | 200 ms 一次；回调**只释放信号量**（跑在 timer service task 里，阻塞会把所有软件定时器卡死） |

> **收发的串行化**：`MotorIo_Exchange()` 先看当前是不是收发任务自己 —— 不是就把请求丢进队列
> 再等任务通知，是就直接收发（否则就是自己等自己）。所以 `Motor_xxx()` 那些高层函数
> 从**任何任务**里调都是线程安全的，上层不用关心队列。
> ⚠ 队列是**值拷贝**：请求 struct 里只放指针，结果用**任务通知的值**回传；
> tx/rx 指针靠调用方阻塞等待保证有效期。
> ⚠ 收发任务只跑一个，不要在别处再起第二个。

协议摘录见 [`docs/specification.md`](docs/specification.md)（源文件 `docs/M0603A_111_motor_specification_cn.pdf`）。

| 项目 | 值 |
| --- | --- |
| 串口 | **USART10**：PE2 = `USART10_RX`、PE3 = `USART10_TX`（AF4 / AF11，CubeMX 生成） |
| ⚠ TX 电平 | **PE3 必须配成开漏 `GPIO_MODE_AF_OD`**（CubeMX 里 PE3 的 Output type = Open Drain）。电机控制板是 **5V TTL** 单总线，高电平靠总线上拉；TX 用推挽时输出 MOS 管会常通，和 5V 上拉互相灌电流/把总线拉出错误电平 → **误发信号**。改代码没用，要在 CubeMX 里改（见 `usart.c` 的 `USART10_MspInit 1` 备注） |
| 参数 | **38400**, 8N1，一问一答，固定 **10 字节**帧 |
| 电机 ID | **1（只测 1 台）** —— ID 脚接地 = ID1、接高 = ID2，上电时锁存；要挂第二台就把 `main.c` 里 `MOTOR_COUNT` 改回 `2U` 并补全 `motor_ids[]` |
| 触发 | **USER_KEY = PA15**（`MX_GPIO_Init` 里配成输入、`GPIO_NOPULL`，按下为低电平）。轮询 + 20ms 消抖 + 等松手，按住不放只算一次（`motor_ctrl.c` 里用 `osDelay`） |
| 动作①：使能 | 按一次键 → 对 `motor_ids[]` 里的电机（当前只有 ID1）使能：发 `0xA0/0x08`，**失败每 200 ms 重试、最多约 5 s**（手册要求上电后第一条命令要反复重试）→ 打印 `rx=` 原始 10 字节 |
| 动作②：失能 | **电机正在来回走的时候再按一下键** → 对每台电机发 `0xA0/0x09` 失能（失败每 100 ms 重试，最多 3 次），灯灭，打印 `rx=` 原始 10 字节 → 回到等按键（第三次按键又会使能）。按键是在「动作间隙的停顿」里查的（`MotorCtrl_DelayOrKeyPress`，切成 10 ms 一段），所以不用等当前那一步走完才响应 |
| 成功判据 | **收到合法的 `0xA1` 回帧**（ID/功能码/CRC 都对）即算使能成功。⚠ 反馈的 `DATA[2]` 是**使能之后的实际模式**，不是回显 `0x08`：实测 ID1 回 `01 A1 01 00 00 00 00 00 00 E0`，模式值 `0x01` = 默认电流环 |
| 后续①：版本号 | 使能 OK 后各查一次 `0xFD`（等 `0xFE`），打印日期（年 = 20xx）/ 型号 / 软硬件版本；失败也把收到的 10 字节打出来 |
| 后续②：切位置环 | 发 `0xA0`，模式值 `0x03`。⚠ **手册 5.3 的模式表里没有位置环**，这个值来自厂家上位机工程（`docs/M0603A 系列直驱电机.bit` 的模式值下拉框里有「位置环 = 03」）和 `docs/M63A快捷指令集.cfg` 里的「电机1切换位置环」；失败就回到等按键 |
| 后续②b：确认模式 | 切完**再查一次模式**（`0x75`/`0x76`，紧跟切模式之后）：必须报 `0x03` 才继续，否则打 `position loop NOT active (mode=0x..)`、失能、退回等按键 —— 因为不认 0x03 的固件也会回一个合法的 `0xA1`，光看「有没有回帧」会误判 |
| 后续③：单次测试 | 位置环下用 `0x64` 把给定值当**目标位置**（0~32767 ↔ 0~360°）：先走 `0`（0°），再分 `MOTOR_TEST_STEPS`(4) 步、每步 `MOTOR_TEST_STEP_POS`(8191 = 90°) 走到一整圈。每步**等到位置不再变**（每 200 ms 读一次 `0x74`，连续 3 次变化 ≤10 计数 算停稳；最多 8 s，可被按键打断）后打一行 `single move 2/4 -> 180.0deg: position 61 (0.6deg) -> 16444 (180.7deg), error=+61 (+0.7deg), mileage 0 -> 0 (0 turns)`。**`error` 就是「这一步走够没有」**（命令 90° 却只走了 45° → `error` = `-4096` = `-45.0°`），4 步走完里程应该 `+1`。要恢复 0°↔90° 往返，把 `MOTOR_SINGLE_MOVE_TEST` 改成 `0U` |
| ⚠️ 位置环停稳很慢 | 一段 90° 大约要 **1.7~2 s** 才停稳，而且是**接近目标时慢慢爬过去**的（实测每 200 ms 还有 200~700 计数 = 1~4°/200ms，越接近越慢）。所以测量必须等它停稳 —— 固定等 1.5 s 会读到“还差 8~13°”的中间值，**误判成“没走够”**。这不是分辨率/颗粒度问题：编码器 12 位、位置值 32768/圈 ≈ 0.011°，分辨率比这个误差细两个数量级 |
| ❓待确认：`0x65` 的“速度”字段 | 位置环下它看着**不像速度**：电机停着时是 `-4`（≈0），而电机在动的那几次是 `7415 / 15410 / 23502`，和当时的**位置值**（`7424 / 15400 / 23493`）几乎一样。要用这个字段当速度得先单独验一下 |
| ⚠️ 位置值是一圈 | **0~32767 是「一圈」，而且 `32767 ≡ 0`（同一个点）**：直接命令 `32767` 等于“回到 0 点”，位置环按**最短路径**走，电机本来就在 0° 附近 → 它压根不动（实测：命令 32767 后 `position` 一直停在 `61/65`（0.6~0.7°）、里程 `0 -> 0`，看着像“没转”）。**要走满一圈必须分步走**；想验证“90° 走够没有”，也要么分步走、要么命令一个中间角度再读 `0x74` 对比 |
| 后续④：失能 | 来回走的时候再按一下键 → 发 `0xA0/0x09`（`Motor_Disable`）失能每台电机，把回帧的 `mode` 也打出来，然后回到等按键。失能只是「不使劲」，通信还在，所以 `motorPoll` 那一路的 0x74 轮询照旧 |
| 状态轮询 | 定时器每 **200 ms**（`MOTOR_CTRL_POLL_MS`）唤醒 `MotorCtrl_PollTask`，查一轮 `0x74`（等 `0x75`），打印**里程圈数、位置原始值（并换算成 0.1°）、故障码（按位译成 hall/overcurrent/stall/overtemp/link-loss/voltage，未知位写 other）** |
| 灯 | 只由 `MotorCtrl_Task` **一个**任务点（WS2812 走 SPI6，两个任务同时点会把帧打断）。每次点灯从枚举里**依次**取一个颜色（`RED → GREEN → BLUE → YELLOW → CYAN → MAGENTA → WHITE → RED`，跳过 `OFF`）：上电红 → 按键后绿 → 之后由「每走一步换一个颜色」当心跳；**失能时直接点 `OFF`（灭）**，看到灯灭就知道电机不使劲了（再按键使能时接着按枚举换下一个颜色）。亮度不在业务里设，用驱动默认 `WS2812_DEFAULT_BRIGHTNESS = 32`（含义是 R+G+B 之和；要调就改 `ws2812.h` 那个宏或开机调 `WS2812_SetBrightness()`），所以 红/绿/蓝 分别是 `(32,0,0)`、`(0,32,0)`、`(0,0,32)` |
| 日志 | 打在 **无线口 USART1**（115200）上，同时镜像到 UART7，见「无线 OTA 升级」和「串口输出」两节 |

帧格式：`ID | 功能码 | DATA[2..8] | CRC8`

- 使能 `ID | 0xA0 | 0x08 0 0 0 0 0 0 | CRC8` → 反馈 `ID | 0xA1 | 模式值 0 0 0 0 0 0 | CRC8`
  （反馈的模式值是**切换后的实际模式**：发 0x08 使能后回的是 `0x01` 电流环，实测帧 `01 A1 01 00 00 00 00 00 00 E0`，CRC 已对过）
- 失能 `ID | 0xA0 | 0x09 0 0 0 0 0 0 | CRC8` → 反馈 `ID | 0xA1 | 模式值 0 0 0 0 0 0 | CRC8`
- 切位置环 `ID | 0xA0 | 0x03 0 0 0 0 0 0 | CRC8` → 反馈 `ID | 0xA1 | 0x03 0 0 0 0 0 0 | CRC8`
  （发送帧 CRC = `0xD9`，见 `docs/M63A快捷指令集.cfg`；0x03 不在手册 5.3 的模式表里）
- 走位置 `ID | 0x64 | 位置H 位置L 0 0 加速时间 刹车 0 | CRC8` → 反馈 `ID | 0x65 | 速度H 速度L 电流H 电流L 加速时间 温度 故障码 | CRC8`
  （位置值 0~32767 ↔ 0~360°，所以 0° = 0、90° = 8191，换算用 `Motor_AngleToPosition()`。
  参考 cfg 里的「电机1旋转转速30RPM」= `01 64 01 2C 00 00 00 00 00 A6`：`0x012C` = 300，
  而速度环范围是 ±3800 ↔ ±380rpm，所以这个数是 **rpm x 10**）
- 版本号查询 `ID | 0xFD | 0 0 0 0 0 0 0 | CRC8` → 反馈 `ID | 0xFE | 年 月 日 型号 软件版本 硬件版本 保留 | CRC8`
  （年字节是 20XX 的 XX，2021 年 = `0x15` = 十进制 21，所以日志里直接按十进制打）
- 状态查询 `ID | 0x74 | 0 0 0 0 0 0 0 | CRC8` → 反馈 `ID | 0x75 | 里程×4 位置H 位置L 故障码 | CRC8`

- 驱动在 `Device/Motor/`：协议 `motor.c`、收发 `motor_io.c`、格式化 `motor_fmt.c`、业务 `motor_ctrl.c`；
  cmake 片段 `cmake/motor.cmake` + `cmake/motor_ctrl.cmake`，按仓库惯例由根 `CMakeLists.txt`
  里的 `include()` 拉进来，regenerate 不会丢。日志在 `Device/UartLog/`（`cmake/uart_log.cmake`）。
- 驱动里还实现了手册的其它命令，暂时没在主流程用：
  `Motor_QueryMode()`（0x75→0x76）、`Motor_Disable()`（0x09）。底层都走同一个 `Motor_Transaction()`。
- ⚠ **CRC**：手册「CRC8 值」一节写明是 **CRC-8/MAXIM**（多项式 x⁸+x⁵+x⁴+1，反射，初值 0x00），
  所以代码里用反射形式的多项式 `0x8C`（`MOTOR_CRC8_POLY`）。校验值自查：对 `"123456789"`
  应得 `0xA1`。ID1 使能帧 = `01 A0 08 00 00 00 00 00 00 6F`。
- ⚠ **按键电平**：`main.c` 里的 `USER_KEY_PRESSED_LEVEL` 默认 `GPIO_PIN_RESET`（按下接地）。
  如果实测反了（不按也一直触发），改成 `GPIO_PIN_SET`。
- ⚠ PA15 默认是 **JTDI**：配成 GPIO 后仍可用 SWD（PA13/PA14）调试，不要再接 JTAG。
- ⚠ **位置环（0x03）必须先确认真的切过去了**：`0xA0/0x03` 的反馈 `DATA[2]` 是**切换后的实际模式**，
  电机不认 0x03 时也会回一个合法的 `0xA1`（只是模式值不是 0x03），此时它还在电流环/速度环 ——
  再发 `0x64`（我们按“位置”给的 8191）就被当成电流/转速，**电机会一直转下去**。
  现在主流程（`MotorCtrl_Task`）的步骤：
  ① `0x75` 查一次模式（切换前，应该报 `0x01` 电流环）→ ② 发 `0xA0/0x03` →
  ③ **再查一次模式**（`0x75`/`0x76`，就在 `MotorCtrl_EnterPositionLoop()` 返回之后），
  **必须报 `0x03`** 才往下走；否则打 `position loop NOT active (mode=0x..)`、失能、退回等按键，
  不进来回走。
  排查时先看 `mode before switch` / `mode after switch` 下面那两行 `mode query (0x75)`，
  以及 `set mode (0xA0/0x03 ...)` 这行的 `mode=` 是多少
  （手册 5.3 的模式表里**没有** 0x03，它只出现在厂家上位机工程和 `docs/M63A快捷指令集.cfg`，
  所以不保证所有固件都认）。
- 🛡 **防跑飞**（`motor_ctrl.c` 的 `MotorCtrl_TurnsRunaway()`）：来回走时每一步都查一次里程（`0x74`），
  只让走 90° 却让里程变化超过 `MOTOR_MAX_TURNS_PER_MOVE`（1 圈）= 电机没在位置环里，
  立刻 `0x64 value=0` 急停 + `0xA0/0x09` 失能，并打 `RUNAWAY: mileage ...`，不让它继续转。
- 排查：`rx=` **全 0** = 一个字节都没收到（没接线 / 没上电 / 波特率不对）；
  有数据但 `reply=NONE` = 帧对不上（CRC 或错位），把 raw 打出来比对。
- BMI088 的初始化、打包、发送三处都用 `#if 0` 关着（`Core/Src/main.c`），
  要放开时把这三处改回 `#if 1`，并同时把 WS2812 的绿灯状态逻辑接回去。

## 无线 OTA 升级（以后不用再插 ST-Link）

无线模块接在 **USART1（PA9 = TX / PA10 = RX）** 上，电脑那边是一个配对的串口。
这一个口平时就是**调试日志 + 控制命令**，要升级时同一套协议里夹着传固件。
设计和取舍（为什么 A/B 双槽、为什么单 bank 不能自己改自己、各种异常怎么办）见
[`docs/ota_design.md`](docs/ota_design.md)；这一节只说怎么用。

### Flash 怎么分的、为什么不会变砖

```
0x08000000  128K  Bootloader（永不 OTA 更新，只在第一次用 ST-Link 烧）
0x08020000  384K  Slot A  ←  motor_control.bin
0x08080000  384K  Slot B  ←  motor_control_slotB.bin（同一份源码、只换链接基址）
0x080E0000  128K  元数据（在哪个槽 / 版本 / CRC / 续传进度，顺序追加写）
```

* 正在跑的槽**永远不写**（协议里直接拒绘）—— 新固件写进另一个槽，校验通过后重启，由 Bootloader 切槽；
* 新固件跑起来 2 s 后要「报到」（写启动确认）。连着 3 次没报到 = 判坏，**自动回滚**到旧槽；
* 两个槽都跑不起来 → 上电自动进 Bootloader 的**恢复台**，同一个口重发固件就行
  （`flash --slot A/B` 指定写哪个槽）；
* 传输断了/掉电了不要紧：进度每 32 KB 落一次盘，重新 `flash` 会从断点续传；
* Bootloader 自己不 OTA（H723 单 bank 没有硬件 bank swap，自升级失败就真砖了），
  所以它只在第一次烧一次 —— 这是整个方案的地基。

### 第一次（也是最后一次）用 ST-Link

| 烧什么 | 烧到哪 |
| --- | --- |
| `build/Debug/motor_boot.hex`（或 .bin/.elf） | `0x08000000` |
| `build/Debug/motor_control.hex`（或 .bin/.elf） | `0x08020000` |

第一次上电时元数据是空的，Bootloader 会「看着向量表像 App」先跳进去；
App 跑满 2 s 后把自己登记到元数据里（长度 + CRC32 + 版本），之后就是正常状态。

#### ⚠⚠ 烧录时最容易把板子"烧哑"的两个坑

1. **勾了 Full chip erase / Erase all 之后，两个 hex 都要烧回去！**
   只烧 App 的话 `0x08000000` 是空的，CPU 从空白 Flash 启动 =
   **上电后串口一个字符都没有**（看起来像板子坏了，其实只是 BL 没了）。
2. **别用 `.bin` 乱填地址。** `.bin` 不带地址信息，起始地址填错（尤其是把 App
   填成 `0x08000000`）会把 Bootloader 覆盖掉。优先烧 `.hex`（自带地址）。

想确认烧对了没有，用 ST-Link 读一下 `0x08000000` 的前 8 字节：
应该是 `00 02 00 20`（SP = `0x20020000`）后面跟一个 `0x0800xxxx`
（PC，最低位必须是 1）—— 那就是 Bootloader 的向量表。
`0x08020000` 处同理，只是 PC 变成 `0x0802xxxx`。

#### 元数据（`0x080E0000`）自己坏掉了怎么办

不用慌，不会变砖：

* 新版 Bootloader 上电发现某个 flash word 坏了（读它 NMI/总线错误）会自动
  **把坏掉的扇区擦掉再复位**（`S0` 也就是 Bootloader 自己永不擦），然后正常启动；
* 也可以手动解决：ST-Link 把 `0x080E0000` 那个扇区擦掉（或者干脆全片擦除后
  把上面两个 hex 重烧一遍）。擦了元数据只是丢掉 A/B 记录，App 跑起来会重新登记；
* App 里如果登记失败，串口会打 `ota: boot confirm FAILED (st=..)`，不再静默。

### 日常升级

```bash
pip install pyserial
python tools/ota.py --port COM7 info                       # 看当前在哪个槽、版本、两个槽的 CRC
python tools/ota.py --port COM7 flash build/Debug/motor_control_slotB.bin
python tools/ota.py --port COM7 rollback                   # 新固件有问题 → 一键切回旧槽
python tools/ota.py --port COM7 reboot --boot              # 手动进 Bootloader 恢复台（救砖）
python tools/ota.py --port COM7 monitor                    # 当串口监视器看日志
python tools/ota.py --port COM7 status                     # 电机里程/位置/故障码
python tools/ota.py --port COM7 ctrl disable               # 电机失能（enable/posloop/movepos/... 同理）
```

`flash` 会自动挑槽（写在非活动槽）并**检查你给的 .bin 是给哪个槽编的**
（.bin 里的复位向量一看就知道），给错了直接拒练，避免把 A 的镜像写进 B 槽、跳过去必崩。

### 关于 CRC32：设备算的是「Flash 内容」，不是「.bin 文件」

设备侧的 CRC32（元数据登记、Bootloader 的槽校验、OTA 的 `END` 校验）**永远是把它槽里的
Flash 内容读回来算的**，而上位机算的是 `.bin` 文件本身。多数时候两者一样，
但有一种情况会不一样：

* 链接脚本会在两个 section 之间留下**对齐空洞**（本工程是 `0x080202CC` 那 4 字节）；
* `objcopy -O binary` 把空洞填成 `0x00`，而 ST-Link 烧 `.hex` 时那些位置是**跳过**的、
  保持擦除态 `0xFF`。

于是同一个镜像有两个 CRC32：走 ST-Link 烧进去的是 `0xFF` 版本（实测 App = `0x27F6680E`），
上位机对 `.bin` 算出的是 `0x00` 版本（当前构建 = `0xABAA221F`）。

**这不影响功能**：设备侧永远自洽 —— 元数据里登记的就是它自己算的那个值，
Bootloader 用完全相同的方式重算再比；走 OTA 时空洞会被写成 `0x00`，
所以设备读回算出的 CRC 又和上位机的 `.bin` CRC 一致（`flash` 最后一步就是这么比的）。
**但别拿 `info` 里的 CRC32 去和 `selftest` 报的 `.bin` CRC 硬比**（除非那次是走 OTA 写进去的）。

构建一次会出三个镜像：

| 产物 | 链接基址 | 用途 |
| --- | --- | --- |
| `motor_control.bin` | 0x08020000 (Slot A) | 主固件 |
| `motor_control_slotB.bin` | 0x08080000 (Slot B) | **同一份源码**、只换基址（A/B 双槽必需） |
| `motor_boot.bin` | 0x08000000 | Bootloader，只在第一次烧 |

### 这一个口上的三种东西怎么共存

| 内容 | 形式 |
| --- | --- |
| 调试日志 | **裸文本**（ASCII + `\r\n`），和以前打 UART7 一模一样（UART7 现在是镜像口） |
| 控制信息 | 二进制帧：`CTRL_CMD`（使能/失能/切位置环/走位置/急停/静音/停轮询）、`CTRL_STATUS`（里程/位置/故障码） |
| 固件 | 二进制帧：`OTA_BEGIN/DATA/END`（1 KB 一包，停等 + 重传 + 续传） |

帧的封装是 `0x00 + COBS(帧) + 0x00`：文本里不出现 `0x00`，COBS 编码后的帧里也不出现 `0x00`，
所以上位机看到两个 `0x00` 之间的一段就试着解帧、CRC32 不过就当文本丢掉 —— 三种内容不打架。

### 升级期间板子会做什么

`OTA_BEGIN` 一到：**电机全部失能** + **停 200 ms 状态轮询** + **静音日志**（别抢带宽），
然后擦除目标槽的 3 个扇区（一次做完，之后只剩 32 字节 flash word 的编程停顿），逐块收数据；
30 s 收不到任何帧就自动放弃并恢复日志/轮询（已写入的内容保留，下次续传）。
升级完成后需要**用户自己**再按键/发命令使能电机，不会自动转。

### ⚠ 别在 CubeMX 里勾 USART1

`Device/Ota/ota_com.c` **自己**初始化 USART1（RCC + GPIO + NVIC + `HAL_UART_Init`）：
Bootloader 目标里根本没有 `Core/Src/usart.c` / `stm32h7xx_it.c`（它们是生成文件、每次重写），
放自己文件里两个目标都能用、也不会被覆盖。
如果你更习惯 CubeMX：勾上 USART1 之后，把 `ota_com.c` 里的硬件初始化和 `USART1_IRQHandler`
删掉、改用 `MX_USART1_UART_Init()` + `huart1` —— **两套只能留一套**，否则重复定义。

## 构建

工具链不在默认 `PATH` 里，需要先指向 STM32Cube 的 bundle：

```bash
export PATH="/home/taishan/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin:/home/taishan/.local/share/stm32cube/bundles/ninja/1.13.2+st.1/bin:$PATH"

cube-cmake --preset Debug && ninja -C build/Debug      # Release 换成 --preset Release / build/Release
```

## CubeMX 重新生成代码后要检查什么（★ 容易忘）

1. `cmake/stm32cubemx/CMakeLists.txt` **每次** “Generate Code” 都会被重写；
   根 `CMakeLists.txt` 只在**第一次**生成，之后的用户改动不会被覆盖。
2. 自己新增的驱动**不要**直接写进生成文件，统一走「独立 cmake 片段」模式：
   - 配置放在 `cmake/bmi088.cmake`、`cmake/ws2812.cmake`、`cmake/motor.cmake`、
     `cmake/motor_ctrl.cmake`、`cmake/uart_log.cmake`、`cmake/ota.cmake`
     （CubeMX 不认这些文件，永远不会被覆盖）
   - 由根 `CMakeLists.txt` 里的几行 `include(cmake/xxx.cmake)` 拉进来
   - ⚠ **请求队列不在 CubeMX 里**：`Device/Motor/motor_io.c` 自己用 `xQueueCreate()` 建。
     以前在 CubeMX 里配的那个 `motorQueue`（item type `uint16_t`，2 字节）装不下一个请求，
     已经删掉了；别在 CubeMX 里再加回来。
   - ⚠ **无线 OTA 的三个镜像**（`motor_control` / `motor_control_slotB` / `motor_boot`）都在
     `cmake/ota.cmake` 里建，链接基址靠 `STM32H723xG_slots.ld` + `--defsym APP_BASE`，
     同样不要写进生成文件。
3. 所以 regenerate 之后如果发现某个驱动没编进固件，**先看那几行 `include()` 还在不在**，
   不要急着改 `cmake/stm32cubemx/CMakeLists.txt`（改了下次还会丢）。
4. `freertos.c` 里、`USER CODE` 段之外的**任务入口名**（`StartDefaultTask` / `MotorTask` /
   `motorPosCallback`）要和 `.ioc` 一致；实际的业务都在 `Device/Motor/motor_ctrl.c` /
   `motor_io.c` 里，`freertos.c` 只负责把它们接上，不要在生成区里写新逻辑。
5. 从别的工程复制过来之后，记得把根 `CMakeLists.txt` 里的
   `set(CMAKE_PROJECT_NAME ...)` 改成自己的工程名（因为它不会被重新生成，
   不改的话产物一直叫 `hello_imu.elf`；`cmake/stm32cubemx/CMakeLists.txt` 里
   全用 `${CMAKE_PROJECT_NAME}`，不用动）。

> 教训（2026-09-16）：BMI088 的源文件一开始是直接加在生成文件里的，一次 “Generate Code”
> 就被抹掉了，之后才改成现在的片段写法。

6. **`STM32H723xG_flash.ld` 已经被弃用**：CubeMX 每次还是会重新生成它，但根 `CMakeLists.txt`
   把工具链里那行写死的 `-T .../STM32H723xG_flash.ld`（和 `-Map` 名字）摘掉了，
   三个镜像统一用 **`STM32H723xG_slots.ld`** + `--defsym APP_BASE`。
   所以：不要去改那个生成出来的 `.ld`（改了也没用），内存布局有变化时同步
   `STM32H723xG_slots.ld` 里那几行 `MEMORY` 就行。
7. **不要**在 CubeMX 里勾 USART1（原因见「无线 OTA 升级」一节最后）。引脚清单里
   PA9/PA10 应该一直是“未分配”。
8. 升级/分区相关的动态都在 `USER CODE` 段里：`Core/Src/main.c` 只多一行 `SCB->VTOR = OTA_APP_BASE;`
   （**必须在 `MPU_Config()` 之前**），`Core/Src/freertos.c` 里只改了日志口和加上 OTA 服务初始化。

## 硬件要点（DM-MC-Board02）

| 功能 | 引脚 |
| --- | --- |
| 可控 5V 使能（`Power_5V_EN`，高有效） | **PC15** |
| 用户按键 `USER_KEY`（触发电机使能） | **PA15**（输入，无上下拉；按下为低电平。默认是 JTDI，配成 GPIO 后只能用 SWD 调试） |
| BMI088 加速度片选 / 陀螺片选 | PC0 / PC3 |
| SPI2：SCK / MOSI / MISO | PB13 / PC1 / PC2_C |
| SPI2 中断：ACC_INT / GYRO_INT | PE10 / PE12 |
| UART7（调试日志镜像，115200） | PE7 (RX) / PE8 (TX) |
| **USART1（无线串口：日志 + 控制 + OTA，115200）** | **PA9 = TX / PA10 = RX**（AF7，模块侧标 UART0） |
| USART10（电机，38400） | PE2 (RX) / PE3 (TX)，AF4 / AF11；**PE3 是开漏 AF_OD**（电机板 5V TTL，推挽会误发信号） |
| 板载 WS2812：DIN | **PA7 = SPI6_MOSI** |

- 5V 在 CubeMX 里上电默认是**关**的，`main()` 的 USER CODE 2 里才打开，等 100 ms
  让电源轨稳定后再用（板载 WS2812 和 BMI088 都吃这一路；BMI088 要求 VDD 有效后
  陀螺仪约 30 ms 才可访问）。
- WS2812 不走普通 GPIO，而是用 SPI6 的 MOSI 波形模拟单总线：**1 个 SPI 字节 = 1 个数据位**
  （`0` → `0x60`，`1` → `0x78`），一帧 24 字节（G-R-B），之后补 ≥50 µs 低电平锁存。
  SPI6 内核时钟取 HSE 24 MHz，预分频 4 → 6 MHz。
- 驱动接口分两层（`Device/WS2812/inc/ws2812.h`）：
  - 底层 `WS2812_Ctrl(r,g,b)`：直接写各通道 PWM 值（0-255）并锁存。
  - 上层：`WS2812_SetBrightness(b)` 只调亮度；颜色用枚举 `WS2812_SetColor(WS2812_COLOR_xxx)`
    （只选色，亮度沿用当前值）；唯一一个直接给通道值的接口是 `WS2812_SetColorRGB(r,g,b)`
    （0-255 只是混色比例）。枚举值：
    `OFF / RED / GREEN / BLUE / YELLOW / CYAN / MAGENTA / WHITE`。
    没有"同时设颜色 + 亮度"的接口，要两个一起改就先 `SetBrightness()` 再 `SetColor()`。
    内部保存"颜色比例 + 总亮度"，按 `通道输出 = 总亮度 x 通道权重 / 权重和`
    均摊后刷新（整数除法，向下取整）；总亮度含义是 **R+G+B 之和**，上电默认
    `WS2812_DEFAULT_BRIGHTNESS = 32`，颜色权重为 0 时输出全 0（灭）。
    读回用 `WS2812_GetBrightness()` / `WS2812_GetColorRGB()`（比例）/ `WS2812_GetOutput()`（实际 PWM 值）。

### ⚠️ SPI6 的 SCK 占用了 PA5（按键 / ADC1_CH18）

- SPI 主机（哪怕是“只发不收”）**必须输出 SCK**，所以 CubeMX 把 `PA5 → SPI6_SCK` 也配上了，
  SPI6_SCK 的另一个可选脚 PB3 已经是 `SPI1_SCK`。
- 板子上 **PA5 = `ADC1_CH18_KEY`**（按键 / ADC 采样脚）。
- 后果：只要 SPI6 还开着，PA5 就不能再当按键 / ADC18 输入用；而且每次刷新灯珠都会在这个脚上
  打出约 32 µs 的 6 MHz 时钟（按着键正好撞上这个窗口还可能干扰这一帧灯珠数据）。
- 以后要用 PA5 的按键/ADC 时，必须先把 WS2812 改成不占 SPI 外设的方案
  （例如 PA7 当普通 GPIO 位翻转时序），然后就可以在 CubeMX 里把 SPI6 关掉。

## 串口输出（无线口 USART1 = 主日志口，UART7 = 镜像）

- 日志现在从 **USART1（PA9/PA10，接无线模块）** 出去，同时镜像到 **UART7（PE7/PE8）**；
  两个口都是 **115200** 8N1（UART7 内核时钟 = D2PCLK1 = 120 MHz → USARTDIV 1041.625，误差 ≈ -0.004%）。
- 电机相关的日志都是纯文本、`\r\n` 结尾。上电先打：
  - `motor control (FreeRTOS): USART10 38400, press USER_KEY (PA15) to start`（等待按键）
  - 按键后：`key pressed, enabling motor (0xA0/0x08, retry until answered)...`
  - 成功：`id=1 enable OK (0xA1), mode=0x01 (current loop), rx=01 A1 01 00 00 00 00 00 00 E0`
    （`rx=` 是**收到的原始 10 字节**，全 0 = 一个字节都没收到）
  - 失败：`id=1 enable FAILED (no valid 0xA1), rx=...` 再打 `enable FAILED, press USER_KEY to retry`
- 使能 OK 后切位置环、并复查模式的日志（用来确认真的切过去了）：
  - `mode before switch (expect 0x01 current loop):` + `id=1 mode query (0x75): reply=OK, mode=0x01 (current loop), rx=...`
  - `id=1 set mode (0xA0/0x03 position loop): reply=OK, mode=0x?? (...), rx=...`
  - `mode after switch (expect 0x03 position loop):` + `id=1 mode query (0x75): reply=OK, mode=0x03 (position loop), rx=...`
  - 确认成功：`position loop confirmed (0x03), sweeping 0deg <-> 90deg ...`；
    没成功：`id=1 position loop NOT active (mode=0x01, current loop): ... - disabling instead of sweeping`
    + `motor disabled, press USER_KEY to enable again`
- 位置环没生效（`0x64` 被当成电流/转速）时会急停：
  - `id=1 RUNAWAY: mileage 9 -> 12 (3 turns) while moving <=270deg`
  - `id=1 stop (0x64 value=0): reply=OK, speed=..., current=..., rx=...`
  - `id=1 mode query (0x75): reply=OK, mode=0x01 (current loop), rx=...`（看它到底在哪个模式）
  - `motor stopped & disabled, press USER_KEY to retry`
- 走的过程中**再按一下键**（失能）：
  - `key pressed again, disabling motor (0xA0/0x09)...`
  - `id=1 disable (0xA0/0x09): reply=OK, mode=0x01 (current loop), rx=...`
    （⚠ 回帧的 `mode` 是**切换后的实际模式**，不一定等于 0x09，和使能一样只看「有没有合法 0xA1 回帧」）
  - `motor disabled, press USER_KEY to enable again`（同时灯灭）
- 使能全部 OK 后接着打：
  - 版本号：`id=1 version query (0xFD): reply=OK, rx=...`，成功再补一行
    `id=1 version: date=2023-09-05, model=0x3F, fw=0x0E, hw=0x37`
  - 然后进位置环开始动作：`position loop confirmed (0x03), start moving`；
    轮询任务同时每 200 ms 打一行。`MotorCtrl_PollOnce()` 的日志格式：
    `id=1 status (0x74): reply=OK, mileage=..., position=... (...deg), fault=..., rx=...`
- 实测正常一轮的输出（ID1，已使能；位置环的 `0x03` / `0x64` 那几行是**期待的格式**，
  接上电机跑一遍把 `rx=` 和 `speed/current/temp` 实际值填回去）：

```
motor control (FreeRTOS): USART10 38400, press USER_KEY (PA15) to start
key pressed, enabling motor (0xA0/0x08, retry until answered)...
id=1 enable OK (0xA1), mode=0x01 (current loop), rx=01 A1 01 00 00 00 00 00 00 E0
id=1 version query (0xFD): reply=OK, rx=01 FE 17 09 05 3F 0E 37 00 53
id=1 version: date=2023-09-05, model=0x3F, fw=0x0E, hw=0x37
mode before switch (expect 0x01 current loop):
id=1 mode query (0x75): reply=OK, mode=0x01 (current loop), rx=...
id=1 set mode (0xA0/0x03 position loop): reply=OK, mode=0x03 (position loop), rx=...
mode after switch (expect 0x03 position loop):
id=1 mode query (0x75): reply=OK, mode=0x03 (position loop), rx=...
position loop confirmed (0x03), start moving
id=1 single move 0 -> 0.0deg (position=0, 0x64): reply=OK, speed=0, current=-251, temp=27C, fault=0x00 (none), rx=...
single move 0 -> 0.0deg: position 61 (0.6deg) -> 61 (0.6deg), error=+61 (+0.7deg), mileage 0 -> 0 (0 turns)
id=1 single move 1/4 -> 90.0deg (position=8191, 0x64): reply=OK, speed=..., current=..., temp=..., fault=..., rx=...
single move 1/4 -> 90.0deg: position 61 (0.6deg) -> 8252 (90.6deg), error=+61 (+0.7deg), mileage 0 -> 0 (0 turns)
id=1 single move 2/4 -> 180.0deg (position=16383, 0x64): reply=OK, ...
single move 2/4 -> 180.0deg: position 8252 (90.6deg) -> 16444 (180.7deg), error=+61 (+0.7deg), mileage 0 -> 0 (0 turns)
...（3/4、4/4 同理；4/4 会跨过 0°/360° 那个点，position 会“跳”回 0.6°，里程 +1）
single-move test done: 4 x 90deg should be 1 turn -> mileage 0 -> 1 (1 turns)
press USER_KEY to disable, again to re-run
key pressed again, disabling motor (0xA0/0x09)...
id=1 disable (0xA0/0x09): reply=OK, mode=0x01 (current loop), rx=...
motor disabled, press USER_KEY to enable again
...
```

以上已验证的部分：使能回帧 `01 A1 01 00 00 00 00 00 00 E0`（模式 `0x01` = 电流环）；
版本 `0x17 09 05` = 2023-09-05，型号 `0x3F`、软/硬件版本 `0x0E / 0x37`；
位置 `31278 / 32767 × 360° ≈ 343.6°`。里程/位置会随电机转动变化。

> 日志里的 `rx=` 是**收到的原始 10 字节**，全 0 = 一个字节都没收到。
> 这个格式由 `motor_ctrl.c` 的 `MotorCtrl_HexToText()` 生成，和以前的 `%02X` 手写输出一致。
- 波特率存在 `motor_control.ioc` 里，regenerate 不会丢；要改波特率请在 CubeMX 里改，不要只改代码。
  可选的档位受网页限制：9600 / 57600 / 115200 / 230400 / 250000，改完记得网页上选同一个值。

下面这段是 **BMI088 的二进制包（当前用 `#if 0` 关着，先留着备查）**：

- 100 Hz 发一个 **20 字节二进制包**（对应网页 `Binary packet format`，同步头前面允许有
  ASCII 文本，网页会自己丢掉）：

| 字节 | 内容 | 类型 |
| --- | --- | --- |
| 0–1 | `0xAA 0xFF` 同步头 | — |
| 2–3 / 4–5 / 6–7 | 加速度 X / Y / Z | int16，小端，原始 LSB |
| 8–9 / 10–11 / 12–13 | 陀螺仪 X / Y / Z | int16，小端，原始 LSB |
| 14–19 | 磁力 X / Y / Z | int16，小端，本板没有磁力计 → 固定 0 |

发的是传感器**原始 LSB**（`BMI088_read_raw()`），不是 rad/s / g；网页自己按选定的量程换算。
温度不在协议里，固件仍在读但不往外发。

用串口助手之类的工具直接看会是一堆乱码，这是正常的（二进制协议）。
