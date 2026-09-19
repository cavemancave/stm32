# motor_control

STM32H723VGT6（达妙 DM-MC-Board02 / CtrBoard-H7）上的**串口电机控制测试**固件，
从 `hello_imu` 工程复制而来。

- **当前主流程**：通过 USART10 和电机一问一答，**按键（PA15 / USER_KEY）触发一次使能（0xA0/0x08）**，
  使能成功后先查一次**版本号（0xFD）**，随后**每隔 200 ms 循环查里程/位置/故障码（0x74）**，
  结果全部打印到 UART7（见「电机测试」一节），板载 WS2812 指示状态。
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

这一版先不跑 IMU，主流程是**按键触发的使能 + 状态轮询**（当前只挂 1 台电机，ID1）：
上电初始化完 → 按下 USER_KEY → 给 `motor_ids[]` 里的电机各发一次使能帧 → 把 RX 收到的 10 字节原样打印到 UART7；
使能全部 OK 后 → 查一次版本号 → 之后每 200 ms 查一轮里程/位置/故障码并打印（不再等按键）。

协议摘录见 [`docs/specification.md`](docs/specification.md)（源文件 `docs/M0603A_111_motor_specification_cn.pdf`）。

| 项目 | 值 |
| --- | --- |
| 串口 | **USART10**：PE2 = `USART10_RX`、PE3 = `USART10_TX`（AF4 / AF11，CubeMX 生成） |
| ⚠ TX 电平 | **PE3 必须配成开漏 `GPIO_MODE_AF_OD`**（CubeMX 里 PE3 的 Output type = Open Drain）。电机控制板是 **5V TTL** 单总线，高电平靠总线上拉；TX 用推挽时输出 MOS 管会常通，和 5V 上拉互相灌电流/把总线拉出错误电平 → **误发信号**。改代码没用，要在 CubeMX 里改（见 `usart.c` 的 `USART10_MspInit 1` 备注） |
| 参数 | **38400**, 8N1，一问一答，固定 **10 字节**帧 |
| 电机 ID | **1（只测 1 台）** —— ID 脚接地 = ID1、接高 = ID2，上电时锁存；要挂第二台就把 `main.c` 里 `MOTOR_COUNT` 改回 `2U` 并补全 `motor_ids[]` |
| 触发 | **USER_KEY = PA15**（`MX_GPIO_Init` 里配成输入、`GPIO_NOPULL`，按下为低电平）。轮询 + 20ms 消抖 + 等松手，按住不放只算一次 |
| 动作 | 按一次键 → 对 `motor_ids[]` 里的电机（当前只有 ID1）各发**一次** `0xA0/0x08`（**不重试**，内部收发各 100ms 超时）→ 打印 `rx=` 原始 10 字节 |
| 成功判据 | **收到合法的 `0xA1` 回帧**（ID/功能码/CRC 都对）即算使能成功。⚠ 反馈的 `DATA[2]` 是**使能之后的实际模式**，不是回显 `0x08`：实测 ID1 回 `01 A1 01 00 00 00 00 00 00 E0`，模式值 `0x01` = 默认电流环 |
| 后续①：版本号 | 使能全部 OK 后，各查一次 `0xFD`（等 `0xFE`），打印日期（年 = 20xx）/ 型号 / 软硬件版本；失败也把收到的 10 字节打出来 |
| 后续②：状态轮询 | 接着进 `while(1)`：每隔 **200 ms**（`main.c` 的 `MOTOR_STATUS_POLL_MS`）查一轮 `0x74`（等 `0x75`），打印**里程圈数、位置原始值（0~32767 ↔ 0~360°）、故障码（按位译成 hall/overcurrent/stall/overtemp/link-loss/voltage，未知位写 other）**；每轮换一个灯色当心跳 |
| 灯 | 每次点灯从枚举里**依次**取一个颜色（`RED → GREEN → BLUE → YELLOW → CYAN → MAGENTA → WHITE → RED`，跳过 `OFF`）：上电第一个状态 = 红，按键后换绿，使能全部 OK 再换蓝……失败则不换色，再按一次重来。亮度不在主流程设，用驱动默认 `WS2812_DEFAULT_BRIGHTNESS = 32`（含义是 R+G+B 之和；要调就改 `ws2812.h` 那个宏或开机调 `WS2812_SetBrightness()`），所以 红/绿/蓝 分别是 `(32,0,0)`、`(0,32,0)`、`(0,0,32)` |
| 日志 | 打在 **UART7**（115200）上，见下面「串口输出」一节 |

帧格式：`ID | 功能码 | DATA[2..8] | CRC8`

- 使能 `ID | 0xA0 | 0x08 0 0 0 0 0 0 | CRC8` → 反馈 `ID | 0xA1 | 模式值 0 0 0 0 0 0 | CRC8`
  （反馈的模式值是**切换后的实际模式**：发 0x08 使能后回的是 `0x01` 电流环，实测帧 `01 A1 01 00 00 00 00 00 00 E0`，CRC 已对过）
- 版本号查询 `ID | 0xFD | 0 0 0 0 0 0 0 | CRC8` → 反馈 `ID | 0xFE | 年 月 日 型号 软件版本 硬件版本 保留 | CRC8`
  （年字节是 20XX 的 XX，2021 年 = `0x15` = 十进制 21，所以日志里直接按十进制打）
- 状态查询 `ID | 0x74 | 0 0 0 0 0 0 0 | CRC8` → 反馈 `ID | 0x75 | 里程×4 位置H 位置L 故障码 | CRC8`

- 驱动在 `Device/Motor/`（`motor.c` / `inc/motor.h`），cmake 片段 `cmake/motor.cmake`，
  按仓库惯例由根 `CMakeLists.txt` 里的 `include()` 拉进来，regenerate 不会丢。
- 驱动里还实现了手册的其它命令，暂时没在主流程用：
  `Motor_QueryMode()`（0x75→0x76）、`Motor_QueryStatus()`（0x74→0x75，里程/位置/故障码）、
  `Motor_Disable()`（0x09）、`Motor_QueryVersion()`（0xFD→0xFE）。底层都走同一个 `Motor_Transaction()`。
- ⚠ **CRC**：手册「CRC8 值」一节写明是 **CRC-8/MAXIM**（多项式 x⁸+x⁵+x⁴+1，反射，初值 0x00），
  所以代码里用反射形式的多项式 `0x8C`（`MOTOR_CRC8_POLY`）。校验值自查：对 `"123456789"`
  应得 `0xA1`。ID1 使能帧 = `01 A0 08 00 00 00 00 00 00 6F`。
- ⚠ **按键电平**：`main.c` 里的 `USER_KEY_PRESSED_LEVEL` 默认 `GPIO_PIN_RESET`（按下接地）。
  如果实测反了（不按也一直触发），改成 `GPIO_PIN_SET`。
- ⚠ PA15 默认是 **JTDI**：配成 GPIO 后仍可用 SWD（PA13/PA14）调试，不要再接 JTAG。
- 排查：`rx=` **全 0** = 一个字节都没收到（没接线 / 没上电 / 波特率不对）；
  有数据但 `reply=NONE` = 帧对不上（CRC 或错位），把 raw 打出来比对。
- BMI088 的初始化、打包、发送三处都用 `#if 0` 关着（`Core/Src/main.c`），
  要放开时把这三处改回 `#if 1`，并同时把 WS2812 的绿灯状态逻辑接回去。

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
   - 配置放在 `cmake/bmi088.cmake`、`cmake/ws2812.cmake`、`cmake/motor.cmake`
     （CubeMX 不认这些文件，永远不会被覆盖）
   - 由根 `CMakeLists.txt` 里的三行 `include(cmake/xxx.cmake)` 拉进来（当前在 `CMakeLists.txt:70-72`）
3. 所以 regenerate 之后如果发现某个驱动没编进固件，**先看那几行 `include()` 还在不在**，
   不要急着改 `cmake/stm32cubemx/CMakeLists.txt`（改了下次还会丢）。
4. 从别的工程复制过来之后，记得把根 `CMakeLists.txt` 里的
   `set(CMAKE_PROJECT_NAME ...)` 改成自己的工程名（因为它不会被重新生成，
   不改的话产物一直叫 `hello_imu.elf`；`cmake/stm32cubemx/CMakeLists.txt` 里
   全用 `${CMAKE_PROJECT_NAME}`，不用动）。

> 教训（2026-09-16）：BMI088 的源文件一开始是直接加在生成文件里的，一次 “Generate Code”
> 就被抹掉了，之后才改成现在的片段写法。

## 硬件要点（DM-MC-Board02）

| 功能 | 引脚 |
| --- | --- |
| 可控 5V 使能（`Power_5V_EN`，高有效） | **PC15** |
| 用户按键 `USER_KEY`（触发电机使能） | **PA15**（输入，无上下拉；按下为低电平。默认是 JTDI，配成 GPIO 后只能用 SWD 调试） |
| BMI088 加速度片选 / 陀螺片选 | PC0 / PC3 |
| SPI2：SCK / MOSI / MISO | PB13 / PC1 / PC2_C |
| SPI2 中断：ACC_INT / GYRO_INT | PE10 / PE12 |
| UART7（调试日志，115200） | PE7 (RX) / PE8 (TX) |
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

## 串口输出（UART7 = 调试日志）

- UART7，**115200** 8N1（UART7 内核时钟 = D2PCLK1 = 120 MHz → USARTDIV 1041.625，误差 ≈ -0.004%）。
- 电机相关的日志都打在 UART7 上，纯文本、`\r\n` 结尾。每次按键先打：
  - `press USER_KEY (PA15) to send enable ...`（等待按键）
  - `key pressed (#N), sending enable (0xA0/0x08)...`
  - `id=1 enable sent, reply=OK/NONE, rx=xx xx xx ...`（**RX 收到的 10 字节原样**，全 0 = 一个字节都没收到）
  - 收到合法回帧时再补一行：`id=1 ack 0xA1 mode=0x01 (current loop)`
  - 全部 OK：`enable OK. LED -> next color (0,0,32), brightness=32.`
- 使能全部 OK 后接着打：
  - 版本号（各一次）：`id=1 version query (0xFD): reply=OK, rx=...`，成功再补一行
    `id=1 version: date=2021-11-28, model=0x3F, fw=0x01, hw=0x01`
  - 然后进入轮询：`enter status loop (0x74: mileage/position/fault) ...`，之后每 200 ms 一行
- 实测正常一轮的输出（ID1，已使能，默认电流环）：

```
press USER_KEY (PA15) to send enable ...
key pressed (#1), sending enable (0xA0/0x08)...
id=1 enable sent, reply=OK, rx=01 A1 01 00 00 00 00 00 00 E0
id=1 ack 0xA1 mode=0x01 (current loop)
enable OK. LED -> next color (0,0,32), brightness=32.
id=1 version query (0xFD): reply=OK, rx=01 FE 17 09 05 3F 0E 37 00 53
id=1 version: date=2023-09-05, model=0x3F, fw=0x0E, hw=0x37
enter status loop (0x74: mileage/position/fault) ...
id=1 status (0x74): reply=OK, mileage=0, position=31278, fault=0x00 (none), rx=01 75 00 00 00 00 7A 2E 00 DF
id=1 status (0x74): reply=OK, mileage=0, position=31278, fault=0x00 (none), rx=01 75 00 00 00 00 7A 2E 00 DF
...
```

以上是一条实测（ID1，未接负载）：版本 `0x17 09 05` = 2023-09-05，型号 `0x3F`、软/硬件版本 `0x0E / 0x37`；
位置 `31278 / 32767 × 360° ≈ 343.6°`，故障码 `0x00`。里程/位置会随电机转动变化。
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
