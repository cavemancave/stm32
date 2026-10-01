# 硬件要点 / 串口 / 电源监测（DM-MC-Board02）

> 返回 [README](../README.md)。电机协议见 [`motor_protocol.md`](motor_protocol.md)。

## 引脚表

| 功能 | 引脚 |
| --- | --- |
| 可控 5V 使能（`Power_5V_EN`，高有效） | **PC15** |
| **电机电源使能（`VCC_OUT1_EN`，高有效）** | **PC14** |
| 用户按键 `USER_KEY`（**只有 Bootloader 用**：上电时按住进恢复台） | **PA15**（输入，无上下拉；按下为低电平。默认是 JTDI，配成 GPIO 后只能用 SWD 调试） |
| BMI088 加速度片选 / 陀螺片选 | PC0 / PC3 |
| SPI2：SCK / MOSI / MISO | PB13 / PC1 / PC2_C |
| SPI2 中断：ACC_INT / GYRO_INT | PE10 / PE12 |
| UART7（调试日志镜像，115200） | PE7 (RX) / PE8 (TX) |
| **USART1（无线串口：日志 + 控制 + OTA，921600）** | **PA9 = TX / PA10 = RX**（AF7，模块侧标 UART0） |
| **电源电压采集（`ADC1_INP4`）** | **PC4**（VCC_IN → R86 1 MΩ → PC4 → R87 100 kΩ → GND，分压比 11:1；C53 10 nF 滤波） |
| USART10（电机，38400） | PE2 (RX) / PE3 (TX)，AF4 / AF11；**PE3 是开漏 AF_OD**（电机板 5V TTL，推挽会误发信号）。**两台电机挂同一条总线**，靠帧里的 ID 区分（ID1 / ID2） |
| 板载 WS2812：DIN | **PA7 = SPI6_MOSI** |

- 5V 在 CubeMX 里上电默认是**关**的，`main()` 的 USER CODE 2 里才打开，等 100 ms
  让电源轨稳定后再用（板载 WS2812 和 BMI088 都吃这一路；BMI088 要求 VDD 有效后
  陀螺仪约 30 ms 才可访问）。
- **电机电源是另一路**：接在一路可控电源输出上，由 **PC14** 控制，**高电平 = 使能**。
  同样是上电默认关断（`main()` 里 `MotorPwr_Init()` 就配成输出并拉低）。
  发 `motor.py enable` / `motor.py speedloop` / `motor.py drive` 会先上电并等 `MOTOR_PWR_SETTLE_MS`(500 ms) 再发使能帧；
  电源本身只有一条命令：`motor.py pwoff` / `pwron` / `pwcycle`（断电 / 上电 / 断电重启）；
  `disable` 只失能，**不动这一路电源**。
  ⚠ PC14/PC15 是 **OSC32_IN / OSC32_OUT**（这板子没焊 32.768 kHz 晶振，才能当 GPIO 用），
  属于备份域、驱动能力很弱（几 mA），**只能当使能信号**，电流得电源那边出，别直接带负载。
- ⚠ **调试只用 SWD，别再接 JTAG**：PA15 出厂是 **JTDI**，本工程把它配成 GPIO 输入
  (`USER_KEY`) 给 **Bootloader** 用（上电时按住 = 强制进恢复台，App 里没有任何功能）。
  一旦 PA15 不再做 JTDI，板上就只剩 **SWD**（PA13 = SWDIO / PA14 = SWCLK）能连调试器了。
  PA15 无内部上下拉（按下为低电平），靠板上的外部电路。

## 电源电压监测（PC4 = ADC1_INP4）

VCC_IN（电机那一路电源）经 **R86 1 MΩ / R87 100 kΩ** 分压后进 PC4，所以
**量程 = 3.3 V × 11 ≈ 36.3 V**，12 V 开关电源和 **6S 电池（满 25.2 V）** 都在里面。

* 代码：`Device/Power/power_mon.c`（`PowerMon_Init/Start`，1 Hz 后台采样 + 状态跳变才打日志）；
  ADC 本体（PLL2 = 48 MHz 时钟、16 位、387.5 周期采样）是 **CubeMX 生成**的 `Core/Src/adc.c`。
* 只看 `motor.py status`：回帧末尾就带着电压 / 单片 / 告警标志 / 剩余百分比
  （**电机没答上也照样打印**，因为这几项跟电机无关）。
* **单片电压的串数是自动判的**（按总压，不需要设）：两种电源量程不重合 ——
  - 6S 电池（单片窗口 3.30~4.25 V）⇒ 整组 19.8~25.5 V ⇒ 按 **6** 串算；
  - 12 V 开关电源（当 3S 算）⇒ 9.9~12.75 V ⇒ 按 **3** 串算。

  分界取 16 V（低于它 = 3 串，高于 = 6 串）。没有“不判单片”这种特例：有特例就得多一堆
  `cells==0` 分支，换算/告警/显示就不是同一条路了。
* **剩余电量百分比**按锂聚合物放电曲线从单片电压估（4.20 V/片 = 100%、3.30 V/片 = 0%，中间插值，
  不是库仑计）：带载会偏低、刚充完偏高，只当“大概还剩多少”看。12 V 那套按 3S 算时它同样只是个数字。
* 低于/高于单片窗口，或读数顶到量程上限（≥34 V 饱和）、采样失败，都会在串口打一条
  `[power] LOW/OVER-VOLT/RANGE …`（只在**状态跳变**时打，不会一秒一条刷屏）。
* ⚠ 精度取决于 VDDA/VREF+ 到底是不是 3.300 V（按它算的）；量程/换算对不上时先核对
  R86/R87 和 VREF（`Device/Power/inc/power_mon.h` 里改宏即可）。

## 两台电机（同一条总线，靠 ID 区分）

两台电机挂在**同一条 USART10 单总线**上（LIN 那种半双工），靠**帧里的 ID** 区分，最多两台：
ID 是**上电时由电机的 ID 脚锁存**的（低 = ID1、高 = ID2），所以接线时把第二台的 ID 脚接高即可；
两台**共用** PC14 那一路电机电源（VCC_OUT1_EN）。

- 固件里就是 `Device/Motor/motor_ctrl.c` 的 `motor_ids[]`（`{1U, 2U}`）和 `MOTOR_COUNT`：
  使能 / 失能 / 轮询都是按这个数组循环，两台都覆盖。
- 无线命令用 `--motor` 指定哪一台。**不填**时按安全默认：失能/急停作用于**全部**、
  使能/切位置环/走位作用于 **1 号机**：

  ```bash
  python tools/motor.py status                        # 两台都问（回帧末尾带电源状态 + 电压）
  python tools/motor.py enable  --motor 2             # 只使能 2 号机
  python tools/motor.py disable --motor 2             # 只失能 2 号机
  python tools/motor.py disable                       # 两台都失能（电源不动）
  python tools/motor.py pwoff                         # 断电：单独一条命令
  ```

  ⚠ `disable` 只是让它“不使劲”：通信/编码器/故障码都照旧能读，PC14 那一路不受影响
  （单台、两台都一样）。收工要断电就再发一条 `pwoff`；进 OTA 模式也只失能、不断电。

## 板载 WS2812

- WS2812 不走普通 GPIO，而是用 SPI6 的 MOSI 波形模拟单总线：**1 个 SPI 字节 = 1 个数据位**
  （`0` → `0x60`，`1` → `0x78`），一帧 24 字节（G-R-B），之后补 ≥50 µs 低电平锁存。
  SPI6 内核时钟取 HSE 24 MHz，预分频 4 → 6 MHz。
- 驱动接口分两层（`Device/WS2812/inc/ws2812.h`）：
  - 底层 `WS2812_Ctrl(r,g,b)`：直接写各通道 PWM 值（0-255）并锁存。
  - 上层：`WS2812_SetBrightness(b)` 只调亮度；颜色用枚举 `WS2812_SetColor(WS2812_COLOR_xxx)`
    （只选色，亮度沿用当前值）；唯一一个直接给通道值的接口是 `WS2812_SetColorRGB(r,g,b)`。
    枚举值：`OFF / RED / GREEN / BLUE / YELLOW / CYAN / MAGENTA / WHITE`。
    没有“同时设颜色 + 亮度”的接口，要两个一起改就先 `SetBrightness()` 再 `SetColor()`。
    内部保存“颜色比例 + 总亮度”，按 `通道输出 = 总亮度 x 通道权重 / 权重和` 均摊后刷新；
    总亮度含义是 **R+G+B 之和**，上电默认 `WS2812_DEFAULT_BRIGHTNESS = 32`。
    读回用 `WS2812_GetBrightness()` / `WS2812_GetColorRGB()` / `WS2812_GetOutput()`。
- ⚠ **App 里没有任务驱动灯珠**：上电 `main()` 只发一帧全灭。
- ⚠ **SPI6 的 SCK 占用了 PA5**：SPI 主机必须输出 SCK，所以 CubeMX 把 `PA5 → SPI6_SCK` 也配上了
  （另一个可选的 PB3 已是 `SPI1_SCK`）。板子上 **PA5 = `ADC1_CH18_KEY`**（按键 / ADC 采样脚）——
  只要 SPI6 还开着，PA5 就不能再当按键/ADC18 用。以后要用 PA5，得先把 WS2812 改成
  不占 SPI 外设的方案（例如 PA7 当普通 GPIO 位翻转），再在 CubeMX 里关掉 SPI6。

## 串口输出（无线口 USART1 = 主日志口，UART7 = 镜像）

- 日志从 **USART1（PA9/PA10，接无线模块）** 出去，同时镜像到 **UART7（PE7/PE8）**；
  **USART1 = 921600**、**UART7 = 115200**，都是 8N1（UART7 内核时钟 = D2PCLK1 = 120 MHz →
  USARTDIV 1041.625，误差 ≈ -0.004%；USART1 走 D2PCLK2，921600 下 USARTDIV = 130.2、误差 +0.02%）。
  ⚠ Bootloader 与 App 共用 `OTA_PORT_BAUD`（`ota_com.c` 就一份），但它**不参与 OTA**
  ⇒ 改了这个宏要用 SWD/ST-Link 重烧一次 `motor_boot.hex`。两者现在都是 921600。
  ⚠ **接回无线模块之前**：两侧模块的“串口波特率”都要设成 921600（用模块的配置工具），
  否则 App 会听不见。
- **日志风格约定（2026-09-19 起）**：
  - 设备侧日志**一律英文**（短、好 grep、任何串口助手都不会乱码），中文解释留在注释和 `docs/` 里；
  - **一行太长就拆成两行**（第二行按前缀缩进，例如 `[power]   ADC1_INP4/PC4 ...`）；
  - 开机第一条日志把 USART1 的波特率也打出来：`[ota_com] USART1 up @ 921600 8N1 (...)`。
- 电机相关的日志都是纯文本、`\r\n` 结尾。上电先打：
  - `motor control (FreeRTOS): USART10 38400 (2 motors), wireless USART1 921600`
  - `no key/bench flow: drive the car over the wireless link (tools/car.py)`
- 之后全部由无线命令驱动（**没有按键流程**），典型日志：
  - `motor.py speedloop [--motor N]`：`id=1 speed loop confirmed (0x02)`
  - `motor.py speed [--motor N]`：`id=1 speed +300.0rpm (raw=300, accel=10ms): reply=OK`
    + `id=1 speed ack: speed=300, current=..., temp=..C, fault=0x00, rx=...`
  - `motor.py disable`：`id=1 disable (0xA0/0x09): reply=OK, mode=0x01 (current loop), rx=...`
    （⚠ 回帧的 `mode` 是**切换后的实际模式**，不一定等于 0x09；只看「有没有合法 0xA1 回帧」）
  - `motor.py posloop [--motor N]`（位置环用）：
    `id=1 mode before switch (expect 0x01 current loop): 0x01 (current loop)`
    → `id=1 set mode (0xA0/0x03 position loop): reply=OK, mode=0x03 (position loop), rx=...`
    → `id=1 mode query (0x75): reply=OK, mode=0x03 (position loop), rx=...`
    → `position loop confirmed (0x03)`；没切上则 `id=1 position loop NOT active (mode=0x.., ...) - refusing`
  - 200 ms 状态轮询：`id=1 status (0x74): reply=OK, mileage=..., position=... (...deg), fault=..., rx=...`
- 小的 `[drive]` 1 Hz 行（小车闭环）见 [`car.md`](car.md)。排查：`rx=` **全 0** = 一个字节都没收到；
  有数据但 `reply=NONE` = 帧对不上（CRC 或错位），把 raw 打出来比对。
- 波特率存在 `motor_control.ioc` 里，regenerate 不会丢；要改波特率请在 CubeMX 里改，不要只改代码。

## BMI088 二进制包（当前 `#if 0` 关着，备查）

- 100 Hz 发一个 **20 字节二进制包**（对应网页 `Binary packet format`，同步头前面允许有
  ASCII 文本，网页会自己丢掉）：

| 字节 | 内容 | 类型 |
| --- | --- | --- |
| 0–1 | `0xAA 0xFF` 同步头 | — |
| 2–3 / 4–5 / 6–7 | 加速度 X / Y / Z | int16，小端，原始 LSB |
| 8–9 / 10–11 / 12–13 | 陀螺仪 X / Y / Z | int16，小端，原始 LSB |
| 14–19 | 磁力 X / Y / Z | int16，小端，本板没有磁力计 → 固定 0 |

发的是传感器**原始 LSB**（`BMI088_read_raw()`），不是 rad/s / g；网页自己按选定的量程换算。
温度不在协议里，固件仍在读但不往外发。用串口助手直接看会是一堆乱码，这是正常的（二进制协议）。
