# uart_rx_dma — USART1 RX/TX DMA + 环形缓冲 + 帧状态机 + 消息分发

STM32H723VGT6（DM-MC-Board02）上的串口收发示例工程：

```
USART1 RX (PA10)
   │  DMA1_Stream1，循环模式，1024 B
   ▼
DMA 缓冲（RAM_D2，DMA 可访问）
   │  IDLE / HT / TC 中断
   ▼
软件环形缓冲 8192 B（ISR 生产、主循环消费）
   │
   ▼
帧状态机（帧头/类型/长度/payload/CRC32/帧尾/超时）
   │
   ▼
消息分发（3 个计算消息 + PING + STATS）
   │
   ├──► 响应 / NACK ──► TX FIFO 8192 B ──► DMA1_Stream0 ──► USART1 TX (PA9)
   └──► 统计 / 心跳
```

最初需求写的是 UART7；按实际接线已改为 **USART1（PA9/PA10）**。
协议层和 App 层与具体串口无关，换回 UART7 只需改 `Core/Src/usart.c`、`.ioc`
和 `App/Src/app.c` 里的 `huart1`。

---

## 0. 当前实测状态

| 项目 | 值 |
| --- | --- |
| 板子 | STM32H723VGT6 / DM-MC-Board02 |
| 串口 | **USART1：PA9 = TX，PA10 = RX** |
| 波特率 | **115200 8N1**（CubeMX 里配置） |
| USB-TTL | CH343，Windows 上枚举为 **COM12** |
| 调试器 | ST-Link V2，SWD（PA13/PA14） |
| 固件 | `build/Debug/motor_control.elf` |
| Python 测试 | **67/67 检查通过** |
| 1000 帧压力测试 | **697.2 帧/s，TX 84.7%，RX 96.8%，RTT 2.61 ms，固件处理 ≈2 µs，0 丢帧/0 溢出/0 TX 丢包** |

> RTT 是电脑侧测的，包含 CH343/USB 驱动和 Python 线程调度；固件内部
> 处理时间通常约 **2 µs**（DWT 周期计数器实测，偶发到十几 µs 是测量窗口里
> 进了 UART/DMA 中断）。

---

## 1. 硬件与接线

### 1.1 USART1

| MCU 引脚 | 方向 | 接到 USB-TTL |
| --- | --- | --- |
| **PA9** | USART1_TX | **CH343 的 RX** |
| **PA10** | USART1_RX | **CH343 的 TX** |
| GND | 地 | GND（必须共地） |

- 8N1、无硬件流控、3.3 V TTL。USB-TTL 的电平选择跳线请放 **3.3 V**。
- **TX/RX 必须交叉**：MCU 的 TX 接 USB-TTL 的 RX，MCU 的 RX 接 USB-TTL 的 TX。
  如果接成 TX-TX / RX-RX，表现是“电脑能收到板子的心跳，但板子收不到电脑的 PING”。
- 如果收不到任何心跳，按顺序检查：波特率 → PA9/PA10 是否接反 → GND → 固件是否在跑。
- `tools/probe.py` 可以快速验证链路（发 PING、打印原始字节）。

### 1.2 调试口

只用 SWD：PA13 = SWDIO，PA14 = SWCLK。PA15 在本工程里仍按模板配成
`USER_KEY` 输入，没有业务功能。

---

## 2. 快速开始

### 2.1 编译

VS Code + STM32Cube for VS Code 插件里直接选 `Debug` preset 即可。命令行：

```powershell
$env:Path = "C:\Users\taishan\AppData\Local\stm32cube\bundles\gnu-tools-for-stm32\14.3.1+st.2\bin;$env:Path"
$ninja = "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"

cmake -S . -B build/Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug `
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake `
  -DCMAKE_MAKE_PROGRAM="$ninja"
cmake --build build/Debug
```

### 2.2 烧录

```powershell
& "C:\ProgramFiles\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" `
  -c port=SWD freq=4000 -w build\Debug\motor_control.elf -v -rst
```

### 2.3 跑测试

```powershell
pip install pyserial
python tools/test_uart.py --list
python tools/test_uart.py --port COM12 --baud 115200
python tools/test_uart.py --port COM12 --baud 115200 --perf-count 1000
python tools/test_uart.py --port COM12 --baud 115200 --tests perf
```

报告写到 `tools/test_report.json`（已加入 `.gitignore`）。

---

## 3. 帧格式

采用通用“帧头 + 类型 + 长度 + 数据 + CRC32 + 帧尾”：

| 偏移 | 字段 | 长度 | 说明 |
| --- | --- | --- | --- |
| 0 | `SOF` | 1 | 固定 `0xA5` |
| 1 | `TYPE` | 1 | 消息类型 |
| 2 | `LEN` | 1 | payload 长度，**0..8** |
| 3 | `PAYLOAD` | `LEN` | 数据，小端 |
| 3+LEN | `CRC32` | 4 | 小端，覆盖 `TYPE + LEN + PAYLOAD` |
| 7+LEN | `EOF` | 1 | 固定 `0x5A` |

- 整帧长度 = `8 + LEN`，最大 **16 字节**。
- CRC 是 **CRC-32/ISO-HDLC**，和 Python `zlib.crc32` 完全一致：
  `poly=0x04C11DB7`（反射形式 `0xEDB88320`）、`init=0xFFFFFFFF`、
  `refin/refout=true`、`xorout=0xFFFFFFFF`。
- CRC 不覆盖 `SOF` 和 `EOF`。
- 示例：

```
PING 请求（LEN=0）：
A5 10 00 AE 00 1B 0B 5A

计算算数和 a=1,b=2,c=3（LEN=6）：
A5 01 06 01 00 02 00 03 00 20 A2 20 80 5A
```

---

## 4. 消息定义

### 4.1 请求 / 响应

| 类型 | 值 | 方向 | payload |
| --- | --- | --- | --- |
| `CALC_SUM_REQ` | `0x01` | 电脑→板 | 6 B：`a int16, b int16, c int16` |
| `CALC_SQSUM_REQ` | `0x02` | 电脑→板 | 同上 |
| `CALC_WEIGHTED_REQ` | `0x03` | 电脑→板 | 同上 |
| `PING_REQ` | `0x10` | 电脑→板 | 无 |
| `STATS_REQ` | `0x11` | 电脑→板 | 1 B：page（0..5） |
| `CALC_SUM_RSP` | `0x81` | 板→电脑 | 8 B：`result int32, proc_us uint16, status uint8, reserved uint8` |
| `CALC_SQSUM_RSP` | `0x82` | 板→电脑 | 同上 |
| `CALC_WEIGHTED_RSP` | `0x83` | 板→电脑 | 同上 |
| `PONG_RSP` | `0x90` | 板→电脑 | 8 B：`uptime_ms uint32, proc_us uint16, valid_frames uint16` |
| `STATS_RSP` | `0x91` | 板→电脑 | 8 B：`page uint8, flags uint8, v0 uint16, v1 uint16, v2 uint16` |
| `HEARTBEAT` | `0x7D` | 板→电脑（1 Hz） | 8 B：`uptime_ms uint32, valid_frames uint16, uart_errors uint16` |
| `READY` | `0x7E` | 板→电脑（上电一次） | 8 B：`version uint16, max_payload uint8, dma_buf uint16, ring_size uint16, timeout_ms uint8` |
| `NACK` | `0xEE` | 板→电脑 | 8 B：`err_code, type, len, state, raw[4]` |

### 4.2 三个计算消息

请求 payload 固定 6 字节（`int16` 小端）：

- `CALC_SUM_REQ`：`result = a + b + c`
- `CALC_SQSUM_REQ`：`result = a² + b² + c²`
- `CALC_WEIGHTED_REQ`：`result = a*1 + b*2 + c*3`

响应里的 `result` 是 `int32`。平方和可能超过 `INT32_MAX`，这时结果饱和到
`INT32_MAX`/`INT32_MIN`，`status=1` 表示发生过饱和；正常 `status=0`。
`proc_us` 是固件分发处理时间（不含串口发送）。

### 4.3 NACK 错误码

| err_code | 名称 | 含义 |
| --- | --- | --- |
| 1 | `BAD_LEN` | LEN > 8 |
| 2 | `CRC` | CRC32 不匹配 |
| 3 | `TAIL` | 帧尾不是 `0x5A` |
| 4 | `TIMEOUT` | 帧内字节间超时（默认 50 ms） |
| 5 | `UNKNOWN_TYPE` | 未知消息类型 |
| 6 | `PAYLOAD_LEN` | payload 长度不符合该消息 |
| 7 | `RING_OVERFLOW` | 软件环形缓冲溢出 |
| 8 | `UART` | UART 硬件错误（ORE/FE/NE/PE） |
| 9 | `STATS_PAGE` | 未知统计页 |

NACK 的 `type/len` 是出错帧的类型/长度，`state` 是解析器出错时所在状态，
`raw[4]` 是出错帧的前 4 个字节（`SOF/TYPE/LEN/...`）。

### 4.4 统计页

| page | v0 | v1 | v2 |
| --- | --- | --- | --- |
| 0 | `valid_frames` | `calc_frames` | `ping_frames` |
| 1 | `crc_err` | `len_err` | `tail_err` |
| 2 | `timeout_err` | `ring_drop_bytes` | `uart_err` |
| 3 | `proc_avg_us` | `proc_min_us` | `proc_max_us` |
| 4 | `tx_frames` | `unknown_type_err` | `payload_len_err` |
| 5 | `tx_drop_bytes` | `tx_err` | `tx_pending` |

计数在固件里是 `uint32_t`，上报时饱和到 `uint16_t`，`flags bit0=1` 表示至少
一个值饱和。

---

## 5. 软件结构

```
App/
  Inc/
    app.h / app_config.h      应用入口、可调参数（波特率/缓冲/超时）
    bytes.h                   小端读写、u16 饱和
    crc32.h                   CRC-32/ISO-HDLC
    frame_parser.h            帧结构、错误码、解析器状态
    messages.h                消息类型编号
    protocol.h                编解码/分发接口
    ringbuf.h                 SPSC 环形缓冲
    stats.h                   统计计数
    timebase.h                DWT 周期计数器
    uart_dma_rx.h             DMA RX 接口
    uart_tx.h                 TX DMA 发送队列接口
  Src/
    app.c                     初始化、主循环、回包、错误上报
    crc32.c                   CRC 实现
    frame_parser.c            状态机
    protocol.c                帧编码、消息分发表、响应/NACK
    ringbuf.c                 SPSC 环形缓冲
    stats.c                   统计页构造
    timebase.c                DWT 初始化/换算
    uart_dma_rx.c             DMA + IDLE/HT/TC -> ring
    uart_tx.c                 TX FIFO + DMA 暂存/启动
Core/
  Src/usart.c                 USART1 + DMA1_Stream1(RX) + DMA1_Stream0(TX)
  Src/dma.c                   DMA1 时钟/中断
  Src/stm32h7xx_it.c          USART1 / DMA1_Stream0 / DMA1_Stream1 IRQ
STM32H723xG_flash.ld          .dma_buffer 段放在 RAM_D2
tools/
  proto.py                    协议层/串口读线程/解码
  test_uart.py                测试套件
  probe.py                    快速 PING/心跳探测
```

### 5.1 数据流

1. `HAL_UARTEx_ReceiveToIdle_DMA()` 启动 USART1 RX DMA，循环模式，1024 B。
2. 一帧结束、线路空闲 → **IDLE 中断**；连续流 → **HT/TC 中断**。
3. `HAL_UARTEx_RxEventCallback(huart, Size)` 把 DMA 缓冲里
   `[last_pos, Size)` 的新字节搬进 **8192 B ring**。
4. 主循环 `app_poll()` 每次从 ring 取最多 64 B，逐字节喂给帧状态机。
5. 状态机通过 `on_frame` 回调把完整帧交给 `protocol_handle()`。
6. `protocol_handle()` 查表分发，构造响应或 NACK；`app.c` 调
   `uart_tx_write()` 把整帧写进 **8192 B TX FIFO**。
7. `uart_tx_poll()` 在 DMA 空闲时把 FIFO 数据搬到 512 B 暂存区
   （RAM_D2），启动 `HAL_UART_Transmit_DMA()`；完成中断只清 busy。
8. 每 1 s 发一条 `HEARTBEAT`；错误/溢出还会主动发 NACK。

### 5.2 为什么用 IDLE，而不是只用半满/全满中断

如果只靠 DMA 的 HT/TC 中断搬数据，DMA 要攒满 **512 B** 才会通知 CPU。
请求-应答式的小帧只有 8–16 B，会一直躺在 DMA 缓冲里，表现为“心跳能发，
但 PING 完全收不到”。本工程用 `ReceiveToIdle_DMA`：

- 小帧：一帧结束就 IDLE，立刻搬走；
- 粘包/高速流：HT/TC 每 512 B 兜底；
- 两条路径都进 `HAL_UARTEx_RxEventCallback`，用 `Size`（DMA 缓冲内位置）
  计算增量并处理回绕。

### 5.3 DMA 缓冲为什么放 RAM_D2

STM32H7 的 DMA1/DMA2 **不能访问 DTCM（0x2000_0000）**，而本工程 `.bss`
默认链接在 DTCM。RX 和 TX 的 DMA 缓冲都放 `.dma_buffer`：

```c
static uint8_t s_dma_buf[APP_RX_DMA_BUF_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));
static uint8_t s_tx_dma_buf[APP_TX_DMA_BUF_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));
```

链接脚本里增加：

```ld
.dma_buffer (NOLOAD) :
{
  . = ALIGN(32);
  *(.dma_buffer)
  *(.dma_buffer*)
  . = ALIGN(32);
} >RAM_D2
```

DMA 缓冲在 RAM_D2（0x3000_0000），ring/FIFO 仍在 DTCM（CPU 访问更快）。
RAM_D2 现在用 1536 B：RX 1024 B + TX 512 B。

### 5.4 CRC32 的坑

最初用了左移/MSB-first 的 CRC 实现，算出来是 CRC-32/MPEG-2 风格的值，
和 Python `zlib.crc32` 对不上，导致所有 PING 都被固件判成 CRC 错。
现在 `crc32.c` 用反射/右移算法（`0xEDB88320`），和 zlib 一致；
`tools/proto.py` 也直接用 `zlib.crc32`，两边可互相校验。

### 5.5 处理时间

`timebase.c` 打开 DWT CYCCNT，`protocol_handle()` 在分发前后取周期差，
按 480 MHz 换算成 µs，填进计算/PONG 响应的 `proc_us`，同时累计到 stats
page 3 的 avg/min/max。处理时间只覆盖“查表 + 计算 + 构造响应”，
不包含 UART 发送（TX 走 DMA）。

---

## 6. 溢出与边界分析

### 6.1 DMA 缓冲会不会被覆盖？

DMA 缓冲 **1024 B**，循环模式。半满 HT 每 **512 B** 触发一次：

| 波特率 | 字节率（8N1） | 512 B 时间 | 1024 B 时间 |
| --- | --- | --- | --- |
| 921600 | 92160 B/s | **5.56 ms** | 11.1 ms |
| 115200 | 11520 B/s | **44.4 ms** | 88.9 ms |

只要 CPU 在 5.56 ms（921600）内能进一次 HT/TC/IDLE 中断，DMA 就不会
覆盖未读数据。发送走 **TX DMA**，主循环不再等一条 16 B 响应发完；
最长的处理是解析 64 B 数据 + 协议计算，微秒级，而且这些操作不关中断。

TX DMA 的 512 B 暂存区也放 RAM_D2；`uart_tx_poll()` 只在 `busy=0` 时
填下一块，完成中断清 `busy`，所以不会在 DMA 还在读的时候覆盖暂存区。

> 如果 TX 引脚被外部拉死，TX DMA 可能一直不完成；`busy` 会保持，但
> RX DMA 中断不受影响。`uart_tx_on_error()` 会在 TX 已经离开 BUSY_TX
> 状态时清 `busy`，避免死锁。

**局限**：本设计没有 DMA 回绕计数器，如果中断被关超过 5.56 ms，
DMA 会静默覆盖，固件无法知道具体丢了多少。当前实测 0 次；如果以后加入
Flash 擦写、长临界区等 >5 ms 的阻塞，需要加大 DMA 缓冲、提高 DMA 中断
优先级，或增加 TC 回绕计数做溢出检测。

### 6.2 软件 ring 会不会溢出？

ring **8192 B**，`ringbuf_write()` 是“满时丢弃新数据”，绝不覆盖未读数据：

- 丢弃字节累计在 `dropped`；
- `app_report_ring_overflow()` 最多 10 Hz 发一条 `NACK_RING_OVERFLOW`；
- stats page 2 的 `ring_drop_bytes` 可查累计值；
- 丢字节后残帧由 50 ms 超时或 CRC 错误回收，状态机自动重新同步。

实测边界（921600 下做过的 4 KB / 8 KB 对比）：

- ring **4096 B** 时，1000 帧一次性突发会丢 **48 B**，并产生 1 次
  `TIMEOUT`（残帧）；1000 帧里收到 996 个响应。
- 改成 **8192 B** 后，同样 1000 帧：0 丢字节、0 超时、1000/1000 响应。
- 当前 **115200 + 8192 B** 的 1000 帧实测也是 **0 丢字节、0 超时**。

所以最终选 8192 B。

### 6.3 持续满速发送的理论上限

计算请求 14 B，响应 16 B。固件是“收一帧→TX FIFO→DMA 发一帧”，
串口线速率仍是瓶颈，发送字节比接收长：

| 项目 | 921600 | 115200 |
| --- | --- | --- |
| 请求帧时间（14 B） | 0.152 ms → 6583 帧/s | 1.215 ms → 823 帧/s |
| 响应帧时间（16 B） | 0.174 ms → 5760 帧/s | 1.389 ms → 720 帧/s |
| 净积压 | ≈ 823 帧/s × 14 B ≈ 11.5 KB/s | ≈ 103 帧/s × 14 B ≈ 1.44 KB/s |
| 8192 B ring 可吸收 | ≈ 712 ms | ≈ 5.7 s |

结论：

- **短突发**（几百到几千帧）没问题，1000 帧实测 0 溢出；
- 如果电脑**永不节流、持续满速**发请求，响应比请求长，ring 最终会满；
  这是“请求-响应 + 单串口全双工”的协议级上限，不是 DMA 配错。
- 要支持无限满速流，需要应用层流控（窗口/ACK）或降低响应长度；
  TX DMA + FIFO 已经做了，但它只是把 CPU 从发送里解放出来，不会改变线上字节数。
  当前测试脚本是发一批、收一批，不会触发这个上限。

### 6.4 波特率降低，缓冲更容易还是更不容易溢出？

分两个缓冲看，结论不一样：

**DMA 缓冲：波特率越低越安全。** 半满 512 B 的时间与波特率成反比：

| 波特率 | 512 B 时间 | 1024 B 时间 |
| --- | --- | --- |
| 921600 | 5.56 ms | 11.1 ms |
| 115200 | 44.4 ms | 88.9 ms |

115200 下 CPU 有 8 倍时间进 HT/TC/IDLE 中断，DMA 缓冲被覆盖的风险明显更低。

**软件 ring：不能简单说“更不容易”，要看比较的是“同样帧数”还是“同样时间”。**

- 请求 14 B、响应 16 B，固件“收一帧→TX FIFO→DMA 发一帧”，所以净积压是
  **每帧 2 B**，这个比例和波特率无关。对于**同样 1000 帧**的满速突发：
   - 921600：约 2 KB 积压，满速约 0.2 s 跑完；
   - 115200：仍然是约 2 KB 积压，但要约 1.4 s 才跑完。
  → ring 的**字节高水位差不多**，只是时间拉长 8 倍。低波特率**不会**把
  “1000 帧需要 2 KB 缓冲”这件事变成“只需要 256 B”。
- 如果是**固定墙钟时间**的突发（比如“连续猛发 100 ms”）：
  115200 只发出 1/8 的字节，积压也只有 1/8，**更不容易溢出**。
- 如果是**固定字节数的突发**（比如 1000 帧、5000 帧）：
  积压字节数相同；低波特率给 CPU/中断的时间余量更大，但 ring 的**容量需求**
  不变。8 KB ring 在 115200 下能吸收约 4096 帧的净积压，在 921600 下也是
  约 4096 帧；只是 115200 要 5 s 才到，921600 约 0.6 s。
- 如果是**持续满速、永不节流**：两种波特率最终都会溢出，因为响应比请求长；
  低波特率只是把“溢出时间”推迟 8 倍，不会改变协议级上限。

一句话：**低波特率让“时间余量”变大、DMA/中断更安全，但不会改变“每帧
2 B 净积压”的比例；固定帧数突发需要的 ring 字节数基本不变。**

### 6.5 解析器 / payload 缓冲

- `frame_t.raw[16]`、`payload[8]`；
- `LEN > 8` 在 `FP_LEN` 状态立即报 `BAD_LEN` 并复位，**不会写 payload**；
- 最大帧正好 16 B，所有下标有界；
- CRC 计算长度 `2 + LEN ≤ 10`，有界；
- 50 ms 帧内超时防止残帧永久占住状态机；
- 坏 CRC/坏帧尾/坏长度都会复位并重新找 `SOF`。

### 6.6 UART 硬件溢出

DMA 一直挂着接收请求，正常不会 ORE。若发生 ORE/FE/NE/PE：

- `HAL_UART_ErrorCallback()` 计数并记录 `ErrorCode`；
- 若 HAL 已把 RX 状态置回 READY，则重新启动
  `HAL_UARTEx_ReceiveToIdle_DMA()`；
- 通过 `NACK_UART` 和 stats page 2 的 `uart_err` 上报。

实测 1000 帧压力测试 `uart_err = 0`。

### 6.7 TX FIFO / TX DMA

响应发送现在是 **TX FIFO + DMA**，不再阻塞主循环：

- `uart_tx_write()`：整帧原子写入 **8192 B TX FIFO**（DTCM）。
  空间不够就整帧丢弃并累计 `tx_drop_bytes`，不会写半帧把流打乱。
- `uart_tx_poll()`：DMA 空闲时从 FIFO 取最多 **512 B** 到
  `s_tx_dma_buf`（RAM_D2），调用 `HAL_UART_Transmit_DMA()`。
- `HAL_UART_TxCpltCallback()` 在 DMA 完成中断里只清 `busy`；主循环下次
  `uart_tx_poll()` 再取下一块。`app_poll()` 每圈都会推动一次。
- 如果 TX 出错，`HAL_UART_ErrorCallback()` 会调 `uart_tx_on_error()`；
  只有 `gState != BUSY_TX` 时才清 `busy`，避免 DMA 还在读暂存区时被覆盖。
- stats page 5 可查 `tx_drop_bytes / tx_err / tx_pending`。

实测 1000 帧：`tx_drop_bytes = 0, tx_err = 0, tx_pending = 0`，没有 TX 丢包。
持续满速时 TX FIFO 和 RX ring 面对同一个“每帧 2 B 净积压”的上限：
8192 B 约能吸收 4096 帧，低波特率只是把时间拉长 8 倍，理由同 6.4。

---

## 7. Python 测试覆盖

`tools/test_uart.py` 的 67 项检查：

| # | 场景 | 内容 |
| --- | --- | --- |
| 1 | 正常 PING | 5 次 PING/PONG，检查 uptime、proc_us、RTT |
| 2 | 三个计算消息 | 8 组输入（正/负/零/大数饱和），校验结果和 status |
| 3 | 粘包 | 4 帧一次写、前导垃圾 + 3 帧、单帧拆两次写 |
| 4 | 帧中断 | 半帧后停 100 ms，收 `NACK_TIMEOUT`，再发 PING 恢复；重复两次 |
| 5 | CRC 失败 | 改 CRC 字节，收 `NACK_CRC`；坏帧+好帧粘包，先 NACK 后正常 |
| 6 | 非法帧 | LEN=9、错误帧尾、未知类型、LEN=8 最大 16 字节帧、calc payload 长度不符，均能恢复 |
| 7 | 统计 | 查询 page 0..5，检查页号和 `valid_frames` 递增 |
| 8 | 性能 | 30 次顺序 RTT + 1000 帧连续突发，校验结果/统计增量/无丢帧/无 TX 丢包 |

实测输出（节选）：

```
通过 67/67，失败 0
性能: 697.2 帧/s, RTT avg 2.61 ms, 固件处理 avg 2.0 us

帧数                1000
总耗时              1434.2 ms
吞吐                697.2 帧/s
TX 字节 / 占用      14000 B / 84.7%
RX 字节 / 占用      16000 B / 96.8%
RTT min/avg/max     2.52 / 2.61 / 2.73 ms
固件处理时间        2 / 2.0 / 4 us
```

> `总耗时` 是从第一帧发出到最后一个响应到达；包含电脑 USB-TTL 驱动和
> Python 读线程的开销。固件内部处理时间看 `proc_us`（2 µs 量级）。

---

## 8. CubeMX 重新生成注意事项

`.ioc` 当前配置：

- USART1：Asynchronous，**PA9 = TX / PA10 = RX**，**115200** 8N1；
- USART1_RX：**DMA1_Stream1**，Circular，Byte，MemInc；
- USART1_TX：**DMA1_Stream0**，Normal，Byte，MemInc；
- NVIC：USART1 全局中断、DMA1_Stream0/1 中断使能；
- `main.c` 的 `USER CODE BEGIN 2` 调用 `app_init()`，
  `USER CODE BEGIN 3` 调用 `app_poll()`。

重新生成时注意：

1. **`App/` 目录不会被 CubeMX 覆盖**；顶层 `CMakeLists.txt` 是用户维护的，
   CubeMX 不会改它，里面已经加了 `App/Src/*.c` 和 `App/Inc`。
2. `usart.c` 由 CubeMX 生成，重新生成后会覆盖。只要 CubeMX 里 USART1 +
   DMA1_Stream1(RX，Circular) + DMA1_Stream0(TX，Normal) 配好，生成的
   `huart1`/`hdma_usart1_rx`/`hdma_usart1_tx` 就能直接
   被 `App/` 使用。
3. `STM32H723xG_flash.ld` 是 CubeMX 生成的，**重新生成可能删掉
   `.dma_buffer` 段**；如果 DMA 完全收不到/发不出数据，先检查这个段还在不在，
   不在就把 5.3 节的链接片段加回去。
4. 改波特率：在 CubeMX 里改 USART1 的 Baud Rate 并重新生成，`usart.c`
   里是生成的 `huart1.Init.BaudRate = 115200;`。`app_config.h` 里的
   `APP_UART_BAUD` 只作工程记录，不参与生成代码的波特率。
5. 如果只改 `.ioc` 不改代码，重新生成后也建议跑一遍
   `python tools/test_uart.py`，确认 `huart1`/DMA 流号没变。

---

## 9. 已知限制

- 响应发送走 TX DMA + 8192 B FIFO；单帧 16 B 在 115200 下 1.39 ms，
  FIFO 完全吸收。持续满速流的上限见 6.3 / 6.7。
- 没有应用层序列号/ACK；响应按请求顺序返回。粘包/丢帧时靠 NACK 和
  stats 定位。
- stats 计数上报是 `uint16_t` 饱和值，长时间跑会封顶；需要精确累计可扩成
  32 位多页。
- 心跳 1 Hz；上电 `READY` 只发一次，电脑晚打开会错过，但心跳能补上。
- RTT 包含 USB-TTL 驱动延迟；不要把它当成固件处理时间。

---

## 10. 排障

| 现象 | 优先检查 |
| --- | --- |
| 完全收不到心跳 | 波特率、PA9/PA10 是否接反、GND、固件是否复位运行 |
| 有心跳但 PING 无响应 | USB-TTL TX 是否真的接到 PA10；用 `tools/probe.py` 看原始字节 |
| 偶发 CRC 错 | 线太长/杜邦线接触不良/电平 5V；先检查接线和共地 |
| 大量 TIMEOUT | 帧内字节间隔超过 50 ms（`APP_FRAME_TIMEOUT_MS`）；检查发送端是否分片太慢 |
| `ring_drop_bytes` 增长 | 主机发送太快/太长突发；加大 ring 或加流控；见 6.3 |
| `tx_drop_bytes` 增长 | 响应产生速度超过 TX DMA 排空速度；加大 TX FIFO/流控；见 6.7 |
| DMA 完全没数据 | `.dma_buffer` 段是否还在 RAM_D2；DMA1_Stream1(RX)/0(TX) 是否使能 |

快速探测：

```powershell
python tools/probe.py --port COM12 --baud 115200 --seconds 3 --repeat 20
```

---

## 11. 实测环境备注

- 电脑：Windows，Python 3.14，pyserial 3.5；
- USB-TTL：CH343（VID:PID = 1A86:55D3），COM12；
- 固件：`Debug`，GCC 14.3.1（ST GNU Tools for STM32）；
- 烧录：STM32CubeProgrammer 2.23.0，ST-Link V2；
- 测试时间：2026-11-23。

如果换板/换线，先跑 `tools/test_uart.py` 的 ping/calc，再跑 `--perf-count 1000`。
