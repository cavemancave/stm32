# motor_control — 两轮差速小车固件

STM32H723VGT6（达妙 DM-MC-Board02 / CtrBoard-H7）上的**两轮差速小车**固件，
从 `hello_imu` 工程复制而来（IMU 部分保留但暂时关着，见下）。

一句话硬件拓扑：

```
        ┌──────────────── STM32H723 ────────────────┐
PC14 ──►│ 电机电源使能（高有效，上电默认关断）          │
        │                                            │
        │  USART10 (PE2/PE3, 38400) ──► 电机1 ┐       │
        │                                     ├ 同一条总线，靠 ID1 / ID2 区分
        │                               ──► 电机2 ┘       │
        │                                            │
        │  USART1  (PA9/PA10, 921600) ◄──► 无线串口模块 ◄──► 电脑
        │     └─ 上报日志 + 收控制命令 + OTA 三合一               │
        │                                            │
        │  UART7   (PE7/PE8, 115200) 日志镜像（有线调试用）      │
        └────────────────────────────────────────────┘
```

- **两个轮子（两台电机）共用一条 `USART10`**：38400 8N1、一问一答、固定 10 字节帧，
  靠帧里的 **ID** 区分（ID1 / ID2，ID 脚在上电时锁存）。轮子用**速度环**（`0xA0/0x02`）——
  要的是“一直以某个转速转”，不是位置环那种“转到位就停”。
- **`USART1`（PA9 = TX / PA10 = RX，921600）接无线串口模块**：电脑那边是一个配对的串口。
  这一个口同时承担三件事：**上报调试日志（裸文本）+ 收控制命令（二进制帧）+ 无线 OTA 升级**。
- 小车侧自己做**里程差闭环（走直线）+ 看门狗**：上位机只发“左右轮各多少 rpm”，
  设备 25 ms 一拍读两台位置、把偏差补回左右轮；超时没新命令就自动停。
- 上位机工具在 `tools/`：`ota.py`（固件升级）、`motor.py`（电机命令）、`car.py`（小车：左右轮识别 / 差速走 / 键盘遥控）。
  三者共用协议层 `proto.py`（串口 / COBS / CRC32 / 组帧），互相独立。
- 板载 WS2812：App 现在**不点灯**（上电只发一帧全灭），接口/接线见 [docs/hardware.md](docs/hardware.md)；
  电机电源由 **PC14** 控制（上电默认关断，要动电机先上电）。
- 无线 OTA 是 **A/B 双槽**：新固件写另一个槽，校验通过后重启切换，跑不起来自动回滚，
  平时不用再插 ST-Link。

> **BMI088 暂时关掉**：`Core/Src/main.c` 里初始化 / 打包 / 发送三处都用 `#if 0` 关着，
> 要跑 IMU 时把这三处改回 `#if 1`（网页版上位机 <https://imu.steppeschool.com/>，
> 115200、陀螺仪量程选 ±2000 dps）。

## 文档

- **使用手册**
  - [两轮差速小车（速度环）](docs/car.md) —— **主用途**：左右轮识别 / 遥控 / 走直线闭环
  - [无线 OTA 升级](docs/ota.md) —— 以后不用再插 ST-Link
- **参考**
  - [电机 10 字节协议](docs/motor_protocol.md)
  - [代码结构 / 任务 / CMake 片段](docs/architecture.md)
  - [硬件要点 / 串口 / 电源监测](docs/hardware.md)
  - [构建与 CubeMX 注意事项](docs/build.md)
  - [OTA 方案设计取舍](docs/ota_design.md)
  - [电机规格书摘录](docs/specification.md)

## 快速开始（两轮差速小车）

> 细节（每条命令、闭环原理、左右轮映射、遥控）在 [docs/car.md](docs/car.md)，这里只给最短路径。

串口**不用写**：系统上只有一个 USB 串口时脚本会自己用它（Windows 的 `COM*` 和 Linux 的
`/dev/ttyACM*`、`/dev/ttyUSB*` 都认，用了哪个会打一行提示）。要换就 `--port COM7`
（写在子命令后面也行）、或者设环境变量 `CAR_PORT`。

```bash
pip install pyserial

python tools/car.py setup                     # ① 电机上电 + 两台使能 + 切速度环 + 配闭环
python tools/car.py status                    #    看两台的模式 / 位置 / 里程 / 故障码 / 电源
python tools/car.py id                        # ② 识别哪台是左轮、哪侧要反向（车架空！）
python tools/car.py drive --left 20 --right 20 --seconds 3    # ③ 直行 3 秒
python tools/car.py teleop --speed 30         # ④ 键盘遥控：按住就走、松手就停
python tools/car.py stop --disable --poweroff # ⑤ 收工：停车 + 失能 + 断电机电源
```

⚠ **速度环下电机一旦给了转速就会一直转**（不像位置环走到位自己停）。`car.py` 每个动作
都是“给转速 → 等一会 → 一定给 0 停下”（Ctrl+C 也会在 `finally` 里停），但**手别离开电源开关**，
第一次测试先把轮子架空。

> 板上的 PA15（USER_KEY）现在只留给 **Bootloader**：上电时按住 = 强制进恢复台。

