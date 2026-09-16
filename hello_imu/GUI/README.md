# BMI088 上位机

这是一个基于 Python/Tkinter 的串口监视器，读取 `hello_imu` 输出的 CSV 数据并实时显示陀螺仪、加速度和温度曲线。

## 运行

在 `GUI` 目录打开 PowerShell：

```powershell
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe imu_viewer.py
```

串口设置在 `config.ini` 中，当前默认是 `COM12`、`921600`、`8N1`，无需在界面里重复配置。点击“预览数据”可在没有开发板时查看界面效果。

## 两种数据来源：串口 / UDP

界面顶栏有两个连接按钮，同一时刻只跑一路数据源（点另一个会自动断开当前的）：

| 按钮 | 数据链路 | 配置段 |
| --- | --- | --- |
| `连接串口` | USB/TTL 直连开发板 | `config.ini` 的 `[serial]` |
| `连接 UDP` | 开发板 → ESP32 → WiFi/UDP → 本机 | `config.ini` 的 `[udp]` |
| `断开` | 停止当前数据源 | — |

走 WiFi 时链路是：

```text
STM32 (UART7) ──串口──> ESP32 (UART2) ──WiFi/UDP──> 本机 imu_viewer.py
                                              （本机只监听，不主动找设备）
```

和 ESP32 固件（`udpserial` 工程）的分工一致：**本机是纯服务端**，只 `bind` 端口等 ESP32 发来第一包数据，所以这里不需要填 ESP32 的 IP；设备地址是从收到的包里学来的，换设备/换端口都能自动跟上。要连上需要满足：

- ESP32 固件里的 `UDP_SERVER_PORT` 和 `[udp] port` 一致（默认都是 `3333`）。
- 固件靠 mDNS 找 PC（默认解析 `server.local`），所以跑本程序的机器主机名要能解析成 `server.local`；不想用 mDNS 就在固件的 `src/wifi_config.h` 里写死 `UDP_SERVER_IP`。
- 防火墙上放行该 UDP 端口。

注意 ESP32 是**按“串口空闲 3ms”切包**转发的，一个 UDP 包里可能是半行、也可能是好几行（丢包还会让行首错位）。所以本程序收到包后按 `\n` 重新拼行再解析，坏行直接丢掉并计数，不影响后续数据；界面底部的状态栏会显示收到的包数/字节数、有效行数和坏行数，方便判断链路质量。

同一个 UDP 端口不要被两程序同时监听（比如又开 `udpserial/tools/udp_server.py`）：UDP 包只会投给其中一个，两边都会收到一半、都显示不全。

程序启动后会在 Python 控制台输出串口打开、连接、断开和异常堆栈信息。需要查看每一行原始串口数据时，在 PowerShell 中运行：

```powershell
$env:BMI088_LOG_LEVEL = "DEBUG"
py imu_viewer.py
```

默认日志级别为 `INFO`，避免 100 Hz 数据流刷满控制台。

界面左侧的姿态视图使用互补滤波：加速度计修正 `roll/pitch`，陀螺仪积分 `yaw`。当前协议没有磁力计，因此 `yaw` 没有绝对方位参考，长时间运行会产生漂移；如果需要完全恢复绝对三轴朝向，需要增加磁力计或其他外部方位参考。

协议行格式：

```text
gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,accel_x_g,accel_y_g,accel_z_g,temp_c
```