# BMI088 上位机

这是一个基于 Python/Tkinter 的串口监视器，读取 `hello_imu` 输出的 CSV 数据并实时显示陀螺仪、加速度和温度曲线。

## 运行

在 `GUI` 目录打开 PowerShell：

```powershell
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe imu_viewer.py
```

串口设置在 `config.ini` 中，当前默认是 `COM12`、`115200`、`8N1`，无需在界面里重复配置。点击“预览数据”可在没有开发板时查看界面效果。

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