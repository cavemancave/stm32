# 无线 OTA 升级（以后不用再插 ST-Link）

> 返回 [README](../README.md)。方案设计/取舍（为什么 A/B 双槽、为什么单 bank 不能自己改自己、
> 各种异常怎么办）见 [`ota_design.md`](ota_design.md)；本页只说怎么用。

无线模块接在 **USART1（PA9 = TX / PA10 = RX）** 上，电脑那边是一个配对的串口。
这一个口平时就是**调试日志 + 控制命令**，要升级时同一套协议里夹着传固件。

## 硬件上已经跑通的（2026-09-19 实测）

| 能力 | 验证方式 |
| --- | --- |
| 无线升级固件 | `upgrade` → 96380/96380 字节、`校验通过：槽 B crc32=…`、设备自己重启切槽 |
| A/B 切槽 | BL 打印 `pending image OK -> switch to slot B` + `jump slot B PC=0x08094CCD`（PC 落在 B 槽） |
| 启动确认 + 清计数 | App 跑满 2 s 打印 `ota: boot confirmed: slot=B, size=…, crc=…`，`info` 里 `启动计数 0/3` |
| 一键回滚 | `rollback` → 重启后 `活动槽 A` |
| BL 恢复台 + 超时兜底 | `reboot --boot` → `flags=0x81`、`info` 提示“设备正在 Bootloader 恢复台”；15 s 不动自动尝试启动 |
| 两台电机（同总线 ID1/ID2） | `motor.py status` 两台都有回复；`motor.py movepos … --motor 2` 能让 2 号机动作 |
| 电机电源开关（PC14） | 上电默认关断；`motor.py enable`/`speedloop`/`drive` 先上电并等 500 ms 再发使能帧 |
| 电源电压监测（PC4/ADC1_INP4） | Boot 打两行（英文）：`[power] VCC_IN = 23.87V, cell 3.97V (6S), ~77%` + `[power]   ADC1_INP4/PC4 raw=43092, divider 11:1, VREF=3300 mV, full scale 36.30V`；`motor.py status` 每台回帧末尾都带电压/单片/剩余百分比（**电机没答上也打印**） |
| 两台电机各走一圈（每步 90°） | 先 `motor.py pwron` + 两台 `motor.py enable` + `motor.py posloop`（都复核 `mode=0x03`），再 `motor.py movedeg 90/180/270/0` 各 4 步：8 条全部 `reply=OK`，结束时两台**里程都是 1 圈**、回到起始角度（348.2° / 346.0°），fault=0x00 |
| 升级速度实测 | 113 KB 镜像：1024 块/115200 **4.2 KB/s、28.2 s** → 4096 块/115200 **6.4 KB/s、18.8 s** → 4096 块/921600 **12.7 KB/s、10.3 s** → 修掉上位机读串口白等 **18.0 KB/s、7.3 s** → 只擦镜像占用的扇区（BEGIN 里 `erase=1 sector(s)`） **26.6 KB/s、5.3 s**（累计 **5.3×**） |
| 无线升级（同一天又刷 4 次） | A↔B 往返刷了 4 次（含改代码后重新构建的），每次都 `校验通过` → 切槽 → `启动计数 0/3` |
| 断点续传 / 坏包拒绝 | ⬜ **还没单独实测**（设计如此：进度 32 KB 落盘；`END` 用读回 CRC32 把关，不过就不提交） |

## Flash 怎么分的、为什么不会变砖

```
0x08000000  128K  Bootloader（永不 OTA 更新，只在第一次用 ST-Link 烧）
0x08020000  384K  Slot A  ←  motor_control.bin
0x08080000  384K  Slot B  ←  motor_control_slotB.bin（同一份源码、只换链接基址）
0x080E0000  128K  元数据（在哪个槽 / 版本 / CRC / 续传进度，顺序追加写）
```

* 正在跑的槽**永远不写**（协议里直接拒绝）—— 新固件写进另一个槽，校验通过后重启，由 Bootloader 切槽；
* 新固件跑起来 2 s 后要「报到」（写启动确认）。连着 3 次没报到 = 判坏，**自动回滚**到旧槽；
* 两个槽都跑不起来 → 上电自动进 Bootloader 的**恢复台**，同一个口重发固件就行
  （`flash --slot A/B` 指定写哪个槽）；
* 传输断了/掉电了不要紧：进度每 32 KB 落一次盘，重新 `flash` 会从断点续传；
* Bootloader 自己不 OTA（H723 单 bank 没有硬件 bank swap，自升级失败就真砖了），
  所以它只在第一次烧一次 —— 这是整个方案的地基。

## 第一次（也是最后一次）用 ST-Link

| 烧什么 | 烧到哪 |
| --- | --- |
| `build/Debug/motor_boot.hex`（或 .bin/.elf） | `0x08000000` |
| `build/Debug/motor_control.hex`（或 .bin/.elf） | `0x08020000` |

第一次上电时元数据是空的，Bootloader 会「看着向量表像 App」先跳进去；
App 跑满 2 s 后把自己登记到元数据里（长度 + CRC32 + 版本），之后就是正常状态。

### ⚠⚠ 烧录时最容易把板子“烧哑”的两个坑

1. **勾了 Full chip erase / Erase all 之后，两个 hex 都要烧回去！**
   只烧 App 的话 `0x08000000` 是空的，CPU 从空白 Flash 启动 =
   **上电后串口一个字符都没有**（看起来像板子坏了，其实只是 BL 没了）。
2. **别用 `.bin` 乱填地址。** `.bin` 不带地址信息，起始地址填错（尤其是把 App
   填成 `0x08000000`）会把 Bootloader 覆盖掉。优先烧 `.hex`（自带地址）。

想确认烧对了没有，用 ST-Link 读一下 `0x08000000` 的前 8 字节：
应该是 `00 02 00 20`（SP = `0x20020000`）后面跟一个 `0x0800xxxx`
（PC，最低位必须是 1）—— 那就是 Bootloader 的向量表。
`0x08020000` 处同理，只是 PC 变成 `0x0802xxxx`。

### 元数据（`0x080E0000`）自己坏掉了怎么办

不用慌，不会变砖：

* 新版 Bootloader 上电发现某个 flash word 坏了（读它 NMI/总线错误）会自动
  **把坏掉的扇区擦掉再复位**（`S0` 也就是 Bootloader 自己永不擦），然后正常启动；
* 也可以手动解决：ST-Link 把 `0x080E0000` 那个扇区擦掉（或者干脆全片擦除后
  把上面两个 hex 重烧一遍）。擦了元数据只是丢掉 A/B 记录，App 跑起来会重新登记；
* App 里如果登记失败，串口会打 `ota: boot confirm FAILED (st=..)`，不再静默。

## 日常升级

**串口不用写**：系统上只有一个 USB 串口时脚本自己用它（Windows 的 `COM*`、Linux 的
`/dev/ttyACM*` / `/dev/ttyUSB*` 都认，用了哪个会打一行提示）—— 下面省掉了 `--port`，
要换口就 `--port /dev/ttyUSB0`（写在子命令后面也行），或者设环境变量 `CAR_PORT`。
（口不存在时脚本会把系统上能用的串口列出来；能看见口但打不开是权限问题：
`sudo usermod -aG dialout $USER` 后重新登录。）

```bash
pip install pyserial
python tools/ota.py selftest                  # 不连板子：自检 CRC32/COBS/组帧/镜像槽识别
python tools/ota.py info                      # 看当前在哪个槽、版本、两个槽的 CRC
python tools/ota.py upgrade                   # ★一条命令升级：自动挑槽 + 自动挑镜像
python tools/ota.py flash build/Debug/motor_control_slotB.bin
python tools/ota.py rollback                  # 新固件有问题 → 一键切回旧槽
python tools/ota.py reboot --boot             # 手动进 Bootloader 恢复台（救砖）
python tools/ota.py erase A                   # 擦掉某个槽
python tools/ota.py monitor                   # 当串口监视器看日志
```

> 电机命令（使能/转速/走位/电源/电压…）在 `tools/motor.py`；开小车用 `tools/car.py`。

`flash` 会自动挑槽（写在非活动槽）并**检查你给的 .bin 是给哪个槽编的**
（.bin 里的复位向量一看就知道），给错了直接拒绝，避免把 A 的镜像写进 B 槽、跳过去必崩。

**`upgrade` 比 `flash` 更省事**：不带文件时会先问设备「现在跑在哪个槽」，再发**另一个槽**
对应的那份 `.bin`（A 槽在跑就发 `motor_control_slotB.bin`，反之发 `motor_control.bin`）。
所以日常升级就是一条命令；升级成功后设备切到另一个槽，**再跑一次 `upgrade` 又会切回来** ——
两个槽永远都是当前构建（`upgrade` 之后想回旧的用 `rollback`）。
也能显式指定：`upgrade --slot B`、`upgrade 某个.bin --force`（参数和 `flash` 完全一样）。

## 关于 CRC32：设备算的是「Flash 内容」，不是「.bin 文件」

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

## 这一个口上的三种东西怎么共存

| 内容 | 形式 |
| --- | --- |
| 调试日志 | **裸文本**（ASCII + `\r\n`），和以前打 UART7 一模一样（UART7 现在是镜像口） |
| 控制信息 | 二进制帧：`CTRL_CMD`（使能/失能/切位置环/走位置/急停/静音/停轮询）、`CTRL_STATUS`（里程/位置/故障码） |
| 固件 | 二进制帧：`OTA_BEGIN/DATA/END`（1 KB 一包，停等 + 重传 + 续传） |

帧的封装是 `0x00 + COBS(帧) + 0x00`：文本里不出现 `0x00`，COBS 编码后的帧里也不出现 `0x00`，
所以上位机看到两个 `0x00` 之间的一段就试着解帧、CRC32 不过就当文本丢掉 —— 三种内容不打架。

## 升级期间板子会做什么

`OTA_BEGIN` 一到：**电机全部失能** + **停 200 ms 状态轮询** + **静音日志**（别抢带宽），
然后擦除目标槽的 3 个扇区（一次做完，之后只剩 32 字节 flash word 的编程停顿），逐块收数据；
30 s 收不到任何帧就自动放弃并恢复日志/轮询（已写入的内容保留，下次续传）。
升级完成后需要**用户自己**再发命令（`motor.py speedloop` / `motor.py enable`）使能电机，不会自动转。

## ⚠ 别在 CubeMX 里勾 USART1

`Device/Ota/ota_com.c` **自己**初始化 USART1（RCC + GPIO + NVIC + `HAL_UART_Init`）：
Bootloader 目标里根本没有 `Core/Src/usart.c` / `stm32h7xx_it.c`（它们是生成文件、每次重写），
放自己文件里两个目标都能用、也不会被覆盖。
如果你更习惯 CubeMX：勾上 USART1 之后，把 `ota_com.c` 里的硬件初始化和 `USART1_IRQHandler`
删掉、改用 `MX_USART1_UART_Init()` + `huart1` —— **两套只能留一套**，否则重复定义。
