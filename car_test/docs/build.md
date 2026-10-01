# 构建 与 CubeMX 注意事项

> 返回 [README](../README.md)。

## 构建

在 VS Code 里直接用 CMake Tools 构建即可（`CMakePresets.json` 已有 `Debug` / `Release`），
产物在 `build/Debug/`。命令行（Linux）时工具链不在默认 `PATH` 里：

```bash
export PATH="$HOME/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin:$HOME/.local/share/stm32cube/bundles/ninja/1.13.2+st.1/bin:$PATH"

cube-cmake --preset Debug && ninja -C build/Debug      # Release 换成 --preset Release / build/Release
```

构建一次会出三个镜像（用途见 [`ota.md`](ota.md)）：

| 产物 | 链接基址 | 用途 |
| --- | --- | --- |
| `motor_control.hex` / `.bin` | 0x08020000 (Slot A) | 主固件 |
| `motor_control_slotB.hex` / `.bin` | 0x08080000 (Slot B) | 同一份源码、只换基址 |
| `motor_boot.hex` / `.bin` | 0x08000000 | Bootloader，只在第一次烧 |

⚠ **换了机器/换 OS 之后 `build/` 里的路径是旧的**（`CMakeCache.txt`、`build.ninja` 里写着
`C:\Users\...\cmake.exe`），直接 `ninja` 会报 `cmake.exe: not found`。清一次重新 configure
就行（`build/` 全是生成物，删掉不心疼）。

⚠ **时间戳陷阱**：如果工程文件的时间戳比系统时钟晚（本机踩过：文件是 19:44、系统时间 14:07），
ninja 会一直报 `manifest 'build.ninja' still dirty after 100 tries, perhaps system time is not set`。
处理办法：把所有源码的时间戳规范化（排除 `build/`），再删掉 `build/Debug` 重新配置：

```powershell
$root='<repo>'; $t=(Get-Date).AddSeconds(-5)
Get-ChildItem -Path $root -Recurse -File | Where-Object { $_.FullName -notlike '*\build\*' } |
    ForEach-Object { $_.LastWriteTime = $t }
Remove-Item -Recurse -Force "$root\build\Debug"
```

## CubeMX 重新生成代码后要检查什么（★ 容易忘）

1. `cmake/stm32cubemx/CMakeLists.txt` **每次** “Generate Code” 都会被重写；
   根 `CMakeLists.txt` 只在**第一次**生成，之后的用户改动不会被覆盖。
2. 自己新增的驱动**不要**直接写进生成文件，统一走「独立 cmake 片段」模式：
   - 配置放在 `cmake/bmi088.cmake`、`cmake/ws2812.cmake`、`cmake/motor.cmake`、
     `cmake/motor_ctrl.cmake`、`cmake/motor_drive.cmake`、`cmake/uart_log.cmake`、
     `cmake/power_mon.cmake`、`cmake/ota.cmake`（CubeMX 不认这些文件，永远不会被覆盖）
   - 由根 `CMakeLists.txt` 里的几行 `include(cmake/xxx.cmake)` 拉进来
   - ⚠⚠ **`cmake/ota.cmake` 里 SlotB 那份库清单是手写的**：主目标的 `include(cmake/xxx.cmake)`
     里那些 `target_link_libraries(${CMAKE_PROJECT_NAME} xxx)` **不会自动跟过去**。
     新加一个库（比如 `motor_drive`）时**两处都要加**，否则只有 B 槽编译镜像时
     报 `undefined reference`（A 槽没事），看起来像“同一份源码两份镜像行为不一样”。
   - ⚠ **请求队列不在 CubeMX 里**：`Device/Motor/motor_io.c` 自己用 `xQueueCreate()` 建。
     以前在 CubeMX 里配的那个 `motorQueue`（item type `uint16_t`，2 字节）装不下一个请求，
     已经删掉了；别在 CubeMX 里再加回来。
   - ⚠ **无线 OTA 的三个镜像**都在 `cmake/ota.cmake` 里建，链接基址靠
     `STM32H723xG_slots.ld` + `--defsym APP_BASE`，同样不要写进生成文件。
3. regenerate 之后如果发现某个驱动没编进固件，**先看那几行 `include()` 还在不在**，
   不要急着改 `cmake/stm32cubemx/CMakeLists.txt`（改了下次还会丢）。
4. `freertos.c` 里、`USER CODE` 段之外的**任务入口名**（`StartDefaultTask` / `MotorTask` /
   `motorPosCallback`）要和 `.ioc` 一致；实际业务都在 `Device/` 下，`freertos.c` 只负责把它们接上。
5. 从别的工程复制过来之后，记得把根 `CMakeLists.txt` 里的
   `set(CMAKE_PROJECT_NAME ...)` 改成自己的工程名（因为它不会被重新生成，
   不改的话产物一直叫 `hello_imu.elf`）。

> 教训（2026-09-16）：BMI088 的源文件一开始是直接加在生成文件里的，一次 “Generate Code”
> 就被抹掉了，之后才改成现在的片段写法。

6. **`STM32H723xG_flash.ld` 已经被弃用**：CubeMX 每次还是会重新生成它，但根 `CMakeLists.txt`
   把工具链里那行写死的 `-T .../STM32H723xG_flash.ld`（和 `-Map` 名字）摘掉了，
   三个镜像统一用 **`STM32H723xG_slots.ld`** + `--defsym APP_BASE`。
   所以不要去改那个生成出来的 `.ld`（改了也没用），内存布局有变化时同步
   `STM32H723xG_slots.ld` 里那几行 `MEMORY` 就行。
7. **不要**在 CubeMX 里勾 USART1（原因见 [`ota.md`](ota.md)）。引脚清单里 PA9/PA10 应该一直是“未分配”。
   ⚠ 反例：**ADC1 / PC4 是勾在 CubeMX 里的**（`Core/Src/adc.c` 由它生成，含 PLL2 = 48 MHz
   的 ADC 时钟、16 位分辨率、387.5 周期采样、PC4 模拟模式）。重新生成之后别把 ADC1 取消了，
   也**不要把采样时间改短**（源阻抗 ≈ 91 kΩ，见 `Device/Power/power_mon.h`）。
8. 升级/分区相关的动态都在 `USER CODE` 段里：`Core/Src/main.c` 只多一行 `SCB->VTOR = OTA_APP_BASE;`
   （**必须在 `MPU_Config()` 之前**），`Core/Src/freertos.c` 里只改了日志口和加上 OTA 服务初始化。
