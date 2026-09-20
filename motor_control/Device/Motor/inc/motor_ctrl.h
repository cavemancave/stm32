#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__

#include "main.h"

/*
 * 电机业务流程（应用层）
 *
 * 分工：
 *   MotorCtrl_Task      —— 主控制流程：等按键 → 使能(重试) → 查版本 → 切位置环
 *                          → **再查一次模式确认真的在 0x03**（0x75/0x76，不是就失能退出）
 *                          → **单次测试**：回 0° → 分 4 步（每步 90°）走满一圈，
 *                            每步打一行“命令的位置 → 实际停的位置 + 误差 + 里程差”
 *                            （`motor_ctrl.c` 的 `MOTOR_SINGLE_MOVE_TEST` 改成 `0U` 就恢复 0°/90° 来回走）
 *                          → 再按一下键 → 失能(0xA0/0x09) → 回到等按键（再次按键重新使能）
 *   MotorCtrl_PollTask  —— 状态轮询：被 motorPos 定时器唤醒，查一轮 0x74
 *                          （里程/位置/故障码）打到日志口
 *   MotorCtrl_OnPollTimer —— 上面那个定时器的回调，**只释放信号量，不阻塞**
 *
 * 三个任务 + 一个定时器的接线都在 Core/Src/freertos.c 里，见那边的 USER CODE 段。
 */

/* 状态轮询周期（ms）。motorPos 是 CubeMX 建的 osTimerPeriodic，
   周期在 freertos.c 的 osTimerStart(motorPosHandle, ...) 里用它设定 */
#define MOTOR_CTRL_POLL_MS        200U

/* 模式查询（0x75）没答上时的返回值：0xFF 不可能是合法模式值 */
#define MOTOR_MODE_UNKNOWN        0xFFU

/* 建轮询信号量 + 轮询任务。必须在 osKernelInitialize() 之后调用 */
void MotorCtrl_Init(void);

/* 主控制流程（不返回）。由 freertos.c 里的 defaultTask 入口调用 */
void MotorCtrl_Task(void *argument);

/* 轮询定时器回调。由 freertos.c 里的 motorPosCallback 调用。
   ⚠ 跑在 FreeRTOS timer service task 里：绝对不能阻塞，只能释放信号量 */
void MotorCtrl_OnPollTimer(void);

/* ---- 无线控制入口（Device/Ota 的 OtaService 用） ----
 * 这个口平时不打 OTA 时就是"控制信息"通道：
 *   上位机发一帧 OTA_T_CTRL / OTA_T_STATUS，OtaService 转到这里执行。
 * 返回 OTA_OK / OTA_E_xxx（见 Device/Ota/inc/ota_layout.h） */

/* 进入/退出 OTA 模式：进入 = 电机失能 + 停轮询 + 禁止走动命令 */
void    MotorCtrl_SetOtaMode(uint8_t on);
uint8_t MotorCtrl_OtaMode(void);

/* 控制命令：cmd = OTA_CTRL_xxx，arg 是参数，out/out2/out3 回传三个附加信息
 * （对应无线回帧里的 data / data2 / data3；后两个可以传 NULL）。
 *
 * motor_index = 电机序号（= motor_ids[] 里的下标 +1，也就是“1 号机 / 2 号机”，不是总线 ID）：
 *   0            = 没指定。安全类命令（失能/急停）作用于**全部**；
 *                  运动类（使能/切位置环/走位）默认作用于 **1 号机**（老的命令行不变）。
 *   1..MOTOR_COUNT = 只作用于那一台。
 * 序号越界返回 OTA_E_PARAM。
 * ⚠ 单台失能不会切电源（两台共用一路电源，另一台可能还在干活）；
 *   “失能全部”（motor_index = 0）才会把 PC14 那一路电也断掉。
 * ⚠ **跟随在跑的时候，任何手动命令都会先把跟随停掉**（除了 FOLLOW/SPIN 自己）：
 *   否则跟随循环下一拍就把电机拽回目标位置，急停/失能会“按不住”。 */
uint8_t MotorCtrl_RemoteCmd(uint8_t motor_index, uint8_t cmd, uint32_t arg,
                            uint32_t *out, uint32_t *out2, uint32_t *out3);

/* 只读状态快照（0x74 + 0x75）：里程 / 位置原始值 / 故障码 / 当前模式。
   motor_index 含义同上（0 = 1 号机） */
uint8_t MotorCtrl_RemoteStatus(uint8_t motor_index, int32_t *mileage, uint16_t *position,
                               uint8_t *fault, uint8_t *mode);

/* ---- 给跟随模块（motor_follow.c）用的内部工具 ----
 * 跟随循环要用到的“使能 / 切位置环 / 急停 / 停轮询”都已经在这里实现过一遍了，
 * 直接复用，免得两套实现以后各自漂移。 */

/* 无线命令里的“电机序号”（1 起）→ 总线 ID（帧首字节）；越界返回 0 */
uint8_t MotorCtrl_MotorIdOfIndex(uint8_t index);

/* 查一次模式值（0x75 -> 0x76），**不打日志**：跟随模块判断“要不要重新配位置环”用。
   查不到（超时/不答）返回 MOTOR_MODE_UNKNOWN */
uint8_t MotorCtrl_QueryModeValue(uint8_t motor_id);

/* 把一台电机弄进位置环：使能（自带重试）→ 0xA0/0x03 → 0x75 复核模式值真是 0x03。
   返回 1 = 已在位置环（过程打日志）；0 = 没成功（原因在日志里） */
uint8_t MotorCtrl_PreparePositionLoop(uint8_t motor_id);

/* 急停两台（0x64 给定值 0）→ 失能两台 → 切 PC14 电源。
   给跟随模块的“出事就停”和别的地方的异常回退用。 */
void    MotorCtrl_SafeStopAll(void);

/* 暂停 / 恢复 200 ms 状态轮询（内部计数，可以嵌套：OTA 会话和跟随各停一次，
   两边都恢复了才开始轮询 —— 否则先恢复的那个会把轮询开回来抢总线）。 */
void    MotorCtrl_PollPause(uint8_t on);

#endif /* __MOTOR_CTRL_H__ */
