#ifndef __MOTOR_DRIVE_H__
#define __MOTOR_DRIVE_H__

#include "main.h"

/*
 * 小车底盘驱动：**里程差闭环（走直线）** + 看门狗，跑在自己的任务里。
 *
 * 为什么闭环放设备侧、不放上位机：
 *   上位机每 25 ms 读一次两台位置，每读一次都要过无线（10~30 ms 抖动），
 *   闭环会被链路抖死；而设备侧本来就在总线旁边。本工程一拍 = 2×0x74（读位置）
 *   + 2×0x64（给新转速）≈ 4 帧 ≈ 24 ms（10 字节帧 @38400 单程 2.6 ms），
 *   占空比 ~50%，和位置跟随那套同量级（它能跑到 40~50 Hz），放得下。
 *
 * 闭环在做什么（就是"走直线"那件事）：
 *   一台轮子走多了，车头就歪了。拿两台的**位置增量之差**积分当航向误差：
 *       e += (Δs_左 − Δs_右)        （乘上"泄漏"，约 1.6 s 时间常数，防漂）
 *       trim = e / TRIM_DIV         （限幅 ±TRIM_MAX）
 *       v_左 = 基准_左 − trim ;  v_右 = 基准_右 + trim
 *   ⚠ ψ 的增量 Δψ = (Δs_右 − Δs_左)/L 这个关系**对前进和后退都成立**，
 *     所以 trim 直接加在"左 − 右"上就行，不用管正反转（正 rpm 永远是"把车往前推"，
 *     左右/正反的映射由上位机在发命令前就做掉了）。
 *
 * 只在"直行类"动作下积分：两侧同号、且速度差不超过快的那侧的 1/MOTOR_DRIVE_STRAIGHT_DIV。
 *   转圈（一正一负）和明显走弧都自动停积分并清 trim —— 否则积分会跟你的转向指令打架
 *   （转圈时航向本来就在变，拿它当"误差"去修，车会越转越怪）。
 *
 * 里程/位置用哪个：
 *   **只用位置值（0~32767）的"最短路径"增量累加**，不用 0x74 的里程字段 ——
 *   实测里程在位置刚过零点时会掉一整圈（详见 motor_follow.c 的说明），
 *   拿它做差分会被当成"瞬移"。
 *
 * 看门狗（每条 DRIVE 命令自带"多久没新命令就自己停"）：
 *   遥控 20 Hz 心跳时给 400 ms；`car.py drive --seconds N` 给 N+1 s。
 *   这样松手、断线、笔记本睡死，车都会在超时后自己停，而不是靠补一条 stop 命令。
 */

/* 一拍多少 ms。2×0x74 + 2×0x64 ≈ 24 ms，再小就会"每拍都超时"（和跟随一个道理） */
#define MOTOR_DRIVE_PERIOD_MS      25U

/* 累计里程差（计数）→ trim（0.1rpm）的比例：
   546 计数 ≈ 10 秒的 1 个 0.1rpm 差 → 补 10 个 0.1rpm（约 1rpm 的修正量） */
#define MOTOR_DRIVE_TRIM_DIV       55

/* 积分的"泄漏"（每拍 e -= e / N）：N=64、一拍 25 ms ⇒ 时间常数 ≈ 1.6 s。
   作用是让稳态误差自己归零，避免长期偏置把 trim 顶到限幅 */
#define MOTOR_DRIVE_TRIM_LEAK      64

/* trim 输出限幅：±12 rpm。比这更大的修正说明不是"轮径差"，是别的问题，别硬顶 */
#define MOTOR_DRIVE_TRIM_MAX       120

/* 积分的上下限（计数）：±1.6e6 ≈ 1000 个 trim 单位，先夹住再除，防止溢出 */
#define MOTOR_DRIVE_ERR_MAX        (MOTOR_DRIVE_TRIM_DIV * 8 * (int32_t)MOTOR_DRIVE_TRIM_MAX)

/* "直行类"的判据：两侧同号且 |v1-v2| ≤ max(|v1|,|v2|) / N */
#define MOTOR_DRIVE_STRAIGHT_DIV   4U

/* DRIVE 命令第 6 字节的看门狗单位（ms）：0 = 不启用，最大 255×50 = 12.75 s */
#define MOTOR_DRIVE_WD_UNIT_MS     50U

/* 连续几拍读不到某台电机就停（"控制不住"比"乱动"安全） */
#define MOTOR_DRIVE_MISS_LIMIT     5U

/* 建任务（懒创建：第一次 Set 时建）。返回 1 = 任务已就绪 */
uint8_t MotorDrive_Init(void);

/*
 * 设基准转速（0.1rpm，按 motor_ids[] 的序号：speeds[0] = 1 号机、speeds[1] = 2 号机）。
 * 正 = 那台轮子把车往前推。全 0 = 停车。
 * watchdog_ms = 0 不启用看门狗；否则超过这么久没收到新的 Set 就自己停。
 * 返回 1 = 命令已被接受（真正发帧由循环做；电机没答上会在日志里看到）。
 */
uint8_t MotorDrive_Set(const int16_t *speeds, uint8_t count, uint32_t watchdog_ms);

/* 停车：两台 0x64 = 0，停循环、恢复 200 ms 轮询。可重复调用 */
uint8_t MotorDrive_Stop(void);

/* 1 = 闭环循环在跑 */
uint8_t MotorDrive_IsRunning(void);

/*
 * 里程差闭环开关 + "哪台是左轮"：
 *   arg = 0                → 关（trim 清零，积分清零）
 *   arg = 1..MOTOR_COUNT   → 开，并且这个数就是左轮的**电机序号**（1 起，= motor_ids[] 下标+1）
 * ⚠ 只能在没跑的时候改：跑着改"哪台是左"会让车突然反向修，那一下很冲。
 * 返回 1 = 已生效
 */
uint8_t MotorDrive_SetTrim(uint8_t left_index);

/* 读回闭环状态：*enabled = 0/1、*left_index = 左轮序号（0 = 没配）、*trim_out = 当前 trim */
void MotorDrive_GetTrim(uint8_t *enabled, uint8_t *left_index, int32_t *trim_out);

#endif /* __MOTOR_DRIVE_H__ */
