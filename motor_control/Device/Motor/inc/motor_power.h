#ifndef __MOTOR_POWER_H__
#define __MOTOR_POWER_H__

#include "main.h"

/*
 * 电机电源开关。
 *
 * 硬件：电机（连同它的驱动板）现在不是直接吃板载 5V，而是接在一路**可控电源输出**上，
 *       由 **PC14** 控制：**高电平 = 使能（供电）**，低电平 = 关断。
 *
 * ⚠ 引脚说明：PC14 = OSC32_IN、PC15 = OSC32_OUT。这板子上没焊 32.768 kHz 晶振，
 *   所以这两个脚当普通 GPIO 用（板载 5V 的使能 Power_5V_EN 就是 PC15，一直在用）。
 *   它们属于备份域、驱动能力很弱（几 mA），**只能当使能信号**，真正的电流由电源那边出；
 *   别拿它直接带负载。
 *
 * 设计取舍：
 *   - **上电默认关断**：板子一上电电机不带电，要动它才使能（安全 + 省电 + 升级时不带电）。
 *   - 使能之后要等电源轨稳定（电容充电 + 驱动器复位 + 电机内部启动），用
 *     MotorPwr_OnAndSettle() 而不是裸 Enable()。电机刚上电那几条命令本来就会被丢掉
 *     （motor_ctrl.c 的使能流程有重试），先等一下能省掉大部分重试。
 *   - 关断之后要等泄放，"断电重启"才不会变成"没断干净就上电"。
 */

/* 使能后等电源轨稳定：驱动板复位 + 电机内部启动需要的时间 */
#define MOTOR_PWR_SETTLE_MS      500U

/* 关断后等电源轨泄放（PowerCycle 用） */
#define MOTOR_PWR_DISCHARGE_MS   500U

/* 配 PC14 为推挽输出并保持关断。越早调越好（复位后到配置前，那根使能线是悬空的） */
void MotorPwr_Init(void);

/* 开/关（1 = 高电平使能，0 = 关断）。状态没变时不重复写、不重复打日志 */
void MotorPwr_Enable(uint8_t on);

/* 现在是使能状态吗 */
uint8_t MotorPwr_IsOn(void);

/* 使能 + 等 MOTOR_PWR_SETTLE_MS */
void MotorPwr_OnAndSettle(void);

/* 断电重启：关 → 等泄放 → 开 → 等稳定（电机状态卡死、或者想让它重新自检时用得上） */
void MotorPwr_PowerCycle(void);

#endif /* __MOTOR_POWER_H__ */
