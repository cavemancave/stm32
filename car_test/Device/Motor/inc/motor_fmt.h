#ifndef __MOTOR_FMT_H__
#define __MOTOR_FMT_H__

#include "main.h"

#include <stddef.h>

/*
 * 电机协议值的「人话化」小工具：只做纯计算/格式化，不碰串口、不碰 RTOS。
 * 日志里打模式名和故障码都靠这里，换成别的显示方式时只改这一个文件。
 */

/* 模式值 -> 可读名字（只用于日志）。
   注意 0xA0 反馈里的模式值是**切换后的实际模式**：发 0x08（使能）之后
   电机回的是 0x01（默认电流环），不会把 0x08 回显回来。 */
const char *Motor_ModeName(uint8_t mode);

/* 故障码 -> 可读文本（按位拼，多个故障用 | 连起来；无故障 = "none"）。
   认不出来的位统一写成 "other"，原始码由调用方自己打（fault=0x%02X）。 */
void Motor_FaultText(uint8_t fault, char *out, size_t out_size);

/* 角度 -> 位置环给定值：位置值 0~32767 对应 0~360°（见 docs/specification.md）。
   例：0° -> 0、90° -> 8191、360° -> 32767。 */
uint16_t Motor_AngleToPosition(uint16_t angle_deg);

/* 位置值 -> 角度，返回**0.1°** 为单位（整数，避免用浮点）。
   例：8191 -> 900（= 90.0°）。日志里按 %u.%u 打即可。 */
uint16_t Motor_PositionToDeciDeg(uint16_t position);

#endif /* __MOTOR_FMT_H__ */
