/**
 * @file    power_monitor.h
 * @brief   外部电源（VCC_IN）检测：PC4 = ADC1_INP4，1 MΩ/100 kΩ 分压 11:1。
 *
 * 用途：只接 ST-Link 的 SWD 时 MCU 可能能跑，但 USART1/USB-TTL 不一定工作。
 * 通过 PC4 读 VCC_IN，> 5 V 认为外部电源已上电（12 V / 6S 电池），
 * 测试脚本据此提示。断电时输入可能浮空到 2~3 V，用 5 V 门槛避免误判。
 */
#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

void     power_monitor_init(void);
void     power_monitor_poll(uint32_t now_ms);

bool     power_monitor_valid(void);
bool     power_monitor_present(void);
uint16_t power_monitor_vcc_mv(void);
uint16_t power_monitor_pin_mv(void);

#endif /* POWER_MONITOR_H */
