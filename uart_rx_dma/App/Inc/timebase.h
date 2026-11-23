/**
 * @file    timebase.h
 * @brief   DWT 周期计数器，用于微秒级处理时间统计。
 */
#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

void timebase_init(void);
uint32_t timebase_cycles(void);
uint32_t timebase_cycles_to_us(uint32_t cycles);

#endif /* TIMEBASE_H */
