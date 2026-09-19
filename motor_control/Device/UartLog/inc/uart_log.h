#ifndef __UART_LOG_H__
#define __UART_LOG_H__

#include "main.h"

/*
 * 调试日志口（UART7，115200）。
 *
 * 只是给 HAL_UART_Transmit 套一层**互斥**：改 FreeRTOS 之后可能有多个任务
 * 同时打日志，不加锁的话两行文本会互相插进对方中间。
 *
 * ⚠ 用的是阻塞发送（HAL_MAX_DELAY）：
 *   - 只在任务里调用
 *   - 别在定时器回调 / 中断里调用
 */

/* 绑定日志串口 + 建互斥量。必须在 osKernelInitialize() 之后调用 */
void UartLog_Init(UART_HandleTypeDef *huart);

/* 打一行（字符串自己带 \r\n），内部加锁，可多任务并发调用 */
void UartLog_Print(const char *text);

#endif /* __UART_LOG_H__ */
