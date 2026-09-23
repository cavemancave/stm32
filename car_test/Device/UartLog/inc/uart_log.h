#ifndef __UART_LOG_H__
#define __UART_LOG_H__

#include "main.h"

/*
 * 调试日志口。现在主口是**无线口 USART1**（App 侧：日志 + 控制帧 + OTA 三合一），
 * 还可以再挂镜像口（比如板上有线的 UART7）——两个口打同样的一行。
 *
 * 内部给 HAL_UART_Transmit 套了一层**互斥**：有多个任务同时打日志，
 * 不加锁的话两行文本会互相插进对方中间。
 *
 * ⚠ 用的是阻塞发送（HAL_MAX_DELAY）：
 *   - 只在任务里调用
 *   - 别在定时器回调 / 中断里调用
 */

/* 最多挂几个口（主口 + 镜像口） */
#define UART_LOG_MAX_SINKS   3U

/* 绑定主日志口 + 建互斥量。必须在 osKernelInitialize() 之后调用 */
void UartLog_Init(UART_HandleTypeDef *huart);

/* 再挂一个镜像口（同一个口挂两次会被忽略），要跟在 UartLog_Init 后面 */
void UartLog_AddSink(UART_HandleTypeDef *huart);

/* 打一行（字符串自己带 \r\n），内部加锁，可多任务并发调用 */
void UartLog_Print(const char *text);

/* 打开/关掉日志输出（OTA 升级期间关掉，别抢串口带宽） */
void UartLog_SetEnabled(uint8_t enabled);

/* 需要把"同一串口上的二进制帧"和文本日志串行化时用（发帧前调）。
   调度器还没跑起来时是空操作 */
void UartLog_Lock(void);
void UartLog_Unlock(void);

#endif /* __UART_LOG_H__ */
