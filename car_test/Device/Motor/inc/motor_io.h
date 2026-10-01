#ifndef __MOTOR_IO_H__
#define __MOTOR_IO_H__

#include "main.h"
#include "motor.h"

/*
 * 电机串口「一问一答」收发服务（FreeRTOS）
 *
 * 为什么要有这一层：
 *   HAL_UART_Transmit/Receive 是**轮询阻塞**的，而且电机超时最长要等
 *   MOTOR_FRAME_TIMEOUT_MS(100ms)。如果让控制任务直接收发，控制任务就被卡住了。
 *   所以把 USART10 交给一个专门的低优先级任务（MotorIo_Task）独占，
 *   别的任务通过队列把请求丢进去，然后等通知拿结果。
 *
 * 用起来跟同步函数一样（内部阻塞），但**线程安全**：
 *
 *     uint8_t rx[10] = {0};
 *     uint8_t tx[10] = { ... };
 *     if (MotorIo_Exchange(tx, 10, rx, 10) != 0U) { ... }
 *
 * 谁调用都行 —— MotorIo_Exchange 会看当前是不是收发任务自己：
 *   - 不是 → 组一个请求丢进队列，然后阻塞等收发任务通知（拿返回值）
 *   - 是   → 直接收发，不绕队列（否则自己等自己，死锁）
 * 所以 motor.c 里的 Motor_Transaction() 不用改调用方式，
 * 上层 Motor_Enable() / Motor_QueryStatus() / ... 全自动变线程安全。
 *
 * ⚠ 收发任务只被 MotorIo_Task 一个任务执行，MotorIo_Task 由 freertos.c 里的
 *    motor_task（CubeMX 建的）入口调用，不要在别的地方再起一个。
 */

/* 请求队列深度：同时能挂多少个待处理的收发请求 */
#define MOTOR_IO_QUEUE_LEN             8U

/* 清 RX 时最多丢掉多少字节（**硬上限，绝不能不设上限**）。
   原因见 motor_io.c 里 MotorIo_FlushRx 的注释：
   单线 5V 总线上如果线被拉低 / 电机没上电 / 波特率不对，
   USART 会按波特率源源不断产生字节，没有上限的 while 就永远出不来 ——
   表现是「开机卡死在这里、一声不吭」。 */
#define MOTOR_IO_FLUSH_MAX_BYTES       64U

/* 打印警告时顺手把「线上到底是什么」抓前几个字节出来看 */
#define MOTOR_IO_FLUSH_SNIFF_LEN       8U

/* 往队列里投递的最长等待（队列满时才用得上） */
#define MOTOR_IO_QUEUE_TIMEOUT_MS      200U

/* 等收发任务回结果的最长等待：一次收发最坏 = 发超时 + 收超时，再留点余量 */
#define MOTOR_IO_EXCHANGE_TIMEOUT_MS   (MOTOR_FRAME_TIMEOUT_MS * 2U + 100U)

/*
 * 初始化：绑定串口 + 建请求队列。必须在 osKernelInitialize() 之后调用
 * （freertos.c 的 MX_FREERTOS_Init() 里会调）。
 *   huart - 电机所在串口，传 &huart10
 *
 * 内部会顺手把 RX 里残留的字节清掉（有上限，最多 MOTOR_IO_FLUSH_MAX_BYTES），
 * 清不干净只会放弃并打一行日志，**不会阻塞启动**。
 */
void MotorIo_Init(UART_HandleTypeDef *huart);

/*
 * 收发任务本体。**不返回**，由 freertos.c 里的 motor_task 入口调用。
 */
void MotorIo_Task(void *argument);

/*
 * 一次「发 tx_len 字节 → 收 rx_len 字节」的问答，内部自己串行化。
 *
 *   tx / tx_len - 要发的数据（本函数返回前调用方要保证它们还有效）
 *   rx / rx_len - 接收缓冲，**调用前请先清零**：收不满时没被写到的字节保持 0，
 *                 失败时打出来就是「收不完整」的现场（全 0 = 一个字节都没收到）
 *
 * 返回 1 = rx_len 个字节都收到了；0 = 发送失败 / 收超时 / 排队超时。
 */
uint8_t MotorIo_Exchange(const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len);

#endif /* __MOTOR_IO_H__ */
