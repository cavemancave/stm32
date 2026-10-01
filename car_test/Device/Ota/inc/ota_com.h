/**
  ******************************************************************************
  * @file    ota_com.h
  * @brief   无线 OTA 的串口层：USART1（PA9=TX / PA10=RX）独占收发
  *
  * 这个口是"三合一"的：平时打调试日志 + 传控制帧，要升级时收固件。
  *
  * 为什么自己初始化 USART1、而不是在 CubeMX 里配：
  *   `Core/Src/usart.c`、`stm32h7xx_it.c` 每次 "Generate Code" 都会被重写，
  *   而 Bootloader 目标里**根本没有**这两个文件（它是独立的最小工程）。
  *   放这里两个目标都能用，也不会被 CubeMX 覆盖。详见 docs/ota_design.md §7.3。
  *
  * 收：中断收单字节 + 16 字节硬件 FIFO + OTA_PORT_RX_RING 字节软件环形缓冲。
  *   ⚠ 打开 FIFO 很重要：写 Flash 时 CPU 会被 stall 几十 µs，只有 FIFO 能顶住不丢字节。
  *   （没用 DMA：省得动 CubeMX 的 DMA 配置，115200 下中断收 1 字节的负载可以接受。）
  ******************************************************************************
  */

#ifndef __OTA_COM_H__
#define __OTA_COM_H__

#include <stdint.h>
#include "stm32h7xx_hal.h"

/* 初始化 USART1（含 RCC/GPIO，不需要 CubeMX 里配）。
   ⚠ 它**不**开接收中断：收由 OtaCom_StartRx() 单独开，
     因为 App 里必须等调度器跑起来才能让接收中断进 FreeRTOS 的 API（见 ota_service.c）。 */
void OtaCom_Init(uint32_t baud);

/* 开始接收（使能 NVIC + 武装 HAL_UART_Receive_IT）。重复调用无副作用。
   BL 在 OtaCom_Init 之后直接调；App 在 otaSvc 任务里调（那时调度器已经跑起来了） */
void OtaCom_StartRx(void);

/* App 用它当调试日志口（UartLog_Init(OtaCom_Handle())） */
UART_HandleTypeDef *OtaCom_Handle(void);

/* 串口是不是已经配好了（收/发都能用）。诊断代码用它决定走哪个句柄 */
uint8_t OtaCom_IsReady(void);

/* 阻塞发送（别在中断/定时器回调里调用） */
void OtaCom_Write(const uint8_t *data, uint32_t len);

/* 从环形缓冲取字节（非阻塞），返回取到的个数 */
uint16_t OtaCom_Read(uint8_t *dst, uint16_t max);

/* 收到新字节时在**中断上下文**里被调用，用来唤醒任务
   （App 里是 vTaskNotifyGiveFromISR；Bootloader 不用，传 NULL 即可） */
void OtaCom_SetRxHook(void (*hook)(void));

/* 丢弃还没取走的接收数据（比如刚建立会话、想忽略旧字节时） */
void OtaCom_FlushRx(void);

/* 关掉串口（可选：BL 跳转前的“彻底清理”用）。
   ⚠ 现在 bl_start_app() **故意不调它** —— 留着 USART1 好打跳转前的最后一行诊断，
     App 的 OtaCom_Init() 反正会把 RCC/GPIO/寄存器全部重配一遍。
   别的场合（比如要彻底关掉无线口）才需要它。 */
void OtaCom_DeInit(void);

#endif /* __OTA_COM_H__ */
