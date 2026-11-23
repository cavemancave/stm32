/**
 * @file    uart_tx.h
 * @brief   USART1 TX DMA 发送队列。
 *
 * 主循环把完整帧写进软件 FIFO，uart_tx_poll() 在 DMA 空闲时把 FIFO 里的
 * 数据搬到 DMA 暂存区并发起 HAL_UART_Transmit_DMA()；DMA 完成中断只清
 * busy 标志。这样解析器不再被阻塞发送拖住。
 */
#ifndef UART_TX_H
#define UART_TX_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

void     uart_tx_init(UART_HandleTypeDef *huart);
bool     uart_tx_write(const uint8_t *data, uint16_t len);
void     uart_tx_poll(void);
void     uart_tx_on_error(UART_HandleTypeDef *huart);

uint32_t uart_tx_dropped_bytes(void);
uint32_t uart_tx_error_count(void);
uint16_t uart_tx_pending(void);
bool     uart_tx_busy(void);

#endif /* UART_TX_H */
