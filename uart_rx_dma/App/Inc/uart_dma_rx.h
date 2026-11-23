/**
 * @file    uart_dma_rx.h
 * @brief   UART7 RX DMA（循环模式）-> 软件环形缓冲。
 *
 * DMA 缓冲每半满触发 HT/TC 中断，中断里把刚写满的那一半搬进 ring。
 * 主循环只从 ring 取数据，不再碰 DMA 寄存器。
 */
#ifndef UART_DMA_RX_H
#define UART_DMA_RX_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "ringbuf.h"

bool     uart_dma_rx_start(UART_HandleTypeDef *huart, ringbuf_t *ring);
uint32_t uart_dma_rx_half_events(void);
uint32_t uart_dma_rx_full_events(void);
uint32_t uart_dma_rx_error_count(void);
uint32_t uart_dma_rx_last_error(void);

#endif /* UART_DMA_RX_H */
