/**
 * @file    uart_dma_rx.h
 * @brief   UART RX DMA（循环 + IDLE）-> 软件环形缓冲。
 *
 * 用 HAL_UARTEx_ReceiveToIdle_DMA：一帧结束（IDLE）立即搬进 ring，
 * 粘包/高速时由 HT/TC 事件兜底；主循环只从 ring 取数据。
 */
#ifndef UART_DMA_RX_H
#define UART_DMA_RX_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "ringbuf.h"

bool     uart_dma_rx_start(UART_HandleTypeDef *huart, ringbuf_t *ring);
uint32_t uart_dma_rx_events(void);
uint32_t uart_dma_rx_error_count(void);
uint32_t uart_dma_rx_last_error(void);

#endif /* UART_DMA_RX_H */
