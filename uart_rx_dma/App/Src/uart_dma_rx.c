/**
 * @file    uart_dma_rx.c
 * @brief   UART RX DMA 循环接收 + IDLE/HT/TC 事件搬运到环形缓冲。
 *
 * 为什么不用"半满/全满中断搬一半"：
 *   请求-应答式的小帧（8~16 字节）要等 DMA 攒满 512 字节才会进 ring，
 *   单帧会在 DMA 缓冲里躺很久，表现为"收不到"。
 *   这里改用 HAL_UARTEx_ReceiveToIdle_DMA：
 *     - 一帧结束、线路空闲 -> IDLE 事件，立刻把新字节搬进 ring；
 *     - 连续粘包/高速流 -> HT/TC 事件每 512 字节搬一次；
 *   两条路径都走 HAL_UARTEx_RxEventCallback，Size 是 DMA 缓冲内的位置。
 *
 * DMA 缓冲必须放 .dma_buffer（RAM_D2）：
 *   STM32H7 的 DMA1/DMA2 不能访问 DTCM，而 .bss 默认在 DTCM。
 *   见 STM32H723xG_flash.ld。
 */
#include "uart_dma_rx.h"
#include "app_config.h"

/* DMA 可见内存：见 STM32H723xG_flash.ld 的 .dma_buffer (RAM_D2)。 */
static uint8_t s_dma_buf[APP_RX_DMA_BUF_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));

typedef struct {
    UART_HandleTypeDef *huart;
    ringbuf_t *ring;
    uint16_t last_pos;
    volatile uint32_t rx_events;
    volatile uint32_t error_count;
    volatile uint32_t last_error;
    volatile bool started;
} uart_dma_rx_ctx_t;

static uart_dma_rx_ctx_t s_ctx;

/* 把 DMA 缓冲中 [last_pos, pos) 的新字节搬进 ring；pos 可以等于 size（回绕点）。 */
static void rx_copy_to(uint16_t pos)
{
    uint16_t last = s_ctx.last_pos;
    uint16_t size = (uint16_t)APP_RX_DMA_BUF_SIZE;

    if (pos > size) {
        pos = size;
    }
    if (pos == size) {
        if (last < size) {
            (void)ringbuf_write(s_ctx.ring, &s_dma_buf[last], (uint16_t)(size - last));
        }
        s_ctx.last_pos = 0U;
        return;
    }

    if (pos >= last) {
        if (pos > last) {
            (void)ringbuf_write(s_ctx.ring, &s_dma_buf[last], (uint16_t)(pos - last));
        }
    } else {
        if (last < size) {
            (void)ringbuf_write(s_ctx.ring, &s_dma_buf[last], (uint16_t)(size - last));
        }
        if (pos > 0U) {
            (void)ringbuf_write(s_ctx.ring, &s_dma_buf[0], pos);
        }
    }
    s_ctx.last_pos = pos;
}

bool uart_dma_rx_start(UART_HandleTypeDef *huart, ringbuf_t *ring)
{
    s_ctx.huart = huart;
    s_ctx.ring = ring;
    s_ctx.last_pos = 0U;
    s_ctx.rx_events = 0U;
    s_ctx.error_count = 0U;
    s_ctx.last_error = 0U;
    s_ctx.started = false;

    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, s_dma_buf, (uint16_t)sizeof(s_dma_buf)) != HAL_OK) {
        return false;
    }
    s_ctx.started = true;
    return true;
}

uint32_t uart_dma_rx_events(void)
{
    return s_ctx.rx_events;
}

uint32_t uart_dma_rx_error_count(void)
{
    return s_ctx.error_count;
}

uint32_t uart_dma_rx_last_error(void)
{
    return s_ctx.last_error;
}

/* ---- HAL 回调（覆盖 weak 定义） ------------------------------------------ */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if ((s_ctx.started) && (huart == s_ctx.huart)) {
        s_ctx.rx_events++;
        rx_copy_to(Size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((s_ctx.started) && (huart == s_ctx.huart)) {
        s_ctx.error_count++;
        s_ctx.last_error = huart->ErrorCode;

        /* ORE/FE/NE 等错误会让 HAL 停掉 RX DMA；如果已经回到 READY，重启。
         * 非致命错误（例如 PE）HAL 仍保持 BUSY_RX，此时不能重复启动。 */
        if (huart->RxState == HAL_UART_STATE_READY) {
            s_ctx.last_pos = 0U;
            (void)HAL_UARTEx_ReceiveToIdle_DMA(huart, s_dma_buf, (uint16_t)sizeof(s_dma_buf));
        }
    }
}
