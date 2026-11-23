/**
 * @file    uart_dma_rx.c
 * @brief   UART RX DMA 循环接收 + 半满/全满中断搬运到环形缓冲。
 *
 * 为什么 DMA 缓冲必须放 .dma_buffer：
 *   STM32H7 的 DMA1/DMA2 不能访问 DTCM（0x2000_0000），而本工程的 .bss
 *   默认链接到 DTCM。DMA 缓冲放到 RAM_D2（0x3000_0000）的 .dma_buffer 段，
 *   由链接脚本单独放置；软件 ring 仍在 DTCM，CPU 访问更快。
 */
#include "uart_dma_rx.h"
#include "app_config.h"

/* DMA 可见内存：见 STM32H723xG_flash.ld 的 .dma_buffer (RAM_D2)。 */
static uint8_t s_dma_buf[APP_RX_DMA_BUF_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));

typedef struct {
    UART_HandleTypeDef *huart;
    ringbuf_t *ring;
    volatile uint32_t half_events;
    volatile uint32_t full_events;
    volatile uint32_t error_count;
    volatile uint32_t last_error;
    volatile bool started;
} uart_dma_rx_ctx_t;

static uart_dma_rx_ctx_t s_ctx;

static void rx_half_done(void)
{
    s_ctx.half_events++;
    (void)ringbuf_write(s_ctx.ring, s_dma_buf, (uint16_t)(APP_RX_DMA_BUF_SIZE / 2U));
}

static void rx_full_done(void)
{
    s_ctx.full_events++;
    (void)ringbuf_write(s_ctx.ring,
                        &s_dma_buf[APP_RX_DMA_BUF_SIZE / 2U],
                        (uint16_t)(APP_RX_DMA_BUF_SIZE / 2U));
}

bool uart_dma_rx_start(UART_HandleTypeDef *huart, ringbuf_t *ring)
{
    s_ctx.huart = huart;
    s_ctx.ring = ring;
    s_ctx.half_events = 0U;
    s_ctx.full_events = 0U;
    s_ctx.error_count = 0U;
    s_ctx.last_error = 0U;
    s_ctx.started = false;

    if (HAL_UART_Receive_DMA(huart, s_dma_buf, (uint16_t)sizeof(s_dma_buf)) != HAL_OK) {
        return false;
    }
    s_ctx.started = true;
    return true;
}

uint32_t uart_dma_rx_half_events(void)
{
    return s_ctx.half_events;
}

uint32_t uart_dma_rx_full_events(void)
{
    return s_ctx.full_events;
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

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if ((s_ctx.started) && (huart == s_ctx.huart)) {
        rx_half_done();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((s_ctx.started) && (huart == s_ctx.huart)) {
        rx_full_done();
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
            (void)HAL_UART_Receive_DMA(huart, s_dma_buf, (uint16_t)sizeof(s_dma_buf));
        }
    }
}
