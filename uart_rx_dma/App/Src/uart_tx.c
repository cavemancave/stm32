/**
 * @file    uart_tx.c
 * @brief   USART1 TX DMA 发送队列实现。
 *
 * 数据流：
 *   协议层 proto_encode() -> uart_tx_write() -> 软件 FIFO
 *   uart_tx_poll() -> 拷贝到 s_tx_dma_buf（RAM_D2）-> HAL_UART_Transmit_DMA()
 *   DMA 完成中断 -> HAL_UART_TxCpltCallback() -> 清 busy
 *
 * 注意：DMA 不能访问 DTCM，s_tx_dma_buf 必须放 .dma_buffer（RAM_D2）。
 */
#include "uart_tx.h"
#include "app_config.h"
#include "ringbuf.h"

static uint8_t s_tx_ring_mem[APP_TX_RING_SIZE];
static ringbuf_t s_tx_ring;

/* DMA 可见内存：见 STM32H723xG_flash.ld 的 .dma_buffer (RAM_D2)。 */
static uint8_t s_tx_dma_buf[APP_TX_DMA_BUF_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));

static UART_HandleTypeDef *s_huart;
static volatile uint8_t s_busy;
static volatile uint16_t s_tx_len;
static volatile uint32_t s_dropped_bytes;
static volatile uint32_t s_error_count;

void uart_tx_init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    ringbuf_init(&s_tx_ring, s_tx_ring_mem, APP_TX_RING_SIZE);
    s_busy = 0U;
    s_tx_len = 0U;
    s_dropped_bytes = 0U;
    s_error_count = 0U;
}

bool uart_tx_write(const uint8_t *data, uint16_t len)
{
    if ((s_huart == NULL) || (data == NULL) || (len == 0U)) {
        return false;
    }
    /* 整帧原子写入：空间不够就整帧丢弃，不能写半帧把流打乱。 */
    if ((uint32_t)ringbuf_available(&s_tx_ring) + (uint32_t)len > APP_TX_RING_SIZE) {
        s_dropped_bytes += len;
        return false;
    }
    return ringbuf_write(&s_tx_ring, data, len) == len;
}

void uart_tx_poll(void)
{
    if ((s_huart == NULL) || (s_busy != 0U)) {
        return;
    }

    if (s_tx_len == 0U) {
        s_tx_len = ringbuf_read(&s_tx_ring, s_tx_dma_buf, (uint16_t)sizeof(s_tx_dma_buf));
        if (s_tx_len == 0U) {
            return;
        }
    }

    /* 上一次启动失败时 s_tx_len 保留，下次继续重试，不丢数据。
     * HAL_BUSY 只是临时状态，不算错误；HAL_ERROR 才计数。 */
    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(s_huart, s_tx_dma_buf, s_tx_len);
    if (st == HAL_OK) {
        s_busy = 1U;
    } else if (st != HAL_BUSY) {
        s_error_count++;
    }
}

void uart_tx_on_error(UART_HandleTypeDef *huart)
{
    if ((s_huart == NULL) || (huart != s_huart)) {
        return;
    }
    /* RX 错误也可能走到 HAL_UART_ErrorCallback；只有 TX 已不在 BUSY 状态
     * 时才清 busy，避免 DMA 还在读暂存区时被覆盖。 */
    if (huart->gState != HAL_UART_STATE_BUSY_TX) {
        s_busy = 0U;
        s_tx_len = 0U;
        s_error_count++;
    }
}

uint32_t uart_tx_dropped_bytes(void)
{
    return s_dropped_bytes;
}

uint32_t uart_tx_error_count(void)
{
    return s_error_count;
}

uint16_t uart_tx_pending(void)
{
    return ringbuf_available(&s_tx_ring);
}

bool uart_tx_busy(void)
{
    return s_busy != 0U;
}

/* ---- HAL 回调（覆盖 weak 定义） ------------------------------------------ */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_huart) {
        s_tx_len = 0U;
        s_busy = 0U;
    }
}
