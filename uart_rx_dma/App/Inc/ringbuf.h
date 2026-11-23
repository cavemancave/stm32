/**
 * @file    ringbuf.h
 * @brief   单生产者/单消费者字节环形缓冲（ISR -> 主循环）。
 *
 * 生产者只在 DMA 的 HT/TC 回调里调用 ringbuf_write()；
 * 消费者只在主循环里调用 ringbuf_read()/ringbuf_read_byte()。
 * 大小必须是 2 的幂，head/tail 自由回绕（uint16_t）。
 */
#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t *buf;
    uint16_t size;          /* 2 的幂 */
    uint16_t mask;          /* size - 1 */
    volatile uint16_t head; /* 生产者写 */
    volatile uint16_t tail; /* 消费者写 */
    volatile uint16_t max_used;
    volatile uint32_t dropped; /* 因满而丢弃的字节数（累计） */
} ringbuf_t;

void     ringbuf_init(ringbuf_t *rb, uint8_t *storage, uint16_t size);
uint16_t ringbuf_write(ringbuf_t *rb, const uint8_t *src, uint16_t len);
uint16_t ringbuf_read(ringbuf_t *rb, uint8_t *dst, uint16_t len);
bool     ringbuf_read_byte(ringbuf_t *rb, uint8_t *byte);
uint16_t ringbuf_available(const ringbuf_t *rb);
uint16_t ringbuf_max_used(const ringbuf_t *rb);
uint32_t ringbuf_dropped(const ringbuf_t *rb);
void     ringbuf_clear(ringbuf_t *rb);

#endif /* RINGBUF_H */
