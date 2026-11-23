/**
 * @file    ringbuf.c
 * @brief   SPSC 字节环形缓冲。
 *
 * 关键点：
 *  - head/tail 是自由回绕的 uint16_t，size 必须整除 65536（2 的幂即可）；
 *  - 生产者写完数据后 __DMB() 再发布 head，消费者看到 head 时数据一定可见；
 *  - 消费者更新 tail 前 __DMB()，避免编译器/CPU 提前释放空间；
 *  - 满时丢弃新数据并累计 dropped，绝不覆盖未读数据。
 */
#include "ringbuf.h"
#include "main.h"
#include <string.h>

void ringbuf_init(ringbuf_t *rb, uint8_t *storage, uint16_t size)
{
    rb->buf = storage;
    rb->size = size;
    rb->mask = (uint16_t)(size - 1U);
    rb->head = 0U;
    rb->tail = 0U;
    rb->max_used = 0U;
    rb->dropped = 0U;
}

uint16_t ringbuf_write(ringbuf_t *rb, const uint8_t *src, uint16_t len)
{
    uint16_t head = rb->head;
    uint16_t tail = rb->tail;
    uint16_t used = (uint16_t)(head - tail);
    uint16_t space;

    if (used > rb->size) {
        /* 只可能是外部破坏；按满处理，避免无符号下溢成巨大空间 */
        used = rb->size;
    }
    space = (uint16_t)(rb->size - used);

    if (len > space) {
        rb->dropped += (uint32_t)(len - space);
        len = space;
    }
    if (len == 0U) {
        return 0U;
    }

    uint16_t offset = (uint16_t)(head & rb->mask);
    uint16_t first = (uint16_t)(rb->size - offset);
    if (first > len) {
        first = len;
    }
    memcpy(&rb->buf[offset], src, first);
    if (len > first) {
        memcpy(&rb->buf[0], &src[first], (size_t)(len - first));
    }

    __DMB();
    rb->head = (uint16_t)(head + len);

    uint16_t new_used = (uint16_t)(rb->head - rb->tail);
    if (new_used > rb->max_used) {
        rb->max_used = new_used;
    }
    return len;
}

uint16_t ringbuf_read(ringbuf_t *rb, uint8_t *dst, uint16_t len)
{
    uint16_t tail = rb->tail;
    uint16_t head = rb->head;
    uint16_t used = (uint16_t)(head - tail);

    if (used > rb->size) {
        used = rb->size;
    }
    if (len > used) {
        len = used;
    }
    if (len == 0U) {
        return 0U;
    }

    uint16_t offset = (uint16_t)(tail & rb->mask);
    uint16_t first = (uint16_t)(rb->size - offset);
    if (first > len) {
        first = len;
    }
    memcpy(&dst[0], &rb->buf[offset], first);
    if (len > first) {
        memcpy(&dst[first], &rb->buf[0], (size_t)(len - first));
    }

    __DMB();
    rb->tail = (uint16_t)(tail + len);
    return len;
}

bool ringbuf_read_byte(ringbuf_t *rb, uint8_t *byte)
{
    uint16_t tail = rb->tail;
    if (tail == rb->head) {
        return false;
    }
    *byte = rb->buf[tail & rb->mask];
    __DMB();
    rb->tail = (uint16_t)(tail + 1U);
    return true;
}

uint16_t ringbuf_available(const ringbuf_t *rb)
{
    uint16_t used = (uint16_t)(rb->head - rb->tail);
    if (used > rb->size) {
        used = rb->size;
    }
    return used;
}

uint16_t ringbuf_max_used(const ringbuf_t *rb)
{
    return rb->max_used;
}

uint32_t ringbuf_dropped(const ringbuf_t *rb)
{
    return rb->dropped;
}

void ringbuf_clear(ringbuf_t *rb)
{
    __DMB();
    rb->tail = rb->head;
}
