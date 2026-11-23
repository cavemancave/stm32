/**
 * @file    stats.h
 * @brief   运行统计：收帧、错误、处理时间、发送帧数。
 *
 * 所有计数用 uint32_t 累加，上报时饱和到 uint16_t（页里放得下）。
 */
#ifndef STATS_H
#define STATS_H

#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"
#include "frame_parser.h"

typedef struct {
    uint32_t valid_frames;      /* 通过 CRC/帧尾校验的帧总数 */
    uint32_t calc_frames;       /* 其中计算类 */
    uint32_t ping_frames;       /* 其中 ping */

    uint32_t crc_err;
    uint32_t len_err;           /* LEN 字段 > 8 */
    uint32_t tail_err;
    uint32_t timeout_err;
    uint32_t uart_err;
    uint32_t unknown_type_err;
    uint32_t payload_len_err;
    uint32_t stats_page_err;

    uint32_t tx_frames;
    uint32_t tx_err;
    uint32_t ring_drop_bytes;

    uint32_t proc_count;
    uint32_t proc_sum_us;
    uint32_t proc_min_us;
    uint32_t proc_max_us;
} stats_t;

void stats_init(stats_t *s);
void stats_note_valid(stats_t *s, uint8_t type);
void stats_note_parse_error(stats_t *s, uint8_t code);
void stats_note_proc(stats_t *s, uint32_t us);

/* page 0..4；成功写入 8 字节 payload 返回 true，未知页返回 false。 */
bool stats_build_page(const stats_t *s, uint8_t page, uint8_t out[APP_FRAME_MAX_PAYLOAD]);

#endif /* STATS_H */
