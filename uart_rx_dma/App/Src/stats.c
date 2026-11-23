/**
 * @file    stats.c
 * @brief   统计计数与上报页构造。
 *
 * 统计页（每页 8 字节 payload）：
 *   0: valid_frames, calc_frames, ping_frames
 *   1: crc_err, len_err, tail_err
 *   2: timeout_err, ring_drop_bytes, uart_err
 *   3: proc_avg_us, proc_min_us, proc_max_us
 *   4: tx_frames, unknown_type_err, payload_len_err
 */
#include "stats.h"
#include "messages.h"
#include "bytes.h"

void stats_init(stats_t *s)
{
    *s = (stats_t){0};
    s->proc_min_us = 0xFFFFFFFFU;
}

void stats_note_valid(stats_t *s, uint8_t type)
{
    s->valid_frames++;
    switch (type) {
    case MSG_CALC_SUM_REQ:
    case MSG_CALC_SQSUM_REQ:
    case MSG_CALC_WEIGHTED_REQ:
        s->calc_frames++;
        break;
    case MSG_PING_REQ:
        s->ping_frames++;
        break;
    default:
        break;
    }
}

void stats_note_parse_error(stats_t *s, uint8_t code)
{
    switch (code) {
    case FRAME_ERR_LEN:
        s->len_err++;
        break;
    case FRAME_ERR_CRC:
        s->crc_err++;
        break;
    case FRAME_ERR_TAIL:
        s->tail_err++;
        break;
    case FRAME_ERR_TIMEOUT:
        s->timeout_err++;
        break;
    case FRAME_ERR_UART:
        s->uart_err++;
        break;
    case FRAME_ERR_UNKNOWN_TYPE:
        s->unknown_type_err++;
        break;
    case FRAME_ERR_PAYLOAD_LEN:
        s->payload_len_err++;
        break;
    case FRAME_ERR_STATS_PAGE:
        s->stats_page_err++;
        break;
    default:
        break;
    }
}

void stats_note_proc(stats_t *s, uint32_t us)
{
    s->proc_count++;
    s->proc_sum_us += us;
    if (us < s->proc_min_us) {
        s->proc_min_us = us;
    }
    if (us > s->proc_max_us) {
        s->proc_max_us = us;
    }
}

static void put3(uint8_t out[APP_FRAME_MAX_PAYLOAD], uint8_t page, uint8_t flags,
                 uint32_t v0, uint32_t v1, uint32_t v2)
{
    out[0] = page;
    out[1] = flags;
    wr_u16le(&out[2], sat_u16(v0));
    wr_u16le(&out[4], sat_u16(v1));
    wr_u16le(&out[6], sat_u16(v2));
}

bool stats_build_page(const stats_t *s, uint8_t page, uint8_t out[APP_FRAME_MAX_PAYLOAD])
{
    uint8_t flags = 0U;
    uint32_t v0 = 0U;
    uint32_t v1 = 0U;
    uint32_t v2 = 0U;

    switch (page) {
    case 0U:
        v0 = s->valid_frames;
        v1 = s->calc_frames;
        v2 = s->ping_frames;
        break;
    case 1U:
        v0 = s->crc_err;
        v1 = s->len_err;
        v2 = s->tail_err;
        break;
    case 2U:
        v0 = s->timeout_err;
        v1 = s->ring_drop_bytes;
        v2 = s->uart_err;
        break;
    case 3U: {
        uint32_t avg = (s->proc_count != 0U) ? (s->proc_sum_us / s->proc_count) : 0U;
        uint32_t min = (s->proc_count != 0U) ? s->proc_min_us : 0U;
        v0 = avg;
        v1 = min;
        v2 = s->proc_max_us;
        break;
    }
    case 4U:
        v0 = s->tx_frames;
        v1 = s->unknown_type_err;
        v2 = s->payload_len_err;
        break;
    default:
        return false;
    }

    if (v0 > 0xFFFFU || v1 > 0xFFFFU || v2 > 0xFFFFU) {
        flags |= 0x01U; /* 至少一个计数器饱和 */
    }
    put3(out, page, flags, v0, v1, v2);
    return true;
}
