/**
 * @file    frame_parser.c
 * @brief   帧解析状态机实现。
 */
#include "frame_parser.h"
#include "crc32.h"
#include <string.h>

static void parser_start(frame_parser_t *p, uint8_t byte, uint32_t now_ms)
{
    p->state = FP_TYPE;
    p->raw[0] = byte;
    p->raw_len = 1U;
    p->type = 0U;
    p->len = 0U;
    p->payload_pos = 0U;
    p->crc_pos = 0U;
    p->crc_recv = 0U;
    p->last_tick_ms = now_ms;
}

void frame_parser_reset(frame_parser_t *p)
{
    p->state = FP_WAIT_SOF;
    p->raw_len = 0U;
    p->type = 0U;
    p->len = 0U;
    p->payload_pos = 0U;
    p->crc_pos = 0U;
    p->crc_recv = 0U;
}

void frame_parser_init(frame_parser_t *p,
                       frame_parser_frame_cb_t on_frame,
                       frame_parser_error_cb_t on_error,
                       void *ctx,
                       uint32_t timeout_ms)
{
    memset(p, 0, sizeof(*p));
    p->on_frame = on_frame;
    p->on_error = on_error;
    p->ctx = ctx;
    p->timeout_ms = timeout_ms;
    frame_parser_reset(p);
}

static void emit_error(frame_parser_t *p, uint8_t code)
{
    frame_error_t e;
    memset(&e, 0, sizeof(e));
    e.code = code;
    e.type = p->type;
    e.len = p->len;
    e.state = (uint8_t)p->state;
    e.raw_len = (p->raw_len > sizeof(e.raw)) ? (uint8_t)sizeof(e.raw) : p->raw_len;
    memcpy(e.raw, p->raw, e.raw_len);

    p->errors++;
    if (p->on_error != NULL) {
        p->on_error(&e, p->ctx);
    }
}

void frame_parser_feed(frame_parser_t *p, uint8_t byte, uint32_t now_ms)
{
    switch (p->state) {
    case FP_WAIT_SOF:
        if (byte == APP_FRAME_SOF) {
            parser_start(p, byte, now_ms);
        }
        break;

    case FP_TYPE:
        p->raw[p->raw_len++] = byte;
        p->type = byte;
        p->last_tick_ms = now_ms;
        p->state = FP_LEN;
        break;

    case FP_LEN:
        p->raw[p->raw_len++] = byte;
        p->len = byte;
        p->last_tick_ms = now_ms;
        if (p->len > APP_FRAME_MAX_PAYLOAD) {
            emit_error(p, FRAME_ERR_LEN);
            frame_parser_reset(p);
        } else if (p->len == 0U) {
            p->state = FP_CRC;
        } else {
            p->state = FP_PAYLOAD;
        }
        break;

    case FP_PAYLOAD:
        p->raw[p->raw_len++] = byte;
        p->payload[p->payload_pos++] = byte;
        p->last_tick_ms = now_ms;
        if (p->payload_pos >= p->len) {
            p->state = FP_CRC;
        }
        break;

    case FP_CRC:
        p->raw[p->raw_len++] = byte;
        p->crc_recv |= ((uint32_t)byte << (8U * p->crc_pos));
        p->crc_pos++;
        p->last_tick_ms = now_ms;
        if (p->crc_pos >= 4U) {
            uint32_t calc = crc32_calc(&p->raw[1], (size_t)(2U + p->len));
            if (calc != p->crc_recv) {
                emit_error(p, FRAME_ERR_CRC);
                frame_parser_reset(p);
            } else {
                p->state = FP_EOF;
            }
        }
        break;

    case FP_EOF:
        p->raw[p->raw_len++] = byte;
        p->last_tick_ms = now_ms;
        if (byte == APP_FRAME_EOF) {
            frame_t frame;
            frame.type = p->type;
            frame.len = p->len;
            memcpy(frame.payload, p->payload, p->len);
            frame.raw_len = p->raw_len;
            memcpy(frame.raw, p->raw, p->raw_len);
            p->frames_ok++;
            if (p->on_frame != NULL) {
                p->on_frame(&frame, p->ctx);
            }
            frame_parser_reset(p);
        } else {
            emit_error(p, FRAME_ERR_TAIL);
            frame_parser_reset(p);
            if (byte == APP_FRAME_SOF) {
                /* 坏帧尾恰好是下一帧的帧头：立即重新开始，不丢这一帧。 */
                parser_start(p, byte, now_ms);
            }
        }
        break;

    default:
        frame_parser_reset(p);
        break;
    }
}

void frame_parser_tick(frame_parser_t *p, uint32_t now_ms)
{
    if (p->state == FP_WAIT_SOF) {
        return;
    }
    if ((uint32_t)(now_ms - p->last_tick_ms) >= p->timeout_ms) {
        emit_error(p, FRAME_ERR_TIMEOUT);
        frame_parser_reset(p);
    }
}
