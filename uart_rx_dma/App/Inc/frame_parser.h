/**
 * @file    frame_parser.h
 * @brief   字节流 -> 帧 的状态机。
 *
 * 只负责"找帧、验长度、验 CRC32、验帧尾、帧内超时"，
 * 不关心消息类型/业务，通过回调把结果交给上层。
 */
#ifndef FRAME_PARSER_H
#define FRAME_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"

/* 错误码：和 NACK 帧里的 err_code 一一对应 */
typedef enum {
    FRAME_ERR_NONE = 0,
    FRAME_ERR_LEN = 1,          /* LEN > 8 */
    FRAME_ERR_CRC = 2,          /* CRC32 不匹配 */
    FRAME_ERR_TAIL = 3,         /* 帧尾不是 0x5A */
    FRAME_ERR_TIMEOUT = 4,      /* 帧内字节间超时 */
    FRAME_ERR_UNKNOWN_TYPE = 5, /* 分发层：未知消息类型 */
    FRAME_ERR_PAYLOAD_LEN = 6,  /* 分发层：payload 长度不符合该消息 */
    FRAME_ERR_RING_OVERFLOW = 7,/* 软件环形缓冲溢出 */
    FRAME_ERR_UART = 8,         /* UART 硬件错误（ORE/FE/NE/PE） */
    FRAME_ERR_STATS_PAGE = 9    /* 分发层：未知统计页 */
} frame_err_code_t;

typedef struct {
    uint8_t type;
    uint8_t len;
    uint8_t payload[APP_FRAME_MAX_PAYLOAD];
    uint8_t raw[APP_FRAME_MAX_SIZE];
    uint8_t raw_len;
} frame_t;

typedef struct {
    uint8_t code;
    uint8_t type;
    uint8_t len;
    uint8_t state;       /* 出错时解析器所在状态，便于定位 */
    uint8_t raw[4];      /* 出错帧的前 4 个字节（不足补 0） */
    uint8_t raw_len;
} frame_error_t;

typedef void (*frame_parser_frame_cb_t)(const frame_t *frame, void *ctx);
typedef void (*frame_parser_error_cb_t)(const frame_error_t *error, void *ctx);

typedef enum {
    FP_WAIT_SOF = 0,
    FP_TYPE,
    FP_LEN,
    FP_PAYLOAD,
    FP_CRC,
    FP_EOF
} frame_parser_state_t;

typedef struct {
    frame_parser_state_t state;
    uint8_t raw[APP_FRAME_MAX_SIZE];
    uint8_t raw_len;
    uint8_t type;
    uint8_t len;
    uint8_t payload[APP_FRAME_MAX_PAYLOAD];
    uint8_t payload_pos;
    uint8_t crc_pos;
    uint32_t crc_recv;
    uint32_t last_tick_ms;
    uint32_t timeout_ms;
    frame_parser_frame_cb_t on_frame;
    frame_parser_error_cb_t on_error;
    void *ctx;
    uint32_t frames_ok;
    uint32_t errors;
} frame_parser_t;

void frame_parser_init(frame_parser_t *p,
                       frame_parser_frame_cb_t on_frame,
                       frame_parser_error_cb_t on_error,
                       void *ctx,
                       uint32_t timeout_ms);

void frame_parser_feed(frame_parser_t *p, uint8_t byte, uint32_t now_ms);
void frame_parser_tick(frame_parser_t *p, uint32_t now_ms);
void frame_parser_reset(frame_parser_t *p);

#endif /* FRAME_PARSER_H */
