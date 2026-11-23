/**
 * @file    protocol.h
 * @brief   帧格式编解码 + 消息分发表 + 响应/错误帧构造。
 *
 * 这一层不直接碰 UART：协议处理只负责把请求变成响应（或 NACK），
 * 发送由 app.c 统一完成。
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"
#include "frame_parser.h"
#include "stats.h"
#include "messages.h"

/* NACK err_code（与 frame_err_code_t 对齐，另加分发层错误） */
#define PROTO_ERR_PAYLOAD_LEN   FRAME_ERR_PAYLOAD_LEN
#define PROTO_ERR_STATS_PAGE    FRAME_ERR_STATS_PAGE

typedef struct {
    uint8_t type;
    uint8_t len;
    uint8_t payload[APP_FRAME_MAX_PAYLOAD];
} proto_msg_t;

typedef struct {
    stats_t *stats;
    uint32_t boot_ms;
    uint32_t now_ms;
} protocol_ctx_t;

/* 把 type/payload 编成完整帧；返回帧长（8+len），参数非法返回 0。 */
uint8_t proto_encode(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out);

/* 处理一个合法帧，总是产出 1 条响应（正常响应或 NACK）。 */
bool protocol_handle(const frame_t *req, proto_msg_t *rsp, protocol_ctx_t *ctx);

/* 解析层错误 -> NACK；raw 固定 4 字节（不足由调用方补 0）。 */
void protocol_build_error(proto_msg_t *msg, uint8_t code,
                          uint8_t type, uint8_t len, uint8_t state,
                          const uint8_t raw[4]);

#endif /* PROTOCOL_H */
