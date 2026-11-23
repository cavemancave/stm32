/**
 * @file    protocol.c
 * @brief   帧编码 + 3 个计算消息 + ping + stats + NACK。
 */
#include "protocol.h"
#include "messages.h"
#include "bytes.h"
#include "crc32.h"
#include "timebase.h"
#include <string.h>

typedef uint8_t (*proto_handler_t)(const frame_t *req, proto_msg_t *rsp, protocol_ctx_t *ctx);

typedef struct {
    uint8_t type;
    uint8_t min_len;
    uint8_t max_len;
    proto_handler_t handler;
} proto_desc_t;

/* -------------------------------------------------------------------------- */
/* 帧编码                                                                     */
/* -------------------------------------------------------------------------- */
uint8_t proto_encode(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out)
{
    if ((out == NULL) || (len > APP_FRAME_MAX_PAYLOAD) || ((payload == NULL) && (len != 0U))) {
        return 0U;
    }

    out[0] = APP_FRAME_SOF;
    out[1] = type;
    out[2] = len;
    if (len != 0U) {
        memcpy(&out[3], payload, len);
    }

    uint32_t crc = crc32_calc(&out[1], (size_t)(2U + len));
    wr_u32le(&out[3U + len], crc);
    out[7U + len] = APP_FRAME_EOF;
    return (uint8_t)(8U + len);
}

void protocol_build_error(proto_msg_t *msg, uint8_t code,
                          uint8_t type, uint8_t len, uint8_t state,
                          const uint8_t raw[4])
{
    msg->type = MSG_NACK;
    msg->len = APP_FRAME_MAX_PAYLOAD;
    msg->payload[0] = code;
    msg->payload[1] = type;
    msg->payload[2] = len;
    msg->payload[3] = state;
    memcpy(&msg->payload[4], raw, 4U);
}

static void build_nack(proto_msg_t *rsp, uint8_t code, const frame_t *req)
{
    uint8_t raw[4] = {0U, 0U, 0U, 0U};
    uint8_t n = (req->raw_len > sizeof(raw)) ? (uint8_t)sizeof(raw) : req->raw_len;
    if (n != 0U) {
        memcpy(raw, req->raw, n);
    }
    protocol_build_error(rsp, code, req->type, req->len, 0U, raw);
}

/* -------------------------------------------------------------------------- */
/* 计算消息                                                                   */
/* -------------------------------------------------------------------------- */
static uint8_t calc_common(const frame_t *req, proto_msg_t *rsp, uint8_t rsp_type, uint8_t op)
{
    int16_t a = (int16_t)(int32_t)rd_u16le(&req->payload[0]);
    int16_t b = (int16_t)(int32_t)rd_u16le(&req->payload[2]);
    int16_t c = (int16_t)(int32_t)rd_u16le(&req->payload[4]);

    int64_t result = 0;
    switch (op) {
    case 0U:
        result = (int64_t)a + (int64_t)b + (int64_t)c;
        break;
    case 1U:
        result = (int64_t)a * (int64_t)a
               + (int64_t)b * (int64_t)b
               + (int64_t)c * (int64_t)c;
        break;
    default:
        result = (int64_t)a * 1 + (int64_t)b * 2 + (int64_t)c * 3;
        break;
    }

    bool clamped = false;
    if (result > INT32_MAX) {
        result = INT32_MAX;
        clamped = true;
    } else if (result < INT32_MIN) {
        result = INT32_MIN;
        clamped = true;
    }

    rsp->type = rsp_type;
    rsp->len = APP_FRAME_MAX_PAYLOAD;
    wr_u32le(&rsp->payload[0], (uint32_t)(int32_t)result);
    wr_u16le(&rsp->payload[4], 0U); /* 处理时间由 protocol_handle 回填 */
    rsp->payload[6] = clamped ? 1U : 0U;
    rsp->payload[7] = 0U;
    return 0U;
}

static uint8_t handle_sum(const frame_t *req, proto_msg_t *rsp, protocol_ctx_t *ctx)
{
    (void)ctx;
    return calc_common(req, rsp, MSG_CALC_SUM_RSP, 0U);
}

static uint8_t handle_sqsum(const frame_t *req, proto_msg_t *rsp, protocol_ctx_t *ctx)
{
    (void)ctx;
    return calc_common(req, rsp, MSG_CALC_SQSUM_RSP, 1U);
}

static uint8_t handle_weighted(const frame_t *req, proto_msg_t *rsp, protocol_ctx_t *ctx)
{
    (void)ctx;
    return calc_common(req, rsp, MSG_CALC_WEIGHTED_RSP, 2U);
}

/* -------------------------------------------------------------------------- */
/* PING / STATS                                                               */
/* -------------------------------------------------------------------------- */
static uint8_t handle_ping(const frame_t *req, proto_msg_t *rsp, protocol_ctx_t *ctx)
{
    (void)req;
    rsp->type = MSG_PONG_RSP;
    rsp->len = APP_FRAME_MAX_PAYLOAD;
    wr_u32le(&rsp->payload[0], (uint32_t)(ctx->now_ms - ctx->boot_ms));
    wr_u16le(&rsp->payload[4], 0U); /* 处理时间回填 */
    wr_u16le(&rsp->payload[6], sat_u16(ctx->stats->valid_frames));
    return 0U;
}

static uint8_t handle_stats(const frame_t *req, proto_msg_t *rsp, protocol_ctx_t *ctx)
{
    uint8_t page = (req->len >= 1U) ? req->payload[0] : 0U;
    if (!stats_build_page(ctx->stats, page, rsp->payload)) {
        return PROTO_ERR_STATS_PAGE;
    }
    rsp->type = MSG_STATS_RSP;
    rsp->len = APP_FRAME_MAX_PAYLOAD;
    return 0U;
}

/* -------------------------------------------------------------------------- */
/* 分发表                                                                     */
/* -------------------------------------------------------------------------- */
static const proto_desc_t s_descs[] = {
    { MSG_CALC_SUM_REQ,      6U, 6U, handle_sum      },
    { MSG_CALC_SQSUM_REQ,    6U, 6U, handle_sqsum    },
    { MSG_CALC_WEIGHTED_REQ, 6U, 6U, handle_weighted },
    { MSG_PING_REQ,          0U, 0U, handle_ping     },
    { MSG_STATS_REQ,         0U, 1U, handle_stats    },
};

static const proto_desc_t *find_desc(uint8_t type)
{
    for (size_t i = 0U; i < (sizeof(s_descs) / sizeof(s_descs[0])); ++i) {
        if (s_descs[i].type == type) {
            return &s_descs[i];
        }
    }
    return NULL;
}

bool protocol_handle(const frame_t *req, proto_msg_t *rsp, protocol_ctx_t *ctx)
{
    uint32_t start = timebase_cycles();
    const proto_desc_t *desc = find_desc(req->type);
    uint8_t err = 0U;

    if (desc == NULL) {
        build_nack(rsp, FRAME_ERR_UNKNOWN_TYPE, req);
        stats_note_parse_error(ctx->stats, FRAME_ERR_UNKNOWN_TYPE);
    } else if ((req->len < desc->min_len) || (req->len > desc->max_len)) {
        build_nack(rsp, FRAME_ERR_PAYLOAD_LEN, req);
        stats_note_parse_error(ctx->stats, FRAME_ERR_PAYLOAD_LEN);
    } else {
        err = desc->handler(req, rsp, ctx);
        if (err != 0U) {
            build_nack(rsp, err, req);
            stats_note_parse_error(ctx->stats, err);
        } else {
            stats_note_valid(ctx->stats, req->type);
        }
    }

    uint32_t us = timebase_cycles_to_us(timebase_cycles() - start);
    stats_note_proc(ctx->stats, us);

    if ((rsp->type == MSG_CALC_SUM_RSP) ||
        (rsp->type == MSG_CALC_SQSUM_RSP) ||
        (rsp->type == MSG_CALC_WEIGHTED_RSP) ||
        (rsp->type == MSG_PONG_RSP)) {
        wr_u16le(&rsp->payload[4], sat_u16(us));
    }
    return true;
}
