/**
 * @file    app.c
 * @brief   应用胶水层：初始化、轮询、回包、错误上报。
 */
#include "app.h"
#include "app_config.h"
#include "ringbuf.h"
#include "frame_parser.h"
#include "uart_dma_rx.h"
#include "protocol.h"
#include "stats.h"
#include "timebase.h"
#include "bytes.h"
#include "messages.h"
#include "usart.h"
#include "main.h"
#include <string.h>

static uint8_t s_ring_mem[APP_RX_RING_SIZE];
static ringbuf_t s_ring;
static frame_parser_t s_parser;
static stats_t s_stats;

static uint32_t s_boot_ms;
static uint32_t s_reported_drops;
static uint32_t s_reported_uart_errors;
static uint32_t s_last_drop_report_ms;
static uint32_t s_last_uart_report_ms;
static uint32_t s_last_heartbeat_ms;

/* -------------------------------------------------------------------------- */
/* 发送                                                                       */
/* -------------------------------------------------------------------------- */
static bool app_send_msg(const proto_msg_t *msg)
{
    uint8_t buf[APP_FRAME_MAX_SIZE];
    uint8_t n = proto_encode(msg->type, msg->payload, msg->len, buf);
    if (n == 0U) {
        return false;
    }
    if (HAL_UART_Transmit(&huart1, buf, n, 100U) != HAL_OK) {
        s_stats.tx_err++;
        return false;
    }
    s_stats.tx_frames++;
    return true;
}

/* -------------------------------------------------------------------------- */
/* 解析器回调                                                                 */
/* -------------------------------------------------------------------------- */
static void app_on_frame(const frame_t *frame, void *ctx)
{
    (void)ctx;
    protocol_ctx_t pctx;
    pctx.stats = &s_stats;
    pctx.boot_ms = s_boot_ms;
    pctx.now_ms = HAL_GetTick();

    proto_msg_t rsp;
    memset(&rsp, 0, sizeof(rsp));
    if (protocol_handle(frame, &rsp, &pctx)) {
        (void)app_send_msg(&rsp);
    }
}

static void app_on_parser_error(const frame_error_t *error, void *ctx)
{
    (void)ctx;
    uint8_t raw[4] = {0U, 0U, 0U, 0U};
    uint8_t n = (error->raw_len > sizeof(raw)) ? (uint8_t)sizeof(raw) : error->raw_len;
    if (n != 0U) {
        memcpy(raw, error->raw, n);
    }

    proto_msg_t msg;
    protocol_build_error(&msg, error->code, error->type, error->len, error->state, raw);
    (void)app_send_msg(&msg);
    stats_note_parse_error(&s_stats, error->code);
}

/* -------------------------------------------------------------------------- */
/* 环形缓冲 / UART 错误上报                                                   */
/* -------------------------------------------------------------------------- */
static void app_report_ring_overflow(uint32_t now_ms)
{
    uint32_t dropped = ringbuf_dropped(&s_ring);
    if (dropped == s_reported_drops) {
        return;
    }
    if ((uint32_t)(now_ms - s_last_drop_report_ms) < 100U) {
        return; /* 最多 10 Hz，避免溢出时刷屏 */
    }
    s_last_drop_report_ms = now_ms;
    s_reported_drops = dropped;
    s_stats.ring_drop_bytes = dropped;

    uint8_t raw[4];
    wr_u32le(raw, dropped);
    proto_msg_t msg;
    protocol_build_error(&msg, FRAME_ERR_RING_OVERFLOW, 0U, 0U, 0U, raw);
    (void)app_send_msg(&msg);
}

static void app_report_uart_error(uint32_t now_ms)
{
    uint32_t errors = uart_dma_rx_error_count();
    if (errors == s_reported_uart_errors) {
        return;
    }
    if ((uint32_t)(now_ms - s_last_uart_report_ms) < 100U) {
        return;
    }
    s_last_uart_report_ms = now_ms;
    s_reported_uart_errors = errors;
    s_stats.uart_err = errors;

    uint8_t raw[4];
    wr_u32le(raw, uart_dma_rx_last_error());
    proto_msg_t msg;
    protocol_build_error(&msg, FRAME_ERR_UART, 0U, 0U, 0U, raw);
    (void)app_send_msg(&msg);
}

/* -------------------------------------------------------------------------- */
/* 初始化 / 轮询                                                              */
/* -------------------------------------------------------------------------- */
static void app_send_ready(void)
{
    proto_msg_t msg;
    msg.type = MSG_READY;
    msg.len = APP_FRAME_MAX_PAYLOAD;
    wr_u16le(&msg.payload[0], APP_FW_VERSION);
    msg.payload[2] = APP_FRAME_MAX_PAYLOAD;
    wr_u16le(&msg.payload[3], APP_RX_DMA_BUF_SIZE);
    wr_u16le(&msg.payload[5], APP_RX_RING_SIZE);
    msg.payload[7] = APP_FRAME_TIMEOUT_MS;
    (void)app_send_msg(&msg);
}

static void app_send_heartbeat(uint32_t now_ms)
{
    proto_msg_t msg;
    msg.type = MSG_HEARTBEAT;
    msg.len = APP_FRAME_MAX_PAYLOAD;
    wr_u32le(&msg.payload[0], (uint32_t)(now_ms - s_boot_ms));
    wr_u16le(&msg.payload[4], sat_u16(s_stats.valid_frames));
    wr_u16le(&msg.payload[6], sat_u16(s_stats.tx_frames));
    (void)app_send_msg(&msg);
}

void app_init(void)
{
    timebase_init();
    stats_init(&s_stats);
    ringbuf_init(&s_ring, s_ring_mem, APP_RX_RING_SIZE);

    frame_parser_init(&s_parser,
                      app_on_frame,
                      app_on_parser_error,
                      NULL,
                      APP_FRAME_TIMEOUT_MS);

    s_boot_ms = HAL_GetTick();
    s_reported_drops = 0U;
    s_reported_uart_errors = 0U;
    s_last_drop_report_ms = 0U;
    s_last_uart_report_ms = 0U;
    s_last_heartbeat_ms = HAL_GetTick();

    if (!uart_dma_rx_start(&huart1, &s_ring)) {
        Error_Handler();
    }

    app_send_ready();
}

void app_poll(void)
{
    uint8_t chunk[APP_POLL_CHUNK];
    uint32_t now_ms = HAL_GetTick();

    uint16_t n = ringbuf_read(&s_ring, chunk, (uint16_t)sizeof(chunk));
    for (uint16_t i = 0U; i < n; ++i) {
        frame_parser_feed(&s_parser, chunk[i], now_ms);
    }

    /* 用新的 tick 做超时判断：上面的 feed 可能已经花了几毫秒。 */
    frame_parser_tick(&s_parser, HAL_GetTick());

    now_ms = HAL_GetTick();
    app_report_ring_overflow(now_ms);
    app_report_uart_error(now_ms);

    if ((uint32_t)(now_ms - s_last_heartbeat_ms) >= 1000U) {
        s_last_heartbeat_ms = now_ms;
        app_send_heartbeat(now_ms);
    }
}
