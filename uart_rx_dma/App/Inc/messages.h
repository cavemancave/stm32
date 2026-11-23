/**
 * @file    messages.h
 * @brief   消息类型编号（请求/响应/通知）。
 */
#ifndef MESSAGES_H
#define MESSAGES_H

/* 请求类型 */
#define MSG_CALC_SUM_REQ        0x01U
#define MSG_CALC_SQSUM_REQ      0x02U
#define MSG_CALC_WEIGHTED_REQ   0x03U
#define MSG_PING_REQ            0x10U
#define MSG_STATS_REQ           0x11U

/* 响应类型 */
#define MSG_CALC_SUM_RSP        0x81U
#define MSG_CALC_SQSUM_RSP      0x82U
#define MSG_CALC_WEIGHTED_RSP   0x83U
#define MSG_PONG_RSP            0x90U
#define MSG_STATS_RSP           0x91U

/* 主动/异步通知 */
#define MSG_READY               0x7EU
#define MSG_HEARTBEAT           0x7DU
#define MSG_NACK                0xEEU

#endif /* MESSAGES_H */
