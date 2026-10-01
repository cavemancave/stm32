/**
  ******************************************************************************
  * @file    ota_link.h
  * @brief   无线 OTA 的链路层：`0x00 + COBS(帧) + 0x00`
  *
  * 帧（COBS 编码之前）：
  *   0xAA 0x55 │ VER │ TYPE │ SEQ │ LEN(2,LE) │ PAYLOAD[LEN] │ CRC32(4,LE)
  *              └──────────── CRC32 覆盖 VER..PAYLOAD ───────────┘
  *
  * 为什么用 0x00 定界 + COBS：
  *   - 串口上还跑着**裸文本日志**（ASCII，永远不会出现 0x00），
  *     COBS 编码后的帧里也永远不出现 0x00 —— 主机只要看到两个 0x00 之间的一段
  *     就是"候选帧"，解出来验 CRC32 就知道是不是；不是就当成日志文本丢掉。
  *   - 丢一个字节也不会一直错位：下一个 0x00 就重新对齐了。
  *
  * 收发都在字节流上做（收到一个字节喂 OtaLink_Feed 一个），
  * 所以 Bootloader（无 RTOS）和 App（有 RTOS）能共用同一份代码。
  ******************************************************************************
  */

#ifndef __OTA_LINK_H__
#define __OTA_LINK_H__

#include <stdint.h>
#include "ota_layout.h"

/* 收到一个完整且校验通过的帧时被调用（在调用 OtaLink_Feed 的上下文里，不要阻塞） */
typedef void (*OtaLink_Handler)(uint8_t type, uint8_t seq, const uint8_t *payload, uint16_t len);

void OtaLink_Init(OtaLink_Handler handler);

/* 喂一个字节（从串口环形缓冲里读出来之后逐个喂） */
void OtaLink_Feed(uint8_t byte);

/* 发一帧（内部组帧 + COBS + 前后 0x00 定界）。payload 可以为 NULL（len=0） */
void OtaLink_Send(uint8_t type, uint8_t seq, const uint8_t *payload, uint16_t len);

/* ---- 小端读写（两边的协议解析都用这几个，别手写移位） ---- */
static inline uint16_t Ota_GetLe16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}

static inline uint32_t Ota_GetLe32(const uint8_t *p)
{
    return ((uint32_t)p[0]) | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void Ota_PutLe16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static inline void Ota_PutLe32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

#endif /* __OTA_LINK_H__ */
