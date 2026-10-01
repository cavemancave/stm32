/**
  ******************************************************************************
  * @file    ota_link.c
  * @brief   组帧 / COBS 编解码（算法和 Python 版逐个用例比对过，见 docs/ota_design.md）
  ******************************************************************************
  */

#include "ota_link.h"
#include "ota_com.h"
#include "ota_crc.h"

#include <string.h>

#define OTA_LINK_RAW_MAX   (OTA_FRAME_OVERHEAD + OTA_FRAME_MAX_PAYLOAD)
#define OTA_LINK_ENC_MAX   (OTA_LINK_RAW_MAX + (OTA_LINK_RAW_MAX / 254U) + 2U)

/* Private variables ---------------------------------------------------------*/

static uint8_t s_raw[OTA_LINK_RAW_MAX];   /* 组帧/解帧后的明文帧 */
static uint8_t s_enc[OTA_LINK_ENC_MAX];   /* COBS 编码结果 */
static uint8_t s_rx[OTA_LINK_ENC_MAX];    /* 收：两个 0x00 之间的候选帧 */

static uint16_t       s_rx_len;
static uint8_t        s_rx_drop;          /* 候选帧太长：丢弃到下一个 0x00 */
static OtaLink_Handler s_handler;

/* Private functions ---------------------------------------------------------*/

/* 返回编码后长度（不含前后定界用的 0x00）。
   注意最后一定要补一个 code 字节 —— 这是 COBS 的规范（不然解出来会少一个结尾的 0x00） */
static uint16_t OtaLink_CobsEncode(const uint8_t *in, uint16_t len, uint8_t *out)
{
    uint16_t rd      = 0U;
    uint16_t wr      = 1U;
    uint16_t code_pos = 0U;
    uint8_t  code    = 1U;

    while (rd < len)
    {
        if (in[rd] != 0x00U)
        {
            out[wr] = in[rd];
            wr++;
            code++;
        }

        if (code == 0xFFU)
        {
            out[code_pos] = code;
            code_pos      = wr;
            out[wr]       = 0x00U;   /* 占位，下次循环会被覆盖 */
            wr++;
            code = 1U;
        }
        else if (in[rd] == 0x00U)
        {
            out[code_pos] = code;
            code_pos      = wr;
            out[wr]       = 0x00U;
            wr++;
            code = 1U;
        }

        rd++;
    }

    out[code_pos] = code;

    return wr;
}

/* 返回 0 = 成功；其他 = 丢弃这帧 */
static int8_t OtaLink_CobsDecode(const uint8_t *in, uint16_t len, uint8_t *out,
                                 uint32_t out_max, uint16_t *out_len)
{
    uint32_t rd = 0U;
    uint32_t wr = 0U;

    while (rd < len)
    {
        uint8_t  code = in[rd];
        uint16_t i;

        rd++;

        if (code == 0x00U)
        {
            return -1;
        }

        for (i = 1U; i < (uint16_t)code; i++)
        {
            if (rd >= len)
            {
                return -1;      /* 被截断了 */
            }
            if (wr >= out_max)
            {
                return -1;
            }

            out[wr] = in[rd];
            wr++;
            rd++;
        }

        /* code < 0xFF 说明这个块是被一个 0x00 截断的（还有后续块） */
        if ((code < 0xFFU) && (rd < len))
        {
            if (wr >= out_max)
            {
                return -1;
            }

            out[wr] = 0x00U;
            wr++;
        }
    }

    *out_len = (uint16_t)wr;

    return 0;
}

/* 候选帧收全（遇到 0x00）：解 COBS + 校验，通过就交给上层 */
static void OtaLink_TryFrame(void)
{
    uint16_t len = 0U;

    if (OtaLink_CobsDecode(s_rx, s_rx_len, s_raw, (uint32_t)sizeof(s_raw), &len) != 0)
    {
        return;
    }

    if (len < OTA_FRAME_OVERHEAD)
    {
        return;
    }
    if ((s_raw[0] != OTA_FRAME_SYNC0) || (s_raw[1] != OTA_FRAME_SYNC1))
    {
        return;
    }
    if (s_raw[2] != OTA_PROTO_VER)
    {
        return;
    }

    uint16_t plen = Ota_GetLe16(&s_raw[5]);

    if ((uint32_t)plen + OTA_FRAME_OVERHEAD != (uint32_t)len)
    {
        return;
    }

    /* CRC32 覆盖 VER..PAYLOAD，也就是 s_raw[2 .. 7+plen) */
    uint32_t crc_stored = Ota_GetLe32(&s_raw[7U + plen]);

    if (Ota_Crc32(&s_raw[2], (uint32_t)(5U + plen)) != crc_stored)
    {
        return;
    }

    if (s_handler != NULL)
    {
        s_handler(s_raw[3], s_raw[4], &s_raw[7], plen);
    }
}

/* Exported functions --------------------------------------------------------*/

void OtaLink_Init(OtaLink_Handler handler)
{
    s_handler = handler;
    s_rx_len  = 0U;
    s_rx_drop = 0U;
}

void OtaLink_Feed(uint8_t byte)
{
    if (byte == 0x00U)
    {
        /* 定界符：把攒下的候选帧拿去试一次，然后重新开始 */
        if ((s_rx_len > 0U) && (s_rx_drop == 0U))
        {
            OtaLink_TryFrame();
        }

        s_rx_len  = 0U;
        s_rx_drop = 0U;

        return;
    }

    if (s_rx_len < (uint16_t)sizeof(s_rx))
    {
        s_rx[s_rx_len] = byte;
        s_rx_len++;
    }
    else
    {
        s_rx_drop = 1U;    /* 太长：不是帧，丢弃到下一个定界符 */
    }
}

void OtaLink_Send(uint8_t type, uint8_t seq, const uint8_t *payload, uint16_t len)
{
    static const uint8_t delim = 0x00U;
    uint16_t enc_len;

    if (len > OTA_FRAME_MAX_PAYLOAD)
    {
        return;
    }

    s_raw[0] = OTA_FRAME_SYNC0;
    s_raw[1] = OTA_FRAME_SYNC1;
    s_raw[2] = OTA_PROTO_VER;
    s_raw[3] = type;
    s_raw[4] = seq;
    Ota_PutLe16(&s_raw[5], len);

    if ((payload != NULL) && (len > 0U))
    {
        memcpy(&s_raw[7], payload, len);
    }

    Ota_PutLe32(&s_raw[7U + len], Ota_Crc32(&s_raw[2], (uint32_t)(5U + len)));

    enc_len = OtaLink_CobsEncode(s_raw, (uint16_t)(OTA_FRAME_OVERHEAD + len), s_enc);

    /* 前后各一个 0x00：中间丢字节也能靠下一个定界符重新对齐 */
    OtaCom_Write(&delim, 1U);
    OtaCom_Write(s_enc, enc_len);
    OtaCom_Write(&delim, 1U);
}
