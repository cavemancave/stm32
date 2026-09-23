/**
  ******************************************************************************
  * @file    ota_crc.c
  * @brief   CRC-32/ISO-HDLC（和 Python zlib.crc32 完全一致）
  *
  * 用 16 项"半字节表"而不是 256 项全表：只要 64 字节常量，速度也够
  * （校验 384 KB 一个槽也就几 ms 量级，只在启动和 OTA 结束时做）。
  * 算法和表值已用 Python 对着 zlib.crc32 逐字节比对过（见 docs/ota_design.md）。
  ******************************************************************************
  */

#include "ota_crc.h"

/* 反射多项式 0xEDB88320 对应的 4 bit 表 */
static const uint32_t s_crc32_nibble[16] =
{
    0x00000000U, 0x1DB71064U, 0x3B6E20C8U, 0x26D930ACU,
    0x76DC4190U, 0x6B6B51F4U, 0x4DB26158U, 0x5005713CU,
    0xEDB88320U, 0xF00F9344U, 0xD6D6A3E8U, 0xCB61B38CU,
    0x9B64C2B0U, 0x86D3D2D4U, 0xA00AE278U, 0xBDBDF21CU
};

uint32_t Ota_Crc32Update(uint32_t crc, const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;

    while (len > 0U)
    {
        crc ^= (uint32_t)(*p);

        /* 一次啃 4 bit，两轮 = 一个字节 */
        crc = s_crc32_nibble[crc & 0x0FU] ^ (crc >> 4);
        crc = s_crc32_nibble[crc & 0x0FU] ^ (crc >> 4);

        p++;
        len--;
    }

    return crc;
}

uint32_t Ota_Crc32(const void *data, uint32_t len)
{
    return Ota_Crc32Final(Ota_Crc32Update(OTA_CRC32_INIT, data, len));
}
