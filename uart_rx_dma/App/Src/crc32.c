/**
 * @file    crc32.c
 * @brief   CRC-32/ISO-HDLC（和 Python zlib.crc32 完全一致）。
 *
 * poly=0x04C11DB7（反射形式 0xEDB88320）, init=0xFFFFFFFF,
 * refin/refout=true, xorout=0xFFFFFFFF。
 *
 * 注意：这里必须用"反射/右移"的算法，不能用左移的 MSB-first 版本
 * （那是 CRC-32/MPEG-2 的算法，结果和 zlib 对不上）。
 */
#include "crc32.h"

#define CRC32_POLY_REFLECTED 0xEDB88320U

uint32_t crc32_begin(void)
{
    return 0xFFFFFFFFU;
}

uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    while (len-- > 0U) {
        crc ^= (uint32_t)(*data++);
        for (uint32_t i = 0U; i < 8U; ++i) {
            if ((crc & 1U) != 0U) {
                crc = (crc >> 1) ^ CRC32_POLY_REFLECTED;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint32_t crc32_final(uint32_t crc)
{
    return crc ^ 0xFFFFFFFFU;
}

uint32_t crc32_calc(const uint8_t *data, size_t len)
{
    return crc32_final(crc32_update(crc32_begin(), data, len));
}
