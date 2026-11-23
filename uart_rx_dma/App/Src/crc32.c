/**
 * @file    crc32.c
 * @brief   CRC-32/ISO-HDLC，逐字节位运算实现。
 *
 * 帧最大 16 字节，速度不是瓶颈；逐位实现代码小、无表、易核对。
 */
#include "crc32.h"

#define CRC32_POLY 0x04C11DB7U

uint32_t crc32_begin(void)
{
    return 0xFFFFFFFFU;
}

uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    while (len-- > 0U) {
        crc ^= ((uint32_t)(*data++) << 24);
        for (uint32_t i = 0U; i < 8U; ++i) {
            if ((crc & 0x80000000U) != 0U) {
                crc = (crc << 1) ^ CRC32_POLY;
            } else {
                crc <<= 1;
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
