/**
 * @file    crc32.h
 * @brief   CRC-32/ISO-HDLC（和 Python zlib.crc32 一致）。
 *
 * poly=0x04C11DB7, init=0xFFFFFFFF, refin/refout=true, xorout=0xFFFFFFFF。
 */
#ifndef CRC32_H
#define CRC32_H

#include <stdint.h>
#include <stddef.h>

uint32_t crc32_begin(void);
uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len);
uint32_t crc32_final(uint32_t crc);
uint32_t crc32_calc(const uint8_t *data, size_t len);

#endif /* CRC32_H */
