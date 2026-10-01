/**
  ******************************************************************************
  * @file    ota_crc.h
  * @brief   CRC-32/ISO-HDLC（= Python zlib.crc32 / binascii.crc32 那一套）
  *
  * ⚠ 别和电机协议的 CRC-8/MAXIM 搞混，那是另一套（见 Device/Motor/motor.c）。
  *
  * 参数：多项式 0x04C11DB7（反射写法 0xEDB88320），初值 0xFFFFFFFF，
  *       输入/输出反射，终值取反。
  *       自检：CRC32("123456789") == 0xCBF43926
  ******************************************************************************
  */

#ifndef __OTA_CRC_H__
#define __OTA_CRC_H__

#include <stdint.h>

/* 一次性算完（内部会做初值/终值处理） */
uint32_t Ota_Crc32(const void *data, uint32_t len);

/* 分块算：crc 传上一次的返回值，第一次传 OTA_CRC32_INIT。
   注意返回值**已经**是"中间值"，不要自己做取反。 */
#define OTA_CRC32_INIT   0xFFFFFFFFU
uint32_t Ota_Crc32Update(uint32_t crc, const void *data, uint32_t len);

/* 把中间值收尾成最终 CRC32 */
#define Ota_Crc32Final(crc)   ((crc) ^ 0xFFFFFFFFU)

#endif /* __OTA_CRC_H__ */
