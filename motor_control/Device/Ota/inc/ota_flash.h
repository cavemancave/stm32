/**
  ******************************************************************************
  * @file    ota_flash.h
  * @brief   内部 Flash 擦/写 + 槽校验（Bootloader 和 App 共用）
  *
  * ⚠ H723 是**单 bank**：擦写 Flash 时如果 CPU 正在从 Flash 取指，取指会被 stall
  *   一直到操作结束（不是 HardFault）。所以：
  *     - 擦除（ms 级）集中在 OTA_BEGIN 一次做完；
  *     - 传输中只剩 32 字节 flash word 的编程（几十 µs），靠串口 16 字节 FIFO 顶住。
  ******************************************************************************
  */

#ifndef __OTA_FLASH_H__
#define __OTA_FLASH_H__

#include <stdint.h>
#include "ota_layout.h"

/* 擦掉某个槽（3 个扇区，ms 级 —— 只在 OTA_BEGIN 里调用） */
int8_t OtaFlash_EraseSlot(uint8_t slot);

/* 擦掉元数据扇区（只在元数据记录写满 512 条时才会用到） */
int8_t OtaFlash_EraseMeta(void);

/*
 * 擦掉**任意一个**扇区（1..7；0 号是 Bootloader 自己，会被拒绝）。
 *
 * 给"自愈"用：一个 flash word 被重复编程之后 ECC 会变成双位错，**一读它就 NMI +
 * 精确总线错误**（BFAR = 那个地址），唯一的活路是把整个扇区擦掉。
 * 所以异常处理里拿到 BFAR 就能算出扇区号，调这个函数擦掉再复位。
 */
int8_t OtaFlash_EraseSector(uint32_t sector);

/* 写一段数据：addr 必须 32 字节对齐；末尾不足 32 字节的补 0xFF。
   返回 OTA_OK / OTA_E_PARAM / OTA_E_WRITE */
int8_t OtaFlash_Write(uint32_t addr, const uint8_t *data, uint32_t len);

/* 从 Flash 里**读回** size 字节算 CRC32（OTA 结束时的整包校验、BL 启动前的自检） */
int8_t OtaFlash_SlotCrc(uint8_t slot, uint32_t size, uint32_t *crc);

/* 槽前 64 字节全 0xFF = 空槽 */
uint8_t OtaFlash_IsBlank(uint8_t slot);

/* 槽开头看起来像个合法镜像吗（向量表：栈顶在 RAM、复位入口在槽内且带 Thumb 位）。
   只在"元数据里没登记过这个槽"时用来兜底（比如刚用 ST-Link 烧完第一次）。 */
uint8_t OtaFlash_ImageLooksValid(uint8_t slot);

#endif /* __OTA_FLASH_H__ */
