/**
  ******************************************************************************
  * @file    ota_meta.h
  * @brief   OTA 元数据记录（顺序追加 + 双份原子性，Bootloader 和 App 共用）
  *
  * 元数据扇区 128 KB = 512 条 × 256 字节。每次提交只"追加"一条新的，
  * 掉电写到一半的那条 CRC32 不过 → 扫描时遇到就停 → 自动回退到上一条，
  * 所以不需要文件系统也有原子性。
  ******************************************************************************
  */

#ifndef __OTA_META_H__
#define __OTA_META_H__

#include <stdint.h>
#include "ota_layout.h"

/* 填一份默认值（没有任何记录时的初始状态：两个槽都没登记） */
void OtaMeta_Init(ota_meta_t *m);

/* 扫描元数据扇区，取最后一条有效记录。
   返回 0 = 读到有效记录；1 = 空的（首次上电/刚擦过），out 已填默认值；
   <0 = 出错（理论上不会有） */
int8_t OtaMeta_Load(ota_meta_t *out);

/* 追加一条记录（自动填 magic / seq / 自身 CRC32）。
   seq 由本模块维护递增；记录写满 512 条时会自动擦掉整个元数据扇区再从头写。 */
int8_t OtaMeta_Write(ota_meta_t *m);

/* 这个槽在元数据里登记过"有效的长度 + CRC32"吗 */
uint8_t OtaMeta_SlotValid(const ota_meta_t *m, uint8_t slot);

#endif /* __OTA_META_H__ */
