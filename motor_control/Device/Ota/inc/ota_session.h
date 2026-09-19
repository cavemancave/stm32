/**
  ******************************************************************************
  * @file    ota_session.h
  * @brief   下载会话状态机：擦除 → 收块写 Flash → 读回校验 → 提交
  *
  * 和 Flash/元数据的细节都封在这里，App 和 Bootloader 共用。
  * 断链/掉电续传靠元数据里的 dl_* 字段（每 32 KB 落一次盘）。
  ******************************************************************************
  */

#ifndef __OTA_SESSION_H__
#define __OTA_SESSION_H__

#include <stdint.h>
#include "ota_layout.h"

typedef enum
{
    OTA_SESS_IDLE = 0,   /* 没有会话 */
    OTA_SESS_RECV = 1,   /* 正在收 */
    OTA_SESS_DONE = 2    /* 收完且校验通过、已提交（等重启） */
} ota_sess_state_t;

/* 开始会话（会做擦除）。
   slot=0..1、size、crc32（整包）、ver、force=1 强制重新擦除不要续传。
   *resume_off 返回"主机该从哪个偏移开始发"（续传时 != 0）。
   返回 OTA_OK 或 OTA_E_xxx */
int8_t OtaSession_Begin(uint8_t slot, uint32_t size, uint32_t crc, uint32_t ver,
                        uint8_t force, uint32_t *resume_off);

/* 收一块数据。off 必须等于当前偏移（32 字节对齐）。*next_off 返回下一个期望偏移 */
int8_t OtaSession_Data(uint32_t off, const uint8_t *data, uint16_t len, uint32_t *next_off);

/* 收尾：读回 Flash 重新算 CRC32，通过才提交到元数据（pending_slot）。
   *out_crc 返回实际算出来的 CRC32（不管成没成，方便上位机排查） */
int8_t OtaSession_End(uint32_t *out_crc);

/* 放弃当前会话（保留已写入的内容和 dl_offset → 下次可以续传） */
void OtaSession_Abort(void);

ota_sess_state_t OtaSession_State(void);
uint32_t         OtaSession_Offset(void);
uint8_t          OtaSession_Slot(void);

#endif /* __OTA_SESSION_H__ */
