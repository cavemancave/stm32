/**
  ******************************************************************************
  * @file    ota_service.h
  * @brief   App 侧 OTA 服务：USART1 上的"日志 + 控制 + 升级"三合一
  *
  * 一个 AboveNormal 优先级的任务专门收 USART1：
  *   - 裸文本（ASCII）不管，那是设备自己打出去的日志；
  *   - 二进制帧交给 OtaHost（INFO/BEGIN/DATA/END/…）和 CTRL_CMD（控制电机）。
  *
  * 升级期间会自动：失能电机 + 停 200 ms 轮询 + 静音日志（别抢带宽）。
  ******************************************************************************
  */

#ifndef __OTA_SERVICE_H__
#define __OTA_SERVICE_H__

#include <stdint.h>
#include "ota_layout.h"

/* 建串口、建任务。必须在 osKernelInitialize() 之后调用
   （在 Core/Src/freertos.c 的 MX_FREERTOS_Init() 里） */
void OtaService_Init(void);

/* 临时静音/恢复日志（升级期间自动开，也可以由 CTRL_CMD 控制） */
void OtaService_MuteLog(uint8_t mute);

/* 正在升级吗（上位机要求"安静"时用得到） */
uint8_t OtaService_Active(void);

#endif /* __OTA_SERVICE_H__ */
