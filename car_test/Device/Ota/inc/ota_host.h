/**
  ******************************************************************************
  * @file    ota_host.h
  * @brief   设备侧的协议处理（INFO/BEGIN/DATA/END/ABORT/ERASE/ROLLBACK/REBOOT）
  *
  * Bootloader（恢复台）和 App（在线升级）都用这一份，差异靠 hooks 注入：
  *   App  → 拒绝写"正在运行的槽"、会话期间失能电机+静音日志
  *   BL   → 哪个槽都能写；"自动选槽"= 当前 active 槽（就是那个坏掉的）
  *
  * 回复帧的载荷布局见 ota_host.c 顶部注释（上位机 tools/ota.py 按同一份布局解析）。
  ******************************************************************************
  */

#ifndef __OTA_HOST_H__
#define __OTA_HOST_H__

#include <stdint.h>
#include "ota_layout.h"

typedef struct
{
    /* 设备现在"跑"在哪个槽（Bootloader 里返回 OTA_SLOT_NONE）。
       用来：拒绝写运行中的槽 / INFO 上报 / slot=0xFF（自动）时挑目标槽 */
    uint8_t (*get_running_slot)(void);

    /* 允许写这个槽吗？返回 OTA_OK 或 OTA_E_NOTALLOWED 等 */
    int8_t (*check_target_slot)(uint8_t slot);

    /* 一次下载会话开始/结束（App 用它们失能电机、静音日志；BL 可以给 NULL） */
    void (*on_session_start)(void);
    void (*on_session_stop)(void);

    /* 让调用方自己打一行文本日志（可以为 NULL） */
    void (*log)(const char *line);

    /* 把"同一串口上的文本日志"和"二进制帧"串行化：
       App 传 UartLog_Lock/UartLog_Unlock；Bootloader 没有多任务，传 NULL 即可 */
    void (*lock)(void);
    void (*unlock)(void);
} ota_host_hooks_t;

void OtaHost_Init(const ota_host_hooks_t *hooks);

/* 处理一帧。返回 1 = 是本模块的命令（回复已经发出去了）；0 = 不认识，调用方自己处理。
   *reboot 非 0 = 回复发完之后请重启（由调用方做，因为重启前要留时间发完） */
uint8_t OtaHost_HandleFrame(uint8_t type, uint8_t seq, const uint8_t *payload,
                            uint16_t len, uint8_t *reboot);

/* 会话看门狗（每 100 ms 左右调一次）：超时自动放弃，避免一直卡在"静音/电机失能"状态 */
void OtaHost_Tick(void);

/* 当前有下载会话在进行吗（上位机要求静音日志/停轮询时用得到） */
uint8_t OtaHost_SessionActive(void);

/* 放弃当前会话（超时/用户 ABORT/出错时用），保留续传进度 */
void OtaHost_AbortSession(void);

#endif /* __OTA_HOST_H__ */
