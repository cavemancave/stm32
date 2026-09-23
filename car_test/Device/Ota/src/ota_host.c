/**
  ******************************************************************************
  * @file    ota_host.c
  * @brief   设备侧协议处理（App 和 Bootloader 共用）
  *
  * 回复帧载荷布局（每个回复的 [0] 都是 status，见 ota_layout.h 的 OTA_E_xxx）：
  *
  *   INFO     (0x01) → 38 字节
  *     [0] status  [1] active [2] boot [3] pending [4] attempts [5] flags
  *     [6..17]  槽 A: size(4) crc(4) ver(4)
  *     [18..29] 槽 B: size(4) crc(4) ver(4)
  *     [30..33] 当前运行版本   [34..37] Bootloader 版本
  *
  *   BEGIN    (0x02) 请求: [0] slot(0xFF=自动) [1..4] size [5..8] crc32 [9..12] ver [13] flags(b0=强制重下)
  *                   → 7 字节: [0] status [1..4] resume_off [5..6] chunk
  *   DATA     (0x03) 请求: [0..3] off [4..] data
  *                   → 5 字节: [0] status [1..4] next_off（status≠OK 时表示"从这儿重发"）
  *   END      (0x04) → 10 字节: [0] status [1] slot [2..5] crc [6..9] 已写入字节数
  *   ABORT    (0x05) → 5 字节: [0] status [1..4] 已写入字节数（下次可以接着传）
  *   REBOOT   (0x06) 请求: [0] mode（0=正常重启，1=重启进 BL 恢复台）→ 1 字节
  *   ERASE    (0x07) 请求: [0] slot → 1 字节
  *   ROLLBACK (0x08) → 2 字节: [0] status [1] boot_slot
  ******************************************************************************
  */

#include "ota_host.h"
#include "ota_link.h"
#include "ota_meta.h"
#include "ota_flash.h"
#include "ota_session.h"
#include "ota_crc.h"

#include "stm32h7xx_hal.h"

#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/

static ota_host_hooks_t s_hooks;
static uint32_t         s_last_frame;      /* 最近一次收到帧的时刻（会话看门狗用） */
static uint8_t          s_session_notified;/* 已经调过 on_session_start 还没调 stop */

/* Private functions ---------------------------------------------------------*/

static uint8_t ota_host_running_slot(void)
{
    uint8_t slot = OTA_SLOT_NONE;

    if ((s_hooks.get_running_slot != NULL))
    {
        slot = s_hooks.get_running_slot();
    }

    return slot;
}

static void ota_host_log(const char *line)
{
    if ((s_hooks.log != NULL) && (line != NULL))
    {
        s_hooks.log(line);
    }
}

static void ota_host_session_start(void)
{
    if (s_session_notified == 0U)
    {
        s_session_notified = 1U;

        if (s_hooks.on_session_start != NULL)
        {
            s_hooks.on_session_start();
        }
    }
}

static void ota_host_session_stop(void)
{
    if (s_session_notified != 0U)
    {
        s_session_notified = 0U;

        if (s_hooks.on_session_stop != NULL)
        {
            s_hooks.on_session_stop();
        }
    }
}

/* slot = 0xFF（自动）时挑一个目标槽：
   - 正常运行（App 知道自己跑在哪个槽）→ 挑另一个槽
   - 恢复台（BL，running = NONE）→ 挑 active 槽（也就是坏掉的那个，就是要覆盖它） */
static uint8_t ota_host_auto_slot(const ota_meta_t *m)
{
    uint8_t running = ota_host_running_slot();

    if (running < OTA_SLOT_COUNT)
    {
        return Ota_OtherSlot(running);
    }

    if (m->active_slot < OTA_SLOT_COUNT)
    {
        return m->active_slot;
    }

    return OTA_SLOT_A;
}

/* ---- 各命令 -------------------------------------------------------------- */

static uint8_t ota_host_info(uint8_t *rsp)
{
    ota_meta_t m;
    uint8_t    running;
    uint8_t    in_bootloader;

    (void)OtaMeta_Load(&m);

    running = ota_host_running_slot();

    in_bootloader = (running >= OTA_SLOT_COUNT) ? 1U : 0U;

    if (running >= OTA_SLOT_COUNT)
    {
        running = m.active_slot;    /* BL 里"当前版本"按 active 槽算 */
    }

    rsp[0] = (uint8_t)OTA_OK;
    rsp[1] = m.active_slot;
    rsp[2] = m.boot_slot;
    rsp[3] = m.pending_slot;
    rsp[4] = m.boot_attempts;
    rsp[5] = (uint8_t)(m.flags | ((in_bootloader != 0U) ? OTA_INFO_FLAG_BOOTLOADER : 0U));

    Ota_PutLe32(&rsp[6],  m.slot_size[OTA_SLOT_A]);
    Ota_PutLe32(&rsp[10], m.slot_crc[OTA_SLOT_A]);
    Ota_PutLe32(&rsp[14], m.slot_ver[OTA_SLOT_A]);
    Ota_PutLe32(&rsp[18], m.slot_size[OTA_SLOT_B]);
    Ota_PutLe32(&rsp[22], m.slot_crc[OTA_SLOT_B]);
    Ota_PutLe32(&rsp[26], m.slot_ver[OTA_SLOT_B]);

    if ((running < OTA_SLOT_COUNT) && (m.slot_ver[running] != 0U))
    {
        Ota_PutLe32(&rsp[30], m.slot_ver[running]);
    }
    else
    {
        Ota_PutLe32(&rsp[30], OTA_FW_VERSION);
    }

    Ota_PutLe32(&rsp[34], OTA_BL_VERSION);

    return 38U;
}

static uint8_t ota_host_begin(const uint8_t *pl, uint16_t len, uint8_t *rsp)
{
    uint8_t  slot;
    uint32_t size;
    uint32_t crc;
    uint32_t ver;
    uint8_t  flags;
    uint32_t resume = 0U;
    int8_t   st;

    if (len < 14U)
    {
        rsp[0] = (uint8_t)OTA_E_PARAM;
        return 1U;
    }

    slot  = pl[0];
    size  = Ota_GetLe32(&pl[1]);
    crc   = Ota_GetLe32(&pl[5]);
    ver   = Ota_GetLe32(&pl[9]);
    flags = pl[13];

    if (slot == OTA_SLOT_NONE)
    {
        ota_meta_t m;

        (void)OtaMeta_Load(&m);
        slot = ota_host_auto_slot(&m);
    }

    if (slot >= OTA_SLOT_COUNT)
    {
        rsp[0] = (uint8_t)OTA_E_PARAM;
        return 1U;
    }

    if ((s_hooks.check_target_slot != NULL) &&
        (s_hooks.check_target_slot(slot) != (int8_t)OTA_OK))
    {
        rsp[0] = (uint8_t)OTA_E_NOTALLOWED;
        return 1U;
    }

    st = OtaSession_Begin(slot, size, crc, ver, (flags & 0x01U), &resume);

    if (st != OTA_OK)
    {
        rsp[0] = (uint8_t)st;
        return 1U;
    }

    /* 这行要打在"静音日志"之前，不然就被自己静掉了 */
    {
        char line[128];

        (void)snprintf(line, sizeof(line),
                       "ota: BEGIN slot=%c size=%lu crc=0x%08lX resume=%lu erase=%lu sector(s)\r\n",
                       (slot == OTA_SLOT_A) ? 'A' : 'B',
                       (unsigned long)size, (unsigned long)crc,
                       (unsigned long)resume, (unsigned long)OtaFlash_LastEraseSectors());
        ota_host_log(line);
    }

    ota_host_session_start();
    s_last_frame = HAL_GetTick();

    rsp[0] = (uint8_t)OTA_OK;
    Ota_PutLe32(&rsp[1], resume);
    Ota_PutLe16(&rsp[5], OTA_CHUNK_DEFAULT);

    return 7U;
}

static uint8_t ota_host_data(const uint8_t *pl, uint16_t len, uint8_t *rsp)
{
    uint32_t off;
    uint32_t next = 0U;
    int8_t   st;

    if (len < 5U)
    {
        rsp[0] = (uint8_t)OTA_E_PARAM;
        Ota_PutLe32(&rsp[1], OtaSession_Offset());
        return 5U;
    }

    off = Ota_GetLe32(pl);
    st  = OtaSession_Data(off, &pl[4], (uint16_t)(len - 4U), &next);

    rsp[0] = (uint8_t)st;
    Ota_PutLe32(&rsp[1], next);

    return 5U;
}

static uint8_t ota_host_end(uint8_t *rsp, uint8_t *reboot)
{
    uint32_t crc = 0U;
    int8_t   st  = OtaSession_End(&crc);

    rsp[0] = (uint8_t)st;
    rsp[1] = OtaSession_Slot();
    Ota_PutLe32(&rsp[2], crc);
    Ota_PutLe32(&rsp[6], OtaSession_Offset());

    ota_host_session_stop();

    if (st == OTA_OK)
    {
        ota_host_log("ota: image verified, pending slot switch, rebooting ...\r\n");
        *reboot = 1U;      /* 回复发完就重启，由 Bootloader 切槽（一次命令刷完） */
    }

    return 10U;
}

static uint8_t ota_host_erase(const uint8_t *pl, uint16_t len, uint8_t *rsp)
{
    uint8_t    slot;
    ota_meta_t m;
    int8_t     st;

    if (len < 1U)
    {
        rsp[0] = (uint8_t)OTA_E_PARAM;
        return 1U;
    }

    slot = pl[0];

    if (slot >= OTA_SLOT_COUNT)
    {
        rsp[0] = (uint8_t)OTA_E_PARAM;
        return 1U;
    }

    if ((s_hooks.check_target_slot != NULL) &&
        (s_hooks.check_target_slot(slot) != (int8_t)OTA_OK))
    {
        rsp[0] = (uint8_t)OTA_E_NOTALLOWED;
        return 1U;
    }

    st = OtaFlash_EraseSlot(slot);

    if (st == OTA_OK)
    {
        /* 这个槽的登记信息和可能存在的续传进度一起作废 */
        (void)OtaMeta_Load(&m);
        m.slot_size[slot] = 0U;
        m.slot_crc[slot]  = 0U;
        m.slot_ver[slot]  = 0U;

        if (m.pending_slot == slot)
        {
            m.pending_slot = OTA_SLOT_NONE;
        }
        if (m.dl_slot == slot)
        {
            m.dl_active = 0U;
        }

        (void)OtaMeta_Write(&m);
    }

    rsp[0] = (uint8_t)st;

    return 1U;
}

static uint8_t ota_host_rollback(uint8_t *rsp)
{
    ota_meta_t m;
    uint8_t    running;
    uint8_t    target;
    uint32_t   crc = 0U;
    int8_t     st;

    (void)OtaMeta_Load(&m);

    running = ota_host_running_slot();

    if (running >= OTA_SLOT_COUNT)
    {
        running = (m.active_slot < OTA_SLOT_COUNT) ? m.active_slot : OTA_SLOT_A;
    }

    target = Ota_OtherSlot(running);

    if (OtaMeta_SlotValid(&m, target) != 0U)
    {
        st = OtaFlash_SlotCrc(target, m.slot_size[target], &crc);

        if ((st != OTA_OK) || (crc != m.slot_crc[target]))
        {
            rsp[0] = (uint8_t)OTA_E_CRC;
            rsp[1] = target;
            return 2U;
        }
    }
    else if (OtaFlash_ImageLooksValid(target) == 0U)
    {
        rsp[0] = (uint8_t)OTA_E_NOTALLOWED;   /* 另一个槽是空的：没得回滚 */
        rsp[1] = target;
        return 2U;
    }

    m.boot_slot      = target;
    m.boot_attempts  = 0U;
    m.pending_slot   = OTA_SLOT_NONE;
    m.flags          = 0U;

    st = OtaMeta_Write(&m);

    rsp[0] = (uint8_t)((st == OTA_OK) ? OTA_OK : st);
    rsp[1] = target;

    return 2U;
}

static uint8_t ota_host_reboot(const uint8_t *pl, uint16_t len, uint8_t *rsp, uint8_t *reboot)
{
    uint8_t    mode = ((len >= 1U) ? pl[0] : 0U);
    ota_meta_t m;

    (void)OtaMeta_Load(&m);

    /* 只在标志真的要变的时候才写 Flash（每次复位都写一条太浪费寿命） */
    if ((mode == 1U) && ((m.flags & 0x01U) == 0U))
    {
        m.flags |= 0x01U;              /* 请求下次停在 Bootloader 恢复台 */
        (void)OtaMeta_Write(&m);
    }
    else if ((mode == 0U) && ((m.flags & 0x01U) != 0U))
    {
        m.flags &= (uint8_t)~0x01U;    /* 取消"停在 BL"（在恢复台里用它跳出来） */
        (void)OtaMeta_Write(&m);
    }

    rsp[0] = (uint8_t)OTA_OK;
    *reboot = 1U;

    return 1U;
}

/* Exported functions --------------------------------------------------------*/

void OtaHost_Init(const ota_host_hooks_t *hooks)
{
    if (hooks != NULL)
    {
        s_hooks = *hooks;
    }
    else
    {
        memset(&s_hooks, 0, sizeof(s_hooks));
    }

    s_last_frame       = 0U;
    s_session_notified = 0U;
}

uint8_t OtaHost_HandleFrame(uint8_t type, uint8_t seq, const uint8_t *payload,
                            uint16_t len, uint8_t *reboot)
{
    uint8_t rsp[64];
    uint8_t rlen = 0U;

    *reboot = 0U;

    s_last_frame = HAL_GetTick();

    switch (type)
    {
        case OTA_T_INFO:
            rlen = ota_host_info(rsp);
            break;

        case OTA_T_BEGIN:
            rlen = ota_host_begin(payload, len, rsp);
            break;

        case OTA_T_DATA:
            rlen = ota_host_data(payload, len, rsp);
            break;

        case OTA_T_END:
            rlen = ota_host_end(rsp, reboot);
            break;

        case OTA_T_ABORT:
            OtaHost_AbortSession();
            rsp[0] = (uint8_t)OTA_OK;
            Ota_PutLe32(&rsp[1], OtaSession_Offset());
            rlen = 5U;
            break;

        case OTA_T_ERASE:
            rlen = ota_host_erase(payload, len, rsp);
            break;

        case OTA_T_ROLLBACK:
            rlen = ota_host_rollback(rsp);
            break;

        case OTA_T_REBOOT:
            rlen = ota_host_reboot(payload, len, rsp, reboot);
            break;

        default:
            return 0U;    /* 不是本模块的命令 */
    }

    if (s_hooks.lock != NULL)
    {
        s_hooks.lock();
    }

    OtaLink_Send((uint8_t)(type | 0x80U), seq, rsp, rlen);

    if (s_hooks.unlock != NULL)
    {
        s_hooks.unlock();
    }

    return 1U;
}

void OtaHost_AbortSession(void)
{
    if (OtaSession_State() != OTA_SESS_IDLE)
    {
        OtaSession_Abort();
        ota_host_session_stop();
        ota_host_log("ota: session aborted, resume offset kept\r\n");
    }
}

void OtaHost_Tick(void)
{
    if ((OtaSession_State() != OTA_SESS_IDLE) && (s_session_notified != 0U) &&
        ((HAL_GetTick() - s_last_frame) > OTA_SESSION_TIMEOUT_MS))
    {
        ota_host_log("ota: session timeout (no frame for 30 s), aborted\r\n");
        OtaHost_AbortSession();
    }
}

uint8_t OtaHost_SessionActive(void)
{
    return (s_session_notified != 0U) ? 1U : 0U;
}
