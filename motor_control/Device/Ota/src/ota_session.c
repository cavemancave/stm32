/**
  ******************************************************************************
  * @file    ota_session.c
  * @brief   下载会话状态机
  ******************************************************************************
  */

#include "ota_session.h"
#include "ota_flash.h"
#include "ota_meta.h"

#include <stddef.h>

/* Private variables ---------------------------------------------------------*/

typedef struct
{
    ota_sess_state_t state;
    uint8_t          slot;
    uint8_t          pad[3];
    uint32_t         size;    /* 整包长度 */
    uint32_t         crc;     /* 整包 CRC32（主机给的） */
    uint32_t         ver;     /* 版本号（主机给的，写进元数据） */
    uint32_t         off;     /* 已经写进 Flash 的字节数 */
    uint32_t         saved;   /* 上次落盘的 off */
} ota_session_ctx_t;

static ota_session_ctx_t s_sess;

/* Private functions ---------------------------------------------------------*/

/* 把"已经写到哪了"落盘（断链/掉电后能续传） */
static void ota_session_save_progress(void)
{
    ota_meta_t m;

    if (OtaMeta_Load(&m) < 0)
    {
        return;
    }

    m.dl_active = 1U;
    m.dl_slot   = s_sess.slot;
    m.dl_size   = s_sess.size;
    m.dl_crc    = s_sess.crc;
    m.dl_offset = s_sess.off;

    if (OtaMeta_Write(&m) == OTA_OK)
    {
        s_sess.saved = s_sess.off;
    }
}

/* Exported functions --------------------------------------------------------*/

int8_t OtaSession_Begin(uint8_t slot, uint32_t size, uint32_t crc, uint32_t ver,
                        uint8_t force, uint32_t *resume_off)
{
    ota_meta_t m;
    int8_t     st;

    if ((slot >= OTA_SLOT_COUNT) || (resume_off == NULL) ||
        (size == 0U) || (size > OTA_SLOT_SIZE))
    {
        return OTA_E_PARAM;
    }

    if (OtaMeta_Load(&m) < 0)
    {
        return OTA_E_STATE;
    }

    /* 续传：同一个槽 + 同样的长度和 CRC，而且上次是"擦过再写的"（槽里还是那份数据）
       —— 这时候不能重新擦，否则已写的前半段就白传了 */
    if ((force == 0U) && (m.dl_active != 0U) && (m.dl_slot == slot) &&
        (m.dl_size == size) && (m.dl_crc == crc) && (m.dl_offset <= size))
    {
        s_sess.state = OTA_SESS_RECV;
        s_sess.slot  = slot;
        s_sess.size  = size;
        s_sess.crc   = crc;
        s_sess.ver   = ver;
        s_sess.off   = m.dl_offset;
        s_sess.saved = m.dl_offset;

        *resume_off = m.dl_offset;

        return OTA_OK;
    }

    /* 全新下载：**先把整个槽擦干净**。
       整个流程里唯一的长时间 Flash 操作，放在 BEGIN（此时还没有数据在传），
       之后就只剩 32 字节 flash word 的编程停顿了。 */
    st = OtaFlash_EraseSlot(slot);

    if (st != OTA_OK)
    {
        return st;
    }

    m.dl_active       = 1U;
    m.dl_slot         = slot;
    m.dl_size         = size;
    m.dl_crc          = crc;
    m.dl_offset       = 0U;
    m.slot_size[slot] = 0U;    /* 旧镜像的登记信息作废 */
    m.slot_crc[slot]  = 0U;

    if (OtaMeta_Write(&m) != OTA_OK)
    {
        return OTA_E_WRITE;
    }

    s_sess.state = OTA_SESS_RECV;
    s_sess.slot  = slot;
    s_sess.size  = size;
    s_sess.crc   = crc;
    s_sess.ver   = ver;
    s_sess.off   = 0U;
    s_sess.saved = 0U;

    *resume_off = 0U;

    return OTA_OK;
}

int8_t OtaSession_Data(uint32_t off, const uint8_t *data, uint16_t len, uint32_t *next_off)
{
    int8_t st;

    if (next_off == NULL)
    {
        return OTA_E_PARAM;
    }

    *next_off = s_sess.off;

    if (s_sess.state != OTA_SESS_RECV)
    {
        return OTA_E_STATE;
    }

    /* 偏移必须是 32 字节对齐的（Flash 写法要求），而且只能顺序往后写 */
    if (((off & (OTA_FLASH_WRITE_ALIGN - 1U)) != 0U) || (off != s_sess.off))
    {
        return OTA_E_OFFSET;
    }

    if ((len == 0U) || ((off + len) > s_sess.size))
    {
        return OTA_E_PARAM;
    }

    st = OtaFlash_Write(Ota_SlotBase(s_sess.slot) + off, data, len);

    if (st != OTA_OK)
    {
        return st;
    }

    s_sess.off += len;
    *next_off   = s_sess.off;

    if ((s_sess.off - s_sess.saved) >= OTA_DL_SAVE_STEP)
    {
        ota_session_save_progress();
    }

    return OTA_OK;
}

int8_t OtaSession_End(uint32_t *out_crc)
{
    ota_meta_t m;
    uint32_t   crc = 0U;
    int8_t     st;

    if (s_sess.state != OTA_SESS_RECV)
    {
        return OTA_E_STATE;
    }

    if (s_sess.off != s_sess.size)
    {
        return OTA_E_PARAM;    /* 还没收全 */
    }

    /* 从 Flash 里**读回**重算，而不是"边收边算"：这样 Flash 写失败也能发现 */
    st = OtaFlash_SlotCrc(s_sess.slot, s_sess.size, &crc);

    if (st != OTA_OK)
    {
        return st;
    }

    if (out_crc != NULL)
    {
        *out_crc = crc;
    }

    if (crc != s_sess.crc)
    {
        return OTA_E_CRC;      /* 不提交，旧固件继续跑，主机重来 */
    }

    if (OtaMeta_Load(&m) < 0)
    {
        return OTA_E_STATE;
    }

    m.pending_slot        = s_sess.slot;
    m.slot_size[s_sess.slot] = s_sess.size;
    m.slot_crc[s_sess.slot]  = crc;
    m.slot_ver[s_sess.slot]  = s_sess.ver;
    m.dl_active           = 0U;

    if (OtaMeta_Write(&m) != OTA_OK)
    {
        return OTA_E_WRITE;
    }

    s_sess.state = OTA_SESS_DONE;

    return OTA_OK;
}

void OtaSession_Abort(void)
{
    /* 故意保留 dl_* / dl_offset：下次 BEGIN 同样的 size+crc 就能接着传 */
    s_sess.state = OTA_SESS_IDLE;
}

ota_sess_state_t OtaSession_State(void)
{
    return s_sess.state;
}

uint32_t OtaSession_Offset(void)
{
    return s_sess.off;
}

uint8_t OtaSession_Slot(void)
{
    return (s_sess.state == OTA_SESS_IDLE) ? OTA_SLOT_NONE : s_sess.slot;
}
