/**
  ******************************************************************************
  * @file    ota_meta.c
  * @brief   OTA 元数据记录读写
  ******************************************************************************
  */

#include "ota_meta.h"
#include "ota_flash.h"
#include "ota_crc.h"

#include <string.h>

/* Private variables ---------------------------------------------------------*/

static uint32_t s_next_rec;    /* 下一条记录写到第几条（0..511） */
static uint32_t s_seq;         /* 最后一条有效记录的 seq */
static uint8_t  s_loaded;      /* 是否已经 Load 过（Write 前保证已经扫描过） */
static uint8_t  s_dirty;       /* 扇区里有"写了一半"的记录 → 写新记录前必须先擦扇区 */

/* Private functions ---------------------------------------------------------*/

/* 这条记录里有没有非 0xFF 的字节（用来区分"干净的空位"和"写了一半"）

   ⚠ 为什么必须区分：干净的空位直接写就行；而"写了一半"的那一格如果直接往上写，
   就是**对已编程过的 flash word 重复编程** —— H7 会因此把它的 ECC 弄成双位错，
   以后一读那个地址就 NMI + 精确总线错误（板子直接哑掉，而且擦之前不会好）。 */
static uint8_t ota_meta_has_data(const uint8_t *rec)
{
    for (uint32_t i = 0U; i < OTA_META_RECORD_SIZE; i++)
    {
        if (rec[i] != 0xFFU)
        {
            return 1U;
        }
    }

    return 0U;
}

/* 这条记录有效吗？有效就拷到 out */
static uint8_t ota_meta_valid(const uint8_t *rec, ota_meta_t *out)
{
    uint32_t magic;
    uint32_t stored;

    memcpy(&magic, rec, sizeof(magic));

    if (magic != OTA_META_MAGIC)
    {
        return 0U;
    }

    memcpy(&stored, rec + OTA_META_CRC_OFFSET, sizeof(stored));

    if (Ota_Crc32(rec, OTA_META_CRC_OFFSET) != stored)
    {
        return 0U;    /* 写到一半掉电 / 记录损坏 */
    }

    memcpy(out, rec, sizeof(ota_meta_t));

    return 1U;
}

/* Exported functions --------------------------------------------------------*/

void OtaMeta_Init(ota_meta_t *m)
{
    if (m == NULL)
    {
        return;
    }

    memset(m, 0, sizeof(*m));

    m->magic        = OTA_META_MAGIC;
    m->seq          = 0U;
    m->active_slot  = OTA_SLOT_A;
    m->boot_slot    = OTA_SLOT_A;
    m->pending_slot = OTA_SLOT_NONE;
    m->dl_slot      = OTA_SLOT_NONE;
    m->dl_active    = 0U;
}

int8_t OtaMeta_Load(ota_meta_t *out)
{
    ota_meta_t tmp;
    uint8_t    rec[OTA_META_RECORD_SIZE];
    uint8_t    found = 0U;
    uint32_t   i;

    s_next_rec = 0U;
    s_seq      = 0U;
    s_loaded   = 1U;
    s_dirty    = 0U;

    for (i = 0U; i < OTA_META_RECORD_COUNT; i++)
    {
        const uint8_t *p = (const uint8_t *)(uintptr_t)(OTA_META_BASE +
                                                        (i * OTA_META_RECORD_SIZE));

        memcpy(rec, p, sizeof(rec));

        if (ota_meta_valid(rec, &tmp) == 0U)
        {
            /* 无效记录分两种，处理方式完全不同：
               - 整条都是 0xFF：从没写过 → 干净的空位，下一句就往这里写；
               - 有数据但 CRC 不过：上次写到一半掉电 → 这一格**绝对不能再编程**
                 （重复编程 → ECC 双位错 → 一读就 NMI），标记成脏，
                   下一次 Write 先把整个扇区擦掉。 */
            if (ota_meta_has_data(rec) != 0U)
            {
                s_dirty = 1U;
            }

            s_next_rec = i;      /* 顺序追加：遇到第一条无效就停下，下一条写这里 */
            break;
        }

        if (out != NULL)
        {
            *out = tmp;
        }

        s_seq  = tmp.seq;
        found  = 1U;
    }

    if (i >= OTA_META_RECORD_COUNT)
    {
        s_next_rec = OTA_META_RECORD_COUNT;   /* 满了，下次 Write 会先擦扇区 */
    }

    if (found == 0U)
    {
        if (out != NULL)
        {
            OtaMeta_Init(out);
        }

        return 1;    /* 空的：首次上电 / 刚擦过 */
    }

    return 0;
}

int8_t OtaMeta_Write(ota_meta_t *m)
{
    uint8_t rec[OTA_META_RECORD_SIZE];

    if (m == NULL)
    {
        return OTA_E_PARAM;
    }

    /* 没扫描过就先扫描一遍，保证 s_next_rec/s_seq 是对的（别把老记录覆盖了） */
    if (s_loaded == 0U)
    {
        ota_meta_t tmp;

        (void)OtaMeta_Load(&tmp);
    }

    /* 需要先擦整个扇区的两种情况：
       ① 扫描时发现有"写了一半"的记录（脏）：那一格不能再用，后面的也不可信；
       ② 512 条全写满了。 */
    if ((s_dirty != 0U) || (s_next_rec >= OTA_META_RECORD_COUNT))
    {
        if (OtaFlash_EraseMeta() != OTA_OK)
        {
            return OTA_E_ERASE;
        }

        s_dirty    = 0U;
        s_next_rec = 0U;
    }

    s_seq++;
    m->magic = OTA_META_MAGIC;
    m->seq   = s_seq;

    memset(rec, 0xFF, sizeof(rec));
    memcpy(rec, m, sizeof(*m));

    /* 记录自身的 CRC32 放在最后 4 字节 */
    {
        uint32_t crc = Ota_Crc32(rec, OTA_META_CRC_OFFSET);

        memcpy(rec + OTA_META_CRC_OFFSET, &crc, sizeof(crc));
    }

    if (OtaFlash_Write(OTA_META_BASE + (s_next_rec * OTA_META_RECORD_SIZE),
                       rec, sizeof(rec)) != OTA_OK)
    {
        return OTA_E_WRITE;
    }

    s_next_rec++;

    return OTA_OK;
}

uint8_t OtaMeta_SlotValid(const ota_meta_t *m, uint8_t slot)
{
    if ((m == NULL) || (slot >= OTA_SLOT_COUNT))
    {
        return 0U;
    }

    return ((m->slot_size[slot] != 0U) && (m->slot_size[slot] <= OTA_SLOT_SIZE)) ? 1U : 0U;
}
