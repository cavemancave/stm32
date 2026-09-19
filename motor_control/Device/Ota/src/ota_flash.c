/**
  ******************************************************************************
  * @file    ota_flash.c
  * @brief   内部 Flash 擦/写（HAL_FLASH + HAL_FLASHEx）
  ******************************************************************************
  */

#include "ota_flash.h"
#include "ota_crc.h"

#include "stm32h7xx_hal.h"

#include <string.h>

/* 扇区号 ↔ 槽 的对应关系（见 ota_layout.h 的图）：
   s0 = BL、s1..s3 = A、s4..s6 = B、s7 = 元数据 */
#define OTA_SLOT_SECTOR_FIRST   { FLASH_SECTOR_1, FLASH_SECTOR_4 }
#define OTA_SLOT_SECTOR_COUNT   3U
#define OTA_META_SECTOR         FLASH_SECTOR_7

/* Private functions ---------------------------------------------------------*/

static uint32_t s_last_erase_sectors;    /* 最近一次槽擦除擦了几个扇区（只给日志用） */

static void ota_flash_clear_errors(void);   /* 下面几个 helper 定义在后面 */

/* 从槽的第一个扇区开始擦 n 个扇区（n 必须是 1..OTA_SLOT_SECTOR_COUNT） */
static int8_t ota_flash_erase_slot_sectors(uint8_t slot, uint32_t n)
{
    static const uint32_t first_sector[OTA_SLOT_COUNT] = OTA_SLOT_SECTOR_FIRST;
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t               sector_error = 0U;
    HAL_StatusTypeDef      st;

    if ((slot >= OTA_SLOT_COUNT) || (n == 0U) || (n > OTA_SLOT_SECTOR_COUNT))
    {
        return OTA_E_PARAM;
    }

    s_last_erase_sectors = n;

    (void)HAL_FLASH_Unlock();
    ota_flash_clear_errors();

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Banks        = FLASH_BANK_1;
    erase.Sector       = first_sector[slot];
    erase.NbSectors    = n;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;   /* VDD 2.7~3.6 V（板子 3.3 V） */

    st = HAL_FLASHEx_Erase(&erase, &sector_error);
    (void)HAL_FLASH_Lock();

    return (st == HAL_OK) ? (int8_t)OTA_OK : (int8_t)OTA_E_ERASE;
}

/* 擦之前/写之前都要清一次错误标志：H7 的 ECC/写保护错误标志会粘住，
   不清的话后面每次操作都会直接返回错误 */
static void ota_flash_clear_errors(void)
{
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);
}

/* 这一个 flash word（32 字节）是空的（全 0xFF）吗？

   ⚠⚠ 为什么编程前必须问这一句：往**没擦过**的地方再编程一次叫"重复编程"，
   H7 的 flash 会因此把这一格的 ECC 弄成双位错 —— 以后**只要读它就 NMI +
   精确总线错误**，而且这个错擦之前永远不会消失。
   2026-09-19 实测后果：元数据扇区的第一个 flash word 就这么坏的，
   而 BL 是在打横幅**之前**读元数据的 → 上电后串口一个字符都没有（最难查的形）。
   代价是每次编程前多读 32 字节（传输 96 KB 总共也就多读 96 KB，忽略不计）。 */
static uint8_t ota_flash_is_blank_word(uint32_t addr)
{
    const uint32_t *p = (const uint32_t *)(uintptr_t)addr;

    for (uint32_t i = 0U; i < (OTA_FLASH_WORD_SIZE / 4U); i++)
    {
        if (p[i] != 0xFFFFFFFFU)
        {
            return 0U;
        }
    }

    return 1U;
}

/* Exported functions --------------------------------------------------------*/

int8_t OtaFlash_EraseSlot(uint8_t slot)
{
    return ota_flash_erase_slot_sectors(slot, OTA_SLOT_SECTOR_COUNT);
}

int8_t OtaFlash_EraseSlotForSize(uint8_t slot, uint32_t size)
{
    /* 只擦镜像真正会占用的那些扇区：ceil(size / 128 KB)，1~3 个。

       为什么可以不全擦（2026-09-19 实测 + 代码核对）：
         - 总是从槽的**第一个**扇区开始往上擦 ⇒ 槽开头的向量表和 64 字节空白采样
           （OtaFlash_IsBlank 就是看开头）都是干净的；
         - BL 判断镜像好坏用的是元数据里的 size/crc，OtaFlash_SlotCrc 只读 size 字节，
           镜像尾巴后面剩下的旧数据根本没人读；
         - 回滚/切槽靠的是**另一个**槽，我们不动它。
       好处：一个 113 KB 的镜像从“擦 3 个扇区（~2.9 s）”变成“擦 1 个（~1 s）”。
       实测扇区擦除 0.97 s/个（384 KB / 3 个 ≈ 2.9 s），是整次升级里最大的一块时间。 */
    uint32_t n = (size + OTA_FLASH_SECTOR_SIZE - 1U) / OTA_FLASH_SECTOR_SIZE;

    if (n == 0U)
    {
        n = 1U;      /* size = 0 时也留一个扇区，别把参数错误变成无声的“什么都没干” */
    }

    if (n > OTA_SLOT_SECTOR_COUNT)
    {
        n = OTA_SLOT_SECTOR_COUNT;
    }

    return ota_flash_erase_slot_sectors(slot, n);
}

uint32_t OtaFlash_LastEraseSectors(void)
{
    return s_last_erase_sectors;     /* 给日志用的，看这次到底擦了几个扇区 */
}

int8_t OtaFlash_EraseMeta(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t               sector_error = 0U;
    HAL_StatusTypeDef      st;

    (void)HAL_FLASH_Unlock();
    ota_flash_clear_errors();

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Banks        = FLASH_BANK_1;
    erase.Sector       = OTA_META_SECTOR;
    erase.NbSectors    = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    st = HAL_FLASHEx_Erase(&erase, &sector_error);
    (void)HAL_FLASH_Lock();

    return (st == HAL_OK) ? (int8_t)OTA_OK : (int8_t)OTA_E_ERASE;
}

int8_t OtaFlash_EraseSector(uint32_t sector)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t               sector_error = 0U;
    HAL_StatusTypeDef      st;

    /* 0 号扇区就是 Bootloader 自己：擦了板子就真没救了，直接拒绝 */
    if ((sector == 0U) || (sector >= OTA_FLASH_SECTOR_COUNT))
    {
        return OTA_E_PARAM;
    }

    (void)HAL_FLASH_Unlock();
    ota_flash_clear_errors();

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Banks        = FLASH_BANK_1;
    erase.Sector       = sector;
    erase.NbSectors    = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    st = HAL_FLASHEx_Erase(&erase, &sector_error);
    (void)HAL_FLASH_Lock();

    return (st == HAL_OK) ? (int8_t)OTA_OK : (int8_t)OTA_E_ERASE;
}

int8_t OtaFlash_Write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    /* 32 字节对齐的落地缓冲：HAL 的 FLASHWORD 编程一次从 buf 搬走整个 flash word（32 字节） */
    uint8_t word[OTA_FLASH_WORD_SIZE] __attribute__((aligned(OTA_FLASH_WRITE_ALIGN)));

    if ((data == NULL) || (len == 0U) ||
        ((addr & (OTA_FLASH_WRITE_ALIGN - 1U)) != 0U))
    {
        return OTA_E_PARAM;
    }

    /* 越界保护：只允许写"两个槽"和"元数据扇区"（防止参数算错把 BL 抹了） */
    if (!(((addr >= OTA_SLOT_A_BASE) && ((addr + len) <= (OTA_SLOT_B_BASE + OTA_SLOT_SIZE))) ||
          ((addr >= OTA_META_BASE) && ((addr + len) <= (OTA_META_BASE + OTA_META_SIZE)))))
    {
        return OTA_E_PARAM;
    }

    /* 没解锁就别去写：直接 store 到 Flash 地址 = 精确总线错误（比起"默默写坏"还不如直接报错） */
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return OTA_E_WRITE;
    }

    while (len > 0U)
    {
        uint32_t n = (len >= OTA_FLASH_WORD_SIZE) ? OTA_FLASH_WORD_SIZE : len;

        memset(word, 0xFF, sizeof(word));
        memcpy(word, data, n);

        /* ⚠ 编程前先确认这一格是空的：往写过的 flash word 上再编程会把它的 ECC 弄成
           双位错，以后一读就 NMI（详见 ota_flash_is_blank_word 的注释）。
           不是空的就直接报错让上层处理（擦掉重来 / 报告给主机），绝不硬写。 */
        if (ota_flash_is_blank_word(addr) == 0U)
        {
            (void)HAL_FLASH_Lock();
            return OTA_E_WRITE;
        }

        ota_flash_clear_errors();

        /* 一次调用 = 一个 flash word，addr 始终是 32 字节对齐的（下面按 WORD_SIZE 步进）
           ⚠ 绝不能写成 "for (i = 0; i < ALIGN; i += WORD_SIZE)"：WORD_SIZE 一旦比 ALIGN 小，
             第二次调用的地址就不对齐了 → 精确总线错误（BFAR = 该 flash word 基址）。

           实测（DWT 周期计数，2026-09-19）：blank 检查 3.2 µs/字、HAL_FLASH_Program
           98 µs/字 ⇒ 113 KB 镜像总共只花 0.35 s，写 flash 从来就不是瓶颈 ——
           真正的大头是 BEGIN 里的扇区擦除（0.97 s/扇区），见 OtaFlash_EraseSlotForSize。 */
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr,
                              (uint32_t)(uintptr_t)word) != HAL_OK)
        {
            (void)HAL_FLASH_Lock();
            return OTA_E_WRITE;
        }

        addr += OTA_FLASH_WORD_SIZE;
        data += n;
        len  -= n;
    }

    (void)HAL_FLASH_Lock();

    return OTA_OK;
}

int8_t OtaFlash_SlotCrc(uint8_t slot, uint32_t size, uint32_t *crc)
{
    const uint8_t *p;
    uint32_t       c;
    uint32_t       left;

    if ((slot >= OTA_SLOT_COUNT) || (crc == NULL) ||
        (size == 0U) || (size > OTA_SLOT_SIZE))
    {
        return OTA_E_PARAM;
    }

    p    = (const uint8_t *)(uintptr_t)Ota_SlotBase(slot);
    c    = OTA_CRC32_INIT;
    left = size;

    while (left > 0U)
    {
        uint32_t n = (left > 1024U) ? 1024U : left;

        c    = Ota_Crc32Update(c, p, n);
        p   += n;
        left -= n;
    }

    *crc = Ota_Crc32Final(c);

    return OTA_OK;
}

uint8_t OtaFlash_IsBlank(uint8_t slot)
{
    const uint8_t *p;
    uint32_t       i;

    if (slot >= OTA_SLOT_COUNT)
    {
        return 0U;
    }

    p = (const uint8_t *)(uintptr_t)Ota_SlotBase(slot);

    for (i = 0U; i < 64U; i++)
    {
        if (p[i] != 0xFFU)
        {
            return 0U;
        }
    }

    return 1U;
}

uint8_t OtaFlash_ImageLooksValid(uint8_t slot)
{
    const uint32_t *vec;
    uint32_t        sp;
    uint32_t        pc;

    if (slot >= OTA_SLOT_COUNT)
    {
        return 0U;
    }

    vec = (const uint32_t *)(uintptr_t)Ota_SlotBase(slot);
    sp  = vec[0];
    pc  = vec[1];

    /* 栈顶：H7 的 RAM 全在 0x20000000~0x3FFFFFFF（DTCM/AXI/D2/D3），且 4 字节对齐 */
    if ((sp < 0x20000000U) || (sp > 0x3FFFFFFFU) || ((sp & 0x3U) != 0U))
    {
        return 0U;
    }

    /* 复位入口：必须是槽内的奇数地址（Thumb） */
    if (((pc & 0x1U) == 0U) || (Ota_SlotOfAddress(pc & ~1U) != slot))
    {
        return 0U;
    }

    return 1U;
}
