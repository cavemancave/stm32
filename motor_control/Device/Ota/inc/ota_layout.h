/**
  ******************************************************************************
  * @file    ota_layout.h
  * @brief   无线 OTA：Flash 分区表 + 常量 + 协议常量（Bootloader 和 App 共用）
  *
  * 完整设计见 docs/ota_design.md，这里只放"两边都要一致"的数字。
  *
  *  0x08000000 ┌──────────────────────────────┐
  *             │ s0  128K  Bootloader (BL)    │  永不 OTA 更新
  *  0x08020000 ├──────────────────────────────┤
  *             │ s1..s3     Slot A (384K)     │  App 之一（链接基址 A）
  *  0x08080000 ├──────────────────────────────┤
  *             │ s4..s6     Slot B (384K)     │  App 之二（同一份源码，换链接基址）
  *  0x080E0000 ├──────────────────────────────┤
  *             │ s7  128K   元数据（顺序追加） │  OTA 状态记录
  *  0x08100000 └──────────────────────────────┘
  *
  * ⚠ 改这里的任何数字都要同时改 STM32H723xG_slots.ld 里对应的 --defsym 用法，
  *   否则链接出来的镜像基址和协议里的分区表会对不上。
  ******************************************************************************
  */

#ifndef __OTA_LAYOUT_H__
#define __OTA_LAYOUT_H__

#include <stdint.h>

/* ---- Flash 基本参数（STM32H723VG：1 MB 单 bank，8 × 128 KB 扇区） ---- */
#define OTA_FLASH_BASE            0x08000000U
#define OTA_FLASH_END             0x08100000U
#define OTA_FLASH_SECTOR_SIZE     (128U * 1024U)
#define OTA_FLASH_SECTOR_COUNT    8U      /* 1 MB 单 bank = 8 × 128 KB */

/* ---- 分区 ---- */
#define OTA_BOOT_BASE             0x08000000U
#define OTA_BOOT_SIZE             (128U * 1024U)

#define OTA_SLOT_A_BASE           0x08020000U
#define OTA_SLOT_B_BASE           0x08080000U
#define OTA_SLOT_SIZE             (384U * 1024U)

#define OTA_META_BASE             0x080E0000U
#define OTA_META_SIZE             (128U * 1024U)

/* ---- 槽编号 ---- */
#define OTA_SLOT_A                0U
#define OTA_SLOT_B                1U
#define OTA_SLOT_COUNT            2U
#define OTA_SLOT_NONE             0xFFU

/* ---- 元数据记录 ---- */
#define OTA_META_MAGIC            0x4F54414DU   /* 'OTAM' */
#define OTA_META_RECORD_SIZE      256U
#define OTA_META_RECORD_COUNT     (OTA_META_SIZE / OTA_META_RECORD_SIZE)   /* 512 条/扇区 */
#define OTA_META_CRC_OFFSET       (OTA_META_RECORD_SIZE - 4U)              /* 252 */

/* ---- Flash 写法 ---- */
/* ⚠⚠ 编程单位（flash word）**必须跟芯片对上**：
   STM32H723 是 256 bit = **32 字节**（`stm32h723xx.h`：`FLASH_NB_32BITWORD_IN_FLASHWORD = 8`），
   不是 H7A3/B3 那种 128 bit = 16 字节。
   `HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, buf)`：
     - 一次写满**一个 flash word（32 字节）**，buf 必须给够 32 字节；
     - **addr 必须按 32 字节对齐** —— 不对齐会当场精确总线错误
       （CFSR=0x00008200、BFAR = 该 flash word 的基址）。
   ⚠ 2026-09-19 实测踩过：原来写成 WORD_SIZE=16、一块 32 字节里调两次 HAL，
     第二次就是往 0x080E0010（非 flash word 对齐）编程 → 写第一条元数据时
     App 直接 HardFault（PC 还因为诊断读到错栈而看起来像跳飞了）。
   所以：块大小 = word 大小 = 32，一块只调一次 HAL；最后不足 32 字节的补 0xFF。 */
#define OTA_FLASH_WRITE_ALIGN     32U   /* 调用方（下载会话）必须按它对齐 offset */
#define OTA_FLASH_WORD_SIZE       32U   /* 芯片一次能编程的单位 */

/* ---- 版本号（0xMMmmppp：主.次.补丁，只用来显示/比较，不参与逻辑） ---- */
#define OTA_FW_VERSION            0x00010000U   /* App 自己登记的版本（上位机 --version 可以覆盖） */
#define OTA_BL_VERSION            0x00010000U   /* Bootloader 版本（BL 不 OTA，改了要重新烧） */

/* OTA_INFO 回复里 flags 字节的 bit7：
   1 = 设备正跑在 Bootloader 恢复台里（没有 App 在运行）——
   上位机靠它知道「现在可以往任何一个槽写，包括元数据里的 active 槽」。 */
#define OTA_INFO_FLAG_BOOTLOADER  0x80U

/* ---- 启动/回滚 ---- */
#define OTA_MAX_BOOT_ATTEMPTS     3U        /* 新固件连续 3 次没"确认"就判坏、回滚 */
#define OTA_CONFIRM_DELAY_MS      2000U     /* App 跑够这么久才写"启动确认" */

/* ---- 下载会话 ----
   块大小是“链路速度”的主旋钮：每块都要等一个回复（一问一答），
   实测每块的往返延迟 ~140 ms，而且几乎和块大小无关 ⇒ 块越大吞吐越高。
   1024 → 4096 把往返次数除以 4。
   ⚠ 上限受两个东西约束：
     1) `OTA_PORT_RX_RING` 得装得下一整个编码后的帧（见下面的备注）；
     2) `OTA_LINK_*_MAX` 那几个静态缓冲会跟着涨（.bss，不吃 FreeRTOS 堆）。 */
#define OTA_CHUNK_MAX             4096U
#define OTA_CHUNK_DEFAULT         4096U
#define OTA_DL_SAVE_STEP          (32U * 1024U)   /* 每写这么多就落一次盘（续传用） */
#define OTA_SESSION_TIMEOUT_MS    30000U          /* 这么久没收到帧就放弃这次会话 */

/* ---- 无线口 ----
   **921600 = 115200 × 8**（2026-09-19 改）。H7 这边没精度问题：USART1 走 D2PCLK2 = 120 MHz，
   16 倍过采样下 USARTDIV = 130.2 → 实际 921748，误差 +0.02%。

   ⚠ 这个值必须和**无线模块的串口波特率**对得上，否则就是“设备听不见”：
     1) [已完成] 本宏 = 921600、已重新构建并烧进 App；
     2) [**接无线前要做**] 电机侧 + PC 侧两个模块的“串口波特率”都改成 921600。
        ⚠ 2026-09-19 调波特率时台架上接的是**有线**串口（链路上没有模块），
          所以当时“模块不用改、自己就跟上了”的说法**不成立** —— 别按那个结论去接无线；
     3) 升级固件时，如果设备跑的还是旧固件（旧波特率），那一次要 `--baud 115200`。
   ⚠ Bootloader **不参与 OTA**：它和 App 共用这个宏，所以**改了波特率必须用 ST-Link 重烧一次
     `motor_boot.hex`**（2026-09-19 已烧：现在 BL 恢复台也是 921600，升级时 BL 的 banner 能直接读）。
     没重烧的旧 BL 还在旧波特率，那时它的 banner 会是乱码、要进恢复台得加 `--baud <旧值>`。
   ⚠ 万一模块改不了/配错，App 会听不见（而 App 自己会照常确认启动、**不会**自动回滚）：
     把模块改回 115200，然后**按住 USER_KEY 上电**进 BL 控制台把旧固件刷回去。
   ⚠ 实测：瓶颈是“写 flash”以外的东西（每边扇区擦除 ~1 s、上位机每包白等 ~110 ms），
     不要指望 ×8 能快 8 倍 —— 真正的大头是把块大小 1024 加到 4096、只擦必要的扇区
     （见上面 OTA_CHUNK_* 和 ota_flash.c 的 OtaFlash_EraseSlotForSize） */
#define OTA_PORT_BAUD             921600
/* ⚠ 上面故意**不带 U 后缀**：日志里是 OTA_STR(OTA_PORT_BAUD)，宏参数会被原样字符串化，
   带后缀就会打出 "921600U"。（这个值只往 uint32_t 里赋，有没有 U 都一样） */
#define OTA_PORT_RX_RING          8192U     /* 软件环形缓冲（硬件还有 16 字节 FIFO）。
                                               必须 ≥ 一帧编码后的长度 + 2：
                                               11 + (4 + CHUNK) 再乘 COBS 膨胀（+1/254），
                                               4096 块 ≈ 4131 字节 */
#define OTA_PORT_TX_TIMEOUT_MS    500U

/* 把宏的数值变成字符串，给日志用：OTA_STR(OTA_PORT_BAUD) → "921600" */
#define OTA_STR_INNER(x)          #x
#define OTA_STR(x)                OTA_STR_INNER(x)

/* ---- 协议版本/同步头/帧开销 ---- */
#define OTA_PROTO_VER             0x01U
#define OTA_FRAME_SYNC0           0xAAU
#define OTA_FRAME_SYNC1           0x55U
/* 帧头尾一共多少字节：SYNC(2) + VER(1) + TYPE(1) + SEQ(1) + LEN(2) + CRC32(4) */
#define OTA_FRAME_OVERHEAD        11U
#define OTA_FRAME_MAX_PAYLOAD     (4U + OTA_CHUNK_MAX)   /* OTA_DATA = off(4) + 数据 */

/* ---- 帧类型（主机 → 设备；回复 = 请求 | 0x80） ---- */
enum
{
    OTA_T_INFO      = 0x01U,   /* 查询分区/版本状态 */
    OTA_T_BEGIN     = 0x02U,   /* 开始一次下载：slot,size,crc32,ver,flags */
    OTA_T_DATA      = 0x03U,   /* 数据块：off(4) + data */
    OTA_T_END       = 0x04U,   /* 结束并校验、提交 */
    OTA_T_ABORT     = 0x05U,   /* 放弃本次会话（保留续传进度） */
    OTA_T_REBOOT    = 0x06U,   /* 重启：mode 0=正常 1=进 BL 恢复台 */
    OTA_T_ERASE     = 0x07U,   /* 擦除某个槽 */
    OTA_T_ROLLBACK  = 0x08U,   /* 切回另一个槽并重启 */
    OTA_T_CTRL      = 0x10U,   /* 控制命令（电机等） */
    OTA_T_STATUS    = 0x11U,   /* 只读状态快照 */
    OTA_T_LOG       = 0x20U    /* 设备 → 主机：结构化日志（默认不用，日志走裸文本） */
};

/* CTRL 子命令（OTA_T_CTRL 的 payload[0]）—— OTA 平时就是靠这个传控制信息 */
enum
{
    OTA_CTRL_DISABLE      = 0x00U,   /* 失能所有电机 */
    OTA_CTRL_ENABLE       = 0x01U,   /* 使能（会重试） */
    OTA_CTRL_POS_LOOP     = 0x02U,   /* 切位置环（0xA0/0x03） */
    OTA_CTRL_MOVE_POS     = 0x03U,   /* 走位置：arg = 0..32767 */
    OTA_CTRL_MOVE_DEG     = 0x04U,   /* 走角度：arg = 0..359 */
    OTA_CTRL_STOP         = 0x05U,   /* 急停（0x64 给定值 = 0） */
    OTA_CTRL_LOG_MUTE     = 0x06U,   /* 静音/恢复日志：arg = 0/1 */
    OTA_CTRL_POLL_PAUSE   = 0x07U,   /* 暂停/恢复 200 ms 状态轮询：arg = 0/1 */
    OTA_CTRL_PWR          = 0x08U,   /* 电机电源（PC14 可控电源输出）：arg = 0 断电 / 1 上电 / 2 断电重启 */
    OTA_CTRL_VMON         = 0x09U,   /* 读电源电压（PC4/ADC1_INP4 分压取样）：arg 保留（0），
                                        回复 data = 总压 mV、data2 = 电池串数、data3 = 剩余百分比
                                        （0..100，0xFF=无效）。
                                        **不是电机命令**，所以升级/失能时也能问 */
    OTA_CTRL_CELLS        = 0x0AU    /* 设置电池串数（算"单片电压"/百分比用，1..8）：arg = 串数，
                                        回复 data = 生效后的串数。
                                        6 = 6S 电池、3 = 12V 那套（当 3S 算），见 power_mon.h */
};

/* OTA_T_CTRL 回复的 payload 布局（13 字节）：
 *   [0]     = OTA_OK / OTA_E_xxx（见下面状态码）
 *   [1..4]  = data  （各命令自己定：状态值 / 电压 mV / 串数 ...）
 *   [5..8]  = data2 （后加的第二个 32 位，用不上时为 0）
 *   [9..12] = data3 （只用得上一个参数的命令在 data2 里传；再不够才动 data3，
 *                    目前只有 vmon 用它（剩余百分比 0..100，0xFF=无效））
 * ⚠ data2/data3 都只加在**后面**：老上位机（只读前 5 字节）照样能用。 */

/* ---- 状态码：回复帧 payload[0] ---- */
enum
{
    OTA_OK            = 0,   /* 成功 */
    OTA_E_BADFRAME    = 1,   /* 帧格式/CRC 不对（设备侧一般不会回这个，直接丢帧） */
    OTA_E_STATE       = 2,   /* 状态机不允许（比如没 BEGIN 就 DATA） */
    OTA_E_OFFSET      = 3,   /* 偏移不对（回复里带上期望的偏移） */
    OTA_E_ERASE       = 4,   /* Flash 擦除失败 */
    OTA_E_WRITE       = 5,   /* Flash 写入失败 */
    OTA_E_CRC         = 6,   /* 整包 CRC32 不匹配 */
    OTA_E_PARAM       = 7,   /* 参数非法（长度/槽号/越界） */
    OTA_E_NOTALLOWED  = 8    /* 不允许（比如想写正在运行的那个槽） */
};

/* ---- 元数据记录（256 字节一条，尾部 4 字节是自身 CRC32） ----
   顺序追加写：掉电只会让"最后一条"CRC 不过，扫描时遇到第一条无效记录就停，
   自然回退到上一次提交点 —— 这是不用文件系统也能有原子性的做法。 */
typedef struct
{
    uint32_t magic;             /* OTA_META_MAGIC */
    uint32_t seq;               /* 递增序号 */
    uint32_t fw_version;        /* 当前 active 槽的固件版本（04 字节） */

    uint8_t  active_slot;       /* App 最后一次确认过的槽 */
    uint8_t  boot_slot;         /* 下次启动跳哪个槽 */
    uint8_t  boot_attempts;     /* 已连续跳进 boot_slot 但还没被确认的次数 */
    uint8_t  flags;             /* bit0 = 本次重启停在 Bootloader */

    uint32_t slot_size[OTA_SLOT_COUNT];   /* 各槽镜像长度（0 = 未知/未登记） */
    uint32_t slot_crc[OTA_SLOT_COUNT];    /* 各槽镜像 CRC32 */
    uint32_t slot_ver[OTA_SLOT_COUNT];    /* 各槽镜像版本号 */

    uint8_t  pending_slot;      /* OTA_SLOT_NONE = 无待切换镜像 */
    uint8_t  dl_slot;           /* 下载会话目标槽 */
    uint8_t  dl_active;         /* 下载会话进行中（用于断链/掉电续传） */
    uint8_t  reserved0;

    uint32_t dl_size;           /* 下载总长度 */
    uint32_t dl_crc;            /* 下载目标 CRC32 */
    uint32_t dl_offset;         /* 已写入字节数（32 字节对齐） */

    uint32_t reserved[8];       /* 将来用（参数区/签名……） */
} ota_meta_t;

/* 记录本体必须塞得下（前面是结构体，后面留 4 字节 CRC32） */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(ota_meta_t) <= OTA_META_CRC_OFFSET,
               "ota_meta_t 太大了：OTA_META_CRC_OFFSET 之前必须放得下");
#endif

/* ---- 小工具 ---- */
static inline uint32_t Ota_SlotBase(uint8_t slot)
{
    return (slot == OTA_SLOT_B) ? OTA_SLOT_B_BASE : OTA_SLOT_A_BASE;
}

/* 给定地址落在哪个槽；不是槽的话返回 OTA_SLOT_NONE */
static inline uint8_t Ota_SlotOfAddress(uint32_t addr)
{
    if ((addr >= OTA_SLOT_A_BASE) && (addr < (OTA_SLOT_A_BASE + OTA_SLOT_SIZE)))
    {
        return OTA_SLOT_A;
    }
    if ((addr >= OTA_SLOT_B_BASE) && (addr < (OTA_SLOT_B_BASE + OTA_SLOT_SIZE)))
    {
        return OTA_SLOT_B;
    }
    return OTA_SLOT_NONE;
}

static inline uint8_t Ota_OtherSlot(uint8_t slot)
{
    return (slot == OTA_SLOT_A) ? OTA_SLOT_B : OTA_SLOT_A;
}

/* ---------------------------------------------------------------------------
 * App 自己的信息：由链接脚本给出（见 STM32H723xG_slots.ld）
 *   __app_base       : 本镜像链接的槽基址（0x08020000 或 0x08080000）
 *   __app_image_size : 本镜像在 Flash 里占多少字节（== objcopy 出来的 .bin 大小）
 * 这两个是"绝对符号"，用 (uint32_t)&符号 取到的是它们的**值**。
 * ------------------------------------------------------------------------- */
extern const uint32_t __app_base;
extern const uint32_t __app_image_size;

#define OTA_APP_BASE        ((uint32_t)(uintptr_t)&__app_base)
#define OTA_APP_IMAGE_SIZE  ((uint32_t)(uintptr_t)&__app_image_size)
#define OTA_APP_SLOT        (Ota_SlotOfAddress(OTA_APP_BASE))

#endif /* __OTA_LAYOUT_H__ */
