/**
  ******************************************************************************
  * @file    ota_boot.c
  * @brief   Bootloader：Flash 最前面的 128 KB（0x08000000），上电第一段代码
  *
  * 职责（详见 docs/ota_design.md §3）：
  *   1. 读元数据 → 决定跳哪个槽（A/B），跳之前每次都校验一遍那个槽的 CRC32；
  *   2. 有待切换的新镜像（pending_slot）→ 校验通过就切换 + 落盘；
  *   3. 新固件连续 OTA_MAX_BOOT_ATTEMPTS 次没被 App"确认"→ 判坏、回滚到另一个槽；
  *   4. 两个槽都不能跑（第一次烧写、或者都坏了）→ 进**恢复台**：
  *      在同一个串口上用同一套帧协议收固件，指定写哪个槽。
  *
  * 为什么 BL 不做自升级：H723 单 bank 没有硬件 bank swap，BL 自更新一失败就真变砖了。
  * 它只在第一次用 ST-Link 烧一次，之后永不改变 —— 这是整个方案的地基。
  *
  * ⚠ 这个文件**不链接** FreeRTOS/App 的任何东西，只依赖 HAL + Device/Ota 的公共层。
  ******************************************************************************
  */

#include "ota_layout.h"
#include "ota_com.h"
#include "ota_link.h"
#include "ota_host.h"
#include "ota_session.h"
#include "ota_meta.h"
#include "ota_flash.h"

#include "main.h"          /* USER_KEY_Pin / USER_KEY_GPIO_Port（PA15） */
#include "stm32h7xx_hal.h"

#include <stdio.h>
#include <string.h>

/* 恢复台空转多久没收到任何帧就尝试启动（有能跑的槽时），单位 ms */
#define BL_CONSOLE_IDLE_MS      15000U

/* Private function prototypes -----------------------------------------------*/

static void bl_print(const char *text);
static void bl_clock_config(void);
static uint8_t bl_key_held(void);
static void bl_on_frame(uint8_t type, uint8_t seq, const uint8_t *payload, uint16_t len);
static uint8_t bl_pick_slot(const ota_meta_t *m, uint8_t *crc_ok);
static void __attribute__((noreturn)) bl_prepare_and_boot(ota_meta_t *m, uint8_t slot);
static void __attribute__((noreturn)) bl_start_app(uint8_t slot);
static void __attribute__((noreturn)) bl_console(ota_meta_t *m);

/* 异常处理 + 自愈（BL 里没有 stm32h7xx_it.c，startup 里那些弱定义会掉进死循环） */
static void __attribute__((noreturn)) bl_fault(const char *tag);
static void bl_raw_text(const char *text);
static void bl_raw_hex32(uint32_t value);
static void bl_raw_dec(uint32_t value);

/* Private variables ---------------------------------------------------------*/

static const ota_host_hooks_t s_bl_hooks =
{
    .get_running_slot  = NULL,      /* BL 不"跑"在任何槽上 */
    .check_target_slot = NULL,      /* 哪个槽都能写（恢复台就是来覆盖坏槽的） */
    .on_session_start  = NULL,
    .on_session_stop   = NULL,
    .log               = bl_print,
};

/* Private functions ---------------------------------------------------------*/

static void bl_print(const char *text)
{
    if (text != NULL)
    {
        OtaCom_Write((const uint8_t *)text, (uint32_t)strlen(text));
    }
}

/* ---- 故障现场 + 自愈（BL 专用）------------------------------------------ */

/*
 * ⚠⚠ BL 里**没有** Core/Src/stm32h7xx_it.c，startup 里的 NMI/HardFault/BusFault
 *   入口全是**弱定义**（死循环）。于是"某个 flash word 坏了"这种故障的表现就是
 *   **上电后串口一个字符都没有** —— 最难查的一种形（2026-09-19 实测被坑了一轮）。
 *
 * 最常见的坏法：往没擦过的地方重复编程 → 那个 flash word 的 ECC 变双位错 →
 *   **一读它就 NMI + 精确总线错误，BFAR 就是它所在的地址**。
 * 处理办法：把 BFAR 所在的那个扇区整块擦掉再复位。这正是 A/B 双槽的意义 ——
 *   坏的那个槽整块扔掉就行，另一个还能跑；元数据扇区坏了也无所谓（App 会重新登记）。
 *   只有 0 号扇区（BL 自己）永远不擦，擦了就真成砖了。
 */

/* 自愈计数器：故意放在 DTCM 里一段"启动代码不会清零"的空地上
   （.bss 结束 0x20006508 ～ 栈顶 0x20020000 之间没人用）。
   NVIC_SystemReset() 不清 RAM，所以它能跨复位保留，用来防止
   "擦扇区 → 复位 → 又坏" 的死循环；真掉电才会清。 */
#define BL_HEAL_COUNT_ADDR   0x20007000U
#define BL_HEAL_MAX          3U
static volatile uint32_t *const s_heal_count = (volatile uint32_t *)BL_HEAL_COUNT_ADDR;

/*
 * 故障时的打印：**不能**用 bl_print。HAL_UART_Transmit 的超时靠 HAL_GetTick()，
 * 而异常处理的优先级比 TIM6 高 → tick 永远不会来 → 那个等待永远出不去，
 * 现场就什么也打不出来了。所以这里直接怼 USART1 寄存器，
 * 所有等待都是有界自旋（USART1 的寄存器早在 OtaCom_Init() 里就配好了）。
 */
static void bl_raw_char(char c)
{
    uint32_t spin = 200000U;

    while (((USART1->ISR & USART_ISR_TXE_TXFNF) == 0U) && (spin > 0U))
    {
        spin--;
    }

    USART1->TDR = (uint32_t)(uint8_t)c;
}

static void bl_raw_text(const char *text)
{
    while ((text != NULL) && (*text != '\0'))
    {
        bl_raw_char(*text);
        text++;
    }
}

static void bl_raw_hex32(uint32_t value)
{
    static const char digits[] = "0123456789ABCDEF";

    bl_raw_text("0x");

    for (int i = 28; i >= 0; i -= 4)
    {
        bl_raw_char(digits[(value >> (uint32_t)i) & 0xFU]);
    }
}

static void bl_raw_dec(uint32_t value)
{
    char     buf[10];
    uint32_t n = 0U;

    if (value == 0U)
    {
        bl_raw_char('0');
        return;
    }

    while ((value > 0U) && (n < sizeof(buf)))
    {
        buf[n] = (char)('0' + (char)(value % 10U));
        value /= 10U;
        n++;
    }

    while (n > 0U)
    {
        n--;
        bl_raw_char(buf[n]);
    }
}

static void __attribute__((noreturn)) bl_fault(const char *tag)
{
    const uint32_t flash_base = OTA_FLASH_END - (OTA_FLASH_SECTOR_COUNT * OTA_FLASH_SECTOR_SIZE);
    uint32_t       cfsr = SCB->CFSR;
    uint32_t       bfar = SCB->BFAR;
    uint32_t       sector = 0U;
    uint8_t        can_heal = 0U;
    int8_t         st;

    __disable_irq();

    bl_raw_text("\r\n*** BL FAULT: ");
    bl_raw_text((tag != NULL) ? tag : "?");
    bl_raw_text(" ***\r\n  CFSR=");
    bl_raw_hex32(cfsr);
    bl_raw_text(" HFSR=");
    bl_raw_hex32(SCB->HFSR);
    bl_raw_text(" BFAR=");
    bl_raw_hex32(bfar);

    /* 精确总线错误（BFSR.PRECISERR, bit9）+ BFAR 有效（BFSR.BFARVALID, bit15）
       + 地址落在 Flash 里 = 那个 flash word 坏了（典型：重复编程 → ECC 双错） */
    if (((cfsr & (1U << 9)) != 0U) && ((cfsr & (1U << 15)) != 0U) &&
        (bfar >= flash_base) && (bfar < OTA_FLASH_END))
    {
        sector = (bfar - flash_base) / OTA_FLASH_SECTOR_SIZE;

        /* 0 号扇区是 BL 自己：擦了真没救了，只报警 */
        can_heal = (sector != 0U) ? 1U : 0U;
    }

    if (can_heal != 0U)
    {
        bl_raw_text("\r\n  → 扇区 S");
        bl_raw_dec(sector);
        bl_raw_text(" 里有坏掉的 flash word");

        if (*s_heal_count < BL_HEAL_MAX)
        {
            (*s_heal_count)++;

            bl_raw_text("，正在擦掉它并复位（第 ");
            bl_raw_dec(*s_heal_count);
            bl_raw_text("/3 次）...\r\n");

            st = OtaFlash_EraseSector(sector);

            if (st == OTA_OK)
            {
                bl_raw_text("  → 擦除成功，复位后应该能正常启动\r\n");
                NVIC_SystemReset();
            }

            bl_raw_text("  → 擦除失败（st=");
            bl_raw_dec((uint32_t)(uint8_t)st);
            bl_raw_text("），只能靠 ST-Link 全片擦除了\r\n");
        }
        else
        {
            bl_raw_text("，但自愈已经连续失败 3 次，不再折腾了\r\n");
        }
    }

    bl_raw_text("  → 停在 Bootloader（重新上电可再试）\r\n");

    for (;;)
    {
    }
}

/* 这四个是 startup 里的弱符号：BL 里由我们接管，避免"一声不响" */
void NMI_Handler(void)        { bl_fault("NMI"); }
void HardFault_Handler(void)  { bl_fault("HardFault"); }
void MemManage_Handler(void)  { bl_fault("MemManage"); }
void BusFault_Handler(void)   { bl_fault("BusFault"); }
void UsageFault_Handler(void) { bl_fault("UsageFault"); }

/*
 * 时钟：App 的 SystemClock_Config() 的拷贝（见 Core/Src/main.c）。
 * ⚠ 两处必须一致：串口波特率是按 PCLK 算出来的，不一致就会通信乱码。
 *   改 App 的时钟时这里要一起改。
 * 和 App 不同的是：这里**失败不 Error_Handler**，退回默认时钟继续跑 ——
 * 恢复台能通信比跑得快重要。
 */
static void bl_clock_config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    uint32_t           wait;

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    wait = 1000000U;
    while ((__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY) == 0U) && (wait > 0U))
    {
        wait--;
    }

    if (wait == 0U)
    {
        SystemCoreClockUpdate();
        return;
    }

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState            = RCC_HSE_ON;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM            = 2;
    osc.PLL.PLLN            = 40;
    osc.PLL.PLLP            = 1;
    osc.PLL.PLLQ            = 2;
    osc.PLL.PLLR            = 2;
    osc.PLL.PLLRGE          = RCC_PLL1VCIRANGE_3;
    osc.PLL.PLLVCOSEL       = RCC_PLL1VCOWIDE;
    osc.PLL.PLLFRACN        = 0;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
        SystemCoreClockUpdate();     /* HSE 没起振：用 HSI 也能当恢复台 */
        return;
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                         RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    clk.AHBCLKDivider  = RCC_HCLK_DIV2;
    clk.APB3CLKDivider = RCC_APB3_DIV2;
    clk.APB1CLKDivider = RCC_APB1_DIV2;
    clk.APB2CLKDivider = RCC_APB2_DIV2;
    clk.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK)
    {
        SystemCoreClockUpdate();
    }
}

/*
 * 挑一个能启动的槽：
 *   order[0] = 元数据里指定的 boot_slot，order[1] = 另一个（回滚/兜底）
 * 判定优先级：
 *   ① 元数据登记过 size+crc，而且读回算出来的 CRC32 对得上 → 干净（crc_ok=1）
 *   ② 元数据对不上，但向量表看着像 App → 还是让它跑（crc_ok=0）
 *      —— 覆盖"刚用 ST-Link 重烧过 / 元数据是旧的"这种情况；
 *         真要是半途写坏的镜像，App 起不来 → Bootloader 数到 3 次会回滚。
 *   ③ 空槽（全 0xFF）→ 跳过
 */
static uint8_t bl_pick_slot(const ota_meta_t *m, uint8_t *crc_ok)
{
    uint8_t first = (m->boot_slot < OTA_SLOT_COUNT) ? m->boot_slot : OTA_SLOT_A;
    uint8_t order[2];

    order[0] = first;
    order[1] = Ota_OtherSlot(first);

    for (uint32_t i = 0U; i < 2U; i++)
    {
        uint8_t slot = order[i];

        if (OtaFlash_IsBlank(slot) != 0U)
        {
            continue;
        }

        if (OtaMeta_SlotValid(m, slot) != 0U)
        {
            uint32_t crc = 0U;

            if ((OtaFlash_SlotCrc(slot, m->slot_size[slot], &crc) == OTA_OK) &&
                (crc == m->slot_crc[slot]))
            {
                *crc_ok = 1U;
                return slot;
            }
        }

        if (OtaFlash_ImageLooksValid(slot) != 0U)
        {
            *crc_ok = 0U;
            return slot;
        }
    }

    return OTA_SLOT_NONE;
}

/*
 * 跳转前把"这次启动"记下来（attempts++），App 跑起来 2 s 后会把 attempts 清 0。
 * 如果它一直不清，说明新固件跑不起来，下一次复位就会走到回滚分支。
 */
static void __attribute__((noreturn)) bl_prepare_and_boot(ota_meta_t *m, uint8_t slot)
{
    char line[96];

    if (m->boot_slot != slot)
    {
        m->boot_slot     = slot;      /* 从另一个槽兜底启动：计数重新开始 */
        m->boot_attempts = 1U;
    }
    else if (m->boot_attempts < 250U)
    {
        m->boot_attempts++;
    }

    m->flags = (uint8_t)(m->flags & (uint8_t)~0x01U);   /* 已经决定启动：清"停在 BL"标志 */
    (void)OtaMeta_Write(m);

    (void)snprintf(line, sizeof(line),
                   "bootloader: boot slot %c (attempt %u/%u) ...\r\n",
                   (slot == OTA_SLOT_A) ? 'A' : 'B',
                   (unsigned int)m->boot_attempts, (unsigned int)OTA_MAX_BOOT_ATTEMPTS);
    bl_print(line);

    HAL_Delay(30);        /* 让串口把这行发完 */

    bl_start_app(slot);
}

static void __attribute__((noreturn)) bl_start_app(uint8_t slot)
{
    uint32_t base  = Ota_SlotBase(slot);
    uint32_t sp    = *(volatile uint32_t *)(uintptr_t)base;
    uint32_t pc    = *(volatile uint32_t *)(uintptr_t)(base + 4U);
    void   (*entry)(void) = (void (*)(void))(uintptr_t)pc;
    char     line[96];
    uint32_t i;

    /* 把 App 向量表的头两个字（栈顶 / 复位入口）打出来：跳过去之后要是没反应，
       看这一行就知道镜像和槽对不对得上 */
    (void)snprintf(line, sizeof(line),
                   "bootloader: jump slot %c  SP=0x%08lX PC=0x%08lX\r\n",
                   (slot == OTA_SLOT_A) ? 'A' : 'B',
                   (unsigned long)sp, (unsigned long)pc);
    bl_print(line);

    /* ⚠ 这里**故意不关 USART1**（不调 OtaCom_DeInit）：留着自己最后还能打一行诊断，
       把"BL 卡在跳转前"和"跳过去之后 App 卡死"这两种情况分开看。
       USART1 不用清：App 的 OtaCom_Init() 会把 RCC/GPIO/波特率全部重新配一遍，
       而它的 NVIC 中断在上面那轮清扫里已经被禁掉，不会跳到错的 handler 上。 */

    __disable_irq();

    /* 清 NVIC 的使能/挂起位。⚠ SysTick / PendSV 的挂起位不在 NVIC->ICPR 里，在 SCB->ICSR */
    for (i = 0U; i < 8U; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFFU;
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;

    /*
     * ⚠⚠ 两个必须这么写的地方（踩过：表现是"跳过去之后一片安静，连启动日志都没有"）：
     *
     * 1) 这里**不要调 HAL_RCC_DeInit()**。它内部会调 HAL_InitTick()，而 Bootloader 里
     *    解析到的是 HAL 的弱实现（SysTick 版）→ 它会把 **SysTick 重新打开**
     *    （HAL_SYSTICK_Config = CLKSOURCE|TICKINT|ENABLE）。而 App 里 SysTick_Handler
     *    是 CMSIS-RTOS2 的（cmsis_os2.c → xPortSysTickHandler = FreeRTOS 的调度节拍），
     *    调度器还没初始化就被它打断 → 取 pxDelayedTaskList(NULL) → HardFault → 卡死。
     *    时钟也不用退：BL 配的就是 App 要的那套，App 的 SystemClock_Config() 自己会
     *    重新校验/切换（哪怕只剩 64 MHz HSI 它也能配起来）。
     *
     * 2) SysTick 必须在**所有可能打开它的调用之后**关，并且连挂起位一起清。
     *    不清挂起位的话，App 一开中断立刻进一次 SysTick 中断，同样跑飞。
     */
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL  = 0U;

    HAL_MPU_Disable();

    /* 诊断 2/2：能打出这行 = BL 的清理全部做完了，接下来就是把 CPU 交给 App。
       ⚠ 这行之后**不能再调 HAL_Delay / 任何等 tick 的东西**：SysTick 已经关了，
          tick 不走了（串口是轮询发出去的，不需要 tick）。 */
    __enable_irq();
    bl_print("bootloader: cleanup done -> calling App Reset_Handler\r\n");
    __disable_irq();

    SCB->VTOR = base;         /* App 的向量表在它自己那个槽的基址上 */

    __set_CONTROL(0U);        /* 特权 + MSP */
    __set_MSP(sp);
    __DSB();
    __ISB();

    /* ⚠ 一定要开中断：BL 上面刚关了，App 里没人会再开
       （复位后 PRIMASK=0 是 CPU 的默认状态，所以 App 自己不会去开）。
       开之前 SysTick 已经关掉、挂起位也清了，App 会在自己初始化时重新配 tick */
    __enable_irq();

    entry();

    for (;;)    /* 回不来了 */
    {
    }
}

/* ---- 恢复台 --------------------------------------------------------------- */

static void bl_on_frame(uint8_t type, uint8_t seq, const uint8_t *payload, uint16_t len)
{
    uint8_t reboot = 0U;

    if (OtaHost_HandleFrame(type, seq, payload, len, &reboot) != 0U)
    {
        if (reboot != 0U)
        {
            HAL_Delay(200);       /* 让回复发完 */
            NVIC_SystemReset();
        }

        return;
    }

    if (type == OTA_T_REBOOT)     /* 走不到：ota_host 已经处理了，留着防呆 */
    {
        NVIC_SystemReset();
    }
}

/*
 * 上电/复位时按住 USER_KEY（PA15，按下为低）= 强制进恢复台。
 *
 * 这是**无线 OTA 的命门**：App 跑飞（或者两个槽都写坏）时，只要 BL 还能跑，
 * 就能从这个串口重新灌固件，不用拆机、不用 ST-Link。
 * 没有它的话，BL 只能"向量表看着像 App 就跳"，跳进去死了就再也回不到恢复台 ——
 * 实测踩过：App 一启动就卡死，板子就只能上台架了。
 */
static uint8_t bl_key_held(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin  = USER_KEY_Pin;      /* PA15 */
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;       /* 板上已经有上拉，这里再补一个防悬空 */

    HAL_GPIO_Init(USER_KEY_GPIO_Port, &gpio);

    HAL_Delay(30U);                /* 消抖 */

    if (HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) != GPIO_PIN_RESET)
    {
        return 0U;
    }

    HAL_Delay(200U);               /* 按住超过 200 ms 才算真的要做恢复 */

    return (HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static void __attribute__((noreturn)) bl_console(ota_meta_t *m)
{
    uint8_t  buf[128];
    uint32_t last_rx = HAL_GetTick();

    bl_print("bootloader: recovery console ready.\r\n"
             "            use tools/ota.py to send a firmware image (frames over this same UART).\r\n");

    OtaLink_Init(bl_on_frame);
    OtaCom_FlushRx();

    for (;;)
    {
        uint16_t n = OtaCom_Read(buf, (uint16_t)sizeof(buf));

        if (n > 0U)
        {
            for (uint16_t i = 0U; i < n; i++)
            {
                OtaLink_Feed(buf[i]);
            }

            last_rx = HAL_GetTick();
            continue;
        }

        OtaHost_Tick();

        /* 主机一直不说话（比如它只是想断电重启一下），又有能跑的槽 → 别一直杵在这儿 */
        if ((HAL_GetTick() - last_rx) > BL_CONSOLE_IDLE_MS)
        {
            uint8_t crc_ok = 0U;
            uint8_t slot   = bl_pick_slot(m, &crc_ok);
            char    line[96];

            if (slot != OTA_SLOT_NONE)
            {
                (void)snprintf(line, sizeof(line),
                               "bootloader: console idle, trying slot %c\r\n",
                               (slot == OTA_SLOT_A) ? 'A' : 'B');
                bl_print(line);

                (void)OtaMeta_Load(m);
                bl_prepare_and_boot(m, slot);
            }

            last_rx = HAL_GetTick();      /* 两个槽都不能跑：继续等主机 */
        }
    }
}

/* Public functions ---------------------------------------------------------- */

/* SysTick 中断：HAL 的时基 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

void Error_Handler(void)
{
    __disable_irq();

    for (;;)
    {
    }
}

int main(void)
{
    ota_meta_t m;
    uint8_t    boot_slot;
    uint8_t    crc_ok = 0U;
    int8_t     meta_state;
    char       line[160];

    HAL_Init();
    bl_clock_config();

    /* 串口先起来：BL 的提示信息也走这个口（和 App 同一个日志口）。
       接收要显式开一下（OtaCom_Init 本身不开接收中断） */
    OtaCom_Init(OTA_PORT_BAUD);
    OtaCom_StartRx();
    OtaHost_Init(&s_bl_hooks);

    /* ⚠⚠ 顺序很关键：**先把横幅打出去，再去碰 Flash**。
       读一个 ECC 坏掉的 flash word 会当场 NMI/总线错误（见 bl_fault 的注释），
       以前这里顺序是"先 OtaMeta_Load()、后打横幅" —— 元数据一旦坏掉，
       上电后串口一个字符都没有，而且看起来像"什么都没跑"，非常难查。 */
    (void)snprintf(line, sizeof(line),
                   "\r\nbootloader v%lu.%lu.%lu (slot A @0x%08lX, slot B @0x%08lX, %lu KB each)\r\n",
                   (unsigned long)((OTA_BL_VERSION >> 16) & 0xFFU),
                   (unsigned long)((OTA_BL_VERSION >> 8) & 0xFFU),
                   (unsigned long)(OTA_BL_VERSION & 0xFFU),
                   (unsigned long)OTA_SLOT_A_BASE, (unsigned long)OTA_SLOT_B_BASE,
                   (unsigned long)(OTA_SLOT_SIZE / 1024U));
    bl_print(line);

    /* 元数据先给一份默认值：下面"按住按键进恢复台"这条路可能还在扫描之前就进来了 */
    OtaMeta_Init(&m);

    /* 按住 USER_KEY 上电 → 强制进恢复台（App 跑飞 / 元数据坏了时的唯一救命绳）。
       ⚠ 必须在**任何 Flash 读取之前**：否则元数据一坏，连恢复台都进不去。 */
    if (bl_key_held() != 0U)
    {
        bl_print("bootloader: USER_KEY held at boot -> recovery console\r\n");
        bl_console(&m);
    }

    /* 到这里才开始碰 Flash：扫元数据 */
    meta_state = OtaMeta_Load(&m);

    if (meta_state == 1)
    {
        bl_print("bootloader: metadata empty (first boot after ST-Link flash)\r\n");
    }
    else
    {
        (void)snprintf(line, sizeof(line),
                       "bootloader: meta active=%c boot=%c pending=%c attempts=%u flags=0x%02X"
                       "  A[%lu,0x%08lX] B[%lu,0x%08lX]\r\n",
                       (m.active_slot == OTA_SLOT_A) ? 'A' : 'B',
                       (m.boot_slot == OTA_SLOT_A) ? 'A' : 'B',
                       (m.pending_slot < OTA_SLOT_COUNT)
                           ? ((m.pending_slot == OTA_SLOT_A) ? 'A' : 'B') : '-',
                       (unsigned int)m.boot_attempts, (unsigned int)m.flags,
                       (unsigned long)m.slot_size[OTA_SLOT_A],
                       (unsigned long)m.slot_crc[OTA_SLOT_A],
                       (unsigned long)m.slot_size[OTA_SLOT_B],
                       (unsigned long)m.slot_crc[OTA_SLOT_B]);
        bl_print(line);
    }

    /* ② 主机要求停在 BL（OTA_REBOOT mode=1）→ 直接进恢复台 */
    if ((m.flags & 0x01U) != 0U)
    {
        bl_print("bootloader: stay-in-bootloader requested -> recovery console\r\n");
        bl_console(&m);
    }

    /* ③ 有等着切换的新镜像：校验通过才承认 */
    if (m.pending_slot < OTA_SLOT_COUNT)
    {
        uint8_t  slot = m.pending_slot;
        uint32_t crc  = 0U;
        uint8_t  good = 0U;

        if ((OtaMeta_SlotValid(&m, slot) != 0U) &&
            (OtaFlash_SlotCrc(slot, m.slot_size[slot], &crc) == OTA_OK) &&
            (crc == m.slot_crc[slot]))
        {
            good = 1U;
        }

        if (good != 0U)
        {
            m.boot_slot     = slot;
            m.boot_attempts = 0U;
            m.pending_slot  = OTA_SLOT_NONE;
            m.flags         = 0U;
            (void)OtaMeta_Write(&m);

            (void)snprintf(line, sizeof(line),
                           "bootloader: pending image OK -> switch to slot %c\r\n",
                           (slot == OTA_SLOT_A) ? 'A' : 'B');
            bl_print(line);
        }
        else
        {
            m.pending_slot = OTA_SLOT_NONE;    /* 坏镜像：丢掉，继续跑旧的 */
            (void)OtaMeta_Write(&m);
            bl_print("bootloader: pending image CRC bad -> DROPPED, keep current slot\r\n");
        }
    }

    /* ④ 挑一个能启动的槽 */
    boot_slot = bl_pick_slot(&m, &crc_ok);

    if (boot_slot == OTA_SLOT_NONE)
    {
        bl_print("bootloader: no bootable image -> recovery console\r\n");
        bl_console(&m);
    }

    if (crc_ok == 0U)
    {
        (void)snprintf(line, sizeof(line),
                       "bootloader: slot %c not matching metadata, booting anyway (App will re-register)\r\n",
                       (boot_slot == OTA_SLOT_A) ? 'A' : 'B');
        bl_print(line);
    }

    /* ⑤ 回滚判断：这个槽连着 OTA_MAX_BOOT_ATTEMPTS 次都没被 App 确认 → 判坏，换另一个 */
    if ((m.boot_attempts >= OTA_MAX_BOOT_ATTEMPTS) && (boot_slot == m.boot_slot))
    {
        uint8_t other;

        other = Ota_OtherSlot(boot_slot);

        (void)snprintf(line, sizeof(line),
                       "bootloader: slot %c NOT confirmed in %u attempts -> ROLLBACK\r\n",
                       (boot_slot == OTA_SLOT_A) ? 'A' : 'B', (unsigned int)m.boot_attempts);
        bl_print(line);

        m.boot_attempts = 0U;

        if ((OtaFlash_IsBlank(other) == 0U) &&
            ((OtaFlash_ImageLooksValid(other) != 0U) || (OtaMeta_SlotValid(&m, other) != 0U)))
        {
            bl_prepare_and_boot(&m, other);
        }

        bl_print("bootloader: other slot unusable -> recovery console\r\n");
        (void)OtaMeta_Write(&m);
        bl_console(&m);
    }

    /* ⑥ 落盘 boot_attempts++，然后跳过去 */
    *s_heal_count = 0U;      /* 要真的跳 App 了：说明这次启动是好的，清掉自愈计数 */
    bl_prepare_and_boot(&m, boot_slot);
}
