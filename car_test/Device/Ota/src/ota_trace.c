/**
  ******************************************************************************
  * @file    ota_trace.c
  * @brief   启动诊断（Debug 构建才有输出，见 ota_trace.h）
  ******************************************************************************
  */

#include "ota_trace.h"
#include "ota_layout.h"
#include "ota_com.h"

#include "stm32h7xx_hal.h"

#if defined(DEBUG)

/* Private variables ---------------------------------------------------------*/

static UART_HandleTypeDef s_trace_uart;
static uint8_t            s_trace_ready;
static uint32_t           s_trace_pclk2;

/* 每次发送的超时：**不能用 HAL_MAX_DELAY** —— 诊断代码自己把启动卡死就没意义了 */
#define OTA_TRACE_TX_TIMEOUT_MS   30U

/* Private functions ---------------------------------------------------------*/

/*
 * 时钟一变就重算 BRR。
 *
 * ⚠ 必须做：`SystemClock_Config()` 会把 HSE/PLL 重配一遍，而这份诊断句柄的 BRR
 *   是按**改之前**的 PCLK2 算的 → 那之后打出来的标记全是乱码 ——
 *   而 "D/E/F/G 变乱码" 正好发生在最需要看清的地方（实测踩过）。
 */
static void ota_trace_sync(void)
{
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();

    if ((s_trace_ready != 0U) && (pclk2 == s_trace_pclk2))
    {
        return;
    }

    s_trace_pclk2 = pclk2;

    if (HAL_UART_Init(&s_trace_uart) == HAL_OK)
    {
        s_trace_ready = 1U;
    }
}

static void ota_trace_write(const uint8_t *data, uint16_t len)
{
    /* OTA 口已经起来了就走它发：它的波特率是按当前时钟算的，不会变成乱码 */
    if (OtaCom_IsReady() != 0U)
    {
        OtaCom_Write(data, len);
        return;
    }

    ota_trace_sync();

    if (s_trace_ready != 0U)
    {
        (void)HAL_UART_Transmit(&s_trace_uart, (uint8_t *)data, len, OTA_TRACE_TX_TIMEOUT_MS);
    }
}

/* Exported functions --------------------------------------------------------*/

void OtaTrace_Init(void)
{
    RCC_PeriphCLKInitTypeDef clk  = {0};
    GPIO_InitTypeDef         gpio = {0};

    if (s_trace_ready != 0U)
    {
        return;
    }

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    clk.PeriphClockSelection  = RCC_PERIPHCLK_USART1;
    clk.Usart16ClockSelection = RCC_USART16910CLKSOURCE_D2PCLK2;
    (void)HAL_RCCEx_PeriphCLKConfig(&clk);

    gpio.Pin       = GPIO_PIN_9;          /* PA9 = USART1_TX */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    s_trace_uart.Instance          = USART1;
    s_trace_uart.Init.BaudRate     = OTA_PORT_BAUD;
    s_trace_uart.Init.WordLength   = UART_WORDLENGTH_8B;
    s_trace_uart.Init.StopBits     = UART_STOPBITS_1;
    s_trace_uart.Init.Parity       = UART_PARITY_NONE;
    s_trace_uart.Init.Mode         = UART_MODE_TX;      /* 只发不收，不和 OTA 的收发任务抢 */
    s_trace_uart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_trace_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    s_trace_uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    s_trace_uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    s_trace_uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&s_trace_uart) == HAL_OK)
    {
        s_trace_ready = 1U;
        s_trace_pclk2 = HAL_RCC_GetPCLK2Freq();
    }
}

void OtaTrace_Text(const char *text)
{
    uint16_t len;

    if ((text == NULL) || (s_trace_ready == 0U))
    {
        return;
    }

    for (len = 0U; text[len] != '\0'; len++)
    {
    }

    ota_trace_write((const uint8_t *)text, len);
}

void OtaTrace_Mark(char mark)
{
    ota_trace_write((const uint8_t *)&mark, 1U);
}

/* ---- 故障 / 断言现场 -------------------------------------------------------
 * 这一段**故意不用 HAL**：只写 USART1 的寄存器，而且每个等待都有循环上限 ——
 * 故障现场 HAL 的状态可能已经不可信了，能打出去多少算多少。
 * ------------------------------------------------------------------------- */

static void ota_trace_raw_char(char c)
{
    uint32_t guard = 1000000U;

    while (((USART1->ISR & USART_ISR_TXE_TXFNF) == 0U) && (guard > 0U))
    {
        guard--;
    }

    USART1->TDR = (uint32_t)(uint8_t)c;
}

static void ota_trace_raw_text(const char *text)
{
    while (*text != '\0')
    {
        ota_trace_raw_char(*text);
        text++;
    }
}

static void ota_trace_raw_hex32(uint32_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    ota_trace_raw_text("0x");

    for (int32_t shift = 28; shift >= 0; shift -= 4)
    {
        ota_trace_raw_char(hex[(value >> (uint32_t)shift) & 0x0FU]);
    }
}

static void ota_trace_raw_dec(uint32_t value)
{
    char     buf[12];
    uint32_t n = 0U;

    if (value == 0U)
    {
        ota_trace_raw_char('0');
        return;
    }

    while ((value > 0U) && (n < (uint32_t)sizeof(buf)))
    {
        buf[n] = (char)('0' + (char)(value % 10U));
        value /= 10U;
        n++;
    }

    while (n > 0U)
    {
        n--;
        ota_trace_raw_char(buf[n]);
    }
}

/* 故障位翻译：把最常见的几种直接说成人话，省得每次翻手册 */
static void ota_trace_fault_hints(uint32_t cfsr)
{
    if ((cfsr & 0x00000200U) != 0U)      /* BFSR.PRECISERR */
    {
        ota_trace_raw_text("\r\n  -> precise bus error: BFAR is the faulting address");
    }

    if ((cfsr & 0x00000400U) != 0U)      /* BFSR.IMPRECISERR */
    {
        ota_trace_raw_text("\r\n  -> imprecise bus error (write buffer, BFAR not reliable)");
    }

    if ((cfsr & 0x00010000U) != 0U)      /* UFSR.UNDEFINSTR */
    {
        ota_trace_raw_text("\r\n  -> invalid instruction: PC ran away (bad fn pointer / return addr)");
    }

    if ((cfsr & 0x00040000U) != 0U)      /* UFSR.INVPC */
    {
        ota_trace_raw_text("\r\n  -> bad EXC_RETURN: the stack is most likely corrupted");
    }

    if ((cfsr & 0x01000000U) != 0U)      /* UFSR.UNALIGNED */
    {
        ota_trace_raw_text("\r\n  -> unaligned access: a cast to uint32_t*/struct pointer is the usual suspect");
    }
}

/* 这个地址像不像"栈"？异常帧必须落在 RAM 里；不确定就不解引用，
   否则在 fault handler 里再 fault 一次，就真的什么输出都没有了 */
static uint8_t ota_trace_sp_ok(uint32_t sp)
{
    if ((sp >= 0x20000000U) && (sp <= (0x20020000U - 0x40U))) { return 1U; }   /* DTCM（本工程） */
    if ((sp >= 0x24000000U) && (sp <= (0x24050000U - 0x40U))) { return 1U; }   /* AXI SRAM */
    if ((sp >= 0x30000000U) && (sp <= (0x30008000U - 0x40U))) { return 1U; }   /* D2 SRAM */
    if ((sp >= 0x38000000U) && (sp <= (0x38004000U - 0x40U))) { return 1U; }   /* D3 SRAM */

    return 0U;
}

/* 帧看着像不像真的：xPSR 的 T 位（bit24）必须是 1（Thumb），PC 必须落在 Flash 里 */
static uint8_t ota_trace_frame_ok(const uint32_t *f)
{
    if (((f[7] & 0x01000000U) == 0U) || (f[6] < 0x08000000U) || (f[6] > 0x08100000U))
    {
        return 0U;
    }

    return 1U;
}

/*
 * 故障现场。frame_sp / exc_lr 由 OTATRACE_FAULT() 宏在 handler 第一句话里抓好传进来：
 *   exc_lr(EXC_RETURN) 的 bit2：0 = 出错时在 handler 里（帧在 MSP），1 = 在任务里（帧在 PSP）
 * ⚠ 以前这里是无条件 `mrs msp` 当帧地址：任务里出错时帧其实在 PSP 上，
 *   打出来的 R0-R3/PC/LR 全是主栈上的垃圾值（2026-09-19 就被那个假 PC 带偏过一轮）。
 */
void OtaTrace_Fault(const char *tag, uint32_t frame_sp, uint32_t exc_lr)
{
    const uint32_t *f = (const uint32_t *)(uintptr_t)frame_sp;

    ota_trace_raw_text("\r\n*** FAULT: ");
    ota_trace_raw_text((tag != NULL) ? tag : "?");
    ota_trace_raw_text(" ***\r\n  CFSR=");
    ota_trace_raw_hex32(SCB->CFSR);
    ota_trace_raw_text(" HFSR=");
    ota_trace_raw_hex32(SCB->HFSR);
    ota_trace_raw_text(" MMFAR=");
    ota_trace_raw_hex32(SCB->MMFAR);
    ota_trace_raw_text(" BFAR=");
    ota_trace_raw_hex32(SCB->BFAR);

    ota_trace_raw_text("\r\n  fault in ");
    ota_trace_raw_text(((exc_lr & 0x4U) != 0U) ? "task (PSP)" : "handler/ISR (MSP)");
    ota_trace_raw_text(" EXC_RETURN=");
    ota_trace_raw_hex32(exc_lr);
    ota_trace_raw_text(" frame@");
    ota_trace_raw_hex32(frame_sp);

    ota_trace_fault_hints(SCB->CFSR);

    if ((ota_trace_sp_ok(frame_sp) == 0U) || (ota_trace_frame_ok(f) == 0U))
    {
        ota_trace_raw_text("\r\n  (no exception frame on stack, registers skipped - CFSR/BFAR above is enough)\r\n");
        return;
    }

    ota_trace_raw_text("\r\n  R0=");
    ota_trace_raw_hex32(f[0]);
    ota_trace_raw_text(" R1=");
    ota_trace_raw_hex32(f[1]);
    ota_trace_raw_text(" R2=");
    ota_trace_raw_hex32(f[2]);
    ota_trace_raw_text(" R3=");
    ota_trace_raw_hex32(f[3]);
    ota_trace_raw_text(" R12=");
    ota_trace_raw_hex32(f[4]);
    ota_trace_raw_text("\r\n  PC(fault)=");
    ota_trace_raw_hex32(f[6]);
    ota_trace_raw_text(" LR(caller)=");
    ota_trace_raw_hex32(f[5]);
    ota_trace_raw_text("\r\n  lookup: arm-none-eabi-addr2line -f -C -e build/Debug/motor_control.elf <PC> <LR>\r\n");
}

void OtaTrace_AssertFailed(const char *file, int line, const char *expr)
{
    ota_trace_raw_text("\r\n*** ASSERT FAILED ***\r\n  ");
    ota_trace_raw_text((expr != NULL) ? expr : "?");
    ota_trace_raw_text("\r\n  at ");
    ota_trace_raw_text((file != NULL) ? file : "?");
    ota_trace_raw_char(':');
    ota_trace_raw_dec((uint32_t)line);
    ota_trace_raw_text("\r\n");
}

#else  /* !DEBUG：Release 里全是空函数，一点开销都没有 */

void OtaTrace_Init(void)             { }
void OtaTrace_Text(const char *text) { (void)text; }
void OtaTrace_Mark(char mark)        { (void)mark; }
void OtaTrace_Fault(const char *tag, uint32_t frame_sp, uint32_t exc_lr)
{
    (void)tag;
    (void)frame_sp;
    (void)exc_lr;
}
void OtaTrace_AssertFailed(const char *file, int line, const char *expr)
{
    (void)file;
    (void)line;
    (void)expr;
}

#endif /* DEBUG */
