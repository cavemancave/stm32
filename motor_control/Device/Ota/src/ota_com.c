/**
  ******************************************************************************
  * @file    ota_com.c
  * @brief   无线串口 USART1 收发（Bootloader 和 App 共用）
  ******************************************************************************
  */

#include "ota_com.h"
#include "ota_layout.h"
#include "ota_trace.h"

/* Private variables ---------------------------------------------------------*/

static UART_HandleTypeDef s_uart;
static uint8_t            s_rx_byte;                 /* HAL 逐字节接收的落点 */
static uint8_t            s_ring[OTA_PORT_RX_RING];
static volatile uint16_t  s_head;
static volatile uint16_t  s_tail;
static volatile uint32_t  s_rx_total;                /* 统计：一共收到多少字节 */
static volatile uint32_t  s_rx_lost;                 /* 统计：环形缓冲溢出丢了多少 */
static void             (*s_rx_hook)(void);
static uint8_t            s_inited;
static uint8_t            s_rx_armed;

/* Private functions ---------------------------------------------------------*/

/* 硬件初始化：RCC + GPIO + NVIC。
   ⚠ 生成文件里的 HAL_UART_MspInit() 只管 UART7/USART10，不会管 USART1，
     所以这里必须把 RCC/GPIO 全做了（HAL_UART_Init 里调 MSP 时我们对它来说是"没分支"的）。 */
static void OtaCom_HwInit(uint32_t baud)
{
    RCC_PeriphCLKInitTypeDef clk = {0};
    GPIO_InitTypeDef         gpio = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* USART1 走 D2PCLK2（和 UART7/USART10 同源，120 MHz）：要和 usart.c 里那两处保持一致 */
    clk.PeriphClockSelection   = RCC_PERIPHCLK_USART1;
    clk.Usart16ClockSelection  = RCC_USART16910CLKSOURCE_D2PCLK2;
    (void)HAL_RCCEx_PeriphCLKConfig(&clk);

    /* PA9 = USART1_TX，AF7，推挽 */
    gpio.Pin       = GPIO_PIN_9;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PA10 = USART1_RX，AF7。模块的 TX 是推挽驱动，上拉只是防"模块没插/没上电"时收噪声 */
    gpio.Pin  = GPIO_PIN_10;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);

    s_uart.Instance          = USART1;
    s_uart.Init.BaudRate     = baud;
    s_uart.Init.WordLength   = UART_WORDLENGTH_8B;
    s_uart.Init.StopBits     = UART_STOPBITS_1;
    s_uart.Init.Parity       = UART_PARITY_NONE;
    s_uart.Init.Mode         = UART_MODE_TX_RX;
    s_uart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    s_uart.Init.OneBitSampling   = UART_ONE_BIT_SAMPLE_DISABLE;
    s_uart.Init.ClockPrescaler   = UART_PRESCALER_DIV1;
    s_uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&s_uart) != HAL_OK)
    {
        /* 初始化失败：OtaCom_Write/Read 会因为 s_inited==0 直接返回，
           表现就是「跳过来之后 App 一片安静」—— 这里必须把它喊出来 */
        OtaTrace_Text("[ota_com] FATAL: USART1 HAL_UART_Init FAILED -> no log, no OTA\r\n");
        return;
    }

    OtaTrace_Text("[ota_com] USART1 up (log + control + OTA)\r\n");

    /* 16 字节 RX FIFO。threshold 设 1/8：FIFO 里攒到 2 个字节就中断，别等满了 */
    if (HAL_UARTEx_EnableFifoMode(&s_uart) == HAL_OK)
    {
        (void)HAL_UARTEx_SetTxFifoThreshold(&s_uart, UART_TXFIFO_THRESHOLD_1_8);
        (void)HAL_UARTEx_SetRxFifoThreshold(&s_uart, UART_RXFIFO_THRESHOLD_1_8);
    }

    /* 中断优先级 5 == configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY：
       ISR 里可以安全调用 FreeRTOS 的 ...FromISR()（和 SPI2/DMA 那几处一致） */
    HAL_NVIC_SetPriority(USART1_IRQn, 5U, 0U);

    s_head   = 0U;
    s_tail   = 0U;
    s_inited = 1U;

    /* ⚠ 这里**不**使能 NVIC / 不武装接收：交给 OtaCom_StartRx()。
       App 必须在调度器跑起来之后才开接收中断，否则中断里调 FreeRTOS API 会立刻
       触发 PendSV 切进还没初始化的调度器 → HardFault（实测踩过，见 ota_service.c） */
}

void OtaCom_StartRx(void)
{
    if ((s_inited == 0U) || (s_rx_armed != 0U))
    {
        return;
    }

    s_rx_armed = 1U;

    HAL_NVIC_EnableIRQ(USART1_IRQn);

    (void)HAL_UART_Receive_IT(&s_uart, &s_rx_byte, 1U);
}

/* Exported functions --------------------------------------------------------*/

void OtaCom_Init(uint32_t baud)
{
    if (s_inited != 0U)
    {
        return;
    }

    OtaCom_HwInit(baud);
}

UART_HandleTypeDef *OtaCom_Handle(void)
{
    return &s_uart;
}

uint8_t OtaCom_IsReady(void)
{
    return s_inited;
}

void OtaCom_Write(const uint8_t *data, uint32_t len)
{
    if ((s_inited == 0U) || (data == NULL) || (len == 0U))
    {
        return;
    }

    /* 阻塞发完（无线模块没有流控，不会永久卡住） */
    (void)HAL_UART_Transmit(&s_uart, (uint8_t *)data, (uint16_t)len, OTA_PORT_TX_TIMEOUT_MS);
}

uint16_t OtaCom_Read(uint8_t *dst, uint16_t max)
{
    uint16_t n = 0U;

    if (dst == NULL)
    {
        return 0U;
    }

    /* 单生产者（ISR）/单消费者（任务）：head/tail 都是 16 bit 对齐访问，天然原子，
       不需要关中断 */
    while ((n < max) && (s_tail != s_head))
    {
        dst[n] = s_ring[s_tail];
        n++;
        s_tail = (uint16_t)((s_tail + 1U) % OTA_PORT_RX_RING);
    }

    return n;
}

void OtaCom_SetRxHook(void (*hook)(void))
{
    s_rx_hook = hook;
}

void OtaCom_FlushRx(void)
{
    s_tail = s_head;
}

void OtaCom_DeInit(void)
{
    if (s_inited == 0U)
    {
        return;
    }

    HAL_NVIC_DisableIRQ(USART1_IRQn);
    (void)HAL_UART_DeInit(&s_uart);
    __HAL_RCC_USART1_CLK_DISABLE();

    s_inited   = 0U;
    s_rx_armed = 0U;
}

/* ---- 中断入口 ------------------------------------------------------------ */

/* ⚠ 如果以后决定把 USART1 放进 CubeMX 配：
     CubeMX 会在 Core/Src/stm32h7xx_it.c 里生成 USART1_IRQHandler，
     和下面这个**重复定义**。二选一：要么保持现在这样（推荐，不动生成文件），
     要么删掉下面这个 handler 和 OtaCom_HwInit 里的硬件初始化，改成用 MX_USART1_UART_Init()。 */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&s_uart);
}

/* HAL 收完一个字节的回调（中断上下文） */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART1))
    {
        return;
    }

    uint16_t next = (uint16_t)((s_head + 1U) % OTA_PORT_RX_RING);

    if (next != s_tail)
    {
        s_ring[s_head] = s_rx_byte;
        s_head         = next;
        s_rx_total++;

        if (s_rx_hook != NULL)
        {
            s_rx_hook();
        }
    }
    else
    {
        s_rx_lost++;    /* 环形缓冲满了：上层处理不过来，直接丢（帧 CRC 会兜住） */
    }

    /* 立刻重新武装，只收 1 个字节（FIFO 里还有的话会紧接着再中断一次） */
    (void)HAL_UART_Receive_IT(&s_uart, &s_rx_byte, 1U);
}
