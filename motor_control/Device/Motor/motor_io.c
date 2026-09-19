/**
  ******************************************************************************
  * @file    motor_io.c
  * @brief   电机串口收发服务：请求队列 + 独占串口的收发任务
  *
  * 结构：
  *   MotorIo_Exchange()  ──(非收发任务)──▶  xQueueSend ──▶ MotorIo_Task
  *          ▲                                                    │
  *          └──────────── xTaskNotify(结果) / xTaskNotifyWait ◀──┘
  *
  *   MotorIo_Exchange()  ──(收发任务自己调)──▶ 直接收发
  *
  * 几个容易踩的点，见下面各处的注释：
  *   1. 队列是**值拷贝**，所以请求 struct 里只放指针，结果用**任务通知的值**回传
  *      （struct 里的字段在收发任务那边写，调用方是看不到的）
  *   2. 等待通知前先清一次遗留通知，否则会「假成功」
  *   3. tx/rx 指针的有效期：调用方一直阻塞到收发结束，所以栈上的缓冲是安全的
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_io.h"
#include "uart_log.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/

/* 一次收发请求：会按值拷进队列，所以这里只放指针/长度 */
typedef struct
{
    const uint8_t *tx;      /* 发送缓冲（调用方持有，阻塞期间有效） */
    uint16_t       tx_len;
    uint8_t       *rx;      /* 接收缓冲（调用方持有，调用前已清零） */
    uint16_t       rx_len;
    TaskHandle_t   caller;  /* 谁发起的；结果用任务通知回给它 */
} MotorIoRequest;

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *s_uart  = NULL;   /* 电机串口，MotorIo_Init 绑定 */
static QueueHandle_t       s_queue = NULL;   /* 请求队列 */
static TaskHandle_t        s_owner = NULL;   /* 收发任务自己，用来识别重入 */

/* 最近一次清 RX 时「偰看」到的前几个字节：线上到底在发生什么，一看就知道
   （全 00 / 全 FF = 线被拉低或电机没上电；有规律 = 电机一直在说话） */
static uint8_t s_flush_sniff[MOTOR_IO_FLUSH_SNIFF_LEN];
static uint8_t s_flush_sniff_len;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  清空串口接收，让每次一问一答都从干净状态开始
  * @note   上一轮超时/校验失败时可能还有半帧残留在 RX 里，不清掉下一轮就会错位；
  *         接收过程中出现的溢出/帧错误标志也一起清掉。
  * @retval 实际丢掉的字节数（上限 MOTOR_IO_FLUSH_MAX_BYTES）
  *
  * ⚠⚠ 这里**绝对不能**写成 while (HAL_UART_Receive(...) == HAL_OK) {} ：
  *   1) HAL_UART_Receive 的 timeout 只在「一个字节都收不到」时才生效，
  *      一旦线上持续有数据，它就每次都立刻返回 HAL_OK；
  *   2) 电机总线是单线 5V TTL（PE2/PE3 开漏 + 总线拉高），
  *      电机没上电 / 总线被拉低 / 波特率不对时，USART 会把持续的
  *      低电平当成连续起始位，按波特率源源不断产生帧错误字节。
  *   两个加起来 ⇒ 那个 while 永远出不来，而且不报错、不断言，
  *   表现就是「开机停在 MotorIo_Init 这里、串口一声不吭」。
  *   所以这里用「直接看 RXNE 标志 + 硬上限」，清不掉就放弃（并打日志）。
  */
static uint16_t MotorIo_FlushRx(void)
{
    uint16_t n = 0U;

    s_flush_sniff_len = 0U;

    if (s_uart == NULL)
    {
        return 0U;
    }

    while (n < MOTOR_IO_FLUSH_MAX_BYTES)
    {
        if (__HAL_UART_GET_FLAG(s_uart, UART_FLAG_RXNE) == RESET)
        {
            break;      /* 没有新字节了：正常出口 */
        }

        /* 读 RDR 就自动清了 RXNE，不用走 HAL 的加锁/状态机 */
        uint8_t byte = (uint8_t)(s_uart->Instance->RDR & 0xFFU);

        if (s_flush_sniff_len < MOTOR_IO_FLUSH_SNIFF_LEN)
        {
            s_flush_sniff[s_flush_sniff_len] = byte;
            s_flush_sniff_len++;
        }

        n++;
    }

    s_uart->ErrorCode = HAL_UART_ERROR_NONE;
    __HAL_UART_CLEAR_FLAG(s_uart,
                          UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);

    return n;
}

/**
  * @brief  清不干净时吐一行日志，把「线上到底是什么」也带上，方便直接定位硬件问题
  */
static void MotorIo_WarnDirtyRx(uint16_t flushed)
{
    char   line[256];
    char   hex[3U * MOTOR_IO_FLUSH_SNIFF_LEN + 1U];
    size_t k = 0U;

    for (uint8_t i = 0U; i < s_flush_sniff_len; i++)
    {
        (void)snprintf(&hex[k], sizeof(hex) - k, "%02X ", s_flush_sniff[i]);
        k += 3U;
    }
    hex[k] = '\0';

    (void)snprintf(line, sizeof(line),
                   "[motor] ⚠ 电机串口 RX 一直在收数据（清了 %u 字节，前 %u 字: %s）\r\n"
                   "        线被拉低 / 电机没上电 / 波特率不对 —— 已放弃清理继续启动，不会卡在这里\r\n",
                   (unsigned)flushed, (unsigned)s_flush_sniff_len, hex);
    UartLog_Print(line);
}

/**
  * @brief  真正的收发：谁是收发任务就由谁执行
  * @retval 1 = rx_len 字节都收到，0 = 失败
  */
static uint8_t MotorIo_DoExchange(const uint8_t *tx, uint16_t tx_len,
                                  uint8_t *rx, uint16_t rx_len)
{
    MotorIo_FlushRx();

    if (HAL_UART_Transmit(s_uart, (uint8_t *)tx, tx_len, MOTOR_FRAME_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }

    /* 收不满也直接返回失败：rx 里已经落地的字节保持原样（调用方清零过），
       失败时打出来就是「收到几个、剩下全是 0」的现场 */
    if (HAL_UART_Receive(s_uart, rx, rx_len, MOTOR_FRAME_TIMEOUT_MS) != HAL_OK)
    {
        MotorIo_FlushRx();
        return 0U;
    }

    return 1U;
}

/* Exported functions --------------------------------------------------------*/

void MotorIo_Init(UART_HandleTypeDef *huart)
{
    char     line[96];
    uint16_t flushed;

    s_uart  = huart;
    s_queue = xQueueCreate(MOTOR_IO_QUEUE_LEN, sizeof(MotorIoRequest));

#if defined(DEBUG)
    /* 分两步打：万一下一行出不来，就知道是死在「建队列」还是「清 RX」 */
    UartLog_Print("[motor] io: queue created, flushing motor uart rx ...\r\n");
#endif

    flushed = MotorIo_FlushRx();

    (void)snprintf(line, sizeof(line),
                   "[motor] io ready (queue=%u, flushed=%u B)\r\n",
                   (unsigned)((s_queue != NULL) ? 1U : 0U), (unsigned)flushed);
    UartLog_Print(line);

    if (flushed >= MOTOR_IO_FLUSH_MAX_BYTES)
    {
        MotorIo_WarnDirtyRx(flushed);
    }
}

void MotorIo_Task(void *argument)
{
    MotorIoRequest req;

    (void)argument;

    /* 先登记自己：MotorIo_Exchange 靠它判断「这条请求是不是我发的」 */
    s_owner = xTaskGetCurrentTaskHandle();

    for (;;)
    {
        if (xQueueReceive(s_queue, &req, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        uint8_t ok = MotorIo_DoExchange(req.tx, req.tx_len, req.rx, req.rx_len);

        /* 结果用通知的**值**回传（eSetValueWithOverwrite：只留最后一个，不累计） */
        if (req.caller != NULL)
        {
            (void)xTaskNotify(req.caller, (uint32_t)ok, eSetValueWithOverwrite);
        }
    }
}

uint8_t MotorIo_Exchange(const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len)
{
    MotorIoRequest req;
    TickType_t     ticks;
    uint32_t       result = 0U;

    if ((s_uart == NULL) || (tx == NULL) || (rx == NULL) || (tx_len == 0U) || (rx_len == 0U))
    {
        return 0U;
    }

    /* 收发任务自己调：直接收发。走队列的话就是自己等自己，死锁 */
    if (xTaskGetCurrentTaskHandle() == s_owner)
    {
        return MotorIo_DoExchange(tx, tx_len, rx, rx_len);
    }

    if (s_queue == NULL)
    {
        return 0U;
    }

    req.tx     = tx;
    req.tx_len = tx_len;
    req.rx     = rx;
    req.rx_len = rx_len;
    req.caller = xTaskGetCurrentTaskHandle();

    /* 先清掉可能遗留的通知：不清的话下面 xTaskNotifyWait 会立刻返回上一次的结果 */
    (void)ulTaskNotifyTake(pdTRUE, 0U);

    ticks = pdMS_TO_TICKS(MOTOR_IO_QUEUE_TIMEOUT_MS);
    if (xQueueSend(s_queue, &req, ticks) != pdTRUE)
    {
        return 0U;   /* 队列满（收发任务被别的请求堵住了） */
    }

    ticks = pdMS_TO_TICKS(MOTOR_IO_EXCHANGE_TIMEOUT_MS);
    if (xTaskNotifyWait(0U, 0U, &result, ticks) != pdTRUE)
    {
        return 0U;   /* 连收发任务都没回音，链路出问题了 */
    }

    return (uint8_t)result;
}
