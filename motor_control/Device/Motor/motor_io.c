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

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

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

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  清空串口接收，让每次一问一答都从干净状态开始
  * @note   上一轮超时/校验失败时可能还有半帧残留在 RX 里，不清掉下一轮就会错位；
  *         接收过程中出现的溢出/帧错误标志也一起清掉。
  */
static void MotorIo_FlushRx(void)
{
    uint8_t dummy;

    /* 一字节一字节地读，读到读不出来了为止（残留不会太多） */
    while (HAL_UART_Receive(s_uart, &dummy, 1U, 1U) == HAL_OK)
    {
        /* 丢掉 */
    }

    s_uart->ErrorCode = HAL_UART_ERROR_NONE;
    __HAL_UART_CLEAR_FLAG(s_uart,
                          UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
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
    s_uart  = huart;
    s_queue = xQueueCreate(MOTOR_IO_QUEUE_LEN, sizeof(MotorIoRequest));

    if (s_uart != NULL)
    {
        MotorIo_FlushRx();
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
