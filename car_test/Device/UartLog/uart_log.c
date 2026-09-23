/**
  ******************************************************************************
  * @file    uart_log.c
  * @brief   UART7 调试日志：给阻塞发送套一层互斥，多任务可并发调用
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "uart_log.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#include <string.h>

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *s_sink[UART_LOG_MAX_SINKS];
static uint8_t             s_sink_count;
static osMutexId_t         s_mutex   = NULL;
static uint8_t             s_enabled = 1U;

/* 发送超时：**不能用 HAL_MAX_DELAY**。无线模块掉线/串口异常时，
   HAL_MAX_DELAY 会把调用它的任务永远卡在发送里（日志、轮询、OTA 全停）。 */
#define UART_LOG_TX_TIMEOUT_MS   200U

/* Private functions ---------------------------------------------------------*/

/* 调度器还没跑起来的时候不能拿互斥量（xSemaphoreTake 在启动前用是未定义行为） */
static uint8_t uart_log_can_lock(void)
{
    return (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) ? 1U : 0U;
}

/* Exported functions --------------------------------------------------------*/

void UartLog_Init(UART_HandleTypeDef *huart)
{
    s_sink_count = 0U;
    s_enabled    = 1U;

    (void)memset(s_sink, 0, sizeof(s_sink));

    if (huart != NULL)
    {
        s_sink[0]    = huart;
        s_sink_count = 1U;
    }

    if (s_mutex == NULL)
    {
        s_mutex = osMutexNew(NULL);
    }
}

void UartLog_AddSink(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (s_sink_count >= UART_LOG_MAX_SINKS))
    {
        return;
    }

    for (uint8_t i = 0U; i < s_sink_count; i++)
    {
        if (s_sink[i] == huart)
        {
            return;    /* 已经挂过了 */
        }
    }

    s_sink[s_sink_count] = huart;
    s_sink_count++;
}

void UartLog_Print(const char *text)
{
    uint16_t len;

    if ((text == NULL) || (s_sink_count == 0U) || (s_enabled == 0U))
    {
        return;
    }

    len = (uint16_t)strlen(text);

    if (len == 0U)
    {
        return;
    }

    UartLog_Lock();

    for (uint8_t i = 0U; i < s_sink_count; i++)
    {
        if (s_sink[i] != NULL)
        {
            (void)HAL_UART_Transmit(s_sink[i], (uint8_t *)text, len, UART_LOG_TX_TIMEOUT_MS);
        }
    }

    UartLog_Unlock();
}

void UartLog_SetEnabled(uint8_t enabled)
{
    s_enabled = (enabled != 0U) ? 1U : 0U;
}

void UartLog_Lock(void)
{
    if ((s_mutex != NULL) && (uart_log_can_lock() != 0U))
    {
        (void)osMutexAcquire(s_mutex, osWaitForever);
    }
}

void UartLog_Unlock(void)
{
    if ((s_mutex != NULL) && (uart_log_can_lock() != 0U))
    {
        (void)osMutexRelease(s_mutex);
    }
}
