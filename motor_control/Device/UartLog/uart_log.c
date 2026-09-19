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
static UART_HandleTypeDef *s_uart  = NULL;
static osMutexId_t         s_mutex = NULL;

/* Exported functions --------------------------------------------------------*/

void UartLog_Init(UART_HandleTypeDef *huart)
{
    s_uart  = huart;
    s_mutex = osMutexNew(NULL);
}

void UartLog_Print(const char *text)
{
    if ((s_uart == NULL) || (text == NULL))
    {
        return;
    }

    /* 调度器还没跑起来的时候不能拿互斥量（xSemaphoreTake 在启动前用是未定义行为） */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        (void)osMutexAcquire(s_mutex, osWaitForever);
    }

    (void)HAL_UART_Transmit(s_uart, (uint8_t *)text, (uint16_t)strlen(text), HAL_MAX_DELAY);

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        (void)osMutexRelease(s_mutex);
    }
}
