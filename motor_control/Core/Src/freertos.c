/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "motor_io.h"
#include "motor_ctrl.h"
#include "uart_log.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTimer_t osStaticTimerDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motor_task */
osThreadId_t motor_taskHandle;
const osThreadAttr_t motor_task_attributes = {
  .name = "motor_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for motorPos */
osTimerId_t motorPosHandle;
osStaticTimerDef_t motorPosControlBlock;
const osTimerAttr_t motorPos_attributes = {
  .name = "motorPos",
  .cb_mem = &motorPosControlBlock,
  .cb_size = sizeof(motorPosControlBlock),
};

/* 请求队列不在这里：它是收发层的东西，在 Device/Motor/motor_io.c 里
   随串口一起建（MotorIo_Init）。所以 CubeMX 的 FreeRTOS 配置里
   也不需要再配 motorQueue 了，别再加回来。 */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void MotorTask(void *argument);
void motorPosCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* 日志口（UART7）和电机串口（USART10 + 请求队列）。
     这两个都要在 osKernelInitialize() 之后才能建互斥量/队列，
     而 MX_FREERTOS_Init() 正好是那个时候被调的。 */
  UartLog_Init(&huart7);
  MotorIo_Init(&huart10);

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of motorPos */
  motorPosHandle = osTimerNew(motorPosCallback, osTimerPeriodic, NULL, &motorPos_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* 状态轮询定时器：周期到了只释放信号量（回调在 timer service task 里跑，
     绝对不能阻塞），真正的收发由 MotorCtrl_PollTask 做。 */
  (void)osTimerStart(motorPosHandle, pdMS_TO_TICKS(MOTOR_CTRL_POLL_MS));
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* 请求队列在 MotorIo_Init() 里建（见上面 USER CODE Init） */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of motor_task */
  motor_taskHandle = osThreadNew(MotorTask, NULL, &motor_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* 轮询信号量 + 轮询任务（CubeMX 的线程列表里没有它，所以在这里建） */
  MotorCtrl_Init();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* 主控制流程：使能 → 切位置环 → 0 点 / 3 点来回（不返回） */
  MotorCtrl_Task(argument);

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_MotorTask */
/**
* @brief Function implementing the motor_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorTask */
void MotorTask(void *argument)
{
  /* USER CODE BEGIN MotorTask */
  /* 独占 USART10 的收发任务：从队列取请求 → 收发 → 通知发起者（不返回） */
  MotorIo_Task(argument);

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MotorTask */
}

/* motorPosCallback function */
void motorPosCallback(void *argument)
{
  /* USER CODE BEGIN motorPosCallback */
  /* 只释放信号量，不阻塞（这里跑在 timer service task 里） */
  MotorCtrl_OnPollTimer();
  /* USER CODE END motorPosCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

