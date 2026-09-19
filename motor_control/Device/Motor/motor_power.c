/**
  ******************************************************************************
  * @file    motor_power.c
  * @brief   电机电源开关（PC14 控制的可控电源输出，高电平使能）
  *
  * 细节和取舍见 motor_power.h 的注释。这里只说实现上的两个点：
  *   1. 配成输出**之前**先把 ODR 写 0：否则 GPIO 一切换到输出模式的瞬间，
  *      输出电平取决于 ODR 的旧值，有可能"啪"一下把电机带上电。
  *   2. 延时用 osDelay（任务里让出 CPU）；启动早期调度器还没跑就用 HAL_Delay。
  *      这样 MotorPwr_OnAndSettle() 在 main() 里也能用。
  ******************************************************************************
  */

#include "motor_power.h"
#include "uart_log.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#include <stdio.h>

/* PC14 的引脚定义：CubeMX 生成之后用生成出来的宏（main.h 里按标签生成：
   VCC_OUT1_EN_Pin / VCC_OUT1_EN_GPIO_Port），还没生成（.ioc 刚改、没跑 Generate Code）
   就用下面这对兜底，保证代码立刻能编译能跑。
   ⚠ 以后在 CubeMX 里换引脚（或改标签）时，记得这里的兜底也要改；
     或者干脆重新生成一次代码，让它走上面那个宏。 */
#if defined(VCC_OUT1_EN_Pin)
#define MOTOR_PWR_PORT      VCC_OUT1_EN_GPIO_Port
#define MOTOR_PWR_PIN       VCC_OUT1_EN_Pin
#else
#define MOTOR_PWR_PORT      GPIOC
#define MOTOR_PWR_PIN       GPIO_PIN_14
#endif

/* Private variables ---------------------------------------------------------*/

static uint8_t s_on;      /* 当前状态（0 = 关断；上电默认 0） */

/* Private functions ---------------------------------------------------------*/

static void motor_pwr_delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        osDelay(pdMS_TO_TICKS(ms));
    }
    else
    {
        HAL_Delay(ms);      /* 启动早期（还没有调度器）：HAL 的 TIM6 节拍是好的 */
    }
}

static void motor_pwr_write(uint8_t on)
{
    HAL_GPIO_WritePin(MOTOR_PWR_PORT, MOTOR_PWR_PIN,
                      (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Exported functions --------------------------------------------------------*/

void MotorPwr_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* ⚠ 顺序：先写 0，再配成输出（见文件头注释第 1 条） */
    motor_pwr_write(0U);

    gpio.Pin   = MOTOR_PWR_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;    /* 只是使能信号，不需要快边沿（也能少点干扰） */
    HAL_GPIO_Init(MOTOR_PWR_PORT, &gpio);

    s_on = 0U;
}

void MotorPwr_Enable(uint8_t on)
{
    char line[80];

    on = (on != 0U) ? 1U : 0U;

    if (on == s_on)
    {
        return;      /* 没变化：不重复写、不重复刷日志 */
    }

    motor_pwr_write(on);
    s_on = on;

    (void)snprintf(line, sizeof(line), "[motor] power %s (PC14/VCC_OUT1_EN -> %s)\r\n",
                   (on != 0U) ? "ON" : "OFF", (on != 0U) ? "high (enabled)" : "low (cut)");
    UartLog_Print(line);
}

uint8_t MotorPwr_IsOn(void)
{
    return s_on;
}

void MotorPwr_OnAndSettle(void)
{
    MotorPwr_Enable(1U);
    motor_pwr_delay_ms(MOTOR_PWR_SETTLE_MS);
}

void MotorPwr_PowerCycle(void)
{
    MotorPwr_Enable(0U);
    motor_pwr_delay_ms(MOTOR_PWR_DISCHARGE_MS);
    MotorPwr_Enable(1U);
    motor_pwr_delay_ms(MOTOR_PWR_SETTLE_MS);
}
