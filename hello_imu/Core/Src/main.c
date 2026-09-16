/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMI088driver.h"
#include "ws2812.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 板载 WS2812 指示灯亮度(0-255)：满亮度很刺眼，这里用中等亮度 */
#define LED_BRIGHTNESS            64U

/* 使能可控 5V 之后，等 5V 轨稳定再和 BMI088 通信。
   BMI088 数据手册要求 VDD 有效后加速度计 ~1ms、陀螺仪 ~30ms 才能访问，
   5V 轨上的滤波电容充电还需要更多时间，这里统一留 100ms。 */
#define POWER_5V_RAMP_UP_MS       100U

/* BMI088 初始化失败后的重试间隔，以及最多向串口报告多少次错误码 */
#define BMI088_INIT_RETRY_MS      200U
#define BMI088_INIT_LOG_ATTEMPTS  10U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static int format_sensor_value(char *buffer, size_t buffer_size, float value);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float gyro[3], accel[3], temp;

static int format_sensor_value(char *buffer, size_t buffer_size, float value)
{
  int32_t scaled_value = (int32_t)(value * 1000.0f);
  int32_t magnitude = scaled_value;
  int32_t fraction;

  if (magnitude < 0)
  {
    magnitude = -magnitude;
  }
  fraction = magnitude % 1000;

  return snprintf(buffer, buffer_size, scaled_value < 0 ? "-%ld.%03ld" : "%ld.%03ld",
                  (long)(magnitude / 1000), (long)fraction);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART7_Init();
  MX_SPI2_Init();
  MX_SPI6_Init();
  /* USER CODE BEGIN 2 */

  static const uint8_t bmi_header[] = "gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,accel_x_g,accel_y_g,accel_z_g,temp_c\r\n";
  char bmi_uart_buffer[128];
  char bmi_value_buffer[7][16];
  char bmi_error_buffer[64];
  int bmi_uart_length;

  /* 使能可控 5V：板载 WS2812 和 BMI088 都由这一路供电，上电默认是关的 */
  HAL_GPIO_WritePin(Power_5V_EN_GPIO_Port, Power_5V_EN_Pin, GPIO_PIN_SET);

  /* 等 5V 轨爬升稳定后再和传感器通信 */
  HAL_Delay(POWER_5V_RAMP_UP_MS);

  /* 5V 有了再发一帧全灭：MCU 单独复位时灯珠会保留上一次的颜色 */
  WS2812_Ctrl(0U, 0U, 0U);

  if (HAL_UART_Transmit(&huart7, (uint8_t *)bmi_header, sizeof(bmi_header) - 1U, HAL_MAX_DELAY) != HAL_OK)
  {
    Error_Handler();
  }

  /* 等待 BMI088 就绪：红灯 = 还没成功(会一直重试)，绿灯 = 初始化成功 */
  uint8_t bmi_error;
  uint32_t bmi_attempt = 0U;

  while ((bmi_error = BMI088_init()) != BMI088_NO_ERROR)
  {
    WS2812_Ctrl(LED_BRIGHTNESS, 0U, 0U);

    /* 只报告前几次，避免传感器一直没响应时刷屏 */
    if (bmi_attempt < BMI088_INIT_LOG_ATTEMPTS)
    {
      int bmi_error_length = snprintf(bmi_error_buffer, sizeof(bmi_error_buffer),
                                      "BMI088 init failed, error=0x%02X, attempt=%lu\r\n",
                                      (unsigned int)bmi_error, (unsigned long)(bmi_attempt + 1U));

      if (bmi_error_length > 0 && bmi_error_length < (int)sizeof(bmi_error_buffer))
      {
        (void)HAL_UART_Transmit(&huart7, (uint8_t *)bmi_error_buffer,
                                (uint16_t)bmi_error_length, HAL_MAX_DELAY);
      }
    }

    bmi_attempt++;
    HAL_Delay(BMI088_INIT_RETRY_MS);
  }

  WS2812_Ctrl(0U, LED_BRIGHTNESS, 0U);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    BMI088_read(gyro, accel, &temp);

    format_sensor_value(bmi_value_buffer[0], sizeof(bmi_value_buffer[0]), gyro[0]);
    format_sensor_value(bmi_value_buffer[1], sizeof(bmi_value_buffer[1]), gyro[1]);
    format_sensor_value(bmi_value_buffer[2], sizeof(bmi_value_buffer[2]), gyro[2]);
    format_sensor_value(bmi_value_buffer[3], sizeof(bmi_value_buffer[3]), accel[0]);
    format_sensor_value(bmi_value_buffer[4], sizeof(bmi_value_buffer[4]), accel[1]);
    format_sensor_value(bmi_value_buffer[5], sizeof(bmi_value_buffer[5]), accel[2]);
    format_sensor_value(bmi_value_buffer[6], sizeof(bmi_value_buffer[6]), temp);
    bmi_uart_length = snprintf(bmi_uart_buffer, sizeof(bmi_uart_buffer),
                   "%s,%s,%s,%s,%s,%s,%s\r\n",
                   bmi_value_buffer[0], bmi_value_buffer[1], bmi_value_buffer[2],
                   bmi_value_buffer[3], bmi_value_buffer[4], bmi_value_buffer[5],
                   bmi_value_buffer[6]);
    if (bmi_uart_length > 0 && bmi_uart_length < (int)sizeof(bmi_uart_buffer))
    {
      if (HAL_UART_Transmit(&huart7, (uint8_t *)bmi_uart_buffer,
                            (uint16_t)bmi_uart_length, HAL_MAX_DELAY) != HAL_OK)
      {
        Error_Handler();
      }
    }

    HAL_Delay(10);


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
