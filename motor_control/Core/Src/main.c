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
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMI088driver.h"
#include "ws2812.h"
#include "motor_power.h"    /* 电机电源开关（PC14 可控电源输出，高电平使能） */
#include "ota_layout.h"      /* OTA_APP_BASE：本镜像链接在哪个槽（见 STM32H723xG_slots.ld） */
#include "ota_trace.h"       /* 启动诊断（Debug 构建才输出，Release 里是空函数） */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 电机相关的参数（MOTOR_COUNT / KEY_DEBOUNCE_MS / USER_KEY_PRESSED_LEVEL /
   轮询周期 ...）都搬到 Device/Motor/motor_ctrl.c 了，要改去那边改。
   这里只留 main.c 自己用得到的。 */

/* ---- 板载 WS2812 指示灯 ----
   颜色用枚举（见 ws2812.h 的 WS2812_COLOR_xxx），亮度不在主流程里设，
   用驱动里的"总亮度"默认值 WS2812_DEFAULT_BRIGHTNESS(32)——要调就改 ws2812.h 那个宏，
   或者开机时自己调 WS2812_SetBrightness()。
   点灯的状态机（MotorCtrl_LedNextColor）搬到了 Device/Motor/motor_ctrl.c，
   顺序不变：
   RED → GREEN → BLUE → YELLOW → CYAN → MAGENTA → WHITE → RED ...（跳过 OFF） */

/* 使能可控 5V 之后，等 5V 轨稳定再说：板载 WS2812（以及后面的 BMI088）都吃这一路。
   BMI088 数据手册要求 VDD 有效后加速度计 ~1ms、陀螺仪 ~30ms 才能访问，
   5V 轨上的滤波电容充电还需要更多时间，这里统一留 100ms。 */
#define POWER_5V_RAMP_UP_MS       100U

/* ---- 下面这些是 BMI088 的，流程暂时关掉了，先留着方便以后放开 ---- */

/* BMI088 初始化失败后的重试间隔，以及最多向串口报告多少次错误码 */
#define BMI088_INIT_RETRY_MS      200U
#define BMI088_INIT_LOG_ATTEMPTS  10U

/* 上位机(https://imu.steppeschool.com/)要求的是二进制包，固定 20 字节：
   2 字节同步头 0xAA 0xFF，后面 9 个 int16(小端)——
   加速度 xyz、陀螺 xyz、磁力 xyz。本板没有磁力计，磁力 3 轴固定填 0。
   同步头之前的字节(比如下面的初始化错误文本)上位机会直接丢掉，不用特意清空。 */
#define BMI088_PACKET_SYNC1       0xAAU
#define BMI088_PACKET_SYNC2       0xFFU
#define BMI088_PACKET_SIZE        20U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* 业务都在 FreeRTOS 任务里了，见 Device/Motor/ 下的 motor_io.c / motor_ctrl.c。
   main.c 只负责：外设初始化 → 5V 上电时序 → 启动调度器。 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
#if 0 /* BMI088 打包用的小工具，数据暂时不发 */
static void packet_put_int16(uint8_t *dst, int16_t value);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t gyro_raw[3], accel_raw[3], temp_raw;

#if 0 /* ===== BMI088 流程暂时关掉，后面要放开时把 #if 0 改成 #if 1 ===== */
static void packet_put_int16(uint8_t *dst, int16_t value)
{
  dst[0] = (uint8_t)(value & 0xFF);
  dst[1] = (uint8_t)((value >> 8) & 0xFF);
}
#endif /* BMI088 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* 启动诊断（只在 Debug 构建里输出，见 Device/Ota/inc/ota_trace.h）：
     它自己先把 USART1 配好（不依赖后面的 HAL/UART 初始化），
     之后每过一步打一个字母。从 Bootloader 跳过来一片安静时，
     串口上最后看到哪个字母，就知道卡在哪一句。 */
  OtaTrace_Init();
  OtaTrace_Text("[app] A: enter main (raw USART1 trace up)\r\n");

  /* ⚠ 必须在任何中断使能之前，把向量表指到**本镜像所在的那个槽**：
     H7 复位时 Flash 被硬件镜像到 0x00000000，而 system_stm32h7xx.c 里
     SCB->VTOR 那句被 #if defined(USER_VECT_TAB_ADDRESS) 包着、默认**不执行**，
     所以 VTOR 一直是复位值 0。从 Bootloader 跳过来时它指的是 Flash 里的
     0x08000000（= BL 自己的向量表），不改的话所有中断都会跑到 BL 的 handler 里去
     （HAL_Delay 卡死、FreeRTOS 不调度）。
     槽基址由链接脚本的 --defsym APP_BASE 给出。 */
  SCB->VTOR = OTA_APP_BASE;
  OtaTrace_Text("[app] B: VTOR = slot base\r\n");

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* 能打到这里 = MPU_Config() + HAL_Init() 都过了。
     注意 HAL 的时基是 **TIM6**（不是 SysTick，SysTick 留给 FreeRTOS 当调度节拍）：
     TIM6 不工作的话下面 HAL_Delay 会一直转（后面的标记就打不出来） */
  OtaTrace_Text("[app] C: MPU + HAL_Init ok (TIM6 timebase)\r\n");

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* 能打到这里 = HSE / PLL / VOSRDY 都过了 */
  OtaTrace_Text("[app] D: SystemClock_Config ok\r\n");

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART7_Init();
  MX_SPI2_Init();
  MX_SPI6_Init();
  MX_USART10_UART_Init();

  /* USER CODE BEGIN 2 */

  /* 五个外设的 HAL 初始化都过了；下一个可能卡死的是 HAL_Delay（要 TIM6 中断） */
  OtaTrace_Text("[app] E: MX_*_Init ok\r\n");

  /* 电机电源（PC14）：**上电默认关断**。趁早把它配成推挽输出并拉低 ——
     复位后到配好之前，那根使能线是悬空的（高电平就会上电），越早配越安全。
     真正的使能在“按 USER_KEY / 无线 ctrl enable”时才做。
     注意它是和板载 5V（PC15）**两路独立**的供电：5V 给 WS2812/BMI088，
     PC14 这路给电机（及其驱动板）。 */
  MotorPwr_Init();

  /* 使能可控 5V：板载 WS2812 指示灯由这一路供电，上电默认是关的 */
  HAL_GPIO_WritePin(Power_5V_EN_GPIO_Port, Power_5V_EN_Pin, GPIO_PIN_SET);

  /* 等 5V 轨爬升稳定 */
  HAL_Delay(POWER_5V_RAMP_UP_MS);

  /* 5V 有了再发一帧全灭：MCU 单独复位时灯珠会保留上一次的颜色 */
  WS2812_Ctrl(0U, 0U, 0U);

  OtaTrace_Text("[app] F: 5V on + WS2812 ok (HAL_Delay 也过了)\r\n");

  /* ⚠ 电机挂在 USART10 上：PE2 = RX、PE3 = TX，38400 8N1（见 usart.c）。
     PE3 (TX) 必须是开漏 (GPIO_MODE_AF_OD)：电机控制板是 5V TTL 单总线，
     高电平靠总线上拉；TX 推挽会让输出 MOS 管常通，在总线上误发信号。
     这个 Mode 是 CubeMX 按 PE3 的 Output type 生成的，要改请在 CubeMX 里改。 */

  /* 业务相关的初始化（日志互斥、电机串口 + 请求队列、轮询任务/信号量）都在
     MX_FREERTOS_Init() 里做：那些对象要求 osKernelInitialize() 已经跑过。 */

  OtaTrace_Text("[app] G: main() done, calling osKernelInitialize()\r\n");

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  OtaTrace_Text("[app] !! osKernelStart returned (should never happen)\r\n");

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if 0 /* ---- 旧主循环：按键使能 + 200ms 状态轮询 ----
         已搬到 Device/Motor/：控制流程见 motor_ctrl.c 的 MotorCtrl_Task()，
         状态轮询见 MotorCtrl_PollTask()。留在这里备查，不再编译。 ---- */
    /* 等一次按键（PA15），按下后：给总线上每台电机各发一次使能指令
       （0xA0/0x08，只发一次、不重试），再把串口收到的 10 字节原样打到 UART7。 */
    Uart7_Print("press USER_KEY (PA15) to send enable ...\r\n");

    WaitUserKeyPress();

    key_round++;
    WS2812_SetColor(Motor_LedNextColor());   /* 按键已按下：换下一个颜色 */

    (void)snprintf(motor_line, sizeof(motor_line),
                   "key pressed (#%u), sending enable (0xA0/0x08)...\r\n",
                   (unsigned int)key_round);
    Uart7_Print(motor_line);

    uint8_t enabled_ok = 0U;

    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
      /* 只发一次使能：Motor_Enable 内部收发各有 100ms 超时。
         注意：反馈里 DATA[2] 是**使能之后的实际模式**，不是回显 0x08 ——
         实测回帧 01 A1 01 00 00 00 00 00 00 E0，模式值 0x01 = 默认的电流环。
         所以只要收到合法的 0xA1 回帧，就说明使能指令被电机接受了。 */
      uint8_t ack_ok = Motor_Enable(motor_ids[i], &motor_enable_ack[i]);

      /* 不管成没成，都把 RX 收到的 10 字节原样打出来：
         全 0 = 一个字节都没收到（没接线/没上电/波特率不对），
         有数据但校验不过 = 波特率或 CRC 对不上（见 motor.h 的协议说明）。 */
      (void)snprintf(motor_line, sizeof(motor_line),
                     "id=%u enable sent, reply=%s, rx="
                     "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                     (unsigned int)motor_ids[i], (ack_ok != 0U) ? "OK" : "NONE",
                     motor_enable_ack[i].raw[0], motor_enable_ack[i].raw[1],
                     motor_enable_ack[i].raw[2], motor_enable_ack[i].raw[3],
                     motor_enable_ack[i].raw[4], motor_enable_ack[i].raw[5],
                     motor_enable_ack[i].raw[6], motor_enable_ack[i].raw[7],
                     motor_enable_ack[i].raw[8], motor_enable_ack[i].raw[9]);
      Uart7_Print(motor_line);

      if (ack_ok != 0U)
      {
        (void)snprintf(motor_line, sizeof(motor_line),
                       "id=%u ack 0xA1 mode=0x%02X (%s)\r\n",
                       (unsigned int)motor_ids[i], motor_enable_ack[i].mode,
                       Motor_ModeName(motor_enable_ack[i].mode));
        Uart7_Print(motor_line);

        enabled_ok++;
      }
    }

    /* 每台都回了 0xA1 就点下一个颜色；否则保持当前颜色，再按一次重来 */
    if (enabled_ok == MOTOR_COUNT)
    {
      WS2812_SetColor(Motor_LedNextColor());

      uint8_t led_r = 0U;
      uint8_t led_g = 0U;
      uint8_t led_b = 0U;
      WS2812_GetOutput(&led_r, &led_g, &led_b);

      (void)snprintf(motor_line, sizeof(motor_line),
                     "enable OK. LED -> next color (%u,%u,%u), brightness=%u.\r\n",
                     (unsigned int)led_r, (unsigned int)led_g, (unsigned int)led_b,
                     (unsigned int)WS2812_GetBrightness());
      Uart7_Print(motor_line);

      /* ---- ① 版本号（0xFD -> 0xFE）：使能成功之后只查一次 ---- */
      for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
      {
        MotorVersion version;
        uint8_t ver_ok = Motor_QueryVersion(motor_ids[i], &version);

        /* 没收到也把 10 字节原样打出来：全 0 = 一个字节都没收到 */
        (void)snprintf(motor_line, sizeof(motor_line),
                       "id=%u version query (0xFD): reply=%s, rx="
                       "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                       (unsigned int)motor_ids[i], (ver_ok != 0U) ? "OK" : "NONE",
                       version.raw[0], version.raw[1], version.raw[2], version.raw[3],
                       version.raw[4], version.raw[5], version.raw[6], version.raw[7],
                       version.raw[8], version.raw[9]);
        Uart7_Print(motor_line);

        if (ver_ok != 0U)
        {
          /* 年字节是 20XX 的 XX（2021 年 = 0x15 = 十进制 21），所以按十进制打 */
          (void)snprintf(motor_line, sizeof(motor_line),
                         "id=%u version: date=20%02u-%02u-%02u, model=0x%02X, fw=0x%02X, hw=0x%02X\r\n",
                         (unsigned int)motor_ids[i],
                         (unsigned int)version.year, (unsigned int)version.month,
                         (unsigned int)version.day, (unsigned int)version.model,
                         (unsigned int)version.fw_version, (unsigned int)version.hw_version);
          Uart7_Print(motor_line);
        }
      }

      /* ---- ② 之后一直循环查里程/位置/故障码（0x74 -> 0x75），全部打在 UART7 ---- */
      Uart7_Print("enter status loop (0x74: mileage/position/fault) ...\r\n");

      while (1)
      {
        for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
        {
          MotorStatus status;
          char fault_text[48];
          uint8_t st_ok = Motor_QueryStatus(motor_ids[i], &status);

          Motor_FaultText(status.fault, fault_text, sizeof(fault_text));

          /* 位置原始值 0~32767 对应 0~360°，要角度自己换算（x * 360 / 32767） */
          (void)snprintf(motor_line, sizeof(motor_line),
                         "id=%u status (0x74): reply=%s, mileage=%ld, position=%u, "
                         "fault=0x%02X (%s), rx="
                         "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                         (unsigned int)motor_ids[i], (st_ok != 0U) ? "OK" : "NONE",
                         (long)status.mileage, (unsigned int)status.position,
                         (unsigned int)status.fault, fault_text,
                         status.raw[0], status.raw[1], status.raw[2], status.raw[3],
                         status.raw[4], status.raw[5], status.raw[6], status.raw[7],
                         status.raw[8], status.raw[9]);
          Uart7_Print(motor_line);
        }

        /* 每查完一轮换一个灯色 = 轮询心跳（也顺带确认主循环还活着） */
        WS2812_SetColor(Motor_LedNextColor());

        HAL_Delay(MOTOR_STATUS_POLL_MS);
      }
    }
#endif /* 旧主循环 */

#if 0 /* BMI088 的数据本轮先不发，要放开时把这里改回 #if 1，
         同时把 USER CODE 2 / USER CODE 0 里对应的 #if 0 一起打开。 */
    uint8_t bmi_packet[BMI088_PACKET_SIZE];

    BMI088_read_raw(accel_raw, gyro_raw, &temp_raw);

    /* 20 字节包：同步头 + 加速度 + 陀螺 + 磁力(本板没有，填 0)
       本板 BMI088 的贴装和网页参考板不同，这里对传感器数据做了一次轴向映射：
       X = -传感器 Y、Y = 传感器 X、Z 不变(加速度和陀螺用同一套映射)；
       网页本身还会对 ax/gy/gz 取反。如果 3D 模型的转向还是不对，
       就继续调这里 6 个分量的符号/顺序。 */
    bmi_packet[0] = BMI088_PACKET_SYNC1;
    bmi_packet[1] = BMI088_PACKET_SYNC2;
    packet_put_int16(&bmi_packet[2], (int16_t)(-accel_raw[1]));
    packet_put_int16(&bmi_packet[4], accel_raw[0]);
    packet_put_int16(&bmi_packet[6], accel_raw[2]);
    packet_put_int16(&bmi_packet[8], (int16_t)(-gyro_raw[1]));
    packet_put_int16(&bmi_packet[10], gyro_raw[0]);
    packet_put_int16(&bmi_packet[12], gyro_raw[2]);
    packet_put_int16(&bmi_packet[14], 0);
    packet_put_int16(&bmi_packet[16], 0);
    packet_put_int16(&bmi_packet[18], 0);

    if (HAL_UART_Transmit(&huart7, bmi_packet, BMI088_PACKET_SIZE, HAL_MAX_DELAY) != HAL_OK)
    {
      Error_Handler();
    }

    HAL_Delay(10);
#endif /* BMI088 */
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
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
