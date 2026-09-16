/**
  ******************************************************************************
  * @file    ws2812.c
  * @brief   板载 WS2812 指示灯驱动（用 SPI6 MOSI 模拟单总线时序）
  ******************************************************************************
  * 时序原理:
  *   一个 WS2812 数据位用 1 个 SPI 字节 (8 bit) 表示:
  *     '0' -> 0x60，高电平占 2 个 SPI bit
  *     '1' -> 0x78，高电平占 4 个 SPI bit
  *   SPI6 内核时钟取 HSE = 24MHz，预分频 4 => 6MHz，1 个 SPI bit = 166.7ns:
  *     T0H =  333ns   (WS2812B 规格 400ns ±150ns)
  *     T1H =  667ns   (WS2812B 规格 800ns ±150ns)
  *     位周期 = 1333ns (WS2812B 规格 1250ns ±600ns)
  *   一帧 24 bit，字节顺序 G-R-B，之后补一段低电平 >50us 锁存。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ws2812.h"

/* Private define ------------------------------------------------------------*/
#define WS2812_BIT_0        0x60U   /* '0' 的 SPI 波形 */

#define WS2812_BIT_1        0x78U   /* '1' 的 SPI 波形 */

#define WS2812_FRAME_BYTES  24U     /* 3 色 x 8 bit，每 bit 占 1 字节 */

#define WS2812_RESET_BYTES  100U    /* 100 字节低电平 = 133us，用于锁存 */

#define WS2812_TIMEOUT_MS   10U     /* 一帧实际只要 ~165us */

/* Private variables ---------------------------------------------------------*/
/* 锁存用的低电平缓冲，只读，不要在中断里调用本驱动 */
static const uint8_t ws2812_reset_buffer[WS2812_RESET_BYTES] = {0};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  设置灯珠颜色
  * @param  r/g/b - 红/绿/蓝亮度，0-255
  * @retval None
  */
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[WS2812_FRAME_BYTES];

    /* WS2812 高位先发，字节顺序是 G-R-B */
    for (uint8_t i = 0U; i < 8U; i++)
    {
        txbuf[7U - i]  = ((g >> i) & 0x01U) ? WS2812_BIT_1 : WS2812_BIT_0;
        txbuf[15U - i] = ((r >> i) & 0x01U) ? WS2812_BIT_1 : WS2812_BIT_0;
        txbuf[23U - i] = ((b >> i) & 0x01U) ? WS2812_BIT_1 : WS2812_BIT_0;
    }

    if (HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, WS2812_FRAME_BYTES, WS2812_TIMEOUT_MS) != HAL_OK)
    {
        return;
    }

    /* 一帧发送完必须拉低 >50us，灯珠才会把接收到的颜色锁存显示出来 */
    (void)HAL_SPI_Transmit(&WS2812_SPI_UNIT, ws2812_reset_buffer, WS2812_RESET_BYTES, WS2812_TIMEOUT_MS);
}
