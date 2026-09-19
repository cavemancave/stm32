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

/* 上层接口保存的状态：颜色比例 + 总亮度。
   上电默认：灯灭、总亮度 WS2812_DEFAULT_BRIGHTNESS(32) */
static uint8_t ws2812_color_r    = 0U;
static uint8_t ws2812_color_g    = 0U;
static uint8_t ws2812_color_b    = 0U;
static uint8_t ws2812_brightness = WS2812_DEFAULT_BRIGHTNESS;

/* 颜色枚举 → 各通道比例(0-255) */
static const uint8_t ws2812_color_table[WS2812_COLOR_COUNT][3] =
{
    [WS2812_COLOR_OFF]     = {0U,   0U,   0U  },
    [WS2812_COLOR_RED]     = {255U, 0U,   0U  },
    [WS2812_COLOR_GREEN]   = {0U,   255U, 0U  },
    [WS2812_COLOR_BLUE]    = {0U,   0U,   255U},
    [WS2812_COLOR_YELLOW]  = {255U, 255U, 0U  },
    [WS2812_COLOR_CYAN]    = {0U,   255U, 255U},
    [WS2812_COLOR_MAGENTA] = {255U, 0U,   255U},
    [WS2812_COLOR_WHITE]   = {255U, 255U, 255U},
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  按当前"颜色比例 + 总亮度"算出一帧实际 PWM 值并刷新灯珠
  * @note   通道输出 = 总亮度 x 该通道权重 / 权重和（整数除法，向下取整）
  * @retval None
  */
static void WS2812_Refresh(void)
{
    uint16_t weight_sum = (uint16_t)ws2812_color_r + (uint16_t)ws2812_color_g +
                          (uint16_t)ws2812_color_b;
    uint8_t  r = 0U;
    uint8_t  g = 0U;
    uint8_t  b = 0U;

    if (weight_sum != 0U)
    {
        r = (uint8_t)(((uint16_t)ws2812_brightness * (uint16_t)ws2812_color_r) / weight_sum);
        g = (uint8_t)(((uint16_t)ws2812_brightness * (uint16_t)ws2812_color_g) / weight_sum);
        b = (uint8_t)(((uint16_t)ws2812_brightness * (uint16_t)ws2812_color_b) / weight_sum);
    }

    WS2812_Ctrl(r, g, b);
}

/**
  * @brief  底层接口：直接写各通道 PWM 值(0-255)并发送、锁存
  * @note   不含亮度换算；一般用下面的 WS2812_SetBrightness / WS2812_SetColor / WS2812_SetColorRGB
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

/**
  * @brief  只设置总亮度，颜色比例不变
  * @param  brightness - 0-255，R+G+B 之和；0 = 灭
  * @retval None
  */
void WS2812_SetBrightness(uint8_t brightness)
{
    ws2812_brightness = brightness;
    WS2812_Refresh();
}

/**
  * @brief  唯一一个直接给 r/g/b 的接口：各通道权重 0-255，总亮度用当前值
  * @param  r/g/b - 各通道权重 0-255，只表示混色比例
  * @retval None
  */
void WS2812_SetColorRGB(uint8_t r, uint8_t g, uint8_t b)
{
    ws2812_color_r = r;
    ws2812_color_g = g;
    ws2812_color_b = b;
    WS2812_Refresh();
}

/**
  * @brief  用枚举选颜色，总亮度用当前值
  * @param  color - WS2812_COLOR_xxx；越界值按灭处理
  * @retval None
  */
void WS2812_SetColor(WS2812_Color_t color)
{
    if ((uint32_t)color >= (uint32_t)WS2812_COLOR_COUNT)
    {
        color = WS2812_COLOR_OFF;
    }

    WS2812_SetColorRGB(ws2812_color_table[color][0],
                       ws2812_color_table[color][1],
                       ws2812_color_table[color][2]);
}

/**
  * @brief  读回当前总亮度
  * @retval 0-255
  */
uint8_t WS2812_GetBrightness(void)
{
    return ws2812_brightness;
}

/**
  * @brief  读回当前颜色比例
  * @param  r/g/b - 非 NULL 时写入对应通道权重（0-255）
  * @retval None
  */
void WS2812_GetColorRGB(uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (r != NULL) { *r = ws2812_color_r; }
    if (g != NULL) { *g = ws2812_color_g; }
    if (b != NULL) { *b = ws2812_color_b; }
}

/**
  * @brief  读回实际发给灯珠的 PWM 值（也就是上层接口换算后的结果）
  * @param  r/g/b - 非 NULL 时写入对应的 0-255 PWM 值
  * @retval None
  */
void WS2812_GetOutput(uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint16_t weight_sum = (uint16_t)ws2812_color_r + (uint16_t)ws2812_color_g +
                          (uint16_t)ws2812_color_b;

    if (weight_sum == 0U)
    {
        if (r != NULL) { *r = 0U; }
        if (g != NULL) { *g = 0U; }
        if (b != NULL) { *b = 0U; }
        return;
    }

    if (r != NULL) { *r = (uint8_t)(((uint16_t)ws2812_brightness * (uint16_t)ws2812_color_r) / weight_sum); }
    if (g != NULL) { *g = (uint8_t)(((uint16_t)ws2812_brightness * (uint16_t)ws2812_color_g) / weight_sum); }
    if (b != NULL) { *b = (uint8_t)(((uint16_t)ws2812_brightness * (uint16_t)ws2812_color_b) / weight_sum); }
}
