#ifndef __WS2812_H__
#define __WS2812_H__

#include "main.h"
#include "spi.h"

/*
 * 板载 WS2812 指示灯。
 *
 * 数据输入 (DIN) 接在 SPI6_MOSI 上，也就是 PA7，见 hello_imu.ioc 里
 * PA7.Signal=SPI6_MOSI / DM-MC-Board02 原理图 "2812指示灯" 部分。
 * 用 SPI 的 MOSI 波形模拟 WS2812 的单总线时序，所以只配置了 MOSI，
 * 没有占用 SPI6_SCK(PA5)，PA5 仍然是 ADC1_CH18_KEY 按键。
 */
#define WS2812_SPI_UNIT     hspi6

/* 默认总亮度：上电后没调过亮度时用这个值，0-255 */
#define WS2812_DEFAULT_BRIGHTNESS  32U

/* ---- 底层接口：直接写各通道 PWM 值 ---- */

/* 直接把 r/g/b（0-255）发给灯珠并锁存，不做亮度换算，
   调用方自己算好三个值时才用它（上层接口见下面） */
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);

/* ---- 上层接口：颜色(比例) + 总亮度 ----
 * "总亮度" 0-255 指的是 R+G+B 三个通道**之和**；
 * "颜色" 只表示混色比例，驱动按比例把总亮度均摊到各通道：
 *     通道输出 = 总亮度 x 该通道权重 / 权重和
 * 例：颜色 CYAN(0,255,255) + 亮度 32 → (0,16,16)；
 *     颜色 RED(255,0,0)   + 亮度 32 → (32,0,0)（纯色就全给它）。
 * 这样多亮一个通道不会让灯整体变更亮。
 */

/* 颜色枚举：只选色调，亮度不在这里 */
typedef enum
{
    WS2812_COLOR_OFF = 0,   /* 灭 */
    WS2812_COLOR_RED,       /* 红 */
    WS2812_COLOR_GREEN,     /* 绿 */
    WS2812_COLOR_BLUE,      /* 蓝 */
    WS2812_COLOR_YELLOW,    /* 黄 = 红+绿 */
    WS2812_COLOR_CYAN,      /* 青/蓝绿 = 绿+蓝 */
    WS2812_COLOR_MAGENTA,   /* 紫 = 红+蓝 */
    WS2812_COLOR_WHITE,     /* 白 = 红+绿+蓝 */
    WS2812_COLOR_COUNT      /* 枚举个数，不是颜色 */
} WS2812_Color_t;

/* 只改总亮度（0-255），颜色不变，立即刷新 */
void WS2812_SetBrightness(uint8_t brightness);

/* 唯一一个直接给 r/g/b 的接口：各通道 0-255 比例，总亮度用当前值，立即刷新 */
void WS2812_SetColorRGB(uint8_t r, uint8_t g, uint8_t b);

/* 用枚举选颜色，总亮度用当前值，立即刷新 */
void WS2812_SetColor(WS2812_Color_t color);

/* 读回当前的设置（颜色是比例，不是实际 PWM 值） */
uint8_t WS2812_GetBrightness(void);
void    WS2812_GetColorRGB(uint8_t *r, uint8_t *g, uint8_t *b);

/* 读回实际发给灯珠的 PWM 值（0-255），即 WS2812_Ctrl 收到的那三个数 */
void        WS2812_GetOutput(uint8_t *r, uint8_t *g, uint8_t *b);

#endif /* __WS2812_H__ */
