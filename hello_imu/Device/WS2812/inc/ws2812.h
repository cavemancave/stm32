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

/* 设置这一颗灯珠的颜色，r/g/b 取值范围 0-255（0 = 灭） */
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);

#endif /* __WS2812_H__ */
