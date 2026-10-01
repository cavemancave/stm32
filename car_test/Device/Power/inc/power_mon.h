/**
  ******************************************************************************
  * @file    power_mon.h
  * @brief   电源电压监测（ADC1_INP4 / PC4，板上的 VCC_IN 分压取样）
  *
  * 硬件（原理图）：
  *
  *     VCC_IN ──[ R86  1 MΩ ]──┬── PC4 (ADC1_INP4)
  *                             │
  *                          [ R87 100 kΩ ]
  *                             │
  *                            GND            C53 10nF/50V 并在 R87 上（取样点滤波）
  *
  *   所以分压比 = (R86 + R87) / R87 = 11，PC4 上看到的电压只有实际电源电压的 1/11。
  *   3.3 V 参考下量程 ≈ 3.3 V × 11 = 36.3 V。
  *
  *   ⚠ 换电阻只需改 POWER_MON_R_TOP_KOHM / POWER_MON_R_BOTTOM_KOHM 两个宏，
  *     其余换算会自动跟着变。
  *
  * ADC 本体（ADC1 的时钟、通道、采样时间、PC4 的模拟模式）由 CubeMX 生成的
  * Core/Src/adc.c（MX_ADC1_Init）负责 —— 也就是说这一路**是勾在 .ioc 里的**，
  * 不像 USART1 那样自己初始化。这里只做上层：校准 + 采样 + 换算 + 缓存。
  *
  * 用法：
  *   main() 里（调度器启动前）   PowerMon_Init();     —— 校准 + 取第一次值
  *   MX_FREERTOS_Init() 里      PowerMon_Start();    —— 起一个 1 Hz 的后台采样任务
  *   任何任务里                 PowerMon_GetMv();    —— 读缓存（不阻塞、不加锁）
  *
  * 为什么要"后台采样 + 读缓存"而不是"谁要谁来采"：
  *   ADC 只有一套寄存器状态（启动/等转换/停），两个任务同时来读会互相踩；
  *   后台任务独占采样动作，别的任务只读一个 32 位缓存变量 ——
  *   Cortex-M 上对齐的 32 位读写是原子的，天然不用加锁。
  ******************************************************************************
  */

#ifndef __POWER_MON_H__
#define __POWER_MON_H__

#include <stdint.h>

/* ---- 分压电阻（原理图上的 R86 / R87，单位 kΩ） ---- */
#define POWER_MON_R_TOP_KOHM      1000U   /* R86：VCC_IN → PC4 */
#define POWER_MON_R_BOTTOM_KOHM   100U    /* R87：PC4 → GND    */

/* ---- ADC 参考电压 ----
   假定 VDDA / VREF+ 就是 3.3 V（板载 LDO）。若实际是 3.0 V 或 3.3 V 略有偏差，
   读数会按同样比例偏 —— 要更准就得读内部 VREFINT 通道做补偿（目前没做），
   或者在标定之后直接改这个宏。 */
#define POWER_MON_VREF_MV         3300U

/* ---- 单次读取的采样次数（取平均，压一压 16 位模式下的噪声） ---- */
#define POWER_MON_SAMPLES         16U

/* ---- 单次转换的等待上限（48 MHz ADC 时钟、387.5 周期采样，一次约 10 µs） ---- */
#define POWER_MON_ADC_TIMEOUT_MS  10U

/* ---- 后台采样任务的周期 ---- */
#define POWER_MON_PERIOD_MS       1000U

/* ---- 电池串数（算“单片电压”用）----
   不管接的是电池还是开关电源，统一按“N 串电池”建模，只有一个旋钮：
     6 = 6S 锂聚合物（现场那套；满 25.2 V，单片 3.30 V 以下算空 ⇒ 19.8 V 告警）
     3 = 12 V 那套（开关电源也能这么算：12.0 V ⇒ 4.00 V/片，落在窗口中间，不会误报；
         真正的窗口是 9.9–12.75 V，对一个“12 V 电源 / 3S 电池”是合理的）
   发 `ctrl cells N` 改（快捷写法见 tools/ota.py：ctrl 6s / 3s / 12v）。
   只存在 RAM 里，复位后回到默认值。
   ⚠ 没有“不判单片”这种特殊值：有特例就会多出一堆 cells==0 的分支，
     换算/告警/显示就不是同一条路了。 */
#define POWER_MON_DEFAULT_CELLS   6U      /* 默认 6S（现场那套是电池，优先保护它） */
#define POWER_MON_CELLS_MIN       1U      /* 至少按 1 串算（防止除零） */
#define POWER_MON_CELLS_MAX       8U      /* 最多按 8 串算：8S 满电 33.6 V 已经贴近 36.3 V 量程上限，
                                             再往上报（9~12S）永远只能报低压，没意义 */
#define POWER_MON_LOW_CELL_MV     3300U   /* 单片低于 3.30 V ⇒ 低压告警 */
#define POWER_MON_HIGH_CELL_MV    4250U   /* 单片高于 4.25 V ⇒ 过压告警（按 N 串算） */

/* ---- 剩余电量百分比 ----
   由**单片电压**按锂电池放电曲线估出来的（不是库仑计），只当“大概还剩多少”看：
     4.20 V/片 → 100%，3.30 V/片 → 0%（中间按附表插值，见 power_mon.c）。
   ⚠ 带载时电压会跌、刚充完会偏高，所以这个数只是参考；
     12 V 开关电源按 3S 算时它同样只是个“数字”，别当真。 */
#define POWER_MON_PCT_INVALID     0xFFU   /* 还没采到值时的“无效”百分比 */

/* 初始化：ADC 自校准 + 先取一次值（**调度器启动前**调用，内部只用 HAL 轮询）。
   注意 CubeMX 生成的 MX_ADC1_Init() 必须先跑过。 */
void PowerMon_Init(void);

/* 起后台采样任务（**MX_FREERTOS_Init() 里**、调度器启动前调用）。 */
void PowerMon_Start(void);

/* 立即采一轮（POWER_MON_SAMPLES 次取平均）并刷新缓存。
   ⚠ 只允许**一个**任务调用（后台任务已经在调了），别在别处再调。 */
void PowerMon_Update(void);

/* 缓存值：VCC_IN 电压，单位 mV（0 = 还没采过）。任意任务/中断里都能读，不加锁。 */
uint32_t PowerMon_GetMv(void);

/* 缓存值：PC4 引脚上的电压，单位 mV（= VCC_IN / 11）。排查分压电阻/接线时看它。 */
uint32_t PowerMon_GetPinMv(void);

/* 缓存值：最近一次的原始 ADC 码值（0 = 还没采过）。排查 ADC 本身时看它。 */
uint16_t PowerMon_GetRaw(void);

/* 最近一次采样是否成功（0 = 失败：超时 / 还没初始化）。 */
uint8_t PowerMon_Ok(void);

/* 方便打日志：把当前电压格式化成 "12.34V" 这种字符串。 */
void PowerMon_Format(char *out, uint32_t out_size, uint32_t mv);

/* ---- 电池串数 / 单片电压 / 告警 ---- */

/* 设置电池串数（1..POWER_MON_CELLS_MAX，越界会被夹到范围内）。
   任意任务里都能调（一个字节的读写是原子的），并会当场按新串数重判一次告警。 */
void PowerMon_SetCells(uint8_t cells);

/* 当前电池串数（总有值：1..POWER_MON_CELLS_MAX） */
uint8_t PowerMon_GetCells(void);

/* 单片电压 mV（还没采到值时返回 0） */
uint32_t PowerMon_GetCellMv(void);

/* 剩余电量百分比 0..100（按单片电压估，见上面说明）；
   还没采到值时返回 POWER_MON_PCT_INVALID(0xFF)。 */
uint8_t PowerMon_GetPercent(void);

/* 低压告警中（单片 < POWER_MON_LOW_CELL_MV） */
uint8_t PowerMon_Low(void);

/* 过压告警中（单片 > POWER_MON_HIGH_CELL_MV，或总压已顶到量程上限） */
uint8_t PowerMon_High(void);

#endif /* __POWER_MON_H__ */
