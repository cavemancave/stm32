/**
  ******************************************************************************
  * @file    power_mon.c
  * @brief   电源电压监测（ADC1_INP4 / PC4）实现
  *
  * 细节见 power_mon.h。这里只说实现上的几个点：
  *
  *   1. ADC 本体是 CubeMX 生成的（Core/Src/adc.c: MX_ADC1_Init + HAL_ADC_MspInit），
  *      16 位分辨率、通道 4、387.5 周期采样、ADC 时钟来自 PLL2 = 48 MHz。
  *      所以本文件只 include "adc.h" 拿 hadc1，不再自己配时钟/引脚。
  *      ⚠ 采样时间要够长：分压电阻的源阻抗 = R86∥R87 ≈ 91 kΩ，
  *        387.5 周期 @48 MHz ≈ 8.1 µs 才够采样电容充满（12 位 + 91 kΩ 约需 3.7 µs）。
  *        以后在 CubeMX 里把采样时间调短了，读数会偏低 —— 别调。
  *
  *   2. 全量程按**当前配置的分辨率**算（不写死 65535）：万一以后在 CubeMX 里
  *      把 16 位改成 12 位，换算自动跟着对，不用改代码。
  *
  *   3. 换算全程用 uint64_t：raw(65535) × VREF(3300) × 比(11) ≈ 2.4e9 已经超了
  *      32 位有符号的范围，用 uint32_t 会静默算错。
  *
  *   4. 采样动作只有后台任务一个人做（PowerMon_Update），别人只读缓存
  *      （对齐的 32 位读写是原子的），所以整个模块不用加锁。
  ******************************************************************************
  */

#include "power_mon.h"

#include "adc.h"          /* CubeMX 生成的 hadc1 / MX_ADC1_Init() */
#include "uart_log.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"

#include <stdio.h>

/* Private defines -----------------------------------------------------------*/

/* 分压比 = (R86 + R87) / R87（都是 kΩ）。改电阻只需改 power_mon.h 里的两个宏。
   ⚠ 这个宏是"上+下"的和（1100），换算时要除以下面那个 100；
     打印给用户看的“分压比”是 POWER_MON_DIV_RATIO（= 11）—— 别把 1100 直接打出去。 */
#define POWER_MON_RATIO           (POWER_MON_R_TOP_KOHM + POWER_MON_R_BOTTOM_KOHM)
#define POWER_MON_DIV_RATIO       (POWER_MON_RATIO / POWER_MON_R_BOTTOM_KOHM)

/* 顶到多少就算"饱和/不对劲"：分压 11:1 + VREF 3.3 V ⇒ ADC 满量程 = 36.3 V，
   到那里码值就卡在 65535 不再涨了。所以这个阈值**必须小于 36.3 V**，
   否则 ADC 已经饱和也报不出来（图上 VCC_IN 到底接了多少都看不出来）。
   34 V 留了 2.3 V 余量：够区分"刚好饱和"和"真的只有 33 V"。 */
#define POWER_MON_MAX_VALID_MV    34000U

/* 上报状态：日志只在**状态跳变**时打一条，正常时一秒一条会把无线口塞满 */
enum
{
    PM_ST_UNKNOWN = 0,   /* 还没采过 */
    PM_ST_OK,            /* 正常 */
    PM_ST_LOW,           /* 低压（单片低于 POWER_MON_LOW_CELL_MV） */
    PM_ST_HIGH,          /* 过压（单片高于 POWER_MON_HIGH_CELL_MV） */
    PM_ST_OVER,          /* 顶到量程上限（分压电阻/接线/VREF 假设要核对） */
    PM_ST_FAIL           /* 采样失败（ADC 超时） */
};

/* Private variables ---------------------------------------------------------*/

static osThreadId_t  s_task   = NULL;
static uint8_t       s_cal_ok = 0U;       /* 自校准是否成功 */
static uint8_t       s_ok     = 0U;       /* 最近一次采样是否成功 */
static uint8_t       s_state  = PM_ST_UNKNOWN;

/* 电池串数：1..POWER_MON_CELLS_MAX（没有 0，见 power_mon.h）。
   从 OTA 任务里改（ctrl cells）、从采样任务里读，一个字节的读写是原子的，不用加锁。 */
static volatile uint8_t s_cells = POWER_MON_DEFAULT_CELLS;

static volatile uint32_t s_mv     = 0U;   /* 缓存：VCC_IN 电压 mV */
static volatile uint32_t s_pin_mv = 0U;   /* 缓存：PC4 引脚电压 mV */
static volatile uint16_t s_raw    = 0U;   /* 缓存：原始 ADC 码值 */

/* Private functions ---------------------------------------------------------*/

static const osThreadAttr_t s_task_attr =
{
    .name       = "powerMon",
    /* 栈 1024：power_mon_report() 里有 5 个 16 字节小缓冲 + 一个 192 字节行缓冲，
       再加 snprintf 自己的开销（512 时 -Wformat-truncation 之后一路看着很危险）。 */
    .stack_size = 1024U,
    .priority   = (osPriority_t)osPriorityLow, /* 纯监视，优先级放到最低 */
};

/* 全量程 mV：VREF × 分压比（= 3300 × 11 = 36300 mV ⇒ 36.30 V） */
static uint32_t power_mon_full_scale_mv(void)
{
    return (uint32_t)(((uint64_t)POWER_MON_VREF_MV * POWER_MON_RATIO) / POWER_MON_R_BOTTOM_KOHM);
}

/* 全量程：跟着 CubeMX 里配的分辨率走（16 位 → 65535） */
static uint32_t power_mon_full_scale(void)
{
    switch (hadc1.Init.Resolution)
    {
        case ADC_RESOLUTION_16B:      return 65535U;
        case ADC_RESOLUTION_14B:      return 16383U;
        case ADC_RESOLUTION_12B:      return 4095U;
        case ADC_RESOLUTION_10B:      return 1023U;
        case ADC_RESOLUTION_8B:       return 255U;
        default:                      return 65535U;
    }
}

/* 软件启动一次单通道转换并等结果。成功返回 1，*raw 拿到码值。 */
static uint8_t power_mon_convert(uint16_t *raw)
{
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_ADC_PollForConversion(&hadc1, POWER_MON_ADC_TIMEOUT_MS) != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc1);
        return 0U;
    }

    *raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);

    return 1U;
}

/* 单片电压 mV（四舍五入） */
static uint32_t power_mon_cell_mv(uint32_t mv)
{
    uint32_t cells = s_cells;

    return (mv + (cells / 2U)) / cells;
}

/* 锂聚合物（单串）静置电压 → 剩余电量 近似曲线。
   ⚠ 为什么不用一条直线从 3.30 拉到 4.20：锂电的放电曲线中段很平，
     直线会把“3.85 V”估成 60% 左右，而实际大概只剩 55%/50%，中段普遍高估。
     这张表是常见的 LiPo 静置电压/容量近似值，插值算就够了。
   ⚠ 这是**看电压估电量**（不是库仑计）：带载会跌、刚充完会偏高；
     12 V 开关电源按 3S 算时这个百分比同样只是个数。 */
static const uint16_t s_lipo_pct_curve[][2] =
{
    { 4200U, 100U },
    { 4100U,  90U },
    { 4000U,  80U },
    { 3900U,  65U },
    { 3800U,  50U },
    { 3700U,  35U },
    { 3600U,  20U },
    { 3500U,  10U },
    { 3400U,   5U },
    { 3300U,   0U },
};

#define POWER_MON_CURVE_POINTS  (sizeof(s_lipo_pct_curve) / sizeof(s_lipo_pct_curve[0]))

/* 单片电压 mV → 百分比（0..100） */
static uint8_t power_mon_percent_of(uint32_t cell_mv)
{
    if (cell_mv >= s_lipo_pct_curve[0][0])
    {
        return 100U;      /* 4.20 V 以上就是满 */
    }

    for (uint32_t i = 0U; (i + 1U) < POWER_MON_CURVE_POINTS; i++)
    {
        uint32_t v_hi = s_lipo_pct_curve[i][0];
        uint32_t v_lo = s_lipo_pct_curve[i + 1U][0];

        if (cell_mv >= v_lo)
        {
            uint32_t p_hi = s_lipo_pct_curve[i][1];
            uint32_t p_lo = s_lipo_pct_curve[i + 1U][1];

            /* 在这两点之间线性插值（四舍五入） */
            return (uint8_t)(p_lo + ((((cell_mv - v_lo) * (p_hi - p_lo))
                                      + ((v_hi - v_lo) / 2U)) / (v_hi - v_lo)));
        }
    }

    return 0U;            /* 3.30 V 以下就是 0% */
}

/* 由总压 + 当前串数得出“应该报哪个状态”（采样失败单独在任务里判） */
static uint8_t power_mon_state_of(uint32_t mv)
{
    if (mv > POWER_MON_MAX_VALID_MV)
    {
        return PM_ST_OVER;      /* 先看是否顶到量程：这时单片也不算数 */
    }

    {
        uint32_t cell = power_mon_cell_mv(mv);

        if (cell > POWER_MON_HIGH_CELL_MV)
        {
            return PM_ST_HIGH;
        }

        if (cell < POWER_MON_LOW_CELL_MV)
        {
            return PM_ST_LOW;
        }
    }

    return PM_ST_OK;
}

/* 状态跳变时打一条日志（同状态重复调用什么都不做） */
static void power_mon_report(uint8_t state)
{
    /* ⚠ 行缓冲要够大：低压那条把“单片 + 百分比 + 阈值 + 串数 + 总压 + 提醒”都塞进一行，
       最坏 179 字节（GCC 的 -Wformat-truncation 会盯着），所以给 192。 */
    char line[192];
    char txt[16];
    char cell[16];
    char low[16];
    char high[16];
    char full[16];
    uint8_t pct;

    if (state == s_state)
    {
        return;
    }

    PowerMon_Format(txt, sizeof(txt), s_mv);
    PowerMon_Format(cell, sizeof(cell), power_mon_cell_mv(s_mv));
    PowerMon_Format(low, sizeof(low), POWER_MON_LOW_CELL_MV);
    PowerMon_Format(high, sizeof(high), POWER_MON_HIGH_CELL_MV);
    PowerMon_Format(full, sizeof(full), power_mon_full_scale_mv());
    pct = power_mon_percent_of(power_mon_cell_mv(s_mv));

    switch (state)
    {
        case PM_ST_FAIL:
            UartLog_Print("[power] ADC read failed (timeout) - keeping last good value\r\n");
            break;

        case PM_ST_OVER:
            /* 长行拆成两行：无线口上太长的一行在串口助手里很难读 */
            (void)snprintf(line, sizeof(line),
                           "[power] RANGE: %s hit the ceiling (divider %lu:1, full scale %s)\r\n",
                           txt, (unsigned long)POWER_MON_DIV_RATIO, full);
            UartLog_Print(line);
            UartLog_Print("[power]   check wiring / divider resistors / VREF assumption\r\n");
            break;

        case PM_ST_HIGH:
            (void)snprintf(line, sizeof(line),
                           "[power] OVER-VOLT: cell %s (%u%%) > %s/cell (%uS)\r\n",
                           cell, (unsigned int)pct, high, (unsigned int)s_cells);
            UartLog_Print(line);
            break;

        case PM_ST_LOW:
            (void)snprintf(line, sizeof(line),
                           "[power] LOW: cell %s (%u%% left) < %s/cell (%uS pack, total %s)\r\n",
                           cell, (unsigned int)pct, low, (unsigned int)s_cells, txt);
            UartLog_Print(line);
            UartLog_Print("[power]   on the 12V supply? send 'ctrl 12v' (= treat it as 3S)\r\n");
            break;

        case PM_ST_OK:
            if (s_state != PM_ST_UNKNOWN)
            {
                /* 从低压/过压/采样失败回到正常：报一次，方便对着万用表看 */
                (void)snprintf(line, sizeof(line),
                               "[power] VCC_IN back to normal: %s (cell %s, %u%%)\r\n",
                               txt, cell, (unsigned int)pct);
                UartLog_Print(line);
            }
            /* 首次拿到正常值：PowerMon_Start 已经报过了，不重复 */
            break;

        default:
            break;
    }

    s_state = state;
}

static void power_mon_task(void *argument)
{
    (void)argument;

    for (;;)
    {
        osDelay(pdMS_TO_TICKS(POWER_MON_PERIOD_MS));

        PowerMon_Update();

        power_mon_report((s_ok == 0U) ? PM_ST_FAIL : power_mon_state_of(s_mv));
    }
}

/* Exported functions --------------------------------------------------------*/

void PowerMon_Init(void)
{
    /* 上电自校准（H7 的 ADC 必须先校准再用）。
       HAL_ADC_Init 里已经把稳压器打开并等过稳定时间、配了 BOOST，
       这里只做"偏移校准"。 */
    s_cal_ok = (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) == HAL_OK)
               ? 1U : 0U;

    if (s_cal_ok == 0U)
    {
        return;      /* 校准都没过，别采了（PowerMon_Start 里会报出来） */
    }

    PowerMon_Update();
}

void PowerMon_Update(void)
{
    uint32_t sum   = 0U;
    uint32_t count = 0U;
    uint16_t raw   = 0U;

    for (uint32_t i = 0U; i < POWER_MON_SAMPLES; i++)
    {
        if (power_mon_convert(&raw) != 0U)
        {
            sum += raw;
            count++;
        }
    }

    if (count == 0U)
    {
        s_ok = 0U;                 /* 保留上一次的缓存值，别把它清成 0 */
        return;
    }

    uint32_t raw_avg  = sum / count;
    uint32_t full     = power_mon_full_scale();
    uint32_t pin_mv   = (uint32_t)((((uint64_t)raw_avg * POWER_MON_VREF_MV) + (full / 2U)) / full);
    uint32_t input_mv = (uint32_t)((((uint64_t)pin_mv * POWER_MON_RATIO)
                                    + (POWER_MON_R_BOTTOM_KOHM / 2U)) / POWER_MON_R_BOTTOM_KOHM);

    s_raw    = raw_avg;
    s_pin_mv = pin_mv;
    s_mv     = input_mv;
    s_ok     = 1U;
}

uint32_t PowerMon_GetMv(void)
{
    return s_mv;
}

uint32_t PowerMon_GetPinMv(void)
{
    return s_pin_mv;
}

uint16_t PowerMon_GetRaw(void)
{
    return s_raw;
}

uint8_t PowerMon_Ok(void)
{
    return (s_cal_ok != 0U) ? s_ok : 0U;
}

void PowerMon_Format(char *out, uint32_t out_size, uint32_t mv)
{
    if ((out == NULL) || (out_size == 0U))
    {
        return;
    }

    (void)snprintf(out, out_size, "%lu.%02luV",
                   (unsigned long)(mv / 1000U), (unsigned long)((mv % 1000U) / 10U));
}

/* ---- 电池串数 / 单片电压 / 告警 ------------------------------------------ */

void PowerMon_SetCells(uint8_t cells)
{
    if (cells < POWER_MON_CELLS_MIN)
    {
        cells = POWER_MON_CELLS_MIN;
    }

    if (cells > POWER_MON_CELLS_MAX)
    {
        cells = POWER_MON_CELLS_MAX;
    }

    s_cells = cells;

    /* 立刻按新的串数重判一次：比如带 6S 报警时切到 `ctrl 12v`（3S），
       12 V 那套就当场不再算低压了，不用等下一次采样（日志只在状态跳变时打，不会刷屏）。 */
    if (s_ok != 0U)
    {
        power_mon_report(power_mon_state_of(s_mv));
    }
}

uint8_t PowerMon_GetCells(void)
{
    return s_cells;
}

uint32_t PowerMon_GetCellMv(void)
{
    return (s_ok != 0U) ? power_mon_cell_mv(s_mv) : 0U;
}

uint8_t PowerMon_GetPercent(void)
{
    if (s_ok == 0U)
    {
        return POWER_MON_PCT_INVALID;
    }

    return power_mon_percent_of(power_mon_cell_mv(s_mv));
}

uint8_t PowerMon_Low(void)
{
    return ((s_ok != 0U) && (power_mon_state_of(s_mv) == PM_ST_LOW)) ? 1U : 0U;
}

uint8_t PowerMon_High(void)
{
    uint8_t st = (s_ok != 0U) ? power_mon_state_of(s_mv) : PM_ST_FAIL;

    return ((st == PM_ST_HIGH) || (st == PM_ST_OVER)) ? 1U : 0U;
}

void PowerMon_Start(void)
{
    char line[160];
    char txt[16];
    char cell[16];
    char full[16];

    /* ⚠ 日志在这里打、不在 PowerMon_Init 里：Init 是在 main() 的 USER CODE 2 里
       调用的，那会儿 UartLog_Init 还没跑（它在 MX_FREERTOS_Init 里），
       打出去的东西会被直接丢掉。 */

    if (s_task == NULL)
    {
        s_task = osThreadNew(power_mon_task, NULL, &s_task_attr);
    }

    if (s_cal_ok == 0U)
    {
        UartLog_Print("[power] ADC calibration FAILED - readings unreliable\r\n");
        return;
    }

    if (s_ok == 0U)
    {
        UartLog_Print("[power] first sample failed (ADC timeout)\r\n");
        return;
    }

    PowerMon_Format(txt, sizeof(txt), s_mv);
    PowerMon_Format(cell, sizeof(cell), PowerMon_GetCellMv());

    /* 开机就这两行：第一行只给人看（电压/单片/百分比），
       第二行是排查用的原始量（码值/分压/参考/满量程）。
       长行拆开是因为无线口上太长的一行在串口助手里很难读。 */
    (void)snprintf(line, sizeof(line),
                   "[power] VCC_IN = %s, cell %s (%uS), ~%u%%\r\n",
                   txt, cell, (unsigned int)s_cells, (unsigned int)PowerMon_GetPercent());
    UartLog_Print(line);

    PowerMon_Format(full, sizeof(full), power_mon_full_scale_mv());
    (void)snprintf(line, sizeof(line),
                   "[power]   ADC1_INP4/PC4 raw=%u, divider %lu:1, VREF=%lu mV, full scale %s\r\n",
                   (unsigned int)s_raw, (unsigned long)POWER_MON_DIV_RATIO,
                   (unsigned long)POWER_MON_VREF_MV, full);

    UartLog_Print(line);

    /* 开局这条已经报过了，把状态置成实际状态：之后的日志只报跳变。
       （低压开局也算“跳过未知”，没关系 —— 上面这行的单片电压已经写在日志里了。） */
    s_state = power_mon_state_of(s_mv);
}
