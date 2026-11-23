/**
 * @file    power_monitor.c
 * @brief   ADC1_INP4 / PC4 读 VCC_IN（1 MΩ/100 kΩ 分压 11:1）。
 *
 * 硬件（DM-MC-Board02）：
 *     VCC_IN --[R86 1M]--+-- PC4 (ADC1_INP4)
 *                        |
 *                     [R87 100k]
 *                        |
 *                       GND
 *   分压比 = (1M + 100k) / 100k = 11
 *   VCC_IN = PC4 电压 * 11；3.3 V 参考下量程约 36.3 V。
 *
 * 注意：这里直接用的是 HAL ADC 驱动（不是 CubeMX 生成的 adc.c）。
 * 如果以后在 CubeMX 里正式启用 ADC1，会生成 Core/Src/adc.c 和另一个
 * HAL_ADC_MspInit，二者只能留一个。
 */
#include "power_monitor.h"
#include "main.h"

#define PM_R_TOP_KOHM       1000U
#define PM_R_BOTTOM_KOHM    100U
#define PM_RATIO            ((PM_R_TOP_KOHM + PM_R_BOTTOM_KOHM) / PM_R_BOTTOM_KOHM)
#define PM_VREF_MV          3300U
#define PM_SAMPLES          8U
#define PM_ADC_TIMEOUT_MS   10U
#define PM_PRESENT_MV       3000U   /* VCC_IN >= 3 V 认为外部电源存在 */
#define PM_PERIOD_MS        1000U

static ADC_HandleTypeDef s_hadc1;
static uint32_t s_vcc_mv;
static uint32_t s_pin_mv;
static uint32_t s_last_sample_ms;
static uint8_t s_valid;
static uint8_t s_present;
static uint8_t s_inited;

/* CubeMX 生成 adc.c 时会覆盖这个 weak 函数；本工程自己实现。 */
void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if (adcHandle->Instance != ADC1) {
        return;
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInitStruct.PLL2.PLL2M = 2;
    PeriphClkInitStruct.PLL2.PLL2N = 40;
    PeriphClkInitStruct.PLL2.PLL2P = 10;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0.0;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        return;
    }

    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PC4 -> ADC1_INP4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static bool pm_convert(uint16_t *raw)
{
    if (HAL_ADC_Start(&s_hadc1) != HAL_OK) {
        return false;
    }
    if (HAL_ADC_PollForConversion(&s_hadc1, PM_ADC_TIMEOUT_MS) != HAL_OK) {
        (void)HAL_ADC_Stop(&s_hadc1);
        return false;
    }
    *raw = (uint16_t)HAL_ADC_GetValue(&s_hadc1);
    (void)HAL_ADC_Stop(&s_hadc1);
    return true;
}

void power_monitor_init(void)
{
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    if (s_inited != 0U) {
        return;
    }

    s_hadc1.Instance = ADC1;
    s_hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    s_hadc1.Init.Resolution = ADC_RESOLUTION_16B;
    s_hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    s_hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    s_hadc1.Init.LowPowerAutoWait = DISABLE;
    s_hadc1.Init.ContinuousConvMode = DISABLE;
    s_hadc1.Init.NbrOfConversion = 1;
    s_hadc1.Init.DiscontinuousConvMode = DISABLE;
    s_hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    s_hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    s_hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    s_hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    s_hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    s_hadc1.Init.OversamplingMode = DISABLE;
    s_hadc1.Init.Oversampling.Ratio = 1U;

    if (HAL_ADC_Init(&s_hadc1) != HAL_OK) {
        return;
    }

    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&s_hadc1, &multimode) != HAL_OK) {
        return;
    }

    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0U;
    sConfig.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(&s_hadc1, &sConfig) != HAL_OK) {
        return;
    }

    if (HAL_ADCEx_Calibration_Start(&s_hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        return;
    }

    s_inited = 1U;
    s_last_sample_ms = 0U;
    power_monitor_poll(0U);
}

void power_monitor_poll(uint32_t now_ms)
{
    uint32_t sum = 0U;
    uint32_t count = 0U;
    uint16_t raw = 0U;

    if (s_inited == 0U) {
        return;
    }
    if ((s_last_sample_ms != 0U) && ((uint32_t)(now_ms - s_last_sample_ms) < PM_PERIOD_MS)) {
        return;
    }
    s_last_sample_ms = now_ms;

    for (uint32_t i = 0U; i < PM_SAMPLES; ++i) {
        if (pm_convert(&raw)) {
            sum += raw;
            count++;
        }
    }
    if (count == 0U) {
        s_valid = 0U;
        s_present = 0U;
        return;
    }

    uint32_t raw_avg = sum / count;
    uint32_t full = 65535U;
    uint32_t pin_mv = (uint32_t)((((uint64_t)raw_avg * PM_VREF_MV) + (full / 2U)) / full);
    uint32_t vcc_mv = pin_mv * PM_RATIO;

    s_pin_mv = pin_mv;
    s_vcc_mv = vcc_mv;
    s_valid = 1U;
    s_present = (vcc_mv >= PM_PRESENT_MV) ? 1U : 0U;
}

bool power_monitor_valid(void)
{
    return s_valid != 0U;
}

bool power_monitor_present(void)
{
    return s_present != 0U;
}

uint16_t power_monitor_vcc_mv(void)
{
    return (s_vcc_mv > 0xFFFFU) ? 0xFFFFU : (uint16_t)s_vcc_mv;
}

uint16_t power_monitor_pin_mv(void)
{
    return (s_pin_mv > 0xFFFFU) ? 0xFFFFU : (uint16_t)s_pin_mv;
}
