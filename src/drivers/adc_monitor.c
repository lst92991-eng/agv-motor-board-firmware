/**
 * @file adc_monitor.c
 * @brief ADC1母线与双路电流原始采样驱动。
 *
 * PA3为母线分压，PA4/PA5为左右INA180输出。本模块只提供20ms电气监督样本，
 * 不承担PWM同步采样，也不能被描述为电流环。
 */

#include "adc_monitor.h"

#include <string.h>

#include "at32f45x.h"
#include "at32f45x_adc.h"
#include "at32f45x_crm.h"
#include "at32f45x_gpio.h"

#define ADC_CYCLES_PER_US       192UL
#define ADC_INIT_TIMEOUT_US    1000UL
#define ADC_SAMPLE_TIMEOUT_US    50UL

static const adc_channel_select_type adc_channels[ADC_MONITOR_CHANNEL_COUNT] =
{
  ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5
};
static uint16_t adc_raw[ADC_MONITOR_CHANNEL_COUNT];
static uint8_t adc_ready;

/* DWT差值使用无符号回绕语义，避免外设异常时永久阻塞PowerMonitor任务。 */
static uint8_t adc_timed_out(uint32_t started, uint32_t timeout_us)
{
  return ((uint32_t)(DWT->CYCCNT - started) >=
          timeout_us * ADC_CYCLES_PER_US) ? 1U : 0U;
}

/** @brief 初始化ADC1、模拟GPIO和带超时的校准流程。 */
uint8_t adc_monitor_init(void)
{
  gpio_init_type gpio;
  adc_common_config_type common;
  adc_base_config_type base;
  uint32_t started;

  if(adc_ready != 0U) return 1U;
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  gpio_default_para_init(&gpio);
  gpio.gpio_mode = GPIO_MODE_ANALOG;
  /* 模拟输入不配置数字上下拉，避免改变INA180输出和母线分压结果。 */
  gpio.gpio_pins = GPIO_PINS_3 | GPIO_PINS_4 | GPIO_PINS_5;
  gpio_init(GPIOA, &gpio);

  adc_reset();
  adc_common_default_para_init(&common);
  common.combine_mode = ADC_INDEPENDENT_MODE;
  common.div = ADC_HCLK_DIV_6;
  common.common_dma_mode = ADC_COMMON_DMAMODE_DISABLE;
  common.sampling_interval = ADC_SAMPLING_INTERVAL_5CYCLES;
  adc_common_config(&common);
  adc_base_default_para_init(&base);
  base.sequence_mode = FALSE;
  base.repeat_mode = FALSE;
  base.data_align = ADC_RIGHT_ALIGNMENT;
  base.ordinary_channel_length = 1U;
  adc_base_config(ADC1, &base);
  adc_resolution_set(ADC1, ADC_RESOLUTION_12B);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_3, 1U, ADC_SAMPLETIME_92_5);
  adc_ordinary_conversion_trigger_set(ADC1, ADC_ORDINARY_TRIG_TMR1CH1,
                                      ADC_ORDINARY_TRIG_EDGE_NONE);
  adc_dma_mode_enable(ADC1, FALSE);
  adc_enable(ADC1, TRUE);

  started = DWT->CYCCNT;
  while(adc_flag_get(ADC1, ADC_RDY_FLAG) == RESET)
    if(adc_timed_out(started, ADC_INIT_TIMEOUT_US) != 0U) return 0U;
  adc_calibration_init(ADC1);
  started = DWT->CYCCNT;
  while(adc_calibration_init_status_get(ADC1))
    if(adc_timed_out(started, ADC_INIT_TIMEOUT_US) != 0U) return 0U;
  adc_calibration_start(ADC1);
  started = DWT->CYCCNT;
  while(adc_calibration_status_get(ADC1))
    if(adc_timed_out(started, ADC_INIT_TIMEOUT_US) != 0U) return 0U;
  memset(adc_raw, 0, sizeof(adc_raw));
  adc_ready = 1U;
  return 1U;
}

/** @brief 顺序采集母线、左电流和右电流三个12位通道。 */
uint8_t adc_monitor_update(void)
{
  uint8_t index;
  if(adc_ready == 0U) return 0U;
  for(index = 0U; index < ADC_MONITOR_CHANNEL_COUNT; index++)
  {
    uint32_t started;
    adc_ordinary_channel_set(ADC1, adc_channels[index], 1U,
                             ADC_SAMPLETIME_92_5);
    adc_flag_clear(ADC1, ADC_OCCE_FLAG | ADC_OCCO_FLAG | ADC_TCF_FLAG);
    adc_ordinary_software_trigger_enable(ADC1, TRUE);
    started = DWT->CYCCNT;
    while(adc_flag_get(ADC1, ADC_OCCE_FLAG) == RESET)
      if(adc_timed_out(started, ADC_SAMPLE_TIMEOUT_US) != 0U) return 0U;
    adc_raw[index] = adc_ordinary_conversion_data_get(ADC1);
  }
  /* 任一通道超时都会返回0，调用方必须丢弃本周期的整组三通道样本。 */
  return 1U;
}

/** @brief 返回指定通道最近一次原始值，非法通道返回0。 */
uint16_t adc_monitor_get_raw(adc_monitor_channel_t channel)
{
  return (channel < ADC_MONITOR_CHANNEL_COUNT) ? adc_raw[channel] : 0U;
}
