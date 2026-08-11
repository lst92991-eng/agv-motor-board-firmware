/**
  *****************************************************************************
  * @file     adc_monitor.c
  * @brief    PA3、PA4和PA5统一ADC采集
  *
 * PA3用于24V母线分压检测，PA4连接左路、PA5连接右路 INA180 输出。
  * 三个通道采用单通道逐次软件触发，避免连续序列转换时丢失完成标志。
  *****************************************************************************
  */

#include "adc_monitor.h"

#include "at32f45x.h"
#include "at32f45x_adc.h"
#include "at32f45x_crm.h"
#include "at32f45x_gpio.h"

#define ADC_MONITOR_FULL_SCALE          4095UL
#define ADC_MONITOR_REFERENCE_MV        3300UL
#define ADC_MONITOR_TIMEOUT           100000UL

static const adc_channel_select_type adc_monitor_channels[ADC_MONITOR_CHANNEL_COUNT] =
{
  ADC_CHANNEL_3,
  ADC_CHANNEL_4,
  ADC_CHANNEL_5
};

static uint16_t adc_monitor_raw[ADC_MONITOR_CHANNEL_COUNT];
static uint8_t adc_monitor_initialized;

void adc_monitor_init(void)
{
  gpio_init_type gpio_init_struct;
  adc_common_config_type adc_common_struct;
  adc_base_config_type adc_base_struct;
  uint8_t channel_index;

  if(adc_monitor_initialized != 0U)
  {
    return;
  }

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = GPIO_PINS_3 | GPIO_PINS_4 | GPIO_PINS_5;
  gpio_init(GPIOA, &gpio_init_struct);

  adc_reset();

  adc_common_default_para_init(&adc_common_struct);
  adc_common_struct.combine_mode = ADC_INDEPENDENT_MODE;
  adc_common_struct.div = ADC_HCLK_DIV_6;
  adc_common_struct.common_dma_mode = ADC_COMMON_DMAMODE_DISABLE;
  adc_common_struct.common_dma_request_repeat_state = FALSE;
  adc_common_struct.sampling_interval = ADC_SAMPLING_INTERVAL_5CYCLES;
  adc_common_struct.tempervintrv_state = FALSE;
  adc_common_struct.vbat_state = FALSE;
  adc_common_config(&adc_common_struct);

  adc_base_default_para_init(&adc_base_struct);
  adc_base_struct.sequence_mode = FALSE;
  adc_base_struct.repeat_mode = FALSE;
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_struct.ordinary_channel_length = 1;
  adc_base_config(ADC1, &adc_base_struct);
  adc_resolution_set(ADC1, ADC_RESOLUTION_12B);

  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_3, 1, ADC_SAMPLETIME_92_5);
  adc_ordinary_conversion_trigger_set(ADC1,
                                      ADC_ORDINARY_TRIG_TMR1CH1,
                                      ADC_ORDINARY_TRIG_EDGE_NONE);
  adc_dma_mode_enable(ADC1, FALSE);
  adc_dma_request_repeat_enable(ADC1, FALSE);
  adc_occe_each_conversion_enable(ADC1, FALSE);

  adc_enable(ADC1, TRUE);
  while(adc_flag_get(ADC1, ADC_RDY_FLAG) == RESET)
  {
  }

  adc_calibration_init(ADC1);
  while(adc_calibration_init_status_get(ADC1))
  {
  }
  adc_calibration_start(ADC1);
  while(adc_calibration_status_get(ADC1))
  {
  }

  for(channel_index = 0U; channel_index < ADC_MONITOR_CHANNEL_COUNT; channel_index++)
  {
    adc_monitor_raw[channel_index] = 0U;
  }
  adc_monitor_initialized = 1U;
}

uint8_t adc_monitor_update(void)
{
  uint32_t timeout;
  uint8_t channel_index;

  if(adc_monitor_initialized == 0U)
  {
    return 0U;
  }

  for(channel_index = 0U; channel_index < ADC_MONITOR_CHANNEL_COUNT; channel_index++)
  {
    adc_ordinary_channel_set(ADC1,
                             adc_monitor_channels[channel_index],
                             1,
                             ADC_SAMPLETIME_92_5);
    adc_flag_clear(ADC1, ADC_OCCE_FLAG | ADC_OCCO_FLAG | ADC_TCF_FLAG);
    adc_ordinary_software_trigger_enable(ADC1, TRUE);

    timeout = ADC_MONITOR_TIMEOUT;
    while((adc_flag_get(ADC1, ADC_OCCE_FLAG) == RESET) && (timeout > 0UL))
    {
      timeout--;
    }

    if(timeout == 0UL)
    {
      return 0U;
    }

    adc_monitor_raw[channel_index] = adc_ordinary_conversion_data_get(ADC1);
  }

  return 1U;
}

uint16_t adc_monitor_get_raw(adc_monitor_channel_t channel)
{
  if(channel >= ADC_MONITOR_CHANNEL_COUNT)
  {
    return 0U;
  }

  return adc_monitor_raw[channel];
}

uint32_t adc_monitor_get_mv(adc_monitor_channel_t channel)
{
  return ((uint32_t)adc_monitor_get_raw(channel) * ADC_MONITOR_REFERENCE_MV) /
         ADC_MONITOR_FULL_SCALE;
}
