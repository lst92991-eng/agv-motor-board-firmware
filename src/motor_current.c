/**
  *****************************************************************************
  * @file     motor_current.c
  * @brief    双路电机电流 ADC 采集与单位换算
  *
  * 左路 INA180 输出连接 PA4/ADC1_IN4，右路输出连接 PA5/ADC1_IN5。
  * 硬件使用 10mΩ 分流电阻和 INA180A3（100V/V），因此输出灵敏度为 1V/A。
  * 在 3.3V、12 位 ADC 条件下，1 个 ADC 码约等于 0.806mA。
  *****************************************************************************
  */

#include "motor_current.h"

#include "at32f45x.h"
#include "at32f45x_adc.h"
#include "at32f45x_crm.h"
#include "at32f45x_gpio.h"

#define MOTOR_CURRENT_ADC_FULL_SCALE     4095UL
#define MOTOR_CURRENT_ADC_REFERENCE_MV   3300UL
#define MOTOR_CURRENT_ADC_TIMEOUT        100000UL

static uint16_t motor_current_raw[2];
static uint16_t motor_current_zero_raw[2];

void motor_current_init(void)
{
  gpio_init_type gpio_init_struct;
  adc_common_config_type adc_common_struct;
  adc_base_config_type adc_base_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = GPIO_PINS_4 | GPIO_PINS_5;
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
  adc_base_struct.sequence_mode = TRUE;
  adc_base_struct.repeat_mode = FALSE;
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_struct.ordinary_channel_length = 2;
  adc_base_config(ADC1, &adc_base_struct);
  adc_resolution_set(ADC1, ADC_RESOLUTION_12B);

  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_4, 1, ADC_SAMPLETIME_92_5);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_5, 2, ADC_SAMPLETIME_92_5);
  adc_ordinary_conversion_trigger_set(ADC1, ADC_ORDINARY_TRIG_TMR1CH1, ADC_ORDINARY_TRIG_EDGE_NONE);
  adc_dma_mode_enable(ADC1, FALSE);
  adc_dma_request_repeat_enable(ADC1, FALSE);
  adc_occe_each_conversion_enable(ADC1, TRUE);

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

  motor_current_raw[MOTOR_LEFT] = 0;
  motor_current_raw[MOTOR_RIGHT] = 0;
  motor_current_zero_raw[MOTOR_LEFT] = 0;
  motor_current_zero_raw[MOTOR_RIGHT] = 0;
}

uint8_t motor_current_update(void)
{
  uint32_t timeout;
  uint8_t channel_index;

  adc_flag_clear(ADC1, ADC_OCCE_FLAG | ADC_OCCO_FLAG | ADC_TCF_FLAG);
  adc_ordinary_software_trigger_enable(ADC1, TRUE);

  for(channel_index = 0; channel_index < 2U; channel_index++)
  {
    timeout = MOTOR_CURRENT_ADC_TIMEOUT;
    while((adc_flag_get(ADC1, ADC_OCCE_FLAG) == RESET) && (timeout > 0UL))
    {
      timeout--;
    }

    if(timeout == 0UL)
    {
      return 0;
    }

    adc_flag_clear(ADC1, ADC_OCCE_FLAG);
    motor_current_raw[channel_index] = adc_ordinary_conversion_data_get(ADC1);
  }

  return 1;
}

uint8_t motor_current_calibrate_zero(uint16_t sample_count)
{
  uint16_t sample_index;
  uint32_t left_sum = 0;
  uint32_t right_sum = 0;

  if(sample_count == 0U)
  {
    return 0;
  }

  for(sample_index = 0; sample_index < sample_count; sample_index++)
  {
    if(motor_current_update() == 0U)
    {
      return 0;
    }

    left_sum += motor_current_raw[MOTOR_LEFT];
    right_sum += motor_current_raw[MOTOR_RIGHT];
  }

  motor_current_zero_raw[MOTOR_LEFT] = (uint16_t)(left_sum / sample_count);
  motor_current_zero_raw[MOTOR_RIGHT] = (uint16_t)(right_sum / sample_count);
  return 1;
}

uint16_t motor_current_get_raw(motor_id_t motor)
{
  if((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT))
  {
    return 0;
  }

  return motor_current_raw[motor];
}

uint32_t motor_current_get_ma(motor_id_t motor)
{
  uint32_t corrected_raw;

  if((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT))
  {
    return 0;
  }

  if(motor_current_raw[motor] <= motor_current_zero_raw[motor])
  {
    return 0;
  }

  corrected_raw = motor_current_raw[motor] - motor_current_zero_raw[motor];

  /* 1V/A 等价于 1mV/mA，所以 ADC 电压的 mV 数值就是电流的 mA 数值。 */
  return (corrected_raw * MOTOR_CURRENT_ADC_REFERENCE_MV) / MOTOR_CURRENT_ADC_FULL_SCALE;
}
