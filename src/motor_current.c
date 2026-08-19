/**
  *****************************************************************************
  * @file     motor_current.c
  * @brief    双路电机电流 ADC 采集与单位换算
  *
 * 网表确认：左路 INA180(U15) 经 R41 输出连接 PA4/ADC1_IN4，
 * 右路 INA180(U16) 经 R40 输出连接 PA5/ADC1_IN5。
  * 硬件使用 10mΩ 分流电阻和 INA180A3（100V/V），因此输出灵敏度为 1V/A。
  * 在 3.3V、12 位 ADC 条件下，1 个 ADC 码约等于 0.806mA。
  *****************************************************************************
  */

#include "motor_current.h"

#include "adc_monitor.h"

#define MOTOR_CURRENT_ADC_FULL_SCALE     4095UL
#define MOTOR_CURRENT_ADC_REFERENCE_MV   3300UL

static uint16_t motor_current_raw[2];
static uint16_t motor_current_zero_raw[2];

void motor_current_init(void)
{
  adc_monitor_init();

  motor_current_raw[MOTOR_LEFT] = 0;
  motor_current_raw[MOTOR_RIGHT] = 0;
  motor_current_zero_raw[MOTOR_LEFT] = 0;
  motor_current_zero_raw[MOTOR_RIGHT] = 0;
}

uint8_t motor_current_update(void)
{
  if(adc_monitor_update() == 0U)
  {
    return 0U;
  }

  motor_current_raw[MOTOR_LEFT] = adc_monitor_get_raw(ADC_MONITOR_PA4);
  motor_current_raw[MOTOR_RIGHT] = adc_monitor_get_raw(ADC_MONITOR_PA5);
  return 1U;
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
