/**
  *****************************************************************************
  * @file     motor_current.c
  * @brief    双路电机电流 ADC 原始采集
  *
 * 网表确认：左路 INA180(U15) 经 R41 输出连接 PA4/ADC1_IN4，
 * 右路 INA180(U16) 经 R40 输出连接 PA5/ADC1_IN5。
  * 零点、单位换算和保护唯一由power_supervisor负责，避免重复标定源。
  *****************************************************************************
  */

#include "motor_current.h"

#include "adc_monitor.h"

static uint16_t motor_current_raw[2];

uint8_t motor_current_init(void)
{
  if(adc_monitor_init() == 0U)
  {
    return 0U;
  }

  motor_current_raw[MOTOR_LEFT] = 0;
  motor_current_raw[MOTOR_RIGHT] = 0;
  return 1U;
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

uint16_t motor_current_get_raw(motor_id_t motor)
{
  if((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT))
  {
    return 0;
  }

  return motor_current_raw[motor];
}
