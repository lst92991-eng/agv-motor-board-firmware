/**
  *****************************************************************************
  * @file     adc_voltage_test.c
  * @brief    前进、后退过程中的电池电压和双路电机电流测试
  *
  * 输出格式：电池电压V,左电机电流A,右电机电流A。
  * PA3是24V母线经过11:1分压后的电压，PA4和PA5是左右INA180输出。
  * 10mΩ采样电阻配合INA180A3后灵敏度为1V/A。
  * 测试只执行前进和后退，不执行左右转向。
  *****************************************************************************
  */

#include "adc_voltage_test.h"

#include <stdint.h>
#include <stdio.h>

#include "adc_monitor.h"
#include "at32f45x_board.h"
#include "motor_current.h"
#include "motor_driver.h"

#define ADC_VOLTAGE_TEST_PERIOD_MS       100U
#define ADC_VOLTAGE_TEST_START_DELAY_MS 3000U
#define ADC_VOLTAGE_TEST_RUN_TIME_MS     2000U
#define ADC_VOLTAGE_TEST_STOP_TIME_MS    2000U
#define ADC_VOLTAGE_TEST_PWM              500
#define ADC_VOLTAGE_TEST_REFERENCE_MV    3300UL
#define ADC_VOLTAGE_TEST_FULL_SCALE      4095UL
#define ADC_VOLTAGE_TEST_DIVIDER_RATIO     11UL
#define ADC_VOLTAGE_TEST_ZERO_SAMPLES     128U

typedef enum
{
  ADC_VOLTAGE_PHASE_WAIT = 0,
  ADC_VOLTAGE_PHASE_FORWARD,
  ADC_VOLTAGE_PHASE_STOP,
  ADC_VOLTAGE_PHASE_BACKWARD,
  ADC_VOLTAGE_PHASE_DONE
} adc_voltage_phase_t;

static adc_voltage_phase_t adc_voltage_phase;
static uint32_t adc_voltage_phase_elapsed_ms;

static void adc_voltage_test_update_motion(void)
{
  adc_voltage_phase_elapsed_ms += ADC_VOLTAGE_TEST_PERIOD_MS;

  switch(adc_voltage_phase)
  {
    case ADC_VOLTAGE_PHASE_WAIT:
      if(adc_voltage_phase_elapsed_ms >= ADC_VOLTAGE_TEST_START_DELAY_MS)
      {
        motor_driver_set_speeds(ADC_VOLTAGE_TEST_PWM, ADC_VOLTAGE_TEST_PWM);
        adc_voltage_phase = ADC_VOLTAGE_PHASE_FORWARD;
        adc_voltage_phase_elapsed_ms = 0UL;
      }
      break;

    case ADC_VOLTAGE_PHASE_FORWARD:
      if(adc_voltage_phase_elapsed_ms >= ADC_VOLTAGE_TEST_RUN_TIME_MS)
      {
        motor_driver_stop_all(MOTOR_STOP_BRAKE);
        adc_voltage_phase = ADC_VOLTAGE_PHASE_STOP;
        adc_voltage_phase_elapsed_ms = 0UL;
      }
      break;

    case ADC_VOLTAGE_PHASE_STOP:
      if(adc_voltage_phase_elapsed_ms >= ADC_VOLTAGE_TEST_STOP_TIME_MS)
      {
        motor_driver_set_speeds(-ADC_VOLTAGE_TEST_PWM, -ADC_VOLTAGE_TEST_PWM);
        adc_voltage_phase = ADC_VOLTAGE_PHASE_BACKWARD;
        adc_voltage_phase_elapsed_ms = 0UL;
      }
      break;

    case ADC_VOLTAGE_PHASE_BACKWARD:
      if(adc_voltage_phase_elapsed_ms >= ADC_VOLTAGE_TEST_RUN_TIME_MS)
      {
        motor_driver_stop_all(MOTOR_STOP_BRAKE);
        adc_voltage_phase = ADC_VOLTAGE_PHASE_DONE;
        adc_voltage_phase_elapsed_ms = 0UL;
      }
      break;

    case ADC_VOLTAGE_PHASE_DONE:
    default:
      motor_driver_stop_all(MOTOR_STOP_BRAKE);
      break;
  }
}

void adc_voltage_test_init(void)
{
  motor_current_init();
  motor_driver_init();
  motor_driver_stop_all(MOTOR_STOP_BRAKE);

  /* 电机停止时校准PA4和PA5零电流偏置。 */
  delay_ms(50);
  if(motor_current_calibrate_zero(ADC_VOLTAGE_TEST_ZERO_SAMPLES) == 0U)
  {
    printf("ADC zero calibration failed\r\n");
  }

  adc_voltage_phase = ADC_VOLTAGE_PHASE_WAIT;
  adc_voltage_phase_elapsed_ms = 0UL;
}

void adc_voltage_test_loop(void)
{
  uint32_t battery_mv;
  float battery_voltage;
  float left_current;
  float right_current;

  if(motor_current_update() != 0U)
  {
    battery_mv = ((uint32_t)adc_monitor_get_raw(ADC_MONITOR_PA3) *
                  ADC_VOLTAGE_TEST_REFERENCE_MV *
                  ADC_VOLTAGE_TEST_DIVIDER_RATIO) /
                 ADC_VOLTAGE_TEST_FULL_SCALE;
    battery_voltage = (float)battery_mv / 1000.0f;
    left_current = (float)motor_current_get_ma(MOTOR_LEFT) / 1000.0f;
    right_current = (float)motor_current_get_ma(MOTOR_RIGHT) / 1000.0f;

    printf("%.2f,%.3f,%.3f\r\n",
           battery_voltage,
           left_current,
           right_current);
  }
  else
  {
    printf("ADC read failed\r\n");
  }

  delay_ms(ADC_VOLTAGE_TEST_PERIOD_MS);
  adc_voltage_test_update_motion();
}
