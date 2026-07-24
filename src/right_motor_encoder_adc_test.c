/**
  *****************************************************************************
  * @file     right_motor_encoder_adc_test.c
  * @brief    右电机开环转动、编码器和ADC联合测试
  *
  * 右电机以固定占空比持续正转，左电机保持刹车。程序周期输出右编码器
  * 的累计计数、采样周期内增量、非法跳变次数，以及PA3、PA4、PA5的ADC
  * 原始值和引脚电压，便于同时观察电机转动和电流采样变化。
  *****************************************************************************
  */

#include "right_motor_encoder_adc_test.h"

#include <stdint.h>
#include <stdio.h>

#include "at32f45x_board.h"
#include "adc_monitor.h"
#include "motor_driver.h"
#include "motor_encoder.h"

#define RIGHT_MOTOR_ADC_TEST_PERIOD_MS       100U
#define RIGHT_MOTOR_ADC_TEST_PWM             350

static uint32_t right_motor_adc_test_time_ms;
static int32_t right_motor_adc_test_last_count;
static uint32_t right_motor_adc_test_last_invalid;

void right_motor_encoder_adc_test_init(void)
{
  motor_encoder_init();
  motor_driver_init();
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
  motor_encoder_reset_all();

  adc_monitor_init();
  (void)adc_monitor_update();

  right_motor_adc_test_time_ms = 0UL;
  right_motor_adc_test_last_count = 0L;
  right_motor_adc_test_last_invalid = 0UL;

  printf("\r\nRIGHT_MOTOR_ENCODER_ADC_TEST,KEEP_RIGHT_WHEEL_OFF_GROUND\r\n");
  printf("RIGHT_TEST_HEADER,time_ms,pwm,enc_count,enc_delta,invalid_total,"
         "invalid_delta,pa3_raw,pa3_mv,pa4_raw,pa4_mv,pa5_raw,pa5_mv\r\n");
  printf("RIGHT_TEST_EXPECT,RIGHT_MOTOR_FORWARD,LEFT_MOTOR_STOPPED\r\n");

  /* 右电机固定正向开环运行，便于观察编码器和电流ADC变化。 */
  motor_driver_set_speed(MOTOR_RIGHT, RIGHT_MOTOR_ADC_TEST_PWM);
}

void right_motor_encoder_adc_test_loop(void)
{
  int32_t count;
  int32_t delta;
  uint32_t invalid;
  uint32_t invalid_delta;

  delay_ms(RIGHT_MOTOR_ADC_TEST_PERIOD_MS);
  right_motor_adc_test_time_ms += RIGHT_MOTOR_ADC_TEST_PERIOD_MS;

  count = motor_encoder_get_count(MOTOR_RIGHT);
  delta = count - right_motor_adc_test_last_count;
  invalid = motor_encoder_get_invalid_transition_count(MOTOR_RIGHT);
  invalid_delta = invalid - right_motor_adc_test_last_invalid;
  (void)adc_monitor_update();

  printf("RIGHT_TEST,%lu,%d,%ld,%ld,%lu,%lu,%u,%lu,%u,%lu,%u,%lu\r\n",
         right_motor_adc_test_time_ms,
         RIGHT_MOTOR_ADC_TEST_PWM,
         count,
         delta,
         invalid,
         invalid_delta,
         adc_monitor_get_raw(ADC_MONITOR_PA3),
         adc_monitor_get_mv(ADC_MONITOR_PA3),
         adc_monitor_get_raw(ADC_MONITOR_PA4),
         adc_monitor_get_mv(ADC_MONITOR_PA4),
         adc_monitor_get_raw(ADC_MONITOR_PA5),
         adc_monitor_get_mv(ADC_MONITOR_PA5));

  right_motor_adc_test_last_count = count;
  right_motor_adc_test_last_invalid = invalid;
}
