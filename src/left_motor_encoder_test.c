/**
  *****************************************************************************
  * @file     left_motor_encoder_test.c
  * @brief    左电机开环转动与左编码器联合测试
  *
  * 上电后先等待5秒，然后循环执行：正转10秒、刹车5秒、反转10秒、
  * 刹车5秒。较长的稳定运行区间便于使用示波器观察A/B相边沿和振铃。
  * 右电机始终主动刹车，测试前必须把左主动轮架空。
  *****************************************************************************
  */

#include "left_motor_encoder_test.h"

#include <stdint.h>
#include <stdio.h>

#include "at32f45x_board.h"
#include "motor_driver.h"
#include "motor_encoder.h"

#define LEFT_TEST_PERIOD_MS                 100U
#define LEFT_TEST_PWM                        350
#define LEFT_TEST_WAIT_TICKS                  50U
#define LEFT_TEST_RUN_TICKS                  100U
#define LEFT_TEST_SHORT_BRAKE_TICKS           50U
#define LEFT_TEST_LONG_BRAKE_TICKS            50U
#define LEFT_TEST_ENCODER_COUNTS_PER_REV     2640L

typedef enum
{
  LEFT_TEST_PHASE_WAIT = 0,
  LEFT_TEST_PHASE_FORWARD,
  LEFT_TEST_PHASE_FORWARD_BRAKE,
  LEFT_TEST_PHASE_REVERSE,
  LEFT_TEST_PHASE_REVERSE_BRAKE
} left_test_phase_t;

static left_test_phase_t left_test_phase;
static uint16_t left_test_phase_ticks;
static uint32_t left_test_time_ms;
static int32_t left_test_last_count;
static uint32_t left_test_last_invalid;

static const char *left_test_phase_name(left_test_phase_t phase)
{
  switch(phase)
  {
    case LEFT_TEST_PHASE_WAIT:
      return "WAIT";
    case LEFT_TEST_PHASE_FORWARD:
      return "FORWARD";
    case LEFT_TEST_PHASE_FORWARD_BRAKE:
      return "FORWARD_BRAKE";
    case LEFT_TEST_PHASE_REVERSE:
      return "REVERSE";
    case LEFT_TEST_PHASE_REVERSE_BRAKE:
      return "REVERSE_BRAKE";
    default:
      return "UNKNOWN";
  }
}

static int16_t left_test_phase_pwm(left_test_phase_t phase)
{
  if(phase == LEFT_TEST_PHASE_FORWARD)
  {
    return LEFT_TEST_PWM;
  }
  if(phase == LEFT_TEST_PHASE_REVERSE)
  {
    return -LEFT_TEST_PWM;
  }
  return 0;
}

static void left_test_enter_phase(left_test_phase_t phase)
{
  left_test_phase = phase;
  left_test_phase_ticks = 0U;
  motor_driver_stop(MOTOR_RIGHT, MOTOR_STOP_BRAKE);

  if(phase == LEFT_TEST_PHASE_FORWARD)
  {
    motor_encoder_reset(MOTOR_LEFT);
    left_test_last_count = 0L;
    left_test_last_invalid = 0U;
    motor_driver_set_speed(MOTOR_LEFT, LEFT_TEST_PWM);
  }
  else if(phase == LEFT_TEST_PHASE_REVERSE)
  {
    motor_driver_set_speed(MOTOR_LEFT, -LEFT_TEST_PWM);
  }
  else
  {
    motor_driver_stop(MOTOR_LEFT, MOTOR_STOP_BRAKE);
  }

  printf("LEFT_TEST_PHASE,%lu,%s,pwm=%d\r\n",
         left_test_time_ms,
         left_test_phase_name(phase),
         left_test_phase_pwm(phase));
}

static uint16_t left_test_phase_duration(left_test_phase_t phase)
{
  switch(phase)
  {
    case LEFT_TEST_PHASE_WAIT:
      return LEFT_TEST_WAIT_TICKS;
    case LEFT_TEST_PHASE_FORWARD:
    case LEFT_TEST_PHASE_REVERSE:
      return LEFT_TEST_RUN_TICKS;
    case LEFT_TEST_PHASE_FORWARD_BRAKE:
      return LEFT_TEST_SHORT_BRAKE_TICKS;
    case LEFT_TEST_PHASE_REVERSE_BRAKE:
      return LEFT_TEST_LONG_BRAKE_TICKS;
    default:
      return LEFT_TEST_LONG_BRAKE_TICKS;
  }
}

static left_test_phase_t left_test_next_phase(left_test_phase_t phase)
{
  switch(phase)
  {
    case LEFT_TEST_PHASE_WAIT:
      return LEFT_TEST_PHASE_FORWARD;
    case LEFT_TEST_PHASE_FORWARD:
      return LEFT_TEST_PHASE_FORWARD_BRAKE;
    case LEFT_TEST_PHASE_FORWARD_BRAKE:
      return LEFT_TEST_PHASE_REVERSE;
    case LEFT_TEST_PHASE_REVERSE:
      return LEFT_TEST_PHASE_REVERSE_BRAKE;
    case LEFT_TEST_PHASE_REVERSE_BRAKE:
    default:
      return LEFT_TEST_PHASE_FORWARD;
  }
}

void left_motor_encoder_test_init(void)
{
  motor_encoder_init();
  motor_driver_init();
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
  motor_encoder_reset_all();

  left_test_time_ms = 0U;
  left_test_last_count = 0L;
  left_test_last_invalid = 0U;

  printf("\r\nLEFT_MOTOR_ENCODER_TEST,KEEP_LEFT_WHEEL_OFF_GROUND\r\n");
  printf("LEFT_TEST_HEADER,time_ms,phase,pwm,count,delta,rpm_x100,"
         "invalid_total,invalid_delta\r\n");
  printf("LEFT_TEST_EXPECT,FORWARD_DELTA_POSITIVE,REVERSE_DELTA_NEGATIVE\r\n");
  left_test_enter_phase(LEFT_TEST_PHASE_WAIT);
}

void left_motor_encoder_test_loop(void)
{
  int32_t count;
  int32_t delta;
  int32_t rpm_x100;
  uint32_t invalid;
  uint32_t invalid_delta;

  delay_ms(LEFT_TEST_PERIOD_MS);
  left_test_time_ms += LEFT_TEST_PERIOD_MS;

  count = motor_encoder_get_count(MOTOR_LEFT);
  delta = count - left_test_last_count;
  rpm_x100 = (delta * 60000L) / LEFT_TEST_ENCODER_COUNTS_PER_REV;
  invalid = motor_encoder_get_invalid_transition_count(MOTOR_LEFT);
  invalid_delta = invalid - left_test_last_invalid;

  printf("LEFT_TEST,%lu,%s,%d,%ld,%ld,%ld,%lu,%lu\r\n",
         left_test_time_ms,
         left_test_phase_name(left_test_phase),
         left_test_phase_pwm(left_test_phase),
         count,
         delta,
         rpm_x100,
         invalid,
         invalid_delta);

  left_test_last_count = count;
  left_test_last_invalid = invalid;
  left_test_phase_ticks++;
  if(left_test_phase_ticks >= left_test_phase_duration(left_test_phase))
  {
    left_test_enter_phase(left_test_next_phase(left_test_phase));
  }
}
