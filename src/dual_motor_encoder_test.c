/**
  *****************************************************************************
  * @file     dual_motor_encoder_test.c
  * @brief    左右电机开环转动与编码器对照测试
  *
  * 任意时刻只驱动一个电机，另一个电机保持主动刹车。测试前必须把左右
  * 主动轮全部架空，避免单轮驱动造成车体旋转。
  *****************************************************************************
  */

#include "dual_motor_encoder_test.h"

#include <stdint.h>
#include <stdio.h>

#include "at32f45x_board.h"
#include "motor_driver.h"
#include "motor_encoder.h"

#define DUAL_TEST_PERIOD_MS                  100U
#define DUAL_TEST_PWM                         350
#define DUAL_TEST_WAIT_TICKS                   50U
#define DUAL_TEST_RUN_TICKS                    50U
#define DUAL_TEST_SHORT_BRAKE_TICKS            20U
#define DUAL_TEST_FINAL_BRAKE_TICKS            50U
#define DUAL_TEST_ENCODER_COUNTS_PER_REV      2640L

typedef enum
{
  DUAL_TEST_PHASE_WAIT = 0,
  DUAL_TEST_PHASE_LEFT_FORWARD,
  DUAL_TEST_PHASE_LEFT_FORWARD_BRAKE,
  DUAL_TEST_PHASE_LEFT_REVERSE,
  DUAL_TEST_PHASE_LEFT_REVERSE_BRAKE,
  DUAL_TEST_PHASE_RIGHT_FORWARD,
  DUAL_TEST_PHASE_RIGHT_FORWARD_BRAKE,
  DUAL_TEST_PHASE_RIGHT_REVERSE,
  DUAL_TEST_PHASE_FINAL_BRAKE
} dual_test_phase_t;

static dual_test_phase_t dual_test_phase;
static uint16_t dual_test_phase_ticks;
static uint32_t dual_test_time_ms;
static int32_t dual_test_last_left_count;
static int32_t dual_test_last_right_count;
static uint32_t dual_test_last_left_invalid;
static uint32_t dual_test_last_right_invalid;

static const char *dual_test_phase_name(dual_test_phase_t phase)
{
  switch(phase)
  {
    case DUAL_TEST_PHASE_WAIT:
      return "WAIT";
    case DUAL_TEST_PHASE_LEFT_FORWARD:
      return "LEFT_FORWARD";
    case DUAL_TEST_PHASE_LEFT_FORWARD_BRAKE:
      return "LEFT_FORWARD_BRAKE";
    case DUAL_TEST_PHASE_LEFT_REVERSE:
      return "LEFT_REVERSE";
    case DUAL_TEST_PHASE_LEFT_REVERSE_BRAKE:
      return "LEFT_REVERSE_BRAKE";
    case DUAL_TEST_PHASE_RIGHT_FORWARD:
      return "RIGHT_FORWARD";
    case DUAL_TEST_PHASE_RIGHT_FORWARD_BRAKE:
      return "RIGHT_FORWARD_BRAKE";
    case DUAL_TEST_PHASE_RIGHT_REVERSE:
      return "RIGHT_REVERSE";
    case DUAL_TEST_PHASE_FINAL_BRAKE:
      return "FINAL_BRAKE";
    default:
      return "UNKNOWN";
  }
}

static int16_t dual_test_left_pwm(dual_test_phase_t phase)
{
  if(phase == DUAL_TEST_PHASE_LEFT_FORWARD)
  {
    return DUAL_TEST_PWM;
  }
  if(phase == DUAL_TEST_PHASE_LEFT_REVERSE)
  {
    return -DUAL_TEST_PWM;
  }
  return 0;
}

static int16_t dual_test_right_pwm(dual_test_phase_t phase)
{
  if(phase == DUAL_TEST_PHASE_RIGHT_FORWARD)
  {
    return DUAL_TEST_PWM;
  }
  if(phase == DUAL_TEST_PHASE_RIGHT_REVERSE)
  {
    return -DUAL_TEST_PWM;
  }
  return 0;
}

static void dual_test_enter_phase(dual_test_phase_t phase)
{
  dual_test_phase = phase;
  dual_test_phase_ticks = 0U;

  if(phase == DUAL_TEST_PHASE_LEFT_FORWARD)
  {
    motor_encoder_reset_all();
    dual_test_last_left_count = 0L;
    dual_test_last_right_count = 0L;
    dual_test_last_left_invalid = 0U;
    dual_test_last_right_invalid = 0U;
  }

  motor_driver_stop_all(MOTOR_STOP_BRAKE);
  if(dual_test_left_pwm(phase) != 0)
  {
    motor_driver_set_speed(MOTOR_LEFT, dual_test_left_pwm(phase));
  }
  else if(dual_test_right_pwm(phase) != 0)
  {
    motor_driver_set_speed(MOTOR_RIGHT, dual_test_right_pwm(phase));
  }

  printf("DUAL_TEST_PHASE,%lu,%s,left_pwm=%d,right_pwm=%d\r\n",
         dual_test_time_ms,
         dual_test_phase_name(phase),
         dual_test_left_pwm(phase),
         dual_test_right_pwm(phase));
}

static uint16_t dual_test_phase_duration(dual_test_phase_t phase)
{
  switch(phase)
  {
    case DUAL_TEST_PHASE_WAIT:
      return DUAL_TEST_WAIT_TICKS;
    case DUAL_TEST_PHASE_LEFT_FORWARD:
    case DUAL_TEST_PHASE_LEFT_REVERSE:
    case DUAL_TEST_PHASE_RIGHT_FORWARD:
    case DUAL_TEST_PHASE_RIGHT_REVERSE:
      return DUAL_TEST_RUN_TICKS;
    case DUAL_TEST_PHASE_LEFT_FORWARD_BRAKE:
    case DUAL_TEST_PHASE_LEFT_REVERSE_BRAKE:
    case DUAL_TEST_PHASE_RIGHT_FORWARD_BRAKE:
      return DUAL_TEST_SHORT_BRAKE_TICKS;
    case DUAL_TEST_PHASE_FINAL_BRAKE:
    default:
      return DUAL_TEST_FINAL_BRAKE_TICKS;
  }
}

static dual_test_phase_t dual_test_next_phase(dual_test_phase_t phase)
{
  switch(phase)
  {
    case DUAL_TEST_PHASE_WAIT:
      return DUAL_TEST_PHASE_LEFT_FORWARD;
    case DUAL_TEST_PHASE_LEFT_FORWARD:
      return DUAL_TEST_PHASE_LEFT_FORWARD_BRAKE;
    case DUAL_TEST_PHASE_LEFT_FORWARD_BRAKE:
      return DUAL_TEST_PHASE_LEFT_REVERSE;
    case DUAL_TEST_PHASE_LEFT_REVERSE:
      return DUAL_TEST_PHASE_LEFT_REVERSE_BRAKE;
    case DUAL_TEST_PHASE_LEFT_REVERSE_BRAKE:
      return DUAL_TEST_PHASE_RIGHT_FORWARD;
    case DUAL_TEST_PHASE_RIGHT_FORWARD:
      return DUAL_TEST_PHASE_RIGHT_FORWARD_BRAKE;
    case DUAL_TEST_PHASE_RIGHT_FORWARD_BRAKE:
      return DUAL_TEST_PHASE_RIGHT_REVERSE;
    case DUAL_TEST_PHASE_RIGHT_REVERSE:
      return DUAL_TEST_PHASE_FINAL_BRAKE;
    case DUAL_TEST_PHASE_FINAL_BRAKE:
    default:
      return DUAL_TEST_PHASE_LEFT_FORWARD;
  }
}

void dual_motor_encoder_test_init(void)
{
  motor_encoder_init();
  motor_driver_init();
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
  motor_encoder_reset_all();

  dual_test_time_ms = 0U;
  dual_test_last_left_count = 0L;
  dual_test_last_right_count = 0L;
  dual_test_last_left_invalid = 0U;
  dual_test_last_right_invalid = 0U;

  printf("\r\nDUAL_MOTOR_ENCODER_TEST,KEEP_BOTH_WHEELS_OFF_GROUND\r\n");
  printf("DUAL_TEST_HEADER,time_ms,phase,left_pwm,right_pwm,"
         "left_count,left_delta,left_rpm_x100,left_invalid,left_invalid_delta,"
         "right_count,right_delta,right_rpm_x100,right_invalid,right_invalid_delta\r\n");
  printf("DUAL_TEST_EXPECT,POSITIVE_PWM_POSITIVE_DELTA,"
         "NEGATIVE_PWM_NEGATIVE_DELTA\r\n");
  dual_test_enter_phase(DUAL_TEST_PHASE_WAIT);
}

void dual_motor_encoder_test_loop(void)
{
  int32_t left_count;
  int32_t right_count;
  int32_t left_delta;
  int32_t right_delta;
  int32_t left_rpm_x100;
  int32_t right_rpm_x100;
  uint32_t left_invalid;
  uint32_t right_invalid;
  uint32_t left_invalid_delta;
  uint32_t right_invalid_delta;

  delay_ms(DUAL_TEST_PERIOD_MS);
  dual_test_time_ms += DUAL_TEST_PERIOD_MS;

  left_count = motor_encoder_get_count(MOTOR_LEFT);
  right_count = motor_encoder_get_count(MOTOR_RIGHT);
  left_delta = left_count - dual_test_last_left_count;
  right_delta = right_count - dual_test_last_right_count;
  left_rpm_x100 = (left_delta * 60000L) /
                  DUAL_TEST_ENCODER_COUNTS_PER_REV;
  right_rpm_x100 = (right_delta * 60000L) /
                   DUAL_TEST_ENCODER_COUNTS_PER_REV;
  left_invalid = motor_encoder_get_invalid_transition_count(MOTOR_LEFT);
  right_invalid = motor_encoder_get_invalid_transition_count(MOTOR_RIGHT);
  left_invalid_delta = left_invalid - dual_test_last_left_invalid;
  right_invalid_delta = right_invalid - dual_test_last_right_invalid;

  printf("DUAL_TEST,%lu,%s,%d,%d,%ld,%ld,%ld,%lu,%lu,"
         "%ld,%ld,%ld,%lu,%lu\r\n",
         dual_test_time_ms,
         dual_test_phase_name(dual_test_phase),
         dual_test_left_pwm(dual_test_phase),
         dual_test_right_pwm(dual_test_phase),
         left_count,
         left_delta,
         left_rpm_x100,
         left_invalid,
         left_invalid_delta,
         right_count,
         right_delta,
         right_rpm_x100,
         right_invalid,
         right_invalid_delta);

  dual_test_last_left_count = left_count;
  dual_test_last_right_count = right_count;
  dual_test_last_left_invalid = left_invalid;
  dual_test_last_right_invalid = right_invalid;
  dual_test_phase_ticks++;
  if(dual_test_phase_ticks >= dual_test_phase_duration(dual_test_phase))
  {
    dual_test_enter_phase(dual_test_next_phase(dual_test_phase));
  }
}
