/**
  *****************************************************************************
  * @file     motor_speed_tuning_test.c
  * @brief    双电机速度环首轮整定测试
  *
  * 本测试复用 motor_speed_control.c，避免测试代码与正式控制使用不同的
  * PID、前馈和滤波参数。仅可在两侧主动轮均架空且现场有人看护时使用。
  *****************************************************************************
  */

#include "motor_speed_tuning_test.h"

#include <stdint.h>
#include <stdio.h>

#include "at32f45x_board.h"
#include "motor_driver.h"
#include "motor_speed_control.h"
#include "motor_speed_tuning.h"

#define MOTOR_TUNING_TEST_REPORT_PERIOD_MS      100U
#define MOTOR_TUNING_TEST_LOW_RPM                12.0f
#define MOTOR_TUNING_TEST_RUN_RPM                20.0f
#define MOTOR_TUNING_TEST_LOW_TIME_MS           5000U
#define MOTOR_TUNING_TEST_RUN_TIME_MS           5000U
#define MOTOR_TUNING_TEST_STOP_TIME_MS          2000U

typedef enum
{
  MOTOR_TUNING_STATE_FORWARD_LOW = 0,
  MOTOR_TUNING_STATE_FORWARD_RUN,
  MOTOR_TUNING_STATE_STOP_BEFORE_REVERSE,
  MOTOR_TUNING_STATE_REVERSE_LOW,
  MOTOR_TUNING_STATE_REVERSE_RUN,
  MOTOR_TUNING_STATE_STOP_BEFORE_FORWARD
} motor_tuning_state_t;

static motor_tuning_state_t motor_tuning_state;
static uint32_t motor_tuning_state_elapsed_ms;
static uint32_t motor_tuning_report_elapsed_ms;
static float motor_tuning_target_rpm;

static void motor_tuning_apply_target(float target_rpm)
{
  motor_tuning_target_rpm = target_rpm;
  motor_speed_control_set_targets(target_rpm, target_rpm);
}

static void motor_tuning_enter_state(motor_tuning_state_t state)
{
  motor_tuning_state = state;
  motor_tuning_state_elapsed_ms = 0U;

  switch(state)
  {
    case MOTOR_TUNING_STATE_FORWARD_LOW:
      motor_tuning_apply_target(MOTOR_TUNING_TEST_LOW_RPM);
      break;

    case MOTOR_TUNING_STATE_FORWARD_RUN:
      motor_tuning_apply_target(MOTOR_TUNING_TEST_RUN_RPM);
      break;

    case MOTOR_TUNING_STATE_REVERSE_LOW:
      motor_tuning_apply_target(-MOTOR_TUNING_TEST_LOW_RPM);
      break;

    case MOTOR_TUNING_STATE_REVERSE_RUN:
      motor_tuning_apply_target(-MOTOR_TUNING_TEST_RUN_RPM);
      break;

    case MOTOR_TUNING_STATE_STOP_BEFORE_REVERSE:
    case MOTOR_TUNING_STATE_STOP_BEFORE_FORWARD:
    default:
      motor_tuning_target_rpm = 0.0f;
      motor_speed_control_hold_brake();
      break;
  }
}

static void motor_tuning_advance_state(void)
{
  switch(motor_tuning_state)
  {
    case MOTOR_TUNING_STATE_FORWARD_LOW:
      if(motor_tuning_state_elapsed_ms >= MOTOR_TUNING_TEST_LOW_TIME_MS)
      {
        motor_tuning_enter_state(MOTOR_TUNING_STATE_FORWARD_RUN);
      }
      break;

    case MOTOR_TUNING_STATE_FORWARD_RUN:
      if(motor_tuning_state_elapsed_ms >= MOTOR_TUNING_TEST_RUN_TIME_MS)
      {
        motor_tuning_enter_state(MOTOR_TUNING_STATE_STOP_BEFORE_REVERSE);
      }
      break;

    case MOTOR_TUNING_STATE_STOP_BEFORE_REVERSE:
      if(motor_tuning_state_elapsed_ms >= MOTOR_TUNING_TEST_STOP_TIME_MS)
      {
        motor_tuning_enter_state(MOTOR_TUNING_STATE_REVERSE_LOW);
      }
      break;

    case MOTOR_TUNING_STATE_REVERSE_LOW:
      if(motor_tuning_state_elapsed_ms >= MOTOR_TUNING_TEST_LOW_TIME_MS)
      {
        motor_tuning_enter_state(MOTOR_TUNING_STATE_REVERSE_RUN);
      }
      break;

    case MOTOR_TUNING_STATE_REVERSE_RUN:
      if(motor_tuning_state_elapsed_ms >= MOTOR_TUNING_TEST_RUN_TIME_MS)
      {
        motor_tuning_enter_state(MOTOR_TUNING_STATE_STOP_BEFORE_FORWARD);
      }
      break;

    case MOTOR_TUNING_STATE_STOP_BEFORE_FORWARD:
    default:
      if(motor_tuning_state_elapsed_ms >= MOTOR_TUNING_TEST_STOP_TIME_MS)
      {
        motor_tuning_enter_state(MOTOR_TUNING_STATE_FORWARD_LOW);
      }
      break;
  }
}

static void motor_tuning_report(void)
{
  printf("PID_TEST,%lu,%u,%.2f,%.2f,%d,%.2f,%d\r\n",
         motor_tuning_state_elapsed_ms,
         (uint32_t)motor_tuning_state,
         motor_tuning_target_rpm,
         motor_speed_control_get_measured_rpm(MOTOR_LEFT),
         motor_speed_control_get_pwm(MOTOR_LEFT),
         motor_speed_control_get_measured_rpm(MOTOR_RIGHT),
         motor_speed_control_get_pwm(MOTOR_RIGHT));
}

void motor_speed_tuning_test_init(void)
{
  motor_speed_control_init();
  motor_speed_control_hold_brake();
  motor_tuning_report_elapsed_ms = 0U;

  printf("PID_TEST,READY,period_ms=%u,kp=%.2f,ki=%.2f,ff=%.1f+%.1f*rpm\r\n",
         (uint32_t)MOTOR_SPEED_CONTROL_PERIOD_MS,
         MOTOR_SPEED_LEFT_PID_KP,
         MOTOR_SPEED_LEFT_PID_KI,
         MOTOR_SPEED_FEED_FORWARD_STATIC_PWM,
         MOTOR_SPEED_FEED_FORWARD_PER_RPM);
  printf("PID_TEST,WARNING,LIFT_WHEELS_BEFORE_START\r\n");
  delay_sec(3U);
  motor_tuning_enter_state(MOTOR_TUNING_STATE_FORWARD_LOW);
}

void motor_speed_tuning_test_loop(void)
{
  delay_ms(MOTOR_SPEED_CONTROL_PERIOD_MS);
  motor_speed_control_update_now();

  motor_tuning_state_elapsed_ms += MOTOR_SPEED_CONTROL_PERIOD_MS;
  motor_tuning_report_elapsed_ms += MOTOR_SPEED_CONTROL_PERIOD_MS;
  if(motor_tuning_report_elapsed_ms >= MOTOR_TUNING_TEST_REPORT_PERIOD_MS)
  {
    motor_tuning_report_elapsed_ms = 0U;
    motor_tuning_report();
  }

  motor_tuning_advance_state();
}
