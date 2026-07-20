/**
  *****************************************************************************
  * @file     chassis_motion_test.c
  * @brief    两轮差速底盘定圈运动测试
  *
  * 动作阶段：
  * 1 = 前进 10 圈，2 = 后退 10 圈；
  * 3 = 原地左转 1 圈，4 = 原地右转 1 圈；
  * 5 = 测试完成并保持刹车。
  *
  * 85mm 轮胎每圈约 267.04mm，因此前进或后退 10 圈约为 2.67m。
  * 左右动作是差速原地转向，不是车身方向不变的横向平移。
  *****************************************************************************
  */

#include "chassis_motion_test.h"

#include <stdint.h>
#include <stdio.h>

#include "at32f45x_board.h"
#include "motor_encoder.h"
#include "motor_speed_control.h"

#define CHASSIS_ENCODER_COUNTS_PER_REV          2640L
#define CHASSIS_STRAIGHT_TARGET_REVS              10L
#define CHASSIS_TURN_TARGET_REVS                   1L
#define CHASSIS_NORMAL_RPM                        20.0f
#define CHASSIS_APPROACH_RPM                      15.0f
#define CHASSIS_SLOW_DOWN_COUNTS          CHASSIS_ENCODER_COUNTS_PER_REV
#define CHASSIS_PHASE_STOP_TIME_SEC                2U

typedef enum
{
  CHASSIS_PHASE_FORWARD = 1,
  CHASSIS_PHASE_BACKWARD,
  CHASSIS_PHASE_TURN_LEFT,
  CHASSIS_PHASE_TURN_RIGHT,
  CHASSIS_PHASE_DONE
} chassis_motion_phase_t;

static chassis_motion_phase_t chassis_phase;
static int32_t chassis_left_start_count;
static int32_t chassis_right_start_count;
static int32_t chassis_target_counts;

static int32_t chassis_abs_count(int32_t value)
{
  return (value < 0) ? -value : value;
}

static void chassis_motion_enter_phase(chassis_motion_phase_t phase)
{
  chassis_phase = phase;
  chassis_left_start_count = motor_encoder_get_count(MOTOR_LEFT);
  chassis_right_start_count = motor_encoder_get_count(MOTOR_RIGHT);

  if((phase == CHASSIS_PHASE_FORWARD) || (phase == CHASSIS_PHASE_BACKWARD))
  {
    chassis_target_counts = CHASSIS_STRAIGHT_TARGET_REVS * CHASSIS_ENCODER_COUNTS_PER_REV;
  }
  else if((phase == CHASSIS_PHASE_TURN_LEFT) || (phase == CHASSIS_PHASE_TURN_RIGHT))
  {
    chassis_target_counts = CHASSIS_TURN_TARGET_REVS * CHASSIS_ENCODER_COUNTS_PER_REV;
  }
  else
  {
    chassis_target_counts = 0;
    motor_speed_control_hold_brake();
  }
}

static void chassis_motion_get_phase_directions(float *left_direction, float *right_direction)
{
  switch(chassis_phase)
  {
    case CHASSIS_PHASE_FORWARD:
      *left_direction = 1.0f;
      *right_direction = 1.0f;
      break;

    case CHASSIS_PHASE_BACKWARD:
      *left_direction = -1.0f;
      *right_direction = -1.0f;
      break;

    case CHASSIS_PHASE_TURN_LEFT:
      *left_direction = -1.0f;
      *right_direction = 1.0f;
      break;

    case CHASSIS_PHASE_TURN_RIGHT:
      *left_direction = 1.0f;
      *right_direction = -1.0f;
      break;

    case CHASSIS_PHASE_DONE:
    default:
      *left_direction = 0.0f;
      *right_direction = 0.0f;
      break;
  }
}

static float chassis_motion_target_for_wheel(int32_t progress_counts, float direction)
{
  int32_t remaining_counts = chassis_target_counts - progress_counts;

  if((direction == 0.0f) || (remaining_counts <= 0))
  {
    return 0.0f;
  }
  if(remaining_counts <= CHASSIS_SLOW_DOWN_COUNTS)
  {
    return direction * CHASSIS_APPROACH_RPM;
  }

  return direction * CHASSIS_NORMAL_RPM;
}

static void chassis_motion_finish_phase(void)
{
  motor_speed_control_hold_brake();
  delay_sec(CHASSIS_PHASE_STOP_TIME_SEC);

  switch(chassis_phase)
  {
    case CHASSIS_PHASE_FORWARD:
      chassis_motion_enter_phase(CHASSIS_PHASE_BACKWARD);
      break;
    case CHASSIS_PHASE_BACKWARD:
      chassis_motion_enter_phase(CHASSIS_PHASE_TURN_LEFT);
      break;
    case CHASSIS_PHASE_TURN_LEFT:
      chassis_motion_enter_phase(CHASSIS_PHASE_TURN_RIGHT);
      break;
    case CHASSIS_PHASE_TURN_RIGHT:
    default:
      chassis_motion_enter_phase(CHASSIS_PHASE_DONE);
      break;
  }
}

void chassis_motion_test_init(void)
{
  motor_speed_control_init();
  motor_speed_control_hold_brake();

  /* 上电后等待 3 秒，留出放置车辆和清理测试区域的时间。 */
  delay_sec(3);
  chassis_motion_enter_phase(CHASSIS_PHASE_FORWARD);
}

void chassis_motion_test_loop(void)
{
  int32_t left_delta;
  int32_t right_delta;
  int32_t left_progress;
  int32_t right_progress;
  float left_direction;
  float right_direction;
  float left_target_rpm;
  float right_target_rpm;
  float left_turns;
  float right_turns;

  if(chassis_phase == CHASSIS_PHASE_DONE)
  {
    motor_speed_control_hold_brake();
    printf("5,10.00,10.00,0.00,0.00\r\n");
    delay_ms(100);
    return;
  }

  left_delta = motor_encoder_get_count(MOTOR_LEFT) - chassis_left_start_count;
  right_delta = motor_encoder_get_count(MOTOR_RIGHT) - chassis_right_start_count;
  left_progress = chassis_abs_count(left_delta);
  right_progress = chassis_abs_count(right_delta);
  chassis_motion_get_phase_directions(&left_direction, &right_direction);

  left_target_rpm = chassis_motion_target_for_wheel(left_progress, left_direction);
  right_target_rpm = chassis_motion_target_for_wheel(right_progress, right_direction);
  motor_speed_control_set_targets(left_target_rpm, right_target_rpm);
  motor_speed_control_update();

  left_turns = (float)left_progress / (float)CHASSIS_ENCODER_COUNTS_PER_REV;
  right_turns = (float)right_progress / (float)CHASSIS_ENCODER_COUNTS_PER_REV;
  printf("%d,%.2f,%.2f,%.2f,%.2f\r\n",
         (int)chassis_phase,
         left_turns,
         right_turns,
         motor_speed_control_get_measured_rpm(MOTOR_LEFT),
         motor_speed_control_get_measured_rpm(MOTOR_RIGHT));

  if((left_progress >= chassis_target_counts) &&
     (right_progress >= chassis_target_counts))
  {
    chassis_motion_finish_phase();
  }
}
