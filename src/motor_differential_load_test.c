/**
  *****************************************************************************
  * @file     motor_differential_load_test.c
  * @brief    小车带载差速速度闭环测试
  *
  * 测试阶段：
  * 1. 直行前进：左20RPM/3圈，右20RPM/3圈；
  * 2. 直行后退：左-20RPM/3圈，右-20RPM/3圈；
  * 3. 左弧线前进：左10RPM/1圈，右20RPM/2圈；
  * 4. 左弧线原路后退：左-10RPM/1圈，右-20RPM/2圈；
  * 5. 右弧线前进：左20RPM/2圈，右10RPM/1圈；
  * 6. 右弧线原路后退：左-20RPM/2圈，右-10RPM/1圈；
  * 7. 原地左转：左-15RPM/1圈，右15RPM/1圈；
  * 8. 原地右转：左15RPM/1圈，右-15RPM/1圈；
  * 9. 测试完成，保持主动刹车。
  *
  * 串口输出：
  * 阶段,左目标RPM,左实测RPM,左PWM,右目标RPM,右实测RPM,右PWM,左圈数,右圈数
  *****************************************************************************
  */

#include "motor_differential_load_test.h"

#include <stdint.h>
#include <stdio.h>

#include "at32f45x_board.h"
#include "motor_encoder.h"
#include "motor_speed_control.h"

#define MOTOR_LOAD_TEST_COUNTS_PER_REV          2640L
#define MOTOR_LOAD_TEST_DECEL_COUNTS            1320L
#define MOTOR_LOAD_TEST_STOP_TIME_SEC               2U

typedef struct
{
  float left_rpm;
  float right_rpm;
  int32_t left_revolutions;
  int32_t right_revolutions;
} motor_load_test_phase_config_t;

static const motor_load_test_phase_config_t motor_load_test_phases[] =
{
  { 20.0f,  20.0f, 3L, 3L},
  {-20.0f, -20.0f, 3L, 3L},
  { 10.0f,  20.0f, 1L, 2L},
  {-10.0f, -20.0f, 1L, 2L},
  { 20.0f,  10.0f, 2L, 1L},
  {-20.0f, -10.0f, 2L, 1L},
  {-15.0f,  15.0f, 1L, 1L},
  { 15.0f, -15.0f, 1L, 1L}
};

static uint8_t motor_load_test_phase_index;
static int32_t motor_load_test_left_start_count;
static int32_t motor_load_test_right_start_count;
static uint8_t motor_load_test_finished;

static int32_t motor_load_test_abs_count(int32_t value)
{
  return (value < 0) ? -value : value;
}

static float motor_load_test_abs_float(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float motor_load_test_sign(float value)
{
  return (value < 0.0f) ? -1.0f : 1.0f;
}

static void motor_load_test_enter_phase(uint8_t phase_index)
{
  motor_load_test_phase_index = phase_index;
  motor_load_test_left_start_count = motor_encoder_get_count(MOTOR_LEFT);
  motor_load_test_right_start_count = motor_encoder_get_count(MOTOR_RIGHT);
}

/* 接近目标圈数时把20RPM降到15RPM，10RPM和15RPM阶段保持原目标。 */
static float motor_load_test_get_target(float configured_rpm,
                                        int32_t progress_counts,
                                        int32_t target_counts)
{
  int32_t remaining_counts = target_counts - progress_counts;
  float speed_abs;

  if(remaining_counts <= 0)
  {
    return 0.0f;
  }

  speed_abs = motor_load_test_abs_float(configured_rpm);
  if((remaining_counts <= MOTOR_LOAD_TEST_DECEL_COUNTS) && (speed_abs > 15.0f))
  {
    speed_abs = 15.0f;
  }

  return motor_load_test_sign(configured_rpm) * speed_abs;
}

static void motor_load_test_finish_current_phase(void)
{
  uint8_t phase_count = (uint8_t)(sizeof(motor_load_test_phases) /
                                  sizeof(motor_load_test_phases[0]));

  motor_speed_control_hold_brake();
  delay_sec(MOTOR_LOAD_TEST_STOP_TIME_SEC);

  motor_load_test_phase_index++;
  if(motor_load_test_phase_index >= phase_count)
  {
    motor_load_test_finished = 1U;
    motor_speed_control_hold_brake();
  }
  else
  {
    motor_load_test_enter_phase(motor_load_test_phase_index);
  }
}

void motor_differential_load_test_init(void)
{
  motor_speed_control_init();
  motor_speed_control_hold_brake();
  motor_load_test_finished = 0U;

  /* 留出3秒时间，将车辆放入空旷测试区域。 */
  delay_sec(3);
  motor_load_test_enter_phase(0U);
}

void motor_differential_load_test_loop(void)
{
  const motor_load_test_phase_config_t *phase;
  int32_t left_progress;
  int32_t right_progress;
  int32_t left_target_counts;
  int32_t right_target_counts;
  float left_target_rpm;
  float right_target_rpm;
  float left_revolutions;
  float right_revolutions;

  if(motor_load_test_finished != 0U)
  {
    motor_speed_control_hold_brake();
    printf("9,0.00,0.00,0,0.00,0.00,0,0.00,0.00\r\n");
    delay_ms(100);
    return;
  }

  phase = &motor_load_test_phases[motor_load_test_phase_index];
  left_progress = motor_load_test_abs_count(motor_encoder_get_count(MOTOR_LEFT) -
                                             motor_load_test_left_start_count);
  right_progress = motor_load_test_abs_count(motor_encoder_get_count(MOTOR_RIGHT) -
                                              motor_load_test_right_start_count);
  left_target_counts = phase->left_revolutions * MOTOR_LOAD_TEST_COUNTS_PER_REV;
  right_target_counts = phase->right_revolutions * MOTOR_LOAD_TEST_COUNTS_PER_REV;

  left_target_rpm = motor_load_test_get_target(phase->left_rpm,
                                               left_progress,
                                               left_target_counts);
  right_target_rpm = motor_load_test_get_target(phase->right_rpm,
                                                right_progress,
                                                right_target_counts);

  motor_speed_control_set_targets(left_target_rpm, right_target_rpm);
  motor_speed_control_update();

  left_revolutions = (float)left_progress / (float)MOTOR_LOAD_TEST_COUNTS_PER_REV;
  right_revolutions = (float)right_progress / (float)MOTOR_LOAD_TEST_COUNTS_PER_REV;

  printf("%d,%.2f,%.2f,%d,%.2f,%.2f,%d,%.2f,%.2f\r\n",
         (int)(motor_load_test_phase_index + 1U),
         left_target_rpm,
         motor_speed_control_get_measured_rpm(MOTOR_LEFT),
         motor_speed_control_get_pwm(MOTOR_LEFT),
         right_target_rpm,
         motor_speed_control_get_measured_rpm(MOTOR_RIGHT),
         motor_speed_control_get_pwm(MOTOR_RIGHT),
         left_revolutions,
         right_revolutions);

  if((left_progress >= left_target_counts) &&
     (right_progress >= right_target_counts))
  {
    motor_load_test_finish_current_phase();
  }
}
