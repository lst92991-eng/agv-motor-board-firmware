/**
  *****************************************************************************
  * @file     motor_open_loop_map_test.c
  * @brief    电机 PWM 与空载转速映射测试
  *
  * 该测试用于建立 PWM 与实际 RPM 的关系，为速度 PID 前馈参数提供依据。
  * 串口输出格式：pwm,left_rpm,right_rpm。
  * 当前恢复使用 2640 计数/输出轴圈，以便通过 1000 PWM 下的铭牌转速验证比例。
  *****************************************************************************
  */

#include "motor_open_loop_map_test.h"

#include <stdint.h>
#include <stdio.h>

#include "at32f45x_board.h"
#include "motor_driver.h"
#include "motor_encoder.h"

#define MOTOR_MAP_SAMPLE_PERIOD_MS              100U
#define MOTOR_MAP_STEP_TIME_MS                 4000U
#define MOTOR_MAP_INITIAL_STOP_TIME_MS         2000U
#define MOTOR_MAP_FINAL_STOP_TIME_MS           3000U
#define MOTOR_MAP_COUNTS_PER_OUTPUT_REV        2640.0f
#define MOTOR_MAP_FILTER_ALPHA                    0.25f
#define MOTOR_MAP_FEEDBACK_SIGN                  -1.0f

static const int16_t motor_map_pwm_steps[] = {0, 200, 300, 400, 500, 700, 1000, 0};
static uint8_t motor_map_step_index;
static uint32_t motor_map_step_elapsed_ms;
static int32_t motor_map_last_left_count;
static int32_t motor_map_last_right_count;
static float motor_map_left_filtered_rpm;
static float motor_map_right_filtered_rpm;

static uint32_t motor_map_current_step_time_ms(void)
{
  if(motor_map_step_index == 0U)
  {
    return MOTOR_MAP_INITIAL_STOP_TIME_MS;
  }
  if(motor_map_step_index == 7U)
  {
    return MOTOR_MAP_FINAL_STOP_TIME_MS;
  }

  return MOTOR_MAP_STEP_TIME_MS;
}

static void motor_map_apply_current_step(void)
{
  int16_t pwm = motor_map_pwm_steps[motor_map_step_index];

  motor_map_step_elapsed_ms = 0;
  motor_driver_set_speeds(pwm, pwm);
}

void motor_open_loop_map_test_init(void)
{
  motor_encoder_init();
  motor_driver_init();
  motor_driver_stop_all(MOTOR_STOP_COAST);

  delay_sec(3);

  motor_encoder_reset_all();
  motor_map_last_left_count = motor_encoder_get_count(MOTOR_LEFT);
  motor_map_last_right_count = motor_encoder_get_count(MOTOR_RIGHT);
  motor_map_left_filtered_rpm = 0.0f;
  motor_map_right_filtered_rpm = 0.0f;
  motor_map_step_index = 0;
  motor_map_apply_current_step();
}

void motor_open_loop_map_test_loop(void)
{
  int32_t current_left_count;
  int32_t current_right_count;
  int32_t left_delta;
  int32_t right_delta;
  float left_raw_rpm;
  float right_raw_rpm;
  int16_t current_pwm;

  delay_ms(MOTOR_MAP_SAMPLE_PERIOD_MS);

  current_left_count = motor_encoder_get_count(MOTOR_LEFT);
  current_right_count = motor_encoder_get_count(MOTOR_RIGHT);
  left_delta = current_left_count - motor_map_last_left_count;
  right_delta = current_right_count - motor_map_last_right_count;
  motor_map_last_left_count = current_left_count;
  motor_map_last_right_count = current_right_count;

  left_raw_rpm = ((float)left_delta * 60000.0f) /
                 (MOTOR_MAP_COUNTS_PER_OUTPUT_REV * (float)MOTOR_MAP_SAMPLE_PERIOD_MS);
  right_raw_rpm = ((float)right_delta * 60000.0f) /
                  (MOTOR_MAP_COUNTS_PER_OUTPUT_REV * (float)MOTOR_MAP_SAMPLE_PERIOD_MS);
  left_raw_rpm *= MOTOR_MAP_FEEDBACK_SIGN;
  right_raw_rpm *= MOTOR_MAP_FEEDBACK_SIGN;

  motor_map_left_filtered_rpm += MOTOR_MAP_FILTER_ALPHA * (left_raw_rpm - motor_map_left_filtered_rpm);
  motor_map_right_filtered_rpm += MOTOR_MAP_FILTER_ALPHA * (right_raw_rpm - motor_map_right_filtered_rpm);

  current_pwm = motor_map_pwm_steps[motor_map_step_index];
  printf("%d,%.2f,%.2f\r\n",
         current_pwm,
         motor_map_left_filtered_rpm,
         motor_map_right_filtered_rpm);

  motor_map_step_elapsed_ms += MOTOR_MAP_SAMPLE_PERIOD_MS;
  if(motor_map_step_elapsed_ms >= motor_map_current_step_time_ms())
  {
    motor_map_step_index++;
    if(motor_map_step_index >= (sizeof(motor_map_pwm_steps) / sizeof(motor_map_pwm_steps[0])))
    {
      motor_map_step_index = 0;
    }
    motor_map_apply_current_step();
  }
}
