/**
  *****************************************************************************
  * @file     chassis_straight_test.c
  * @brief    任意万向轮初始角度下的前进直线测试
  *
  * 测试只执行前进，不执行后退和原地转向。程序先记录原始航向，
  * 再低速行驶0.1m让万向轮对齐，随后用同向差速恢复原始航向，
  * 最后加速前进并在总行程约1m时减速停止。
  *****************************************************************************
  */

#include "chassis_straight_test.h"

#include <stdint.h>
#include <stdio.h>

#include "at32f45x_board.h"
#include "bmi2_defs.h"
#include "imu_heading.h"
#include "motor_encoder.h"
#include "motor_speed_control.h"

#define CHASSIS_STRAIGHT_LOOP_PERIOD_MS                  10U
#define CHASSIS_STRAIGHT_SPEED_PERIOD_TICKS               2U
#define CHASSIS_STRAIGHT_PRINT_PERIOD_TICKS              10U
#define CHASSIS_STRAIGHT_START_WAIT_TICKS                300U
#define CHASSIS_STRAIGHT_RECOVER_STABLE_TICKS             30U
#define CHASSIS_STRAIGHT_RECOVER_TIMEOUT_TICKS           500U
#define CHASSIS_STRAIGHT_NO_PROGRESS_TIMEOUT_TICKS       300U

#define CHASSIS_STRAIGHT_ENCODER_COUNTS_PER_REV          2640L
#define CHASSIS_STRAIGHT_ALIGN_COUNTS                    1000L
#define CHASSIS_STRAIGHT_TOTAL_COUNTS                    9886L
#define CHASSIS_STRAIGHT_DECEL_COUNTS                    1482L
#define CHASSIS_STRAIGHT_ENCODER_MISMATCH_COUNTS         1200L
#define CHASSIS_STRAIGHT_ENCODER_CHECK_MIN_COUNTS        1500L
#define CHASSIS_STRAIGHT_WHEEL_CIRCUMFERENCE_M            0.267035f

#define CHASSIS_STRAIGHT_ALIGN_RPM                        12.0f
#define CHASSIS_STRAIGHT_RUN_RPM                          20.0f
#define CHASSIS_STRAIGHT_DECEL_END_RPM                    10.0f
#define CHASSIS_STRAIGHT_RAMP_STEP_RPM                     0.2f

#define CHASSIS_HEADING_KP_RPM_PER_DEG                     0.70f
#define CHASSIS_HEADING_KD_RPM_PER_DPS                     0.08f
#define CHASSIS_HEADING_ALIGN_LIMIT_RPM                     2.0f
#define CHASSIS_HEADING_RECOVER_LIMIT_RPM                   5.0f
#define CHASSIS_HEADING_TRACK_LIMIT_RPM                     4.0f
#define CHASSIS_HEADING_RECOVER_ERROR_DEG                   0.8f
#define CHASSIS_HEADING_RECOVER_RATE_DPS                    1.5f
#define CHASSIS_HEADING_CORRECTION_SIGN                     1.0f

typedef enum
{
  CHASSIS_STRAIGHT_STATE_WAIT = 0,
  CHASSIS_STRAIGHT_STATE_CASTER_ALIGN = 1,
  CHASSIS_STRAIGHT_STATE_HEADING_RECOVER = 2,
  CHASSIS_STRAIGHT_STATE_SPEED_RAMP = 3,
  CHASSIS_STRAIGHT_STATE_LINE_TRACK = 4,
  CHASSIS_STRAIGHT_STATE_DECELERATE = 5,
  CHASSIS_STRAIGHT_STATE_DONE = 6,
  CHASSIS_STRAIGHT_STATE_IMU_ERROR = 7,
  CHASSIS_STRAIGHT_STATE_MOTION_ERROR = 8
} chassis_straight_state_t;

static chassis_straight_state_t chassis_straight_state;
static uint16_t chassis_straight_state_ticks;
static uint16_t chassis_straight_recover_stable_ticks;
static uint16_t chassis_straight_no_progress_ticks;
static uint8_t chassis_straight_speed_ticks;
static uint8_t chassis_straight_print_ticks;
static int32_t chassis_straight_test_left_start_count;
static int32_t chassis_straight_test_right_start_count;
static int32_t chassis_straight_state_left_start_count;
static int32_t chassis_straight_state_right_start_count;
static int32_t chassis_straight_left_progress;
static int32_t chassis_straight_right_progress;
static int32_t chassis_straight_center_progress;
static int32_t chassis_straight_state_center_progress;
static int32_t chassis_straight_last_center_progress;
static float chassis_straight_heading_target_deg;
static float chassis_straight_heading_error_deg;
static float chassis_straight_heading_correction_rpm;
static float chassis_straight_base_rpm;
static float chassis_straight_left_target_rpm;
static float chassis_straight_right_target_rpm;
static uint32_t chassis_straight_last_cycle_count;

static int32_t chassis_straight_abs_count(int32_t value)
{
  return (value < 0) ? -value : value;
}

static float chassis_straight_abs_float(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float chassis_straight_limit(float value, float value_min, float value_max)
{
  if(value < value_min)
  {
    return value_min;
  }
  if(value > value_max)
  {
    return value_max;
  }
  return value;
}

static float chassis_straight_wrap_deg(float angle_deg)
{
  while(angle_deg > 180.0f)
  {
    angle_deg -= 360.0f;
  }
  while(angle_deg < -180.0f)
  {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

static float chassis_straight_get_sample_dt_s(void)
{
  uint32_t current_cycle_count = DWT->CYCCNT;
  uint32_t elapsed_cycles = current_cycle_count -
                            chassis_straight_last_cycle_count;
  float dt_s;

  chassis_straight_last_cycle_count = current_cycle_count;
  dt_s = (float)elapsed_cycles / (float)system_core_clock;
  return chassis_straight_limit(dt_s, 0.001f, 0.100f);
}

static void chassis_straight_reset_test_progress(void)
{
  chassis_straight_test_left_start_count = motor_encoder_get_count(MOTOR_LEFT);
  chassis_straight_test_right_start_count = motor_encoder_get_count(MOTOR_RIGHT);
  chassis_straight_left_progress = 0;
  chassis_straight_right_progress = 0;
  chassis_straight_center_progress = 0;
  chassis_straight_last_center_progress = 0;
  chassis_straight_no_progress_ticks = 0U;
}

static void chassis_straight_reset_state_progress(void)
{
  chassis_straight_state_left_start_count = motor_encoder_get_count(MOTOR_LEFT);
  chassis_straight_state_right_start_count = motor_encoder_get_count(MOTOR_RIGHT);
  chassis_straight_state_center_progress = 0;
}

static void chassis_straight_update_progress(void)
{
  int32_t current_left_count = motor_encoder_get_count(MOTOR_LEFT);
  int32_t current_right_count = motor_encoder_get_count(MOTOR_RIGHT);
  int32_t state_left_progress;
  int32_t state_right_progress;

  chassis_straight_left_progress = chassis_straight_abs_count(
      current_left_count - chassis_straight_test_left_start_count);
  chassis_straight_right_progress = chassis_straight_abs_count(
      current_right_count - chassis_straight_test_right_start_count);
  chassis_straight_center_progress =
      (chassis_straight_left_progress + chassis_straight_right_progress) / 2L;

  state_left_progress = chassis_straight_abs_count(
      current_left_count - chassis_straight_state_left_start_count);
  state_right_progress = chassis_straight_abs_count(
      current_right_count - chassis_straight_state_right_start_count);
  chassis_straight_state_center_progress =
      (state_left_progress + state_right_progress) / 2L;
}

static void chassis_straight_enter_state(chassis_straight_state_t next_state)
{
  chassis_straight_state = next_state;
  chassis_straight_state_ticks = 0U;
  chassis_straight_recover_stable_ticks = 0U;
  chassis_straight_reset_state_progress();

  if(next_state == CHASSIS_STRAIGHT_STATE_CASTER_ALIGN)
  {
    motor_speed_control_resync_feedback();
    chassis_straight_base_rpm = CHASSIS_STRAIGHT_ALIGN_RPM;
  }
  else if(next_state == CHASSIS_STRAIGHT_STATE_HEADING_RECOVER)
  {
    chassis_straight_base_rpm = CHASSIS_STRAIGHT_ALIGN_RPM;
  }
  else if((next_state == CHASSIS_STRAIGHT_STATE_DONE) ||
          (next_state == CHASSIS_STRAIGHT_STATE_IMU_ERROR) ||
          (next_state == CHASSIS_STRAIGHT_STATE_MOTION_ERROR))
  {
    chassis_straight_base_rpm = 0.0f;
    chassis_straight_left_target_rpm = 0.0f;
    chassis_straight_right_target_rpm = 0.0f;
    motor_speed_control_hold_brake();
  }
}

static uint8_t chassis_straight_is_driving(void)
{
  return ((chassis_straight_state == CHASSIS_STRAIGHT_STATE_CASTER_ALIGN) ||
          (chassis_straight_state == CHASSIS_STRAIGHT_STATE_HEADING_RECOVER) ||
          (chassis_straight_state == CHASSIS_STRAIGHT_STATE_SPEED_RAMP) ||
          (chassis_straight_state == CHASSIS_STRAIGHT_STATE_LINE_TRACK) ||
          (chassis_straight_state == CHASSIS_STRAIGHT_STATE_DECELERATE)) ? 1U : 0U;
}

static float chassis_straight_get_heading_limit(void)
{
  if(chassis_straight_state == CHASSIS_STRAIGHT_STATE_CASTER_ALIGN)
  {
    return CHASSIS_HEADING_ALIGN_LIMIT_RPM;
  }
  if(chassis_straight_state == CHASSIS_STRAIGHT_STATE_HEADING_RECOVER)
  {
    return CHASSIS_HEADING_RECOVER_LIMIT_RPM;
  }
  return CHASSIS_HEADING_TRACK_LIMIT_RPM;
}

/* 所有纠偏阶段都保持两个车轮向前，只使用有限的同向差速形成平滑弧线。 */
static void chassis_straight_update_heading_targets(void)
{
  float yaw_deg = imu_heading_get_yaw_deg();
  float yaw_rate_dps = imu_heading_get_rate_dps();
  float correction_limit = chassis_straight_get_heading_limit();
  float minimum_forward_rpm = 5.0f;

  chassis_straight_heading_error_deg =
      chassis_straight_wrap_deg(chassis_straight_heading_target_deg - yaw_deg);
  chassis_straight_heading_correction_rpm =
      (CHASSIS_HEADING_KP_RPM_PER_DEG * chassis_straight_heading_error_deg) -
      (CHASSIS_HEADING_KD_RPM_PER_DPS * yaw_rate_dps);
  chassis_straight_heading_correction_rpm *= CHASSIS_HEADING_CORRECTION_SIGN;
  chassis_straight_heading_correction_rpm =
      chassis_straight_limit(chassis_straight_heading_correction_rpm,
                             -correction_limit,
                             correction_limit);

  chassis_straight_left_target_rpm = chassis_straight_base_rpm -
                                     chassis_straight_heading_correction_rpm;
  chassis_straight_right_target_rpm = chassis_straight_base_rpm +
                                      chassis_straight_heading_correction_rpm;
  chassis_straight_left_target_rpm =
      chassis_straight_limit(chassis_straight_left_target_rpm,
                             minimum_forward_rpm,
                             CHASSIS_STRAIGHT_RUN_RPM + correction_limit);
  chassis_straight_right_target_rpm =
      chassis_straight_limit(chassis_straight_right_target_rpm,
                             minimum_forward_rpm,
                             CHASSIS_STRAIGHT_RUN_RPM + correction_limit);
  motor_speed_control_set_targets(chassis_straight_left_target_rpm,
                                  chassis_straight_right_target_rpm);
}

static uint8_t chassis_straight_encoder_mismatch(void)
{
  int32_t maximum_progress;
  int32_t progress_difference;

  maximum_progress = (chassis_straight_left_progress > chassis_straight_right_progress) ?
                     chassis_straight_left_progress :
                     chassis_straight_right_progress;
  progress_difference = chassis_straight_abs_count(
      chassis_straight_left_progress - chassis_straight_right_progress);

  if((maximum_progress >= CHASSIS_STRAIGHT_ENCODER_CHECK_MIN_COUNTS) &&
     (progress_difference >= CHASSIS_STRAIGHT_ENCODER_MISMATCH_COUNTS))
  {
    return 1U;
  }
  return 0U;
}

static void chassis_straight_update_deceleration(void)
{
  int32_t remaining_counts = CHASSIS_STRAIGHT_TOTAL_COUNTS -
                             chassis_straight_center_progress;
  float remaining_ratio;

  if(remaining_counts < 0L)
  {
    remaining_counts = 0L;
  }
  remaining_ratio = (float)remaining_counts /
                    (float)CHASSIS_STRAIGHT_DECEL_COUNTS;
  remaining_ratio = chassis_straight_limit(remaining_ratio, 0.0f, 1.0f);
  chassis_straight_base_rpm = CHASSIS_STRAIGHT_DECEL_END_RPM +
      ((CHASSIS_STRAIGHT_RUN_RPM - CHASSIS_STRAIGHT_DECEL_END_RPM) *
       remaining_ratio);
}

static void chassis_straight_update_state(void)
{
  float yaw_rate_dps = imu_heading_get_rate_dps();

  chassis_straight_state_ticks++;
  chassis_straight_update_progress();

  if(chassis_straight_state == CHASSIS_STRAIGHT_STATE_WAIT)
  {
    if(chassis_straight_state_ticks >= CHASSIS_STRAIGHT_START_WAIT_TICKS)
    {
      /* 原始航向只记录一次，后续对齐和纠偏均不能覆盖该目标。 */
      chassis_straight_heading_target_deg = imu_heading_get_yaw_deg();
      chassis_straight_reset_test_progress();
      chassis_straight_enter_state(CHASSIS_STRAIGHT_STATE_CASTER_ALIGN);
    }
  }
  else if(chassis_straight_is_driving() != 0U)
  {
    if(chassis_straight_center_progress >=
       (chassis_straight_last_center_progress + 2L))
    {
      chassis_straight_no_progress_ticks = 0U;
    }
    else
    {
      chassis_straight_no_progress_ticks++;
    }
    chassis_straight_last_center_progress = chassis_straight_center_progress;

    if(chassis_straight_no_progress_ticks >=
       CHASSIS_STRAIGHT_NO_PROGRESS_TIMEOUT_TICKS)
    {
      chassis_straight_enter_state(CHASSIS_STRAIGHT_STATE_MOTION_ERROR);
      return;
    }

    if(chassis_straight_center_progress >= CHASSIS_STRAIGHT_TOTAL_COUNTS)
    {
      chassis_straight_enter_state(CHASSIS_STRAIGHT_STATE_DONE);
      return;
    }

    if(chassis_straight_encoder_mismatch() != 0U)
    {
      chassis_straight_enter_state(CHASSIS_STRAIGHT_STATE_MOTION_ERROR);
      return;
    }

    if(chassis_straight_state == CHASSIS_STRAIGHT_STATE_CASTER_ALIGN)
    {
      if(chassis_straight_state_center_progress >= CHASSIS_STRAIGHT_ALIGN_COUNTS)
      {
        chassis_straight_enter_state(CHASSIS_STRAIGHT_STATE_HEADING_RECOVER);
      }
    }
    else if(chassis_straight_state == CHASSIS_STRAIGHT_STATE_HEADING_RECOVER)
    {
      if((chassis_straight_abs_float(chassis_straight_heading_error_deg) <=
          CHASSIS_HEADING_RECOVER_ERROR_DEG) &&
         (chassis_straight_abs_float(yaw_rate_dps) <=
          CHASSIS_HEADING_RECOVER_RATE_DPS))
      {
        chassis_straight_recover_stable_ticks++;
      }
      else
      {
        chassis_straight_recover_stable_ticks = 0U;
      }

      if(chassis_straight_recover_stable_ticks >=
         CHASSIS_STRAIGHT_RECOVER_STABLE_TICKS)
      {
        chassis_straight_enter_state(CHASSIS_STRAIGHT_STATE_SPEED_RAMP);
      }
      else if(chassis_straight_state_ticks >=
              CHASSIS_STRAIGHT_RECOVER_TIMEOUT_TICKS)
      {
        chassis_straight_enter_state(CHASSIS_STRAIGHT_STATE_MOTION_ERROR);
      }
    }
    else if(chassis_straight_state == CHASSIS_STRAIGHT_STATE_LINE_TRACK)
    {
      if(chassis_straight_center_progress >=
         (CHASSIS_STRAIGHT_TOTAL_COUNTS - CHASSIS_STRAIGHT_DECEL_COUNTS))
      {
        chassis_straight_enter_state(CHASSIS_STRAIGHT_STATE_DECELERATE);
      }
    }
    else if((chassis_straight_state == CHASSIS_STRAIGHT_STATE_SPEED_RAMP) &&
            (chassis_straight_center_progress >=
             (CHASSIS_STRAIGHT_TOTAL_COUNTS - CHASSIS_STRAIGHT_DECEL_COUNTS)))
    {
      chassis_straight_enter_state(CHASSIS_STRAIGHT_STATE_DECELERATE);
    }
    else if(chassis_straight_state == CHASSIS_STRAIGHT_STATE_DECELERATE)
    {
      chassis_straight_update_deceleration();
      if(chassis_straight_center_progress >= CHASSIS_STRAIGHT_TOTAL_COUNTS)
      {
        chassis_straight_enter_state(CHASSIS_STRAIGHT_STATE_DONE);
      }
    }
  }
}

static void chassis_straight_update_speed_ramp(void)
{
  if(chassis_straight_state != CHASSIS_STRAIGHT_STATE_SPEED_RAMP)
  {
    return;
  }

  chassis_straight_base_rpm += CHASSIS_STRAIGHT_RAMP_STEP_RPM;
  if(chassis_straight_base_rpm >= CHASSIS_STRAIGHT_RUN_RPM)
  {
    chassis_straight_base_rpm = CHASSIS_STRAIGHT_RUN_RPM;
    chassis_straight_enter_state(CHASSIS_STRAIGHT_STATE_LINE_TRACK);
  }
}

static void chassis_straight_print_data(void)
{
  float center_distance_m =
      ((float)chassis_straight_center_progress *
       CHASSIS_STRAIGHT_WHEEL_CIRCUMFERENCE_M) /
      (float)CHASSIS_STRAIGHT_ENCODER_COUNTS_PER_REV;

  printf("%d,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%d,%.2f,%.2f,%d\r\n",
         (int)chassis_straight_state,
         chassis_straight_heading_target_deg,
         imu_heading_get_yaw_deg(),
         chassis_straight_heading_error_deg,
         imu_heading_get_rate_dps(),
         center_distance_m,
         (float)chassis_straight_left_progress /
             (float)CHASSIS_STRAIGHT_ENCODER_COUNTS_PER_REV,
         (float)chassis_straight_right_progress /
             (float)CHASSIS_STRAIGHT_ENCODER_COUNTS_PER_REV,
         chassis_straight_left_target_rpm,
         motor_speed_control_get_measured_rpm(MOTOR_LEFT),
         motor_speed_control_get_pwm(MOTOR_LEFT),
         chassis_straight_right_target_rpm,
         motor_speed_control_get_measured_rpm(MOTOR_RIGHT),
         motor_speed_control_get_pwm(MOTOR_RIGHT));
}

void chassis_straight_test_init(void)
{
  int8_t imu_result;

  motor_speed_control_init();
  motor_speed_control_hold_brake();
  printf("\r\nforward caster alignment test\r\n");
  printf("state,target_yaw,yaw,error,gyro,center_m,left_turns,right_turns,left_target,left_rpm,left_pwm,right_target,right_rpm,right_pwm\r\n");

  imu_result = imu_heading_init();
  if(imu_result != BMI2_OK)
  {
    printf("BMI270 heading init failed rslt=%d\r\n", imu_result);
    chassis_straight_enter_state(CHASSIS_STRAIGHT_STATE_IMU_ERROR);
    return;
  }

  imu_heading_reset();
  chassis_straight_heading_target_deg = 0.0f;
  chassis_straight_heading_error_deg = 0.0f;
  chassis_straight_heading_correction_rpm = 0.0f;
  chassis_straight_base_rpm = 0.0f;
  chassis_straight_left_target_rpm = 0.0f;
  chassis_straight_right_target_rpm = 0.0f;
  chassis_straight_speed_ticks = 0U;
  chassis_straight_print_ticks = 0U;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  chassis_straight_last_cycle_count = DWT->CYCCNT;
  chassis_straight_reset_test_progress();
  chassis_straight_enter_state(CHASSIS_STRAIGHT_STATE_WAIT);
}

void chassis_straight_test_loop(void)
{
  int8_t imu_result;
  float sample_dt_s;

  delay_ms(CHASSIS_STRAIGHT_LOOP_PERIOD_MS);
  sample_dt_s = chassis_straight_get_sample_dt_s();
  imu_result = imu_heading_update(sample_dt_s);
  if(imu_result != BMI2_OK)
  {
    chassis_straight_enter_state(CHASSIS_STRAIGHT_STATE_IMU_ERROR);
  }

  chassis_straight_heading_error_deg = chassis_straight_wrap_deg(
      chassis_straight_heading_target_deg - imu_heading_get_yaw_deg());
  chassis_straight_update_state();

  if(chassis_straight_is_driving() != 0U)
  {
    chassis_straight_update_heading_targets();
  }

  chassis_straight_speed_ticks++;
  if(chassis_straight_speed_ticks >= CHASSIS_STRAIGHT_SPEED_PERIOD_TICKS)
  {
    chassis_straight_speed_ticks = 0U;
    chassis_straight_update_speed_ramp();
    if(chassis_straight_is_driving() != 0U)
    {
      motor_speed_control_update_now();
    }
    else
    {
      motor_speed_control_hold_brake();
    }
  }

  chassis_straight_print_ticks++;
  if(chassis_straight_print_ticks >= CHASSIS_STRAIGHT_PRINT_PERIOD_TICKS)
  {
    chassis_straight_print_ticks = 0U;
    chassis_straight_print_data();
  }
}
