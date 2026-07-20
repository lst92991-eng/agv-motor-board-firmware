/**
  *****************************************************************************
  * @file     chassis_motion_controller.c
  * @brief    串口前向任务和底盘直线闭环控制
  *
  * 当前版本只接受本地坐标系X轴正方向的短距离任务。每次接受任务时记录
  * BMI270当前航向作为零度方向，再通过万向轮低速对齐、航向恢复、速度
  * 斜坡和减速停车完成任务。串口协议和命令结构后续可直接复用于CAN FD。
  *****************************************************************************
  */

#include "chassis_motion_controller.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "at32f45x_board.h"
#include "bmi2_defs.h"
#include "debug_uart_transport.h"
#include "debug_uart_log.h"
#include "imu_heading.h"
#include "motion_command.h"
#include "motor_encoder.h"
#include "motor_speed_control.h"

#define CHASSIS_CONTROL_LOOP_PERIOD_MS                    10U
#define CHASSIS_CONTROL_SPEED_PERIOD_TICKS                 2U
#define CHASSIS_CONTROL_TELEMETRY_PERIOD_TICKS            10U

#define CHASSIS_CONTROL_COUNTS_PER_REV                  2640L
#define CHASSIS_CONTROL_WHEEL_CIRCUMFERENCE_MM           267.035f
#define CHASSIS_CONTROL_ALIGN_COUNTS                    1000L
#define CHASSIS_CONTROL_ALIGN_MAX_COUNTS                2000L
#define CHASSIS_CONTROL_DECEL_DISTANCE_MM                150.0f
#define CHASSIS_CONTROL_ENCODER_MISMATCH_COUNTS         1200L
#define CHASSIS_CONTROL_ENCODER_CHECK_MIN_COUNTS        1500L

#define CHASSIS_CONTROL_MIN_DISTANCE_MM                  300L
#define CHASSIS_CONTROL_MAX_DISTANCE_MM                 3000L
#define CHASSIS_CONTROL_MIN_SPEED_MM_S                    50U
#define CHASSIS_CONTROL_MAX_SPEED_MM_S                   100U

#define CHASSIS_CONTROL_ALIGN_RPM                         12.0f
#define CHASSIS_CONTROL_MAX_RUN_RPM                       20.0f
#define CHASSIS_CONTROL_DECEL_END_RPM                     10.0f
#define CHASSIS_CONTROL_RAMP_STEP_RPM                      0.2f

#define CHASSIS_HEADING_KP_RPM_PER_DEG                     0.70f
#define CHASSIS_HEADING_KD_RPM_PER_DPS                     0.08f
#define CHASSIS_HEADING_ALIGN_LIMIT_RPM                     2.0f
#define CHASSIS_HEADING_RECOVER_LIMIT_RPM                   5.0f
#define CHASSIS_HEADING_TRACK_LIMIT_RPM                     4.0f
#define CHASSIS_HEADING_RECOVER_ERROR_DEG                   0.8f
#define CHASSIS_HEADING_RECOVER_RATE_DPS                    1.5f
#define CHASSIS_HEADING_CORRECTION_SIGN                     1.0f

#define CHASSIS_CONTROL_RECOVER_STABLE_TICKS               30U
#define CHASSIS_CONTROL_RECOVER_TIMEOUT_TICKS              500U
#define CHASSIS_CONTROL_CASTER_STABLE_TICKS                 30U
#define CHASSIS_CONTROL_CASTER_STABLE_RATE_DPS               2.0f
#define CHASSIS_CONTROL_CASTER_SPEED_DIFF_RPM                3.0f
#define CHASSIS_CONTROL_NO_PROGRESS_TIMEOUT_TICKS          300U
#define CHASSIS_CONTROL_TASK_TIMEOUT_MARGIN_TICKS         3000UL
#define CHASSIS_CONTROL_TASK_TIMEOUT_MIN_TICKS            6000UL

#define CHASSIS_FAULT_NONE                              0x0000U
#define CHASSIS_FAULT_ESTOP                             0x0001U
#define CHASSIS_FAULT_IMU                               0x0002U
#define CHASSIS_FAULT_MOTION                            0x0004U

typedef enum
{
  CHASSIS_STATE_IDLE = 0,
  CHASSIS_STATE_CASTER_ALIGN,
  CHASSIS_STATE_HEADING_RECOVER,
  CHASSIS_STATE_SPEED_RAMP,
  CHASSIS_STATE_LINE_TRACK,
  CHASSIS_STATE_DECELERATE,
  CHASSIS_STATE_COMPLETED,
  CHASSIS_STATE_HOLD,
  CHASSIS_STATE_ESTOP,
  CHASSIS_STATE_IMU_ERROR,
  CHASSIS_STATE_MOTION_ERROR
} chassis_state_t;

static chassis_state_t chassis_state;
static uint16_t chassis_fault;
static uint16_t chassis_task_sequence;
static uint8_t chassis_task_active;
static uint16_t chassis_state_ticks;
static uint16_t chassis_recover_stable_ticks;
static uint16_t chassis_caster_stable_ticks;
static uint16_t chassis_no_progress_ticks;
static uint32_t chassis_task_ticks;
static uint32_t chassis_task_timeout_ticks;
static uint8_t chassis_speed_ticks;
static uint8_t chassis_telemetry_ticks;
static uint32_t chassis_uptime_ms;

static int32_t chassis_task_left_start_count;
static int32_t chassis_task_right_start_count;
static int32_t chassis_state_left_start_count;
static int32_t chassis_state_right_start_count;
static int32_t chassis_left_progress;
static int32_t chassis_right_progress;
static int32_t chassis_center_progress;
static int32_t chassis_state_center_progress;
static int32_t chassis_last_center_progress;
static int32_t chassis_last_pose_progress;
static int32_t chassis_total_counts;
static int32_t chassis_target_distance_mm;

static float chassis_heading_target_deg;
static float chassis_heading_error_deg;
static float chassis_heading_correction_rpm;
static float chassis_requested_run_rpm;
static float chassis_base_rpm;
static float chassis_left_target_rpm;
static float chassis_right_target_rpm;
static float chassis_pose_x_mm;
static float chassis_pose_y_mm;
static uint32_t chassis_last_cycle_count;

static int32_t chassis_abs_count(int32_t value)
{
  return (value < 0L) ? -value : value;
}

static float chassis_abs_float(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float chassis_limit(float value, float value_min, float value_max)
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

static float chassis_wrap_deg(float angle_deg)
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

static int32_t chassis_round_to_int(float value)
{
  return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
                           (int32_t)(value - 0.5f);
}

static float chassis_get_sample_dt_s(void)
{
  uint32_t current_cycle_count = DWT->CYCCNT;
  uint32_t elapsed_cycles = current_cycle_count - chassis_last_cycle_count;
  float dt_s;

  chassis_last_cycle_count = current_cycle_count;
  dt_s = (float)elapsed_cycles / (float)system_core_clock;
  return chassis_limit(dt_s, 0.001f, 0.100f);
}

static const char *chassis_get_state_name(void)
{
  switch(chassis_state)
  {
    case CHASSIS_STATE_IDLE:
      return "IDLE";
    case CHASSIS_STATE_CASTER_ALIGN:
      return "CASTER_ALIGN";
    case CHASSIS_STATE_HEADING_RECOVER:
      return "HEADING_RECOVER";
    case CHASSIS_STATE_SPEED_RAMP:
      return "SPEED_RAMP";
    case CHASSIS_STATE_LINE_TRACK:
      return "LINE_TRACK";
    case CHASSIS_STATE_DECELERATE:
      return "DECELERATE";
    case CHASSIS_STATE_COMPLETED:
      return "COMPLETED";
    case CHASSIS_STATE_HOLD:
      return "HOLD";
    case CHASSIS_STATE_ESTOP:
      return "ESTOP";
    case CHASSIS_STATE_IMU_ERROR:
      return "IMU_ERROR";
    case CHASSIS_STATE_MOTION_ERROR:
      return "MOTION_ERROR";
    default:
      return "UNKNOWN";
  }
}

static uint8_t chassis_is_driving(void)
{
  return ((chassis_state == CHASSIS_STATE_CASTER_ALIGN) ||
          (chassis_state == CHASSIS_STATE_HEADING_RECOVER) ||
          (chassis_state == CHASSIS_STATE_SPEED_RAMP) ||
          (chassis_state == CHASSIS_STATE_LINE_TRACK) ||
          (chassis_state == CHASSIS_STATE_DECELERATE)) ? 1U : 0U;
}

static void chassis_reset_task_progress(void)
{
  chassis_task_left_start_count = motor_encoder_get_count(MOTOR_LEFT);
  chassis_task_right_start_count = motor_encoder_get_count(MOTOR_RIGHT);
  chassis_left_progress = 0L;
  chassis_right_progress = 0L;
  chassis_center_progress = 0L;
  chassis_last_center_progress = 0L;
  chassis_last_pose_progress = 0L;
  chassis_no_progress_ticks = 0U;
  chassis_task_ticks = 0U;
}

static void chassis_reset_state_progress(void)
{
  chassis_state_left_start_count = motor_encoder_get_count(MOTOR_LEFT);
  chassis_state_right_start_count = motor_encoder_get_count(MOTOR_RIGHT);
  chassis_state_center_progress = 0L;
}

static void chassis_update_progress(void)
{
  int32_t current_left_count = motor_encoder_get_count(MOTOR_LEFT);
  int32_t current_right_count = motor_encoder_get_count(MOTOR_RIGHT);
  int32_t state_left_progress;
  int32_t state_right_progress;

  chassis_left_progress = chassis_abs_count(
      current_left_count - chassis_task_left_start_count);
  chassis_right_progress = chassis_abs_count(
      current_right_count - chassis_task_right_start_count);
  chassis_center_progress =
      (chassis_left_progress + chassis_right_progress) / 2L;

  state_left_progress = chassis_abs_count(
      current_left_count - chassis_state_left_start_count);
  state_right_progress = chassis_abs_count(
      current_right_count - chassis_state_right_start_count);
  chassis_state_center_progress =
      (state_left_progress + state_right_progress) / 2L;
}

static void chassis_update_pose(void)
{
  int32_t delta_counts = chassis_center_progress - chassis_last_pose_progress;
  float delta_distance_mm;
  float relative_yaw_deg;
  float relative_yaw_rad;

  if(delta_counts <= 0L)
  {
    return;
  }

  chassis_last_pose_progress = chassis_center_progress;
  delta_distance_mm = ((float)delta_counts *
                       CHASSIS_CONTROL_WHEEL_CIRCUMFERENCE_MM) /
                      (float)CHASSIS_CONTROL_COUNTS_PER_REV;
  relative_yaw_deg = chassis_wrap_deg(imu_heading_get_yaw_deg() -
                                      chassis_heading_target_deg);
  relative_yaw_rad = relative_yaw_deg * 0.01745329252f;
  chassis_pose_x_mm += delta_distance_mm * cosf(relative_yaw_rad);
  chassis_pose_y_mm += delta_distance_mm * sinf(relative_yaw_rad);
}

static void chassis_send_task_status(const char *status, uint16_t code)
{
  debug_uart_log_printf("ACK,%u,GOTO,%s,%u\r\n",
                        (uint32_t)chassis_task_sequence,
                        status,
                        (uint32_t)code);
}

static void chassis_cancel_active_task(void)
{
  if(chassis_task_active != 0U)
  {
    chassis_send_task_status("CANCELED", CHASSIS_FAULT_NONE);
    chassis_task_active = 0U;
  }
}

static void chassis_enter_state(chassis_state_t next_state)
{
  chassis_state = next_state;
  chassis_state_ticks = 0U;
  chassis_recover_stable_ticks = 0U;
  chassis_caster_stable_ticks = 0U;
  chassis_reset_state_progress();

  if(next_state == CHASSIS_STATE_CASTER_ALIGN)
  {
    motor_speed_control_resync_feedback();
    chassis_base_rpm = chassis_limit(chassis_requested_run_rpm,
                                     8.0f,
                                     CHASSIS_CONTROL_ALIGN_RPM);
  }
  else if(next_state == CHASSIS_STATE_HEADING_RECOVER)
  {
    chassis_base_rpm = chassis_limit(chassis_requested_run_rpm,
                                     8.0f,
                                     CHASSIS_CONTROL_ALIGN_RPM);
  }
  else if((next_state == CHASSIS_STATE_COMPLETED) ||
          (next_state == CHASSIS_STATE_HOLD) ||
          (next_state == CHASSIS_STATE_ESTOP) ||
          (next_state == CHASSIS_STATE_IMU_ERROR) ||
          (next_state == CHASSIS_STATE_MOTION_ERROR))
  {
    chassis_base_rpm = 0.0f;
    chassis_left_target_rpm = 0.0f;
    chassis_right_target_rpm = 0.0f;
    motor_speed_control_hold_brake();
  }
}

static void chassis_complete_task(void)
{
  chassis_enter_state(CHASSIS_STATE_COMPLETED);
  if(chassis_task_active != 0U)
  {
    chassis_send_task_status("COMPLETED", CHASSIS_FAULT_NONE);
    chassis_task_active = 0U;
  }
}

static void chassis_fail_task(uint16_t fault)
{
  chassis_fault = fault;
  if(fault == CHASSIS_FAULT_IMU)
  {
    chassis_enter_state(CHASSIS_STATE_IMU_ERROR);
  }
  else
  {
    chassis_enter_state(CHASSIS_STATE_MOTION_ERROR);
  }

  if(chassis_task_active != 0U)
  {
    chassis_send_task_status("FAULT", fault);
    chassis_task_active = 0U;
  }
}

/* 在刹车前保留故障瞬间的目标、反馈和编码器诊断信息。 */
static void chassis_fail_motion(const char *reason)
{
  debug_uart_log_printf(
      "MOTION_FAULT,%s,left_count=%ld,right_count=%ld,"
      "left_target=%ld,right_target=%ld,left_rpm=%ld,right_rpm=%ld,"
      "left_pwm=%d,right_pwm=%d,left_invalid=%lu,right_invalid=%lu\r\n",
      reason,
      chassis_left_progress,
      chassis_right_progress,
      chassis_round_to_int(chassis_left_target_rpm * 100.0f),
      chassis_round_to_int(chassis_right_target_rpm * 100.0f),
      chassis_round_to_int(
          motor_speed_control_get_measured_rpm(MOTOR_LEFT) * 100.0f),
      chassis_round_to_int(
          motor_speed_control_get_measured_rpm(MOTOR_RIGHT) * 100.0f),
      motor_speed_control_get_pwm(MOTOR_LEFT),
      motor_speed_control_get_pwm(MOTOR_RIGHT),
      motor_encoder_get_invalid_transition_count(MOTOR_LEFT),
      motor_encoder_get_invalid_transition_count(MOTOR_RIGHT));
  chassis_fail_task(CHASSIS_FAULT_MOTION);
}

static float chassis_get_heading_limit(void)
{
  if(chassis_state == CHASSIS_STATE_CASTER_ALIGN)
  {
    return CHASSIS_HEADING_ALIGN_LIMIT_RPM;
  }
  if(chassis_state == CHASSIS_STATE_HEADING_RECOVER)
  {
    return CHASSIS_HEADING_RECOVER_LIMIT_RPM;
  }
  return CHASSIS_HEADING_TRACK_LIMIT_RPM;
}

/* 所有纠偏阶段都让两个主动轮向前，只通过同向差速形成平滑弧线。 */
static void chassis_update_heading_targets(void)
{
  float yaw_deg = imu_heading_get_yaw_deg();
  float yaw_rate_dps = imu_heading_get_rate_dps();
  float correction_limit = chassis_get_heading_limit();

  chassis_heading_error_deg = chassis_wrap_deg(chassis_heading_target_deg -
                                               yaw_deg);
  chassis_heading_correction_rpm =
      (CHASSIS_HEADING_KP_RPM_PER_DEG * chassis_heading_error_deg) -
      (CHASSIS_HEADING_KD_RPM_PER_DPS * yaw_rate_dps);
  chassis_heading_correction_rpm *= CHASSIS_HEADING_CORRECTION_SIGN;
  chassis_heading_correction_rpm = chassis_limit(
      chassis_heading_correction_rpm, -correction_limit, correction_limit);

  chassis_left_target_rpm = chassis_base_rpm -
                            chassis_heading_correction_rpm;
  chassis_right_target_rpm = chassis_base_rpm +
                             chassis_heading_correction_rpm;
  chassis_left_target_rpm = chassis_limit(
      chassis_left_target_rpm, 5.0f,
      CHASSIS_CONTROL_MAX_RUN_RPM + correction_limit);
  chassis_right_target_rpm = chassis_limit(
      chassis_right_target_rpm, 5.0f,
      CHASSIS_CONTROL_MAX_RUN_RPM + correction_limit);
  motor_speed_control_set_targets(chassis_left_target_rpm,
                                  chassis_right_target_rpm);
}

static uint8_t chassis_encoder_mismatch(void)
{
  int32_t maximum_progress;
  int32_t progress_difference;

  maximum_progress = (chassis_left_progress > chassis_right_progress) ?
                     chassis_left_progress : chassis_right_progress;
  progress_difference = chassis_abs_count(chassis_left_progress -
                                          chassis_right_progress);
  return ((maximum_progress >= CHASSIS_CONTROL_ENCODER_CHECK_MIN_COUNTS) &&
          (progress_difference >=
           CHASSIS_CONTROL_ENCODER_MISMATCH_COUNTS)) ? 1U : 0U;
}

/* 没有万向轮角度传感器，只能用车轮速度和Yaw稳定间接判断摆正。 */
static uint8_t chassis_caster_motion_stable(void)
{
  float left_rpm = motor_speed_control_get_measured_rpm(MOTOR_LEFT);
  float right_rpm = motor_speed_control_get_measured_rpm(MOTOR_RIGHT);
  float speed_difference = chassis_abs_float(left_rpm - right_rpm);

  return ((left_rpm >= 5.0f) &&
          (right_rpm >= 5.0f) &&
          (speed_difference <= CHASSIS_CONTROL_CASTER_SPEED_DIFF_RPM) &&
          (chassis_abs_float(imu_heading_get_rate_dps()) <=
           CHASSIS_CONTROL_CASTER_STABLE_RATE_DPS)) ? 1U : 0U;
}

static void chassis_update_deceleration(void)
{
  float remaining_distance_mm = (float)chassis_target_distance_mm -
                                chassis_pose_x_mm;
  float remaining_ratio;
  float decel_end_rpm = chassis_limit(CHASSIS_CONTROL_DECEL_END_RPM,
                                      5.0f,
                                      chassis_requested_run_rpm);

  if(remaining_distance_mm < 0.0f)
  {
    remaining_distance_mm = 0.0f;
  }
  remaining_ratio = remaining_distance_mm /
                    CHASSIS_CONTROL_DECEL_DISTANCE_MM;
  remaining_ratio = chassis_limit(remaining_ratio, 0.0f, 1.0f);
  chassis_base_rpm = decel_end_rpm +
      ((chassis_requested_run_rpm - decel_end_rpm) * remaining_ratio);
}

static void chassis_update_speed_ramp(void)
{
  if(chassis_state != CHASSIS_STATE_SPEED_RAMP)
  {
    return;
  }

  chassis_base_rpm += CHASSIS_CONTROL_RAMP_STEP_RPM;
  if(chassis_base_rpm >= chassis_requested_run_rpm)
  {
    chassis_base_rpm = chassis_requested_run_rpm;
    chassis_enter_state(CHASSIS_STATE_LINE_TRACK);
  }
}

static void chassis_update_state(void)
{
  float yaw_rate_dps = imu_heading_get_rate_dps();

  if(chassis_is_driving() == 0U)
  {
    return;
  }

  chassis_state_ticks++;
  chassis_task_ticks++;
  chassis_update_progress();
  chassis_update_pose();

  if(chassis_center_progress >=
     (chassis_last_center_progress + 2L))
  {
    chassis_no_progress_ticks = 0U;
  }
  else
  {
    chassis_no_progress_ticks++;
  }
  chassis_last_center_progress = chassis_center_progress;

  if(chassis_no_progress_ticks >=
     CHASSIS_CONTROL_NO_PROGRESS_TIMEOUT_TICKS)
  {
    chassis_fail_motion("NO_PROGRESS");
    return;
  }
  if(chassis_task_ticks >= chassis_task_timeout_ticks)
  {
    chassis_fail_motion("TASK_TIMEOUT");
    return;
  }
  if((chassis_pose_x_mm >= (float)chassis_target_distance_mm) &&
     (chassis_state != CHASSIS_STATE_CASTER_ALIGN) &&
     (chassis_state != CHASSIS_STATE_HEADING_RECOVER))
  {
    chassis_complete_task();
    return;
  }
  if(chassis_encoder_mismatch() != 0U)
  {
    chassis_fail_motion("ENCODER_MISMATCH");
    return;
  }

  if(chassis_state == CHASSIS_STATE_CASTER_ALIGN)
  {
    if(chassis_state_center_progress >= CHASSIS_CONTROL_ALIGN_COUNTS)
    {
      if(chassis_caster_motion_stable() != 0U)
      {
        chassis_caster_stable_ticks++;
      }
      else
      {
        chassis_caster_stable_ticks = 0U;
      }

      if((chassis_caster_stable_ticks >=
          CHASSIS_CONTROL_CASTER_STABLE_TICKS) ||
         (chassis_state_center_progress >=
          CHASSIS_CONTROL_ALIGN_MAX_COUNTS))
      {
        chassis_enter_state(CHASSIS_STATE_HEADING_RECOVER);
      }
    }
  }
  else if(chassis_state == CHASSIS_STATE_HEADING_RECOVER)
  {
    if((chassis_abs_float(chassis_heading_error_deg) <=
        CHASSIS_HEADING_RECOVER_ERROR_DEG) &&
       (chassis_abs_float(yaw_rate_dps) <=
        CHASSIS_HEADING_RECOVER_RATE_DPS))
    {
      chassis_recover_stable_ticks++;
    }
    else
    {
      chassis_recover_stable_ticks = 0U;
    }

    if(chassis_recover_stable_ticks >=
       CHASSIS_CONTROL_RECOVER_STABLE_TICKS)
    {
      chassis_enter_state(CHASSIS_STATE_SPEED_RAMP);
    }
    else if(chassis_state_ticks >=
            CHASSIS_CONTROL_RECOVER_TIMEOUT_TICKS)
    {
      chassis_fail_motion("HEADING_RECOVER_TIMEOUT");
    }
  }
  else if((chassis_state == CHASSIS_STATE_LINE_TRACK) ||
          (chassis_state == CHASSIS_STATE_SPEED_RAMP))
  {
    if(chassis_pose_x_mm >=
       ((float)chassis_target_distance_mm -
        CHASSIS_CONTROL_DECEL_DISTANCE_MM))
    {
      chassis_enter_state(CHASSIS_STATE_DECELERATE);
    }
  }
  else if(chassis_state == CHASSIS_STATE_DECELERATE)
  {
    chassis_update_deceleration();
  }
}

static void chassis_reset_local_pose(void)
{
  chassis_pose_x_mm = 0.0f;
  chassis_pose_y_mm = 0.0f;
  chassis_last_pose_progress = 0L;
}

motion_command_handle_result_t chassis_motion_controller_handle_command(
    const motion_command_t *command)
{
  const motion_goto_local_data_t *goto_data;

  if(command == NULL)
  {
    return MOTION_HANDLE_REJECTED;
  }

  if(command->type == MOTION_CMD_GOTO_LOCAL)
  {
    goto_data = &command->data.goto_local;
    if(chassis_task_active != 0U)
    {
      return MOTION_HANDLE_BUSY;
    }
    if((chassis_state == CHASSIS_STATE_ESTOP) ||
       (chassis_state == CHASSIS_STATE_IMU_ERROR) ||
       (chassis_state == CHASSIS_STATE_MOTION_ERROR) ||
       (imu_heading_get_status() != BMI2_OK))
    {
      debug_uart_log_printf("TASK_REJECT,%u,FAULT_STATE,%u\r\n",
                            (uint32_t)command->sequence,
                            (uint32_t)chassis_fault);
      return MOTION_HANDLE_REJECTED;
    }
    if((goto_data->x_mm < CHASSIS_CONTROL_MIN_DISTANCE_MM) ||
       (goto_data->x_mm > CHASSIS_CONTROL_MAX_DISTANCE_MM) ||
       (goto_data->y_mm != 0L) ||
       (goto_data->yaw_cdeg != 0L) ||
       (goto_data->speed_mm_s < CHASSIS_CONTROL_MIN_SPEED_MM_S) ||
       (goto_data->speed_mm_s > CHASSIS_CONTROL_MAX_SPEED_MM_S))
    {
      debug_uart_log_printf(
          "TASK_REJECT,%u,FORWARD_LIMIT,x=%ld,y=%ld,yaw=%ld,speed=%u\r\n",
          (uint32_t)command->sequence,
          goto_data->x_mm,
          goto_data->y_mm,
          goto_data->yaw_cdeg,
          (uint32_t)goto_data->speed_mm_s);
      return MOTION_HANDLE_REJECTED;
    }

    chassis_task_sequence = command->sequence;
    chassis_target_distance_mm = goto_data->x_mm;
    chassis_total_counts = chassis_round_to_int(
        ((float)chassis_target_distance_mm *
         (float)CHASSIS_CONTROL_COUNTS_PER_REV) /
        CHASSIS_CONTROL_WHEEL_CIRCUMFERENCE_MM);
    chassis_requested_run_rpm = chassis_limit(
        ((float)goto_data->speed_mm_s * 60.0f) /
        CHASSIS_CONTROL_WHEEL_CIRCUMFERENCE_MM,
        8.0f,
        CHASSIS_CONTROL_MAX_RUN_RPM);
    chassis_task_timeout_ticks =
        (((uint32_t)chassis_target_distance_mm * 100U) /
         (uint32_t)goto_data->speed_mm_s) +
        CHASSIS_CONTROL_TASK_TIMEOUT_MARGIN_TICKS;
    if(chassis_task_timeout_ticks < CHASSIS_CONTROL_TASK_TIMEOUT_MIN_TICKS)
    {
      chassis_task_timeout_ticks = CHASSIS_CONTROL_TASK_TIMEOUT_MIN_TICKS;
    }
    chassis_heading_target_deg = imu_heading_get_yaw_deg();
    chassis_heading_error_deg = 0.0f;
    chassis_heading_correction_rpm = 0.0f;
    chassis_fault = CHASSIS_FAULT_NONE;
    chassis_task_active = 1U;
    chassis_reset_local_pose();
    chassis_reset_task_progress();
    chassis_enter_state(CHASSIS_STATE_CASTER_ALIGN);
    debug_uart_log_printf("TASK_START,%u,%ld,%u,%ld\r\n",
                          (uint32_t)chassis_task_sequence,
                          chassis_target_distance_mm,
                          (uint32_t)goto_data->speed_mm_s,
                          chassis_total_counts);
    return MOTION_HANDLE_ACCEPTED;
  }

  if(command->type == MOTION_CMD_STOP)
  {
    chassis_cancel_active_task();
    if((chassis_state != CHASSIS_STATE_ESTOP) &&
       (chassis_state != CHASSIS_STATE_IMU_ERROR) &&
       (chassis_state != CHASSIS_STATE_MOTION_ERROR))
    {
      chassis_enter_state(CHASSIS_STATE_HOLD);
    }
    else
    {
      motor_speed_control_hold_brake();
    }
    return MOTION_HANDLE_ACCEPTED;
  }

  if(command->type == MOTION_CMD_ESTOP)
  {
    chassis_cancel_active_task();
    chassis_fault = CHASSIS_FAULT_ESTOP;
    chassis_enter_state(CHASSIS_STATE_ESTOP);
    return MOTION_HANDLE_ACCEPTED;
  }

  if(command->type == MOTION_CMD_RESET_ODOM)
  {
    if(chassis_task_active != 0U)
    {
      return MOTION_HANDLE_BUSY;
    }
    if(imu_heading_get_status() != BMI2_OK)
    {
      return MOTION_HANDLE_REJECTED;
    }

    motor_speed_control_hold_brake();
    motor_encoder_reset_all();
    motor_speed_control_resync_feedback();
    imu_heading_reset();
    chassis_heading_target_deg = 0.0f;
    chassis_fault = CHASSIS_FAULT_NONE;
    chassis_reset_local_pose();
    chassis_reset_task_progress();
    chassis_enter_state(CHASSIS_STATE_IDLE);
    return MOTION_HANDLE_ACCEPTED;
  }

  return MOTION_HANDLE_REJECTED;
}

static void chassis_send_telemetry(void)
{
  float relative_yaw_deg = chassis_wrap_deg(imu_heading_get_yaw_deg() -
                                            chassis_heading_target_deg);
  float left_rpm = motor_speed_control_get_measured_rpm(MOTOR_LEFT);
  float right_rpm = motor_speed_control_get_measured_rpm(MOTOR_RIGHT);
  float linear_mm_s = ((left_rpm + right_rpm) * 0.5f *
                       CHASSIS_CONTROL_WHEEL_CIRCUMFERENCE_MM) / 60.0f;

  debug_uart_log_printf(
      "POSE,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%s,0x%04X,"
      "%ld,%ld,%d,%d,%ld,%ld,%lu,%lu\r\n",
      chassis_uptime_ms,
      chassis_round_to_int(chassis_pose_x_mm),
      chassis_round_to_int(chassis_pose_y_mm),
      chassis_round_to_int(relative_yaw_deg * 100.0f),
      chassis_round_to_int(linear_mm_s),
      chassis_round_to_int(imu_heading_get_rate_dps() * 100.0f),
      chassis_round_to_int(left_rpm * 100.0f),
      chassis_round_to_int(right_rpm * 100.0f),
      chassis_get_state_name(),
      (uint32_t)chassis_fault,
      chassis_round_to_int(chassis_left_target_rpm * 100.0f),
      chassis_round_to_int(chassis_right_target_rpm * 100.0f),
      motor_speed_control_get_pwm(MOTOR_LEFT),
      motor_speed_control_get_pwm(MOTOR_RIGHT),
      motor_encoder_get_count(MOTOR_LEFT),
      motor_encoder_get_count(MOTOR_RIGHT),
      motor_encoder_get_invalid_transition_count(MOTOR_LEFT),
      motor_encoder_get_invalid_transition_count(MOTOR_RIGHT));
}

void chassis_motion_controller_init(void)
{
  int8_t imu_result;

  debug_uart_transport_init();
  motor_speed_control_init();
  motor_speed_control_hold_brake();

  chassis_state = CHASSIS_STATE_IDLE;
  chassis_fault = CHASSIS_FAULT_NONE;
  chassis_task_sequence = 0U;
  chassis_task_active = 0U;
  chassis_state_ticks = 0U;
  chassis_recover_stable_ticks = 0U;
  chassis_caster_stable_ticks = 0U;
  chassis_no_progress_ticks = 0U;
  chassis_task_ticks = 0U;
  chassis_task_timeout_ticks = CHASSIS_CONTROL_TASK_TIMEOUT_MIN_TICKS;
  chassis_speed_ticks = 0U;
  chassis_telemetry_ticks = 0U;
  chassis_uptime_ms = 0U;
  chassis_target_distance_mm = 0L;
  chassis_total_counts = 0L;
  chassis_heading_target_deg = 0.0f;
  chassis_heading_error_deg = 0.0f;
  chassis_heading_correction_rpm = 0.0f;
  chassis_requested_run_rpm = 0.0f;
  chassis_base_rpm = 0.0f;
  chassis_left_target_rpm = 0.0f;
  chassis_right_target_rpm = 0.0f;
  chassis_reset_local_pose();
  chassis_reset_task_progress();

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  chassis_last_cycle_count = DWT->CYCCNT;

  debug_uart_log_printf(
      "\r\nCHASSIS_CONTROL,IMU_CALIBRATING,KEEP_STILL\r\n");
  imu_result = imu_heading_init();
  if(imu_result == BMI2_OK)
  {
    imu_heading_reset();
    chassis_heading_target_deg = 0.0f;
    debug_uart_log_printf(
        "CHASSIS_CONTROL,READY,FORWARD_ONLY,300_3000MM,50_100MM_S\r\n");
  }
  else
  {
    chassis_fault = CHASSIS_FAULT_IMU;
    chassis_enter_state(CHASSIS_STATE_IMU_ERROR);
    debug_uart_log_printf("CHASSIS_CONTROL,NOT_READY,IMU_ERROR,%d\r\n",
                          imu_result);
  }

  DWT->CYCCNT = 0U;
  chassis_last_cycle_count = DWT->CYCCNT;
}

void chassis_motion_controller_step_10ms(void)
{
  int8_t imu_result;
  float sample_dt_s;

  chassis_uptime_ms += CHASSIS_CONTROL_LOOP_PERIOD_MS;
  sample_dt_s = chassis_get_sample_dt_s();

  imu_result = imu_heading_update(sample_dt_s);
  if((imu_result != BMI2_OK) &&
     (chassis_state != CHASSIS_STATE_IMU_ERROR))
  {
    chassis_fail_task(CHASSIS_FAULT_IMU);
  }

  chassis_heading_error_deg = chassis_wrap_deg(
      chassis_heading_target_deg - imu_heading_get_yaw_deg());
  chassis_update_state();

  if(chassis_is_driving() != 0U)
  {
    chassis_update_heading_targets();
  }

  chassis_speed_ticks++;
  if(chassis_speed_ticks >= CHASSIS_CONTROL_SPEED_PERIOD_TICKS)
  {
    chassis_speed_ticks = 0U;
    chassis_update_speed_ramp();
    if(chassis_is_driving() != 0U)
    {
      motor_speed_control_update_now();
    }
    else
    {
      motor_speed_control_hold_brake();
    }
  }

  chassis_telemetry_ticks++;
  if(chassis_telemetry_ticks >= CHASSIS_CONTROL_TELEMETRY_PERIOD_TICKS)
  {
    chassis_telemetry_ticks = 0U;
    chassis_send_telemetry();
  }
}
