/**
  *****************************************************************************
  * @file     chassis_motion_controller.c
  * @brief    串口/CAN FD直线与原地旋转任务闭环控制
  *
  * 直线任务记录BMI270当前航向，通过万向轮低速对齐、航向恢复、速度
  * 斜坡和减速停车完成。纯旋转任务让两个主动轮反向运行，以编码器弧长
  * 作为完成条件；防撞、通信租约和本地故障仍能随时终止任务。
  *****************************************************************************
  */

#include "chassis_motion_controller.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "at32f45x_board.h"
#include "bmi2_defs.h"
#include "bumper_safety.h"
#include "canfd_protocol.h"
#include "chassis_heading_fusion.h"
#include "debug_uart_transport.h"
#include "debug_uart_log.h"
#include "imu_heading.h"
#include "magnetic_follow_controller.h"
#include "motion_command.h"
#include "motor_encoder.h"
#include "motor_speed_control.h"

#define CHASSIS_CONTROL_LOOP_PERIOD_MS                    10U
#define CHASSIS_CONTROL_SPEED_PERIOD_TICKS                 2U
#define CHASSIS_CONTROL_TELEMETRY_PERIOD_TICKS            10U

#define CHASSIS_CONTROL_COUNTS_PER_REV                  2640L
#define CHASSIS_CONTROL_WHEEL_CIRCUMFERENCE_MM           267.035f
#define CHASSIS_CONTROL_DRIVE_TRACK_MM                    500.0f
#define CHASSIS_CONTROL_PI                                  3.14159265f
#define CHASSIS_CONTROL_ALIGN_COUNTS                    1000L
#define CHASSIS_CONTROL_ALIGN_MAX_COUNTS                2000L
#define CHASSIS_CONTROL_DECEL_DISTANCE_MM                150.0f
#define CHASSIS_CONTROL_HIGH_SPEED_DECEL_DISTANCE_MM     650.0f
#define CHASSIS_CONTROL_ENCODER_MISMATCH_COUNTS         1200L
#define CHASSIS_CONTROL_ENCODER_CHECK_MIN_COUNTS        1500L

#define CHASSIS_CONTROL_MIN_DISTANCE_MM                  300L
#define CHASSIS_CONTROL_MAX_DISTANCE_MM                 3000L
#define CHASSIS_CONTROL_MAX_MAG_DISTANCE_MM             6000L
#define CHASSIS_CONTROL_MIN_SPEED_MM_S                    50U
#define CHASSIS_CONTROL_MAX_SPEED_MM_S                   100U
#define CHASSIS_CONTROL_MAX_ROTATION_SPEED_MM_S          300U
#define CHASSIS_CONTROL_MAX_MAG_SPEED_MM_S               330U
#define CHASSIS_CONTROL_MIN_ROTATION_CDEG               1000L
#define CHASSIS_CONTROL_MAX_ROTATION_CDEG              18000L
#define CHASSIS_CONTROL_ROTATE_DECEL_COUNTS              400L
#define CHASSIS_CONTROL_ROTATE_MIN_FEEDBACK_RPM            3.0f
#define CHASSIS_CONTROL_ROTATE_CONFIRM_TICKS              20U
/* 万向轮换向会延迟一侧车轮起转；延长准入窗口但不放宽双轮方向证据。 */
#define CHASSIS_CONTROL_ROTATE_CONFIRM_TIMEOUT_TICKS     400U

#define CHASSIS_CONTROL_ALIGN_RPM                         12.0f
#define CHASSIS_CONTROL_LOW_SPEED_RUN_RPM                 25.0f
#define CHASSIS_CONTROL_MAX_RUN_RPM                       75.0f
#define CHASSIS_CONTROL_MIN_TRACK_WHEEL_RPM                5.0f
#define CHASSIS_CONTROL_MIN_CURVE_WHEEL_RPM                2.0f
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
#define CHASSIS_HEADING_FUSION_TAU_S                         2.0f
#define CHASSIS_HEADING_ENCODER_RATE_LIMIT_DPS             120.0f
#define CHASSIS_HEADING_DISAGREEMENT_LIMIT_DEG              15.0f
#define CHASSIS_HEADING_DISAGREEMENT_TICKS                  50U

#define CHASSIS_MAG_INVALID_STOP_TICKS                         10U

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
#define CHASSIS_FAULT_BUMPER                            0x0008U
#define CHASSIS_FAULT_COMM_TIMEOUT                      0x0010U
#define CHASSIS_FAULT_CAN_BUSOFF                        0x0020U
#define CHASSIS_FAULT_MAGNETIC                          0x0040U

/*
 * 实车落地验证：底层LEFT/RIGHT通道与车辆物理左右一致，但安装方向不同。
 * 底层左轮负值、右轮正值分别对应车体前进；两侧前进时原始编码器都递减。
 * 底盘层统一使用“正值=车体前进”，避免运动学、里程计和诊断各自翻转符号。
 */
#define CHASSIS_LEFT_MOTOR                              MOTOR_LEFT
#define CHASSIS_RIGHT_MOTOR                             MOTOR_RIGHT
#define CHASSIS_LEFT_COMMAND_SIGN                        -1.0f
#define CHASSIS_RIGHT_COMMAND_SIGN                        1.0f
#define CHASSIS_LEFT_ENCODER_SIGN                         -1L
#define CHASSIS_RIGHT_ENCODER_SIGN                        -1L

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
  CHASSIS_STATE_BUMPER_ERROR,
  CHASSIS_STATE_COMM_ERROR,
  CHASSIS_STATE_CAN_ERROR,
  CHASSIS_STATE_IMU_ERROR,
  CHASSIS_STATE_MOTION_ERROR
} chassis_state_t;

static chassis_state_t chassis_state;
static uint16_t chassis_fault;
static uint32_t chassis_task_sequence;
static uint32_t chassis_task_session;
static motion_command_source_t chassis_task_source;
static motion_command_type_t chassis_task_type;
static uint8_t chassis_task_active;
static uint8_t chassis_motion_enabled;
static uint16_t chassis_state_ticks;
static uint16_t chassis_recover_stable_ticks;
static uint16_t chassis_caster_stable_ticks;
static uint16_t chassis_no_progress_ticks;
static uint32_t chassis_task_ticks;
static uint32_t chassis_task_timeout_ticks;
static uint8_t chassis_speed_ticks;
static uint8_t chassis_telemetry_ticks;
static uint8_t chassis_can_fast_ticks;
static uint8_t chassis_can_odom_ticks;
static uint8_t chassis_can_safety_ticks;
static uint32_t chassis_uptime_ms;

static int32_t chassis_task_left_start_count;
static int32_t chassis_task_right_start_count;
static int32_t chassis_left_delta;
static int32_t chassis_right_delta;
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
static int32_t chassis_target_yaw_cdeg;
static int8_t chassis_translation_direction;
static int8_t chassis_rotation_direction;
static uint8_t chassis_rotation_task;
static uint8_t chassis_rotation_confirmed;
static uint8_t chassis_magnetic_follow_task;
static uint8_t chassis_magnetic_sample_valid;
static uint16_t chassis_rotation_confirm_ticks;
static uint16_t chassis_heading_disagreement_ticks;
static uint16_t chassis_magnetic_invalid_ticks;
static int32_t chassis_heading_last_left_count;
static int32_t chassis_heading_last_right_count;
static chassis_heading_fusion_t chassis_heading_fusion;
static magnetic_follow_controller_t chassis_magnetic_controller;

static float chassis_heading_target_deg;
static float chassis_heading_error_deg;
static float chassis_heading_correction_rpm;
static float chassis_requested_run_rpm;
static float chassis_base_rpm;
static float chassis_left_target_rpm;
static float chassis_right_target_rpm;
static float chassis_pose_x_mm;
static float chassis_pose_y_mm;
static float chassis_path_distance_mm;
static uint32_t chassis_last_cycle_count;

static void chassis_send_safety_can(uint8_t urgent);
static uint8_t chassis_is_driving(void);
static void chassis_fail_task(uint16_t fault);

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

static int16_t chassis_to_i16(int32_t value)
{
  if(value > 32767L)
  {
    return 32767;
  }
  if(value < -32768L)
  {
    return -32768;
  }
  return (int16_t)value;
}

static uint8_t chassis_to_u8(uint32_t value)
{
  return (value > 255UL) ? 255U : (uint8_t)value;
}

static void chassis_convert_magnetic_sample(
    const canfd_magnetic_sample_t *source,
    magnetic_follow_sample_t *destination)
{
  uint8_t index;

  memset(destination, 0, sizeof(*destination));
  destination->receive_time_ms = source->receive_time_ms;
  destination->sequence = source->sequence;
  destination->age_ms = source->age_ms;
  destination->active_mask = source->active_mask;
  destination->offset_x10mm = source->offset_x10mm;
  destination->strength = source->strength;
  destination->status = source->status;
  destination->flags = source->flags;
  destination->protocol_valid = source->protocol_valid;
  for(index = 0U; index < 8U; index++)
  {
    destination->signal[index] = source->signal[index];
  }
}

static uint8_t chassis_magnetic_sample_trackable(
    canfd_magnetic_sample_t *sample,
    uint8_t allow_multi_peak)
{
  magnetic_follow_sample_t control_sample;

  if(sample == NULL)
  {
    return 0U;
  }
  memset(sample, 0, sizeof(*sample));
  if(canfd_protocol_get_magnetic_sample(sample) == 0U)
  {
    return 0U;
  }
  chassis_convert_magnetic_sample(sample, &control_sample);
  return magnetic_follow_sample_is_trackable_for_mode(&control_sample,
                                                       allow_multi_peak);
}

static uint8_t chassis_update_magnetic_follow(float dt_s)
{
  canfd_magnetic_sample_t sample;
  magnetic_follow_sample_t control_sample;

  if(chassis_magnetic_follow_task == 0U)
  {
    return 1U;
  }
  if(chassis_magnetic_sample_trackable(
         &sample, chassis_magnetic_controller.allow_multi_peak) == 0U)
  {
    chassis_magnetic_sample_valid = 0U;
    if(chassis_magnetic_invalid_ticks < CHASSIS_MAG_INVALID_STOP_TICKS)
    {
      chassis_magnetic_invalid_ticks++;
    }
    if(chassis_magnetic_invalid_ticks >= CHASSIS_MAG_INVALID_STOP_TICKS)
    {
      debug_uart_log_printf(
          "MAG_FAULT,age=%u,status=%u,flags=0x%02X,strength=%u,offset=%d\r\n",
          (uint32_t)sample.age_ms,
          (uint32_t)sample.status,
          (uint32_t)sample.flags,
          (uint32_t)sample.strength,
          (int)sample.offset_x10mm);
      chassis_fail_task(CHASSIS_FAULT_MAGNETIC);
    }
    return 0U;
  }

  chassis_magnetic_invalid_ticks = 0U;
  chassis_magnetic_sample_valid = 1U;
  chassis_convert_magnetic_sample(&sample, &control_sample);
  magnetic_follow_controller_update(&chassis_magnetic_controller,
                                      &control_sample,
                                      chassis_requested_run_rpm);

  /*
   * 路径航向只用于诊断和后续控制阶段衔接；磁循迹轮差由目标角速度直接生成，
   * 不等待累计航向误差，避免回到磁条中心后仍保持旧转向。
   */
  chassis_heading_target_deg = chassis_wrap_deg(
      chassis_heading_target_deg +
      (chassis_magnetic_controller.yaw_rate_target_dps * dt_s));
  return 1U;
}

static int32_t chassis_rotation_target_counts(int32_t yaw_cdeg)
{
  float wheel_arc_mm = (CHASSIS_CONTROL_PI *
                        CHASSIS_CONTROL_DRIVE_TRACK_MM *
                        (float)chassis_abs_count(yaw_cdeg)) / 36000.0f;

  return chassis_round_to_int(
      (wheel_arc_mm * (float)CHASSIS_CONTROL_COUNTS_PER_REV) /
      CHASSIS_CONTROL_WHEEL_CIRCUMFERENCE_MM);
}

static float chassis_get_heading_deg(void)
{
  /*
   * 循磁时磁条提供绝对横向基准，BMI270 Z轴承担角速度内环并积分路径航向。
   * 编码器Yaw不参与循磁，避免缺脉冲把纠偏方向拖偏。
   */
  if(chassis_magnetic_follow_task != 0U)
  {
    return chassis_heading_fusion.gyro_yaw_deg;
  }
  return chassis_heading_fusion.fused_yaw_deg;
}

static float chassis_get_translation_remaining_mm(void)
{
  if(chassis_magnetic_follow_task != 0U)
  {
    return (float)chassis_abs_count(chassis_target_distance_mm) -
           chassis_path_distance_mm;
  }
  return ((float)chassis_target_distance_mm - chassis_pose_x_mm) *
         (float)chassis_translation_direction;
}

static int32_t chassis_get_left_encoder_count(void)
{
  return motor_encoder_get_count(CHASSIS_LEFT_MOTOR) *
         CHASSIS_LEFT_ENCODER_SIGN;
}

static int32_t chassis_get_right_encoder_count(void)
{
  return motor_encoder_get_count(CHASSIS_RIGHT_MOTOR) *
         CHASSIS_RIGHT_ENCODER_SIGN;
}

static float chassis_get_left_measured_rpm(void)
{
  return motor_speed_control_get_measured_rpm(CHASSIS_LEFT_MOTOR) *
         CHASSIS_LEFT_COMMAND_SIGN;
}

static float chassis_get_right_measured_rpm(void)
{
  return motor_speed_control_get_measured_rpm(CHASSIS_RIGHT_MOTOR) *
         CHASSIS_RIGHT_COMMAND_SIGN;
}

static int16_t chassis_get_left_pwm(void)
{
  return (int16_t)((float)motor_speed_control_get_pwm(CHASSIS_LEFT_MOTOR) *
                   CHASSIS_LEFT_COMMAND_SIGN);
}

static int16_t chassis_get_right_pwm(void)
{
  return (int16_t)((float)motor_speed_control_get_pwm(CHASSIS_RIGHT_MOTOR) *
                   CHASSIS_RIGHT_COMMAND_SIGN);
}

static void chassis_set_physical_targets(float left_rpm, float right_rpm)
{
  motor_speed_control_set_targets(left_rpm * CHASSIS_LEFT_COMMAND_SIGN,
                                  right_rpm * CHASSIS_RIGHT_COMMAND_SIGN);
}

static void chassis_sync_heading_encoder(void)
{
  chassis_heading_last_left_count = chassis_get_left_encoder_count();
  chassis_heading_last_right_count = chassis_get_right_encoder_count();
}

static void chassis_reset_heading_estimator(void)
{
  imu_heading_reset();
  chassis_heading_fusion_reset(&chassis_heading_fusion);
  chassis_sync_heading_encoder();
  chassis_heading_disagreement_ticks = 0U;
}

static void chassis_update_heading_estimator(float dt_s)
{
  int32_t current_left_count = chassis_get_left_encoder_count();
  int32_t current_right_count = chassis_get_right_encoder_count();
  int32_t left_delta = current_left_count - chassis_heading_last_left_count;
  int32_t right_delta = current_right_count - chassis_heading_last_right_count;
  float encoder_delta_yaw_deg;

  chassis_heading_last_left_count = current_left_count;
  chassis_heading_last_right_count = current_right_count;
  encoder_delta_yaw_deg = chassis_heading_fusion_encoder_delta_deg(
      left_delta,
      right_delta,
      CHASSIS_CONTROL_WHEEL_CIRCUMFERENCE_MM,
      (float)CHASSIS_CONTROL_COUNTS_PER_REV,
      CHASSIS_CONTROL_DRIVE_TRACK_MM);
  chassis_heading_fusion_update(&chassis_heading_fusion,
                                imu_heading_get_rate_dps(),
                                encoder_delta_yaw_deg,
                                dt_s);
}

static void chassis_check_heading_consistency(void)
{
  if((chassis_task_active == 0U) ||
     (chassis_rotation_task != 0U) ||
     (chassis_magnetic_follow_task != 0U) ||
     (chassis_state == CHASSIS_STATE_CASTER_ALIGN) ||
     (chassis_state == CHASSIS_STATE_HEADING_RECOVER) ||
     (chassis_is_driving() == 0U))
  {
    chassis_heading_disagreement_ticks = 0U;
    return;
  }

  if(chassis_abs_float(chassis_heading_fusion.disagreement_deg) >
     CHASSIS_HEADING_DISAGREEMENT_LIMIT_DEG)
  {
    if(chassis_heading_disagreement_ticks <
       CHASSIS_HEADING_DISAGREEMENT_TICKS)
    {
      chassis_heading_disagreement_ticks++;
    }
    if(chassis_heading_disagreement_ticks >=
       CHASSIS_HEADING_DISAGREEMENT_TICKS)
    {
      debug_uart_log_printf(
          "HEADING_DISAGREE,gyro=%ld,encoder=%ld,fused=%ld\r\n",
          chassis_round_to_int(chassis_heading_fusion.gyro_yaw_deg * 100.0f),
          chassis_round_to_int(
              chassis_heading_fusion.encoder_yaw_deg * 100.0f),
          chassis_round_to_int(chassis_heading_fusion.fused_yaw_deg * 100.0f));
      chassis_fail_task(CHASSIS_FAULT_MOTION);
    }
  }
  else
  {
    chassis_heading_disagreement_ticks = 0U;
  }
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
    case CHASSIS_STATE_BUMPER_ERROR:
      return "BUMPER_ERROR";
    case CHASSIS_STATE_COMM_ERROR:
      return "COMM_ERROR";
    case CHASSIS_STATE_CAN_ERROR:
      return "CAN_ERROR";
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
  chassis_task_left_start_count = chassis_get_left_encoder_count();
  chassis_task_right_start_count = chassis_get_right_encoder_count();
  chassis_left_progress = 0L;
  chassis_right_progress = 0L;
  chassis_left_delta = 0L;
  chassis_right_delta = 0L;
  chassis_center_progress = 0L;
  chassis_last_center_progress = 0L;
  chassis_last_pose_progress = 0L;
  chassis_no_progress_ticks = 0U;
  chassis_task_ticks = 0U;
}

static void chassis_reset_state_progress(void)
{
  chassis_state_left_start_count = chassis_get_left_encoder_count();
  chassis_state_right_start_count = chassis_get_right_encoder_count();
  chassis_state_center_progress = 0L;
}

static void chassis_update_progress(void)
{
  int32_t current_left_count = chassis_get_left_encoder_count();
  int32_t current_right_count = chassis_get_right_encoder_count();
  int32_t state_left_progress;
  int32_t state_right_progress;

  chassis_left_delta = current_left_count - chassis_task_left_start_count;
  chassis_right_delta = current_right_count - chassis_task_right_start_count;
  if(chassis_rotation_task != 0U)
  {
    chassis_left_progress = (chassis_rotation_direction > 0) ?
                            -chassis_left_delta : chassis_left_delta;
    chassis_right_progress = (chassis_rotation_direction > 0) ?
                             chassis_right_delta : -chassis_right_delta;
    if(chassis_left_progress < 0L)
    {
      chassis_left_progress = 0L;
    }
    if(chassis_right_progress < 0L)
    {
      chassis_right_progress = 0L;
    }
  }
  else
  {
    chassis_left_progress = chassis_abs_count(chassis_left_delta);
    chassis_right_progress = chassis_abs_count(chassis_right_delta);
  }
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
  if(chassis_rotation_task != 0U)
  {
    return;
  }
  delta_distance_mm = ((float)delta_counts *
                       CHASSIS_CONTROL_WHEEL_CIRCUMFERENCE_MM) /
                      (float)CHASSIS_CONTROL_COUNTS_PER_REV;
  /* L/S形循磁任务按实际轮迹弧长完成，不能用初始X方向投影代替路程。 */
  chassis_path_distance_mm += delta_distance_mm;
  delta_distance_mm *= (float)chassis_translation_direction;
  relative_yaw_deg = chassis_wrap_deg(chassis_get_heading_deg() -
                                      chassis_heading_target_deg);
  relative_yaw_rad = relative_yaw_deg * 0.01745329252f;
  chassis_pose_x_mm += delta_distance_mm * cosf(relative_yaw_rad);
  chassis_pose_y_mm += delta_distance_mm * sinf(relative_yaw_rad);
}

static void chassis_send_task_status(const char *serial_status,
                                     canfd_ack_status_t canfd_status,
                                     motion_command_error_t error,
                                     uint16_t code)
{
  if(chassis_task_source == MOTION_SOURCE_SERIAL)
  {
    debug_uart_log_printf("ACK,%u,%s,%s,%u\r\n",
                          (uint32_t)chassis_task_sequence,
                          motion_command_get_name(chassis_task_type),
                          serial_status,
                          (uint32_t)code);
  }
  else if(chassis_task_source == MOTION_SOURCE_CANFD)
  {
    canfd_protocol_send_command_event(chassis_task_sequence,
                                      chassis_task_session,
                                      chassis_task_type,
                                      canfd_status,
                                      error,
                                      chassis_fault,
                                      (uint8_t)chassis_state);
  }
}

static void chassis_cancel_active_task(void)
{
  if(chassis_task_active != 0U)
  {
    chassis_send_task_status("CANCELED", CANFD_ACK_CANCELED,
                             MOTION_COMMAND_ERROR_NONE,
                             CHASSIS_FAULT_NONE);
    chassis_task_active = 0U;
    chassis_magnetic_follow_task = 0U;
    chassis_magnetic_sample_valid = 0U;
    magnetic_follow_controller_reset(&chassis_magnetic_controller);
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
          (next_state == CHASSIS_STATE_BUMPER_ERROR) ||
          (next_state == CHASSIS_STATE_COMM_ERROR) ||
          (next_state == CHASSIS_STATE_CAN_ERROR) ||
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
    chassis_send_task_status("COMPLETED", CANFD_ACK_COMPLETED,
                             MOTION_COMMAND_ERROR_NONE,
                             CHASSIS_FAULT_NONE);
    chassis_task_active = 0U;
    chassis_magnetic_follow_task = 0U;
    chassis_magnetic_sample_valid = 0U;
    magnetic_follow_controller_reset(&chassis_magnetic_controller);
  }
}

static void chassis_fail_task(uint16_t fault)
{
  chassis_fault |= fault;
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
    chassis_send_task_status("FAULT", CANFD_ACK_FAULT,
                             MOTION_COMMAND_ERROR_FAULT_LOCKED,
                             fault);
    chassis_task_active = 0U;
  }
  chassis_magnetic_follow_task = 0U;
  chassis_magnetic_sample_valid = 0U;
  magnetic_follow_controller_reset(&chassis_magnetic_controller);
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
      chassis_round_to_int(chassis_get_left_measured_rpm() * 100.0f),
      chassis_round_to_int(chassis_get_right_measured_rpm() * 100.0f),
      chassis_get_left_pwm(),
      chassis_get_right_pwm(),
      motor_encoder_get_invalid_transition_count(CHASSIS_LEFT_MOTOR),
      motor_encoder_get_invalid_transition_count(CHASSIS_RIGHT_MOTOR));
  chassis_fail_task(CHASSIS_FAULT_MOTION);
}

static float chassis_get_heading_limit(void)
{
  if(chassis_magnetic_follow_task != 0U)
  {
    return magnetic_follow_controller_correction_limit_rpm(
        (chassis_state == CHASSIS_STATE_CASTER_ALIGN) ? 1U : 0U,
        chassis_magnetic_controller.allow_multi_peak);
  }
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

/* 平移时两个主动轮保持同向，以差速修正航向；正负号由任务方向统一决定。 */
static void chassis_update_heading_targets(void)
{
  float yaw_deg = chassis_get_heading_deg();
  float yaw_rate_dps = imu_heading_get_rate_dps();
  float correction_limit = chassis_get_heading_limit();
  float effective_base_rpm;
  float minimum_wheel_rpm;
  float same_direction_correction_limit_rpm;
  float signed_base_rpm;
  float target_limit_rpm;

  if(chassis_rotation_task != 0U)
  {
    chassis_heading_error_deg = 0.0f;
    chassis_heading_correction_rpm = 0.0f;
    chassis_left_target_rpm =
        (chassis_rotation_direction > 0) ? -chassis_base_rpm :
                                           chassis_base_rpm;
    chassis_right_target_rpm =
        (chassis_rotation_direction > 0) ? chassis_base_rpm :
                                          -chassis_base_rpm;
    chassis_set_physical_targets(chassis_left_target_rpm,
                                 chassis_right_target_rpm);
    return;
  }

  if((chassis_magnetic_follow_task != 0U) &&
     (chassis_magnetic_sample_valid == 0U))
  {
    chassis_left_target_rpm = 0.0f;
    chassis_right_target_rpm = 0.0f;
    motor_speed_control_hold_brake();
    return;
  }

  if(chassis_magnetic_follow_task != 0U)
  {
    /*
     * 磁偏移外环直接给Yaw角速度；按差速运动学换算轮速前馈，再由BMI270
     * Z轴角速度闭环补偿。回中时目标角速度立即归零，不保留累计航向滞后。
     */
    chassis_heading_error_deg = 0.0f;
    chassis_heading_correction_rpm =
        magnetic_follow_controller_wheel_correction_rpm(
            &chassis_magnetic_controller,
            yaw_rate_dps,
            CHASSIS_CONTROL_DRIVE_TRACK_MM,
            CHASSIS_CONTROL_WHEEL_CIRCUMFERENCE_MM);
  }
  else
  {
    chassis_heading_error_deg = chassis_wrap_deg(chassis_heading_target_deg -
                                                 yaw_deg);
    chassis_heading_correction_rpm =
        (CHASSIS_HEADING_KP_RPM_PER_DEG * chassis_heading_error_deg) -
        (CHASSIS_HEADING_KD_RPM_PER_DPS * yaw_rate_dps);
  }
  chassis_heading_correction_rpm *= CHASSIS_HEADING_CORRECTION_SIGN;
  chassis_heading_correction_rpm = chassis_limit(
      chassis_heading_correction_rpm, -correction_limit, correction_limit);

  effective_base_rpm = chassis_base_rpm;
  if((chassis_magnetic_follow_task != 0U) &&
     (effective_base_rpm > chassis_magnetic_controller.speed_limit_rpm))
  {
    effective_base_rpm = chassis_magnetic_controller.speed_limit_rpm;
  }
  minimum_wheel_rpm =
      ((chassis_magnetic_follow_task != 0U) &&
       (chassis_magnetic_controller.allow_multi_peak != 0U)) ?
      CHASSIS_CONTROL_MIN_CURVE_WHEEL_RPM :
      CHASSIS_CONTROL_MIN_TRACK_WHEEL_RPM;
  same_direction_correction_limit_rpm = effective_base_rpm -
                                        minimum_wheel_rpm;
  if(same_direction_correction_limit_rpm < 0.0f)
  {
    same_direction_correction_limit_rpm = 0.0f;
  }
  chassis_heading_correction_rpm = chassis_limit(
      chassis_heading_correction_rpm,
      -same_direction_correction_limit_rpm,
      same_direction_correction_limit_rpm);
  signed_base_rpm = effective_base_rpm *
                     (float)chassis_translation_direction;
  chassis_left_target_rpm = signed_base_rpm -
                            chassis_heading_correction_rpm;
  chassis_right_target_rpm = signed_base_rpm +
                             chassis_heading_correction_rpm;
  target_limit_rpm = (chassis_magnetic_follow_task != 0U) ?
                     CHASSIS_CONTROL_MAX_RUN_RPM :
                     (CHASSIS_CONTROL_MAX_RUN_RPM + correction_limit);
  if(chassis_translation_direction > 0)
  {
    chassis_left_target_rpm = chassis_limit(
        chassis_left_target_rpm, minimum_wheel_rpm,
        target_limit_rpm);
    chassis_right_target_rpm = chassis_limit(
        chassis_right_target_rpm, minimum_wheel_rpm,
        target_limit_rpm);
  }
  else
  {
    chassis_left_target_rpm = chassis_limit(
        chassis_left_target_rpm,
        -target_limit_rpm, -minimum_wheel_rpm);
    chassis_right_target_rpm = chassis_limit(
        chassis_right_target_rpm,
        -target_limit_rpm, -minimum_wheel_rpm);
  }
  chassis_set_physical_targets(chassis_left_target_rpm,
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
  if(chassis_rotation_task != 0U)
  {
    /* 原地越过地面凸起会产生大轮差；旋转改由单轮脱困失败直接判故障。 */
    return 0U;
  }
  /* 循磁会持续差速转向，累计轮差是正常控制结果，不能按纯直行失配处理。 */
  if(chassis_magnetic_follow_task != 0U)
  {
    return 0U;
  }
  return ((maximum_progress >= CHASSIS_CONTROL_ENCODER_CHECK_MIN_COUNTS) &&
          (progress_difference >=
           CHASSIS_CONTROL_ENCODER_MISMATCH_COUNTS)) ? 1U : 0U;
}

/* 没有万向轮角度传感器，只能用车轮速度和Yaw稳定间接判断摆正。 */
static uint8_t chassis_caster_motion_stable(void)
{
  float left_rpm = chassis_get_left_measured_rpm() *
                   (float)chassis_translation_direction;
  float right_rpm = chassis_get_right_measured_rpm() *
                    (float)chassis_translation_direction;
  float speed_difference = chassis_abs_float(left_rpm - right_rpm);

  return ((left_rpm >= 5.0f) &&
          (right_rpm >= 5.0f) &&
          (speed_difference <= CHASSIS_CONTROL_CASTER_SPEED_DIFF_RPM) &&
          (chassis_abs_float(imu_heading_get_rate_dps()) <=
           CHASSIS_CONTROL_CASTER_STABLE_RATE_DPS)) ? 1U : 0U;
}

static float chassis_get_deceleration_distance_mm(void)
{
  float high_speed_ratio;

  high_speed_ratio =
      (chassis_requested_run_rpm - CHASSIS_CONTROL_LOW_SPEED_RUN_RPM) /
      (CHASSIS_CONTROL_MAX_RUN_RPM - CHASSIS_CONTROL_LOW_SPEED_RUN_RPM);
  high_speed_ratio = chassis_limit(high_speed_ratio, 0.0f, 1.0f);
  return CHASSIS_CONTROL_DECEL_DISTANCE_MM +
      ((CHASSIS_CONTROL_HIGH_SPEED_DECEL_DISTANCE_MM -
        CHASSIS_CONTROL_DECEL_DISTANCE_MM) * high_speed_ratio);
}

static void chassis_update_deceleration(void)
{
  float remaining_distance_mm = chassis_get_translation_remaining_mm();
  float remaining_ratio;
  float decel_end_rpm = chassis_limit(CHASSIS_CONTROL_DECEL_END_RPM,
                                      5.0f,
                                      chassis_requested_run_rpm);

  if(chassis_rotation_task != 0U)
  {
    int32_t remaining_counts = chassis_total_counts -
                               chassis_center_progress;

    if(remaining_counts < 0L)
    {
      remaining_counts = 0L;
    }
    remaining_ratio = (float)remaining_counts /
                      (float)CHASSIS_CONTROL_ROTATE_DECEL_COUNTS;
    remaining_ratio = chassis_limit(remaining_ratio, 0.0f, 1.0f);
    chassis_base_rpm = decel_end_rpm +
        ((chassis_requested_run_rpm - decel_end_rpm) * remaining_ratio);
    return;
  }

  if(remaining_distance_mm < 0.0f)
  {
    remaining_distance_mm = 0.0f;
  }
  remaining_ratio = remaining_distance_mm /
                    chassis_get_deceleration_distance_mm();
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
  float left_rpm;
  float right_rpm;
  uint8_t rotation_feedback_valid;

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

  if(chassis_rotation_task != 0U)
  {
    left_rpm = chassis_get_left_measured_rpm();
    right_rpm = chassis_get_right_measured_rpm();
    rotation_feedback_valid =
        (chassis_rotation_direction > 0) ?
        (((left_rpm <= -CHASSIS_CONTROL_ROTATE_MIN_FEEDBACK_RPM) &&
          (right_rpm >= CHASSIS_CONTROL_ROTATE_MIN_FEEDBACK_RPM)) ? 1U : 0U) :
        (((left_rpm >= CHASSIS_CONTROL_ROTATE_MIN_FEEDBACK_RPM) &&
          (right_rpm <= -CHASSIS_CONTROL_ROTATE_MIN_FEEDBACK_RPM)) ? 1U : 0U);
    if(rotation_feedback_valid != 0U)
    {
      if(chassis_rotation_confirm_ticks <
         CHASSIS_CONTROL_ROTATE_CONFIRM_TICKS)
      {
        chassis_rotation_confirm_ticks++;
      }
    }
    else
    {
      chassis_rotation_confirm_ticks = 0U;
    }
    if(chassis_rotation_confirm_ticks >=
       CHASSIS_CONTROL_ROTATE_CONFIRM_TICKS)
    {
      if(chassis_rotation_confirmed == 0U)
      {
        chassis_rotation_confirmed = 1U;
      }
    }
    if((chassis_rotation_confirmed == 0U) &&
       (chassis_task_ticks >=
        CHASSIS_CONTROL_ROTATE_CONFIRM_TIMEOUT_TICKS))
    {
      chassis_fail_motion("ROTATION_NOT_MOVING");
      return;
    }
  }

  if(chassis_no_progress_ticks >=
     CHASSIS_CONTROL_NO_PROGRESS_TIMEOUT_TICKS)
  {
    chassis_fail_motion("NO_PROGRESS");
    return;
  }
  if((chassis_rotation_task != 0U) &&
     (motor_speed_control_has_stall_recovery_failed() != 0U))
  {
    chassis_fail_motion("STALL_RECOVERY_FAILED");
    return;
  }
  if(chassis_task_ticks >= chassis_task_timeout_ticks)
  {
    chassis_fail_motion("TASK_TIMEOUT");
    return;
  }
  if((chassis_rotation_task != 0U) &&
     (chassis_rotation_confirmed != 0U) &&
     (chassis_abs_float(
          chassis_heading_fusion.fused_yaw_unwrapped_deg) >=
      ((float)chassis_abs_count(chassis_target_yaw_cdeg) / 100.0f)))
  {
    chassis_complete_task();
    return;
  }
  if((chassis_rotation_task == 0U) &&
     (chassis_get_translation_remaining_mm() <= 0.0f))
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
        /*
         * 循磁目标航向会随横向偏差逐帧变化，不能套用固定航向的稳定等待，
         * 否则即使偏差持续收敛也会触发HEADING_RECOVER_TIMEOUT。
         */
        chassis_enter_state((chassis_magnetic_follow_task != 0U) ?
                            CHASSIS_STATE_SPEED_RAMP :
                            CHASSIS_STATE_HEADING_RECOVER);
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
    uint8_t rotation_decelerates =
        ((chassis_rotation_task != 0U) &&
         (chassis_center_progress >=
          (chassis_total_counts - CHASSIS_CONTROL_ROTATE_DECEL_COUNTS))) ?
        1U : 0U;

    if((rotation_decelerates != 0U) ||
       ((chassis_rotation_task == 0U) &&
        (chassis_get_translation_remaining_mm() <=
         chassis_get_deceleration_distance_mm())))
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
  chassis_path_distance_mm = 0.0f;
  chassis_last_pose_progress = 0L;
}

motion_command_handle_result_t chassis_motion_controller_handle_command(
    const motion_command_t *command)
{
  const motion_goto_local_data_t *goto_data;
  const motion_follow_mag_data_t *follow_data;
  motion_goto_local_data_t follow_as_goto;
  canfd_magnetic_sample_t magnetic_sample;
  magnetic_follow_sample_t magnetic_control_sample;
  int32_t translation_limit_mm;
  uint8_t translation_task;
  uint8_t rotation_task;

  if(command == NULL)
  {
    return MOTION_HANDLE_REJECTED;
  }

  if(command->type == MOTION_CMD_ENABLE)
  {
    if((chassis_task_active != 0U) ||
       (chassis_fault != CHASSIS_FAULT_NONE) ||
       (imu_heading_get_status() != BMI2_OK) ||
       (bumper_safety_can_clear() == 0U))
    {
      return MOTION_HANDLE_REJECTED;
    }
    chassis_motion_enabled = 1U;
    chassis_enter_state(CHASSIS_STATE_IDLE);
    return MOTION_HANDLE_ACCEPTED;
  }

  if(command->type == MOTION_CMD_DISABLE)
  {
    chassis_cancel_active_task();
    chassis_motion_enabled = 0U;
    chassis_enter_state(CHASSIS_STATE_IDLE);
    return MOTION_HANDLE_ACCEPTED;
  }

  if((command->type == MOTION_CMD_GOTO_LOCAL) ||
     (command->type == MOTION_CMD_FOLLOW_MAG))
  {
    if(command->type == MOTION_CMD_FOLLOW_MAG)
    {
      follow_data = &command->data.follow_mag;
      memset(&follow_as_goto, 0, sizeof(follow_as_goto));
      follow_as_goto.x_mm = follow_data->distance_mm;
      follow_as_goto.speed_mm_s = follow_data->speed_mm_s;
      follow_as_goto.timeout_ms = follow_data->timeout_ms;
      goto_data = &follow_as_goto;
    }
    else
    {
      follow_data = NULL;
      goto_data = &command->data.goto_local;
    }
    translation_limit_mm = (follow_data != NULL) ?
                           CHASSIS_CONTROL_MAX_MAG_DISTANCE_MM :
                           CHASSIS_CONTROL_MAX_DISTANCE_MM;
    translation_task =
        ((((goto_data->x_mm >= CHASSIS_CONTROL_MIN_DISTANCE_MM) &&
           (goto_data->x_mm <= translation_limit_mm)) ||
          ((goto_data->x_mm <= -CHASSIS_CONTROL_MIN_DISTANCE_MM) &&
           (goto_data->x_mm >= -translation_limit_mm))) &&
         (goto_data->y_mm == 0L) &&
         (goto_data->yaw_cdeg == 0L)) ? 1U : 0U;
    rotation_task =
        ((goto_data->x_mm == 0L) &&
         (goto_data->y_mm == 0L) &&
         (chassis_abs_count(goto_data->yaw_cdeg) >=
          CHASSIS_CONTROL_MIN_ROTATION_CDEG) &&
         (chassis_abs_count(goto_data->yaw_cdeg) <=
          CHASSIS_CONTROL_MAX_ROTATION_CDEG)) ? 1U : 0U;
    if(chassis_task_active != 0U)
    {
      return MOTION_HANDLE_BUSY;
    }
    if((command->source == MOTION_SOURCE_CANFD) &&
       (chassis_motion_enabled == 0U))
    {
      return MOTION_HANDLE_REJECTED;
    }
    if((chassis_fault != CHASSIS_FAULT_NONE) ||
       (imu_heading_get_status() != BMI2_OK))
    {
      debug_uart_log_printf("TASK_REJECT,%u,FAULT_STATE,%u\r\n",
                            (uint32_t)command->sequence,
                            (uint32_t)chassis_fault);
      return MOTION_HANDLE_REJECTED;
    }
    if(((translation_task == 0U) && (rotation_task == 0U)) ||
       (goto_data->speed_mm_s < CHASSIS_CONTROL_MIN_SPEED_MM_S) ||
        (goto_data->speed_mm_s >
         ((follow_data != NULL) ? CHASSIS_CONTROL_MAX_MAG_SPEED_MM_S :
          ((rotation_task != 0U) ?
           CHASSIS_CONTROL_MAX_ROTATION_SPEED_MM_S :
           CHASSIS_CONTROL_MAX_SPEED_MM_S))))
    {
      debug_uart_log_printf(
          "TASK_REJECT,%u,POSE_LIMIT,x=%ld,y=%ld,yaw=%ld,speed=%u\r\n",
          (uint32_t)command->sequence,
          goto_data->x_mm,
          goto_data->y_mm,
          goto_data->yaw_cdeg,
          (uint32_t)goto_data->speed_mm_s);
      return MOTION_HANDLE_REJECTED;
    }
    if((follow_data != NULL) &&
       (chassis_magnetic_sample_trackable(
            &magnetic_sample, follow_data->allow_multi_peak) == 0U))
    {
      debug_uart_log_printf(
          "TASK_REJECT,%u,MAG_NOT_TRACKABLE,status=%u,flags=0x%02X,"
          "strength=%u,offset=%d,age=%u\r\n",
          (uint32_t)command->sequence,
          (uint32_t)magnetic_sample.status,
          (uint32_t)magnetic_sample.flags,
          (uint32_t)magnetic_sample.strength,
          (int)magnetic_sample.offset_x10mm,
          (uint32_t)magnetic_sample.age_ms);
      return MOTION_HANDLE_REJECTED;
    }

    chassis_task_sequence = command->sequence;
    chassis_task_session = command->session_id;
    chassis_task_source = command->source;
    chassis_task_type = command->type;
    chassis_target_distance_mm = goto_data->x_mm;
    chassis_target_yaw_cdeg = goto_data->yaw_cdeg;
    chassis_translation_direction =
        (goto_data->x_mm < 0L) ? -1 : 1;
    chassis_rotation_task = rotation_task;
    chassis_magnetic_follow_task = (follow_data != NULL) ? 1U : 0U;
    chassis_magnetic_sample_valid = chassis_magnetic_follow_task;
    chassis_magnetic_invalid_ticks = 0U;
    magnetic_follow_controller_reset(&chassis_magnetic_controller);
    chassis_rotation_confirmed = 0U;
    chassis_rotation_confirm_ticks = 0U;
    chassis_rotation_direction =
        (goto_data->yaw_cdeg > 0L) ? 1 : -1;
    if(chassis_rotation_task != 0U)
    {
      chassis_total_counts =
          chassis_rotation_target_counts(chassis_target_yaw_cdeg);
    }
    else
    {
      chassis_total_counts = chassis_round_to_int(
          ((float)chassis_abs_count(chassis_target_distance_mm) *
           (float)CHASSIS_CONTROL_COUNTS_PER_REV) /
          CHASSIS_CONTROL_WHEEL_CIRCUMFERENCE_MM);
    }
    chassis_requested_run_rpm = chassis_limit(
        ((float)goto_data->speed_mm_s * 60.0f) /
        CHASSIS_CONTROL_WHEEL_CIRCUMFERENCE_MM,
        8.0f,
        CHASSIS_CONTROL_MAX_RUN_RPM);
    if(follow_data != NULL)
    {
      chassis_convert_magnetic_sample(&magnetic_sample,
                                      &magnetic_control_sample);
      magnetic_follow_controller_start(
          &chassis_magnetic_controller,
          follow_data->target_offset_x10mm,
          follow_data->allow_multi_peak,
          &magnetic_control_sample,
          chassis_requested_run_rpm);
    }
    if(goto_data->timeout_ms != 0UL)
    {
      chassis_task_timeout_ticks =
          (goto_data->timeout_ms + CHASSIS_CONTROL_LOOP_PERIOD_MS - 1U) /
          CHASSIS_CONTROL_LOOP_PERIOD_MS;
    }
    else
    {
      if(chassis_rotation_task != 0U)
      {
        chassis_task_timeout_ticks =
            CHASSIS_CONTROL_TASK_TIMEOUT_MIN_TICKS;
      }
      else
      {
        chassis_task_timeout_ticks =
            (((uint32_t)chassis_abs_count(chassis_target_distance_mm) * 100U) /
             (uint32_t)goto_data->speed_mm_s) +
            CHASSIS_CONTROL_TASK_TIMEOUT_MARGIN_TICKS;
      }
      if(chassis_task_timeout_ticks < CHASSIS_CONTROL_TASK_TIMEOUT_MIN_TICKS)
      {
        chassis_task_timeout_ticks = CHASSIS_CONTROL_TASK_TIMEOUT_MIN_TICKS;
      }
    }
    chassis_reset_heading_estimator();
    chassis_heading_target_deg = 0.0f;
    chassis_heading_error_deg = 0.0f;
    chassis_heading_correction_rpm = 0.0f;
    chassis_task_active = 1U;
    chassis_reset_local_pose();
    chassis_reset_task_progress();
    if(chassis_rotation_task != 0U)
    {
      motor_speed_control_resync_feedback();
      chassis_base_rpm = 8.0f;
      chassis_enter_state(CHASSIS_STATE_SPEED_RAMP);
      debug_uart_log_printf("TASK_ROTATE_START,%u,%ld,%u,%ld\r\n",
                            (uint32_t)chassis_task_sequence,
                            chassis_target_yaw_cdeg,
                            (uint32_t)goto_data->speed_mm_s,
                            chassis_total_counts);
    }
    else
    {
      chassis_enter_state(CHASSIS_STATE_CASTER_ALIGN);
      if(chassis_magnetic_follow_task != 0U)
      {
        debug_uart_log_printf(
            "TASK_MAG_START,%u,%ld,%u,%ld,target=%d,offset=%d,strength=%u\r\n",
            (uint32_t)chassis_task_sequence,
            chassis_target_distance_mm,
            (uint32_t)goto_data->speed_mm_s,
            chassis_total_counts,
            (int)chassis_magnetic_controller.target_offset_x10mm,
            (int)chassis_magnetic_controller.offset_x10mm,
            (uint32_t)chassis_magnetic_controller.strength);
      }
      else
      {
        debug_uart_log_printf("TASK_START,%u,%ld,%u,%ld\r\n",
                              (uint32_t)chassis_task_sequence,
                              chassis_target_distance_mm,
                              (uint32_t)goto_data->speed_mm_s,
                              chassis_total_counts);
      }
    }
    return MOTION_HANDLE_ACCEPTED;
  }

  if(command->type == MOTION_CMD_STOP)
  {
    chassis_cancel_active_task();
    if(chassis_fault == CHASSIS_FAULT_NONE)
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
    chassis_motion_enabled = 0U;
    chassis_fault |= CHASSIS_FAULT_ESTOP;
    chassis_enter_state(CHASSIS_STATE_ESTOP);
    chassis_send_safety_can(1U);
    return MOTION_HANDLE_ACCEPTED;
  }

  if(command->type == MOTION_CMD_COMM_TIMEOUT)
  {
    chassis_cancel_active_task();
    chassis_motion_enabled = 0U;
    chassis_fault |= CHASSIS_FAULT_COMM_TIMEOUT;
    chassis_enter_state(CHASSIS_STATE_COMM_ERROR);
    chassis_send_safety_can(1U);
    return MOTION_HANDLE_ACCEPTED;
  }

  if(command->type == MOTION_CMD_CAN_BUSOFF)
  {
    chassis_cancel_active_task();
    chassis_motion_enabled = 0U;
    chassis_fault |= CHASSIS_FAULT_CAN_BUSOFF;
    chassis_enter_state(CHASSIS_STATE_CAN_ERROR);
    chassis_send_safety_can(1U);
    return MOTION_HANDLE_ACCEPTED;
  }

  if(command->type == MOTION_CMD_RESET_ODOM)
  {
    if(chassis_task_active != 0U)
    {
      return MOTION_HANDLE_BUSY;
    }
    if((chassis_fault != CHASSIS_FAULT_NONE) ||
       (imu_heading_get_status() != BMI2_OK))
    {
      return MOTION_HANDLE_REJECTED;
    }

    motor_speed_control_hold_brake();
    motor_encoder_reset_all();
    motor_speed_control_resync_feedback();
    chassis_reset_heading_estimator();
    chassis_heading_target_deg = 0.0f;
    chassis_reset_local_pose();
    chassis_reset_task_progress();
    chassis_enter_state(CHASSIS_STATE_IDLE);
    return MOTION_HANDLE_ACCEPTED;
  }

  if(command->type == MOTION_CMD_CLEAR_FAULT)
  {
    canfd_magnetic_sample_t clear_magnetic_sample;

    if(chassis_task_active != 0U)
    {
      return MOTION_HANDLE_BUSY;
    }
    if((((chassis_fault & CHASSIS_FAULT_CAN_BUSOFF) != 0U) &&
         (canfd_protocol_is_bus_healthy() == 0U)) ||
       (((chassis_fault & CHASSIS_FAULT_MAGNETIC) != 0U) &&
         (chassis_magnetic_sample_trackable(&clear_magnetic_sample,
                                            0U) == 0U)) ||
       (imu_heading_get_status() != BMI2_OK) ||
       (bumper_safety_can_clear() == 0U))
    {
      return MOTION_HANDLE_REJECTED;
    }

    motor_speed_control_hold_brake();
    if(bumper_safety_clear_latch() == 0U)
    {
      return MOTION_HANDLE_REJECTED;
    }
    motor_speed_control_resync_feedback();
    chassis_fault = CHASSIS_FAULT_NONE;
    chassis_motion_enabled = 0U;
    chassis_magnetic_follow_task = 0U;
    chassis_magnetic_sample_valid = 0U;
    magnetic_follow_controller_reset(&chassis_magnetic_controller);
    chassis_reset_task_progress();
    chassis_enter_state(CHASSIS_STATE_IDLE);
    chassis_send_safety_can(1U);
    return MOTION_HANDLE_ACCEPTED;
  }

  /* 速度流协议已经定义，运动学和反向里程计实车验证前保持显式拒绝。 */
  if(command->type == MOTION_CMD_SET_VELOCITY)
  {
    return MOTION_HANDLE_REJECTED;
  }

  return MOTION_HANDLE_REJECTED;
}

static void chassis_send_telemetry(void)
{
  canfd_motor_status_t motor_status;
  float relative_yaw_deg = chassis_wrap_deg(chassis_get_heading_deg() -
                                            chassis_heading_target_deg);
  float left_rpm = chassis_get_left_measured_rpm();
  float right_rpm = chassis_get_right_measured_rpm();
  float linear_mm_s = ((left_rpm + right_rpm) * 0.5f *
                       CHASSIS_CONTROL_WHEEL_CIRCUMFERENCE_MM) / 60.0f;

  debug_uart_log_printf(
      "POSE,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%s,0x%04X,"
      "%ld,%ld,%d,%d,%ld,%ld,%lu,%lu,%ld,%ld,%ld,%lu\r\n",
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
      chassis_get_left_pwm(),
      chassis_get_right_pwm(),
      chassis_get_left_encoder_count(),
      chassis_get_right_encoder_count(),
      motor_encoder_get_invalid_transition_count(CHASSIS_LEFT_MOTOR),
      motor_encoder_get_invalid_transition_count(CHASSIS_RIGHT_MOTOR),
      chassis_round_to_int(chassis_heading_fusion.gyro_yaw_deg * 100.0f),
      chassis_round_to_int(chassis_heading_fusion.encoder_yaw_deg * 100.0f),
      chassis_round_to_int(chassis_heading_fusion.disagreement_deg * 100.0f),
      chassis_heading_fusion.rejected_encoder_samples);

  if(chassis_magnetic_follow_task != 0U)
  {
    debug_uart_log_printf(
        "MAGFOLLOW,offset=%d,target=%d,strength=%u,age=%u,valid=%u,"
        "heading=%ld,rate=%ld,offset_rate=%ld,speed_limit=%ld,"
        "speed_phase=%u,adaptive=%u,path=%ld\r\n",
        (int)chassis_magnetic_controller.offset_x10mm,
        (int)chassis_magnetic_controller.target_offset_x10mm,
        (uint32_t)chassis_magnetic_controller.strength,
        (uint32_t)chassis_magnetic_controller.age_ms,
        (uint32_t)chassis_magnetic_sample_valid,
        chassis_round_to_int(chassis_heading_target_deg * 100.0f),
        chassis_round_to_int(
            chassis_magnetic_controller.yaw_rate_target_dps * 100.0f),
        chassis_round_to_int(
            chassis_magnetic_controller.offset_rate_mm_s * 100.0f),
        chassis_round_to_int(
            chassis_magnetic_controller.speed_limit_rpm * 100.0f),
        (uint32_t)chassis_magnetic_controller.speed_phase,
        (uint32_t)chassis_magnetic_controller.adaptive_high_speed,
        chassis_round_to_int(chassis_path_distance_mm));
  }

  motor_status.uptime_ms = chassis_uptime_ms;
  motor_status.left_target_rpm_x100 = chassis_to_i16(
      chassis_round_to_int(chassis_left_target_rpm * 100.0f));
  motor_status.right_target_rpm_x100 = chassis_to_i16(
      chassis_round_to_int(chassis_right_target_rpm * 100.0f));
  motor_status.left_measured_rpm_x100 = chassis_to_i16(
      chassis_round_to_int(left_rpm * 100.0f));
  motor_status.right_measured_rpm_x100 = chassis_to_i16(
      chassis_round_to_int(right_rpm * 100.0f));
  motor_status.left_pwm = chassis_get_left_pwm();
  motor_status.right_pwm = chassis_get_right_pwm();
  motor_status.left_encoder_count = chassis_get_left_encoder_count();
  motor_status.right_encoder_count = chassis_get_right_encoder_count();
  motor_status.left_invalid_count = chassis_to_u8(
      motor_encoder_get_invalid_transition_count(CHASSIS_LEFT_MOTOR));
  motor_status.right_invalid_count = chassis_to_u8(
      motor_encoder_get_invalid_transition_count(CHASSIS_RIGHT_MOTOR));
  motor_status.state = (uint8_t)chassis_state;
  canfd_protocol_send_motor_status(&motor_status);
}

static uint8_t chassis_get_safety_source_mask(void)
{
  uint8_t mask = 0U;

  if((chassis_fault & CHASSIS_FAULT_BUMPER) != 0U)
  {
    mask |= 0x01U;
  }
  if((chassis_fault & CHASSIS_FAULT_ESTOP) != 0U)
  {
    mask |= 0x02U;
  }
  if((chassis_fault & (CHASSIS_FAULT_COMM_TIMEOUT |
                       CHASSIS_FAULT_CAN_BUSOFF |
                       CHASSIS_FAULT_IMU |
                       CHASSIS_FAULT_MAGNETIC |
                       CHASSIS_FAULT_MOTION)) != 0U)
  {
    mask |= 0x04U;
  }
  return mask;
}

static uint8_t chassis_get_application_state(void)
{
  if((chassis_fault & (CHASSIS_FAULT_ESTOP | CHASSIS_FAULT_BUMPER)) != 0U)
  {
    return 6U;
  }
  if(chassis_fault != CHASSIS_FAULT_NONE)
  {
    return 5U;
  }
  if(chassis_is_driving() != 0U)
  {
    return 3U;
  }
  return (chassis_motion_enabled != 0U) ? 2U : 1U;
}

static uint8_t chassis_get_motion_state(void)
{
  if((chassis_fault & (CHASSIS_FAULT_ESTOP | CHASSIS_FAULT_BUMPER)) != 0U)
  {
    return 6U;
  }
  if(chassis_fault != CHASSIS_FAULT_NONE)
  {
    return 7U;
  }
  if(chassis_is_driving() != 0U)
  {
    return 2U;
  }
  return (chassis_motion_enabled != 0U) ? 1U : 0U;
}

static void chassis_send_safety_can(uint8_t urgent)
{
  bumper_safety_snapshot_t bumper;
  canfd_safety_status_t status;

  bumper_safety_get_snapshot(&bumper);
  status.uptime_ms = chassis_uptime_ms;
  status.fault_mask = (uint32_t)chassis_fault;
  status.warning_mask = 0UL;
  status.bumper_mask = (uint8_t)(bumper.active_mask | bumper.latched_mask);
  status.estop_source_mask = chassis_get_safety_source_mask();
  if((chassis_fault & (CHASSIS_FAULT_ESTOP | CHASSIS_FAULT_BUMPER)) != 0U)
  {
    status.safety_state = 2U;
  }
  else if(chassis_fault != CHASSIS_FAULT_NONE)
  {
    status.safety_state = 3U;
  }
  else
  {
    status.safety_state = 0U;
  }
  canfd_protocol_send_safety_status(&status, urgent);
}

static void chassis_send_drive_can(void)
{
  canfd_drive_status_t status;
  float relative_yaw_deg = chassis_wrap_deg(chassis_get_heading_deg() -
                                             chassis_heading_target_deg);

  status.application_state = chassis_get_application_state();
  status.motion_state = chassis_get_motion_state();
  status.motion_mode = (chassis_task_active == 0U) ? 0U :
                       ((chassis_magnetic_follow_task != 0U) ? 2U : 1U);
  status.flags = 0x04U;
  if(canfd_protocol_is_session_valid() != 0U)
  {
    status.flags |= 0x01U;
  }
  if(chassis_motion_enabled != 0U)
  {
    status.flags |= 0x02U;
  }
  if(imu_heading_get_status() == BMI2_OK)
  {
    status.flags |= 0x08U;
  }
  status.command_id = (chassis_task_active != 0U) ?
                      chassis_task_sequence : 0UL;
  status.session_id = canfd_protocol_get_session_id();
  status.fault_mask = (uint32_t)chassis_fault;
  status.warning_mask = 0UL;
  status.x_mm = chassis_round_to_int(chassis_pose_x_mm);
  status.y_mm = chassis_round_to_int(chassis_pose_y_mm);
  status.yaw_cdeg = chassis_round_to_int(relative_yaw_deg * 100.0f);
  canfd_protocol_send_drive_status(&status);
}

static void chassis_send_odom_can(void)
{
  canfd_odom_status_t status;
  float relative_yaw_deg = chassis_wrap_deg(chassis_get_heading_deg() -
                                            chassis_heading_target_deg);
  float left_rpm = chassis_get_left_measured_rpm();
  float right_rpm = chassis_get_right_measured_rpm();
  float linear_mm_s = ((left_rpm + right_rpm) * 0.5f *
                       CHASSIS_CONTROL_WHEEL_CIRCUMFERENCE_MM) / 60.0f;

  status.uptime_ms = chassis_uptime_ms;
  status.x_mm = chassis_round_to_int(chassis_pose_x_mm);
  status.y_mm = chassis_round_to_int(chassis_pose_y_mm);
  status.yaw_cdeg = chassis_round_to_int(relative_yaw_deg * 100.0f);
  status.linear_mm_s = chassis_to_i16(chassis_round_to_int(linear_mm_s));
  status.yaw_rate_cdeg_s = chassis_to_i16(chassis_round_to_int(
      imu_heading_get_rate_dps() * 100.0f));
  status.valid_flags = (imu_heading_get_status() == BMI2_OK) ? 0x0007U :
                                                               0x0003U;
  canfd_protocol_send_odom_status(&status);
}

static void chassis_handle_bumper_safety(void)
{
  bumper_safety_snapshot_t bumper;
  uint8_t changed = bumper_safety_step_10ms();

  bumper_safety_get_snapshot(&bumper);
  if(bumper.latched_mask != 0U)
  {
    if((chassis_fault & CHASSIS_FAULT_BUMPER) == 0U)
    {
      chassis_cancel_active_task();
      chassis_motion_enabled = 0U;
      chassis_fault |= CHASSIS_FAULT_BUMPER;
      chassis_enter_state(CHASSIS_STATE_BUMPER_ERROR);
      debug_uart_log_printf(
          "BUMPER_FAULT,raw=0x%02X,active=0x%02X,latched=0x%02X\r\n",
          bumper.raw_mask, bumper.active_mask, bumper.latched_mask);
      changed = 1U;
    }
    else
    {
      motor_speed_control_hold_brake();
    }
  }
  if(changed != 0U)
  {
    chassis_send_safety_can(1U);
  }
}

static void chassis_update_can_status(void)
{
  chassis_can_fast_ticks++;
  chassis_can_odom_ticks++;
  chassis_can_safety_ticks++;

  if(chassis_can_fast_ticks >= 10U)
  {
    chassis_can_fast_ticks = 0U;
    chassis_send_drive_can();
  }
  if(chassis_can_odom_ticks >= 2U)
  {
    chassis_can_odom_ticks = 0U;
    chassis_send_odom_can();
  }
  if(chassis_can_safety_ticks >= 10U)
  {
    chassis_can_safety_ticks = 0U;
    chassis_send_safety_can(0U);
  }
}

void chassis_motion_controller_init(void)
{
  int8_t imu_result;

  debug_uart_transport_init();
  motor_speed_control_init();
  /* 编码器使用外部10kΩ上拉到5V，内部上下拉保持关闭。 */
  motor_speed_control_hold_brake();
  chassis_heading_fusion_init(&chassis_heading_fusion,
                              CHASSIS_HEADING_FUSION_TAU_S,
                              CHASSIS_HEADING_ENCODER_RATE_LIMIT_DPS);
  chassis_sync_heading_encoder();
  bumper_safety_init();

  chassis_state = CHASSIS_STATE_IDLE;
  chassis_fault = CHASSIS_FAULT_NONE;
  chassis_task_sequence = 0U;
  chassis_task_session = 0U;
  chassis_task_source = MOTION_SOURCE_LOCAL_SAFETY;
  chassis_task_type = MOTION_CMD_STOP;
  chassis_task_active = 0U;
  chassis_motion_enabled = 0U;
  chassis_state_ticks = 0U;
  chassis_recover_stable_ticks = 0U;
  chassis_caster_stable_ticks = 0U;
  chassis_no_progress_ticks = 0U;
  chassis_task_ticks = 0U;
  chassis_task_timeout_ticks = CHASSIS_CONTROL_TASK_TIMEOUT_MIN_TICKS;
  chassis_speed_ticks = 0U;
  chassis_telemetry_ticks = 0U;
  chassis_can_fast_ticks = 0U;
  chassis_can_odom_ticks = 0U;
  chassis_can_safety_ticks = 0U;
  chassis_uptime_ms = 0U;
  chassis_target_distance_mm = 0L;
  chassis_target_yaw_cdeg = 0L;
  chassis_translation_direction = 1;
  chassis_rotation_direction = 0;
  chassis_rotation_task = 0U;
  chassis_rotation_confirmed = 0U;
  chassis_rotation_confirm_ticks = 0U;
  chassis_magnetic_follow_task = 0U;
  chassis_magnetic_sample_valid = 0U;
  chassis_magnetic_invalid_ticks = 0U;
  magnetic_follow_controller_reset(&chassis_magnetic_controller);
  chassis_heading_disagreement_ticks = 0U;
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
    chassis_reset_heading_estimator();
    chassis_heading_target_deg = 0.0f;
    debug_uart_log_printf(
        "CHASSIS_CONTROL,READY,TRANSLATE_300_3000MM,"
        "ROTATE_10_90DEG,50_100MM_S\r\n");
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

  /* 防撞条优先于姿态更新和普通运动状态机，触发后本周期立即保持刹车。 */
  chassis_handle_bumper_safety();

  imu_result = imu_heading_update(sample_dt_s);
  if((imu_result != BMI2_OK) &&
     (chassis_state != CHASSIS_STATE_IMU_ERROR))
  {
    chassis_fail_task(CHASSIS_FAULT_IMU);
  }
  else if(imu_result == BMI2_OK)
  {
    chassis_update_heading_estimator(sample_dt_s);
  }

  if((chassis_task_active != 0U) &&
     (chassis_magnetic_follow_task != 0U))
  {
    (void)chassis_update_magnetic_follow(sample_dt_s);
  }
  chassis_heading_error_deg = chassis_wrap_deg(
      chassis_heading_target_deg - chassis_get_heading_deg());
  chassis_check_heading_consistency();
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
      /* 停车状态同步编码器基准并清零旧滤波值，避免遥测保持停车前速度。 */
      motor_speed_control_resync_feedback();
      motor_speed_control_hold_brake();
    }
  }

  chassis_telemetry_ticks++;
  if(chassis_telemetry_ticks >= CHASSIS_CONTROL_TELEMETRY_PERIOD_TICKS)
  {
    chassis_telemetry_ticks = 0U;
    chassis_send_telemetry();
  }

  chassis_update_can_status();
}

uint8_t chassis_motion_controller_get_state(void)
{
  return (uint8_t)chassis_state;
}

uint16_t chassis_motion_controller_get_fault_mask(void)
{
  return chassis_fault;
}
