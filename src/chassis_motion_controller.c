/**
  ****************************************************************************
  * @file     chassis_motion_controller.c
  * @brief    External MotionReference底盘执行器与本地安全边界
  *
  * Supervisor拥有任务与路径决策；电机板只执行已获租约的(v, omega, TTL)
  * 参考，并在20ms速度周期完成差速解算、共同限幅、斜率限制和速度闭环。
  * Bumper、ESTOP、通信、IMU和电气故障始终在本板锁存并优先刹车。
  ****************************************************************************
  */

#include "chassis_motion_controller.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "at32f45x_board.h"
#include "bmi2_defs.h"
#include "bumper_safety.h"
#include "canfd_protocol.h"
#include "chassis_heading_fusion.h"
#include "debug_uart_transport.h"
#include "dock_ir_receiver.h"
#include "imu_heading.h"
#include "math_constants.h"
#include "motion_reference_control.h"
#include "motion_reference_receiver.h"
#include "motor_encoder.h"
#include "motor_speed_calibration.h"
#include "motor_speed_control.h"
#include "power_supervisor.h"

#define CHASSIS_LOOP_PERIOD_MS                         10U
#define CHASSIS_SPEED_PERIOD_TICKS \
    (MOTOR_SPEED_CONTROL_PERIOD_MS / CHASSIS_LOOP_PERIOD_MS)
#define CHASSIS_MOTOR_STATUS_PERIOD_TICKS              10U
#define CHASSIS_DRIVE_STATUS_PERIOD_TICKS              10U
#define CHASSIS_ODOM_STATUS_PERIOD_TICKS                2U
#define CHASSIS_SAFETY_STATUS_PERIOD_TICKS             10U

#define CHASSIS_WHEEL_CIRCUMFERENCE_MM              267.035f
#define CHASSIS_DRIVE_TRACK_MM                       500.0f
#define CHASSIS_MAX_LINEAR_MM_S                       330U
#define CHASSIS_MAX_ANGULAR_MRAD_S                   1336U
#define CHASSIS_MIN_ACCEL_MM_S2                        10U
#define CHASSIS_MAX_ACCEL_MM_S2                       500U
#define CHASSIS_MIN_DECEL_MM_S2                        10U
#define CHASSIS_MAX_DECEL_MM_S2                      1000U
#define CHASSIS_YAW_RATE_FEEDBACK_GAIN                 0.35f
#define CHASSIS_REFERENCE_CONTROL_DT_S \
    ((float)MOTOR_SPEED_CONTROL_PERIOD_MS / 1000.0f)

#define CHASSIS_HEADING_FUSION_TAU_S                    2.0f
#define CHASSIS_HEADING_ENCODER_RATE_LIMIT_DPS        120.0f
#define CHASSIS_IMU_FRESH_LIMIT_MS                      50U

#define CHASSIS_FAULT_NONE                          0x0000U
#define CHASSIS_FAULT_ESTOP                         0x0001U
#define CHASSIS_FAULT_IMU                           0x0002U
#define CHASSIS_FAULT_MOTION                        0x0004U
#define CHASSIS_FAULT_BUMPER                        0x0008U
#define CHASSIS_FAULT_COMM_TIMEOUT                  0x0010U
#define CHASSIS_FAULT_CAN_BUSOFF                    0x0020U
#define CHASSIS_FAULT_POWER                         0x0080U

/* 底层左右安装方向不同；本文件内始终使用“车体前进为正”。 */
#define CHASSIS_LEFT_MOTOR                          MOTOR_LEFT
#define CHASSIS_RIGHT_MOTOR                         MOTOR_RIGHT
#define CHASSIS_LEFT_COMMAND_SIGN                    -1.0f
#define CHASSIS_RIGHT_COMMAND_SIGN                    1.0f
#define CHASSIS_LEFT_ENCODER_SIGN                     -1L
#define CHASSIS_RIGHT_ENCODER_SIGN                    -1L

typedef enum
{
  CHASSIS_STATE_IDLE = 0,
  CHASSIS_STATE_HOLD = 7,
  CHASSIS_STATE_ESTOP = 8,
  CHASSIS_STATE_BUMPER_ERROR = 9,
  CHASSIS_STATE_COMM_ERROR = 10,
  CHASSIS_STATE_CAN_ERROR = 11,
  CHASSIS_STATE_IMU_ERROR = 12,
  CHASSIS_STATE_MOTION_ERROR = 13,
  CHASSIS_STATE_VELOCITY_CONTROL = 14
} chassis_state_t;

static const motion_reference_platform_limits_t chassis_reference_limits =
{
  CHASSIS_MAX_LINEAR_MM_S,
  CHASSIS_MAX_ANGULAR_MRAD_S,
  CHASSIS_MIN_ACCEL_MM_S2,
  CHASSIS_MAX_ACCEL_MM_S2,
  CHASSIS_MIN_DECEL_MM_S2,
  CHASSIS_MAX_DECEL_MM_S2,
  CHASSIS_DRIVE_TRACK_MM,
  CHASSIS_WHEEL_CIRCUMFERENCE_MM,
  MOTOR_SPEED_TARGET_LIMIT_RPM,
  CHASSIS_YAW_RATE_FEEDBACK_GAIN
};

static chassis_state_t chassis_state;
static uint16_t chassis_fault;
static uint8_t chassis_motion_enabled;
static uint8_t chassis_reference_active;
static uint8_t chassis_reference_valid;
static uint8_t chassis_speed_ticks;
static uint8_t chassis_motor_status_ticks;
static uint8_t chassis_drive_status_ticks;
static uint8_t chassis_odom_status_ticks;
static uint8_t chassis_safety_status_ticks;
static uint32_t chassis_uptime_ms;
static uint32_t chassis_last_cycle_count;

static agv_motion_reference_t chassis_reference;
static motion_reference_control_t chassis_reference_control;
static float chassis_left_target_rpm;
static float chassis_right_target_rpm;

static chassis_heading_fusion_t chassis_heading_fusion;
static int32_t chassis_heading_last_left_count;
static int32_t chassis_heading_last_right_count;
static int32_t chassis_odom_last_left_count;
static int32_t chassis_odom_last_right_count;
static float chassis_odom_last_yaw_deg;
static float chassis_pose_x_mm;
static float chassis_pose_y_mm;

static uint32_t chassis_power_status_epoch;
static volatile uint32_t chassis_power_status_snapshot;

static void chassis_send_safety_can(uint8_t urgent);
static void chassis_stop_with_fault(uint16_t fault);

static float chassis_limit(float value, float minimum, float maximum)
{
  if(value < minimum)
  {
    return minimum;
  }
  if(value > maximum)
  {
    return maximum;
  }
  return value;
}

static int32_t chassis_round_to_int(float value)
{
  return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
                           (int32_t)(value - 0.5f);
}

static int16_t chassis_to_i16(int32_t value)
{
  if(value > INT16_MAX)
  {
    return INT16_MAX;
  }
  if(value < INT16_MIN)
  {
    return INT16_MIN;
  }
  return (int16_t)value;
}

static uint8_t chassis_to_u8(uint32_t value)
{
  return (value > 255UL) ? 255U : (uint8_t)value;
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

static void chassis_set_wheel_targets(float left_rpm, float right_rpm)
{
  motor_speed_control_set_targets(left_rpm * CHASSIS_LEFT_COMMAND_SIGN,
                                  right_rpm * CHASSIS_RIGHT_COMMAND_SIGN);
}

static uint8_t chassis_imu_is_fresh(void)
{
  return ((imu_heading_get_status() == BMI2_OK) &&
          (imu_heading_get_gyro_age_ms() < CHASSIS_IMU_FRESH_LIMIT_MS)) ?
         1U : 0U;
}

static void chassis_hold_brake(void)
{
  chassis_left_target_rpm = 0.0f;
  chassis_right_target_rpm = 0.0f;
  motion_reference_control_reset(&chassis_reference_control);
  motor_speed_control_hold_brake();
}

static void chassis_clear_reference(void)
{
  memset(&chassis_reference, 0, sizeof(chassis_reference));
  chassis_reference_active = 0U;
  chassis_reference_valid = 0U;
  motion_reference_control_reset(&chassis_reference_control);
}

static uint8_t chassis_reference_is_driving(void)
{
  return ((chassis_reference_active != 0U) &&
          (chassis_reference_valid != 0U) &&
          (chassis_reference.kind ==
           (uint8_t)AGV_MOTION_KIND_BODY_VELOCITY)) ? 1U : 0U;
}

static chassis_state_t chassis_state_for_fault(void)
{
  if((chassis_fault & CHASSIS_FAULT_ESTOP) != 0U)
  {
    return CHASSIS_STATE_ESTOP;
  }
  if((chassis_fault & CHASSIS_FAULT_BUMPER) != 0U)
  {
    return CHASSIS_STATE_BUMPER_ERROR;
  }
  if((chassis_fault & CHASSIS_FAULT_CAN_BUSOFF) != 0U)
  {
    return CHASSIS_STATE_CAN_ERROR;
  }
  if((chassis_fault & CHASSIS_FAULT_COMM_TIMEOUT) != 0U)
  {
    return CHASSIS_STATE_COMM_ERROR;
  }
  if((chassis_fault & CHASSIS_FAULT_IMU) != 0U)
  {
    return CHASSIS_STATE_IMU_ERROR;
  }
  return CHASSIS_STATE_MOTION_ERROR;
}

static void chassis_stop_with_fault(uint16_t fault)
{
  /* 所有可能阻塞的遥测之前先撤目标并主动刹车。 */
  chassis_hold_brake();
  chassis_clear_reference();
  chassis_motion_enabled = 0U;
  chassis_fault |= fault;
  chassis_state = chassis_state_for_fault();
  chassis_send_safety_can(1U);
}

static void chassis_publish_power_status(void)
{
  uint32_t status = 0UL;

  if((chassis_motion_enabled == 0U) &&
     (chassis_fault == CHASSIS_FAULT_NONE) &&
     (chassis_reference_active == 0U) &&
     (motor_speed_control_get_pwm(MOTOR_LEFT) == 0) &&
     (motor_speed_control_get_pwm(MOTOR_RIGHT) == 0))
  {
    status |= CHASSIS_POWER_STATUS_ZERO_SAFE;
  }
  if((chassis_fault & CHASSIS_FAULT_POWER) != 0U)
  {
    status |= CHASSIS_POWER_STATUS_FAULT_LATCHED;
  }
  chassis_power_status_epoch++;
  chassis_power_status_snapshot = (chassis_power_status_epoch << 2U) | status;
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

static void chassis_sync_heading_counts(void)
{
  chassis_heading_last_left_count = chassis_get_left_encoder_count();
  chassis_heading_last_right_count = chassis_get_right_encoder_count();
}

static void chassis_sync_odometry_counts(void)
{
  chassis_odom_last_left_count = chassis_get_left_encoder_count();
  chassis_odom_last_right_count = chassis_get_right_encoder_count();
  chassis_odom_last_yaw_deg = chassis_heading_fusion.fused_yaw_deg;
}

static void chassis_reset_heading(void)
{
  imu_heading_reset();
  chassis_heading_fusion_reset(&chassis_heading_fusion);
  chassis_sync_heading_counts();
  chassis_sync_odometry_counts();
}

static uint8_t chassis_update_motion_estimate(float dt_s)
{
  chassis_signed_odometry_pose_t pose;
  int32_t left_count = chassis_get_left_encoder_count();
  int32_t right_count = chassis_get_right_encoder_count();
  float encoder_yaw_delta;
  float current_yaw;

  encoder_yaw_delta = chassis_heading_fusion_encoder_delta_deg(
      left_count - chassis_heading_last_left_count,
      right_count - chassis_heading_last_right_count,
      CHASSIS_WHEEL_CIRCUMFERENCE_MM,
      MOTOR_SPEED_ENCODER_COUNTS_PER_OUTPUT_REV,
      CHASSIS_DRIVE_TRACK_MM);
  chassis_heading_last_left_count = left_count;
  chassis_heading_last_right_count = right_count;
  chassis_heading_fusion_update(&chassis_heading_fusion,
                                imu_heading_get_rate_dps(),
                                encoder_yaw_delta,
                                dt_s);

  pose.x_mm = chassis_pose_x_mm;
  pose.y_mm = chassis_pose_y_mm;
  current_yaw = chassis_heading_fusion.fused_yaw_deg;
  if(chassis_signed_odometry_integrate(
       &pose,
       left_count - chassis_odom_last_left_count,
       right_count - chassis_odom_last_right_count,
       CHASSIS_WHEEL_CIRCUMFERENCE_MM,
       MOTOR_SPEED_ENCODER_COUNTS_PER_OUTPUT_REV,
       chassis_odom_last_yaw_deg,
       current_yaw) == 0U)
  {
    return 0U;
  }
  chassis_pose_x_mm = pose.x_mm;
  chassis_pose_y_mm = pose.y_mm;
  chassis_odom_last_left_count = left_count;
  chassis_odom_last_right_count = right_count;
  chassis_odom_last_yaw_deg = current_yaw;
  return 1U;
}

static void chassis_reset_odometry(void)
{
  chassis_hold_brake();
  motor_encoder_reset_all();
  motor_speed_control_resync_feedback();
  chassis_pose_x_mm = 0.0f;
  chassis_pose_y_mm = 0.0f;
  chassis_reset_heading();
}

static void chassis_update_reference_input(void)
{
  agv_motion_reference_t reference;

  if((chassis_motion_enabled == 0U) ||
     (chassis_fault != CHASSIS_FAULT_NONE))
  {
    if(chassis_reference_active != 0U)
    {
      chassis_hold_brake();
      chassis_clear_reference();
    }
    return;
  }

  if(canfd_protocol_get_motion_reference(&reference) != 0U)
  {
    if(agv_motion_reference_is_fresh(&reference, chassis_uptime_ms) == 0U)
    {
      chassis_stop_with_fault(CHASSIS_FAULT_COMM_TIMEOUT);
      return;
    }
    if(chassis_reference_valid == 0U)
    {
      motion_reference_control_reset(&chassis_reference_control);
    }
    chassis_reference = reference;
    chassis_reference_active = 1U;
    chassis_reference_valid = 1U;
    if(reference.kind == (uint8_t)AGV_MOTION_KIND_HOLD)
    {
      chassis_hold_brake();
      chassis_state = CHASSIS_STATE_HOLD;
    }
    else
    {
      chassis_state = CHASSIS_STATE_VELOCITY_CONTROL;
    }
    return;
  }

  chassis_hold_brake();
  if(chassis_reference_active != 0U)
  {
    chassis_reference_valid = 0U;
    chassis_state = CHASSIS_STATE_HOLD;
    if(motion_reference_miss_requires_fault(
         &chassis_reference,
         chassis_uptime_ms,
         canfd_protocol_is_session_valid()) != 0U)
    {
      chassis_stop_with_fault(CHASSIS_FAULT_COMM_TIMEOUT);
    }
  }
  else
  {
    chassis_state = CHASSIS_STATE_IDLE;
  }
}

static void chassis_run_speed_tick(void)
{
  motion_reference_wheel_targets_t targets;
  float yaw_rate_mrad_s;

  if((chassis_motion_enabled == 0U) ||
     (chassis_fault != CHASSIS_FAULT_NONE) ||
     (chassis_reference_active == 0U) ||
     (chassis_reference_valid == 0U))
  {
    /* 停车期间仍采样实际轮速，换向门不能被重同步伪造成零速。 */
    motor_speed_control_set_targets(0.0f, 0.0f);
    motor_speed_control_update_now();
    chassis_hold_brake();
    return;
  }
  if(chassis_reference.kind == (uint8_t)AGV_MOTION_KIND_HOLD)
  {
    chassis_hold_brake();
    return;
  }
  if(agv_motion_reference_is_fresh(
       &chassis_reference, chassis_uptime_ms) == 0U)
  {
    chassis_stop_with_fault(CHASSIS_FAULT_COMM_TIMEOUT);
    return;
  }

  yaw_rate_mrad_s = imu_heading_get_rate_dps() *
                    AGV_MATH_PI_F * 1000.0f / 180.0f;
  if(motion_reference_control_step(
       &chassis_reference_control,
       &chassis_reference,
       &chassis_reference_limits,
       yaw_rate_mrad_s,
       CHASSIS_REFERENCE_CONTROL_DT_S,
       &targets) == 0U)
  {
    chassis_stop_with_fault(CHASSIS_FAULT_MOTION);
    return;
  }

  chassis_left_target_rpm = targets.left_rpm;
  chassis_right_target_rpm = targets.right_rpm;
  chassis_set_wheel_targets(chassis_left_target_rpm,
                            chassis_right_target_rpm);
  motor_speed_control_update_now();
  if(motor_speed_control_has_fault() != 0U)
  {
    chassis_stop_with_fault(CHASSIS_FAULT_MOTION);
  }
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
                       CHASSIS_FAULT_MOTION |
                       CHASSIS_FAULT_POWER)) != 0U)
  {
    mask |= 0x04U;
  }
  return mask;
}

static uint8_t chassis_get_application_state(void)
{
  if((chassis_fault & (CHASSIS_FAULT_ESTOP |
                       CHASSIS_FAULT_BUMPER)) != 0U)
  {
    return 6U;
  }
  if(chassis_fault != CHASSIS_FAULT_NONE)
  {
    return 5U;
  }
  if(chassis_reference_is_driving() != 0U)
  {
    return 3U;
  }
  return (chassis_motion_enabled != 0U) ? 2U : 1U;
}

static uint8_t chassis_get_motion_state(void)
{
  if((chassis_fault & (CHASSIS_FAULT_ESTOP |
                       CHASSIS_FAULT_BUMPER)) != 0U)
  {
    return 6U;
  }
  if(chassis_fault != CHASSIS_FAULT_NONE)
  {
    return 7U;
  }
  if(chassis_reference_is_driving() != 0U)
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
  status.warning_mask = (uint32_t)power_supervisor_get_fault_mask();
  status.bumper_mask = (uint8_t)(bumper.active_mask | bumper.latched_mask);
  status.estop_source_mask = chassis_get_safety_source_mask();
  if((chassis_fault & (CHASSIS_FAULT_ESTOP |
                       CHASSIS_FAULT_BUMPER)) != 0U)
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

  status.application_state = chassis_get_application_state();
  status.motion_state = chassis_get_motion_state();
  status.motion_mode = (chassis_reference_active != 0U) ? 3U : 0U;
  status.flags = 0x04U;
  if(canfd_protocol_is_session_valid() != 0U)
  {
    status.flags |= 0x01U;
  }
  if(chassis_motion_enabled != 0U)
  {
    status.flags |= 0x02U;
  }
  if(chassis_imu_is_fresh() != 0U)
  {
    status.flags |= 0x08U;
  }
  status.command_id = (chassis_reference_active != 0U) ?
                      chassis_reference.reference_id : 0UL;
  status.session_id = canfd_protocol_get_session_id();
  status.fault_mask = (uint32_t)chassis_fault;
  status.warning_mask = (uint32_t)power_supervisor_get_fault_mask();
  status.x_mm = chassis_round_to_int(chassis_pose_x_mm);
  status.y_mm = chassis_round_to_int(chassis_pose_y_mm);
  status.yaw_cdeg = chassis_round_to_int(
      chassis_heading_fusion.fused_yaw_deg * 100.0f);
  canfd_protocol_send_drive_status(&status);
}

static void chassis_send_odom_can(void)
{
  canfd_odom_status_t status;
  float left_rpm = chassis_get_left_measured_rpm();
  float right_rpm = chassis_get_right_measured_rpm();
  float linear_mm_s = ((left_rpm + right_rpm) * 0.5f *
                       CHASSIS_WHEEL_CIRCUMFERENCE_MM) / 60.0f;

  status.uptime_ms = chassis_uptime_ms;
  status.x_mm = chassis_round_to_int(chassis_pose_x_mm);
  status.y_mm = chassis_round_to_int(chassis_pose_y_mm);
  status.yaw_cdeg = chassis_round_to_int(
      chassis_heading_fusion.fused_yaw_deg * 100.0f);
  status.linear_mm_s = chassis_to_i16(chassis_round_to_int(linear_mm_s));
  status.yaw_rate_cdeg_s = chassis_to_i16(chassis_round_to_int(
      imu_heading_get_rate_dps() * 100.0f));
  status.accel_x_mg = imu_heading_get_linear_accel_x_mg();
  status.accel_y_mg = imu_heading_get_linear_accel_y_mg();
  status.accel_z_mg = imu_heading_get_linear_accel_z_mg();
  status.valid_flags = (chassis_imu_is_fresh() != 0U) ? 0x000FU : 0x0003U;
  canfd_protocol_send_odom_status(&status);
}

static void chassis_send_motor_can(void)
{
  canfd_motor_status_t status;

  status.uptime_ms = chassis_uptime_ms;
  status.left_target_rpm_x100 = chassis_to_i16(
      chassis_round_to_int(chassis_left_target_rpm * 100.0f));
  status.right_target_rpm_x100 = chassis_to_i16(
      chassis_round_to_int(chassis_right_target_rpm * 100.0f));
  status.left_measured_rpm_x100 = chassis_to_i16(
      chassis_round_to_int(chassis_get_left_measured_rpm() * 100.0f));
  status.right_measured_rpm_x100 = chassis_to_i16(
      chassis_round_to_int(chassis_get_right_measured_rpm() * 100.0f));
  status.left_pwm = chassis_get_left_pwm();
  status.right_pwm = chassis_get_right_pwm();
  status.left_encoder_count = chassis_get_left_encoder_count();
  status.right_encoder_count = chassis_get_right_encoder_count();
  status.left_invalid_count = chassis_to_u8(
      motor_encoder_get_invalid_transition_count(CHASSIS_LEFT_MOTOR));
  status.right_invalid_count = chassis_to_u8(
      motor_encoder_get_invalid_transition_count(CHASSIS_RIGHT_MOTOR));
  status.state = (uint8_t)chassis_state;
  status.ir_activity = dock_ir_receiver_get_activity();
  status.ir_activity_percent = dock_ir_receiver_get_activity_percent();
  status.ir_raw_level = dock_ir_receiver_get_raw_level();
  status.ir_falling_edges = dock_ir_receiver_get_falling_edges();
  canfd_protocol_send_motor_status(&status);
}

static void chassis_update_can_status(void)
{
  chassis_motor_status_ticks++;
  chassis_drive_status_ticks++;
  chassis_odom_status_ticks++;
  chassis_safety_status_ticks++;

  if(chassis_motor_status_ticks >= CHASSIS_MOTOR_STATUS_PERIOD_TICKS)
  {
    chassis_motor_status_ticks = 0U;
    chassis_send_motor_can();
  }
  if(chassis_drive_status_ticks >= CHASSIS_DRIVE_STATUS_PERIOD_TICKS)
  {
    chassis_drive_status_ticks = 0U;
    chassis_send_drive_can();
  }
  if(chassis_odom_status_ticks >= CHASSIS_ODOM_STATUS_PERIOD_TICKS)
  {
    chassis_odom_status_ticks = 0U;
    chassis_send_odom_can();
  }
  if(chassis_safety_status_ticks >= CHASSIS_SAFETY_STATUS_PERIOD_TICKS)
  {
    chassis_safety_status_ticks = 0U;
    chassis_send_safety_can(0U);
  }
}

static void chassis_handle_bumper(void)
{
  bumper_safety_snapshot_t bumper;
  uint8_t changed = bumper_safety_step_10ms();

  bumper_safety_get_snapshot(&bumper);
  if(bumper.latched_mask != 0U)
  {
    if((chassis_fault & CHASSIS_FAULT_BUMPER) == 0U)
    {
      chassis_stop_with_fault(CHASSIS_FAULT_BUMPER);
      return;
    }
    chassis_hold_brake();
  }
  if(changed != 0U)
  {
    chassis_send_safety_can(1U);
  }
}

motion_command_handle_result_t chassis_motion_controller_handle_command(
    const motion_command_t *command)
{
  if(command == NULL)
  {
    return MOTION_HANDLE_REJECTED;
  }

  if(command->type == MOTION_CMD_ENABLE)
  {
    if((chassis_motion_enabled != 0U) ||
       (chassis_reference_active != 0U))
    {
      return MOTION_HANDLE_BUSY;
    }
    if((chassis_fault != CHASSIS_FAULT_NONE) ||
       (power_supervisor_is_ready() == 0U) ||
       (chassis_imu_is_fresh() == 0U) ||
       (bumper_safety_can_clear() == 0U) ||
       ((command->source == MOTION_SOURCE_CANFD) &&
        (canfd_protocol_is_session_valid() == 0U)))
    {
      return MOTION_HANDLE_REJECTED;
    }
    chassis_hold_brake();
    chassis_clear_reference();
    chassis_motion_enabled = 1U;
    chassis_state = CHASSIS_STATE_IDLE;
    return MOTION_HANDLE_ACCEPTED;
  }

  if((command->type == MOTION_CMD_DISABLE) ||
     (command->type == MOTION_CMD_RELEASE))
  {
    chassis_hold_brake();
    chassis_clear_reference();
    chassis_motion_enabled = 0U;
    chassis_state = (chassis_fault == CHASSIS_FAULT_NONE) ?
                    CHASSIS_STATE_IDLE : chassis_state_for_fault();
    return MOTION_HANDLE_ACCEPTED;
  }

  if(command->type == MOTION_CMD_STOP)
  {
    chassis_hold_brake();
    chassis_clear_reference();
    /* STOP同时撤销运动使能，防止接收器中的旧0x401在Disable前短暂复活。 */
    chassis_motion_enabled = 0U;
    if(chassis_fault == CHASSIS_FAULT_NONE)
    {
      chassis_state = CHASSIS_STATE_HOLD;
    }
    return MOTION_HANDLE_ACCEPTED;
  }

  if(command->type == MOTION_CMD_ESTOP)
  {
    chassis_stop_with_fault(CHASSIS_FAULT_ESTOP);
    return MOTION_HANDLE_ACCEPTED;
  }
  if(command->type == MOTION_CMD_COMM_TIMEOUT)
  {
    chassis_stop_with_fault(CHASSIS_FAULT_COMM_TIMEOUT);
    return MOTION_HANDLE_ACCEPTED;
  }
  if(command->type == MOTION_CMD_CAN_BUSOFF)
  {
    chassis_stop_with_fault(CHASSIS_FAULT_CAN_BUSOFF);
    return MOTION_HANDLE_ACCEPTED;
  }
  if(command->type == MOTION_CMD_POWER_FAULT)
  {
    chassis_stop_with_fault(CHASSIS_FAULT_POWER);
    return MOTION_HANDLE_ACCEPTED;
  }

  if(command->type == MOTION_CMD_RESET_ODOM)
  {
    if(chassis_reference_active != 0U)
    {
      return MOTION_HANDLE_BUSY;
    }
    if((chassis_fault != CHASSIS_FAULT_NONE) ||
       (chassis_imu_is_fresh() == 0U))
    {
      return MOTION_HANDLE_REJECTED;
    }
    chassis_reset_odometry();
    chassis_state = CHASSIS_STATE_IDLE;
    return MOTION_HANDLE_ACCEPTED;
  }

  if(command->type == MOTION_CMD_CLEAR_FAULT)
  {
    if(chassis_reference_active != 0U)
    {
      return MOTION_HANDLE_BUSY;
    }
    if((((chassis_fault & CHASSIS_FAULT_CAN_BUSOFF) != 0U) &&
         (canfd_protocol_is_bus_healthy() == 0U)) ||
       (chassis_imu_is_fresh() == 0U) ||
       (bumper_safety_can_clear() == 0U))
    {
      return MOTION_HANDLE_REJECTED;
    }

    /* 第一次CLEAR只请求功率监督；新样本确认后第二次才释放底盘锁存。 */
    if(((chassis_fault & CHASSIS_FAULT_POWER) != 0U) &&
       ((power_supervisor_get_fault_mask() &
         POWER_FAULT_SEVERE_MASK) != 0U))
    {
      return (power_supervisor_clear_fault() != 0U) ?
             MOTION_HANDLE_BUSY : MOTION_HANDLE_REJECTED;
    }

    chassis_hold_brake();
    if(bumper_safety_clear_latch() == 0U)
    {
      return MOTION_HANDLE_REJECTED;
    }
    chassis_clear_reference();
    motor_speed_control_clear_fault();
    chassis_fault = CHASSIS_FAULT_NONE;
    chassis_motion_enabled = 0U;
    chassis_state = CHASSIS_STATE_IDLE;
    chassis_send_safety_can(1U);
    return MOTION_HANDLE_ACCEPTED;
  }

  return MOTION_HANDLE_REJECTED;
}

void chassis_motion_controller_init(void)
{
  int8_t imu_result;

  debug_uart_transport_init();
  motor_speed_control_init();
  motor_speed_control_hold_brake();
  bumper_safety_init();
  chassis_heading_fusion_init(&chassis_heading_fusion,
                              CHASSIS_HEADING_FUSION_TAU_S,
                              CHASSIS_HEADING_ENCODER_RATE_LIMIT_DPS);

  chassis_state = CHASSIS_STATE_IDLE;
  chassis_fault = CHASSIS_FAULT_NONE;
  chassis_motion_enabled = 0U;
  chassis_reference_active = 0U;
  chassis_reference_valid = 0U;
  chassis_speed_ticks = 0U;
  chassis_motor_status_ticks = 0U;
  chassis_drive_status_ticks = 0U;
  chassis_odom_status_ticks = 0U;
  chassis_safety_status_ticks = 0U;
  chassis_uptime_ms = 0UL;
  chassis_left_target_rpm = 0.0f;
  chassis_right_target_rpm = 0.0f;
  chassis_pose_x_mm = 0.0f;
  chassis_pose_y_mm = 0.0f;
  chassis_power_status_epoch = 0UL;
  chassis_power_status_snapshot = 0UL;
  memset(&chassis_reference, 0, sizeof(chassis_reference));
  motion_reference_control_reset(&chassis_reference_control);

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0UL;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  imu_result = imu_heading_init();
  if(imu_result == BMI2_OK)
  {
    chassis_reset_heading();
  }
  else
  {
    chassis_fault = CHASSIS_FAULT_IMU;
    chassis_state = CHASSIS_STATE_IMU_ERROR;
    chassis_sync_heading_counts();
    chassis_sync_odometry_counts();
  }

  DWT->CYCCNT = 0UL;
  chassis_last_cycle_count = DWT->CYCCNT;
  chassis_publish_power_status();
}

void chassis_motion_controller_step_10ms(void)
{
  int8_t imu_result;
  float sample_dt_s;

  chassis_uptime_ms += CHASSIS_LOOP_PERIOD_MS;
  sample_dt_s = chassis_get_sample_dt_s();

  /* 本地防撞先于所有普通控制和遥测。 */
  chassis_handle_bumper();

  imu_result = imu_heading_update(sample_dt_s);
  if(imu_result != BMI2_OK)
  {
    if((chassis_fault & CHASSIS_FAULT_IMU) == 0U)
    {
      chassis_stop_with_fault(CHASSIS_FAULT_IMU);
    }
    else
    {
      chassis_hold_brake();
    }
  }
  else if(chassis_update_motion_estimate(sample_dt_s) == 0U)
  {
    chassis_stop_with_fault(CHASSIS_FAULT_MOTION);
  }

  chassis_update_reference_input();

  chassis_speed_ticks++;
  if(chassis_speed_ticks >= CHASSIS_SPEED_PERIOD_TICKS)
  {
    chassis_speed_ticks = 0U;
    chassis_run_speed_tick();
  }

  chassis_update_can_status();
  chassis_publish_power_status();
}

uint8_t chassis_motion_controller_get_state(void)
{
  return (uint8_t)chassis_state;
}

uint16_t chassis_motion_controller_get_fault_mask(void)
{
  return chassis_fault;
}

uint32_t chassis_motion_controller_get_uptime_ms(void)
{
  return chassis_uptime_ms;
}

uint32_t chassis_motion_controller_get_power_status_snapshot(void)
{
  return chassis_power_status_snapshot;
}

uint8_t chassis_motion_controller_has_power_fault(void)
{
  return ((chassis_power_status_snapshot &
           CHASSIS_POWER_STATUS_FAULT_LATCHED) != 0UL) ? 1U : 0U;
}
