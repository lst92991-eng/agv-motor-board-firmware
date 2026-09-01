/**
 * @file chassis_motion_controller.c
 * @brief 唯一底盘Reference执行器和本地安全状态机
 *
 * 本文件是正常运动状态与PWM目标的唯一应用层写者。命令只管理使能/停车/复位，
 * 连续运动量唯一来自0x401；任何本地故障都先制动、再清Reference、最后发布状态。
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
#include "imu_heading.h"
#include "math_constants.h"
#include "motion_reference_control.h"
#include "motor_encoder.h"
#include "motor_speed_calibration.h"
#include "motor_speed_control.h"
#include "power_supervisor.h"

#define CHASSIS_PERIOD_MS                     10U
#define CHASSIS_SPEED_TICKS                    2U
#define CHASSIS_ODOM_TICKS                     2U
#define CHASSIS_STATUS_TICKS                  10U
#define CHASSIS_WHEEL_CIRCUMFERENCE_MM     267.035f
#define CHASSIS_DRIVE_TRACK_MM              500.0f
#define CHASSIS_IMU_FRESH_MS                  50U
#define CHASSIS_LEFT_SIGN                    -1.0f
#define CHASSIS_RIGHT_SIGN                    1.0f
#define CHASSIS_ENCODER_SIGN                 -1L

#define CHASSIS_FAULT_ESTOP                 0x01U
#define CHASSIS_FAULT_IMU                   0x02U
#define CHASSIS_FAULT_MOTION                0x04U
#define CHASSIS_FAULT_BUMPER                0x08U
#define CHASSIS_FAULT_TIMEOUT               0x10U
#define CHASSIS_FAULT_CAN                   0x20U
#define CHASSIS_FAULT_POWER                 0x80U

typedef enum
{
  CHASSIS_IDLE = 0,
  CHASSIS_HOLD = 7,
  CHASSIS_ESTOP = 8,
  CHASSIS_FAULT = 13,
  CHASSIS_VELOCITY = 14
} chassis_state_t;

static const motion_reference_platform_limits_t chassis_limits =
{
  /* v/omega、加减速度、轮距、轮周长、轮速和Yaw反馈增益的唯一平台配置。 */
  300U, 1200U, 10U, 500U, 10U, 1000U,
  CHASSIS_DRIVE_TRACK_MM,
  CHASSIS_WHEEL_CIRCUMFERENCE_MM,
  MOTOR_SPEED_TARGET_LIMIT_RPM,
  0.35f
};

static chassis_state_t chassis_state;
static uint16_t chassis_fault;
static uint8_t chassis_enabled;
static uint8_t chassis_reference_active;
static uint8_t chassis_speed_ticks;
static uint8_t chassis_odom_ticks;
static uint8_t chassis_status_ticks;
static uint32_t chassis_uptime_ms;
static uint32_t chassis_last_cycle;
/* 低两位为Power条件，高位递增使跨任务读取能检测控制周期变化。 */
static volatile uint32_t chassis_power_snapshot;
static agv_motion_reference_t chassis_reference;
static motion_reference_control_t chassis_control;
static chassis_heading_fusion_t chassis_heading;
static int32_t chassis_heading_left;
static int32_t chassis_heading_right;
static int32_t chassis_odom_left;
static int32_t chassis_odom_right;
static float chassis_odom_yaw;
static float chassis_x_mm;
static float chassis_y_mm;

/** @brief 读取并归一化左轮累计计数，使车体前进为正。 */
static int32_t chassis_left_count(void)
{
  return motor_encoder_get_count(MOTOR_LEFT) * CHASSIS_ENCODER_SIGN;
}

/** @brief 读取并归一化右轮累计计数，使车体前进为正。 */
static int32_t chassis_right_count(void)
{
  return motor_encoder_get_count(MOTOR_RIGHT) * CHASSIS_ENCODER_SIGN;
}

/** @brief 读取左轮滤波RPM并转换到车体统一轮向。 */
static float chassis_left_rpm(void)
{
  return motor_speed_control_get_measured_rpm(MOTOR_LEFT) * CHASSIS_LEFT_SIGN;
}

/** @brief 读取右轮滤波RPM并转换到车体统一轮向。 */
static float chassis_right_rpm(void)
{
  return motor_speed_control_get_measured_rpm(MOTOR_RIGHT) * CHASSIS_RIGHT_SIGN;
}

/** @brief 以四舍五入方式把内部float物理量转换为有符号整数。 */
static int32_t chassis_round(float value)
{
  return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
                           (int32_t)(value - 0.5f);
}

/** @brief 饱和转换为CAN遥测使用的int16范围。 */
static int16_t chassis_i16(int32_t value)
{
  if(value > INT16_MAX) return INT16_MAX;
  if(value < INT16_MIN) return INT16_MIN;
  return (int16_t)value;
}

/** @brief 检查BMI270状态和最近GYR DRDY样本是否仍小于50ms。 */
static uint8_t chassis_imu_fresh(void)
{
  return ((imu_heading_get_status() == BMI2_OK) &&
          (imu_heading_get_gyro_age_ms() < CHASSIS_IMU_FRESH_MS)) ? 1U : 0U;
}

/** @brief 清除轮速斜率/PID输出并让双H桥主动制动。 */
static void chassis_brake(void)
{
  motion_reference_control_reset(&chassis_control);
  motor_speed_control_hold_brake();
}

/** @brief 清空当前Reference和斜率历史，但不清除故障或会话。 */
static void chassis_clear_reference(void)
{
  memset(&chassis_reference, 0, sizeof(chassis_reference));
  chassis_reference_active = 0U;
  motion_reference_control_reset(&chassis_control);
}

/** @brief 组装并按普通或紧急优先级发送0x481安全状态。 */
static void chassis_send_safety(uint8_t urgent)
{
  canfd_safety_status_t status;
  bumper_safety_snapshot_t bumper;

  memset(&status, 0, sizeof(status));
  bumper_safety_get_snapshot(&bumper);
  status.uptime_ms = chassis_uptime_ms;
  status.fault_mask = chassis_fault;
  status.warning_mask = power_supervisor_get_fault_mask();
  status.bumper_mask = (uint8_t)(bumper.active_mask | bumper.latched_mask);
  status.estop_source_mask =
      ((chassis_fault & CHASSIS_FAULT_BUMPER) != 0U ? 0x01U : 0U) |
      ((chassis_fault & CHASSIS_FAULT_ESTOP) != 0U ? 0x02U : 0U) |
      ((chassis_fault & (uint16_t)~(CHASSIS_FAULT_BUMPER |
                                    CHASSIS_FAULT_ESTOP)) != 0U ? 0x04U : 0U);
  status.safety_state = ((chassis_fault & (CHASSIS_FAULT_ESTOP |
      CHASSIS_FAULT_BUMPER)) != 0U) ? 2U :
      ((chassis_fault != 0U) ? 3U : 0U);
  canfd_protocol_send_safety_status(&status, urgent);
}

/** @brief 按固定顺序制动、清Reference、锁存故障并紧急上报。 */
static void chassis_fault_stop(uint16_t fault)
{
  /* 停车动作必须先于状态和遥测更新，不能等待CAN发送完成。 */
  chassis_brake();
  chassis_clear_reference();
  chassis_enabled = 0U;
  chassis_fault |= fault;
  chassis_state = ((fault & (CHASSIS_FAULT_ESTOP |
                             CHASSIS_FAULT_BUMPER)) != 0U) ?
                  CHASSIS_ESTOP : CHASSIS_FAULT;
  chassis_send_safety(1U);
}

/** @brief 从DWT计算真实控制间隔并限制异常调度值。 */
static float chassis_sample_dt(void)
{
  uint32_t cycle = DWT->CYCCNT;
  uint32_t elapsed = cycle - chassis_last_cycle;
  float dt = (float)elapsed / (float)system_core_clock;
  chassis_last_cycle = cycle;
  /* 限制异常调度间隔，避免积分和融合在单次长暂停后跳变。 */
  if(dt < 0.001f) return 0.001f;
  if(dt > 0.100f) return 0.100f;
  return dt;
}

/** @brief 在主动制动状态清零编码器、融合状态和累计XY位姿。 */
static void chassis_reset_pose(void)
{
  chassis_brake();
  motor_encoder_reset_all();
  motor_speed_control_resync_feedback();
  chassis_heading_fusion_reset(&chassis_heading);
  chassis_x_mm = 0.0f;
  chassis_y_mm = 0.0f;
  chassis_heading_left = chassis_odom_left = chassis_left_count();
  chassis_heading_right = chassis_odom_right = chassis_right_count();
  chassis_odom_yaw = 0.0f;
}

/** @brief 融合本周期Yaw并用有符号轮程更新累计里程计。 */
static uint8_t chassis_update_pose(float dt)
{
  int32_t left = chassis_left_count();
  int32_t right = chassis_right_count();
  chassis_signed_odometry_pose_t pose;
  /* 位置使用有符号计数；Yaw由陀螺预测和编码器差分低频约束。 */
  float encoder_yaw = chassis_heading_fusion_encoder_delta_deg(
      left - chassis_heading_left, right - chassis_heading_right,
      CHASSIS_WHEEL_CIRCUMFERENCE_MM,
      MOTOR_SPEED_ENCODER_COUNTS_PER_OUTPUT_REV,
      CHASSIS_DRIVE_TRACK_MM);

  chassis_heading_left = left;
  chassis_heading_right = right;
  chassis_heading_fusion_update(&chassis_heading,
      imu_heading_get_rate_dps(), encoder_yaw, dt);
  pose.x_mm = chassis_x_mm;
  pose.y_mm = chassis_y_mm;
  if(chassis_signed_odometry_integrate(&pose,
      left - chassis_odom_left, right - chassis_odom_right,
      CHASSIS_WHEEL_CIRCUMFERENCE_MM,
      MOTOR_SPEED_ENCODER_COUNTS_PER_OUTPUT_REV,
      chassis_odom_yaw, chassis_heading.fused_yaw_deg) == 0U) return 0U;
  chassis_x_mm = pose.x_mm;
  chassis_y_mm = pose.y_mm;
  chassis_odom_left = left;
  chassis_odom_right = right;
  chassis_odom_yaw = chassis_heading.fused_yaw_deg;
  return 1U;
}

/** @brief 获取最新有效0x401；活动引用丢失时锁存通信超时。 */
static void chassis_update_reference(void)
{
  agv_motion_reference_t next;

  if((chassis_enabled == 0U) || (chassis_fault != 0U))
  {
    chassis_brake();
    chassis_clear_reference();
    return;
  }
  if(canfd_protocol_get_motion_reference(&next) != 0U)
  {
    if(chassis_reference_active == 0U)
      motion_reference_control_reset(&chassis_control);
    chassis_reference = next;
    chassis_reference_active = 1U;
    chassis_state = (next.kind == AGV_MOTION_KIND_HOLD) ?
                    CHASSIS_HOLD : CHASSIS_VELOCITY;
    if(next.kind == AGV_MOTION_KIND_HOLD) chassis_brake();
    return;
  }
  chassis_brake();
  if(chassis_reference_active != 0U)
  {
    /* 活动Reference消失表示TTL/租约/链路失效，必须锁存超时。 */
    chassis_clear_reference();
    chassis_fault_stop(CHASSIS_FAULT_TIMEOUT);
  }
  else
  {
    chassis_state = CHASSIS_IDLE;
  }
}

/** @brief 每20ms把有效Reference换算为轮目标并执行双轮速度PID。 */
static void chassis_speed_step(void)
{
  motion_reference_wheel_targets_t target;
  float yaw_rate;

  if((chassis_enabled == 0U) || (chassis_fault != 0U) ||
     (chassis_reference_active == 0U) ||
     (chassis_reference.kind == AGV_MOTION_KIND_HOLD))
  {
    motor_speed_control_set_targets(0.0f, 0.0f);
    motor_speed_control_update_now();
    chassis_brake();
    return;
  }
  /* 速度PID固定20ms运行，Yaw反馈由deg/s显式换算为mrad/s。 */
  yaw_rate = imu_heading_get_rate_dps() * AGV_MATH_PI_F * 1000.0f / 180.0f;
  if(motion_reference_control_step(&chassis_control, &chassis_reference,
      &chassis_limits, yaw_rate, 0.020f, &target) == 0U)
  {
    chassis_fault_stop(CHASSIS_FAULT_MOTION);
    return;
  }
  motor_speed_control_set_targets(target.left_rpm * CHASSIS_LEFT_SIGN,
                                  target.right_rpm * CHASSIS_RIGHT_SIGN);
  motor_speed_control_update_now();
  if(motor_speed_control_has_fault() != 0U)
    chassis_fault_stop(CHASSIS_FAULT_MOTION);
}

/** @brief 组装融合位姿、车体速度、角速度和加速度0x381状态。 */
static void chassis_send_odom(void)
{
  canfd_odom_status_t status;
  float linear = (chassis_left_rpm() + chassis_right_rpm()) *
      CHASSIS_WHEEL_CIRCUMFERENCE_MM / 120.0f;

  memset(&status, 0, sizeof(status));
  status.uptime_ms = chassis_uptime_ms;
  status.x_mm = chassis_round(chassis_x_mm);
  status.y_mm = chassis_round(chassis_y_mm);
  status.yaw_cdeg = chassis_round(chassis_heading.fused_yaw_deg * 100.0f);
  status.linear_mm_s = chassis_i16(chassis_round(linear));
  status.yaw_rate_cdeg_s = chassis_i16(chassis_round(
      imu_heading_get_rate_dps() * 100.0f));
  status.accel_x_mg = imu_heading_get_linear_accel_x_mg();
  status.accel_y_mg = imu_heading_get_linear_accel_y_mg();
  status.accel_z_mg = imu_heading_get_linear_accel_z_mg();
  status.valid_flags = (chassis_imu_fresh() != 0U) ? 0x000FU : 0x0003U;
  canfd_protocol_send_odom_status(&status);
}

/** @brief 组装应用状态、会话、故障和当前位姿0x181状态。 */
static void chassis_send_status(void)
{
  canfd_drive_status_t drive;

  memset(&drive, 0, sizeof(drive));
  drive.application_state = ((chassis_fault & (CHASSIS_FAULT_ESTOP |
      CHASSIS_FAULT_BUMPER)) != 0U) ? 6U :
      ((chassis_fault != 0U) ? 5U :
      ((chassis_reference_active != 0U) ? 3U :
       ((chassis_enabled != 0U) ? 2U : 1U)));
  drive.motion_state = ((chassis_fault & (CHASSIS_FAULT_ESTOP |
      CHASSIS_FAULT_BUMPER)) != 0U) ? 6U :
      ((chassis_fault != 0U) ? 7U :
      ((chassis_reference_active != 0U) ? 4U : 0U));
  drive.motion_mode = (chassis_reference_active != 0U) ? 3U : 0U;
  drive.flags = (canfd_protocol_is_session_valid() != 0U ? 0x01U : 0U) |
      (chassis_enabled != 0U ? 0x02U : 0U) |
      (chassis_imu_fresh() != 0U ? 0x08U : 0U);
  drive.command_id = (chassis_reference_active != 0U) ?
      chassis_reference.reference_id : 0U;
  drive.session_id = canfd_protocol_get_session_id();
  drive.fault_mask = chassis_fault;
  drive.warning_mask = power_supervisor_get_fault_mask();
  drive.x_mm = chassis_round(chassis_x_mm);
  drive.y_mm = chassis_round(chassis_y_mm);
  drive.yaw_cdeg = chassis_round(chassis_heading.fused_yaw_deg * 100.0f);
  canfd_protocol_send_drive_status(&drive);

  chassis_send_safety(0U);
}

/** @brief 更新防撞锁存并在首次事件出现时立即进入故障停车。 */
static void chassis_handle_bumper(void)
{
  bumper_safety_snapshot_t bumper;
  uint8_t changed = bumper_safety_step_10ms();
  bumper_safety_get_snapshot(&bumper);
  if((bumper.latched_mask != 0U) &&
     ((chassis_fault & CHASSIS_FAULT_BUMPER) == 0U))
  {
    chassis_fault_stop(CHASSIS_FAULT_BUMPER);
  }
  else if(changed != 0U)
  {
    chassis_send_safety(1U);
  }
}

/**
 * @brief 处理一个管理或本地安全命令。
 * @return RECEIVED/ACCEPTED/BUSY/REJECTED；连续运动量不经过本函数。
 */
motion_command_handle_result_t chassis_motion_controller_handle_command(
    const motion_command_t *command)
{
  if(command == NULL) return MOTION_HANDLE_REJECTED;
  switch(command->type)
  {
    case MOTION_CMD_ENABLE:
      if((chassis_enabled != 0U) || (chassis_fault != 0U) ||
         (power_supervisor_is_ready() == 0U) ||
         (chassis_imu_fresh() == 0U) ||
         (bumper_safety_can_clear() == 0U) ||
         ((command->source == MOTION_SOURCE_CANFD) &&
          (canfd_protocol_is_session_valid() == 0U)))
        return MOTION_HANDLE_REJECTED;
      chassis_brake();
      chassis_clear_reference();
      chassis_enabled = 1U;
      chassis_state = CHASSIS_IDLE;
      return MOTION_HANDLE_ACCEPTED;

    case MOTION_CMD_DISABLE:
    case MOTION_CMD_RELEASE:
      chassis_brake();
      chassis_clear_reference();
      chassis_enabled = 0U;
      return MOTION_HANDLE_ACCEPTED;

    case MOTION_CMD_STOP:
      /* 普通STOP保持会话和ENABLE，只清运动引用并进入主动制动HOLD。 */
      chassis_brake();
      chassis_clear_reference();
      if(chassis_fault == 0U) chassis_state = CHASSIS_HOLD;
      return MOTION_HANDLE_ACCEPTED;

    case MOTION_CMD_ESTOP: chassis_fault_stop(CHASSIS_FAULT_ESTOP); return MOTION_HANDLE_ACCEPTED;
    case MOTION_CMD_COMM_TIMEOUT: chassis_fault_stop(CHASSIS_FAULT_TIMEOUT); return MOTION_HANDLE_ACCEPTED;
    case MOTION_CMD_CAN_BUSOFF: chassis_fault_stop(CHASSIS_FAULT_CAN); return MOTION_HANDLE_ACCEPTED;
    case MOTION_CMD_POWER_FAULT: chassis_fault_stop(CHASSIS_FAULT_POWER); return MOTION_HANDLE_ACCEPTED;

    case MOTION_CMD_RESET_ODOM:
      if((chassis_reference_active != 0U) || (chassis_fault != 0U) ||
         (chassis_imu_fresh() == 0U)) return MOTION_HANDLE_REJECTED;
      chassis_reset_pose();
      chassis_state = CHASSIS_IDLE;
      return MOTION_HANDLE_ACCEPTED;

    case MOTION_CMD_CLEAR_FAULT:
      /* 原因必须先恢复；清除后仍保持DISABLED且不恢复旧Reference。 */
      if((chassis_reference_active != 0U) ||
         (((chassis_fault & CHASSIS_FAULT_CAN) != 0U) &&
          (canfd_protocol_is_bus_healthy() == 0U)) ||
         (chassis_imu_fresh() == 0U) ||
         (bumper_safety_can_clear() == 0U)) return MOTION_HANDLE_REJECTED;
      if(((chassis_fault & CHASSIS_FAULT_POWER) != 0U) &&
         ((power_supervisor_get_fault_mask() & POWER_FAULT_SEVERE_MASK) != 0U))
        return (power_supervisor_clear_fault() != 0U) ?
            MOTION_HANDLE_BUSY : MOTION_HANDLE_REJECTED;
      if(bumper_safety_clear_latch() == 0U) return MOTION_HANDLE_REJECTED;
      chassis_brake();
      chassis_clear_reference();
      motor_speed_control_clear_fault();
      chassis_fault = 0U;
      chassis_enabled = 0U;
      chassis_state = CHASSIS_IDLE;
      chassis_send_safety(1U);
      return MOTION_HANDLE_ACCEPTED;

    default: return MOTION_HANDLE_REJECTED;
  }
}

/** @brief 初始化驱动、传感器、融合、累计位姿和DISABLED安全状态。 */
void chassis_motion_controller_init(void)
{
  motor_speed_control_init();
  motor_speed_control_hold_brake();
  bumper_safety_init();
  chassis_heading_fusion_init(&chassis_heading, 2.0f, 120.0f);
  chassis_state = CHASSIS_IDLE;
  chassis_fault = 0U;
  chassis_enabled = 0U;
  chassis_reference_active = 0U;
  chassis_uptime_ms = 0U;
  chassis_x_mm = chassis_y_mm = 0.0f;
  memset(&chassis_reference, 0, sizeof(chassis_reference));
  motion_reference_control_reset(&chassis_control);
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  if(imu_heading_init() == BMI2_OK)
  {
    chassis_reset_pose();
  }
  else
  {
    chassis_fault = CHASSIS_FAULT_IMU;
    chassis_state = CHASSIS_FAULT;
    chassis_heading_left = chassis_odom_left = chassis_left_count();
    chassis_heading_right = chassis_odom_right = chassis_right_count();
  }
  DWT->CYCCNT = 0U;
  chassis_last_cycle = DWT->CYCCNT;
}

/** @brief 执行一次10ms底盘安全、感知、Reference、轮控和遥测更新。 */
void chassis_motion_controller_step_10ms(void)
{
  float dt;
  uint32_t power = 0U;

  /* 固定顺序：时间/IMU/防撞/位姿/Reference/速度/遥测/Power快照。 */
  chassis_uptime_ms += CHASSIS_PERIOD_MS;
  dt = chassis_sample_dt();
  chassis_handle_bumper();
  if(imu_heading_update(dt) != BMI2_OK)
  {
    if((chassis_fault & CHASSIS_FAULT_IMU) == 0U)
      chassis_fault_stop(CHASSIS_FAULT_IMU);
  }
  else if(chassis_update_pose(dt) == 0U)
  {
    chassis_fault_stop(CHASSIS_FAULT_MOTION);
  }
  chassis_update_reference();
  if(++chassis_speed_ticks >= CHASSIS_SPEED_TICKS)
  {
    chassis_speed_ticks = 0U;
    chassis_speed_step();
  }
  if(++chassis_odom_ticks >= CHASSIS_ODOM_TICKS)
  {
    chassis_odom_ticks = 0U;
    chassis_send_odom();
  }
  if(++chassis_status_ticks >= CHASSIS_STATUS_TICKS)
  {
    chassis_status_ticks = 0U;
    chassis_send_status();
  }
  if((chassis_enabled == 0U) && (chassis_fault == 0U) &&
     (chassis_reference_active == 0U) &&
     (motor_speed_control_get_pwm(MOTOR_LEFT) == 0) &&
     (motor_speed_control_get_pwm(MOTOR_RIGHT) == 0)) power |= CHASSIS_POWER_STATUS_ZERO_SAFE;
  if((chassis_fault & CHASSIS_FAULT_POWER) != 0U) power |= CHASSIS_POWER_STATUS_FAULT_LATCHED;
  chassis_power_snapshot += 4U;
  chassis_power_snapshot = (chassis_power_snapshot & ~3UL) | power;
}

/** @brief 返回当前内部底盘状态枚举值。 */
uint8_t chassis_motion_controller_get_state(void) { return (uint8_t)chassis_state; }
/** @brief 返回当前底盘锁存故障位图。 */
uint16_t chassis_motion_controller_get_fault_mask(void) { return chassis_fault; }
/** @brief 返回供PowerMonitor复核静止窗口的32位状态/epoch快照。 */
uint32_t chassis_motion_controller_get_power_status_snapshot(void) { return chassis_power_snapshot; }
/** @brief 判断底盘是否已经锁存Power故障。 */
uint8_t chassis_motion_controller_has_power_fault(void)
{
  return ((chassis_power_snapshot & CHASSIS_POWER_STATUS_FAULT_LATCHED) != 0U) ? 1U : 0U;
}
