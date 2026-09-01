/**
 * @file motion_reference_control.c
 * @brief 车体v/omega到双轮RPM的统一差速执行与斜率限制。
 *
 * 所有上层来源共用本实现。单轮超限时按同一比例缩放左右轮以保持曲率；换向
 * 必须先把两轮目标降到零，避免一侧提前反向改变车体运动方向。
 */

#include "motion_reference_control.h"

#include <limits.h>
#include <stddef.h>

/** @brief 返回单精度绝对值。 */
static float motion_abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

/** @brief 将车体或轮速中间量限制到对称区间。 */
static float motion_clamp(float value, float limit)
{
  if(value > limit) return limit;
  if(value < -limit) return -limit;
  return value;
}

/** @brief 校验平台几何、速度、斜率和Yaw反馈配置是否自洽。 */
static uint8_t motion_limits_valid(
    const motion_reference_platform_limits_t *limit)
{
  return ((limit != NULL) &&
      (limit->max_linear_mm_s > 0U) &&
      (limit->max_linear_mm_s <= INT16_MAX) &&
      (limit->max_angular_mrad_s > 0U) &&
      (limit->max_angular_mrad_s <= INT16_MAX) &&
      (limit->min_accel_mm_s2 > 0U) &&
      (limit->min_accel_mm_s2 <= limit->max_accel_mm_s2) &&
      (limit->min_decel_mm_s2 > 0U) &&
      (limit->min_decel_mm_s2 <= limit->max_decel_mm_s2) &&
      (limit->drive_track_mm > 0.0f) &&
      (limit->wheel_circumference_mm > 0.0f) &&
      (limit->max_wheel_rpm > 0.0f) &&
      (limit->yaw_rate_feedback_gain >= 0.0f) &&
      (limit->yaw_rate_feedback_gain <= 10.0f)) ? 1U : 0U;
}

/** @brief 校验Reference字段并确认业务请求没有放宽平台上限。 */
static uint8_t motion_reference_valid(
    const agv_motion_reference_t *reference,
    const motion_reference_platform_limits_t *limit)
{
  int32_t linear;
  int32_t angular;

  if((motion_limits_valid(limit) == 0U) ||
     (agv_motion_reference_is_valid(reference) == 0U)) return 0U;
  if(reference->kind == AGV_MOTION_KIND_HOLD) return 1U;
  linear = reference->linear_mm_s;
  angular = reference->angular_mrad_s;
  if(linear < 0) linear = -linear;
  if(angular < 0) angular = -angular;
  return (((uint32_t)linear <= limit->max_linear_mm_s) &&
      ((uint32_t)angular <= limit->max_angular_mrad_s) &&
      (reference->accel_mm_s2 >= limit->min_accel_mm_s2) &&
      (reference->accel_mm_s2 <= limit->max_accel_mm_s2) &&
      (reference->decel_mm_s2 >= limit->min_decel_mm_s2) &&
      (reference->decel_mm_s2 <= limit->max_decel_mm_s2)) ? 1U : 0U;
}

/** @brief 按差速逆运动学计算左右轮RPM并执行共同轮速缩放。 */
static void motion_wheels(float linear_mm_s, float angular_mrad_s,
                          const motion_reference_platform_limits_t *limit,
                          motion_reference_wheel_targets_t *target)
{
  float turn = limit->drive_track_mm * angular_mrad_s / 2000.0f;
  float maximum;

  /* base_link约定X向前、左转为正：左轮减、右轮加。 */
  target->left_rpm = (linear_mm_s - turn) * 60.0f /
      limit->wheel_circumference_mm;
  target->right_rpm = (linear_mm_s + turn) * 60.0f /
      limit->wheel_circumference_mm;
  maximum = motion_abs(target->left_rpm);
  if(motion_abs(target->right_rpm) > maximum)
    maximum = motion_abs(target->right_rpm);
  target->common_scale = (maximum > limit->max_wheel_rpm) ?
      limit->max_wheel_rpm / maximum : 1.0f;
  /* 共同缩放保留omega/v，禁止分别clamp左右轮导致路径曲率改变。 */
  target->left_rpm *= target->common_scale;
  target->right_rpm *= target->common_scale;
  target->slew_scale = 1.0f;
}

/** @brief 无状态计算一个Reference对应的曲率保持轮速。 */
uint8_t motion_reference_control_compute(
    const agv_motion_reference_t *reference,
    const motion_reference_platform_limits_t *limit,
    motion_reference_wheel_targets_t *target)
{
  if((target == NULL) ||
     (motion_reference_valid(reference, limit) == 0U)) return 0U;
  if(reference->kind == AGV_MOTION_KIND_HOLD)
  {
    target->left_rpm = target->right_rpm = 0.0f;
    target->common_scale = target->slew_scale = 1.0f;
  }
  else
  {
    motion_wheels((float)reference->linear_mm_s,
        (float)reference->angular_mrad_s, limit, target);
  }
  return 1U;
}

/** @brief 清零左右轮斜率历史。 */
void motion_reference_control_reset(motion_reference_control_t *control)
{
  if(control != NULL) control->left_rpm = control->right_rpm = 0.0f;
}

/** @brief 根据同向增速或其他情况选择本周期加速度/减速度步长。 */
static float motion_step_limit(float current, float target,
                               float accel, float decel)
{
  return (((current * target) >= 0.0f) &&
          (motion_abs(target) > motion_abs(current))) ? accel : decel;
}

/** @brief 执行20ms角速度反馈、差速、换向归零和共同斜率更新。 */
uint8_t motion_reference_control_step(
    motion_reference_control_t *control,
    const agv_motion_reference_t *reference,
    const motion_reference_platform_limits_t *limit,
    float actual_yaw_rate_mrad_s,
    float dt_s,
    motion_reference_wheel_targets_t *target)
{
  motion_reference_wheel_targets_t desired;
  float angular;
  float accel;
  float decel;
  float left_delta;
  float right_delta;
  float scale = 1.0f;
  float candidate;

  if((control == NULL) || (target == NULL) ||
     (motion_reference_valid(reference, limit) == 0U) ||
     (dt_s <= 0.0f) || (dt_s > 1.0f) ||
     (actual_yaw_rate_mrad_s < -1000000.0f) ||
     (actual_yaw_rate_mrad_s > 1000000.0f)) return 0U;
  if(reference->kind == AGV_MOTION_KIND_HOLD)
  {
    motion_reference_control_reset(control);
    return motion_reference_control_compute(reference, limit, target);
  }

  angular = (float)reference->angular_mrad_s +
      limit->yaw_rate_feedback_gain *
      ((float)reference->angular_mrad_s - actual_yaw_rate_mrad_s);
  /* 角速度反馈只补偿执行偏差，不改变上层路径或任务语义。 */
  angular = motion_clamp(angular, (float)limit->max_angular_mrad_s);
  motion_wheels((float)reference->linear_mm_s, angular, limit, &desired);
  if(((control->left_rpm * desired.left_rpm) < 0.0f) ||
     ((control->right_rpm * desired.right_rpm) < 0.0f))
    /* 任一轮需要换向时两轮目标先共同归零。 */
    desired.left_rpm = desired.right_rpm = 0.0f;

  accel = reference->accel_mm_s2 * dt_s * 60.0f /
      limit->wheel_circumference_mm;
  decel = reference->decel_mm_s2 * dt_s * 60.0f /
      limit->wheel_circumference_mm;
  left_delta = desired.left_rpm - control->left_rpm;
  right_delta = desired.right_rpm - control->right_rpm;
  if(motion_abs(left_delta) > 0.0f)
  {
    candidate = motion_step_limit(control->left_rpm, desired.left_rpm,
                                  accel, decel) / motion_abs(left_delta);
    if(candidate < scale) scale = candidate;
  }
  if(motion_abs(right_delta) > 0.0f)
  {
    candidate = motion_step_limit(control->right_rpm, desired.right_rpm,
                                  accel, decel) / motion_abs(right_delta);
    if(candidate < scale) scale = candidate;
  }
  if(scale > 1.0f) scale = 1.0f;
  /* 使用两轴中更严格的比例，使本周期左右目标沿同一直线插值。 */
  control->left_rpm += left_delta * scale;
  control->right_rpm += right_delta * scale;
  *target = desired;
  target->left_rpm = control->left_rpm;
  target->right_rpm = control->right_rpm;
  target->slew_scale = scale;
  return 1U;
}
