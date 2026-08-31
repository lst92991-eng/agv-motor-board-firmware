#include "motion_reference_control.h"

#include <limits.h>
#include <stddef.h>

#define MOTION_REFERENCE_NUMERIC_MAX_GEOMETRY_MM 1000000.0f
#define MOTION_REFERENCE_NUMERIC_MAX_WHEEL_RPM   1000000.0f

static float motion_reference_abs_float(float value)
{
  return (value < 0.0f) ? -value : value;
}

static uint32_t motion_reference_abs_i16(int16_t value)
{
  int32_t widened = value;

  return (uint32_t)((widened < 0) ? -widened : widened);
}

static uint8_t motion_reference_limits_are_valid(
    const motion_reference_platform_limits_t *limits)
{
  if((limits == NULL) ||
     (limits->max_linear_mm_s == 0U) ||
     (limits->max_linear_mm_s > INT16_MAX) ||
     (limits->max_angular_mrad_s == 0U) ||
     (limits->max_angular_mrad_s > INT16_MAX) ||
     (limits->min_accel_mm_s2 == 0U) ||
     (limits->min_accel_mm_s2 > limits->max_accel_mm_s2) ||
     (limits->min_decel_mm_s2 == 0U) ||
     (limits->min_decel_mm_s2 > limits->max_decel_mm_s2) ||
     !(limits->drive_track_mm > 0.0f) ||
     (limits->drive_track_mm > MOTION_REFERENCE_NUMERIC_MAX_GEOMETRY_MM) ||
     !(limits->wheel_circumference_mm > 0.0f) ||
     (limits->wheel_circumference_mm >
      MOTION_REFERENCE_NUMERIC_MAX_GEOMETRY_MM) ||
     !(limits->max_wheel_rpm > 0.0f) ||
     (limits->max_wheel_rpm > MOTION_REFERENCE_NUMERIC_MAX_WHEEL_RPM) ||
     !(limits->yaw_rate_feedback_gain >= 0.0f) ||
     (limits->yaw_rate_feedback_gain > 10.0f))
  {
    return 0U;
  }
  return 1U;
}

static uint8_t motion_reference_body_is_within_limits(
    const agv_motion_reference_t *reference,
    const motion_reference_platform_limits_t *limits)
{
  if((agv_motion_reference_is_valid(reference) == 0U) ||
     ((reference->kind == (uint8_t)AGV_MOTION_KIND_BODY_VELOCITY) &&
      ((motion_reference_abs_i16(reference->linear_mm_s) >
        limits->max_linear_mm_s) ||
       (motion_reference_abs_i16(reference->angular_mrad_s) >
        limits->max_angular_mrad_s) ||
       (reference->accel_mm_s2 < limits->min_accel_mm_s2) ||
       (reference->accel_mm_s2 > limits->max_accel_mm_s2) ||
       (reference->decel_mm_s2 < limits->min_decel_mm_s2) ||
       (reference->decel_mm_s2 > limits->max_decel_mm_s2))))
  {
    return 0U;
  }
  return 1U;
}

static void motion_reference_compute_body(
    float linear_mm_s,
    float angular_mrad_s,
    const motion_reference_platform_limits_t *limits,
    motion_reference_wheel_targets_t *targets)
{
  float turn_speed_mm_s;
  float maximum_rpm;

  targets->common_scale = 1.0f;
  targets->slew_scale = 1.0f;
  turn_speed_mm_s = (limits->drive_track_mm * angular_mrad_s) / 2000.0f;
  targets->left_rpm = ((linear_mm_s - turn_speed_mm_s) * 60.0f) /
                      limits->wheel_circumference_mm;
  targets->right_rpm = ((linear_mm_s + turn_speed_mm_s) * 60.0f) /
                       limits->wheel_circumference_mm;
  maximum_rpm = motion_reference_abs_float(targets->left_rpm);
  if(motion_reference_abs_float(targets->right_rpm) > maximum_rpm)
  {
    maximum_rpm = motion_reference_abs_float(targets->right_rpm);
  }
  if(maximum_rpm > limits->max_wheel_rpm)
  {
    targets->common_scale = limits->max_wheel_rpm / maximum_rpm;
    targets->left_rpm *= targets->common_scale;
    targets->right_rpm *= targets->common_scale;
  }
}

uint8_t motion_reference_control_compute(
    const agv_motion_reference_t *reference,
    const motion_reference_platform_limits_t *limits,
    motion_reference_wheel_targets_t *targets)
{
  if((targets == NULL) ||
     (motion_reference_limits_are_valid(limits) == 0U) ||
     (motion_reference_body_is_within_limits(reference, limits) == 0U))
  {
    return 0U;
  }

  if(reference->kind == (uint8_t)AGV_MOTION_KIND_HOLD)
  {
    targets->left_rpm = 0.0f;
    targets->right_rpm = 0.0f;
    targets->common_scale = 1.0f;
    targets->slew_scale = 1.0f;
    return 1U;
  }

  motion_reference_compute_body((float)reference->linear_mm_s,
                                (float)reference->angular_mrad_s,
                                limits, targets);
  return 1U;
}

void motion_reference_control_reset(motion_reference_control_t *control)
{
  if(control != NULL)
  {
    control->left_rpm = 0.0f;
    control->right_rpm = 0.0f;
  }
}

static float motion_reference_wheel_step_limit(float current_rpm,
                                               float target_rpm,
                                               float accel_step_rpm,
                                               float decel_step_rpm)
{
  return (((current_rpm * target_rpm) >= 0.0f) &&
          (motion_reference_abs_float(target_rpm) >
           motion_reference_abs_float(current_rpm))) ?
         accel_step_rpm : decel_step_rpm;
}

uint8_t motion_reference_control_step(
    motion_reference_control_t *control,
    const agv_motion_reference_t *reference,
    const motion_reference_platform_limits_t *limits,
    float actual_yaw_rate_mrad_s,
    float dt_s,
    motion_reference_wheel_targets_t *targets)
{
  motion_reference_wheel_targets_t desired;
  float corrected_angular_mrad_s;
  float accel_step_rpm;
  float decel_step_rpm;
  float left_delta_rpm;
  float right_delta_rpm;
  float left_step_limit_rpm;
  float right_step_limit_rpm;
  float slew_scale = 1.0f;
  uint8_t reversing;

  if((control == NULL) || (targets == NULL) ||
     (motion_reference_limits_are_valid(limits) == 0U) ||
     (motion_reference_body_is_within_limits(reference, limits) == 0U) ||
     !(dt_s > 0.0f) || (dt_s > 1.0f) ||
     !((actual_yaw_rate_mrad_s >= -1000000.0f) &&
       (actual_yaw_rate_mrad_s <= 1000000.0f)))
  {
    return 0U;
  }
  if(reference->kind == (uint8_t)AGV_MOTION_KIND_HOLD)
  {
    motion_reference_control_reset(control);
    targets->left_rpm = 0.0f;
    targets->right_rpm = 0.0f;
    targets->common_scale = 1.0f;
    targets->slew_scale = 1.0f;
    return 1U;
  }

  corrected_angular_mrad_s = (float)reference->angular_mrad_s +
      (limits->yaw_rate_feedback_gain *
       ((float)reference->angular_mrad_s - actual_yaw_rate_mrad_s));
  if(corrected_angular_mrad_s > (float)limits->max_angular_mrad_s)
  {
    corrected_angular_mrad_s = (float)limits->max_angular_mrad_s;
  }
  else if(corrected_angular_mrad_s <
          -(float)limits->max_angular_mrad_s)
  {
    corrected_angular_mrad_s = -(float)limits->max_angular_mrad_s;
  }
  motion_reference_compute_body((float)reference->linear_mm_s,
                                corrected_angular_mrad_s,
                                limits, &desired);

  reversing = (((control->left_rpm * desired.left_rpm) < 0.0f) ||
               ((control->right_rpm * desired.right_rpm) < 0.0f)) ? 1U : 0U;
  if(reversing != 0U)
  {
    desired.left_rpm = 0.0f;
    desired.right_rpm = 0.0f;
  }
  accel_step_rpm = ((float)reference->accel_mm_s2 * dt_s * 60.0f) /
                   limits->wheel_circumference_mm;
  decel_step_rpm = ((float)reference->decel_mm_s2 * dt_s * 60.0f) /
                   limits->wheel_circumference_mm;
  left_delta_rpm = desired.left_rpm - control->left_rpm;
  right_delta_rpm = desired.right_rpm - control->right_rpm;
  left_step_limit_rpm = motion_reference_wheel_step_limit(
      control->left_rpm, desired.left_rpm,
      accel_step_rpm, decel_step_rpm);
  right_step_limit_rpm = motion_reference_wheel_step_limit(
      control->right_rpm, desired.right_rpm,
      accel_step_rpm, decel_step_rpm);
  if((motion_reference_abs_float(left_delta_rpm) > left_step_limit_rpm) &&
     ((left_step_limit_rpm /
       motion_reference_abs_float(left_delta_rpm)) < slew_scale))
  {
    slew_scale = left_step_limit_rpm /
                 motion_reference_abs_float(left_delta_rpm);
  }
  if((motion_reference_abs_float(right_delta_rpm) > right_step_limit_rpm) &&
     ((right_step_limit_rpm /
       motion_reference_abs_float(right_delta_rpm)) < slew_scale))
  {
    slew_scale = right_step_limit_rpm /
                 motion_reference_abs_float(right_delta_rpm);
  }
  control->left_rpm += left_delta_rpm * slew_scale;
  control->right_rpm += right_delta_rpm * slew_scale;
  targets->left_rpm = control->left_rpm;
  targets->right_rpm = control->right_rpm;
  targets->common_scale = desired.common_scale;
  targets->slew_scale = slew_scale;
  return 1U;
}
