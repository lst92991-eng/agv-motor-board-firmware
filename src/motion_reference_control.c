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
     (limits->max_wheel_rpm > MOTION_REFERENCE_NUMERIC_MAX_WHEEL_RPM))
  {
    return 0U;
  }
  return 1U;
}

uint8_t motion_reference_control_compute(
    const agv_motion_reference_t *reference,
    const motion_reference_platform_limits_t *limits,
    motion_reference_wheel_targets_t *targets)
{
  float turn_speed_mm_s;
  float left_rpm;
  float right_rpm;
  float maximum_rpm;
  float common_scale = 1.0f;

  if((targets == NULL) ||
     (motion_reference_limits_are_valid(limits) == 0U) ||
     (agv_motion_reference_is_valid(reference) == 0U))
  {
    return 0U;
  }

  if(reference->kind == (uint8_t)AGV_MOTION_KIND_HOLD)
  {
    targets->left_rpm = 0.0f;
    targets->right_rpm = 0.0f;
    targets->common_scale = 1.0f;
    return 1U;
  }

  if((motion_reference_abs_i16(reference->linear_mm_s) >
      limits->max_linear_mm_s) ||
     (motion_reference_abs_i16(reference->angular_mrad_s) >
      limits->max_angular_mrad_s) ||
     (reference->accel_mm_s2 < limits->min_accel_mm_s2) ||
     (reference->accel_mm_s2 > limits->max_accel_mm_s2) ||
     (reference->decel_mm_s2 < limits->min_decel_mm_s2) ||
     (reference->decel_mm_s2 > limits->max_decel_mm_s2))
  {
    return 0U;
  }

  turn_speed_mm_s = (limits->drive_track_mm *
                     (float)reference->angular_mrad_s) / 2000.0f;
  left_rpm = (((float)reference->linear_mm_s - turn_speed_mm_s) *
              60.0f) / limits->wheel_circumference_mm;
  right_rpm = (((float)reference->linear_mm_s + turn_speed_mm_s) *
               60.0f) / limits->wheel_circumference_mm;
  maximum_rpm = motion_reference_abs_float(left_rpm);
  if(motion_reference_abs_float(right_rpm) > maximum_rpm)
  {
    maximum_rpm = motion_reference_abs_float(right_rpm);
  }
  if(maximum_rpm > limits->max_wheel_rpm)
  {
    common_scale = limits->max_wheel_rpm / maximum_rpm;
    left_rpm *= common_scale;
    right_rpm *= common_scale;
  }

  targets->left_rpm = left_rpm;
  targets->right_rpm = right_rpm;
  targets->common_scale = common_scale;
  return 1U;
}
