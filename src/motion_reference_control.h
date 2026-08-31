#ifndef MOTION_REFERENCE_CONTROL_H
#define MOTION_REFERENCE_CONTROL_H

#include <stdint.h>

#include "agv/motion_contract.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint16_t max_linear_mm_s;
  uint16_t max_angular_mrad_s;
  uint16_t min_accel_mm_s2;
  uint16_t max_accel_mm_s2;
  uint16_t min_decel_mm_s2;
  uint16_t max_decel_mm_s2;
  float drive_track_mm;
  float wheel_circumference_mm;
  float max_wheel_rpm;
  float yaw_rate_feedback_gain;
} motion_reference_platform_limits_t;

typedef struct
{
  float left_rpm;
  float right_rpm;
  float common_scale;
  float slew_scale;
} motion_reference_wheel_targets_t;

typedef struct
{
  float left_rpm;
  float right_rpm;
} motion_reference_control_t;

/* Validate one reference and convert it to curvature-preserving wheel RPM. */
uint8_t motion_reference_control_compute(
    const agv_motion_reference_t *reference,
    const motion_reference_platform_limits_t *limits,
    motion_reference_wheel_targets_t *targets);

void motion_reference_control_reset(motion_reference_control_t *control);

/* Stateful wheel target update; intended to run on the 20 ms speed tick. */
uint8_t motion_reference_control_step(
    motion_reference_control_t *control,
    const agv_motion_reference_t *reference,
    const motion_reference_platform_limits_t *limits,
    float actual_yaw_rate_mrad_s,
    float dt_s,
    motion_reference_wheel_targets_t *targets);

#ifdef __cplusplus
}
#endif

#endif
