/**
 * @file motion_reference_control.h
 * @brief 差速几何、平台硬限制和双轮目标斜率状态。
 */

#ifndef MOTION_REFERENCE_CONTROL_H
#define MOTION_REFERENCE_CONTROL_H

#include <stdint.h>

#include "agv/motion_contract.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  /* 业务Reference只能收紧这些平台硬上限，不能放宽。 */
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

/* 校验Reference并换算为保持曲率的左右轮目标RPM，不修改历史斜率状态。 */
uint8_t motion_reference_control_compute(
    const agv_motion_reference_t *reference,
    const motion_reference_platform_limits_t *limits,
    motion_reference_wheel_targets_t *targets);

/** 清零左右轮斜率历史，不修改平台配置。 */
void motion_reference_control_reset(motion_reference_control_t *control);

/* 20ms有状态更新：角速度反馈、共同限幅、反向归零和加减速斜率。 */
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
