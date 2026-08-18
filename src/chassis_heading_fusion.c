/**
  ****************************************************************************
  * @file     chassis_heading_fusion.c
  * @brief    底盘陀螺仪与双编码器相对航向互补融合
  *
  * 陀螺仪负责快速转向响应，左右轮差分里程对低频漂移提供约束。两种来源
  * 都是相对量，不能替代磁条、RFID或定位系统提供的绝对位置基准。
  ****************************************************************************
  */

#include "chassis_heading_fusion.h"

#include <stddef.h>

#define CHASSIS_HEADING_RAD_TO_DEG 57.2957795f

static float chassis_heading_abs_float(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float chassis_heading_wrap_deg(float angle_deg)
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

void chassis_heading_fusion_init(chassis_heading_fusion_t *fusion,
                                 float correction_tau_s,
                                 float max_encoder_rate_dps)
{
  if(fusion == NULL)
  {
    return;
  }

  fusion->correction_tau_s =
      (correction_tau_s >= 0.1f) ? correction_tau_s : 0.1f;
  fusion->max_encoder_rate_dps =
      (max_encoder_rate_dps >= 1.0f) ? max_encoder_rate_dps : 1.0f;
  chassis_heading_fusion_reset(fusion);
}

void chassis_heading_fusion_update(chassis_heading_fusion_t *fusion,
                                   float gyro_rate_dps,
                                   float encoder_delta_yaw_deg,
                                   float dt_s)
{
  float encoder_rate_dps;
  float predicted_fused_yaw_deg;
  float correction_gain;
  float correction_error_deg;

  if((fusion == NULL) || (dt_s < 0.001f) || (dt_s > 0.100f))
  {
    return;
  }

  fusion->gyro_rate_dps = gyro_rate_dps;
  fusion->gyro_yaw_deg = chassis_heading_wrap_deg(
      fusion->gyro_yaw_deg + (gyro_rate_dps * dt_s));
  predicted_fused_yaw_deg = chassis_heading_wrap_deg(
      fusion->fused_yaw_deg + (gyro_rate_dps * dt_s));

  encoder_rate_dps = encoder_delta_yaw_deg / dt_s;
  if(chassis_heading_abs_float(encoder_rate_dps) <=
     fusion->max_encoder_rate_dps)
  {
    fusion->encoder_rate_dps = encoder_rate_dps;
    fusion->encoder_yaw_deg = chassis_heading_wrap_deg(
        fusion->encoder_yaw_deg + encoder_delta_yaw_deg);
    correction_gain = dt_s / (fusion->correction_tau_s + dt_s);
    correction_error_deg = chassis_heading_wrap_deg(
        fusion->encoder_yaw_deg - predicted_fused_yaw_deg);
    fusion->fused_yaw_deg = chassis_heading_wrap_deg(
        predicted_fused_yaw_deg + (correction_gain * correction_error_deg));
  }
  else
  {
    fusion->encoder_rate_dps = 0.0f;
    fusion->fused_yaw_deg = predicted_fused_yaw_deg;
    fusion->rejected_encoder_samples++;
  }

  fusion->disagreement_deg = chassis_heading_wrap_deg(
      fusion->gyro_yaw_deg - fusion->encoder_yaw_deg);
}

float chassis_heading_fusion_encoder_delta_deg(
    int32_t left_delta_counts,
    int32_t right_delta_counts,
    float wheel_circumference_mm,
    float counts_per_revolution,
    float drive_track_mm)
{
  if((wheel_circumference_mm <= 0.0f) ||
     (counts_per_revolution <= 0.0f) ||
     (drive_track_mm <= 0.0f))
  {
    return 0.0f;
  }

  return ((((float)right_delta_counts - (float)left_delta_counts) *
           wheel_circumference_mm) /
          (counts_per_revolution * drive_track_mm)) *
         CHASSIS_HEADING_RAD_TO_DEG;
}

void chassis_heading_fusion_reset(chassis_heading_fusion_t *fusion)
{
  if(fusion == NULL)
  {
    return;
  }

  fusion->gyro_yaw_deg = 0.0f;
  fusion->encoder_yaw_deg = 0.0f;
  fusion->fused_yaw_deg = 0.0f;
  fusion->gyro_rate_dps = 0.0f;
  fusion->encoder_rate_dps = 0.0f;
  fusion->disagreement_deg = 0.0f;
  fusion->rejected_encoder_samples = 0UL;
}
