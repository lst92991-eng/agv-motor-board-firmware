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

#include <math.h>
#include <stddef.h>

#define CHASSIS_HEADING_RAD_TO_DEG 57.2957795f
#define CHASSIS_HEADING_DEG_TO_RAD 0.01745329252f

/** @brief 返回单精度绝对值，避免融合核心依赖额外宏。 */
static float chassis_heading_abs_float(float value)
{
  return (value < 0.0f) ? -value : value;
}

/** @brief 将航向角归一化到-180～180度。 */
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

/** @brief 初始化互补融合参数并清零全部相对状态。 */
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

/** @brief 用陀螺预测并由有效编码器差分缓慢修正相对Yaw。 */
void chassis_heading_fusion_update(chassis_heading_fusion_t *fusion,
                                   float gyro_rate_dps,
                                   float encoder_delta_yaw_deg,
                                   float dt_s)
{
  float encoder_rate_dps;
  float predicted_fused_yaw_unwrapped_deg;
  float correction_gain;
  float correction_error_deg;

  if((fusion == NULL) || (dt_s < 0.001f) || (dt_s > 0.100f))
  {
    return;
  }

  predicted_fused_yaw_unwrapped_deg =
      fusion->fused_yaw_unwrapped_deg + (gyro_rate_dps * dt_s);

  encoder_rate_dps = encoder_delta_yaw_deg / dt_s;
  /* 编码器毛刺只影响本次低频修正，陀螺预测仍保持连续。 */
  if(chassis_heading_abs_float(encoder_rate_dps) <=
     fusion->max_encoder_rate_dps)
  {
    fusion->encoder_yaw_unwrapped_deg += encoder_delta_yaw_deg;
    /* 一阶互补增益由真实dt和时间常数计算，调度抖动不会改变物理带宽。 */
    correction_gain = dt_s / (fusion->correction_tau_s + dt_s);
    correction_error_deg = fusion->encoder_yaw_unwrapped_deg -
                           predicted_fused_yaw_unwrapped_deg;
    fusion->fused_yaw_unwrapped_deg =
        predicted_fused_yaw_unwrapped_deg +
        (correction_gain * correction_error_deg);
  }
  else
  {
    fusion->fused_yaw_unwrapped_deg = predicted_fused_yaw_unwrapped_deg;
    fusion->rejected_encoder_samples++;
  }

  fusion->fused_yaw_deg = chassis_heading_wrap_deg(
      fusion->fused_yaw_unwrapped_deg);

}

/** @brief 将左右轮有符号计数差换算为本周期编码器Yaw增量。 */
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

/** @brief 使用融合航向中点把有符号中心轮程积分到XY平面。 */
uint8_t chassis_signed_odometry_integrate(
    chassis_signed_odometry_pose_t *pose,
    int32_t left_delta_counts,
    int32_t right_delta_counts,
    float wheel_circumference_mm,
    float counts_per_revolution,
    float previous_yaw_deg,
    float current_yaw_deg)
{
  float center_delta_counts;
  float distance_mm;
  float yaw_delta_deg;
  float midpoint_yaw_rad;

  if((pose == NULL) || !(wheel_circumference_mm > 0.0f) ||
     !(counts_per_revolution > 0.0f))
  {
    return 0U;
  }
  center_delta_counts = ((float)left_delta_counts +
                         (float)right_delta_counts) * 0.5f;
  distance_mm = center_delta_counts * wheel_circumference_mm /
                counts_per_revolution;
  yaw_delta_deg = chassis_heading_wrap_deg(current_yaw_deg -
                                            previous_yaw_deg);
  /* 用本周期中点航向投影轮程，兼顾直线极限与连续圆弧。 */
  midpoint_yaw_rad = (previous_yaw_deg + (0.5f * yaw_delta_deg)) *
                     CHASSIS_HEADING_DEG_TO_RAD;
  pose->x_mm += distance_mm * cosf(midpoint_yaw_rad);
  pose->y_mm += distance_mm * sinf(midpoint_yaw_rad);
  return 1U;
}

/** @brief 清零相对航向、编码器约束状态和拒绝计数。 */
void chassis_heading_fusion_reset(chassis_heading_fusion_t *fusion)
{
  if(fusion == NULL)
  {
    return;
  }

  fusion->fused_yaw_deg = 0.0f;
  fusion->encoder_yaw_unwrapped_deg = 0.0f;
  fusion->fused_yaw_unwrapped_deg = 0.0f;
  fusion->rejected_encoder_samples = 0UL;
}
