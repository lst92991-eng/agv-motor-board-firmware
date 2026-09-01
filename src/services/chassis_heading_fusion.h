/**
 * @file chassis_heading_fusion.h
 * @brief 陀螺预测、编码器差分修正和有符号里程计接口。
 */

#ifndef CHASSIS_HEADING_FUSION_H
#define CHASSIS_HEADING_FUSION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  /* wrapped值用于状态上报，unwrapped值用于跨±180°连续融合。 */
  float fused_yaw_deg;
  float encoder_yaw_unwrapped_deg;
  float fused_yaw_unwrapped_deg;
  float correction_tau_s;
  float max_encoder_rate_dps;
  uint32_t rejected_encoder_samples;
} chassis_heading_fusion_t;

typedef struct
{
  float x_mm;
  float y_mm;
} chassis_signed_odometry_pose_t;

/* 清零相对航向；correction_tau_s 越小，编码器低频约束越强。 */
void chassis_heading_fusion_init(chassis_heading_fusion_t *fusion,
                                 float correction_tau_s,
                                 float max_encoder_rate_dps);

/* 输入本周期三轴投影后的陀螺仪角速度和编码器差分角度。 */
void chassis_heading_fusion_update(chassis_heading_fusion_t *fusion,
                                   float gyro_rate_dps,
                                   float encoder_delta_yaw_deg,
                                   float dt_s);

/* 将已归一化为前进为正的左右编码器增量换算为车体Yaw增量。 */
float chassis_heading_fusion_encoder_delta_deg(
    int32_t left_delta_counts,
    int32_t right_delta_counts,
    float wheel_circumference_mm,
    float counts_per_revolution,
    float drive_track_mm);

/* 使用前后融合航向的中点积分有符号轮程，避免末端航向Euler偏差。 */
uint8_t chassis_signed_odometry_integrate(
    chassis_signed_odometry_pose_t *pose,
    int32_t left_delta_counts,
    int32_t right_delta_counts,
    float wheel_circumference_mm,
    float counts_per_revolution,
    float previous_yaw_deg,
    float current_yaw_deg);

/* 清零相对航向与拒绝计数；不会重置硬件编码器。 */
void chassis_heading_fusion_reset(chassis_heading_fusion_t *fusion);

#ifdef __cplusplus
}
#endif

#endif
