/**
 * @file imu_heading.h
 * @brief BMI270固定安装Z轴Yaw角速度和线加速度接口。
 */

#ifndef IMU_HEADING_H
#define IMU_HEADING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化BMI270，并在车辆静止时校准Z轴零偏、噪声和重力基线。 */
int8_t imu_heading_init(void);

/* 读取一次新DRDY样本并更新滤波量；dt_s为调用间隔，单位秒。 */
int8_t imu_heading_update(float dt_s);

/* 读取经过零偏修正的航向角速度，单位为度每秒。 */
float imu_heading_get_rate_dps(void);

/* 读取扣除上电静止重力基线后的车体水平加速度，单位为 mg。 */
int16_t imu_heading_get_linear_accel_x_mg(void);
/** 返回Y轴线加速度，单位mg。 */
int16_t imu_heading_get_linear_accel_y_mg(void);
/** 返回Z轴线加速度，单位mg。 */
int16_t imu_heading_get_linear_accel_z_mg(void);

/* 返回最近一次初始化或采样结果。 */
int8_t imu_heading_get_status(void);

/* 返回自最近一次真实GYR DRDY样本以来的本地累计时间。 */
uint16_t imu_heading_get_gyro_age_ms(void);

#ifdef __cplusplus
}
#endif

#endif
