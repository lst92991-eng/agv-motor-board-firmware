#ifndef IMU_HEADING_H
#define IMU_HEADING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 BMI270，并在车辆静止时校准航向角速度零偏。 */
int8_t imu_heading_init(void);

/* 读取一次陀螺仪并更新航向角，dt_s 为本次积分时间，单位为秒。 */
int8_t imu_heading_update(float dt_s);

/* 将当前方向设为零度航向。 */
void imu_heading_reset(void);

/* 读取积分得到的相对航向角，范围为 -180~180 度。 */
float imu_heading_get_yaw_deg(void);

/* 读取经过零偏修正的航向角速度，单位为度每秒。 */
float imu_heading_get_rate_dps(void);

/* 读取减去上电零偏但尚未软件滤波的角速度，供调试使用。 */
float imu_heading_get_raw_rate_dps(void);

/* 读取5Hz一阶低通后的角速度，尚未经过静止死区。 */
float imu_heading_get_filtered_rate_dps(void);

/* 读取本次上电根据静止噪声计算出的动态死区。 */
float imu_heading_get_deadband_dps(void);

/* 返回最近一次初始化或采样结果。 */
int8_t imu_heading_get_status(void);

#ifdef __cplusplus
}
#endif

#endif
