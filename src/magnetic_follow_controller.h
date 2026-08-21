#ifndef MAGNETIC_FOLLOW_CONTROLLER_H
#define MAGNETIC_FOLLOW_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint32_t receive_time_ms;
  uint16_t sequence;
  uint16_t age_ms;
  uint16_t active_mask;
  int16_t offset_x10mm;
  uint16_t strength;
  uint16_t signal[8];
  uint8_t status;
  uint8_t flags;
  uint8_t protocol_valid;
} magnetic_follow_sample_t;

typedef enum
{
  MAGNETIC_FOLLOW_SPEED_STANDARD = 0,
  MAGNETIC_FOLLOW_SPEED_FAST_STRAIGHT,
  MAGNETIC_FOLLOW_SPEED_CURVE_SUSPECT,
  MAGNETIC_FOLLOW_SPEED_CURVE_TRACK,
  MAGNETIC_FOLLOW_SPEED_CURVE_EMERGENCY
} magnetic_follow_speed_phase_t;

typedef struct
{
  int16_t target_offset_x10mm;
  int16_t offset_x10mm;
  uint16_t strength;
  uint16_t age_ms;
  uint16_t last_sequence;
  uint32_t last_receive_time_ms;
  float last_offset_error_mm;
  float offset_rate_mm_s;
  float yaw_rate_target_dps;
  float speed_limit_rpm;
  uint16_t curve_evidence_ticks;
  uint16_t straight_evidence_ticks;
  uint8_t allow_multi_peak;
  uint8_t sequence_valid;
  uint8_t adaptive_high_speed;
  uint8_t speed_phase;
} magnetic_follow_controller_t;

/* 清空控制器历史，不保留上一条磁路任务的偏移变化率。 */
void magnetic_follow_controller_reset(
    magnetic_follow_controller_t *controller);

/* 检查时效、flags、强度、偏移范围和真正分离的多峰。 */
uint8_t magnetic_follow_sample_is_trackable(
    const magnetic_follow_sample_t *sample);

/* 曲线模式允许多峰，但仍使用磁导航板对全部通道计算的整体偏移。 */
uint8_t magnetic_follow_sample_is_trackable_for_mode(
    const magnetic_follow_sample_t *sample,
    uint8_t allow_multi_peak);

/* 用任务首帧建立变化率基准和初始限速，不把首帧偏移改写成新零点。 */
void magnetic_follow_controller_start(
    magnetic_follow_controller_t *controller,
    int16_t target_offset_x10mm,
    uint8_t allow_multi_peak,
    const magnetic_follow_sample_t *sample,
    float requested_run_rpm);

/* 对已通过质量检查的新样本更新偏移PD外环和动态限速。 */
void magnetic_follow_controller_update(
    magnetic_follow_controller_t *controller,
    const magnetic_follow_sample_t *sample,
    float requested_run_rpm);

/* 将目标Yaw角速度换算为差速轮速前馈，并用陀螺仪角速度闭环补偿。 */
float magnetic_follow_controller_wheel_correction_rpm(
    const magnetic_follow_controller_t *controller,
    float actual_yaw_rate_dps,
    float drive_track_mm,
    float wheel_circumference_mm);

/* 捕获阶段±6RPM；直线稳定循迹±8RPM；曲线稳定循迹±11RPM。 */
float magnetic_follow_controller_correction_limit_rpm(
    uint8_t capture_phase,
    uint8_t allow_multi_peak);

#ifdef __cplusplus
}
#endif

#endif
