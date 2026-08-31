#ifndef MAGNETIC_FOLLOW_CONTROLLER_H
#define MAGNETIC_FOLLOW_CONTROLLER_H

#include <stdint.h>

#include "agv/motion_contract.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 已由CAN解码层转换完成的磁导航样本。
 * receive_time_ms使用电机板本地单调时钟；offset_x10mm的单位为0.1 mm。
 */
typedef struct
{
  uint32_t receive_time_ms;  // 电机板收到该帧的本地单调时间，单位ms
  uint16_t sequence;         // 磁导航板样本序号，用于识别新样本或冻结
  uint16_t age_ms;           // 样本在发送端累计的年龄，单位ms
  uint16_t active_mask;      // 低8位对应8个超过有效阈值的磁传感通道
  int16_t offset_x10mm;      // 相对磁条中心的带符号横向偏移，0.1mm/LSB
  uint16_t strength;         // 当前8通道磁场的综合强度
  uint16_t signal[8];        // 按CAN协议通道顺序排列的8路独立磁场强度
  uint8_t status;            // 磁导航板状态码；0表示当前样本不可用
  uint8_t flags;             // 协议状态标志；低三位必须全部有效
  uint8_t protocol_valid;    // 本地字段检查结果；1表示状态与保留位合法
} magnetic_follow_sample_t;

typedef enum
{
  MAGNETIC_FOLLOW_SPEED_STANDARD = 0,        // 普通循迹，按横向偏差连续限速
  MAGNETIC_FOLLOW_SPEED_FAST_STRAIGHT,       // 已稳定居中，允许提升直线速度
  MAGNETIC_FOLLOW_SPEED_CURVE_SUSPECT,       // 出现弯道迹象，立即采用疑似弯道限速
  MAGNETIC_FOLLOW_SPEED_CURVE_TRACK,         // 低速捕获磁条或跟踪已确认弯道
  MAGNETIC_FOLLOW_SPEED_CURVE_EMERGENCY      // 横向误差过大，采用最低应急速度
} magnetic_follow_speed_phase_t;

/*
 * 单次磁循迹任务的运行状态。调用者为每个活动任务持有一个实例；任务开始后
 * 只能通过本模块函数修改，外部可读取这些字段生成诊断快照。
 */
typedef struct
{
  int16_t target_offset_x10mm;        // 本次任务要求保持的横向偏移，0.1mm/LSB
  int16_t offset_x10mm;               // 最近一次磁样本的实际横向偏移，0.1mm/LSB
  uint16_t strength;                  // 最近一次磁样本的综合强度
  uint16_t age_ms;                    // 最近一次磁样本的年龄，单位ms
  uint16_t last_sequence;             // 已用于更新偏移变化率的最后样本序号
  uint32_t last_receive_time_ms;      // last_sequence对应的本地接收时间，单位ms
  float last_offset_error_mm;         // 上一个新样本相对目标位置的偏差，单位mm
  float offset_rate_mm_s;             // 滤波后的横向偏差变化率，单位mm/s
  float yaw_rate_target_dps;          // 磁循迹外环给出的目标Yaw角速度，单位°/s
  float actual_yaw_rate_dps;          // IMU测得的当前Yaw角速度，单位°/s
  float speed_limit_rpm;              // 当前磁路状态允许的基础轮速上限，单位RPM
  uint16_t curve_evidence_ticks;      // 连续检测到弯道迹象的控制周期数
  uint16_t straight_evidence_ticks;   // 连续检测到居中或恢复条件的控制周期数
  uint8_t allow_multi_peak;           // 1表示当前任务允许弯道产生的多峰磁场
  uint8_t sequence_valid;             // 1表示偏移变化率的历史基准已经建立
  uint8_t adaptive_high_speed;        // 1表示启用直线高速、弯道降速调度
  uint8_t speed_phase;                // 当前magnetic_follow_speed_phase_t阶段值
} magnetic_follow_controller_t;

/* 清空任务状态和历史；NULL输入不执行操作。 */
void magnetic_follow_controller_reset(
    magnetic_follow_controller_t *controller);

/* 检查时效、flags、强度、偏移范围和真正分离的多峰；返回1U表示可用。 */
uint8_t magnetic_follow_sample_is_trackable(
    const magnetic_follow_sample_t *sample);

/* 曲线模式可允许多峰，但仍使用磁导航板对全部通道计算的整体偏移。 */
uint8_t magnetic_follow_sample_is_trackable_for_mode(
    const magnetic_follow_sample_t *sample,
    uint8_t allow_multi_peak);

/*
 * 用已经通过质量检查的任务首帧建立变化率基准和初始限速。
 * target_offset_x10mm单位为0.1 mm；requested_run_rpm必须为正值。
 */
void magnetic_follow_controller_start(
    magnetic_follow_controller_t *controller,
    int16_t target_offset_x10mm,
    uint8_t allow_multi_peak,
    const magnetic_follow_sample_t *sample,
    float requested_run_rpm);

/*
 * 对已经通过质量检查的当前样本快照更新偏移外环、Yaw目标和动态限速。
 * 只有偏移变化率使用sequence判断新旧，其余阶段逻辑按控制周期推进。
 * actual_yaw_rate_dps使用车辆Z轴角速度；本函数不直接产生PWM或停车动作。
 */
void magnetic_follow_controller_update(
    magnetic_follow_controller_t *controller,
    const magnetic_follow_sample_t *sample,
    float requested_run_rpm,
    float actual_yaw_rate_dps);

/* Convert the magnetic body target to the shared reference contract. */
uint8_t magnetic_follow_controller_make_reference(
    const magnetic_follow_controller_t *controller,
    float base_rpm,
    float wheel_circumference_mm,
    int8_t travel_direction,
    uint32_t now_ms,
    uint16_t accel_mm_s2,
    uint16_t decel_mm_s2,
    agv_motion_reference_t *reference);

#ifdef __cplusplus
}
#endif

#endif
