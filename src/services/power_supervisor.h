/**
 * @file power_supervisor.h
 * @brief 电气监督配置、诊断快照和锁存故障接口。
 */

#ifndef POWER_SUPERVISOR_H
#define POWER_SUPERVISOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POWER_SUPERVISOR_ZERO_SAMPLE_COUNT       64U

#define POWER_FAULT_CONFIG_INVALID             0x01U
#define POWER_FAULT_NOT_APPROVED               0x02U
#define POWER_FAULT_ADC                        0x04U
#define POWER_FAULT_UNDERVOLTAGE               0x08U
#define POWER_FAULT_OVERVOLTAGE                0x10U
#define POWER_FAULT_PEAK_CURRENT               0x20U
#define POWER_FAULT_SEVERE_MASK                0x3DU

typedef struct
{
  /* 换算系数单位写入字段名，避免mA/count与uA/count混用。 */
  uint32_t left_current_ua_per_count;
  uint32_t right_current_ua_per_count;
  uint16_t left_zero_raw;
  uint16_t right_zero_raw;
  uint16_t zero_max_raw;
  uint32_t bus_uv_per_count;
  int32_t bus_offset_mv;
  uint32_t undervoltage_trip_mv;
  uint32_t undervoltage_recover_mv;
  uint32_t overvoltage_recover_mv;
  uint32_t overvoltage_trip_mv;
  uint32_t peak_current_ma;
  uint16_t peak_duration_ms;
  uint16_t sample_period_ms;
  uint8_t adc_failure_limit;
  uint8_t limits_approved;
} power_supervisor_config_t;

typedef struct
{
  uint32_t bus_mv;
  uint32_t left_current_ma;
  uint32_t right_current_ma;
  uint16_t left_zero_raw;
  uint16_t right_zero_raw;
  uint8_t zero_samples;
  uint8_t fault_mask;
  uint8_t ready;
  uint8_t can_clear;
} power_supervisor_snapshot_t;

/* 纯状态机：不访问ADC、PWM或RTOS，所有单位换算与阈值来自config。 */
const power_supervisor_config_t *power_supervisor_provisional_config(void);
/** 初始化配置并失败关闭；ready仍需完成64个静止零点样本。 */
void power_supervisor_init(const power_supervisor_config_t *config);
/** 每20ms输入同一采样周期的母线、左电流和右电流原始值。 */
void power_supervisor_step(uint8_t adc_valid, uint16_t bus_raw,
                           uint16_t left_raw, uint16_t right_raw,
                           uint8_t zero_calibration_allowed);
/** 判断是否完成静止零点且当前无故障。 */
uint8_t power_supervisor_is_ready(void);
/** 返回锁存电气故障位图。 */
uint8_t power_supervisor_get_fault_mask(void);
/** 只提交清除请求；严重故障未进入迟滞恢复窗口时返回0。 */
uint8_t power_supervisor_clear_fault(void);
/** 复制最近诊断快照；调用方不得修改内部保护状态。 */
void power_supervisor_get_snapshot(power_supervisor_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
