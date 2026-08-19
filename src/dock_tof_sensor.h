#ifndef DOCK_TOF_SENSOR_H
#define DOCK_TOF_SENSOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  int32_t left_reference_mm;
  int32_t right_reference_mm;
  int32_t zero_deadband_mm;
} dock_tof_calibration_t;

typedef struct
{
  uint32_t sample_sequence;
  uint16_t left_raw_mm;
  uint16_t left_filtered_mm;
  uint16_t right_raw_mm;
  uint16_t right_filtered_mm;
  int32_t distance_error_mm;
  int32_t alignment_error_mm;
  int8_t left_api_status;
  int8_t right_api_status;
  uint8_t left_range_status;
  uint8_t right_range_status;
  uint8_t left_valid;
  uint8_t right_valid;
  uint8_t pair_valid;
  uint8_t left_gpio_level;
  uint8_t right_gpio_level;
} dock_tof_snapshot_t;

/*
 * 2026-07-20机械安装下的实测接触基准。充电桩120度靶标或安装位置变化后，
 * 必须重新标定，不能假设左右距离相等。
 */
extern const dock_tof_calibration_t dock_tof_default_calibration;

/*
 * 初始化两颗VL53L0X并依次改址为0x54和0x56。
 * 该过程包含器件参考校准，会阻塞，禁止从10ms底盘控制任务或中断调用。
 */
uint8_t dock_tof_sensor_init(const dock_tof_calibration_t *calibration);

/* 使用当前标定重新初始化总线和两颗传感器。 */
uint8_t dock_tof_sensor_reinitialize(void);

/* 返回两颗传感器是否均完成初始化。 */
uint8_t dock_tof_sensor_is_ready(void);

/*
 * 完成一次左右测距并更新滤波快照。调用者必须是唯一的低优先级传感器任务。
 * 返回1仅表示本次左右测距都有效，业务层仍需检查误差、时效和运动状态。
 */
uint8_t dock_tof_sensor_sample(dock_tof_snapshot_t *snapshot);

/* 读取软件I2C空闲电平和最近一次底层错误码。 */
void dock_tof_sensor_get_bus_diagnostics(uint8_t *scl_level,
                                         uint8_t *sda_level,
                                         int32_t *last_error);

#ifdef __cplusplus
}
#endif

#endif
