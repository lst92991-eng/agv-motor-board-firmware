#ifndef MOTOR_CURRENT_H
#define MOTOR_CURRENT_H

#include <stdint.h>

#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 初始化电机电流采集 ADC。
 * 左电流接 PA4/ADC1_IN4，右电流接 PA5/ADC1_IN5。
 */
void motor_current_init(void);

/*
 * 触发一次左右电流采样并更新内部数据。
 * 返回 1 表示两路采样成功，返回 0 表示 ADC 等待超时。
 */
uint8_t motor_current_update(void);

/*
 * 电机必须停止时调用此函数，采集多次 ADC 零电流值作为软件零偏。
 * sample_count 为平均次数，传入 0 时不执行校准。
 */
uint8_t motor_current_calibrate_zero(uint16_t sample_count);

/* 读取最近一次采样的 ADC 原始值。 */
uint16_t motor_current_get_raw(motor_id_t motor);

/* 读取扣除零偏后的电流，单位为 mA。当前硬件只能测量电流绝对值。 */
uint32_t motor_current_get_ma(motor_id_t motor);

#ifdef __cplusplus
}
#endif

#endif
