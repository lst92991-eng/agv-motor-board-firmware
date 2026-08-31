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
uint8_t motor_current_init(void);

/*
 * 触发一次左右电流采样并更新内部数据。
 * 返回 1 表示两路采样成功，返回 0 表示 ADC 等待超时。
 */
uint8_t motor_current_update(void);

/* 读取最近一次采样的 ADC 原始值。 */
uint16_t motor_current_get_raw(motor_id_t motor);

#ifdef __cplusplus
}
#endif

#endif
