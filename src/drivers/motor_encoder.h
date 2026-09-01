/**
 * @file motor_encoder.h
 * @brief 双轮正交编码器累计计数与ISR入口。
 */

#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <stdint.h>
#include "motor_driver.h"

/** 初始化GPIO、EXINT、初始AB状态和实车确认的左右方向。 */
void motor_encoder_init(void);
/** 返回累计有符号计数；Cortex-M对齐32位读取为原子快照。 */
int32_t motor_encoder_get_count(motor_id_t motor);
/** 返回非法Gray码跳变累计值，用于发现线束毛刺或漏边沿。 */
uint32_t motor_encoder_get_invalid_transition_count(motor_id_t motor);
/** 仅在停车/复位路径调用，内部会短暂关中断。 */
void motor_encoder_reset_all(void);
/** 由EXINT ISR调用；不得从任务上下文模拟边沿。 */
void motor_encoder_left_edge_handler(void);
/** 由EXINT ISR调用；与左轮入口共享同一Gray码解码器。 */
void motor_encoder_right_edge_handler(void);

#endif
