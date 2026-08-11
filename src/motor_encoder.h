#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <stdint.h>

#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 初始化左右电机的 AB 相编码器输入。
 * 左编码器使用 PA6/PA7，右编码器使用 PC4/PC5，四路信号均采用双边沿中断计数。
 */
void motor_encoder_init(void);

/* 读取指定电机从上次清零开始累计的正交计数值。 */
int32_t motor_encoder_get_count(motor_id_t motor);

/* 读取AB相同时跳变等非法状态转换的累计次数。 */
uint32_t motor_encoder_get_invalid_transition_count(motor_id_t motor);

/* 清零指定电机的累计计数值。 */
void motor_encoder_reset(motor_id_t motor);

/* 同时清零左右电机累计计数值。 */
void motor_encoder_reset_all(void);

/* 以下函数由外部中断服务程序调用，应用层不需要直接调用。 */
void motor_encoder_left_edge_handler(void);
void motor_encoder_right_edge_handler(void);

#ifdef __cplusplus
}
#endif

#endif
