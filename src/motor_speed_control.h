#ifndef MOTOR_SPEED_CONTROL_H
#define MOTOR_SPEED_CONTROL_H

#include <stdint.h>

#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化双路电机驱动、编码器和速度 PI 控制器。 */
void motor_speed_control_init(void);

/* 设置左右输出轴目标转速，单位为 RPM。 */
void motor_speed_control_set_targets(float left_target_rpm, float right_target_rpm);

/* 等待20ms控制周期，更新编码器速度、PI 和电机 PWM。 */
void motor_speed_control_update(void);

/* 立即执行一次速度环计算，不在函数内部等待控制周期。 */
void motor_speed_control_update_now(void);

/* 在车辆停止后重新同步编码器基准，避免下一次启动把停车阶段计数算进速度。 */
void motor_speed_control_resync_feedback(void);

/* 读取指定电机的编码器反馈速度，单位为 RPM。 */
float motor_speed_control_get_measured_rpm(motor_id_t motor);

/* 读取指定电机当前的有符号 PWM，范围为 -1000~1000。 */
int16_t motor_speed_control_get_pwm(motor_id_t motor);

/* 两路目标速度清零，并让 AT8236 保持主动刹车。 */
void motor_speed_control_hold_brake(void);

#ifdef __cplusplus
}
#endif

#endif
