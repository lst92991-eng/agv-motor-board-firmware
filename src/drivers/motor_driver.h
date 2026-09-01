/**
 * @file motor_driver.h
 * @brief 双AT8236 PWM、方向与停车模式接口。
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

typedef enum { MOTOR_LEFT = 0, MOTOR_RIGHT } motor_id_t;
typedef enum { MOTOR_STOP_COAST = 0, MOTOR_STOP_BRAKE } motor_stop_mode_t;

/** 初始化PC6～PC9和TMR8；完成前输出保持Coast。 */
void motor_driver_init(void);
/** 设置单轮有符号PWM千分比，自动限制到-1000～1000。 */
void motor_driver_set_speed(motor_id_t motor, int16_t speed_permille);
/** 设置双轮有符号PWM；正常运行的唯一调用者应为速度环。 */
void motor_driver_set_speeds(int16_t left_speed_permille,
                             int16_t right_speed_permille);
/** 设置单轮Coast或主动Brake；安全停车应使用Brake。 */
void motor_driver_stop(motor_id_t motor, motor_stop_mode_t mode);
/** 以同一Coast或Brake模式处理两路H桥。 */
void motor_driver_stop_all(motor_stop_mode_t mode);

#endif
