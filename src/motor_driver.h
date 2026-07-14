#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 电机编号，与开发板网络表中的左、右电机接口对应。 */
typedef enum
{
  MOTOR_LEFT = 0,
  MOTOR_RIGHT
} motor_id_t;

/* 电机停止方式：滑行时桥臂全部关闭，刹车时桥臂进入短刹状态。 */
typedef enum
{
  MOTOR_STOP_COAST = 0,
  MOTOR_STOP_BRAKE
} motor_stop_mode_t;

/*
 * 初始化双路 AT8236 电机驱动。
 * PC6、PC7 控制左电机，PC8、PC9 控制右电机，PWM 频率为 20kHz。
 * 初始化完成后两路电机都处于滑行停止状态。
 */
void motor_driver_init(void);

/*
 * 设置单个电机速度。
 * speed_permille 的范围为 -1000~1000，符号表示方向，绝对值表示 PWM 千分比。
 */
void motor_driver_set_speed(motor_id_t motor, int16_t speed_permille);

/* 同时设置左右电机速度，参数范围均为 -1000~1000。 */
void motor_driver_set_speeds(int16_t left_speed_permille, int16_t right_speed_permille);

/* 按指定方式停止单个电机。 */
void motor_driver_stop(motor_id_t motor, motor_stop_mode_t stop_mode);

/* 按指定方式同时停止左右电机。 */
void motor_driver_stop_all(motor_stop_mode_t stop_mode);

#ifdef __cplusplus
}
#endif

#endif
