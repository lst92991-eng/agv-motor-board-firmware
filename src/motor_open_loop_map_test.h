#ifndef MOTOR_OPEN_LOOP_MAP_TEST_H
#define MOTOR_OPEN_LOOP_MAP_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化电机 PWM 与空载转速映射测试。 */
void motor_open_loop_map_test_init(void);

/* 按固定 PWM 阶梯驱动电机，并输出 PWM 与左右编码器速度。 */
void motor_open_loop_map_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
