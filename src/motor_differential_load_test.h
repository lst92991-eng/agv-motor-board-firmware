#ifndef MOTOR_DIFFERENTIAL_LOAD_TEST_H
#define MOTOR_DIFFERENTIAL_LOAD_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化带载差速速度闭环测试。 */
void motor_differential_load_test_init(void);

/* 执行直行、弧线和原地旋转动作，并输出PID调试数据。 */
void motor_differential_load_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
