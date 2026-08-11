#ifndef MOTOR_SPEED_PID_TEST_H
#define MOTOR_SPEED_PID_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 兼容旧测试入口；实现已统一到正式速度环的整定测试模块。 */
void motor_speed_pid_test_init(void);
void motor_speed_pid_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
