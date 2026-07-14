#ifndef MOTOR_SPEED_PID_TEST_H
#define MOTOR_SPEED_PID_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化电机速度闭环测试，上电后先保持电机停止 3 秒。 */
void motor_speed_pid_test_init(void);

/* 周期执行速度 PID、更新 PWM，并输出目标速度、实测速度和 PWM。 */
void motor_speed_pid_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
