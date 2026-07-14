#ifndef MOTOR_DRIVER_TEST_H
#define MOTOR_DRIVER_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化电机驱动单项测试，上电后先保持两路电机停止。 */
void motor_driver_test_init(void);

/* 按固定顺序测试左右电机的正转和反转。 */
void motor_driver_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
