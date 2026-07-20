#ifndef LEFT_MOTOR_SCOPE_FORWARD_TEST_H
#define LEFT_MOTOR_SCOPE_FORWARD_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化左电机持续开环正转示波器测试。 */
void left_motor_scope_forward_test_init(void);

/* 保持主循环运行，不计算或输出任何测试数据。 */
void left_motor_scope_forward_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
