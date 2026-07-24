#ifndef RIGHT_MOTOR_SCOPE_FORWARD_TEST_H
#define RIGHT_MOTOR_SCOPE_FORWARD_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化右电机持续开环正转测试。 */
void right_motor_scope_forward_test_init(void);

/* 保持主循环运行，不计算或输出任何测试数据。 */
void right_motor_scope_forward_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
