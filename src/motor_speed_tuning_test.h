#ifndef MOTOR_SPEED_TUNING_TEST_H
#define MOTOR_SPEED_TUNING_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 架空两侧主动轮后使用；此测试运行与正式底盘相同的 20ms 速度环。 */
void motor_speed_tuning_test_init(void);
void motor_speed_tuning_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
