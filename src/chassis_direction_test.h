#ifndef CHASSIS_DIRECTION_TEST_H
#define CHASSIS_DIRECTION_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化小车前进、后退、左转和右转方向测试。 */
void chassis_direction_test_init(void);

/* 按固定PWM依次执行四个基础动作。 */
void chassis_direction_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
