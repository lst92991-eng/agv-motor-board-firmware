#ifndef CHASSIS_STRAIGHT_TEST_H
#define CHASSIS_STRAIGHT_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化任意万向轮角度下的只前进直线行驶测试。 */
void chassis_straight_test_init(void);

/* 以10ms基础周期调度航向采样、50Hz速度环、状态机和串口输出。 */
void chassis_straight_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
