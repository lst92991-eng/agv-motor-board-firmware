#ifndef CHASSIS_MOTION_TEST_H
#define CHASSIS_MOTION_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化底盘前进、后退和原地转向测试。 */
void chassis_motion_test_init(void);

/* 根据编码器圈数执行动作序列，并输出当前阶段和运动数据。 */
void chassis_motion_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
