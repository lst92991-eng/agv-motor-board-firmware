#ifndef MOTOR_SPEED_TEST_H
#define MOTOR_SPEED_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化电机转动与编码器速度联合测试，上电后先保持电机停止 3 秒。 */
void motor_speed_test_init(void);

/* 驱动电机、读取编码器，并以 RPM 为单位输出左右电机速度。 */
void motor_speed_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
