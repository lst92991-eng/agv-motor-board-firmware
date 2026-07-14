#ifndef MOTOR_ENCODER_TEST_H
#define MOTOR_ENCODER_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化编码器单项测试，内部会初始化左右编码器并将累计计数清零。 */
void motor_encoder_test_init(void);

/* 周期输出左、右编码器累计计数，串口格式为：left_count,right_count。 */
void motor_encoder_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
