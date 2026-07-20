#ifndef DUAL_MOTOR_ENCODER_TEST_H
#define DUAL_MOTOR_ENCODER_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化左右电机开环转动与编码器对照测试。 */
void dual_motor_encoder_test_init(void);

/* 每100ms同时输出左右电机和编码器诊断数据。 */
void dual_motor_encoder_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
