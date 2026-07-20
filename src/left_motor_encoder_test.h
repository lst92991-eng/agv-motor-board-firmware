#ifndef LEFT_MOTOR_ENCODER_TEST_H
#define LEFT_MOTOR_ENCODER_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化左电机开环转动与编码器联合测试。 */
void left_motor_encoder_test_init(void);

/* 每100ms采样并输出一次左编码器诊断数据。 */
void left_motor_encoder_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
