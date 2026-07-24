#ifndef RIGHT_MOTOR_ENCODER_ADC_TEST_H
#define RIGHT_MOTOR_ENCODER_ADC_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化右电机、右编码器和三路ADC采集测试。 */
void right_motor_encoder_adc_test_init(void);

/* 周期输出右电机编码器和ADC采样数据。 */
void right_motor_encoder_adc_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
