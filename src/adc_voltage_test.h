#ifndef ADC_VOLTAGE_TEST_H
#define ADC_VOLTAGE_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化前进、后退过程中的电池电压和双路电机电流测试。 */
void adc_voltage_test_init(void);

/* 调度前进、后退动作，并每100ms输出电池电压和左右电流。 */
void adc_voltage_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
