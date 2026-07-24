#ifndef AT32F456_REPLACEMENT_TEST_H
#define AT32F456_REPLACEMENT_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 使用12MHz外晶振配置192MHz系统时钟，失败时保留内部时钟继续自检。 */
void at32f456_replacement_clock_config(void);

/* 执行AT32F456RCT7新板安全自检，并启动外部CAN FD测试帧。 */
void at32f456_replacement_test_init(void);

/* 周期输出运行状态，并检查串口接收、ADC和数字输入。 */
void at32f456_replacement_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
