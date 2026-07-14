#ifndef BMI270_TEST_H
#define BMI270_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 初始化 BMI270 测试模块。
 * 该函数内部会探测 SPI 通信方式、初始化 BMI270，并配置加速度计和陀螺仪。
 */
void bmi270_test_init(void);

/*
 * BMI270 测试模块循环任务。
 * 初始化成功时输出三路欧拉角；初始化失败时定时重试。
 */
void bmi270_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
