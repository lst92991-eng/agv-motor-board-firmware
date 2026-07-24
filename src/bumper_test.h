#ifndef BUMPER_TEST_H
#define BUMPER_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化四路防撞输入并输出接口定义和上电原始电平。 */
void bumper_test_init(void);

/* 处理边沿事件，并在电平稳定后输出确认状态。 */
void bumper_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
