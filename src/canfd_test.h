#ifndef CANFD_TEST_H
#define CANFD_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化FDCAN3、PB3/PB4和CAN FD测试状态机。 */
void canfd_test_init(void);

/* 周期执行内部回环和外部总线收发测试。 */
void canfd_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
