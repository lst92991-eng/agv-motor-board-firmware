#ifndef CANFD_BUMPER_TEST_H
#define CANFD_BUMPER_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化无电机的CAN FD协议与四路防撞安全联调。 */
void canfd_bumper_test_init(void);

/* 1ms调用一次，处理CAN收发、防撞锁存和周期状态上报。 */
void canfd_bumper_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
