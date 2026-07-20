#ifndef TOF_IR_TEST_H
#define TOF_IR_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化左右TOF测试。 */
void tof_ir_test_init(void);

/* 依次测量左右距离并输出全部测试数据。 */
void tof_ir_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
