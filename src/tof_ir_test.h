#ifndef TOF_IR_TEST_H
#define TOF_IR_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化左右TOF和各自的距离滤波器。 */
void tof_ir_test_init(void);

/* 依次测量左右距离，输出原始值、滤波值、有效性和距离差。 */
void tof_ir_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
