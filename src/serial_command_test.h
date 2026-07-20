#ifndef SERIAL_COMMAND_TEST_H
#define SERIAL_COMMAND_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化只接收和解析命令、不初始化电机的安全测试。 */
void serial_command_test_init(void);

/* 非阻塞解析串口命令并输出测试结果。 */
void serial_command_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
