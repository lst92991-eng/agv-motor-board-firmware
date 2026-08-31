#ifndef DEBUG_UART_LOG_H
#define DEBUG_UART_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/* 保留统一初始化入口；实现不创建锁或任务。 */
void debug_uart_log_init(void);

/* 格式化后整条入队；队列满或消息过长时立即返回，不等待硬件。 */
int debug_uart_log_printf(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif
