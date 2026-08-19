#ifndef DEBUG_UART_LOG_H
#define DEBUG_UART_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/* 创建静态串口输出互斥锁，调度器启动前调用。 */
void debug_uart_log_init(void);

/* 以互斥方式输出一条格式化日志，避免不同任务的串口内容交叉。 */
int debug_uart_log_printf(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif
