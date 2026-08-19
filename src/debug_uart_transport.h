#ifndef DEBUG_UART_TRANSPORT_H
#define DEBUG_UART_TRANSPORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化USART1接收中断和静态环形缓冲区。 */
void debug_uart_transport_init(void);

/* 非阻塞读取一个已接收字节；有数据返回1，无数据返回0。 */
uint8_t debug_uart_transport_read_byte(uint8_t *data);

/* 读取并清除环形缓冲区溢出标志。 */
uint8_t debug_uart_transport_take_overflow(void);

/* 由USART1中断服务程序调用，应用层不得直接调用。 */
void debug_uart_transport_irq_handler(void);

#ifdef __cplusplus
}
#endif

#endif
