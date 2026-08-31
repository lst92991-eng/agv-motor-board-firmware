#ifndef DEBUG_UART_TRANSPORT_H
#define DEBUG_UART_TRANSPORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化USART1收发中断和静态环形缓冲区。 */
void debug_uart_transport_init(void);

/* 全量入队一条日志；空间不足时立即失败，绝不等待串口。 */
uint8_t debug_uart_transport_write(const uint8_t *data, uint16_t length);

/* 返回因TX队列空间不足而丢弃的完整日志条数。 */
uint32_t debug_uart_transport_get_tx_drop_count(void);

/* 非阻塞读取一个已接收字节；有数据返回1，无数据返回0。 */
uint8_t debug_uart_transport_read_byte(uint8_t *data);

/* 读取并清除环形缓冲区溢出标志。 */
uint8_t debug_uart_transport_take_overflow(void);

/* 由USART1中断服务程序调用，搬运RX和TX字节。 */
void debug_uart_transport_irq_handler(void);

#ifdef __cplusplus
}
#endif

#endif
