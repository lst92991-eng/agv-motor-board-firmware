/**
  *****************************************************************************
  * @file     debug_uart_transport.c
  * @brief    USART1中断接收和静态环形缓冲区
  *****************************************************************************
  */

#include "debug_uart_transport.h"

#include "at32f45x_board.h"
#include "at32f45x_misc.h"
#include "at32f45x_usart.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_UART_RX_BUFFER_SIZE             512U
#define DEBUG_UART_RX_BUFFER_MASK             (DEBUG_UART_RX_BUFFER_SIZE - 1U)

static volatile uint8_t debug_uart_rx_buffer[DEBUG_UART_RX_BUFFER_SIZE];
static volatile uint16_t debug_uart_rx_head;
static volatile uint16_t debug_uart_rx_tail;
static volatile uint8_t debug_uart_rx_overflow;

void debug_uart_transport_init(void)
{
  debug_uart_rx_head = 0U;
  debug_uart_rx_tail = 0U;
  debug_uart_rx_overflow = 0U;

  while(usart_flag_get(PRINT_UART, USART_RDBF_FLAG) != RESET)
  {
    (void)usart_data_receive(PRINT_UART);
  }

  usart_interrupt_enable(PRINT_UART, USART_RDBF_INT, TRUE);
  /* USART只搬运字节，设为可被FreeRTOS临界区屏蔽的优先级6。 */
  nvic_irq_enable(USART1_IRQn, 6U, 0U);
}

void debug_uart_transport_irq_handler(void)
{
  while(usart_flag_get(PRINT_UART, USART_RDBF_FLAG) != RESET)
  {
    uint8_t data = (uint8_t)usart_data_receive(PRINT_UART);
    uint16_t next_head = (uint16_t)((debug_uart_rx_head + 1U) &
                                    DEBUG_UART_RX_BUFFER_MASK);

    if(next_head == debug_uart_rx_tail)
    {
      debug_uart_rx_overflow = 1U;
    }
    else
    {
      debug_uart_rx_buffer[debug_uart_rx_head] = data;
      debug_uart_rx_head = next_head;
    }
  }
}

uint8_t debug_uart_transport_read_byte(uint8_t *data)
{
  if((data == 0) || (debug_uart_rx_tail == debug_uart_rx_head))
  {
    return 0U;
  }

  *data = debug_uart_rx_buffer[debug_uart_rx_tail];
  debug_uart_rx_tail = (uint16_t)((debug_uart_rx_tail + 1U) &
                                  DEBUG_UART_RX_BUFFER_MASK);
  return 1U;
}

uint8_t debug_uart_transport_take_overflow(void)
{
  uint8_t overflow;

  taskENTER_CRITICAL();
  overflow = debug_uart_rx_overflow;
  debug_uart_rx_overflow = 0U;
  taskEXIT_CRITICAL();
  return overflow;
}
