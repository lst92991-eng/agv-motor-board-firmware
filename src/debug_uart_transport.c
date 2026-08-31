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
#define DEBUG_UART_TX_BUFFER_SIZE            2048U
#define DEBUG_UART_TX_BUFFER_MASK             (DEBUG_UART_TX_BUFFER_SIZE - 1U)

static volatile uint8_t debug_uart_rx_buffer[DEBUG_UART_RX_BUFFER_SIZE];
static volatile uint16_t debug_uart_rx_head;
static volatile uint16_t debug_uart_rx_tail;
static volatile uint8_t debug_uart_rx_overflow;
static uint8_t debug_uart_tx_buffer[DEBUG_UART_TX_BUFFER_SIZE];
static volatile uint16_t debug_uart_tx_head;
static volatile uint16_t debug_uart_tx_tail;
static volatile uint32_t debug_uart_tx_drop_count;

void debug_uart_transport_init(void)
{
  debug_uart_rx_head = 0U;
  debug_uart_rx_tail = 0U;
  debug_uart_rx_overflow = 0U;
  debug_uart_tx_head = 0U;
  debug_uart_tx_tail = 0U;
  debug_uart_tx_drop_count = 0UL;

  while(usart_flag_get(PRINT_UART, USART_RDBF_FLAG) != RESET)
  {
    (void)usart_data_receive(PRINT_UART);
  }

  usart_interrupt_enable(PRINT_UART, USART_RDBF_INT, TRUE);
  usart_interrupt_enable(PRINT_UART, USART_TDBE_INT, FALSE);
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

  if(usart_flag_get(PRINT_UART, USART_TDBE_FLAG) != RESET)
  {
    if(debug_uart_tx_tail != debug_uart_tx_head)
    {
      usart_data_transmit(PRINT_UART,
          debug_uart_tx_buffer[debug_uart_tx_tail]);
      debug_uart_tx_tail = (uint16_t)((debug_uart_tx_tail + 1U) &
                                      DEBUG_UART_TX_BUFFER_MASK);
    }
    if(debug_uart_tx_tail == debug_uart_tx_head)
    {
      usart_interrupt_enable(PRINT_UART, USART_TDBE_INT, FALSE);
    }
  }
}

uint8_t debug_uart_transport_write(const uint8_t *data, uint16_t length)
{
  uint16_t used;
  uint16_t free_bytes;
  uint16_t index;

  if((data == 0) || (length == 0U) ||
     (length >= DEBUG_UART_TX_BUFFER_SIZE))
  {
    return 0U;
  }

  taskENTER_CRITICAL();
  used = (uint16_t)((debug_uart_tx_head - debug_uart_tx_tail) &
                    DEBUG_UART_TX_BUFFER_MASK);
  free_bytes = (uint16_t)(DEBUG_UART_TX_BUFFER_MASK - used);
  if(length > free_bytes)
  {
    debug_uart_tx_drop_count++;
    taskEXIT_CRITICAL();
    return 0U;
  }
  for(index = 0U; index < length; index++)
  {
    debug_uart_tx_buffer[debug_uart_tx_head] = data[index];
    debug_uart_tx_head = (uint16_t)((debug_uart_tx_head + 1U) &
                                    DEBUG_UART_TX_BUFFER_MASK);
  }
  usart_interrupt_enable(PRINT_UART, USART_TDBE_INT, TRUE);
  taskEXIT_CRITICAL();
  return 1U;
}

uint32_t debug_uart_transport_get_tx_drop_count(void)
{
  return debug_uart_tx_drop_count;
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
