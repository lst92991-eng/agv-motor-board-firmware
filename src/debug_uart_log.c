/**
  *****************************************************************************
  * @file     debug_uart_log.c
  * @brief    不阻塞控制任务的格式化日志入队
  *****************************************************************************
  */

#include "debug_uart_log.h"

#include <stdarg.h>
#include <stdio.h>

#include "debug_uart_transport.h"

#define DEBUG_UART_LOG_MESSAGE_SIZE 256U

void debug_uart_log_init(void)
{
}

int debug_uart_log_printf(const char *format, ...)
{
  va_list arguments;
  int result;
  char message[DEBUG_UART_LOG_MESSAGE_SIZE];

  va_start(arguments, format);
  result = vsnprintf(message, sizeof(message), format, arguments);
  va_end(arguments);
  if((result <= 0) || ((uint32_t)result >= sizeof(message)))
  {
    return -1;
  }
  return (debug_uart_transport_write((const uint8_t *)message,
                                     (uint16_t)result) != 0U) ? result : 0;
}
