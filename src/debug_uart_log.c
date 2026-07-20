/**
  *****************************************************************************
  * @file     debug_uart_log.c
  * @brief    FreeRTOS任务间共享的串口输出互斥封装
  *****************************************************************************
  */

#include "debug_uart_log.h"

#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

static StaticSemaphore_t debug_uart_log_mutex_storage;
static SemaphoreHandle_t debug_uart_log_mutex;

void debug_uart_log_init(void)
{
  debug_uart_log_mutex = xSemaphoreCreateMutexStatic(
      &debug_uart_log_mutex_storage);
  configASSERT(debug_uart_log_mutex != NULL);
}

int debug_uart_log_printf(const char *format, ...)
{
  va_list arguments;
  int result;
  uint8_t mutex_locked = 0U;

  if((debug_uart_log_mutex != NULL) &&
     (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
  {
    if(xSemaphoreTake(debug_uart_log_mutex, portMAX_DELAY) == pdPASS)
    {
      mutex_locked = 1U;
    }
  }

  va_start(arguments, format);
  result = vprintf(format, arguments);
  va_end(arguments);

  if(mutex_locked != 0U)
  {
    (void)xSemaphoreGive(debug_uart_log_mutex);
  }
  return result;
}
