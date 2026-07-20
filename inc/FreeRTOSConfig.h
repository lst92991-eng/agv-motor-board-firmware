#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>
#include "system_at32f45x.h"

/* 内核使用1ms抢占式节拍，应用任务优先级范围为0~6。 */
#define configUSE_PREEMPTION                         1
#define configUSE_TIME_SLICING                       1
#define configUSE_IDLE_HOOK                          0
#define configUSE_TICK_HOOK                          0
#define configCPU_CLOCK_HZ                           ((unsigned long)system_core_clock)
#define configTICK_RATE_HZ                           ((TickType_t)1000U)
#define configMAX_PRIORITIES                         7
#define configMINIMAL_STACK_SIZE                     ((uint16_t)128U)
#define configMAX_TASK_NAME_LEN                      16
#define configUSE_16_BIT_TICKS                       0
#define configIDLE_SHOULD_YIELD                      1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION      1

/* 所有应用对象静态分配，运行期间不使用FreeRTOS堆。 */
#define configSUPPORT_STATIC_ALLOCATION              1
#define configSUPPORT_DYNAMIC_ALLOCATION             0

/* 启用命令队列和串口输出互斥锁所需内核能力。 */
#define configUSE_MUTEXES                            1
#define configUSE_RECURSIVE_MUTEXES                  0
#define configUSE_COUNTING_SEMAPHORES                0
#define configQUEUE_REGISTRY_SIZE                    4
#define configUSE_QUEUE_SETS                         0

/* 当前不使用软件定时器、协程和低功耗无节拍模式。 */
#define configUSE_TIMERS                             0
#define configUSE_CO_ROUTINES                        0
#define configMAX_CO_ROUTINE_PRIORITIES              1
#define configUSE_TICKLESS_IDLE                      0

/* 开启栈溢出检查和调度状态查询，便于运行期保护。 */
#define configCHECK_FOR_STACK_OVERFLOW               2
#define configUSE_MALLOC_FAILED_HOOK                 0
#define INCLUDE_vTaskPrioritySet                     0
#define INCLUDE_uxTaskPriorityGet                   0
#define INCLUDE_vTaskDelete                          0
#define INCLUDE_vTaskSuspend                         0
#define INCLUDE_vTaskDelayUntil                      1
#define INCLUDE_vTaskDelay                           1
#define INCLUDE_xTaskGetSchedulerState               1
#define INCLUDE_xTaskGetCurrentTaskHandle            1

#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS                              __NVIC_PRIO_BITS
#else
#define configPRIO_BITS                              4
#endif

/*
 * 数值0优先级最高，15最低。优先级0~4的中断不得调用FreeRTOS API；
 * 优先级5~15的中断才允许调用FromISR接口。
 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY              \
  (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY         \
  (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

void app_freertos_assert_failed(const char *file, int line);
#define configASSERT(expression)                     \
  do                                                 \
  {                                                  \
    if((expression) == 0)                            \
    {                                                \
      app_freertos_assert_failed(__FILE__, __LINE__);\
    }                                                \
  } while(0)

/* 将Cortex-M异常入口映射到FreeRTOS官方移植层。 */
#define vPortSVCHandler                              SVC_Handler
#define xPortPendSVHandler                           PendSV_Handler
#define xPortSysTickHandler                          SysTick_Handler

#endif
