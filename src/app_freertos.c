/**
  *****************************************************************************
  * @file     app_freertos.c
  * @brief    AGV底盘FreeRTOS任务、队列和异常钩子
  *****************************************************************************
  */

#include "app_freertos.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "chassis_motion_controller.h"
#include "canfd_protocol.h"
#include "debug_uart_log.h"
#include "motion_command.h"
#include "motor_speed_control.h"
#include "serial_motion_protocol.h"

#define APP_CONTROL_TASK_PRIORITY                5U
#define APP_SERIAL_TASK_PRIORITY                 3U
#define APP_CANFD_TASK_PRIORITY                  3U
#define APP_CONTROL_TASK_STACK_WORDS          1024U
#define APP_SERIAL_TASK_STACK_WORDS            512U
#define APP_CANFD_TASK_STACK_WORDS             768U
#define APP_COMMAND_QUEUE_LENGTH                  8U
#define APP_CONTROL_PERIOD_MS                    10U
#define APP_SERIAL_POLL_PERIOD_MS                 2U
#define APP_CANFD_POLL_PERIOD_MS                  2U
#define APP_PRIORITY_COMMAND_COUNT                4U

#define APP_PRIORITY_ESTOP_INDEX                  0U
#define APP_PRIORITY_COMM_TIMEOUT_INDEX           1U
#define APP_PRIORITY_CAN_BUSOFF_INDEX              2U
#define APP_PRIORITY_STOP_INDEX                   3U

typedef struct
{
  motion_command_t command;
  uint32_t safety_epoch;
} app_queued_motion_command_t;

static StaticTask_t app_control_task_storage;
static StackType_t app_control_task_stack[APP_CONTROL_TASK_STACK_WORDS];
static TaskHandle_t app_control_task_handle;

static StaticTask_t app_serial_task_storage;
static StackType_t app_serial_task_stack[APP_SERIAL_TASK_STACK_WORDS];
static TaskHandle_t app_serial_task_handle;

static StaticTask_t app_canfd_task_storage;
static StackType_t app_canfd_task_stack[APP_CANFD_TASK_STACK_WORDS];
static TaskHandle_t app_canfd_task_handle;

static StaticQueue_t app_command_queue_storage;
static uint8_t app_command_queue_buffer[
    APP_COMMAND_QUEUE_LENGTH * sizeof(app_queued_motion_command_t)];
static QueueHandle_t app_command_queue;

static motion_command_t app_priority_commands[APP_PRIORITY_COMMAND_COUNT];
static volatile uint8_t app_priority_pending_mask;
static volatile uint32_t app_safety_epoch;

static StaticTask_t app_idle_task_storage;
static StackType_t app_idle_task_stack[configMINIMAL_STACK_SIZE];

static int8_t app_get_priority_command_index(motion_command_type_t type)
{
  if(type == MOTION_CMD_ESTOP)
  {
    return (int8_t)APP_PRIORITY_ESTOP_INDEX;
  }
  if(type == MOTION_CMD_COMM_TIMEOUT)
  {
    return (int8_t)APP_PRIORITY_COMM_TIMEOUT_INDEX;
  }
  if(type == MOTION_CMD_CAN_BUSOFF)
  {
    return (int8_t)APP_PRIORITY_CAN_BUSOFF_INDEX;
  }
  if(type == MOTION_CMD_STOP)
  {
    return (int8_t)APP_PRIORITY_STOP_INDEX;
  }
  return -1;
}

/* 通信任务只负责提交命令，底盘状态只能由控制任务修改。 */
static motion_command_handle_result_t app_submit_motion_command(
    const motion_command_t *command,
    void *context)
{
  QueueHandle_t command_queue = (QueueHandle_t)context;
  app_queued_motion_command_t queued_command;
  int8_t priority_index;

  if((command == NULL) || (command_queue == NULL))
  {
    return MOTION_HANDLE_REJECTED;
  }
  priority_index = app_get_priority_command_index(command->type);
  if(priority_index >= 0)
  {
    uint8_t priority_mask = (uint8_t)(1U << priority_index);

    taskENTER_CRITICAL();
    if((app_priority_pending_mask & priority_mask) == 0U)
    {
      app_safety_epoch++;
      if(app_safety_epoch == 0UL)
      {
        app_safety_epoch = 1UL;
      }
    }
    app_priority_commands[(uint8_t)priority_index] = *command;
    app_priority_pending_mask |= priority_mask;
    taskEXIT_CRITICAL();
    return MOTION_HANDLE_RECEIVED;
  }

  queued_command.command = *command;
  taskENTER_CRITICAL();
  if(app_priority_pending_mask != 0U)
  {
    taskEXIT_CRITICAL();
    return MOTION_HANDLE_BUSY;
  }
  queued_command.safety_epoch = app_safety_epoch;
  taskEXIT_CRITICAL();

  if(xQueueSend(command_queue, &queued_command, 0U) != pdPASS)
  {
    return MOTION_HANDLE_BUSY;
  }
  return MOTION_HANDLE_RECEIVED;
}

static uint8_t app_take_priority_command(motion_command_t *command)
{
  uint8_t index;
  uint8_t found = 0U;

  taskENTER_CRITICAL();
  for(index = 0U; index < APP_PRIORITY_COMMAND_COUNT; index++)
  {
    if((app_priority_pending_mask & (uint8_t)(1U << index)) != 0U)
    {
      *command = app_priority_commands[index];
      app_priority_pending_mask &= (uint8_t)~(1U << index);
      found = 1U;
      break;
    }
  }
  taskEXIT_CRITICAL();
  return found;
}

static uint32_t app_get_safety_epoch(void)
{
  uint32_t safety_epoch;

  taskENTER_CRITICAL();
  safety_epoch = app_safety_epoch;
  taskEXIT_CRITICAL();
  return safety_epoch;
}

static void app_send_motion_result(const motion_command_t *command,
                                   motion_command_handle_result_t result)
{
  if(command->source == MOTION_SOURCE_SERIAL)
  {
    serial_motion_protocol_send_result(command, result);
  }
  else if(command->source == MOTION_SOURCE_CANFD)
  {
    canfd_protocol_send_motion_result(
        command,
        result,
        chassis_motion_controller_get_fault_mask(),
        chassis_motion_controller_get_state());
  }
}

static void app_control_task(void *parameter)
{
  TickType_t last_wake_tick = xTaskGetTickCount();
  app_queued_motion_command_t queued_command;
  motion_command_t command;
  motion_command_handle_result_t result;

  (void)parameter;
  for(;;)
  {
    vTaskDelayUntil(&last_wake_tick,
                    pdMS_TO_TICKS(APP_CONTROL_PERIOD_MS));

    while(app_take_priority_command(&command) != 0U)
    {
      result = chassis_motion_controller_handle_command(&command);
      app_send_motion_result(&command, result);
    }

    while(xQueueReceive(app_command_queue, &queued_command, 0U) == pdPASS)
    {
      if(queued_command.safety_epoch != app_get_safety_epoch())
      {
        app_send_motion_result(&queued_command.command,
                               MOTION_HANDLE_REJECTED);
        continue;
      }
      result = chassis_motion_controller_handle_command(
          &queued_command.command);
      app_send_motion_result(&queued_command.command, result);
    }

    chassis_motion_controller_step_10ms();
  }
}

static void app_canfd_task(void *parameter)
{
  TickType_t last_wake_tick = xTaskGetTickCount();

  (void)parameter;
  for(;;)
  {
    canfd_protocol_poll(
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS));
    vTaskDelayUntil(&last_wake_tick,
                    pdMS_TO_TICKS(APP_CANFD_POLL_PERIOD_MS));
  }
}

static void app_serial_task(void *parameter)
{
  TickType_t last_wake_tick = xTaskGetTickCount();

  (void)parameter;
  for(;;)
  {
    serial_motion_protocol_poll();
    vTaskDelayUntil(&last_wake_tick,
                    pdMS_TO_TICKS(APP_SERIAL_POLL_PERIOD_MS));
  }
}

uint8_t app_freertos_init(void)
{
  debug_uart_log_init();

  app_command_queue = xQueueCreateStatic(
      APP_COMMAND_QUEUE_LENGTH,
      sizeof(app_queued_motion_command_t),
      app_command_queue_buffer,
      &app_command_queue_storage);
  if(app_command_queue == NULL)
  {
    return 0U;
  }
  vQueueAddToRegistry(app_command_queue, "MotionCmdQ");
  app_priority_pending_mask = 0U;
  app_safety_epoch = 1UL;
  serial_motion_protocol_init(app_submit_motion_command,
                              app_command_queue);
  canfd_protocol_init(app_submit_motion_command,
                      app_command_queue);

  app_control_task_handle = xTaskCreateStatic(
      app_control_task,
      "ChassisCtrl",
      APP_CONTROL_TASK_STACK_WORDS,
      NULL,
      APP_CONTROL_TASK_PRIORITY,
      app_control_task_stack,
      &app_control_task_storage);
  app_serial_task_handle = xTaskCreateStatic(
      app_serial_task,
      "SerialComm",
      APP_SERIAL_TASK_STACK_WORDS,
      NULL,
      APP_SERIAL_TASK_PRIORITY,
      app_serial_task_stack,
      &app_serial_task_storage);
  app_canfd_task_handle = xTaskCreateStatic(
      app_canfd_task,
      "CANFD",
      APP_CANFD_TASK_STACK_WORDS,
      NULL,
      APP_CANFD_TASK_PRIORITY,
      app_canfd_task_stack,
      &app_canfd_task_storage);

  if((app_control_task_handle == NULL) ||
     (app_serial_task_handle == NULL) ||
     (app_canfd_task_handle == NULL))
  {
    return 0U;
  }

  debug_uart_log_printf(
      "FREERTOS,READY,CTRL=5,SERIAL=3,CANFD=3,CMDQ=8\r\n");
  return 1U;
}

void app_freertos_start(void)
{
  vTaskStartScheduler();

  /* 调度器返回只可能是内核启动失败，立即保持电机刹车。 */
  motor_speed_control_hold_brake();
  taskDISABLE_INTERRUPTS();
  for(;;)
  {
  }
}

void vApplicationGetIdleTaskMemory(StaticTask_t **task_buffer,
                                   StackType_t **stack_buffer,
                                   uint32_t *stack_size)
{
  *task_buffer = &app_idle_task_storage;
  *stack_buffer = app_idle_task_stack;
  *stack_size = configMINIMAL_STACK_SIZE;
}

void vApplicationStackOverflowHook(TaskHandle_t task,
                                   char *task_name)
{
  (void)task;
  (void)task_name;
  motor_speed_control_hold_brake();
  taskDISABLE_INTERRUPTS();
  for(;;)
  {
  }
}

void app_freertos_assert_failed(const char *file, int line)
{
  (void)file;
  (void)line;
  motor_speed_control_hold_brake();
  taskDISABLE_INTERRUPTS();
  for(;;)
  {
  }
}
