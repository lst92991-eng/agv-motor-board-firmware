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
#include "debug_uart_log.h"
#include "motion_command.h"
#include "motor_speed_control.h"
#include "serial_motion_protocol.h"

#define APP_CONTROL_TASK_PRIORITY                5U
#define APP_SERIAL_TASK_PRIORITY                 3U
#define APP_CONTROL_TASK_STACK_WORDS          1024U
#define APP_SERIAL_TASK_STACK_WORDS            512U
#define APP_COMMAND_QUEUE_LENGTH                  8U
#define APP_CONTROL_PERIOD_MS                    10U
#define APP_SERIAL_POLL_PERIOD_MS                 2U

static StaticTask_t app_control_task_storage;
static StackType_t app_control_task_stack[APP_CONTROL_TASK_STACK_WORDS];
static TaskHandle_t app_control_task_handle;

static StaticTask_t app_serial_task_storage;
static StackType_t app_serial_task_stack[APP_SERIAL_TASK_STACK_WORDS];
static TaskHandle_t app_serial_task_handle;

static StaticQueue_t app_command_queue_storage;
static uint8_t app_command_queue_buffer[
    APP_COMMAND_QUEUE_LENGTH * sizeof(motion_command_t)];
static QueueHandle_t app_command_queue;

static StaticTask_t app_idle_task_storage;
static StackType_t app_idle_task_stack[configMINIMAL_STACK_SIZE];

/* 通信任务只负责提交命令，底盘状态只能由控制任务修改。 */
static motion_command_handle_result_t app_submit_motion_command(
    const motion_command_t *command,
    void *context)
{
  QueueHandle_t command_queue = (QueueHandle_t)context;

  if((command == NULL) || (command_queue == NULL))
  {
    return MOTION_HANDLE_REJECTED;
  }
  if(xQueueSend(command_queue, command, 0U) != pdPASS)
  {
    return MOTION_HANDLE_BUSY;
  }
  return MOTION_HANDLE_RECEIVED;
}

static void app_control_task(void *parameter)
{
  TickType_t last_wake_tick = xTaskGetTickCount();
  motion_command_t command;
  motion_command_handle_result_t result;

  (void)parameter;
  for(;;)
  {
    vTaskDelayUntil(&last_wake_tick,
                    pdMS_TO_TICKS(APP_CONTROL_PERIOD_MS));

    while(xQueueReceive(app_command_queue, &command, 0U) == pdPASS)
    {
      result = chassis_motion_controller_handle_command(&command);
      serial_motion_protocol_send_result(&command, result);
    }

    chassis_motion_controller_step_10ms();
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
      sizeof(motion_command_t),
      app_command_queue_buffer,
      &app_command_queue_storage);
  if(app_command_queue == NULL)
  {
    return 0U;
  }
  vQueueAddToRegistry(app_command_queue, "MotionCmdQ");
  serial_motion_protocol_init(app_submit_motion_command,
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

  if((app_control_task_handle == NULL) ||
     (app_serial_task_handle == NULL))
  {
    return 0U;
  }

  debug_uart_log_printf("FREERTOS,READY,CTRL=5,SERIAL=3,CMDQ=8\r\n");
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
