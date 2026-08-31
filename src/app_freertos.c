/**
  *****************************************************************************
  * @file     app_freertos.c
  * @brief    AGV底盘FreeRTOS任务、队列和异常钩子
  *****************************************************************************
  */

#include "app_freertos.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "adc_monitor.h"
#include "chassis_motion_controller.h"
#include "canfd_protocol.h"
#include "debug_uart_log.h"
#include "dock_ir_receiver.h"
#include "dock_tof_sensor.h"
#include "motion_command.h"
#include "motor_current.h"
#include "motor_encoder.h"
#include "motor_speed_control.h"
#include "power_supervisor.h"
#include "serial_motion_protocol.h"
#include "system_watchdog.h"

#define APP_CONTROL_TASK_PRIORITY                5U
#define APP_POWER_TASK_PRIORITY                  2U
#define APP_SERIAL_TASK_PRIORITY                 3U
#define APP_CANFD_TASK_PRIORITY                  3U
#define APP_DOCK_TASK_PRIORITY                   2U
#define APP_WATCHDOG_TASK_PRIORITY               1U
#define APP_CONTROL_TASK_STACK_WORDS          1024U
#define APP_POWER_TASK_STACK_WORDS             256U
#define APP_SERIAL_TASK_STACK_WORDS            512U
#define APP_CANFD_TASK_STACK_WORDS             768U
#define APP_DOCK_TASK_STACK_WORDS             2048U
#define APP_WATCHDOG_TASK_STACK_WORDS           256U
#define APP_COMMAND_QUEUE_LENGTH                  8U
#define APP_CONTROL_PERIOD_MS                    10U
#define APP_POWER_DIAGNOSTIC_PERIOD_SAMPLES      25U
#define APP_SERIAL_POLL_PERIOD_MS                 2U
#define APP_CANFD_POLL_PERIOD_MS                  2U
#define APP_DOCK_SAMPLE_PERIOD_MS                 5U
#define APP_WATCHDOG_PERIOD_MS                   20U
#define APP_PRIORITY_COMMAND_COUNT                5U

#define APP_PRIORITY_ESTOP_INDEX                  0U
#define APP_PRIORITY_POWER_FAULT_INDEX            1U
#define APP_PRIORITY_COMM_TIMEOUT_INDEX           2U
#define APP_PRIORITY_CAN_BUSOFF_INDEX              3U
#define APP_PRIORITY_STOP_INDEX                   4U

typedef struct
{
  motion_command_t command;
  uint32_t safety_epoch;
} app_queued_motion_command_t;

static StaticTask_t app_control_task_storage;
static StackType_t app_control_task_stack[APP_CONTROL_TASK_STACK_WORDS];
static TaskHandle_t app_control_task_handle;

static StaticTask_t app_power_task_storage;
static StackType_t app_power_task_stack[APP_POWER_TASK_STACK_WORDS];
static TaskHandle_t app_power_task_handle;

static StaticTask_t app_serial_task_storage;
static StackType_t app_serial_task_stack[APP_SERIAL_TASK_STACK_WORDS];
static TaskHandle_t app_serial_task_handle;

static StaticTask_t app_canfd_task_storage;
static StackType_t app_canfd_task_stack[APP_CANFD_TASK_STACK_WORDS];
static TaskHandle_t app_canfd_task_handle;

static StaticTask_t app_dock_task_storage;
static StackType_t app_dock_task_stack[APP_DOCK_TASK_STACK_WORDS];
static TaskHandle_t app_dock_task_handle;

static StaticTask_t app_watchdog_task_storage;
static StackType_t app_watchdog_task_stack[APP_WATCHDOG_TASK_STACK_WORDS];
static TaskHandle_t app_watchdog_task_handle;

static StaticQueue_t app_command_queue_storage;
static uint8_t app_command_queue_buffer[
    APP_COMMAND_QUEUE_LENGTH * sizeof(app_queued_motion_command_t)];
static QueueHandle_t app_command_queue;

static motion_command_t app_priority_commands[APP_PRIORITY_COMMAND_COUNT];
static volatile uint8_t app_priority_pending_mask;
static volatile uint32_t app_safety_epoch;
static volatile uint32_t app_control_generation;
static volatile uint32_t app_can_generation;
static volatile uint32_t app_power_generation;

static StaticTask_t app_idle_task_storage;
static StackType_t app_idle_task_stack[configMINIMAL_STACK_SIZE];

static motion_command_handle_result_t app_submit_motion_command(
    const motion_command_t *command,
    void *context);

static int8_t app_get_priority_command_index(motion_command_type_t type)
{
  if(type == MOTION_CMD_ESTOP)
  {
    return (int8_t)APP_PRIORITY_ESTOP_INDEX;
  }
  if(type == MOTION_CMD_POWER_FAULT)
  {
    return (int8_t)APP_PRIORITY_POWER_FAULT_INDEX;
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

static void app_power_task(void *parameter)
{
  TickType_t last_wake_tick = xTaskGetTickCount();
  motion_command_t command;
  uint8_t adc_ready = motor_current_init();
  uint8_t sample_valid;
  uint8_t severe_fault;
  uint8_t zero_allowed;
  uint8_t diagnostic_samples = 0U;
  uint16_t bus_raw;
  uint16_t left_raw;
  uint16_t right_raw;
  power_supervisor_snapshot_t power_snapshot;
  int32_t left_count = motor_encoder_get_count(MOTOR_LEFT);
  int32_t right_count = motor_encoder_get_count(MOTOR_RIGHT);
  int32_t previous_left_count = left_count;
  int32_t previous_right_count = right_count;
  uint32_t power_status_before;
  uint32_t power_status_after;

  (void)parameter;
  memset(&command, 0, sizeof(command));
  command.version = MOTION_PROTOCOL_VERSION;
  command.source = MOTION_SOURCE_LOCAL_SAFETY;
  command.type = MOTION_CMD_POWER_FAULT;
  for(;;)
  {
    power_status_before =
        chassis_motion_controller_get_power_status_snapshot();
    if(adc_ready == 0U)
    {
      adc_ready = motor_current_init();
    }
    sample_valid = ((adc_ready != 0U) &&
                    (motor_current_update() != 0U)) ? 1U : 0U;
    bus_raw = adc_monitor_get_raw(ADC_MONITOR_PA3);
    left_raw = motor_current_get_raw(MOTOR_LEFT);
    right_raw = motor_current_get_raw(MOTOR_RIGHT);
    power_status_after =
        chassis_motion_controller_get_power_status_snapshot();
    left_count = motor_encoder_get_count(MOTOR_LEFT);
    right_count = motor_encoder_get_count(MOTOR_RIGHT);
    zero_allowed =
        ((sample_valid != 0U) &&
         (power_status_before == power_status_after) &&
         ((power_status_after & CHASSIS_POWER_STATUS_ZERO_SAFE) != 0UL) &&
         (left_count == previous_left_count) &&
         (right_count == previous_right_count)) ? 1U : 0U;
    previous_left_count = left_count;
    previous_right_count = right_count;
    power_supervisor_step(
        sample_valid,
        bus_raw,
        left_raw,
        right_raw,
        zero_allowed);
    severe_fault = (uint8_t)(power_supervisor_get_fault_mask() &
                             POWER_FAULT_SEVERE_MASK);
    if((severe_fault != 0U) &&
       (chassis_motion_controller_has_power_fault() == 0U))
    {
      command.sequence = 1UL;
      (void)app_submit_motion_command(&command, app_command_queue);
    }
    diagnostic_samples++;
    if(diagnostic_samples >= APP_POWER_DIAGNOSTIC_PERIOD_SAMPLES)
    {
      diagnostic_samples = 0U;
      power_supervisor_get_snapshot(&power_snapshot);
      (void)debug_uart_log_printf(
          "POWER,valid=%u,raw=%u,%u,%u,bus_mv=%lu,current_ma=%lu,%lu,"
          "zero=%u,%u,samples=%u,fault=0x%02X,ready=%u\r\n",
          (unsigned int)sample_valid,
          (unsigned int)bus_raw,
          (unsigned int)left_raw,
          (unsigned int)right_raw,
          (unsigned long)power_snapshot.bus_mv,
          (unsigned long)power_snapshot.left_current_ma,
          (unsigned long)power_snapshot.right_current_ma,
          (unsigned int)power_snapshot.left_zero_raw,
          (unsigned int)power_snapshot.right_zero_raw,
          (unsigned int)power_snapshot.zero_samples,
          (unsigned int)power_snapshot.fault_mask,
          (unsigned int)power_snapshot.ready);
    }
    app_power_generation++;
    vTaskDelayUntil(&last_wake_tick,
                    pdMS_TO_TICKS(
                        power_supervisor_provisional_config()->sample_period_ms));
  }
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
    app_control_generation++;
  }
}

static void app_canfd_task(void *parameter)
{
  TickType_t last_wake_tick = xTaskGetTickCount();

  (void)parameter;
  for(;;)
  {
    dock_ir_receiver_sample_2ms();
    canfd_protocol_poll(
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS));
    app_can_generation++;
    vTaskDelayUntil(&last_wake_tick,
                    pdMS_TO_TICKS(APP_CANFD_POLL_PERIOD_MS));
  }
}

static void app_watchdog_task(void *parameter)
{
  system_watchdog_health_t health = {0};
  TickType_t last_wake_tick = xTaskGetTickCount();

  (void)parameter;
  for(;;)
  {
    vTaskDelayUntil(&last_wake_tick,
                    pdMS_TO_TICKS(APP_WATCHDOG_PERIOD_MS));
    if(system_watchdog_health_step(&health,
                                   app_control_generation,
                                   app_can_generation,
                                   app_power_generation) != 0U)
    {
      system_watchdog_feed();
    }
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

static int16_t app_dock_to_i16(int32_t value)
{
  if(value > INT16_MAX)
  {
    return INT16_MAX;
  }
  if(value < INT16_MIN)
  {
    return INT16_MIN;
  }
  return (int16_t)value;
}

static void app_dock_task(void *parameter)
{
  dock_tof_snapshot_t snapshot;
  canfd_dock_status_t status;
  canfd_dock_scan_t scan;
  TickType_t last_wake_tick = xTaskGetTickCount();
  uint32_t last_sample_sequence = 0UL;
  uint32_t sample_started_ms;
  uint32_t sample_finished_ms;
  uint32_t sample_time_ms;
  uint8_t scl_level;
  uint8_t sda_level;
  int32_t bus_error;

  (void)parameter;
  memset(&snapshot, 0, sizeof(snapshot));
  (void)dock_tof_sensor_init(NULL);
  for(;;)
  {
    sample_started_ms = chassis_motion_controller_get_uptime_ms();
    (void)dock_tof_sensor_sample(&snapshot);
    sample_finished_ms = chassis_motion_controller_get_uptime_ms();
    if((snapshot.sample_sequence != 0UL) &&
       (snapshot.sample_sequence != last_sample_sequence))
    {
      sample_time_ms = sample_started_ms +
          ((sample_finished_ms - sample_started_ms) / 2UL);
      memset(&scan, 0, sizeof(scan));
      scan.sample_sequence = snapshot.sample_sequence;
      scan.sample_time_ms = sample_time_ms;
      scan.left_raw_mm = snapshot.left_raw_mm;
      scan.right_raw_mm = snapshot.right_raw_mm;
      scan.flags =
          ((dock_tof_sensor_is_ready() != 0U) ? 0x01U : 0U) |
          ((snapshot.left_valid != 0U) ? 0x02U : 0U) |
          ((snapshot.right_valid != 0U) ? 0x04U : 0U) |
          ((snapshot.pair_valid != 0U) ? 0x08U : 0U);
      scan.left_range_status = snapshot.left_range_status;
      scan.right_range_status = snapshot.right_range_status;
      canfd_protocol_send_dock_scan(&scan);

      dock_tof_sensor_get_bus_diagnostics(&scl_level, &sda_level,
                                          &bus_error);
      memset(&status, 0, sizeof(status));
      status.uptime_ms = sample_time_ms;
      status.sample_sequence = snapshot.sample_sequence;
      status.left_mm = snapshot.left_filtered_mm;
      status.right_mm = snapshot.right_filtered_mm;
      status.distance_error_mm = app_dock_to_i16(
          snapshot.distance_error_mm);
      status.alignment_error_mm = app_dock_to_i16(
          snapshot.alignment_error_mm);
      status.ir_falling_edges = dock_ir_receiver_get_falling_edges();
      status.flags = scan.flags |
          ((dock_ir_receiver_get_activity() != 0U) ? 0x10U : 0U) |
          ((scl_level != 0U) ? 0x20U : 0U) |
          ((sda_level != 0U) ? 0x40U : 0U);
      status.ir_activity_percent =
          dock_ir_receiver_get_activity_percent();
      status.left_api_status = snapshot.left_api_status;
      status.right_api_status = snapshot.right_api_status;
      status.left_range_status = snapshot.left_range_status;
      status.right_range_status = snapshot.right_range_status;
      status.bus_error = bus_error;
      status.left_raw_mm = snapshot.left_raw_mm;
      status.right_raw_mm = snapshot.right_raw_mm;
      canfd_protocol_send_dock_status(&status);
      last_sample_sequence = snapshot.sample_sequence;
    }
    vTaskDelayUntil(&last_wake_tick,
                    pdMS_TO_TICKS(APP_DOCK_SAMPLE_PERIOD_MS));
  }
}

uint8_t app_freertos_init(void)
{
  debug_uart_log_init();
  dock_ir_receiver_init();
  power_supervisor_init(power_supervisor_provisional_config());

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
  app_control_generation = 0UL;
  app_can_generation = 0UL;
  app_power_generation = 0UL;
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
  app_power_task_handle = xTaskCreateStatic(
      app_power_task,
      "PowerMonitor",
      APP_POWER_TASK_STACK_WORDS,
      NULL,
      APP_POWER_TASK_PRIORITY,
      app_power_task_stack,
      &app_power_task_storage);
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
  app_dock_task_handle = xTaskCreateStatic(
      app_dock_task,
      "DockSensors",
      APP_DOCK_TASK_STACK_WORDS,
      NULL,
      APP_DOCK_TASK_PRIORITY,
      app_dock_task_stack,
      &app_dock_task_storage);
  app_watchdog_task_handle = xTaskCreateStatic(
      app_watchdog_task,
      "Watchdog",
      APP_WATCHDOG_TASK_STACK_WORDS,
      NULL,
      APP_WATCHDOG_TASK_PRIORITY,
      app_watchdog_task_stack,
      &app_watchdog_task_storage);

  if((app_control_task_handle == NULL) ||
     (app_power_task_handle == NULL) ||
     (app_serial_task_handle == NULL) ||
     (app_canfd_task_handle == NULL) ||
     (app_dock_task_handle == NULL) ||
     (app_watchdog_task_handle == NULL) ||
     (system_watchdog_init() == 0U))
  {
    return 0U;
  }

  debug_uart_log_printf(
      "FREERTOS,READY,CTRL=5,SERIAL=3,CANFD=3,POWER=2,DOCK=2,CMDQ=8\r\n");
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
