/**
 * @file app_freertos.c
 * @brief 底盘四个静态任务和唯一命令队列
 *
 * ChassisCtrl是运动状态唯一写者。安全命令使用固定优先槽并推进safety_epoch，
 * 普通命令携带入队epoch，确保STOP/ESTOP之后不会继续执行旧命令。
 */

#include "app_freertos.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "adc_monitor.h"
#include "canfd_protocol.h"
#include "chassis_motion_controller.h"
#include "motion_command.h"
#include "motor_encoder.h"
#include "motor_speed_control.h"
#include "power_supervisor.h"
#include "system_watchdog.h"

#define APP_CONTROL_PRIORITY       5U
#define APP_CAN_PRIORITY           3U
#define APP_POWER_PRIORITY         2U
#define APP_WATCHDOG_PRIORITY      1U
#define APP_CONTROL_STACK       1024U
#define APP_CAN_STACK            768U
#define APP_POWER_STACK          256U
#define APP_WATCHDOG_STACK       256U
#define APP_COMMAND_QUEUE_LENGTH   8U
#define APP_CONTROL_PERIOD_MS     10U
#define APP_CAN_PERIOD_MS          2U
#define APP_WATCHDOG_PERIOD_MS    20U

typedef struct
{
  motion_command_t command;
  /* 入队时安全代次；与当前代次不一致表示已被新安全事件越过。 */
  uint32_t safety_epoch;
} app_queued_command_t;

enum
{
  APP_PRIORITY_ESTOP,
  APP_PRIORITY_POWER,
  APP_PRIORITY_TIMEOUT,
  APP_PRIORITY_BUSOFF,
  APP_PRIORITY_STOP,
  APP_PRIORITY_COUNT
};

static StaticTask_t app_control_task_storage;
static StaticTask_t app_can_task_storage;
static StaticTask_t app_power_task_storage;
static StaticTask_t app_watchdog_task_storage;
static StackType_t app_control_task_stack[APP_CONTROL_STACK];
static StackType_t app_can_task_stack[APP_CAN_STACK];
static StackType_t app_power_task_stack[APP_POWER_STACK];
static StackType_t app_watchdog_task_stack[APP_WATCHDOG_STACK];
static TaskHandle_t app_control_task_handle;
static TaskHandle_t app_can_task_handle;
static TaskHandle_t app_power_task_handle;
static TaskHandle_t app_watchdog_task_handle;

static StaticQueue_t app_command_queue_storage;
static uint8_t app_command_queue_buffer[
    APP_COMMAND_QUEUE_LENGTH * sizeof(app_queued_command_t)];
static QueueHandle_t app_command_queue;
static motion_command_t app_priority_commands[APP_PRIORITY_COUNT];
/* 每类安全事件保留一个latest槽，不会因普通队列满而阻塞。 */
static volatile uint8_t app_priority_pending;
static volatile uint32_t app_safety_epoch;
static volatile uint32_t app_control_generation;
static volatile uint32_t app_can_generation;
static volatile uint32_t app_power_generation;

static StaticTask_t app_idle_task_storage;
static StackType_t app_idle_task_stack[configMINIMAL_STACK_SIZE];

/** @brief 将安全命令类型映射到固定优先槽，普通命令返回-1。 */
static int8_t app_priority_index(motion_command_type_t type)
{
  switch(type)
  {
    case MOTION_CMD_ESTOP: return APP_PRIORITY_ESTOP;
    case MOTION_CMD_POWER_FAULT: return APP_PRIORITY_POWER;
    case MOTION_CMD_COMM_TIMEOUT: return APP_PRIORITY_TIMEOUT;
    case MOTION_CMD_CAN_BUSOFF: return APP_PRIORITY_BUSOFF;
    case MOTION_CMD_STOP: return APP_PRIORITY_STOP;
    default: return -1;
  }
}

/** @brief 将安全命令投递到优先槽，其余命令带epoch无等待入队。 */
static motion_command_handle_result_t app_submit_command(
    const motion_command_t *command, void *context)
{
  app_queued_command_t queued;
  QueueHandle_t queue = (QueueHandle_t)context;
  int8_t index;

  if((command == NULL) || (queue == NULL))
  {
    return MOTION_HANDLE_REJECTED;
  }
  index = app_priority_index(command->type);
  if(index >= 0)
  {
    uint8_t mask = (uint8_t)(1U << index);

    taskENTER_CRITICAL();
    if((app_priority_pending & mask) == 0U)
    {
      /* 同类事件已挂起时覆盖内容但不重复推进epoch。 */
      app_safety_epoch++;
      if(app_safety_epoch == 0U)
      {
        app_safety_epoch = 1U;
      }
    }
    app_priority_commands[(uint8_t)index] = *command;
    app_priority_pending |= mask;
    taskEXIT_CRITICAL();
    return MOTION_HANDLE_RECEIVED;
  }

  queued.command = *command;
  taskENTER_CRITICAL();
  queued.safety_epoch = app_safety_epoch;
  taskEXIT_CRITICAL();
  return (xQueueSend(queue, &queued, 0U) == pdPASS) ?
      MOTION_HANDLE_RECEIVED : MOTION_HANDLE_BUSY;
}

/** @brief 按固定优先级原子取出一个待处理安全命令。 */
static uint8_t app_take_priority(motion_command_t *command)
{
  uint8_t index;

  /* 数组顺序即安全优先级：ESTOP和电气故障先于普通STOP。 */
  taskENTER_CRITICAL();
  for(index = 0U; index < APP_PRIORITY_COUNT; index++)
  {
    uint8_t mask = (uint8_t)(1U << index);
    if((app_priority_pending & mask) != 0U)
    {
      *command = app_priority_commands[index];
      app_priority_pending &= (uint8_t)~mask;
      taskEXIT_CRITICAL();
      return 1U;
    }
  }
  taskEXIT_CRITICAL();
  return 0U;
}

/** @brief 原子读取当前安全代次，供普通命令执行前复核。 */
static uint32_t app_current_epoch(void)
{
  uint32_t epoch;

  taskENTER_CRITICAL();
  epoch = app_safety_epoch;
  taskEXIT_CRITICAL();
  return epoch;
}

/** @brief 将CAN来源命令的最终处理结果发布到0x281。 */
static void app_finish_command(const motion_command_t *command,
                               motion_command_handle_result_t result)
{
  if(command->source == MOTION_SOURCE_CANFD)
  {
    canfd_protocol_send_motion_result(command, result,
        chassis_motion_controller_get_fault_mask(),
        chassis_motion_controller_get_state());
  }
}

/** @brief 最高优先级10ms任务：安全命令、普通命令和唯一底盘状态机。 */
static void app_control_task(void *parameter)
{
  TickType_t last = xTaskGetTickCount();
  app_queued_command_t queued;
  motion_command_t command;
  motion_command_handle_result_t result;

  (void)parameter;
  for(;;)
  {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(APP_CONTROL_PERIOD_MS));
    /* 每个10ms周期先耗尽安全槽，再处理普通队列。 */
    while(app_take_priority(&command) != 0U)
    {
      result = chassis_motion_controller_handle_command(&command);
      app_finish_command(&command, result);
    }
    while(xQueueReceive(app_command_queue, &queued, 0U) == pdPASS)
    {
      if(queued.safety_epoch != app_current_epoch())
      {
        /* 安全事件之后遗留的普通命令只能被拒绝，不能恢复运动。 */
        app_finish_command(&queued.command, MOTION_HANDLE_REJECTED);
        continue;
      }
      result = chassis_motion_controller_handle_command(&queued.command);
      app_finish_command(&queued.command, result);
    }
    chassis_motion_controller_step_10ms();
    app_control_generation++;
  }
}

/** @brief 2ms通信任务：协议轮询、租约检查、链路恢复和TX推进。 */
static void app_can_task(void *parameter)
{
  TickType_t last = xTaskGetTickCount();

  (void)parameter;
  for(;;)
  {
    canfd_protocol_poll((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS));
    app_can_generation++;
    vTaskDelayUntil(&last, pdMS_TO_TICKS(APP_CAN_PERIOD_MS));
  }
}

/** @brief 20ms电气任务：ADC采样、静止零点证据和Power故障投递。 */
static void app_power_task(void *parameter)
{
  TickType_t last = xTaskGetTickCount();
  motion_command_t fault = {0};
  uint8_t adc_ready = adc_monitor_init();
  int32_t left = motor_encoder_get_count(MOTOR_LEFT);
  int32_t right = motor_encoder_get_count(MOTOR_RIGHT);

  (void)parameter;
  fault.version = MOTION_PROTOCOL_VERSION;
  fault.source = MOTION_SOURCE_LOCAL_SAFETY;
  fault.type = MOTION_CMD_POWER_FAULT;
  for(;;)
  {
    uint32_t before = chassis_motion_controller_get_power_status_snapshot();
    uint8_t valid;
    uint8_t zero_allowed;
    int32_t previous_left = left;
    int32_t previous_right = right;

    if(adc_ready == 0U)
    {
      adc_ready = adc_monitor_init();
    }
    valid = ((adc_ready != 0U) && (adc_monitor_update() != 0U)) ? 1U : 0U;
    left = motor_encoder_get_count(MOTOR_LEFT);
    right = motor_encoder_get_count(MOTOR_RIGHT);
    /* 控制epoch与编码器共同证明本采样周期始终处于零PWM静止状态。 */
    zero_allowed = ((valid != 0U) &&
        (before == chassis_motion_controller_get_power_status_snapshot()) &&
        ((before & CHASSIS_POWER_STATUS_ZERO_SAFE) != 0U) &&
        (left == previous_left) && (right == previous_right)) ? 1U : 0U;
    power_supervisor_step(valid, adc_monitor_get_raw(ADC_MONITOR_PA3),
        adc_monitor_get_raw(ADC_MONITOR_PA4),
        adc_monitor_get_raw(ADC_MONITOR_PA5), zero_allowed);
    if(((power_supervisor_get_fault_mask() & POWER_FAULT_SEVERE_MASK) != 0U) &&
       (chassis_motion_controller_has_power_fault() == 0U))
    {
      fault.sequence++;
      (void)app_submit_command(&fault, app_command_queue);
    }
    app_power_generation++;
    vTaskDelayUntil(&last, pdMS_TO_TICKS(
        power_supervisor_provisional_config()->sample_period_ms));
  }
}

/** @brief 20ms健康任务：三关键任务均有进展时才喂独立WDT。 */
static void app_watchdog_task(void *parameter)
{
  TickType_t last = xTaskGetTickCount();
  system_watchdog_health_t health = {0};

  (void)parameter;
  for(;;)
  {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(APP_WATCHDOG_PERIOD_MS));
    /* Control/CAN/Power任一generation停滞，本周期就不喂硬件WDT。 */
    if(system_watchdog_health_step(&health, app_control_generation,
        app_can_generation, app_power_generation) != 0U)
    {
      system_watchdog_feed();
    }
  }
}

/** @brief 创建静态队列、四任务、协议回调和硬件看门狗。 */
uint8_t app_freertos_init(void)
{
  /* 队列、任务控制块和栈全部为静态存储，运行期不依赖heap。 */
  power_supervisor_init(power_supervisor_provisional_config());
  app_command_queue = xQueueCreateStatic(APP_COMMAND_QUEUE_LENGTH,
      sizeof(app_queued_command_t), app_command_queue_buffer,
      &app_command_queue_storage);
  if(app_command_queue == NULL)
  {
    return 0U;
  }
  app_safety_epoch = 1U;
  app_priority_pending = 0U;
  canfd_protocol_init(app_submit_command, app_command_queue);

  app_control_task_handle = xTaskCreateStatic(app_control_task, "ChassisCtrl",
      APP_CONTROL_STACK, NULL, APP_CONTROL_PRIORITY,
      app_control_task_stack, &app_control_task_storage);
  app_can_task_handle = xTaskCreateStatic(app_can_task, "CANFD",
      APP_CAN_STACK, NULL, APP_CAN_PRIORITY,
      app_can_task_stack, &app_can_task_storage);
  app_power_task_handle = xTaskCreateStatic(app_power_task, "PowerMonitor",
      APP_POWER_STACK, NULL, APP_POWER_PRIORITY,
      app_power_task_stack, &app_power_task_storage);
  app_watchdog_task_handle = xTaskCreateStatic(app_watchdog_task, "Watchdog",
      APP_WATCHDOG_STACK, NULL, APP_WATCHDOG_PRIORITY,
      app_watchdog_task_stack, &app_watchdog_task_storage);

  return ((app_control_task_handle != NULL) &&
          (app_can_task_handle != NULL) &&
          (app_power_task_handle != NULL) &&
          (app_watchdog_task_handle != NULL) &&
          (system_watchdog_init() != 0U)) ? 1U : 0U;
}

/** @brief 启动调度器；意外返回时主动制动并永久关中断。 */
void app_freertos_start(void)
{
  vTaskStartScheduler();
  motor_speed_control_hold_brake();
  taskDISABLE_INTERRUPTS();
  for(;;) {}
}

/** @brief 向FreeRTOS提供静态Idle任务控制块和栈。 */
void vApplicationGetIdleTaskMemory(StaticTask_t **task_buffer,
                                   StackType_t **stack_buffer,
                                   uint32_t *stack_size)
{
  *task_buffer = &app_idle_task_storage;
  *stack_buffer = app_idle_task_stack;
  *stack_size = configMINIMAL_STACK_SIZE;
}

/** @brief 栈溢出钩子，统一进入不可返回的主动制动失败路径。 */
void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
  (void)task;
  (void)task_name;
  app_freertos_assert_failed(NULL, 0);
}

/** @brief 处理断言/调度器失败：主动制动、关中断并停机等待复位。 */
void app_freertos_assert_failed(const char *file, int line)
{
  (void)file;
  (void)line;
  motor_speed_control_hold_brake();
  taskDISABLE_INTERRUPTS();
  for(;;) {}
}
