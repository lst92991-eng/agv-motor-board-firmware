/**
  **************************************************************************
  * @file     main.c
  * @brief    主程序入口
  *
  * main.c只负责硬件初始化，并按照app_mode.h选择正常应用或单项测试。
  **************************************************************************
  */

#include "at32f45x_board.h"
#include "at32f45x_clock.h"
#include "app_mode.h"

#if (APP_ACTIVE_MODE == APP_MODE_CHASSIS_CONTROL)
#include "app_freertos.h"
#include "chassis_motion_controller.h"
#elif (APP_ACTIVE_MODE == APP_MODE_LEFT_MOTOR_ENCODER_TEST)
#include "left_motor_encoder_test.h"
#elif (APP_ACTIVE_MODE == APP_MODE_DUAL_MOTOR_ENCODER_TEST)
#include "dual_motor_encoder_test.h"
#elif (APP_ACTIVE_MODE == APP_MODE_LEFT_MOTOR_SCOPE_FORWARD_TEST)
#include "left_motor_scope_forward_test.h"
#elif (APP_ACTIVE_MODE == APP_MODE_RIGHT_MOTOR_SCOPE_FORWARD_TEST)
#include "right_motor_scope_forward_test.h"
#elif (APP_ACTIVE_MODE == APP_MODE_AT32F456_REPLACEMENT_TEST)
#include "at32f456_replacement_test.h"
#elif (APP_ACTIVE_MODE == APP_MODE_RIGHT_MOTOR_ENCODER_ADC_TEST)
#include "right_motor_encoder_adc_test.h"
#elif (APP_ACTIVE_MODE == APP_MODE_CANFD_TEST)
#include "canfd_test.h"
#elif (APP_ACTIVE_MODE == APP_MODE_RFID_READER_TEST)
#include "rfid_reader_test.h"
#elif (APP_ACTIVE_MODE == APP_MODE_BUMPER_TEST)
#include "bumper_test.h"
#else
#include "tof_ir_test.h"
#endif

int main(void)
{
#if (APP_ACTIVE_MODE == APP_MODE_AT32F456_REPLACEMENT_TEST)
  /* 换芯自检使用可回退的时钟配置，避免外晶振异常时串口完全无输出。 */
  at32f456_replacement_clock_config();
#else
  /* 配置系统时钟。当前工程按板载 12MHz 外部晶振配置。 */
  system_clock_config();
#endif

  /* FreeRTOS要求全部4个优先级位都用于抢占优先级。 */
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  /* 换芯自检不使用官方开发板LED和按键映射，避免驱动自定义板上的其他网络。 */
#if (APP_ACTIVE_MODE == APP_MODE_AT32F456_REPLACEMENT_TEST)
  delay_init();
#else
  /* 初始化板级外设，例如延时函数和基础 GPIO。 */
  at32_board_init();
#endif

  /* 初始化调试串口，后续 printf 会从 USART1 输出。 */
  uart_print_init(115200);

#if (APP_ACTIVE_MODE == APP_MODE_CHASSIS_CONTROL)
  /* 调度器启动前完成电机、编码器和BMI270静态初始化。 */
  chassis_motion_controller_init();

  /* 创建静态任务、命令队列和串口互斥锁。 */
  if(app_freertos_init() == 0U)
  {
    app_freertos_assert_failed(__FILE__, __LINE__);
  }

  app_freertos_start();
  return 0;
#elif (APP_ACTIVE_MODE == APP_MODE_LEFT_MOTOR_ENCODER_TEST)
  /* 独立测试不启动FreeRTOS，只运行左电机和左编码器测试状态机。 */
  left_motor_encoder_test_init();
  for(;;)
  {
    left_motor_encoder_test_loop();
  }
#elif (APP_ACTIVE_MODE == APP_MODE_DUAL_MOTOR_ENCODER_TEST)
  /* 双电机对照测试按固定时序逐个驱动电机，不启动FreeRTOS。 */
  dual_motor_encoder_test_init();
  for(;;)
  {
    dual_motor_encoder_test_loop();
  }
#elif (APP_ACTIVE_MODE == APP_MODE_LEFT_MOTOR_SCOPE_FORWARD_TEST)
  /* 示波器测试只让左电机保持开环正转，不启动FreeRTOS和PID。 */
  left_motor_scope_forward_test_init();
  for(;;)
  {
    left_motor_scope_forward_test_loop();
  }
#elif (APP_ACTIVE_MODE == APP_MODE_RIGHT_MOTOR_SCOPE_FORWARD_TEST)
  /* 右电机测试只保持开环正转，不启动其他控制和采集模块。 */
  right_motor_scope_forward_test_init();
  for(;;)
  {
    right_motor_scope_forward_test_loop();
  }
#elif (APP_ACTIVE_MODE == APP_MODE_AT32F456_REPLACEMENT_TEST)
  /* 换芯自检覆盖核心资源和常用外设，电机始终保持停止。 */
  at32f456_replacement_test_init();
  for(;;)
  {
    at32f456_replacement_test_loop();
  }
#elif (APP_ACTIVE_MODE == APP_MODE_RIGHT_MOTOR_ENCODER_ADC_TEST)
  /* 右电机联合测试只驱动右电机，左电机保持停止。 */
  right_motor_encoder_adc_test_init();
  for(;;)
  {
    right_motor_encoder_adc_test_loop();
  }
#elif (APP_ACTIVE_MODE == APP_MODE_CANFD_TEST)
  /* CAN FD测试不启动电机，只测试FDCAN3和PB3/PB4总线接口。 */
  canfd_test_init();
  for(;;)
  {
    canfd_test_loop();
  }
#elif (APP_ACTIVE_MODE == APP_MODE_RFID_READER_TEST)
  /* RFID测试只使用USART5读卡，并从USART1输出诊断信息。 */
  rfid_reader_test_init();
  for(;;)
  {
    rfid_reader_test_loop();
  }
#elif (APP_ACTIVE_MODE == APP_MODE_BUMPER_TEST)
  /* 防撞条测试只采集四路原始电平，不初始化电机和FreeRTOS。 */
  bumper_test_init();
  for(;;)
  {
    bumper_test_loop();
  }
#else
  /* TOF与红外测试不启动电机，只周期输出左右距离和红外接收状态。 */
  tof_ir_test_init();
  for(;;)
  {
    tof_ir_test_loop();
  }
#endif
}
