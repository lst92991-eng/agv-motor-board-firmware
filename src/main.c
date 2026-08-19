/**
  **************************************************************************
  * @file     main.c
  * @brief    主程序入口
  *
 * main.c只负责正式底盘固件的硬件初始化和FreeRTOS启动。
  **************************************************************************
  */

#include "at32f45x_board.h"
#include "at32f45x_clock.h"
#include "app_freertos.h"
#include "chassis_motion_controller.h"

int main(void)
{
  /* 配置系统时钟。当前工程按板载 12MHz 外部晶振配置。 */
  system_clock_config();

  /* FreeRTOS要求全部4个优先级位都用于抢占优先级。 */
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  /* 初始化板级外设，例如延时函数和基础 GPIO。 */
  at32_board_init();

  /* 初始化调试串口，后续 printf 会从 USART1 输出。 */
  uart_print_init(115200);

  /* 调度器启动前完成电机、编码器和BMI270静态初始化。 */
  chassis_motion_controller_init();

  /* 创建静态任务、命令队列和串口互斥锁。 */
  if(app_freertos_init() == 0U)
  {
    app_freertos_assert_failed(__FILE__, __LINE__);
  }

  app_freertos_start();
  return 0;
}
