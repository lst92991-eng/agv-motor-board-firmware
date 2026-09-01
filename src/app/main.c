/**
  **************************************************************************
  * @file     main.c
  * @brief    主程序入口
  *
 * main.c只负责正式底盘固件的硬件初始化和10ms控制循环。
  **************************************************************************
  */

#include "at32f45x_board.h"
#include "at32f45x_clock.h"
#include "chassis_app.h"
#include "motor_driver.h"

/** @brief 完成板级初始化、创建静态任务并启动唯一生产调度器。 */
int main(void)
{
  /* 配置系统时钟。当前工程按板载 12MHz 外部晶振配置。 */
  system_clock_config();

  /* 所有外设中断使用同一组抢占优先级配置。 */
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  /* 初始化板级外设，例如延时函数和基础 GPIO。 */
  at32_board_init();

  /* 唯一底盘任务完成驱动、协议、安全和控制初始化。 */
  if(chassis_app_init() == 0U)
  {
    motor_driver_stop_all(MOTOR_STOP_BRAKE);
    for(;;) {}
  }

  for(;;)
  {
    chassis_app_step();
    delay_ms(10U);
  }
}
