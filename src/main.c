/**
  **************************************************************************
  * @file     main.c
  * @brief    主程序入口
  *
  * main.c 只负责系统初始化和主循环调度。
  * 当前运行电机速度闭环 PID 测试，具体逻辑统一放在 motor_speed_pid_test.c 中，
  * 主程序只负责系统初始化和测试任务调度。
  **************************************************************************
  */

#include "at32f45x_board.h"
#include "at32f45x_clock.h"
#include "motor_speed_pid_test.h"

int main(void)
{
  /* 配置系统时钟。当前工程按板载 12MHz 外部晶振配置。 */
  system_clock_config();

  /* 初始化板级外设，例如延时函数和基础 GPIO。 */
  at32_board_init();

  /* 初始化调试串口，后续 printf 会从 USART1 输出。 */
  uart_print_init(115200);

  /* 初始化电机速度闭环 PID 测试，上电后会先等待 3 秒。 */
  motor_speed_pid_test_init();

  while(1)
  {
    /* 执行左右电机速度 PID，并输出目标、实测速度和 PWM。 */
    motor_speed_pid_test_loop();
  }
}
