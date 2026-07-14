/**
  **************************************************************************
  * @file     main.c
  * @brief    主程序入口
  *
  * main.c 只负责系统初始化和主循环调度。
  * BMI270 的通信、初始化、数据计算和串口输出统一放在 bmi270_test.c 中，
  * 这样主程序保持干净，后面再测试其他模块时也更容易拆分。
  **************************************************************************
  */

#include "at32f45x_board.h"
#include "at32f45x_clock.h"
#include "bmi270_test.h"

int main(void)
{
  /* 配置系统时钟。当前工程按板载 12MHz 外部晶振配置。 */
  system_clock_config();

  /* 初始化板级外设，例如延时函数和基础 GPIO。 */
  at32_board_init();

  /* 初始化调试串口，后续 printf 会从 USART1 输出。 */
  uart_print_init(115200);

  /* 初始化 BMI270 测试模块。 */
  bmi270_test_init();

  while(1)
  {
    /* 周期读取 BMI270，并以 roll,pitch,yaw 格式输出给 VOFA。 */
    bmi270_test_loop();
  }
}
