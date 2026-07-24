/**
  *****************************************************************************
  * @file     right_motor_scope_forward_test.c
  * @brief    右电机持续开环正转测试
  *
  * 右电机固定使用350千分比PWM正转，左电机保持主动刹车。该测试不启动
  * FreeRTOS、PID、编码器、ADC或传感器，也不计算和输出测试数据。
  *****************************************************************************
  */

#include "right_motor_scope_forward_test.h"

#include "at32f45x_board.h"
#include "motor_driver.h"

#define RIGHT_MOTOR_SCOPE_FORWARD_PWM  350

void right_motor_scope_forward_test_init(void)
{
  /* 只初始化电机PWM，左电机保持刹车，右电机设置一次固定正向PWM。 */
  motor_driver_init();
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
  motor_driver_set_speed(MOTOR_RIGHT, RIGHT_MOTOR_SCOPE_FORWARD_PWM);
}

void right_motor_scope_forward_test_loop(void)
{
  /* 不执行控制、采样、计算或串口输出，仅保持右电机持续正转。 */
  delay_ms(1000U);
}
