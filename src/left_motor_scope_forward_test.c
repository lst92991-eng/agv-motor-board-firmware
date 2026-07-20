/**
  *****************************************************************************
  * @file     left_motor_scope_forward_test.c
  * @brief    左电机持续开环正转示波器测试
  *
  * 左电机固定使用350千分比PWM正转，右电机保持主动刹车。该测试不启动
  * FreeRTOS和PID，不计算或输出测试数据。烧录前必须把左主动轮架空。
  *****************************************************************************
  */

#include "left_motor_scope_forward_test.h"

#include "at32f45x_board.h"
#include "at32f45x_crm.h"
#include "at32f45x_gpio.h"
#include "motor_driver.h"

#define LEFT_MOTOR_SCOPE_FORWARD_PWM  350

static void left_motor_scope_encoder_input_init(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_pins = GPIO_PINS_4 | GPIO_PINS_5;
  gpio_init(GPIOC, &gpio_init_struct);
}

void left_motor_scope_forward_test_init(void)
{
  /* 只配置浮空输入，不启用编码器中断或执行正交解码。 */
  left_motor_scope_encoder_input_init();

  /* 右电机保持刹车，左电机只设置一次固定正向PWM。 */
  motor_driver_init();
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
  motor_driver_set_speed(MOTOR_LEFT, LEFT_MOTOR_SCOPE_FORWARD_PWM);
}

void left_motor_scope_forward_test_loop(void)
{
  /* 不执行控制、计算或串口输出，仅保持左电机持续正转。 */
  delay_ms(1000U);
}
