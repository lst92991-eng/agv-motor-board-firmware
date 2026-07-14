/**
  *****************************************************************************
  * @file     motor_driver_test.c
  * @brief    双路电机驱动开环转动测试
  *
  * 测试顺序：
  * 1. 上电后等待 3 秒，方便操作人员将车轮架空；
  * 2. 左电机正转 2 秒、停止 1 秒；
  * 3. 左电机反转 2 秒、停止 1 秒；
  * 4. 右电机正转 2 秒、停止 1 秒；
  * 5. 右电机反转 2 秒、停止 3 秒，然后重新循环。
  *
  * 测试使用 30% PWM，不读取编码器，也不进行闭环调速。
  *****************************************************************************
  */

#include "motor_driver_test.h"

#include <stdio.h>

#include "at32f45x_board.h"
#include "motor_driver.h"

#define MOTOR_DRIVER_TEST_SPEED          300
#define MOTOR_DRIVER_RUN_TIME_SEC        2U
#define MOTOR_DRIVER_STOP_TIME_SEC       1U
#define MOTOR_DRIVER_CYCLE_TIME_SEC      3U

/* 停止指定电机并等待，使每个测试动作在机械上清楚分开。 */
static void motor_driver_test_stop_and_wait(motor_id_t motor, uint16_t wait_seconds)
{
  motor_driver_stop(motor, MOTOR_STOP_COAST);
  delay_sec(wait_seconds);
}

void motor_driver_test_init(void)
{
  motor_driver_init();
  motor_driver_stop_all(MOTOR_STOP_COAST);

  /* 串口提示不会参与 VOFA 数据解析，本测试只用于观察电机动作。 */
  printf("motor test: keep wheels off ground, start after 3 seconds\r\n");
  delay_sec(3);
}

void motor_driver_test_loop(void)
{
  printf("left forward, pwm=30%%\r\n");
  motor_driver_set_speed(MOTOR_LEFT, MOTOR_DRIVER_TEST_SPEED);
  delay_sec(MOTOR_DRIVER_RUN_TIME_SEC);
  motor_driver_test_stop_and_wait(MOTOR_LEFT, MOTOR_DRIVER_STOP_TIME_SEC);

  printf("left reverse, pwm=30%%\r\n");
  motor_driver_set_speed(MOTOR_LEFT, -MOTOR_DRIVER_TEST_SPEED);
  delay_sec(MOTOR_DRIVER_RUN_TIME_SEC);
  motor_driver_test_stop_and_wait(MOTOR_LEFT, MOTOR_DRIVER_STOP_TIME_SEC);

  printf("right forward, pwm=30%%\r\n");
  motor_driver_set_speed(MOTOR_RIGHT, MOTOR_DRIVER_TEST_SPEED);
  delay_sec(MOTOR_DRIVER_RUN_TIME_SEC);
  motor_driver_test_stop_and_wait(MOTOR_RIGHT, MOTOR_DRIVER_STOP_TIME_SEC);

  printf("right reverse, pwm=30%%\r\n");
  motor_driver_set_speed(MOTOR_RIGHT, -MOTOR_DRIVER_TEST_SPEED);
  delay_sec(MOTOR_DRIVER_RUN_TIME_SEC);
  motor_driver_test_stop_and_wait(MOTOR_RIGHT, MOTOR_DRIVER_CYCLE_TIME_SEC);
}
