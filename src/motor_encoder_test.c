/**
  *****************************************************************************
  * @file     motor_encoder_test.c
  * @brief    左右电机编码器单项测试
  *
  * 测试时不初始化电机驱动，电机不会主动旋转。
  * 用手分别转动左右车轮，串口每 100ms 输出一次累计计数：
  * 左编码器累计计数,右编码器累计计数
  *
  * 正转和反转时计数应向相反方向变化；停止转动后计数应保持稳定。
  *****************************************************************************
  */

#include "motor_encoder_test.h"

#include <stdio.h>

#include "at32f45x_board.h"
#include "motor_encoder.h"

#define MOTOR_ENCODER_TEST_PERIOD_MS     100U

void motor_encoder_test_init(void)
{
  /* 初始化四路 AB 相输入和外部中断，然后从零开始累计。 */
  motor_encoder_init();
  motor_encoder_reset_all();
}

void motor_encoder_test_loop(void)
{
  int32_t left_count;
  int32_t right_count;

  left_count = motor_encoder_get_count(MOTOR_LEFT);
  right_count = motor_encoder_get_count(MOTOR_RIGHT);

  /* 只输出两路数值，既方便串口观察，也可以直接送入 VOFA。 */
  printf("%ld,%ld\r\n", (long)left_count, (long)right_count);

  delay_ms(MOTOR_ENCODER_TEST_PERIOD_MS);
}
