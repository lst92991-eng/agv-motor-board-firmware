/**
  *****************************************************************************
  * @file     motor_speed_test.c
  * @brief    电机转动与编码器速度联合测试
  *
  * 已知电机参数：
  * - 型号：37R-520-2460；
  * - 额定电压：24V；
  * - 减速比：60:1；
  * - 输出轴空载转速：约 100RPM；
  * - 输出轴额定转速：约 75RPM。
  *
  * 开环标定显示 1000 PWM 时约为 97RPM，与铭牌空载 100RPM 一致，
  * 因此按 11 脉冲/电机轴/相、AB 四倍频和 60:1 减速比计算为 2640 计数/圈。
  * 后续若手动转动输出轴一整圈得到的实测计数不同，只需修改下面的宏。
  *
  * 测试过程：50% PWM 正转 8 秒、停止 2 秒、反转 8 秒、停止 3 秒，然后循环。
  * 串口每 200ms 输出：left_rpm,right_rpm，可直接使用 VOFA 观察波形。
  *****************************************************************************
  */

#include "motor_speed_test.h"

#include <stdint.h>
#include <stdio.h>

#include "at32f45x_board.h"
#include "motor_driver.h"
#include "motor_encoder.h"

#define MOTOR_GEAR_RATIO                         60.0f
#define MOTOR_ENCODER_PULSES_PER_MOTOR_REV       11.0f
#define MOTOR_ENCODER_QUADRATURE_MULTIPLIER       4.0f
#define MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV      \
  (MOTOR_GEAR_RATIO * MOTOR_ENCODER_PULSES_PER_MOTOR_REV * MOTOR_ENCODER_QUADRATURE_MULTIPLIER)

#define MOTOR_SPEED_TEST_PWM_PERMILLE             500
#define MOTOR_SPEED_SAMPLE_PERIOD_MS              200U
#define MOTOR_SPEED_FORWARD_TIME_MS               8000U
#define MOTOR_SPEED_REVERSE_TIME_MS               8000U
#define MOTOR_SPEED_STOP_TIME_MS                  2000U
#define MOTOR_SPEED_CYCLE_STOP_TIME_MS            3000U

typedef enum
{
  MOTOR_SPEED_STATE_FORWARD = 0,
  MOTOR_SPEED_STATE_STOP_BEFORE_REVERSE,
  MOTOR_SPEED_STATE_REVERSE,
  MOTOR_SPEED_STATE_STOP_BEFORE_FORWARD
} motor_speed_test_state_t;

static motor_speed_test_state_t motor_speed_test_state;
static uint32_t motor_speed_state_elapsed_ms;
static int32_t last_left_count;
static int32_t last_right_count;

/* 根据当前测试阶段设置电机动作，并重新开始该阶段的计时。 */
static void motor_speed_test_enter_state(motor_speed_test_state_t state)
{
  motor_speed_test_state = state;
  motor_speed_state_elapsed_ms = 0;

  switch(state)
  {
    case MOTOR_SPEED_STATE_FORWARD:
      motor_driver_set_speeds(MOTOR_SPEED_TEST_PWM_PERMILLE, MOTOR_SPEED_TEST_PWM_PERMILLE);
      break;

    case MOTOR_SPEED_STATE_REVERSE:
      motor_driver_set_speeds(-MOTOR_SPEED_TEST_PWM_PERMILLE, -MOTOR_SPEED_TEST_PWM_PERMILLE);
      break;

    case MOTOR_SPEED_STATE_STOP_BEFORE_REVERSE:
    case MOTOR_SPEED_STATE_STOP_BEFORE_FORWARD:
    default:
      motor_driver_stop_all(MOTOR_STOP_COAST);
      break;
  }
}

/* 根据本次计数增量和采样周期计算输出轴每分钟转数。 */
static float motor_speed_count_delta_to_rpm(int32_t count_delta)
{
  return ((float)count_delta * 60000.0f) /
         (MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV * (float)MOTOR_SPEED_SAMPLE_PERIOD_MS);
}

/* 当前阶段达到设定时间后，切换到下一个动作。 */
static void motor_speed_test_update_state(void)
{
  switch(motor_speed_test_state)
  {
    case MOTOR_SPEED_STATE_FORWARD:
      if(motor_speed_state_elapsed_ms >= MOTOR_SPEED_FORWARD_TIME_MS)
      {
        motor_speed_test_enter_state(MOTOR_SPEED_STATE_STOP_BEFORE_REVERSE);
      }
      break;

    case MOTOR_SPEED_STATE_STOP_BEFORE_REVERSE:
      if(motor_speed_state_elapsed_ms >= MOTOR_SPEED_STOP_TIME_MS)
      {
        motor_speed_test_enter_state(MOTOR_SPEED_STATE_REVERSE);
      }
      break;

    case MOTOR_SPEED_STATE_REVERSE:
      if(motor_speed_state_elapsed_ms >= MOTOR_SPEED_REVERSE_TIME_MS)
      {
        motor_speed_test_enter_state(MOTOR_SPEED_STATE_STOP_BEFORE_FORWARD);
      }
      break;

    case MOTOR_SPEED_STATE_STOP_BEFORE_FORWARD:
    default:
      if(motor_speed_state_elapsed_ms >= MOTOR_SPEED_CYCLE_STOP_TIME_MS)
      {
        motor_speed_test_enter_state(MOTOR_SPEED_STATE_FORWARD);
      }
      break;
  }
}

void motor_speed_test_init(void)
{
  /* 先初始化编码器，再初始化电机 PWM；两组引脚互不冲突。 */
  motor_encoder_init();
  motor_driver_init();
  motor_driver_stop_all(MOTOR_STOP_COAST);

  /* 留出 3 秒安全时间，测试前必须保证车轮已经架空。 */
  delay_sec(3);

  motor_encoder_reset_all();
  last_left_count = motor_encoder_get_count(MOTOR_LEFT);
  last_right_count = motor_encoder_get_count(MOTOR_RIGHT);
  motor_speed_test_enter_state(MOTOR_SPEED_STATE_FORWARD);
}

void motor_speed_test_loop(void)
{
  int32_t current_left_count;
  int32_t current_right_count;
  int32_t left_count_delta;
  int32_t right_count_delta;
  float left_rpm;
  float right_rpm;

  delay_ms(MOTOR_SPEED_SAMPLE_PERIOD_MS);

  current_left_count = motor_encoder_get_count(MOTOR_LEFT);
  current_right_count = motor_encoder_get_count(MOTOR_RIGHT);
  left_count_delta = current_left_count - last_left_count;
  right_count_delta = current_right_count - last_right_count;
  last_left_count = current_left_count;
  last_right_count = current_right_count;

  left_rpm = motor_speed_count_delta_to_rpm(left_count_delta);
  right_rpm = motor_speed_count_delta_to_rpm(right_count_delta);

  /* 一个 printf 同时输出左右输出轴速度，单位均为 RPM。 */
  printf("%.2f,%.2f\r\n", left_rpm, right_rpm);

  motor_speed_state_elapsed_ms += MOTOR_SPEED_SAMPLE_PERIOD_MS;
  motor_speed_test_update_state();
}
