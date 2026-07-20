/**
  *****************************************************************************
  * @file     chassis_direction_test.c
  * @brief    小车四方向全功率开环测试
  *
  * 本测试不采集电流、不读取编码器，也不执行过流或堵转保护。
  * 测试阶段：1=前进，2=后退，3=原地左转，4=原地右转，5=完成。
  * 四个动作均使用100%占空比，每个动作运行2秒，整套测试只执行一次。
  * 串口输出格式：阶段,左PWM,右PWM。
  *****************************************************************************
  */

#include "chassis_direction_test.h"

#include <stdint.h>
#include <stdio.h>

#include "at32f45x_board.h"
#include "motor_driver.h"

#define CHASSIS_BRUTE_FORCE_PWM             1000
#define CHASSIS_BRUTE_FORCE_RUN_TIME_SEC       2U
#define CHASSIS_BRUTE_FORCE_STOP_TIME_SEC      2U

typedef enum
{
  CHASSIS_DIRECTION_FORWARD = 1,
  CHASSIS_DIRECTION_BACKWARD,
  CHASSIS_DIRECTION_TURN_LEFT,
  CHASSIS_DIRECTION_TURN_RIGHT,
  CHASSIS_DIRECTION_DONE
} chassis_direction_phase_t;

static chassis_direction_phase_t chassis_direction_phase;

static void chassis_direction_run(int16_t left_pwm, int16_t right_pwm)
{
  printf("%d,%d,%d\r\n", (int)chassis_direction_phase, left_pwm, right_pwm);
  motor_driver_set_speeds(left_pwm, right_pwm);
  delay_sec(CHASSIS_BRUTE_FORCE_RUN_TIME_SEC);
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
  delay_sec(CHASSIS_BRUTE_FORCE_STOP_TIME_SEC);
}

void chassis_direction_test_init(void)
{
  motor_driver_init();
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
  chassis_direction_phase = CHASSIS_DIRECTION_FORWARD;

  /* 上电后等待3秒，留出断电或清理测试区域的时间。 */
  delay_sec(3);
}

void chassis_direction_test_loop(void)
{
  switch(chassis_direction_phase)
  {
    case CHASSIS_DIRECTION_FORWARD:
      chassis_direction_run(CHASSIS_BRUTE_FORCE_PWM,
                            CHASSIS_BRUTE_FORCE_PWM);
      chassis_direction_phase = CHASSIS_DIRECTION_BACKWARD;
      break;

    case CHASSIS_DIRECTION_BACKWARD:
      chassis_direction_run(-CHASSIS_BRUTE_FORCE_PWM,
                            -CHASSIS_BRUTE_FORCE_PWM);
      chassis_direction_phase = CHASSIS_DIRECTION_TURN_LEFT;
      break;

    case CHASSIS_DIRECTION_TURN_LEFT:
      chassis_direction_run(-CHASSIS_BRUTE_FORCE_PWM,
                            CHASSIS_BRUTE_FORCE_PWM);
      chassis_direction_phase = CHASSIS_DIRECTION_TURN_RIGHT;
      break;

    case CHASSIS_DIRECTION_TURN_RIGHT:
      chassis_direction_run(CHASSIS_BRUTE_FORCE_PWM,
                            -CHASSIS_BRUTE_FORCE_PWM);
      chassis_direction_phase = CHASSIS_DIRECTION_DONE;
      break;

    case CHASSIS_DIRECTION_DONE:
    default:
      motor_driver_stop_all(MOTOR_STOP_BRAKE);
      printf("5,0,0\r\n");
      delay_ms(100);
      break;
  }
}
