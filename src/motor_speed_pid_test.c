/**
  *****************************************************************************
  * @file     motor_speed_pid_test.c
  * @brief    兼容旧速度环测试入口
  *
  * 历史版本维护了一套独立的 100ms PID 和前馈参数，容易与正式 20ms
  * 底盘控制漂移。保留 API，但统一转发到 motor_speed_tuning_test。
  *****************************************************************************
  */

#include "motor_speed_pid_test.h"

#include "motor_speed_tuning_test.h"

void motor_speed_pid_test_init(void)
{
  motor_speed_tuning_test_init();
}

void motor_speed_pid_test_loop(void)
{
  motor_speed_tuning_test_loop();
}
