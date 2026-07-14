/**
  *****************************************************************************
  * @file     motor_pid.c
  * @brief    电机速度 PID 控制器
  *****************************************************************************
  */

#include "motor_pid.h"

static float motor_pid_limit(float value, float value_min, float value_max)
{
  if(value < value_min)
  {
    return value_min;
  }
  if(value > value_max)
  {
    return value_max;
  }

  return value;
}

void motor_pid_init(motor_pid_t *pid,
                    float kp,
                    float ki,
                    float kd,
                    float output_min,
                    float output_max)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->output_min = output_min;
  pid->output_max = output_max;
  motor_pid_reset(pid);
}

void motor_pid_reset(motor_pid_t *pid)
{
  pid->integral = 0.0f;
  pid->previous_error = 0.0f;
}

float motor_pid_update(motor_pid_t *pid, float target, float measured, float dt_seconds)
{
  float error;
  float derivative;
  float output;

  if(dt_seconds <= 0.0f)
  {
    return 0.0f;
  }

  error = target - measured;
  pid->integral += error * dt_seconds;

  /* 先限制积分项，避免输出饱和后积分继续累积。 */
  if(pid->ki != 0.0f)
  {
    pid->integral = motor_pid_limit(pid->integral,
                                    pid->output_min / pid->ki,
                                    pid->output_max / pid->ki);
  }

  derivative = (error - pid->previous_error) / dt_seconds;
  output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
  pid->previous_error = error;

  return motor_pid_limit(output, pid->output_min, pid->output_max);
}
