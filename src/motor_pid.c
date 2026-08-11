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
  float candidate_integral;
  float candidate_output;
  float output;

  if(dt_seconds <= 0.0f)
  {
    return 0.0f;
  }

  error = target - measured;
  candidate_integral = pid->integral + (error * dt_seconds);

  /* 先限制候选积分项，避免输出饱和后积分继续累积。 */
  if(pid->ki != 0.0f)
  {
    candidate_integral = motor_pid_limit(candidate_integral,
                                         pid->output_min / pid->ki,
                                         pid->output_max / pid->ki);
  }

  derivative = (error - pid->previous_error) / dt_seconds;
  candidate_output = (pid->kp * error) +
                     (pid->ki * candidate_integral) +
                     (pid->kd * derivative);

  /*
   * 只有控制量仍可向误差方向移动时才接收新积分。
   * 这能避免启动助推、堵转或目标超出能力时的积分风up。
   */
  if(!(((candidate_output > pid->output_max) && (error > 0.0f)) ||
       ((candidate_output < pid->output_min) && (error < 0.0f))))
  {
    pid->integral = candidate_integral;
  }

  output = (pid->kp * error) +
           (pid->ki * pid->integral) +
           (pid->kd * derivative);
  pid->previous_error = error;

  return motor_pid_limit(output, pid->output_min, pid->output_max);
}
