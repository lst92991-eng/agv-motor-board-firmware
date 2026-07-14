#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral;
  float previous_error;
  float output_min;
  float output_max;
} motor_pid_t;

/* 初始化 PID 参数、积分项和上一次误差。 */
void motor_pid_init(motor_pid_t *pid,
                    float kp,
                    float ki,
                    float kd,
                    float output_min,
                    float output_max);

/* 清除积分项和微分环节的历史误差。 */
void motor_pid_reset(motor_pid_t *pid);

/* 根据目标值、反馈值和采样周期计算 PID 输出。 */
float motor_pid_update(motor_pid_t *pid, float target, float measured, float dt_seconds);

#ifdef __cplusplus
}
#endif

#endif
