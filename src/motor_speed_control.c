/**
  *****************************************************************************
  * @file     motor_speed_control.c
  * @brief    可复用的双路电机闭环速度控制
  *
  * 控制参数来自实际开环标定：
  * PWM 200/300/400/500/700/1000 对应约 17/27/37/47/67/97RPM，
  * 因此前馈模型使用 PWM = 30 + 10 * RPM。
  *****************************************************************************
  */

#include "motor_speed_control.h"

#include <stdint.h>

#include "at32f45x_board.h"
#include "motor_encoder.h"
#include "motor_pid.h"

#define MOTOR_SPEED_CONTROL_PERIOD_MS              20U
#define MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV       2640.0f
#define MOTOR_LEFT_SPEED_FEEDBACK_SIGN              -1.0f
#define MOTOR_RIGHT_SPEED_FEEDBACK_SIGN              1.0f

#define MOTOR_SPEED_PID_KP                            1.5f
#define MOTOR_SPEED_PID_KI                            2.0f
#define MOTOR_SPEED_PID_KD                            0.0f
#define MOTOR_SPEED_PID_CORRECTION_LIMIT            350.0f

#define MOTOR_SPEED_FEED_FORWARD_PER_RPM             10.0f
#define MOTOR_SPEED_FEED_FORWARD_STATIC_PWM         120.0f
#define MOTOR_SPEED_FILTER_ALPHA                       0.10f
#define MOTOR_SPEED_PWM_SLEW_PER_CYCLE                 8.0f
#define MOTOR_SPEED_RAW_RPM_LIMIT                     120.0f
#define MOTOR_SPEED_START_BOOST_PWM                   350.0f
#define MOTOR_SPEED_START_BOOST_TOTAL_CYCLES             30U
#define MOTOR_SPEED_START_BOOST_MIN_CYCLES               15U
#define MOTOR_SPEED_START_BOOST_READY_CYCLES              5U
#define MOTOR_SPEED_START_BOOST_EXIT_RPM                 5.0f

typedef struct
{
  motor_pid_t pid;
  float target_rpm;
  float measured_rpm;
  float filtered_rpm;
  float last_valid_raw_rpm;
  float pwm_command;
  int16_t pwm;
  int32_t last_count;
  uint8_t pid_active;
  uint8_t startup_boost_cycles;
  uint8_t startup_ready_cycles;
} motor_speed_axis_t;

static motor_speed_axis_t motor_speed_axes[2];

static float motor_speed_limit(float value, float value_min, float value_max)
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

static float motor_speed_count_delta_to_rpm(int32_t count_delta)
{
  return ((float)count_delta * 60000.0f) /
         (MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV * (float)MOTOR_SPEED_CONTROL_PERIOD_MS);
}

static float motor_speed_filter_raw(float raw_rpm, float *last_valid_rpm)
{
  if((raw_rpm > MOTOR_SPEED_RAW_RPM_LIMIT) ||
     (raw_rpm < -MOTOR_SPEED_RAW_RPM_LIMIT))
  {
    return *last_valid_rpm;
  }

  *last_valid_rpm = raw_rpm;
  return raw_rpm;
}

static void motor_speed_axis_reset_control(motor_speed_axis_t *axis)
{
  motor_pid_reset(&axis->pid);
  axis->pwm_command = 0.0f;
  axis->pwm = 0;
  axis->pid_active = 0U;
  axis->last_valid_raw_rpm = 0.0f;
  axis->startup_boost_cycles = 0U;
  axis->startup_ready_cycles = 0U;
}

static int16_t motor_speed_axis_update_command(motor_speed_axis_t *axis)
{
  float feed_forward;
  float correction;
  float command;
  float command_delta;

  if(axis->target_rpm == 0.0f)
  {
    motor_speed_axis_reset_control(axis);
    return 0;
  }

  /*
   * 重载启动时先用较高PWM克服静摩擦和万向轮摆正阻力。
   * 至少保持300ms，编码器同向达到5RPM并连续稳定100ms后退出，
   * 最长不超过600ms，避免单次毛刺过早结束助推。
   */
  if(axis->startup_boost_cycles != 0U)
  {
    uint8_t elapsed_cycles = (uint8_t)(
        MOTOR_SPEED_START_BOOST_TOTAL_CYCLES -
        axis->startup_boost_cycles);
    float measured_forward_rpm = (axis->target_rpm > 0.0f) ?
                                 axis->measured_rpm :
                                 -axis->measured_rpm;

    if((elapsed_cycles >= MOTOR_SPEED_START_BOOST_MIN_CYCLES) &&
       (measured_forward_rpm >= MOTOR_SPEED_START_BOOST_EXIT_RPM))
    {
      if(axis->startup_ready_cycles < MOTOR_SPEED_START_BOOST_READY_CYCLES)
      {
        axis->startup_ready_cycles++;
      }
    }
    else
    {
      axis->startup_ready_cycles = 0U;
    }

    if(axis->startup_ready_cycles >= MOTOR_SPEED_START_BOOST_READY_CYCLES)
    {
      axis->startup_boost_cycles = 0U;
    }
    else
    {
      axis->startup_boost_cycles--;
      axis->pwm_command = (axis->target_rpm > 0.0f) ?
                          MOTOR_SPEED_START_BOOST_PWM :
                          -MOTOR_SPEED_START_BOOST_PWM;
      return (int16_t)axis->pwm_command;
    }
  }

  if(axis->target_rpm > 0.0f)
  {
    feed_forward = MOTOR_SPEED_FEED_FORWARD_STATIC_PWM +
                   (axis->target_rpm * MOTOR_SPEED_FEED_FORWARD_PER_RPM);
  }
  else
  {
    feed_forward = -MOTOR_SPEED_FEED_FORWARD_STATIC_PWM +
                   (axis->target_rpm * MOTOR_SPEED_FEED_FORWARD_PER_RPM);
  }

  /* 启动斜坡达到前馈值之前冻结积分，防止启动阶段积分饱和。 */
  if(axis->pid_active == 0U)
  {
    correction = 0.0f;
  }
  else
  {
    correction = motor_pid_update(&axis->pid,
                                  axis->target_rpm,
                                  axis->measured_rpm,
                                  (float)MOTOR_SPEED_CONTROL_PERIOD_MS / 1000.0f);
    correction = motor_speed_limit(correction,
                                   -MOTOR_SPEED_PID_CORRECTION_LIMIT,
                                   MOTOR_SPEED_PID_CORRECTION_LIMIT);
  }

  command = feed_forward + correction;
  if(axis->target_rpm > 0.0f)
  {
    command = motor_speed_limit(command, 0.0f, 1000.0f);
  }
  else
  {
    command = motor_speed_limit(command, -1000.0f, 0.0f);
  }

  command_delta = command - axis->pwm_command;
  command_delta = motor_speed_limit(command_delta,
                                    -MOTOR_SPEED_PWM_SLEW_PER_CYCLE,
                                    MOTOR_SPEED_PWM_SLEW_PER_CYCLE);
  axis->pwm_command += command_delta;

  if((axis->pid_active == 0U) &&
     ((axis->pwm_command - feed_forward) < 0.5f) &&
     ((axis->pwm_command - feed_forward) > -0.5f))
  {
    motor_pid_reset(&axis->pid);
    axis->pid_active = 1U;
  }

  return (int16_t)axis->pwm_command;
}

static void motor_speed_axis_set_target(motor_speed_axis_t *axis, float target_rpm)
{
  uint8_t restart_control = 0U;

  target_rpm = motor_speed_limit(target_rpm, -100.0f, 100.0f);

  if((target_rpm == 0.0f) ||
     (axis->target_rpm == 0.0f) ||
     ((target_rpm > 0.0f) && (axis->target_rpm < 0.0f)) ||
     ((target_rpm < 0.0f) && (axis->target_rpm > 0.0f)))
  {
    restart_control = 1U;
  }

  axis->target_rpm = target_rpm;
  if(restart_control != 0U)
  {
    motor_speed_axis_reset_control(axis);
    if(target_rpm != 0.0f)
    {
      axis->startup_boost_cycles = MOTOR_SPEED_START_BOOST_TOTAL_CYCLES;
    }
  }
}

void motor_speed_control_init(void)
{
  uint8_t motor_index;

  motor_encoder_init();
  motor_driver_init();
  motor_driver_stop_all(MOTOR_STOP_COAST);
  motor_encoder_reset_all();

  for(motor_index = 0; motor_index < 2U; motor_index++)
  {
    motor_pid_init(&motor_speed_axes[motor_index].pid,
                   MOTOR_SPEED_PID_KP,
                   MOTOR_SPEED_PID_KI,
                   MOTOR_SPEED_PID_KD,
                   -MOTOR_SPEED_PID_CORRECTION_LIMIT,
                   MOTOR_SPEED_PID_CORRECTION_LIMIT);
    motor_speed_axes[motor_index].target_rpm = 0.0f;
    motor_speed_axes[motor_index].measured_rpm = 0.0f;
    motor_speed_axes[motor_index].filtered_rpm = 0.0f;
    motor_speed_axes[motor_index].last_count = motor_encoder_get_count((motor_id_t)motor_index);
    motor_speed_axis_reset_control(&motor_speed_axes[motor_index]);
  }
}

void motor_speed_control_set_targets(float left_target_rpm, float right_target_rpm)
{
  motor_speed_axis_set_target(&motor_speed_axes[MOTOR_LEFT], left_target_rpm);
  motor_speed_axis_set_target(&motor_speed_axes[MOTOR_RIGHT], right_target_rpm);
}

void motor_speed_control_update(void)
{
  delay_ms(MOTOR_SPEED_CONTROL_PERIOD_MS);

  motor_speed_control_update_now();
}

void motor_speed_control_update_now(void)
{
  uint8_t motor_index;

  for(motor_index = 0; motor_index < 2U; motor_index++)
  {
    motor_speed_axis_t *axis = &motor_speed_axes[motor_index];
    int32_t current_count = motor_encoder_get_count((motor_id_t)motor_index);
    int32_t count_delta = current_count - axis->last_count;
    float raw_rpm;
    float feedback_sign;

    axis->last_count = current_count;
    /* 左右编码器安装方向不同，实车前进时必须分别修正反馈符号。 */
    feedback_sign = (motor_index == MOTOR_LEFT) ?
                    MOTOR_LEFT_SPEED_FEEDBACK_SIGN :
                    MOTOR_RIGHT_SPEED_FEEDBACK_SIGN;
    raw_rpm = motor_speed_count_delta_to_rpm(count_delta) * feedback_sign;
    raw_rpm = motor_speed_filter_raw(raw_rpm, &axis->last_valid_raw_rpm);
    axis->filtered_rpm += MOTOR_SPEED_FILTER_ALPHA * (raw_rpm - axis->filtered_rpm);
    axis->measured_rpm = axis->filtered_rpm;
    axis->pwm = motor_speed_axis_update_command(axis);
  }

  motor_driver_set_speeds(motor_speed_axes[MOTOR_LEFT].pwm,
                          motor_speed_axes[MOTOR_RIGHT].pwm);
}

void motor_speed_control_resync_feedback(void)
{
  uint8_t motor_index;

  for(motor_index = 0U; motor_index < 2U; motor_index++)
  {
    motor_speed_axis_t *axis = &motor_speed_axes[motor_index];

    axis->last_count = motor_encoder_get_count((motor_id_t)motor_index);
    axis->measured_rpm = 0.0f;
    axis->filtered_rpm = 0.0f;
    axis->last_valid_raw_rpm = 0.0f;
  }
}

float motor_speed_control_get_measured_rpm(motor_id_t motor)
{
  if((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT))
  {
    return 0.0f;
  }

  return motor_speed_axes[motor].measured_rpm;
}

int16_t motor_speed_control_get_pwm(motor_id_t motor)
{
  if((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT))
  {
    return 0;
  }

  return motor_speed_axes[motor].pwm;
}

void motor_speed_control_hold_brake(void)
{
  motor_speed_control_set_targets(0.0f, 0.0f);
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
}
