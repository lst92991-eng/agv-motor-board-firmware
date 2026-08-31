/**
  *****************************************************************************
  * @file     motor_speed_control.c
  * @brief    双路电机速度反馈与受限PI控制
  *
  * 目标斜率由上层MotionReference统一限制。本层不再包含启动助推、满PWM
  * 脱困或路径状态；检测到反馈失效/堵转后立即让双轴停止并等待显式清故障。
  *****************************************************************************
  */

#include "motor_speed_control.h"

#include <stdint.h>

#include "at32f45x_board.h"
#include "motor_encoder.h"
#include "motor_pid.h"
#include "motor_speed_calibration.h"

typedef struct
{
  motor_pid_t pid;
  float requested_rpm;
  float target_rpm;
  float raw_rpm;
  float measured_rpm;
  float filtered_rpm;
  int32_t last_count;
  int16_t pwm;
  uint8_t stall_cycles;
  uint8_t feedback_fault;
  uint8_t zero_speed_cycles;
  int8_t last_drive_sign;
} motor_speed_axis_t;

static motor_speed_axis_t motor_speed_axes[2];

static float motor_speed_limit(float value, float minimum, float maximum)
{
  if(value < minimum)
  {
    return minimum;
  }
  return (value > maximum) ? maximum : value;
}

static float motor_speed_abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float motor_speed_count_delta_to_rpm(int32_t count_delta)
{
  return ((float)count_delta * 60000.0f) /
         (MOTOR_SPEED_ENCODER_COUNTS_PER_OUTPUT_REV *
          (float)MOTOR_SPEED_CONTROL_PERIOD_MS);
}

static float motor_speed_feed_forward(float target_rpm)
{
  if(target_rpm > 0.0f)
  {
    return MOTOR_SPEED_FEED_FORWARD_STATIC_PWM +
           target_rpm * MOTOR_SPEED_FEED_FORWARD_PER_RPM;
  }
  if(target_rpm < 0.0f)
  {
    return -MOTOR_SPEED_FEED_FORWARD_STATIC_PWM +
           target_rpm * MOTOR_SPEED_FEED_FORWARD_PER_RPM;
  }
  return 0.0f;
}

static void motor_speed_axis_reset(motor_speed_axis_t *axis)
{
  motor_pid_reset(&axis->pid);
  axis->pwm = 0;
  axis->stall_cycles = 0U;
}

static void motor_speed_axis_measure(motor_speed_axis_t *axis,
                                     motor_id_t motor)
{
  int32_t count;
  int32_t delta;
  float sign;
  float raw_rpm;

  count = motor_encoder_get_count(motor);
  delta = count - axis->last_count;
  axis->last_count = count;
  sign = (motor == MOTOR_LEFT) ? MOTOR_SPEED_LEFT_FEEDBACK_SIGN :
                                MOTOR_SPEED_RIGHT_FEEDBACK_SIGN;
  raw_rpm = motor_speed_count_delta_to_rpm(delta) * sign;
  if(motor_speed_abs(raw_rpm) > MOTOR_SPEED_RAW_RPM_LIMIT)
  {
    axis->feedback_fault = 1U;
    return;
  }
  axis->raw_rpm = raw_rpm;
  if(motor_speed_abs(raw_rpm) <= MOTOR_SPEED_REVERSE_ZERO_RPM)
  {
    if(axis->zero_speed_cycles < MOTOR_SPEED_REVERSE_ZERO_SAMPLES)
    {
      axis->zero_speed_cycles++;
    }
  }
  else
  {
    axis->zero_speed_cycles = 0U;
  }
  axis->filtered_rpm += MOTOR_SPEED_FILTER_ALPHA *
                        (raw_rpm - axis->filtered_rpm);
  axis->measured_rpm = axis->filtered_rpm;
}

static int16_t motor_speed_axis_command(motor_speed_axis_t *axis)
{
  float command;
  float correction;
  float forward_rpm;

  if(axis->feedback_fault != 0U)
  {
    return 0;
  }
  if(axis->target_rpm == 0.0f)
  {
    motor_speed_axis_reset(axis);
    return 0;
  }

  forward_rpm = (axis->target_rpm > 0.0f) ? axis->measured_rpm :
                                            -axis->measured_rpm;
  if((motor_speed_abs(axis->target_rpm) >= MOTOR_SPEED_STALL_TARGET_MIN_RPM) &&
     (forward_rpm <= MOTOR_SPEED_STALL_MEASURED_MAX_RPM))
  {
    if(axis->stall_cycles < MOTOR_SPEED_STALL_DETECT_CYCLES)
    {
      axis->stall_cycles++;
    }
    if(axis->stall_cycles >= MOTOR_SPEED_STALL_DETECT_CYCLES)
    {
      axis->feedback_fault = 1U;
      return 0;
    }
  }
  else
  {
    axis->stall_cycles = 0U;
  }

  correction = motor_pid_update(
      &axis->pid, axis->target_rpm, axis->measured_rpm,
      (float)MOTOR_SPEED_CONTROL_PERIOD_MS / 1000.0f);
  command = motor_speed_feed_forward(axis->target_rpm) + correction;
  if(axis->target_rpm > 0.0f)
  {
    command = motor_speed_limit(command, 0.0f, 1000.0f);
  }
  else
  {
    command = motor_speed_limit(command, -1000.0f, 0.0f);
  }
  return (int16_t)command;
}

static void motor_speed_axis_set_target(motor_speed_axis_t *axis,
                                        float target_rpm)
{
  target_rpm = motor_speed_limit(target_rpm,
                                 -MOTOR_SPEED_TARGET_LIMIT_RPM,
                                 MOTOR_SPEED_TARGET_LIMIT_RPM);
  if((target_rpm == 0.0f) ||
     ((target_rpm > 0.0f) && (axis->target_rpm < 0.0f)) ||
     ((target_rpm < 0.0f) && (axis->target_rpm > 0.0f)))
  {
    motor_speed_axis_reset(axis);
  }
  axis->target_rpm = target_rpm;
  if(target_rpm > 0.0f)
  {
    axis->last_drive_sign = 1;
  }
  else if(target_rpm < 0.0f)
  {
    axis->last_drive_sign = -1;
  }
}

static uint8_t motor_speed_axis_reverse_blocked(
    const motor_speed_axis_t *axis,
    float target_rpm)
{
  int8_t target_sign = (target_rpm > 0.0f) ? 1 :
                       ((target_rpm < 0.0f) ? -1 : 0);

  return ((target_sign != 0) &&
          (axis->last_drive_sign != 0) &&
          (target_sign != axis->last_drive_sign) &&
          (axis->zero_speed_cycles <
           MOTOR_SPEED_REVERSE_ZERO_SAMPLES)) ? 1U : 0U;
}

void motor_speed_control_init(void)
{
  uint8_t index;

  motor_encoder_init();
  motor_driver_init();
  motor_driver_stop_all(MOTOR_STOP_COAST);
  motor_encoder_reset_all();
  for(index = 0U; index < 2U; index++)
  {
    motor_speed_axis_t *axis = &motor_speed_axes[index];
    float kp = (index == MOTOR_LEFT) ? MOTOR_SPEED_LEFT_PID_KP :
                                      MOTOR_SPEED_RIGHT_PID_KP;
    float ki = (index == MOTOR_LEFT) ? MOTOR_SPEED_LEFT_PID_KI :
                                      MOTOR_SPEED_RIGHT_PID_KI;
    float kd = (index == MOTOR_LEFT) ? MOTOR_SPEED_LEFT_PID_KD :
                                      MOTOR_SPEED_RIGHT_PID_KD;

    motor_pid_init(&axis->pid, kp, ki, kd,
                   -MOTOR_SPEED_PID_CORRECTION_LIMIT,
                   MOTOR_SPEED_PID_CORRECTION_LIMIT);
    axis->requested_rpm = 0.0f;
    axis->target_rpm = 0.0f;
    axis->raw_rpm = 0.0f;
    axis->measured_rpm = 0.0f;
    axis->filtered_rpm = 0.0f;
    axis->last_count = motor_encoder_get_count((motor_id_t)index);
    axis->feedback_fault = 0U;
    axis->zero_speed_cycles = 0U;
    axis->last_drive_sign = 0;
    motor_speed_axis_reset(axis);
  }
}

void motor_speed_control_set_targets(float left_rpm, float right_rpm)
{
  motor_speed_axes[MOTOR_LEFT].requested_rpm = motor_speed_limit(
      left_rpm, -MOTOR_SPEED_TARGET_LIMIT_RPM, MOTOR_SPEED_TARGET_LIMIT_RPM);
  motor_speed_axes[MOTOR_RIGHT].requested_rpm = motor_speed_limit(
      right_rpm, -MOTOR_SPEED_TARGET_LIMIT_RPM, MOTOR_SPEED_TARGET_LIMIT_RPM);
}

void motor_speed_control_update(void)
{
  delay_ms(MOTOR_SPEED_CONTROL_PERIOD_MS);
  motor_speed_control_update_now();
}

void motor_speed_control_update_now(void)
{
  uint8_t index;

  for(index = 0U; index < 2U; index++)
  {
    motor_speed_axis_measure(&motor_speed_axes[index], (motor_id_t)index);
  }
  if((motor_speed_axis_reverse_blocked(
        &motor_speed_axes[MOTOR_LEFT],
        motor_speed_axes[MOTOR_LEFT].requested_rpm) != 0U) ||
     (motor_speed_axis_reverse_blocked(
        &motor_speed_axes[MOTOR_RIGHT],
        motor_speed_axes[MOTOR_RIGHT].requested_rpm) != 0U))
  {
    motor_speed_axis_set_target(&motor_speed_axes[MOTOR_LEFT], 0.0f);
    motor_speed_axis_set_target(&motor_speed_axes[MOTOR_RIGHT], 0.0f);
  }
  else
  {
    motor_speed_axis_set_target(
        &motor_speed_axes[MOTOR_LEFT],
        motor_speed_axes[MOTOR_LEFT].requested_rpm);
    motor_speed_axis_set_target(
        &motor_speed_axes[MOTOR_RIGHT],
        motor_speed_axes[MOTOR_RIGHT].requested_rpm);
  }
  for(index = 0U; index < 2U; index++)
  {
    motor_speed_axes[index].pwm =
        motor_speed_axis_command(&motor_speed_axes[index]);
  }
  if(motor_speed_control_has_fault() != 0U)
  {
    motor_speed_axes[MOTOR_LEFT].pwm = 0;
    motor_speed_axes[MOTOR_RIGHT].pwm = 0;
    motor_driver_stop_all(MOTOR_STOP_BRAKE);
    return;
  }
  motor_driver_set_speeds(motor_speed_axes[MOTOR_LEFT].pwm,
                          motor_speed_axes[MOTOR_RIGHT].pwm);
}

void motor_speed_control_resync_feedback(void)
{
  uint8_t index;

  for(index = 0U; index < 2U; index++)
  {
    motor_speed_axis_t *axis = &motor_speed_axes[index];
    axis->last_count = motor_encoder_get_count((motor_id_t)index);
    axis->raw_rpm = 0.0f;
    axis->zero_speed_cycles = 0U;
    axis->measured_rpm = 0.0f;
    axis->filtered_rpm = 0.0f;
    motor_speed_axis_reset(axis);
  }
}

float motor_speed_control_get_measured_rpm(motor_id_t motor)
{
  return ((motor == MOTOR_LEFT) || (motor == MOTOR_RIGHT)) ?
         motor_speed_axes[motor].measured_rpm : 0.0f;
}

int16_t motor_speed_control_get_pwm(motor_id_t motor)
{
  return ((motor == MOTOR_LEFT) || (motor == MOTOR_RIGHT)) ?
         motor_speed_axes[motor].pwm : 0;
}

uint8_t motor_speed_control_has_fault(void)
{
  return ((motor_speed_axes[MOTOR_LEFT].feedback_fault != 0U) ||
          (motor_speed_axes[MOTOR_RIGHT].feedback_fault != 0U)) ? 1U : 0U;
}

void motor_speed_control_clear_fault(void)
{
  motor_speed_axes[MOTOR_LEFT].feedback_fault = 0U;
  motor_speed_axes[MOTOR_RIGHT].feedback_fault = 0U;
  motor_speed_control_resync_feedback();
}

void motor_speed_control_hold_brake(void)
{
  motor_speed_control_set_targets(0.0f, 0.0f);
  motor_speed_axis_set_target(&motor_speed_axes[MOTOR_LEFT], 0.0f);
  motor_speed_axis_set_target(&motor_speed_axes[MOTOR_RIGHT], 0.0f);
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
}
