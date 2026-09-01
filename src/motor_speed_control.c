#include "motor_speed_control.h"

#include "motor_encoder.h"
#include "motor_speed_calibration.h"

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral;
  float previous_error;
  float requested_rpm;
  float target_rpm;
  float measured_rpm;
  float filtered_rpm;
  int32_t last_count;
  int16_t pwm;
  uint8_t stall_cycles;
  uint8_t fault;
  uint8_t zero_cycles;
  uint8_t derivative_valid;
  int8_t last_sign;
} motor_speed_axis_t;

static motor_speed_axis_t motor_axis[2];

static float motor_limit(float value, float limit)
{
  if(value > limit) return limit;
  if(value < -limit) return -limit;
  return value;
}

static float motor_abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float motor_feed_forward(float rpm)
{
  if(rpm > 0.0f) return MOTOR_SPEED_FEED_FORWARD_STATIC_PWM +
      rpm * MOTOR_SPEED_FEED_FORWARD_PER_RPM;
  if(rpm < 0.0f) return -MOTOR_SPEED_FEED_FORWARD_STATIC_PWM +
      rpm * MOTOR_SPEED_FEED_FORWARD_PER_RPM;
  return 0.0f;
}

static void motor_axis_reset(motor_speed_axis_t *axis)
{
  axis->integral = 0.0f;
  axis->previous_error = 0.0f;
  axis->pwm = 0;
  axis->stall_cycles = 0U;
  axis->derivative_valid = 0U;
}

static void motor_axis_measure(motor_speed_axis_t *axis, motor_id_t motor)
{
  int32_t count = motor_encoder_get_count(motor);
  int32_t delta = count - axis->last_count;
  float sign = (motor == MOTOR_LEFT) ? MOTOR_SPEED_LEFT_FEEDBACK_SIGN :
                                      MOTOR_SPEED_RIGHT_FEEDBACK_SIGN;
  float rpm = delta * 60000.0f /
      (MOTOR_SPEED_ENCODER_COUNTS_PER_OUTPUT_REV *
       MOTOR_SPEED_CONTROL_PERIOD_MS) * sign;

  axis->last_count = count;
  if(motor_abs(rpm) > MOTOR_SPEED_RAW_RPM_LIMIT)
  {
    axis->fault = 1U;
    return;
  }
  axis->zero_cycles = (motor_abs(rpm) <= MOTOR_SPEED_REVERSE_ZERO_RPM) ?
      (uint8_t)((axis->zero_cycles < MOTOR_SPEED_REVERSE_ZERO_SAMPLES) ?
                axis->zero_cycles + 1U : axis->zero_cycles) : 0U;
  axis->filtered_rpm += MOTOR_SPEED_FILTER_ALPHA *
      (rpm - axis->filtered_rpm);
  axis->measured_rpm = axis->filtered_rpm;
}

static void motor_axis_target(motor_speed_axis_t *axis, float rpm)
{
  rpm = motor_limit(rpm, MOTOR_SPEED_TARGET_LIMIT_RPM);
  if((rpm == 0.0f) || ((rpm * axis->target_rpm) < 0.0f))
    motor_axis_reset(axis);
  axis->target_rpm = rpm;
  if(rpm > 0.0f) axis->last_sign = 1;
  if(rpm < 0.0f) axis->last_sign = -1;
}

static uint8_t motor_reverse_blocked(const motor_speed_axis_t *axis)
{
  int8_t sign = (axis->requested_rpm > 0.0f) ? 1 :
      ((axis->requested_rpm < 0.0f) ? -1 : 0);
  return ((sign != 0) && (axis->last_sign != 0) &&
          (sign != axis->last_sign) &&
          (axis->zero_cycles < MOTOR_SPEED_REVERSE_ZERO_SAMPLES)) ? 1U : 0U;
}

static int16_t motor_axis_command(motor_speed_axis_t *axis)
{
  float forward;
  float error;
  float candidate_integral;
  float derivative = 0.0f;
  float correction;
  float output;

  if((axis->fault != 0U) || (axis->target_rpm == 0.0f))
  {
    if(axis->target_rpm == 0.0f) motor_axis_reset(axis);
    return 0;
  }
  forward = (axis->target_rpm > 0.0f) ? axis->measured_rpm :
                                        -axis->measured_rpm;
  if((motor_abs(axis->target_rpm) >= MOTOR_SPEED_STALL_TARGET_MIN_RPM) &&
     (forward <= MOTOR_SPEED_STALL_MEASURED_MAX_RPM))
  {
    if(axis->stall_cycles < MOTOR_SPEED_STALL_DETECT_CYCLES)
      axis->stall_cycles++;
    if(axis->stall_cycles >= MOTOR_SPEED_STALL_DETECT_CYCLES)
    {
      axis->fault = 1U;
      return 0;
    }
  }
  else
  {
    axis->stall_cycles = 0U;
  }
  error = axis->target_rpm - axis->measured_rpm;
  if(axis->derivative_valid != 0U)
    derivative = (error - axis->previous_error) * 1000.0f /
        MOTOR_SPEED_CONTROL_PERIOD_MS;
  axis->previous_error = error;
  axis->derivative_valid = 1U;
  candidate_integral = axis->integral +
      error * MOTOR_SPEED_CONTROL_PERIOD_MS / 1000.0f;
  candidate_integral = motor_limit(candidate_integral,
      MOTOR_SPEED_PID_CORRECTION_LIMIT / axis->ki);
  correction = axis->kp * error + axis->ki * candidate_integral +
      axis->kd * derivative;
  if(!(((correction > MOTOR_SPEED_PID_CORRECTION_LIMIT) && (error > 0.0f)) ||
       ((correction < -MOTOR_SPEED_PID_CORRECTION_LIMIT) && (error < 0.0f))))
    axis->integral = candidate_integral;
  correction = motor_limit(axis->kp * error + axis->ki * axis->integral +
                           axis->kd * derivative,
                           MOTOR_SPEED_PID_CORRECTION_LIMIT);
  output = motor_feed_forward(axis->target_rpm) + correction;
  if(axis->target_rpm > 0.0f)
  {
    if(output < 0.0f) output = 0.0f;
    if(output > 1000.0f) output = 1000.0f;
  }
  else
  {
    if(output > 0.0f) output = 0.0f;
    if(output < -1000.0f) output = -1000.0f;
  }
  return (int16_t)output;
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
    motor_speed_axis_t *axis = &motor_axis[index];
    axis->kp = (index == MOTOR_LEFT) ?
        MOTOR_SPEED_LEFT_PID_KP : MOTOR_SPEED_RIGHT_PID_KP;
    axis->ki = (index == MOTOR_LEFT) ?
        MOTOR_SPEED_LEFT_PID_KI : MOTOR_SPEED_RIGHT_PID_KI;
    axis->kd = (index == MOTOR_LEFT) ?
        MOTOR_SPEED_LEFT_PID_KD : MOTOR_SPEED_RIGHT_PID_KD;
    axis->requested_rpm = axis->target_rpm = 0.0f;
    axis->measured_rpm = axis->filtered_rpm = 0.0f;
    axis->last_count = motor_encoder_get_count((motor_id_t)index);
    axis->fault = axis->zero_cycles = 0U;
    axis->last_sign = 0;
    motor_axis_reset(axis);
  }
}

void motor_speed_control_set_targets(float left, float right)
{
  motor_axis[MOTOR_LEFT].requested_rpm = motor_limit(left,
      MOTOR_SPEED_TARGET_LIMIT_RPM);
  motor_axis[MOTOR_RIGHT].requested_rpm = motor_limit(right,
      MOTOR_SPEED_TARGET_LIMIT_RPM);
}

void motor_speed_control_update_now(void)
{
  uint8_t index;
  for(index = 0U; index < 2U; index++)
    motor_axis_measure(&motor_axis[index], (motor_id_t)index);
  if((motor_reverse_blocked(&motor_axis[MOTOR_LEFT]) != 0U) ||
     (motor_reverse_blocked(&motor_axis[MOTOR_RIGHT]) != 0U))
  {
    motor_axis_target(&motor_axis[MOTOR_LEFT], 0.0f);
    motor_axis_target(&motor_axis[MOTOR_RIGHT], 0.0f);
  }
  else
  {
    motor_axis_target(&motor_axis[MOTOR_LEFT],
                      motor_axis[MOTOR_LEFT].requested_rpm);
    motor_axis_target(&motor_axis[MOTOR_RIGHT],
                      motor_axis[MOTOR_RIGHT].requested_rpm);
  }
  for(index = 0U; index < 2U; index++)
    motor_axis[index].pwm = motor_axis_command(&motor_axis[index]);
  if(motor_speed_control_has_fault() != 0U)
  {
    motor_driver_stop_all(MOTOR_STOP_BRAKE);
    return;
  }
  motor_driver_set_speeds(motor_axis[MOTOR_LEFT].pwm,
                          motor_axis[MOTOR_RIGHT].pwm);
}

void motor_speed_control_resync_feedback(void)
{
  uint8_t index;
  for(index = 0U; index < 2U; index++)
  {
    motor_speed_axis_t *axis = &motor_axis[index];
    axis->last_count = motor_encoder_get_count((motor_id_t)index);
    axis->measured_rpm = axis->filtered_rpm = 0.0f;
    axis->zero_cycles = 0U;
    motor_axis_reset(axis);
  }
}

float motor_speed_control_get_measured_rpm(motor_id_t motor)
{
  return (motor <= MOTOR_RIGHT) ? motor_axis[motor].measured_rpm : 0.0f;
}

int16_t motor_speed_control_get_pwm(motor_id_t motor)
{
  return (motor <= MOTOR_RIGHT) ? motor_axis[motor].pwm : 0;
}

uint8_t motor_speed_control_has_fault(void)
{
  return (motor_axis[MOTOR_LEFT].fault | motor_axis[MOTOR_RIGHT].fault) != 0U;
}

void motor_speed_control_clear_fault(void)
{
  motor_axis[MOTOR_LEFT].fault = motor_axis[MOTOR_RIGHT].fault = 0U;
  motor_speed_control_resync_feedback();
}

void motor_speed_control_hold_brake(void)
{
  motor_speed_control_set_targets(0.0f, 0.0f);
  motor_axis_target(&motor_axis[MOTOR_LEFT], 0.0f);
  motor_axis_target(&motor_axis[MOTOR_RIGHT], 0.0f);
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
}
