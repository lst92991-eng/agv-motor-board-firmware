/**
 * @file drive_control.c
 * @brief 20ms车体速度到双AT8236 PWM的唯一闭环执行器。
 */
#include "drive_control.h"

#include "board_config.h"
#include "motor_encoder.h"

typedef struct
{
  float target_rpm;
  float requested_rpm;
  float measured_rpm;
  float filtered_rpm;
  float integral;
  float previous_error;
  int32_t last_count;
  int16_t pwm;
  uint8_t stall_ticks;
  uint8_t zero_ticks;
  uint8_t fault;
  int8_t last_sign;
} drive_axis_t;

static drive_axis_t drive_axis[2];
static float drive_left_ramp;
static float drive_right_ramp;

static float drive_abs(float value) { return (value < 0.0f) ? -value : value; }
static float drive_limit(float value, float limit)
{
  if(value > limit) return limit;
  if(value < -limit) return -limit;
  return value;
}

static float drive_step(float current, float target, float up, float down)
{
  float delta = target - current;
  float limit = (((current * target) >= 0.0f) &&
                 (drive_abs(target) > drive_abs(current))) ? up : down;
  if(delta > limit) delta = limit;
  if(delta < -limit) delta = -limit;
  return current + delta;
}

static uint8_t drive_reverse_blocked(const drive_axis_t *axis)
{
  int8_t requested_sign = (axis->requested_rpm > 0.0f) ? 1 :
      ((axis->requested_rpm < 0.0f) ? -1 : 0);
  return ((requested_sign != 0) && (axis->last_sign != 0) &&
          (requested_sign != axis->last_sign) && (axis->zero_ticks < 3U)) ? 1U : 0U;
}

static void drive_measure(drive_axis_t *axis, motor_id_t motor)
{
  int32_t count = motor_encoder_get_count(motor);
  int32_t delta = count - axis->last_count;
  float sign = (motor == MOTOR_LEFT) ? BOARD_LEFT_FEEDBACK_SIGN :
                                       BOARD_RIGHT_FEEDBACK_SIGN;
  float rpm = (float)delta * 60000.0f /
      (BOARD_ENCODER_COUNTS_PER_REV * BOARD_SPEED_PERIOD_MS) * sign;

  axis->last_count = count;
  if(drive_abs(rpm) > BOARD_RAW_RPM_LIMIT)
  {
    axis->fault = 1U;
    return;
  }
  axis->filtered_rpm += BOARD_RPM_FILTER_ALPHA * (rpm - axis->filtered_rpm);
  axis->measured_rpm = axis->filtered_rpm;
  if(drive_abs(rpm) <= 1.0f)
  {
    if(axis->zero_ticks < 3U) axis->zero_ticks++;
  }
  else axis->zero_ticks = 0U;
}

static int16_t drive_pid(drive_axis_t *axis)
{
  float error;
  float derivative;
  float candidate;
  float correction;
  float feed_forward;
  float output;
  float forward;

  if((axis->fault != 0U) || (axis->target_rpm == 0.0f))
  {
    axis->integral = 0.0f;
    axis->previous_error = 0.0f;
    return 0;
  }
  forward = (axis->target_rpm > 0.0f) ? axis->measured_rpm : -axis->measured_rpm;
  if((drive_abs(axis->target_rpm) >= BOARD_STALL_TARGET_RPM) &&
     (forward <= BOARD_STALL_MEASURED_RPM))
  {
    if(++axis->stall_ticks >= BOARD_STALL_TICKS)
    {
      axis->fault = 1U;
      return 0;
    }
  }
  else axis->stall_ticks = 0U;

  error = axis->target_rpm - axis->measured_rpm;
  derivative = (error - axis->previous_error) * 1000.0f / BOARD_SPEED_PERIOD_MS;
  candidate = drive_limit(axis->integral + error * BOARD_SPEED_PERIOD_MS / 1000.0f,
                          BOARD_PID_LIMIT / BOARD_PID_KI);
  correction = BOARD_PID_KP * error + BOARD_PID_KI * candidate +
      BOARD_PID_KD * derivative;
  if(!(((correction > BOARD_PID_LIMIT) && (error > 0.0f)) ||
       ((correction < -BOARD_PID_LIMIT) && (error < 0.0f))))
    axis->integral = candidate;
  axis->previous_error = error;
  feed_forward = (axis->target_rpm > 0.0f) ? BOARD_FF_STATIC_PWM : -BOARD_FF_STATIC_PWM;
  output = feed_forward + axis->target_rpm * BOARD_FF_PER_RPM +
      drive_limit(BOARD_PID_KP * error + BOARD_PID_KI * axis->integral +
                  BOARD_PID_KD * derivative, BOARD_PID_LIMIT);
  if(axis->target_rpm > 0.0f)
  {
    if(output < 0.0f) output = 0.0f;
    return (int16_t)drive_limit(output, 1000.0f);
  }
  if(output > 0.0f) output = 0.0f;
  return (int16_t)drive_limit(output, 1000.0f);
}

static uint8_t drive_reference_ok(const drive_reference_t *reference)
{
  if(reference == 0) return 0U;
  if((drive_abs((float)reference->linear_mm_s) > BOARD_MAX_LINEAR_MM_S) ||
     (drive_abs((float)reference->angular_mrad_s) > BOARD_MAX_ANGULAR_MRAD_S)) return 0U;
  return ((reference->accel_mm_s2 >= 10U) && (reference->accel_mm_s2 <= 500U) &&
          (reference->decel_mm_s2 >= 10U) && (reference->decel_mm_s2 <= 1000U)) ? 1U : 0U;
}

void drive_control_init(void)
{
  uint8_t index;
  motor_encoder_init();
  motor_driver_init();
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
  motor_encoder_reset_all();
  for(index = 0U; index < 2U; index++)
  {
    drive_axis[index].last_count = motor_encoder_get_count((motor_id_t)index);
    drive_axis[index].filtered_rpm = 0.0f;
    drive_axis[index].fault = 0U;
  }
  drive_left_ramp = drive_right_ramp = 0.0f;
}

void drive_control_stop(void)
{
  uint8_t index;
  for(index = 0U; index < 2U; index++)
  {
    drive_axis[index].target_rpm = 0.0f;
    drive_axis[index].requested_rpm = 0.0f;
    drive_axis[index].integral = 0.0f;
    drive_axis[index].pwm = 0;
  }
  drive_left_ramp = drive_right_ramp = 0.0f;
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
}

void drive_control_clear_fault(void)
{
  drive_axis[MOTOR_LEFT].fault = drive_axis[MOTOR_RIGHT].fault = 0U;
  drive_axis[MOTOR_LEFT].stall_ticks = drive_axis[MOTOR_RIGHT].stall_ticks = 0U;
  drive_control_stop();
  drive_axis[MOTOR_LEFT].last_count = motor_encoder_get_count(MOTOR_LEFT);
  drive_axis[MOTOR_RIGHT].last_count = motor_encoder_get_count(MOTOR_RIGHT);
}

uint8_t drive_control_update(const drive_reference_t *reference,
                             float yaw_rate_mrad_s, float dt_s)
{
  float angular;
  float turn;
  float left;
  float right;
  float scale;
  float acceleration;
  float deceleration;

  if((drive_reference_ok(reference) == 0U) || (dt_s <= 0.0f) || (dt_s > 0.1f))
  {
    drive_axis[MOTOR_LEFT].fault = 1U;
    drive_axis[MOTOR_RIGHT].fault = 1U;
  }
  else
  {
    angular = (float)reference->angular_mrad_s + BOARD_YAW_RATE_GAIN *
        ((float)reference->angular_mrad_s - yaw_rate_mrad_s);
    angular = drive_limit(angular, BOARD_MAX_ANGULAR_MRAD_S);
    turn = BOARD_TRACK_WIDTH_MM * angular / 2000.0f;
    left = ((float)reference->linear_mm_s - turn) * 60.0f / BOARD_WHEEL_CIRCUMFERENCE_MM;
    right = ((float)reference->linear_mm_s + turn) * 60.0f / BOARD_WHEEL_CIRCUMFERENCE_MM;
    scale = drive_abs(left);
    if(drive_abs(right) > scale) scale = drive_abs(right);
    if(scale > BOARD_MAX_WHEEL_RPM)
    {
      left *= BOARD_MAX_WHEEL_RPM / scale;
      right *= BOARD_MAX_WHEEL_RPM / scale;
    }
    if(((left * drive_left_ramp) < 0.0f) || ((right * drive_right_ramp) < 0.0f))
      left = right = 0.0f;
    acceleration = reference->accel_mm_s2 * dt_s * 60.0f / BOARD_WHEEL_CIRCUMFERENCE_MM;
    deceleration = reference->decel_mm_s2 * dt_s * 60.0f / BOARD_WHEEL_CIRCUMFERENCE_MM;
    drive_left_ramp = drive_step(drive_left_ramp, left, acceleration, deceleration);
    drive_right_ramp = drive_step(drive_right_ramp, right, acceleration, deceleration);
    drive_axis[MOTOR_LEFT].requested_rpm = drive_left_ramp * BOARD_LEFT_TARGET_SIGN;
    drive_axis[MOTOR_RIGHT].requested_rpm = drive_right_ramp * BOARD_RIGHT_TARGET_SIGN;
  }

  drive_measure(&drive_axis[MOTOR_LEFT], MOTOR_LEFT);
  drive_measure(&drive_axis[MOTOR_RIGHT], MOTOR_RIGHT);
  if((drive_reverse_blocked(&drive_axis[MOTOR_LEFT]) != 0U) ||
     (drive_reverse_blocked(&drive_axis[MOTOR_RIGHT]) != 0U))
  {
    drive_axis[MOTOR_LEFT].target_rpm = 0.0f;
    drive_axis[MOTOR_RIGHT].target_rpm = 0.0f;
  }
  else
  {
    drive_axis[MOTOR_LEFT].target_rpm = drive_limit(drive_axis[MOTOR_LEFT].requested_rpm,
                                                      BOARD_MAX_WHEEL_RPM);
    drive_axis[MOTOR_RIGHT].target_rpm = drive_limit(drive_axis[MOTOR_RIGHT].requested_rpm,
                                                       BOARD_MAX_WHEEL_RPM);
  }
  if(drive_axis[MOTOR_LEFT].target_rpm > 0.0f) drive_axis[MOTOR_LEFT].last_sign = 1;
  if(drive_axis[MOTOR_LEFT].target_rpm < 0.0f) drive_axis[MOTOR_LEFT].last_sign = -1;
  if(drive_axis[MOTOR_RIGHT].target_rpm > 0.0f) drive_axis[MOTOR_RIGHT].last_sign = 1;
  if(drive_axis[MOTOR_RIGHT].target_rpm < 0.0f) drive_axis[MOTOR_RIGHT].last_sign = -1;
  drive_axis[MOTOR_LEFT].pwm = drive_pid(&drive_axis[MOTOR_LEFT]);
  drive_axis[MOTOR_RIGHT].pwm = drive_pid(&drive_axis[MOTOR_RIGHT]);
  if(drive_control_has_fault() != 0U)
  {
    motor_driver_stop_all(MOTOR_STOP_BRAKE);
    return 0U;
  }
  motor_driver_set_speeds(drive_axis[MOTOR_LEFT].pwm, drive_axis[MOTOR_RIGHT].pwm);
  return 1U;
}

uint8_t drive_control_has_fault(void)
{
  return (drive_axis[MOTOR_LEFT].fault | drive_axis[MOTOR_RIGHT].fault) != 0U;
}

float drive_control_measured_rpm(motor_id_t motor)
{
  return (motor <= MOTOR_RIGHT) ? drive_axis[motor].measured_rpm : 0.0f;
}

int16_t drive_control_pwm(motor_id_t motor)
{
  return (motor <= MOTOR_RIGHT) ? drive_axis[motor].pwm : 0;
}
