#include <stdint.h>
#include <stdio.h>

#include "motor_speed_control.h"

static int failures;
static int32_t encoder_count[2];
static int16_t applied_left_pwm;
static int16_t applied_right_pwm;
static uint32_t brake_count;

static void expect_true(int condition, const char *message)
{
  if(!condition)
  {
    printf("FAIL,%s\n", message);
    failures++;
  }
}

void delay_ms(uint16_t milliseconds)
{
  (void)milliseconds;
}

void motor_encoder_init(void)
{
}

int32_t motor_encoder_get_count(motor_id_t motor)
{
  return encoder_count[motor];
}

void motor_encoder_reset_all(void)
{
  encoder_count[MOTOR_LEFT] = 0;
  encoder_count[MOTOR_RIGHT] = 0;
}

uint32_t motor_encoder_get_invalid_transition_count(motor_id_t motor)
{
  (void)motor;
  return 0U;
}

void motor_driver_init(void)
{
}

void motor_driver_set_speed(motor_id_t motor, int16_t pwm)
{
  if(motor == MOTOR_LEFT)
  {
    applied_left_pwm = pwm;
  }
  else
  {
    applied_right_pwm = pwm;
  }
}

void motor_driver_set_speeds(int16_t left_pwm, int16_t right_pwm)
{
  applied_left_pwm = left_pwm;
  applied_right_pwm = right_pwm;
}

void motor_driver_stop(motor_id_t motor, motor_stop_mode_t mode)
{
  (void)mode;
  motor_driver_set_speed(motor, 0);
}

void motor_driver_stop_all(motor_stop_mode_t mode)
{
  applied_left_pwm = 0;
  applied_right_pwm = 0;
  if(mode == MOTOR_STOP_BRAKE)
  {
    brake_count++;
  }
}

static void reset_fixture(void)
{
  encoder_count[MOTOR_LEFT] = 0;
  encoder_count[MOTOR_RIGHT] = 0;
  applied_left_pwm = 0;
  applied_right_pwm = 0;
  brake_count = 0U;
  motor_speed_control_init();
}

static void advance_feedback(int32_t left_delta, int32_t right_delta)
{
  encoder_count[MOTOR_LEFT] += left_delta;
  encoder_count[MOTOR_RIGHT] += right_delta;
  motor_speed_control_update_now();
}

static void test_single_axis_invalid_feedback_brakes_both(void)
{
  reset_fixture();
  motor_speed_control_set_targets(20.0f, 20.0f);
  advance_feedback(200, -18);
  expect_true(motor_speed_control_has_fault() != 0U,
              "raw feedback overflow latches fault");
  expect_true((applied_left_pwm == 0) && (applied_right_pwm == 0) &&
              (brake_count != 0U),
              "one feedback fault brakes both axes in same tick");
}

static void test_wrong_direction_counts_as_no_progress(void)
{
  uint8_t index;

  reset_fixture();
  motor_speed_control_set_targets(10.0f, 10.0f);
  for(index = 0U; index < 10U; index++)
  {
    advance_feedback(-9, -9);
  }
  expect_true(motor_speed_control_has_fault() != 0U,
              "positive target with negative feedback latches fault");
  expect_true((applied_left_pwm == 0) && (applied_right_pwm == 0),
              "wrong direction fault never leaves one axis driven");
}

static void test_reverse_waits_for_measured_zero(void)
{
  uint8_t index;

  reset_fixture();
  motor_speed_control_set_targets(20.0f, 20.0f);
  advance_feedback(18, -18);
  motor_speed_control_set_targets(-20.0f, -20.0f);
  advance_feedback(18, -18);
  expect_true((applied_left_pwm == 0) && (applied_right_pwm == 0),
              "reverse command is held while measured speed is nonzero");

  for(index = 0U; index < 2U; index++)
  {
    advance_feedback(0, 0);
  }
  expect_true((applied_left_pwm == 0) && (applied_right_pwm == 0),
              "reverse waits for a continuous zero-speed window");
  advance_feedback(0, 0);
  expect_true((applied_left_pwm < 0) && (applied_right_pwm < 0),
              "reverse is allowed after both measured speeds settle");
}

static void test_correct_direction_does_not_false_trip(void)
{
  uint8_t index;

  reset_fixture();
  motor_speed_control_set_targets(10.0f, 10.0f);
  for(index = 0U; index < 12U; index++)
  {
    advance_feedback(9, -9);
  }
  expect_true(motor_speed_control_has_fault() == 0U,
              "correct direction feedback does not accumulate stall fault");
  expect_true((applied_left_pwm > 0) && (applied_right_pwm > 0),
              "healthy axes remain driven after the stall window");
}

static void test_fault_requires_explicit_clear(void)
{
  reset_fixture();
  motor_speed_control_set_targets(20.0f, 20.0f);
  advance_feedback(200, -18);
  motor_speed_control_hold_brake();
  motor_speed_control_resync_feedback();
  expect_true(motor_speed_control_has_fault() != 0U,
              "hold and resync do not clear latched feedback fault");
  motor_speed_control_clear_fault();
  expect_true(motor_speed_control_has_fault() == 0U,
              "explicit clear releases feedback fault");
}

int main(void)
{
  test_single_axis_invalid_feedback_brakes_both();
  test_wrong_direction_counts_as_no_progress();
  test_reverse_waits_for_measured_zero();
  test_correct_direction_does_not_false_trip();
  test_fault_requires_explicit_clear();
  if(failures != 0)
  {
    printf("MOTOR_SPEED_CONTROL_TEST_FAIL,%d\n", failures);
    return 1;
  }
  puts("MOTOR_SPEED_CONTROL_TEST_PASS");
  return 0;
}
