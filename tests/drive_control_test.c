#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "drive_control.h"
#include "motor_encoder.h"

static int32_t encoder_count[2];
static int16_t applied_left;
static int16_t applied_right;
static uint32_t brake_count;

void motor_encoder_init(void) {}
int32_t motor_encoder_get_count(motor_id_t motor) { return encoder_count[motor]; }
void motor_encoder_reset_all(void) { encoder_count[0] = encoder_count[1] = 0; }
uint32_t motor_encoder_get_invalid_transition_count(motor_id_t motor) { (void)motor; return 0U; }
void motor_driver_init(void) {}
void motor_driver_set_speed(motor_id_t motor, int16_t pwm)
{ if(motor == MOTOR_LEFT) applied_left = pwm; else applied_right = pwm; }
void motor_driver_set_speeds(int16_t left, int16_t right) { applied_left = left; applied_right = right; }
void motor_driver_stop(motor_id_t motor, motor_stop_mode_t mode)
{ (void)motor; (void)mode; }
void motor_driver_stop_all(motor_stop_mode_t mode)
{ applied_left = applied_right = 0; if(mode == MOTOR_STOP_BRAKE) brake_count++; }

static drive_reference_t reference(void)
{
  drive_reference_t value = {100, 0, 500U, 1000U};
  return value;
}

static void expect_true(uint8_t value, const char *text)
{
  if(value == 0U) { fprintf(stderr, "%s\n", text); exit(1); }
}

static void test_healthy_drive(void)
{
  drive_reference_t value = reference();
  uint8_t index;
  encoder_count[0] = encoder_count[1] = 0;
  drive_control_init();
  for(index = 0U; index < 12U; index++)
  {
    encoder_count[MOTOR_LEFT] -= 9;
    encoder_count[MOTOR_RIGHT] -= 9;
    expect_true(drive_control_update(&value, 0.0f, 0.02f), "healthy update");
  }
  expect_true(drive_control_has_fault() == 0U, "healthy feedback fault");
  expect_true((applied_left != 0) || (applied_right != 0), "healthy output missing");
}

static void test_bad_feedback_brakes(void)
{
  drive_reference_t value = reference();
  encoder_count[0] = encoder_count[1] = 0;
  drive_control_init();
  encoder_count[MOTOR_LEFT] = 200;
  expect_true(drive_control_update(&value, 0.0f, 0.02f) == 0U, "raw feedback accepted");
  expect_true((applied_left == 0) && (applied_right == 0) && (brake_count != 0U),
              "fault did not brake both motors");
}

int main(void)
{
  test_healthy_drive();
  test_bad_feedback_brakes();
  puts("DRIVE_CONTROL_TEST_PASS");
  return 0;
}
