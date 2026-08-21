#include <math.h>
#include <stdio.h>
#include <string.h>

#include "magnetic_follow_controller.h"

static int failures;

static void expect_true(int condition, const char *message)
{
  if(!condition)
  {
    printf("FAIL,%s\n", message);
    failures++;
  }
}

static magnetic_follow_sample_t make_trackable_sample(void)
{
  magnetic_follow_sample_t sample;

  memset(&sample, 0, sizeof(sample));
  sample.receive_time_ms = 1000UL;
  sample.sequence = 10U;
  sample.age_ms = 10U;
  sample.status = 3U;
  sample.flags = 0x07U;
  sample.protocol_valid = 1U;
  sample.active_mask = 0x6EU;
  sample.offset_x10mm = 50;
  sample.strength = 2095U;
  sample.signal[1] = 88U;
  sample.signal[2] = 526U;
  sample.signal[3] = 523U;
  sample.signal[4] = 10U;
  sample.signal[5] = 569U;
  sample.signal[6] = 389U;
  sample.signal[7] = 7U;
  return sample;
}

static void update_sample(magnetic_follow_controller_t *controller,
                          magnetic_follow_sample_t *sample,
                          int16_t offset_x10mm,
                          float requested_run_rpm)
{
  sample->sequence++;
  sample->receive_time_ms += 10UL;
  sample->offset_x10mm = offset_x10mm;
  magnetic_follow_controller_update(controller, sample, requested_run_rpm);
}

static void test_trackability(void)
{
  magnetic_follow_sample_t sample = make_trackable_sample();

  expect_true(magnetic_follow_sample_is_trackable(&sample) != 0U,
              "single inactive-channel valley remains one wide peak");

  sample.active_mask = 0x63U;
  memset(sample.signal, 0, sizeof(sample.signal));
  sample.signal[0] = 500U;
  sample.signal[1] = 450U;
  sample.signal[5] = 400U;
  sample.signal[6] = 380U;
  expect_true(magnetic_follow_sample_is_trackable(&sample) == 0U,
              "two separated comparable peaks are rejected");

  sample = make_trackable_sample();
  sample.age_ms = 101U;
  expect_true(magnetic_follow_sample_is_trackable(&sample) == 0U,
              "stale sample is rejected");
  sample = make_trackable_sample();
  sample.flags = 0x03U;
  expect_true(magnetic_follow_sample_is_trackable(&sample) == 0U,
              "incomplete validity flags are rejected");
}

static void test_control_direction_and_limits(void)
{
  magnetic_follow_controller_t controller;
  magnetic_follow_sample_t sample = make_trackable_sample();
  float correction_rpm;

  magnetic_follow_controller_start(&controller, 0, 0, &sample, 70.0f);
  sample.sequence++;
  sample.receive_time_ms += 10UL;
  magnetic_follow_controller_update(&controller, &sample, 70.0f);

  expect_true(controller.yaw_rate_target_dps > 0.0f,
              "positive offset commands positive yaw rate");
  expect_true(fabsf(controller.yaw_rate_target_dps - 7.0f) < 0.001f,
              "steady 5mm offset preserves documented 7dps target");
  expect_true(controller.yaw_rate_target_dps <= 8.0f,
              "yaw rate target is bounded");
  expect_true((controller.speed_limit_rpm >= 20.0f) &&
              (controller.speed_limit_rpm <= 70.0f),
              "dynamic speed limit stays within request");
  expect_true(fabsf(controller.speed_limit_rpm - 20.1f) < 0.001f,
              "speed limit preserves 0.1rpm recovery slew");

  correction_rpm = magnetic_follow_controller_wheel_correction_rpm(
      &controller, 0.0f, 500.0f, 267.035f);
  expect_true(correction_rpm > 0.0f,
              "positive yaw target creates positive differential correction");
  expect_true(fabsf(magnetic_follow_controller_correction_limit_rpm(
                        1U, 0U) -
                    6.0f) < 0.001f,
              "capture correction limit remains 6rpm");
  expect_true(fabsf(magnetic_follow_controller_correction_limit_rpm(
                        0U, 0U) -
                    8.0f) < 0.001f,
              "track correction limit remains 8rpm");
}

static void test_curve_response_and_low_speed_limit(void)
{
  magnetic_follow_sample_t sample = make_trackable_sample();
  magnetic_follow_controller_t controller;
  float correction_rpm;

  sample.offset_x10mm = 0;
  magnetic_follow_controller_start(&controller, 0, 1U, &sample, 11.23f);
  expect_true((controller.speed_limit_rpm >= 7.99f) &&
              (controller.speed_limit_rpm <= 8.01f),
              "50mmps curve capture starts at a real lower speed");

  sample.sequence++;
  sample.receive_time_ms += 10UL;
  sample.offset_x10mm = 60;
  magnetic_follow_controller_update(&controller, &sample, 11.23f);
  expect_true(fabsf(controller.yaw_rate_target_dps - 12.0f) < 0.001f,
              "growing 6mm curve error reaches the 12dps turn limit");
  expect_true(controller.speed_limit_rpm <= 8.76f,
              "curve deviation does not bypass low-speed limiting");

  sample.sequence++;
  sample.receive_time_ms += 10UL;
  sample.offset_x10mm = 55;
  magnetic_follow_controller_update(&controller, &sample, 11.23f);
  expect_true(controller.yaw_rate_target_dps > 0.0f,
              "falling error cannot reverse a still-positive curve turn");

  correction_rpm = magnetic_follow_controller_wheel_correction_rpm(
      &controller, 0.0f, 500.0f, 267.035f);
  expect_true(correction_rpm > 8.0f,
              "curve yaw loop can request stronger differential drive");
  expect_true(fabsf(magnetic_follow_controller_correction_limit_rpm(
                        0U, 1U) - 11.0f) < 0.001f,
              "curve tracking has an explicit 11rpm correction limit");
}

static void test_curve_mode_uses_global_offset(void)
{
  magnetic_follow_sample_t sample = make_trackable_sample();
  magnetic_follow_controller_t controller;

  memset(sample.signal, 0, sizeof(sample.signal));
  sample.active_mask = 0xCFU;
  sample.signal[0] = 63U;
  sample.signal[1] = 67U;
  sample.signal[2] = 88U;
  sample.signal[3] = 106U;
  sample.signal[6] = 74U;
  sample.signal[7] = 66U;
  sample.strength = 464U;
  sample.offset_x10mm = -33;

  expect_true(magnetic_follow_sample_is_trackable(&sample) == 0U,
              "strict straight mode still rejects a real double peak");
  expect_true(magnetic_follow_sample_is_trackable_for_mode(
                  &sample, 1U) != 0U,
              "curve mode accepts the same multi-peak field");

  magnetic_follow_controller_start(&controller, 0, 1U, &sample, 70.0f);
  sample.sequence++;
  sample.receive_time_ms += 10UL;
  magnetic_follow_controller_update(&controller, &sample, 70.0f);
  expect_true((controller.yaw_rate_target_dps < 0.0f) &&
              (controller.yaw_rate_target_dps > -5.0f),
              "global -3.3mm centroid commands only a small right correction");
}

static void test_adaptive_high_speed_curve_phases(void)
{
  magnetic_follow_sample_t sample = make_trackable_sample();
  magnetic_follow_controller_t controller;
  uint16_t index;

  sample.offset_x10mm = 0;
  magnetic_follow_controller_start(&controller, 0, 1U, &sample, 74.15f);
  expect_true(controller.adaptive_high_speed != 0U,
              "330mmps request enables adaptive high-speed scheduling");
  expect_true(controller.speed_phase ==
              MAGNETIC_FOLLOW_SPEED_FAST_STRAIGHT,
              "adaptive task starts in fast-straight phase");
  expect_true(fabsf(controller.speed_limit_rpm - 74.15f) < 0.001f,
              "fast-straight phase initially allows requested cruise speed");

  for(index = 0U; index < 40U; index++)
  {
    update_sample(&controller, &sample,
                  ((index & 1U) == 0U) ? 10 : -10, 74.15f);
  }
  expect_true(controller.speed_phase ==
              MAGNETIC_FOLLOW_SPEED_FAST_STRAIGHT,
              "centered quantization jitter does not lock out fast cruise");

  update_sample(&controller, &sample, 30, 74.15f);
  expect_true(controller.speed_phase ==
              MAGNETIC_FOLLOW_SPEED_CURVE_SUSPECT,
              "sustained 3mm trend enters curve-suspect phase");
  expect_true(controller.speed_limit_rpm < 74.15f,
              "curve suspicion begins deceleration immediately");

  for(index = 0U; index < 4U; index++)
  {
    update_sample(&controller, &sample, 50, 74.15f);
  }
  expect_true(controller.speed_phase ==
              MAGNETIC_FOLLOW_SPEED_CURVE_TRACK,
              "confirmed curve selects the 75mmps tracking phase");

  update_sample(&controller, &sample, 90, 74.15f);
  expect_true(controller.speed_phase ==
              MAGNETIC_FOLLOW_SPEED_CURVE_EMERGENCY,
              "large curve error selects the 50mmps emergency phase");
  for(index = 0U; index < 100U; index++)
  {
    update_sample(&controller, &sample, 90, 74.15f);
  }
  expect_true((controller.speed_limit_rpm >= 11.22f) &&
              (controller.speed_limit_rpm <= 11.24f),
              "emergency curve phase reaches the explicit 50mmps cap");

  for(index = 0U; index < 60U; index++)
  {
    update_sample(&controller, &sample,
                  ((index & 1U) == 0U) ? 10 : -10, 74.15f);
  }
  expect_true(controller.speed_phase ==
              MAGNETIC_FOLLOW_SPEED_FAST_STRAIGHT,
              "stable curve exit returns to fast-straight phase");
  expect_true(controller.speed_limit_rpm > 11.23f,
              "fast-straight recovery accelerates with a bounded slew");
}

int main(void)
{
  test_trackability();
  test_control_direction_and_limits();
  test_curve_response_and_low_speed_limit();
  test_curve_mode_uses_global_offset();
  test_adaptive_high_speed_curve_phases();
  if(failures != 0)
  {
    printf("MAGNETIC_FOLLOW_CONTROLLER_TEST_FAIL,%d\n", failures);
    return 1;
  }
  printf("MAGNETIC_FOLLOW_CONTROLLER_TEST_PASS\n");
  return 0;
}
