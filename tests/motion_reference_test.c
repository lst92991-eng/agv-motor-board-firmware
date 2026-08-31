#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "agv/motion_contract.h"
#include "motion_reference_control.h"
#include "motion_reference_receiver.h"

static int failures;

static void expect_true(int condition, const char *message)
{
  if(!condition)
  {
    printf("FAIL,%s\n", message);
    failures++;
  }
}

static void expect_near(float actual, float expected, float tolerance,
                        const char *message)
{
  float error = actual - expected;

  if(error < 0.0f)
  {
    error = -error;
  }
  expect_true(error <= tolerance, message);
}

static void make_payload(uint8_t *payload,
                         uint32_t session_id,
                         uint16_t sequence,
                         uint16_t ttl_ms,
                         int16_t linear_mm_s,
                         int16_t angular_mrad_s,
                         uint16_t accel_mm_s2,
                         uint16_t decel_mm_s2)
{
  memset(payload, 0, AGV_MOTION_REFERENCE_PAYLOAD_SIZE);
  agv_motion_write_u32_le(&payload[0], session_id);
  agv_motion_write_u16_le(&payload[4], sequence);
  agv_motion_write_u16_le(&payload[6], ttl_ms);
  agv_motion_write_u16_le(&payload[8], (uint16_t)linear_mm_s);
  agv_motion_write_u16_le(&payload[10], (uint16_t)angular_mrad_s);
  agv_motion_write_u16_le(&payload[12], accel_mm_s2);
  agv_motion_write_u16_le(&payload[14], decel_mm_s2);
}

static motion_reference_platform_limits_t default_limits(void)
{
  motion_reference_platform_limits_t limits;

  limits.max_linear_mm_s = 1000U;
  limits.max_angular_mrad_s = 2000U;
  limits.min_accel_mm_s2 = 100U;
  limits.max_accel_mm_s2 = 1000U;
  limits.min_decel_mm_s2 = 100U;
  limits.max_decel_mm_s2 = 1000U;
  limits.drive_track_mm = 500.0f;
  limits.wheel_circumference_mm = 60.0f;
  limits.max_wheel_rpm = 500.0f;
  return limits;
}

static agv_motion_reference_t body_reference(int16_t linear_mm_s,
                                             int16_t angular_mrad_s)
{
  agv_motion_reference_t reference;

  memset(&reference, 0, sizeof(reference));
  reference.reference_id = 1U;
  reference.produced_ms = 100U;
  reference.valid_for_ms = 200U;
  reference.linear_mm_s = linear_mm_s;
  reference.angular_mrad_s = angular_mrad_s;
  reference.accel_mm_s2 = 400U;
  reference.decel_mm_s2 = 500U;
  reference.source = (uint8_t)AGV_MOTION_SOURCE_NAVIGATION;
  reference.kind = (uint8_t)AGV_MOTION_KIND_BODY_VELOCITY;
  return reference;
}

static void test_golden_payload_and_ttl(void)
{
  static const uint8_t expected[AGV_MOTION_REFERENCE_PAYLOAD_SIZE] =
  {
    0xD4U, 0xC3U, 0xB2U, 0xA1U, 0x44U, 0x33U, 0x3CU, 0x00U,
    0xFAU, 0x00U, 0xE0U, 0xFCU, 0x90U, 0x01U, 0xF4U, 0x01U
  };
  motion_reference_receiver_t receiver = {0};
  agv_motion_reference_t latest;
  uint8_t payload[AGV_MOTION_REFERENCE_PAYLOAD_SIZE];

  make_payload(payload, UINT32_C(0xA1B2C3D4), 0x3344U, 60U,
               250, -800, 400U, 500U);
  expect_true(memcmp(payload, expected, sizeof(expected)) == 0,
              "golden 0x401 payload");
  motion_reference_receiver_reset(&receiver, UINT32_C(0xA1B2C3D4));
  expect_true(motion_reference_receiver_accept(
                  &receiver, payload, (uint16_t)sizeof(payload),
                  1000U, 150U) == MOTION_REFERENCE_RECEIVER_ACCEPTED,
              "golden reference accepted");
  expect_true(motion_reference_receiver_get_latest(
                  &receiver, 1000U, &latest) != 0U,
              "golden latest available");
  expect_true(latest.reference_id == 0x3344U &&
                  latest.linear_mm_s == 250 &&
                  latest.angular_mrad_s == -800 &&
                  latest.accel_mm_s2 == 400U &&
                  latest.decel_mm_s2 == 500U,
              "golden fields decoded");
  expect_true(latest.valid_for_ms == 60U,
              "TTL is intersected with lease");
  receiver.generation++;
  expect_true(motion_reference_receiver_get_latest(
                  &receiver, 1000U, &latest) == 0U,
              "reader rejects an in-progress writer snapshot");
  receiver.generation++;
  expect_true(motion_reference_receiver_get_latest(
                  &receiver, 1000U, &latest) != 0U,
              "reader accepts a stable even generation");
  expect_true(motion_reference_receiver_get_latest(
                  &receiver, 1060U, &latest) == 0U,
              "effective TTL expires exactly");
}

static void test_sequence_rules(void)
{
  motion_reference_receiver_t receiver = {0};
  agv_motion_reference_t latest;
  uint8_t payload[AGV_MOTION_REFERENCE_PAYLOAD_SIZE];

  motion_reference_receiver_reset(&receiver, 7U);
  make_payload(payload, 7U, 10U, 100U, 100, 0, 400U, 500U);
  expect_true(motion_reference_receiver_accept(
                  &receiver, payload, (uint16_t)sizeof(payload),
                  100U, 500U) == MOTION_REFERENCE_RECEIVER_ACCEPTED,
              "first sequence accepted");
  make_payload(payload, 7U, 10U, 500U, 200, 0, 400U, 500U);
  expect_true(motion_reference_receiver_accept(
                  &receiver, payload, (uint16_t)sizeof(payload),
                  150U, 500U) == MOTION_REFERENCE_RECEIVER_DUPLICATE,
              "duplicate rejected without refresh");
  expect_true(motion_reference_receiver_get_latest(
                  &receiver, 199U, &latest) != 0U &&
                  latest.produced_ms == 100U && latest.linear_mm_s == 100,
              "duplicate retains original latest");
  motion_reference_receiver_invalidate_latest(&receiver);
  expect_true(motion_reference_receiver_get_latest(
                  &receiver, 199U, &latest) == 0U,
              "invalidate hides latest reference");
  expect_true(motion_reference_receiver_accept(
                  &receiver, payload, (uint16_t)sizeof(payload),
                  160U, 500U) == MOTION_REFERENCE_RECEIVER_DUPLICATE,
              "invalidate preserves duplicate sequence history");
  expect_true(motion_reference_receiver_get_latest(
                  &receiver, 200U, &latest) == 0U,
              "duplicate did not extend TTL");

  make_payload(payload, 7U, 9U, 100U, 100, 0, 400U, 500U);
  expect_true(motion_reference_receiver_accept(
                  &receiver, payload, (uint16_t)sizeof(payload),
                  201U, 500U) == MOTION_REFERENCE_RECEIVER_OLD,
              "old sequence rejected");

  motion_reference_receiver_reset(&receiver, 7U);
  make_payload(payload, 7U, UINT16_MAX, 100U, 100, 0, 400U, 500U);
  expect_true(motion_reference_receiver_accept(
                  &receiver, payload, (uint16_t)sizeof(payload),
                  300U, 500U) == MOTION_REFERENCE_RECEIVER_ACCEPTED,
              "65535 accepted");
  make_payload(payload, 7U, 0U, 100U, 100, 0, 400U, 500U);
  expect_true(motion_reference_receiver_accept(
                  &receiver, payload, (uint16_t)sizeof(payload),
                  301U, 500U) == MOTION_REFERENCE_RECEIVER_ACCEPTED,
              "65535 to zero wraps forward");
}

static void test_session_reset(void)
{
  motion_reference_receiver_t receiver = {0};
  agv_motion_reference_t latest;
  uint8_t payload[AGV_MOTION_REFERENCE_PAYLOAD_SIZE];

  motion_reference_receiver_reset(&receiver, 11U);
  make_payload(payload, 11U, 20U, 100U, 100, 0, 400U, 500U);
  expect_true(motion_reference_receiver_accept(
                  &receiver, payload, (uint16_t)sizeof(payload),
                  10U, 200U) == MOTION_REFERENCE_RECEIVER_ACCEPTED,
              "old session reference accepted");
  motion_reference_receiver_reset(&receiver, 12U);
  expect_true(motion_reference_receiver_get_latest(
                  &receiver, 10U, &latest) == 0U,
              "new session clears latest");
  expect_true(motion_reference_receiver_accept(
                  &receiver, payload, (uint16_t)sizeof(payload),
                  11U, 200U) == MOTION_REFERENCE_RECEIVER_REJECTED,
              "old session payload rejected");
  make_payload(payload, 12U, 1U, 100U, 100, 0, 400U, 500U);
  expect_true(motion_reference_receiver_accept(
                  &receiver, payload, (uint16_t)sizeof(payload),
                  12U, 0U) == MOTION_REFERENCE_RECEIVER_REJECTED,
              "zero lease rejected");
  expect_true(motion_reference_receiver_accept(
                  &receiver, payload, (uint16_t)sizeof(payload),
                  12U, 200U) == MOTION_REFERENCE_RECEIVER_ACCEPTED,
              "new session starts fresh sequence history");
}

static void test_control_math(void)
{
  motion_reference_platform_limits_t limits = default_limits();
  motion_reference_wheel_targets_t targets;
  agv_motion_reference_t reference = body_reference(100, 400);

  expect_true(motion_reference_control_compute(
                  &reference, &limits, &targets) != 0U,
              "positive yaw reference computes");
  expect_near(targets.left_rpm, 0.0f, 0.001f,
              "positive yaw slows left wheel");
  expect_near(targets.right_rpm, 200.0f, 0.001f,
              "positive yaw speeds right wheel");
  expect_near(targets.common_scale, 1.0f, 0.001f,
              "unsaturated reference scale");

  limits.drive_track_mm = 1000.0f;
  limits.max_wheel_rpm = 60.0f;
  reference = body_reference(100, 1000);
  expect_true(motion_reference_control_compute(
                  &reference, &limits, &targets) != 0U,
              "saturated reference computes");
  expect_near(targets.left_rpm, -40.0f, 0.001f,
              "common scale preserves left ratio");
  expect_near(targets.right_rpm, 60.0f, 0.001f,
              "common scale caps maximum wheel");
  expect_near(targets.common_scale, 0.1f, 0.001f,
              "common scale is shared");

  limits = default_limits();
  limits.min_accel_mm_s2 = 900U;
  limits.max_accel_mm_s2 = 800U;
  expect_true(motion_reference_control_compute(
                  &reference, &limits, &targets) == 0U,
              "invalid platform limits rejected");
  limits = default_limits();
  reference = body_reference(1001, 0);
  expect_true(motion_reference_control_compute(
                  &reference, &limits, &targets) == 0U,
              "reference outside platform speed rejected");
  reference = body_reference(100, 0);
  reference.accel_mm_s2 = 99U;
  expect_true(motion_reference_control_compute(
                  &reference, &limits, &targets) == 0U,
              "reference below platform acceleration rejected");
  reference = body_reference(100, 0);
  reference.decel_mm_s2 = 1001U;
  expect_true(motion_reference_control_compute(
                  &reference, &limits, &targets) == 0U,
              "reference above platform deceleration rejected");
}

int main(void)
{
  test_golden_payload_and_ttl();
  test_sequence_rules();
  test_session_reset();
  test_control_math();
  if(failures != 0)
  {
    printf("MOTION_REFERENCE_TEST_FAIL,%d\n", failures);
    return 1;
  }
  puts("MOTION_REFERENCE_TEST_PASS");
  return 0;
}
