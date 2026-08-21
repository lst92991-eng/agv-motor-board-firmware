#include <stdio.h>
#include <string.h>

#include "canfd_status_codec.h"

static int failures;

static void expect_true(int condition, const char *message)
{
  if(!condition)
  {
    printf("FAIL,%s\n", message);
    failures++;
  }
}

static void test_drive_status(void)
{
  canfd_drive_status_t status;
  uint8_t payload[CANFD_DRIVE_STATUS_PAYLOAD_SIZE];

  memset(&status, 0, sizeof(status));
  status.application_state = 1U;
  status.motion_state = 2U;
  status.motion_mode = 3U;
  status.flags = 4U;
  status.command_id = 0x78563412UL;
  status.session_id = 0xF0DEBC9AUL;
  status.x_mm = -2L;
  status.y_mm = 0x01020304L;
  status.yaw_cdeg = -100L;

  expect_true(canfd_status_encode_drive(&status, payload,
                                        sizeof(payload)) != 0U,
              "drive status encodes");
  expect_true((payload[0] == 1U) && (payload[3] == 4U),
              "drive state bytes stay ordered");
  expect_true((payload[4] == 0x12U) && (payload[7] == 0x78U),
              "command id is little endian");
  expect_true((payload[20] == 0xFEU) && (payload[23] == 0xFFU),
              "signed x uses two's complement little endian");
}

static void test_odom_and_motor_status(void)
{
  canfd_odom_status_t odom;
  canfd_motor_status_t motor;
  uint8_t odom_payload[CANFD_ODOM_STATUS_PAYLOAD_SIZE];
  uint8_t motor_payload[CANFD_MOTOR_STATUS_PAYLOAD_SIZE];

  memset(&odom, 0, sizeof(odom));
  memset(&motor, 0, sizeof(motor));
  odom.uptime_ms = 0x12345678UL;
  odom.valid_flags = 0x0700U;
  odom.linear_mm_s = -300;
  odom.yaw_rate_cdeg_s = 400;
  expect_true(canfd_status_encode_odom(&odom, 0x55AAU, odom_payload,
                                       sizeof(odom_payload)) != 0U,
              "odom status encodes");
  expect_true((odom_payload[4] == 0xAAU) &&
              (odom_payload[5] == 0x55U),
              "odom sequence is little endian");
  expect_true((odom_payload[20] == 0xD4U) &&
              (odom_payload[21] == 0xFEU),
              "negative linear speed is preserved");

  motor.left_target_rpm_x100 = -1;
  motor.right_target_rpm_x100 = 2;
  motor.left_encoder_count = -3L;
  motor.left_invalid_count = 4U;
  motor.right_invalid_count = 5U;
  motor.state = 6U;
  motor.ir_activity = 1U;
  motor.ir_activity_percent = 24U;
  motor.ir_raw_level = 0U;
  motor.ir_falling_edges = 0x1234U;
  expect_true(canfd_status_encode_motor(&motor, motor_payload,
                                        sizeof(motor_payload)) != 0U,
              "motor status encodes");
  expect_true((motor_payload[4] == 0xFFU) &&
              (motor_payload[5] == 0xFFU),
              "negative target rpm is preserved");
  expect_true((motor_payload[24] == 4U) &&
              (motor_payload[26] == 6U) &&
              (motor_payload[27] == 1U) &&
              (motor_payload[28] == 24U) &&
              (motor_payload[29] == 0U) &&
              (motor_payload[30] == 0x34U) &&
              (motor_payload[31] == 0x12U),
              "motor and IR diagnostics keep a stable layout");
}

static void test_safety_status_and_bounds(void)
{
  canfd_safety_status_t status;
  uint8_t payload[CANFD_SAFETY_STATUS_PAYLOAD_SIZE];

  memset(&status, 0, sizeof(status));
  status.bumper_mask = 0x0FU;
  status.estop_source_mask = 0x05U;
  status.safety_state = 2U;
  status.fault_mask = 0x11223344UL;
  expect_true(canfd_status_encode_safety(&status, payload,
                                         sizeof(payload)) != 0U,
              "safety status encodes");
  expect_true((payload[4] == 0x0FU) && (payload[7] == 0U),
              "safety fields and reserved byte are stable");
  expect_true(canfd_status_encode_safety(&status, payload,
                                         sizeof(payload) - 1U) == 0U,
              "short output buffer is rejected");
  expect_true(canfd_status_encode_safety(NULL, payload,
                                         sizeof(payload)) == 0U,
              "null status is rejected");
}

static void test_dock_status(void)
{
  canfd_dock_status_t status;
  uint8_t payload[CANFD_DOCK_STATUS_PAYLOAD_SIZE];

  memset(&status, 0, sizeof(status));
  status.sample_sequence = 0x12345678UL;
  status.left_mm = 321U;
  status.right_mm = 300U;
  status.distance_error_mm = -12;
  status.alignment_error_mm = 21;
  status.ir_falling_edges = 9U;
  status.flags = 0x7FU;
  status.ir_activity_percent = 18U;
  status.left_api_status = -3;
  status.right_range_status = 4U;
  status.bus_error = -14L;
  expect_true(canfd_status_encode_dock(&status, payload,
                                       sizeof(payload)) != 0U,
              "dock status encodes");
  expect_true((payload[4] == 0x78U) &&
              (payload[7] == 0x12U) &&
              (payload[8] == 0x41U) &&
              (payload[9] == 0x01U),
              "dock sequence and left distance are little endian");
  expect_true((payload[12] == 0xF4U) &&
              (payload[13] == 0xFFU) &&
              (payload[18] == 0x7FU) &&
              (payload[20] == 0xFDU),
              "dock signed errors and flags are stable");
  expect_true((payload[24] == 0xF2U) &&
              (payload[27] == 0xFFU) &&
              (payload[28] == 0U),
              "dock bus error and reserved bytes are stable");
}

int main(void)
{
  test_drive_status();
  test_odom_and_motor_status();
  test_safety_status_and_bounds();
  test_dock_status();
  if(failures != 0)
  {
    printf("CANFD_STATUS_CODEC_TEST_FAIL,%d\n", failures);
    return 1;
  }
  printf("CANFD_STATUS_CODEC_TEST_PASS\n");
  return 0;
}
