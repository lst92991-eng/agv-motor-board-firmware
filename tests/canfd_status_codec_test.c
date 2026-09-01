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

static void test_odom_status(void)
{
  canfd_odom_status_t odom;
  uint8_t odom_payload[CANFD_ODOM_STATUS_PAYLOAD_SIZE];

  memset(&odom, 0, sizeof(odom));
  odom.uptime_ms = 0x12345678UL;
  odom.valid_flags = 0x0700U;
  odom.linear_mm_s = -300;
  odom.yaw_rate_cdeg_s = 400;
  odom.accel_x_mg = -25;
  odom.accel_y_mg = 30;
  odom.accel_z_mg = -5;
  expect_true(canfd_status_encode_odom(&odom, 0x55AAU, odom_payload,
                                       sizeof(odom_payload)) != 0U,
              "odom status encodes");
  expect_true((odom_payload[4] == 0xAAU) &&
              (odom_payload[5] == 0x55U),
              "odom sequence is little endian");
  expect_true((odom_payload[20] == 0xD4U) &&
              (odom_payload[21] == 0xFEU),
              "negative linear speed is preserved");
  expect_true((odom_payload[24] == 0xE7U) &&
              (odom_payload[25] == 0xFFU) &&
              (odom_payload[26] == 30U),
              "signed IMU acceleration extends the odom payload");

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

int main(void)
{
  test_drive_status();
  test_odom_status();
  test_safety_status_and_bounds();
  if(failures != 0)
  {
    printf("CANFD_STATUS_CODEC_TEST_FAIL,%d\n", failures);
    return 1;
  }
  printf("CANFD_STATUS_CODEC_TEST_PASS\n");
  return 0;
}
