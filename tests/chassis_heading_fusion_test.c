#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "chassis_heading_fusion.h"

static void expect_near(const char *name,
                        float actual,
                        float expected,
                        float tolerance)
{
  if(fabsf(actual - expected) > tolerance)
  {
    fprintf(stderr, "%s actual=%.4f expected=%.4f tolerance=%.4f\n",
            name, actual, expected, tolerance);
    exit(1);
  }
}

static void test_stationary_encoder_constrains_gyro_bias(void)
{
  chassis_heading_fusion_t fusion;
  unsigned int sample;

  chassis_heading_fusion_init(&fusion, 2.0f, 120.0f);
  for(sample = 0U; sample < 2000U; sample++)
  {
    chassis_heading_fusion_update(&fusion, 0.10f, 0.0f, 0.01f);
  }

  expect_near("stationary fused yaw", fusion.fused_yaw_deg, 0.20f, 0.02f);
  expect_near("stationary gyro yaw", fusion.gyro_yaw_deg, 2.0f, 0.02f);
}

static void test_consistent_left_turn_tracks_ninety_degrees(void)
{
  chassis_heading_fusion_t fusion;
  unsigned int sample;

  chassis_heading_fusion_init(&fusion, 2.0f, 120.0f);
  for(sample = 0U; sample < 900U; sample++)
  {
    chassis_heading_fusion_update(&fusion, 10.0f, 0.10f, 0.01f);
  }

  expect_near("left turn fused yaw", fusion.fused_yaw_deg, 90.0f, 0.05f);
  expect_near("left turn disagreement", fusion.disagreement_deg, 0.0f, 0.05f);
}

static void test_encoder_spike_is_rejected(void)
{
  chassis_heading_fusion_t fusion;

  chassis_heading_fusion_init(&fusion, 2.0f, 120.0f);
  chassis_heading_fusion_update(&fusion, 0.0f, 30.0f, 0.01f);

  expect_near("spike fused yaw", fusion.fused_yaw_deg, 0.0f, 0.001f);
  if(fusion.rejected_encoder_samples != 1UL)
  {
    fprintf(stderr, "spike reject count=%lu\n",
            (unsigned long)fusion.rejected_encoder_samples);
    exit(1);
  }
}

static void test_encoder_geometry_reports_left_turn(void)
{
  float yaw_deg = chassis_heading_fusion_encoder_delta_deg(
      -3882, 3882, 267.035f, 2640.0f, 500.0f);

  expect_near("encoder left turn", yaw_deg, 90.0f, 0.05f);
}

static void test_wrap_remains_continuous(void)
{
  chassis_heading_fusion_t fusion;
  unsigned int sample;

  chassis_heading_fusion_init(&fusion, 2.0f, 120.0f);
  for(sample = 0U; sample < 2000U; sample++)
  {
    chassis_heading_fusion_update(&fusion, 10.0f, 0.10f, 0.01f);
  }

  expect_near("wrapped fused yaw", fusion.fused_yaw_deg, -160.0f, 0.05f);
}

int main(void)
{
  test_stationary_encoder_constrains_gyro_bias();
  test_consistent_left_turn_tracks_ninety_degrees();
  test_encoder_spike_is_rejected();
  test_encoder_geometry_reports_left_turn();
  test_wrap_remains_continuous();
  puts("CHASSIS_HEADING_FUSION_TEST_PASS");
  return 0;
}
