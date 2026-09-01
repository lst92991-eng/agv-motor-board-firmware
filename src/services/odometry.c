/**
 * @file odometry.c
 * @brief 陀螺仪预测航向、编码器低频修正并积分x/y。
 */
#include "odometry.h"

#include <math.h>

#include "board_config.h"

#define ODOMETRY_RAD_TO_DEG 57.2957795f
#define ODOMETRY_DEG_TO_RAD 0.01745329252f
#define ODOMETRY_CORRECTION_TAU_S 2.0f
#define ODOMETRY_MAX_ENCODER_RATE_DPS 120.0f

static odometry_t odometry_state;
static int32_t odometry_last_left;
static int32_t odometry_last_right;
static float odometry_yaw_unwrapped;
static float odometry_encoder_yaw;

static float odometry_abs(float value) { return (value < 0.0f) ? -value : value; }
static float odometry_wrap(float yaw)
{
  while(yaw > 180.0f) yaw -= 360.0f;
  while(yaw < -180.0f) yaw += 360.0f;
  return yaw;
}

void odometry_init(void)
{
  odometry_reset();
}

void odometry_reset(void)
{
  odometry_state.x_mm = 0.0f;
  odometry_state.y_mm = 0.0f;
  odometry_state.yaw_deg = 0.0f;
  odometry_state.valid = 0U;
  odometry_state.encoder_rejected = 0U;
  odometry_last_left = odometry_last_right = 0;
  odometry_yaw_unwrapped = odometry_encoder_yaw = 0.0f;
}

void odometry_update(int32_t left_count, int32_t right_count,
                     float gyro_z_dps, float dt_s)
{
  int32_t left_delta;
  int32_t right_delta;
  float distance;
  float encoder_delta_yaw;
  float predicted_yaw;
  float encoder_rate;
  float yaw_before;
  float yaw_delta;
  float midpoint;

  if((dt_s < 0.001f) || (dt_s > 0.100f))
  {
    odometry_state.valid = 0U;
    return;
  }
  left_delta = left_count - odometry_last_left;
  right_delta = right_count - odometry_last_right;
  odometry_last_left = left_count;
  odometry_last_right = right_count;
  yaw_before = odometry_yaw_unwrapped;
  predicted_yaw = yaw_before + gyro_z_dps * dt_s;
  encoder_delta_yaw = ((float)(right_delta - left_delta) *
      BOARD_WHEEL_CIRCUMFERENCE_MM / (BOARD_ENCODER_COUNTS_PER_REV *
      BOARD_TRACK_WIDTH_MM)) * ODOMETRY_RAD_TO_DEG;
  encoder_rate = encoder_delta_yaw / dt_s;
  if(odometry_abs(encoder_rate) <= ODOMETRY_MAX_ENCODER_RATE_DPS)
  {
    float gain = dt_s / (ODOMETRY_CORRECTION_TAU_S + dt_s);
    odometry_encoder_yaw += encoder_delta_yaw;
    odometry_yaw_unwrapped = predicted_yaw + gain *
        (odometry_encoder_yaw - predicted_yaw);
  }
  else
  {
    odometry_yaw_unwrapped = predicted_yaw;
    odometry_state.encoder_rejected = 1U;
  }
  yaw_delta = odometry_yaw_unwrapped - yaw_before;
  midpoint = (yaw_before + yaw_delta * 0.5f) * ODOMETRY_DEG_TO_RAD;
  distance = ((float)(left_delta + right_delta) * 0.5f) *
      BOARD_WHEEL_CIRCUMFERENCE_MM / BOARD_ENCODER_COUNTS_PER_REV;
  odometry_state.x_mm += distance * cosf(midpoint);
  odometry_state.y_mm += distance * sinf(midpoint);
  odometry_state.yaw_deg = odometry_wrap(odometry_yaw_unwrapped);
  odometry_state.valid = 1U;
}

void odometry_get(odometry_t *odom)
{
  if(odom != 0) *odom = odometry_state;
}
