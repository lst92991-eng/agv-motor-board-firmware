/** @file odometry.h @brief 编码器与BMI270组成的平面相对里程计。 */
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

typedef struct
{
  float x_mm;
  float y_mm;
  float yaw_deg;
  uint8_t valid;
  uint8_t encoder_rejected;
} odometry_t;

void odometry_init(void);
void odometry_reset(void);
void odometry_update(int32_t left_count, int32_t right_count,
                     float gyro_z_dps, float dt_s);
void odometry_get(odometry_t *odom);

#endif
