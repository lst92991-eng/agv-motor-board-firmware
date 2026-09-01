/** @file drive_control.h @brief 差速换算、双轮速度PID和主动制动。 */
#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include <stdint.h>

#include "motor_driver.h"

typedef struct
{
  int16_t linear_mm_s;
  int16_t angular_mrad_s;
  uint16_t accel_mm_s2;
  uint16_t decel_mm_s2;
} drive_reference_t;

void drive_control_init(void);
void drive_control_stop(void);
void drive_control_clear_fault(void);
uint8_t drive_control_update(const drive_reference_t *reference,
                             float yaw_rate_mrad_s, float dt_s);
uint8_t drive_control_has_fault(void);
float drive_control_measured_rpm(motor_id_t motor);
int16_t drive_control_pwm(motor_id_t motor);

#endif
