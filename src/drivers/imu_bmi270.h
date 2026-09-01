/** @file imu_bmi270.h @brief Bosch BMI270官方库的AT32 SPI3薄适配。 */
#ifndef IMU_BMI270_H
#define IMU_BMI270_H

#include <stdint.h>

uint8_t imu_bmi270_init(void);
uint8_t imu_bmi270_update(float dt_s);
float imu_bmi270_gyro_z_dps(void);
uint8_t imu_bmi270_fresh(void);

#endif
