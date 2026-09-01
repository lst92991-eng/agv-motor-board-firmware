#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

typedef enum { MOTOR_LEFT = 0, MOTOR_RIGHT } motor_id_t;
typedef enum { MOTOR_STOP_COAST = 0, MOTOR_STOP_BRAKE } motor_stop_mode_t;

void motor_driver_init(void);
void motor_driver_set_speed(motor_id_t motor, int16_t speed_permille);
void motor_driver_set_speeds(int16_t left_speed_permille,
                             int16_t right_speed_permille);
void motor_driver_stop(motor_id_t motor, motor_stop_mode_t mode);
void motor_driver_stop_all(motor_stop_mode_t mode);

#endif
