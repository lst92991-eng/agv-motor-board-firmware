#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <stdint.h>
#include "motor_driver.h"

void motor_encoder_init(void);
int32_t motor_encoder_get_count(motor_id_t motor);
uint32_t motor_encoder_get_invalid_transition_count(motor_id_t motor);
void motor_encoder_reset_all(void);
void motor_encoder_left_edge_handler(void);
void motor_encoder_right_edge_handler(void);

#endif
