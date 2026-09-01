#ifndef BUMPER_INPUT_H
#define BUMPER_INPUT_H

#include <stdint.h>

#define BUMPER_INPUT_LEFT_MASK   0x01U
#define BUMPER_INPUT_REAR_MASK   0x02U
#define BUMPER_INPUT_FRONT_MASK  0x04U
#define BUMPER_INPUT_RIGHT_MASK  0x08U
#define BUMPER_INPUT_ALL_MASK    0x0FU

void bumper_input_init(void);
uint8_t bumper_input_read_active_mask(void);
uint8_t bumper_input_take_pressed_latch_mask(void);
uint8_t bumper_input_has_pressed_latch(void);
void bumper_input_edge_irq_handler(uint8_t changed_mask);

#endif
