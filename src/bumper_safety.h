#ifndef BUMPER_SAFETY_H
#define BUMPER_SAFETY_H

#include <stdint.h>

typedef struct
{
  uint8_t active_mask;
  uint8_t latched_mask;
} bumper_safety_snapshot_t;

void bumper_safety_init(void);
uint8_t bumper_safety_step_10ms(void);
uint8_t bumper_safety_can_clear(void);
uint8_t bumper_safety_clear_latch(void);
void bumper_safety_get_snapshot(bumper_safety_snapshot_t *snapshot);

#endif
