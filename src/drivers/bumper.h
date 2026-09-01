/** @file bumper.h @brief 四路低有效防撞条的输入和锁存。 */
#ifndef BUMPER_H
#define BUMPER_H

#include <stdint.h>

#define BUMPER_FRONT 0x01U
#define BUMPER_REAR  0x02U
#define BUMPER_LEFT  0x04U
#define BUMPER_RIGHT 0x08U

void bumper_init(void);
void bumper_edge_irq(uint8_t changed_mask);
void bumper_update(void);
uint8_t bumper_latched(void);
uint8_t bumper_can_clear(void);
uint8_t bumper_clear(void);

#endif
