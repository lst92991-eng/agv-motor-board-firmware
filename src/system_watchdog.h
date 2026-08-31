#ifndef SYSTEM_WATCHDOG_H
#define SYSTEM_WATCHDOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint32_t last_control_generation;
  uint32_t last_can_generation;
  uint8_t initialized;
} system_watchdog_health_t;

/* Feed is allowed only after both safety-critical tasks made new progress. */
static inline uint8_t system_watchdog_health_step(
    system_watchdog_health_t *health,
    uint32_t control_generation,
    uint32_t can_generation)
{
  uint8_t healthy;

  if(health == (system_watchdog_health_t *)0)
  {
    return 0U;
  }
  healthy = ((control_generation != health->last_control_generation) &&
             (can_generation != health->last_can_generation)) ? 1U : 0U;
  health->last_control_generation = control_generation;
  health->last_can_generation = can_generation;
  if(health->initialized == 0U)
  {
    health->initialized = 1U;
    return ((control_generation != 0U) &&
            (can_generation != 0U)) ? 1U : 0U;
  }
  return healthy;
}

/* Configure an approximately 100 ms independent watchdog. */
uint8_t system_watchdog_init(void);
void system_watchdog_feed(void);

#ifdef __cplusplus
}
#endif

#endif
