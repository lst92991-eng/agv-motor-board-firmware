/**
 * @file system_watchdog.h
 * @brief Control、CAN和Power三任务进度门与硬件WDT接口。
 */

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
  uint32_t last_power_generation;
  uint8_t initialized;
} system_watchdog_health_t;

/* 三个关键任务都产生新generation后才允许喂狗；任一停滞都会自然复位。 */
static inline uint8_t system_watchdog_health_step(
    system_watchdog_health_t *health,
    uint32_t control_generation,
    uint32_t can_generation,
    uint32_t power_generation)
{
  uint8_t healthy;

  if(health == (system_watchdog_health_t *)0)
  {
    return 0U;
  }
  healthy = ((control_generation != health->last_control_generation) &&
             (can_generation != health->last_can_generation) &&
             (power_generation != health->last_power_generation)) ? 1U : 0U;
  health->last_control_generation = control_generation;
  health->last_can_generation = can_generation;
  health->last_power_generation = power_generation;
  if(health->initialized == 0U)
  {
    health->initialized = 1U;
    return ((control_generation != 0U) &&
            (can_generation != 0U) &&
            (power_generation != 0U)) ? 1U : 0U;
  }
  return healthy;
}

/* 配置约100ms独立看门狗；寄存器同步失败返回0。 */
uint8_t system_watchdog_init(void);
/* 仅由通过health_step的Watchdog任务调用。 */
void system_watchdog_feed(void);

#ifdef __cplusplus
}
#endif

#endif
