#ifndef BUMPER_SAFETY_H
#define BUMPER_SAFETY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t raw_mask;
  uint8_t active_mask;
  uint8_t latched_mask;
  uint16_t release_stable_ms;
  uint16_t event_counter;
} bumper_safety_snapshot_t;

/* 初始化防撞输入和安全锁存；上电时已按下的方向会立即进入锁存状态。 */
void bumper_safety_init(void);

/* 每10ms调用一次，状态发生变化时返回1。 */
uint8_t bumper_safety_step_10ms(void);

/* 所有方向释放稳定100ms后返回1；故障锁存仍需显式清除。 */
uint8_t bumper_safety_can_clear(void);

/* 满足释放条件时清除锁存并返回1，否则保持锁存并返回0。 */
uint8_t bumper_safety_clear_latch(void);

/* 获取一致的防撞安全状态快照。 */
void bumper_safety_get_snapshot(bumper_safety_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
