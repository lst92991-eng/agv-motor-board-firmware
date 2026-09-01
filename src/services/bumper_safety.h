/**
 * @file bumper_safety.h
 * @brief 防撞条实时状态、事件锁存和显式清除条件。
 */

#ifndef BUMPER_SAFETY_H
#define BUMPER_SAFETY_H

#include <stdint.h>

typedef struct
{
  uint8_t active_mask;   /**< 当前仍为低电平的触点。 */
  uint8_t latched_mask;  /**< 自上次显式清除后发生过的触点。 */
} bumper_safety_snapshot_t;

/** 初始化输入；上电时已按下的方向直接锁存。 */
void bumper_safety_init(void);
/** 每10ms执行一次；状态或锁存变化返回1。 */
uint8_t bumper_safety_step_10ms(void);
/** 全部释放、无ISR待处理事件并稳定100ms后返回1。 */
uint8_t bumper_safety_can_clear(void);
/** 仅在can_clear成立时清除锁存，不负责重新使能电机。 */
uint8_t bumper_safety_clear_latch(void);
/** 复制实时与锁存位图。 */
void bumper_safety_get_snapshot(bumper_safety_snapshot_t *snapshot);

#endif
