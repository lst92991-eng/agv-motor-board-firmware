/**
 * @file bumper_safety.c
 * @brief 防撞条10ms释放消抖与故障锁存状态机。
 *
 * active_mask反映实时触点，latched_mask保存已发生事件。释放只产生“可清除”条件，
 * 不会自动恢复运动，最终清除必须来自底盘显式FAULT_RESET路径。
 */

#include "bumper_safety.h"

#include <stddef.h>
#include "bumper_input.h"

#define BUMPER_RELEASE_TICKS 10U  /* 10ms周期下要求连续释放100ms。 */

static bumper_safety_snapshot_t bumper_state;
static uint8_t bumper_release_ticks;

/** @brief 初始化实时位图、事件锁存和释放稳定计数器。 */
void bumper_safety_init(void)
{
  bumper_input_init();
  bumper_state.active_mask = bumper_input_read_active_mask();
  /* 上电时已经按下的触点同样必须立即进入锁存状态。 */
  bumper_state.latched_mask = bumper_state.active_mask;
  bumper_release_ticks = (bumper_state.active_mask == 0U) ?
      BUMPER_RELEASE_TICKS : 0U;
  (void)bumper_input_take_pressed_latch_mask();
}

/** @brief 每10ms合并ISR证据与实时电平并更新100ms释放窗口。 */
uint8_t bumper_safety_step_10ms(void)
{
  uint8_t old_active = bumper_state.active_mask;
  uint8_t old_latched = bumper_state.latched_mask;
  uint8_t pressed = bumper_input_take_pressed_latch_mask();

  bumper_state.active_mask = bumper_input_read_active_mask();
  /* 合并ISR短脉冲与实时电平，任何一个存在都重新开始释放计时。 */
  if((pressed | bumper_state.active_mask) != 0U)
  {
    bumper_state.latched_mask |= pressed | bumper_state.active_mask;
    bumper_release_ticks = 0U;
  }
  else if(bumper_release_ticks < BUMPER_RELEASE_TICKS)
  {
    bumper_release_ticks++;
  }
  return ((old_active != bumper_state.active_mask) ||
          (old_latched != bumper_state.latched_mask)) ? 1U : 0U;
}

/** @brief 检查实时输入、ISR锁存和释放窗口是否都允许清故障。 */
uint8_t bumper_safety_can_clear(void)
{
  return ((bumper_input_read_active_mask() == 0U) &&
          (bumper_input_has_pressed_latch() == 0U) &&
          (bumper_release_ticks >= BUMPER_RELEASE_TICKS)) ? 1U : 0U;
}

/** @brief 在全部清除条件满足时清空历史防撞锁存。 */
uint8_t bumper_safety_clear_latch(void)
{
  if(bumper_safety_can_clear() == 0U) return 0U;
  bumper_state.latched_mask = 0U;
  return 1U;
}

/** @brief 复制当前实时与锁存方向位图。 */
void bumper_safety_get_snapshot(bumper_safety_snapshot_t *snapshot)
{
  if(snapshot != NULL) *snapshot = bumper_state;
}
