/**
  ****************************************************************************
  * @file     bumper_safety.c
  * @brief    四路防撞条本地安全锁存与释放确认
  *
  * 任意低电平有效输入都会锁存故障。按下不做软件消抖等待，优先保证停车；
  * 全部释放连续稳定100ms后也不会自动恢复，必须由上层显式清除锁存。
  ****************************************************************************
  */

#include "bumper_safety.h"

#include <stddef.h>

#include "bumper_input.h"

#define BUMPER_SAFETY_STEP_MS                 10U
#define BUMPER_SAFETY_RELEASE_STABLE_MS      100U

static bumper_safety_snapshot_t bumper_safety_snapshot;

void bumper_safety_init(void)
{
  bumper_input_init();
  bumper_safety_snapshot.raw_mask = bumper_input_read_raw_mask();
  bumper_safety_snapshot.active_mask =
      (uint8_t)(~bumper_safety_snapshot.raw_mask) &
      BUMPER_INPUT_ALL_MASK;
  bumper_safety_snapshot.latched_mask =
      bumper_safety_snapshot.active_mask;
  bumper_safety_snapshot.release_stable_ms =
      (bumper_safety_snapshot.active_mask == 0U) ?
      BUMPER_SAFETY_RELEASE_STABLE_MS : 0U;
  bumper_safety_snapshot.event_counter =
      (bumper_safety_snapshot.active_mask != 0U) ? 1U : 0U;
  (void)bumper_input_take_changed_mask();
  (void)bumper_input_take_pressed_latch_mask();
}

uint8_t bumper_safety_step_10ms(void)
{
  uint8_t previous_raw = bumper_safety_snapshot.raw_mask;
  uint8_t previous_active = bumper_safety_snapshot.active_mask;
  uint8_t previous_latched = bumper_safety_snapshot.latched_mask;
  uint8_t pressed_latch = bumper_input_take_pressed_latch_mask();

  bumper_safety_snapshot.raw_mask = bumper_input_read_raw_mask();
  bumper_safety_snapshot.active_mask =
      (uint8_t)(~bumper_safety_snapshot.raw_mask) &
      BUMPER_INPUT_ALL_MASK;

  if((pressed_latch | bumper_safety_snapshot.active_mask) != 0U)
  {
    bumper_safety_snapshot.latched_mask |=
        pressed_latch | bumper_safety_snapshot.active_mask;
    bumper_safety_snapshot.release_stable_ms = 0U;
  }
  else if(bumper_safety_snapshot.release_stable_ms <
          BUMPER_SAFETY_RELEASE_STABLE_MS)
  {
    bumper_safety_snapshot.release_stable_ms += BUMPER_SAFETY_STEP_MS;
    if(bumper_safety_snapshot.release_stable_ms >
       BUMPER_SAFETY_RELEASE_STABLE_MS)
    {
      bumper_safety_snapshot.release_stable_ms =
          BUMPER_SAFETY_RELEASE_STABLE_MS;
    }
  }

  (void)bumper_input_take_changed_mask();
  if((previous_raw != bumper_safety_snapshot.raw_mask) ||
     (previous_active != bumper_safety_snapshot.active_mask) ||
     (previous_latched != bumper_safety_snapshot.latched_mask))
  {
    bumper_safety_snapshot.event_counter++;
    return 1U;
  }
  return 0U;
}

uint8_t bumper_safety_can_clear(void)
{
  return ((bumper_safety_snapshot.active_mask == 0U) &&
          (bumper_safety_snapshot.release_stable_ms >=
           BUMPER_SAFETY_RELEASE_STABLE_MS)) ? 1U : 0U;
}

uint8_t bumper_safety_clear_latch(void)
{
  if(bumper_safety_can_clear() == 0U)
  {
    return 0U;
  }
  if(bumper_safety_snapshot.latched_mask != 0U)
  {
    bumper_safety_snapshot.latched_mask = 0U;
    bumper_safety_snapshot.event_counter++;
  }
  return 1U;
}

void bumper_safety_get_snapshot(bumper_safety_snapshot_t *snapshot)
{
  if(snapshot != NULL)
  {
    *snapshot = bumper_safety_snapshot;
  }
}
