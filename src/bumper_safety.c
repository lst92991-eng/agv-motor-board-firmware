#include "bumper_safety.h"

#include <stddef.h>
#include "bumper_input.h"

#define BUMPER_RELEASE_TICKS 10U

static bumper_safety_snapshot_t bumper_state;
static uint8_t bumper_release_ticks;

void bumper_safety_init(void)
{
  bumper_input_init();
  bumper_state.active_mask = bumper_input_read_active_mask();
  bumper_state.latched_mask = bumper_state.active_mask;
  bumper_release_ticks = (bumper_state.active_mask == 0U) ?
      BUMPER_RELEASE_TICKS : 0U;
  (void)bumper_input_take_pressed_latch_mask();
}

uint8_t bumper_safety_step_10ms(void)
{
  uint8_t old_active = bumper_state.active_mask;
  uint8_t old_latched = bumper_state.latched_mask;
  uint8_t pressed = bumper_input_take_pressed_latch_mask();

  bumper_state.active_mask = bumper_input_read_active_mask();
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

uint8_t bumper_safety_can_clear(void)
{
  return ((bumper_input_read_active_mask() == 0U) &&
          (bumper_input_has_pressed_latch() == 0U) &&
          (bumper_release_ticks >= BUMPER_RELEASE_TICKS)) ? 1U : 0U;
}

uint8_t bumper_safety_clear_latch(void)
{
  if(bumper_safety_can_clear() == 0U) return 0U;
  bumper_state.latched_mask = 0U;
  return 1U;
}

void bumper_safety_get_snapshot(bumper_safety_snapshot_t *snapshot)
{
  if(snapshot != NULL) *snapshot = bumper_state;
}
