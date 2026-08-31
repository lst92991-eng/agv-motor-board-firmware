#include "system_watchdog.h"

#include "at32f45x_wdt.h"

#define SYSTEM_WATCHDOG_UPDATE_TIMEOUT 100000UL
/* 40 kHz LICK / 64, (62 + 1) ticks = approximately 100.8 ms. */
#define SYSTEM_WATCHDOG_RELOAD             62U

uint8_t system_watchdog_init(void)
{
  uint32_t timeout;

  wdt_register_write_enable(TRUE);
  wdt_divider_set(WDT_CLK_DIV_64);
  wdt_reload_value_set(SYSTEM_WATCHDOG_RELOAD);
  timeout = SYSTEM_WATCHDOG_UPDATE_TIMEOUT;
  while(((wdt_flag_get(WDT_DIVF_UPDATE_FLAG) != RESET) ||
         (wdt_flag_get(WDT_RLDF_UPDATE_FLAG) != RESET)) &&
        (timeout > 0UL))
  {
    timeout--;
  }
  wdt_register_write_enable(FALSE);
  if(timeout == 0UL)
  {
    return 0U;
  }
  wdt_counter_reload();
  wdt_enable();
  return 1U;
}

void system_watchdog_feed(void)
{
  wdt_counter_reload();
}
