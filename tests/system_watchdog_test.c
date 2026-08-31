#include <stdint.h>
#include <stdio.h>

#include "system_watchdog.h"

static int failures;

static void expect_true(int condition, const char *message)
{
  if(!condition)
  {
    printf("FAIL,%s\n", message);
    failures++;
  }
}

int main(void)
{
  system_watchdog_health_t health = {0};

  expect_true(system_watchdog_health_step(&health, 0U, 0U, 0U) == 0U,
              "zero generations do not feed at startup");
  expect_true(system_watchdog_health_step(&health, 1U, 1U, 1U) != 0U,
              "all safety tasks progressing permits feed");
  expect_true(system_watchdog_health_step(&health, 2U, 1U, 2U) == 0U,
              "stalled CAN task blocks feed");
  expect_true(system_watchdog_health_step(&health, 2U, 2U, 3U) == 0U,
              "control must also progress since the last check");
  expect_true(system_watchdog_health_step(&health, 3U, 3U, 3U) == 0U,
              "stalled PowerMonitor blocks feed");
  expect_true(system_watchdog_health_step(&health, 4U, 4U, 4U) != 0U,
              "all recovered tasks permit feed again");
  if(failures != 0)
  {
    printf("SYSTEM_WATCHDOG_TEST_FAIL,%d\n", failures);
    return 1;
  }
  puts("SYSTEM_WATCHDOG_TEST_PASS");
  return 0;
}
