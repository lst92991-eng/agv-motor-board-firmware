#include <stdio.h>
#include <stdlib.h>

#include "safety.h"

static void expect_true(int value, const char *text)
{
  if(!value) { fprintf(stderr, "%s\n", text); exit(1); }
}

int main(void)
{
  adc_dma_sample_t sample = {2260U, 0U, 0U, 0U};
  safety_status_t status;
  unsigned int index;
  safety_init();
  for(index = 1U; index <= 20U; index++)
  {
    sample.sequence = index;
    safety_update(index * 20U, &sample, 1U);
  }
  expect_true(safety_can_enable() != 0U, "zero calibration did not enable");
  sample.sequence++;
  sample.bus_raw = 100U;
  safety_update(420U, &sample, 0U);
  expect_true((safety_fault() & SAFETY_FAULT_POWER) != 0U, "undervoltage not latched");
  sample.sequence++;
  sample.bus_raw = 2260U;
  safety_update(440U, &sample, 0U);
  expect_true(safety_clear() != 0U, "recovered power fault did not clear");
  safety_get(&status);
  expect_true(status.ready != 0U, "ready not restored");
  puts("SAFETY_TEST_PASS");
  return 0;
}
