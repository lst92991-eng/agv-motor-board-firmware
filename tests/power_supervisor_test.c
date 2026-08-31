#include <stdint.h>
#include <stdio.h>

#include "power_supervisor.h"

static int failures;

static power_supervisor_config_t test_config(uint8_t approved)
{
  power_supervisor_config_t config =
  {
    1000UL, 1000UL, 10U, 10U, 19U, 10000UL, 0L,
    10000UL, 11000UL, 19000UL, 20000UL,
    500UL, 60U, 20U, 3U, approved
  };
  return config;
}

static void expect_true(int condition, const char *message)
{
  if(!condition)
  {
    printf("FAIL,%s\n", message);
    failures++;
  }
}

static void calibrate(uint16_t left_raw, uint16_t right_raw)
{
  uint16_t index;

  for(index = 0U; index < POWER_SUPERVISOR_ZERO_SAMPLE_COUNT; index++)
  {
    power_supervisor_step(1U, 1500U, left_raw, right_raw, 1U);
  }
}

static void test_invalid_and_unapproved_config(void)
{
  power_supervisor_config_t config = test_config(1U);
  const power_supervisor_config_t *provisional;
  power_supervisor_snapshot_t snapshot;

  config.left_current_ua_per_count = 0UL;
  power_supervisor_init(&config);
  expect_true((power_supervisor_get_fault_mask() &
               POWER_FAULT_CONFIG_INVALID) != 0U,
              "invalid config fails closed");

  config = test_config(1U);
  config.zero_max_raw = 20U;
  power_supervisor_init(&config);
  expect_true((power_supervisor_get_fault_mask() &
               POWER_FAULT_CONFIG_INVALID) != 0U,
              "zero window cannot exceed INA180 hardware bound");
  config = test_config(1U);
  config.peak_current_ma = 5000UL;
  power_supervisor_init(&config);
  expect_true((power_supervisor_get_fault_mask() &
               POWER_FAULT_CONFIG_INVALID) != 0U,
              "unobservable current threshold fails closed");
  config = test_config(1U);
  config.overvoltage_trip_mv = 50000UL;
  power_supervisor_init(&config);
  expect_true((power_supervisor_get_fault_mask() &
               POWER_FAULT_CONFIG_INVALID) != 0U,
              "unobservable voltage threshold fails closed");

  config = test_config(1U);
  config.left_current_ua_per_count = 1053721122UL;
  power_supervisor_init(&config);
  expect_true((power_supervisor_get_fault_mask() &
               POWER_FAULT_CONFIG_INVALID) != 0U,
              "full-scale current conversion cannot overflow uint32");

  config = test_config(1U);
  config.left_current_ua_per_count = 1049000000UL;
  power_supervisor_init(&config);
  expect_true((power_supervisor_get_fault_mask() &
               POWER_FAULT_CONFIG_INVALID) != 0U,
              "full-scale check rejects gain inside reduced-window gap");

  config = test_config(0U);
  power_supervisor_init(&config);
  calibrate(10U, 10U);
  expect_true((power_supervisor_get_fault_mask() &
               POWER_FAULT_NOT_APPROVED) != 0U,
              "unapproved limits are reported");
  expect_true(power_supervisor_is_ready() == 0U,
              "unapproved limits block ready");

  provisional = power_supervisor_provisional_config();
  expect_true((provisional->limits_approved == 0U) &&
              (provisional->left_current_ua_per_count == 806UL) &&
              (provisional->zero_max_raw == 19U) &&
              (provisional->bus_uv_per_count == 8864UL) &&
              (provisional->undervoltage_trip_mv == 5500UL) &&
              (provisional->overvoltage_trip_mv == 33000UL) &&
              (provisional->peak_current_ma == 3000UL),
              "production ceilings stay formula-derived and provisional");
  power_supervisor_init(provisional);
  power_supervisor_step(1U, 1000U, 1000U, 1000U, 0U);
  power_supervisor_get_snapshot(&snapshot);
  expect_true((snapshot.bus_mv == 8864UL) &&
              (snapshot.left_current_ma == 806UL),
              "nominal board formulas convert ADC counts");
}

static void test_units_and_zero_calibration(void)
{
  power_supervisor_config_t config = test_config(1U);
  power_supervisor_snapshot_t snapshot;

  power_supervisor_init(&config);
  power_supervisor_step(1U, 1500U, 260U, 110U, 0U);
  power_supervisor_get_snapshot(&snapshot);
  expect_true((snapshot.bus_mv == 15000UL) &&
              (snapshot.left_current_ma == 250UL) &&
              (snapshot.right_current_ma == 100UL),
              "raw samples convert with explicit units");

  power_supervisor_init(&config);
  for(snapshot.zero_samples = 0U; snapshot.zero_samples < 63U;
      snapshot.zero_samples++)
  {
    power_supervisor_step(1U, 1500U, 12U, 12U, 1U);
  }
  power_supervisor_step(1U, 1500U, 20U, 12U, 1U);
  power_supervisor_get_snapshot(&snapshot);
  expect_true((snapshot.zero_samples == 0U) &&
              (power_supervisor_is_ready() == 0U),
              "contaminated zero sample resets calibration window");

  power_supervisor_init(&config);
  calibrate(12U, 15U);
  power_supervisor_get_snapshot(&snapshot);
  expect_true((snapshot.left_zero_raw == 12U) &&
              (snapshot.right_zero_raw == 15U) &&
              (snapshot.zero_samples == 64U) &&
              (snapshot.left_current_ma == 0UL) &&
              (snapshot.right_current_ma == 0UL) &&
              (snapshot.ready != 0U),
              "64 stopped samples establish ready zero");
}

static void test_peak_current_duration(void)
{
  power_supervisor_config_t config = test_config(1U);

  power_supervisor_init(&config);
  calibrate(10U, 10U);
  power_supervisor_step(1U, 1500U, 610U, 10U, 0U);
  power_supervisor_step(1U, 1500U, 610U, 10U, 0U);
  expect_true((power_supervisor_get_fault_mask() &
               POWER_FAULT_PEAK_CURRENT) == 0U,
              "peak current waits for configured duration");
  power_supervisor_step(1U, 1500U, 610U, 10U, 0U);
  expect_true((power_supervisor_get_fault_mask() &
               POWER_FAULT_PEAK_CURRENT) != 0U,
              "Nth peak sample latches fault");
}

static void test_voltage_hysteresis(void)
{
  power_supervisor_config_t config = test_config(1U);

  power_supervisor_init(&config);
  calibrate(10U, 10U);
  power_supervisor_step(1U, 900U, 10U, 10U, 0U);
  expect_true((power_supervisor_get_fault_mask() &
               POWER_FAULT_UNDERVOLTAGE) != 0U,
              "undervoltage latches");
  power_supervisor_step(1U, 1050U, 10U, 10U, 0U);
  expect_true(power_supervisor_clear_fault() == 0U,
              "UV does not clear inside hysteresis band");
  power_supervisor_step(1U, 1200U, 10U, 10U, 0U);
  expect_true(power_supervisor_clear_fault() != 0U,
              "UV clears above recovery threshold");
  expect_true(power_supervisor_is_ready() == 0U,
              "clear request cannot enable before a fresh sample");
  power_supervisor_step(1U, 1200U, 10U, 10U, 0U);

  power_supervisor_step(1U, 2100U, 10U, 10U, 0U);
  expect_true((power_supervisor_get_fault_mask() &
               POWER_FAULT_OVERVOLTAGE) != 0U,
              "overvoltage latches");
  power_supervisor_step(1U, 1950U, 10U, 10U, 0U);
  expect_true(power_supervisor_clear_fault() == 0U,
              "OV does not clear inside hysteresis band");
  power_supervisor_step(1U, 1900U, 10U, 10U, 0U);
  expect_true(power_supervisor_clear_fault() != 0U,
              "OV accepts clear below recovery threshold");
  power_supervisor_step(1U, 1900U, 10U, 10U, 0U);
  expect_true(power_supervisor_is_ready() != 0U,
              "OV clear is committed by a fresh safe sample");
}

static void test_adc_failure_count(void)
{
  power_supervisor_config_t config = test_config(1U);

  power_supervisor_init(&config);
  calibrate(10U, 10U);
  power_supervisor_step(0U, 0U, 0U, 0U, 0U);
  power_supervisor_step(0U, 0U, 0U, 0U, 0U);
  expect_true((power_supervisor_get_fault_mask() & POWER_FAULT_ADC) == 0U,
              "ADC tolerates fewer than N consecutive failures");
  power_supervisor_step(0U, 0U, 0U, 0U, 0U);
  expect_true((power_supervisor_get_fault_mask() & POWER_FAULT_ADC) != 0U,
              "Nth ADC failure latches fault");
  power_supervisor_step(1U, 1500U, 10U, 10U, 0U);
  expect_true(power_supervisor_clear_fault() != 0U,
              "valid sample permits ADC clear request");
  power_supervisor_step(1U, 1500U, 10U, 10U, 0U);
  expect_true(power_supervisor_is_ready() != 0U,
              "fresh ADC sample commits explicit clear");
}

int main(void)
{
  test_invalid_and_unapproved_config();
  test_units_and_zero_calibration();
  test_peak_current_duration();
  test_voltage_hysteresis();
  test_adc_failure_count();
  if(failures != 0)
  {
    printf("POWER_SUPERVISOR_TEST_FAIL,%d\n", failures);
    return 1;
  }
  puts("POWER_SUPERVISOR_TEST_PASS");
  return 0;
}
