#include "power_supervisor.h"

#include <stddef.h>
#include <string.h>

#define POWER_ADC_MAX             4095U
#define POWER_ADC_UV           3300000ULL
#define POWER_CURRENT_DENOM (4095ULL * 100ULL * 10000ULL)
#define POWER_CURRENT_GAIN ((POWER_ADC_UV * 1000000ULL + POWER_CURRENT_DENOM / 2ULL) / POWER_CURRENT_DENOM)
#define POWER_ZERO_MAX ((150ULL * 100ULL * 4095ULL + POWER_ADC_UV - 1ULL) / POWER_ADC_UV)
#define POWER_BUS_GAIN ((POWER_ADC_UV * 51700ULL) / (4095ULL * 4700ULL))
#define POWER_BUS_CODE_MV ((POWER_BUS_GAIN + 999ULL) / 1000ULL)
#define POWER_READY             0x01UL
#define POWER_CAN_CLEAR         0x02UL
#define POWER_FAULT_SHIFT            8U

static const power_supervisor_config_t power_default =
{
  POWER_CURRENT_GAIN, POWER_CURRENT_GAIN, 0U, 0U, POWER_ZERO_MAX,
  POWER_BUS_GAIN, 0L, 18000UL, 19200UL,
  25200UL, 25800UL,
  800UL, 100U, 20U, 3U, 1U
};

static power_supervisor_config_t power_config;
static power_supervisor_snapshot_t power_data;
static uint32_t power_left_sum;
static uint32_t power_right_sum;
static uint32_t power_status;
static uint16_t power_peak_ticks;
static uint8_t power_adc_failures;
static uint8_t power_config_valid;
static uint8_t power_clear_requested;

static void power_publish(void)
{
  power_status = (power_data.ready != 0U ? POWER_READY : 0U) |
      (power_data.can_clear != 0U ? POWER_CAN_CLEAR : 0U) |
      ((uint32_t)power_data.fault_mask << POWER_FAULT_SHIFT);
}

static uint8_t power_config_ok(const power_supervisor_config_t *config)
{
  uint64_t left_max;
  uint64_t right_max;
  int64_t bus_max;

  if((config == NULL) || (config->left_current_ua_per_count == 0U) ||
     (config->right_current_ua_per_count == 0U) ||
     (config->bus_uv_per_count == 0U) ||
     (config->zero_max_raw > POWER_ZERO_MAX) ||
     (config->left_zero_raw > config->zero_max_raw) ||
     (config->right_zero_raw > config->zero_max_raw)) return 0U;
  if(((uint64_t)config->left_current_ua_per_count * POWER_ADC_MAX >
      (uint64_t)UINT32_MAX * 1000ULL) ||
     ((uint64_t)config->right_current_ua_per_count * POWER_ADC_MAX >
      (uint64_t)UINT32_MAX * 1000ULL) ||
     ((uint64_t)config->bus_uv_per_count * POWER_ADC_MAX >
      (uint64_t)UINT32_MAX * 1000ULL)) return 0U;
  left_max = (uint64_t)(POWER_ADC_MAX - config->zero_max_raw) *
      config->left_current_ua_per_count;
  right_max = (uint64_t)(POWER_ADC_MAX - config->zero_max_raw) *
      config->right_current_ua_per_count;
  bus_max = (int64_t)POWER_ADC_MAX * config->bus_uv_per_count +
      (int64_t)config->bus_offset_mv * 1000LL;
  if((left_max > (uint64_t)UINT32_MAX * 1000ULL) ||
     (right_max > (uint64_t)UINT32_MAX * 1000ULL) ||
     (bus_max > (int64_t)UINT32_MAX * 1000LL) ||
     (config->undervoltage_trip_mv == 0U) ||
     (config->undervoltage_recover_mv <= config->undervoltage_trip_mv) ||
     (config->overvoltage_recover_mv <= config->undervoltage_recover_mv) ||
     (config->overvoltage_trip_mv <= config->overvoltage_recover_mv) ||
     ((uint64_t)config->peak_current_ma * 1000ULL > left_max) ||
     ((uint64_t)config->peak_current_ma * 1000ULL > right_max) ||
     ((int64_t)config->overvoltage_trip_mv * 1000LL > bus_max) ||
     (config->peak_duration_ms == 0U) || (config->sample_period_ms == 0U) ||
     (config->adc_failure_limit == 0U) || (config->limits_approved > 1U))
    return 0U;
  return 1U;
}

const power_supervisor_config_t *power_supervisor_provisional_config(void)
{
  return &power_default;
}

static uint32_t power_current(uint16_t raw, uint16_t zero, uint32_t gain)
{
  return (raw > zero) ?
      (uint32_t)(((uint64_t)(raw - zero) * gain + 500ULL) / 1000ULL) : 0U;
}

static uint32_t power_bus(uint16_t raw)
{
  int64_t value = (int64_t)(((uint64_t)raw * power_config.bus_uv_per_count +
      500ULL) / 1000ULL) + power_config.bus_offset_mv;
  if(value <= 0) return 0U;
  return (value > UINT32_MAX) ? UINT32_MAX : (uint32_t)value;
}

static void power_reset_zero(void)
{
  power_left_sum = power_right_sum = 0U;
  power_data.zero_samples = 0U;
}

void power_supervisor_init(const power_supervisor_config_t *config)
{
  memset(&power_config, 0, sizeof(power_config));
  memset(&power_data, 0, sizeof(power_data));
  power_reset_zero();
  power_peak_ticks = power_adc_failures = power_clear_requested = 0U;
  power_config_valid = power_config_ok(config);
  if(power_config_valid != 0U)
  {
    power_config = *config;
    power_data.left_zero_raw = config->left_zero_raw;
    power_data.right_zero_raw = config->right_zero_raw;
    power_data.fault_mask = (config->limits_approved != 0U) ?
        0U : POWER_FAULT_NOT_APPROVED;
  }
  else
  {
    power_data.fault_mask = POWER_FAULT_CONFIG_INVALID;
  }
  power_publish();
}

void power_supervisor_step(uint8_t valid, uint16_t bus_raw,
                           uint16_t left_raw, uint16_t right_raw,
                           uint8_t zero_allowed)
{
  uint16_t peak_limit;
  uint8_t zero_ok;

  if(power_config_valid == 0U) return;
  if((valid == 0U) || (bus_raw > POWER_ADC_MAX) ||
     (left_raw > POWER_ADC_MAX) || (right_raw > POWER_ADC_MAX))
  {
    if(power_adc_failures < power_config.adc_failure_limit)
      power_adc_failures++;
    if(power_adc_failures >= power_config.adc_failure_limit)
    {
      power_data.fault_mask |= POWER_FAULT_ADC;
      power_data.ready = 0U;
    }
    if(power_data.zero_samples < POWER_SUPERVISOR_ZERO_SAMPLE_COUNT)
      power_reset_zero();
    power_data.can_clear = 0U;
    power_publish();
    return;
  }

  power_adc_failures = 0U;
  power_data.bus_mv = power_bus(bus_raw);
  power_data.left_current_ma = power_current(left_raw,
      power_data.left_zero_raw, power_config.left_current_ua_per_count);
  power_data.right_current_ma = power_current(right_raw,
      power_data.right_zero_raw, power_config.right_current_ua_per_count);
  zero_ok = ((zero_allowed != 0U) &&
      (left_raw <= power_config.zero_max_raw) &&
      (right_raw <= power_config.zero_max_raw) &&
      (power_data.bus_mv >= power_config.undervoltage_trip_mv) &&
      (power_data.bus_mv <= power_config.overvoltage_trip_mv));
  if((power_data.zero_samples < POWER_SUPERVISOR_ZERO_SAMPLE_COUNT) && zero_ok)
  {
    power_left_sum += left_raw;
    power_right_sum += right_raw;
    if(++power_data.zero_samples == POWER_SUPERVISOR_ZERO_SAMPLE_COUNT)
    {
      power_data.left_zero_raw = (uint16_t)(
          power_left_sum / POWER_SUPERVISOR_ZERO_SAMPLE_COUNT);
      power_data.right_zero_raw = (uint16_t)(
          power_right_sum / POWER_SUPERVISOR_ZERO_SAMPLE_COUNT);
      power_data.left_current_ma = power_data.right_current_ma = 0U;
    }
  }
  else if(power_data.zero_samples < POWER_SUPERVISOR_ZERO_SAMPLE_COUNT)
  {
    power_reset_zero();
  }

  if(power_data.bus_mv < power_config.undervoltage_trip_mv)
    power_data.fault_mask |= POWER_FAULT_UNDERVOLTAGE;
  if(power_data.bus_mv > power_config.overvoltage_trip_mv)
    power_data.fault_mask |= POWER_FAULT_OVERVOLTAGE;
  peak_limit = (power_config.peak_duration_ms +
      power_config.sample_period_ms - 1U) / power_config.sample_period_ms;
  if((power_data.left_current_ma >= power_config.peak_current_ma) ||
     (power_data.right_current_ma >= power_config.peak_current_ma))
  {
    if(power_peak_ticks < peak_limit) power_peak_ticks++;
  }
  else
  {
    power_peak_ticks = 0U;
  }
  if(power_peak_ticks >= peak_limit)
    power_data.fault_mask |= POWER_FAULT_PEAK_CURRENT;
  power_data.can_clear = (((power_data.fault_mask & POWER_FAULT_SEVERE_MASK) != 0U) &&
      (power_data.bus_mv >= power_config.undervoltage_recover_mv) &&
      (power_data.bus_mv <= power_config.overvoltage_recover_mv) &&
      (power_data.left_current_ma < power_config.peak_current_ma) &&
      (power_data.right_current_ma < power_config.peak_current_ma));
  if(power_clear_requested != 0U)
  {
    if(power_data.can_clear != 0U)
    {
      power_data.fault_mask &= POWER_FAULT_NOT_APPROVED;
      power_peak_ticks = 0U;
    }
    power_clear_requested = 0U;
    power_data.can_clear = 0U;
  }
  power_data.ready = ((power_data.zero_samples == POWER_SUPERVISOR_ZERO_SAMPLE_COUNT) &&
                      (power_data.fault_mask == 0U));
  power_publish();
}

uint8_t power_supervisor_is_ready(void) { return (power_status & POWER_READY) != 0U; }
uint8_t power_supervisor_get_fault_mask(void)
{
  return (uint8_t)(power_status >> POWER_FAULT_SHIFT);
}

uint8_t power_supervisor_clear_fault(void)
{
  if(((power_supervisor_get_fault_mask() & POWER_FAULT_SEVERE_MASK) != 0U) &&
     ((power_status & POWER_CAN_CLEAR) == 0U)) return 0U;
  power_clear_requested = 1U;
  return 1U;
}

void power_supervisor_get_snapshot(power_supervisor_snapshot_t *snapshot)
{
  if(snapshot != NULL) *snapshot = power_data;
}
