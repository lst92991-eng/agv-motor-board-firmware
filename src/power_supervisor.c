/**
  *****************************************************************************
  * @file     power_supervisor.c
  * @brief    母线与双路电流的低速电气监督状态机
  *
  * 本模块不是电流环或转矩环，只锁存ADC失效和绝对电气边界；故障满足
  * 迟滞恢复条件后，仍需CLEAR_FAULT显式清除。
  *****************************************************************************
  */

#include "power_supervisor.h"

#include <stddef.h>
#include <string.h>

#define POWER_ADC_MAX_RAW                    4095U
#define POWER_ADC_REFERENCE_UV          3300000ULL
#define POWER_CURRENT_DENOM (4095ULL * 100ULL * 10000ULL)
#define POWER_CURRENT_UA_PER_COUNT ((POWER_ADC_REFERENCE_UV * 1000000ULL + \
                                    POWER_CURRENT_DENOM / 2ULL) / \
                                   POWER_CURRENT_DENOM)
/* 低侧VCM=0时INA180最大输入偏移150uV，乘A3后向上取整为19码。 */
#define POWER_ZERO_MAX_RAW ((150ULL * 100ULL * 4095ULL + POWER_ADC_REFERENCE_UV - 1ULL) / \
                           POWER_ADC_REFERENCE_UV)
#define POWER_BUS_UV_PER_COUNT ((POWER_ADC_REFERENCE_UV * 51700ULL) / \
                                (4095ULL * 4700ULL))
#define POWER_BUS_ONE_CODE_MV ((POWER_BUS_UV_PER_COUNT + 999ULL) / 1000ULL)
#define POWER_STATUS_READY_BIT                  0x00000001UL
#define POWER_STATUS_CAN_CLEAR_BIT              0x00000002UL
#define POWER_STATUS_FAULT_SHIFT                         8U

/* 33V取母线TVS standoff，低于35V电容与AT8236 36V推荐上限；仍未批准。 */
static const power_supervisor_config_t power_provisional_config =
{
  (uint32_t)POWER_CURRENT_UA_PER_COUNT, (uint32_t)POWER_CURRENT_UA_PER_COUNT,
  0U, 0U, (uint16_t)POWER_ZERO_MAX_RAW,
  (uint32_t)POWER_BUS_UV_PER_COUNT, 0L,
  5500UL, 5800UL, (uint32_t)(33000ULL - POWER_BUS_ONE_CODE_MV), 33000UL,
  3000UL, 20U, 20U, 3U, 0U
};

static power_supervisor_config_t power_config;
static power_supervisor_snapshot_t power_data;
static uint32_t power_left_zero_sum, power_right_zero_sum;
static uint16_t power_peak_ticks;
static uint8_t power_adc_failures, power_config_valid;
static volatile uint32_t power_status;
static volatile uint8_t power_clear_requested;

static void power_publish(void)
{
  power_status = ((power_data.ready != 0U) ? POWER_STATUS_READY_BIT : 0UL) |
      ((power_data.can_clear != 0U) ? POWER_STATUS_CAN_CLEAR_BIT : 0UL) |
      ((uint32_t)power_data.fault_mask << POWER_STATUS_FAULT_SHIFT);
}

static uint8_t power_config_ok(const power_supervisor_config_t *config)
{
  return ((config != NULL) &&
          (config->left_current_ua_per_count != 0UL) && (config->right_current_ua_per_count != 0UL) &&
          (((uint64_t)config->left_current_ua_per_count * POWER_ADC_MAX_RAW) <= ((uint64_t)UINT32_MAX * 1000ULL)) &&
          (((uint64_t)config->right_current_ua_per_count * POWER_ADC_MAX_RAW) <= ((uint64_t)UINT32_MAX * 1000ULL)) &&
          (config->left_zero_raw <= config->zero_max_raw) && (config->right_zero_raw <= config->zero_max_raw) &&
          (config->zero_max_raw <= POWER_ZERO_MAX_RAW) &&
          (config->bus_uv_per_count != 0UL) &&
          (((uint64_t)config->bus_uv_per_count * POWER_ADC_MAX_RAW) <=
           ((uint64_t)UINT32_MAX * 1000ULL)) &&
          (config->undervoltage_trip_mv > 0UL) &&
          (config->undervoltage_recover_mv > config->undervoltage_trip_mv) &&
          (config->overvoltage_recover_mv > config->undervoltage_recover_mv) &&
          (config->overvoltage_trip_mv > config->overvoltage_recover_mv) &&
          (config->peak_current_ma > 0UL) &&
          ((uint64_t)config->peak_current_ma * 1000ULL <= (uint64_t)(POWER_ADC_MAX_RAW - config->zero_max_raw) * config->left_current_ua_per_count) &&
          ((uint64_t)config->peak_current_ma * 1000ULL <= (uint64_t)(POWER_ADC_MAX_RAW - config->zero_max_raw) * config->right_current_ua_per_count) &&
          ((int64_t)config->overvoltage_trip_mv * 1000LL <= (int64_t)config->bus_uv_per_count * POWER_ADC_MAX_RAW + (int64_t)config->bus_offset_mv * 1000LL) &&
          (config->peak_duration_ms > 0U) && (config->sample_period_ms > 0U) &&
          (config->adc_failure_limit > 0U) && (config->limits_approved <= 1U)) ? 1U : 0U;
}

const power_supervisor_config_t *power_supervisor_provisional_config(void)
{ return &power_provisional_config; }

static uint32_t power_to_current_ma(uint16_t raw, uint16_t zero, uint32_t ua_per_count)
{
  return (raw > zero) ? (uint32_t)(((uint64_t)(raw - zero) *
      ua_per_count + 500ULL) / 1000ULL) : 0UL;
}

static uint32_t power_to_bus_mv(uint16_t raw)
{
  int64_t value = (int64_t)(((uint64_t)raw *
      power_config.bus_uv_per_count + 500ULL) / 1000ULL) +
      power_config.bus_offset_mv;
  if(value <= 0)
  { return 0UL; }
  return (value > (int64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)value;
}

static void power_reset_zero(void)
{
  power_left_zero_sum = 0UL; power_right_zero_sum = 0UL;
  power_data.zero_samples = 0U;
}

static void power_update_recovery(void)
{
  power_data.can_clear =
      (((power_data.fault_mask & POWER_FAULT_SEVERE_MASK) != 0U) &&
       (power_adc_failures == 0U) &&
       (power_data.bus_mv >= power_config.undervoltage_recover_mv) &&
       (power_data.bus_mv <= power_config.overvoltage_recover_mv) &&
       (power_data.left_current_ma < power_config.peak_current_ma) &&
       (power_data.right_current_ma < power_config.peak_current_ma)) ? 1U : 0U;
}

void power_supervisor_init(const power_supervisor_config_t *config)
{
  memset(&power_config, 0, sizeof(power_config));
  memset(&power_data, 0, sizeof(power_data));
  power_reset_zero();
  power_peak_ticks = 0U; power_adc_failures = 0U;
  power_clear_requested = 0U;
  power_config_valid = power_config_ok(config);
  if(power_config_valid == 0U)
  {
    power_data.fault_mask = POWER_FAULT_CONFIG_INVALID;
    power_publish();
    return;
  }
  power_config = *config;
  power_data.left_zero_raw = config->left_zero_raw;
  power_data.right_zero_raw = config->right_zero_raw;
  power_data.fault_mask = (config->limits_approved != 0U) ?
                          0U : POWER_FAULT_NOT_APPROVED;
  power_publish();
}

void power_supervisor_step(uint8_t adc_valid, uint16_t bus_raw,
                           uint16_t left_raw, uint16_t right_raw,
                           uint8_t zero_calibration_allowed)
{
  uint16_t peak_limit;
  uint8_t zero_sample_ok;

  if(power_config_valid == 0U)
  {
    return;
  }
  if((adc_valid == 0U) || (bus_raw > POWER_ADC_MAX_RAW) ||
     (left_raw > POWER_ADC_MAX_RAW) || (right_raw > POWER_ADC_MAX_RAW))
  {
    if(power_adc_failures < power_config.adc_failure_limit)
    { power_adc_failures++; }
    if(power_adc_failures >= power_config.adc_failure_limit)
    {
      power_data.fault_mask |= POWER_FAULT_ADC;
      power_data.ready = 0U;
    }
    if(power_data.zero_samples < POWER_SUPERVISOR_ZERO_SAMPLE_COUNT)
    { power_reset_zero(); }
    power_data.can_clear = 0U;
    power_publish();
    return;
  }

  power_adc_failures = 0U;
  power_data.bus_mv = power_to_bus_mv(bus_raw);
  power_data.left_current_ma = power_to_current_ma(
      left_raw, power_data.left_zero_raw,
      power_config.left_current_ua_per_count);
  power_data.right_current_ma = power_to_current_ma(
      right_raw, power_data.right_zero_raw,
      power_config.right_current_ua_per_count);

  zero_sample_ok = ((zero_calibration_allowed != 0U) &&
      (left_raw <= power_config.zero_max_raw) &&
      (right_raw <= power_config.zero_max_raw) &&
      (power_data.bus_mv >= power_config.undervoltage_trip_mv) &&
      (power_data.bus_mv <= power_config.overvoltage_trip_mv)) ? 1U : 0U;
  if((power_data.zero_samples < POWER_SUPERVISOR_ZERO_SAMPLE_COUNT) &&
     (zero_sample_ok != 0U))
  {
    power_left_zero_sum += left_raw;
    power_right_zero_sum += right_raw;
    power_data.zero_samples++;
    if(power_data.zero_samples == POWER_SUPERVISOR_ZERO_SAMPLE_COUNT)
    {
      power_data.left_zero_raw = (uint16_t)(power_left_zero_sum /
          POWER_SUPERVISOR_ZERO_SAMPLE_COUNT);
      power_data.right_zero_raw = (uint16_t)(power_right_zero_sum /
          POWER_SUPERVISOR_ZERO_SAMPLE_COUNT);
      power_data.left_current_ma = 0UL;
      power_data.right_current_ma = 0UL;
    }
  }
  else if(power_data.zero_samples < POWER_SUPERVISOR_ZERO_SAMPLE_COUNT)
  { power_reset_zero(); }

  if(power_data.bus_mv < power_config.undervoltage_trip_mv)
  { power_data.fault_mask |= POWER_FAULT_UNDERVOLTAGE; }
  if(power_data.bus_mv > power_config.overvoltage_trip_mv)
  { power_data.fault_mask |= POWER_FAULT_OVERVOLTAGE; }
  peak_limit = (uint16_t)((power_config.peak_duration_ms +
      power_config.sample_period_ms - 1U) / power_config.sample_period_ms);
  if((power_data.left_current_ma >= power_config.peak_current_ma) ||
     (power_data.right_current_ma >= power_config.peak_current_ma))
  {
    if(power_peak_ticks < peak_limit)
    { power_peak_ticks++; }
  }
  else
  {
    power_peak_ticks = 0U;
  }
  if(power_peak_ticks >= peak_limit)
  { power_data.fault_mask |= POWER_FAULT_PEAK_CURRENT; }
  power_update_recovery();
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
  power_data.ready =
      ((power_data.zero_samples == POWER_SUPERVISOR_ZERO_SAMPLE_COUNT) &&
       (power_data.fault_mask == 0U)) ? 1U : 0U;
  power_publish();
}

uint8_t power_supervisor_is_ready(void)
{ return ((power_status & POWER_STATUS_READY_BIT) != 0UL) ? 1U : 0U; }

uint8_t power_supervisor_get_fault_mask(void)
{ return (uint8_t)(power_status >> POWER_STATUS_FAULT_SHIFT); }

uint8_t power_supervisor_clear_fault(void)
{
  if(((power_supervisor_get_fault_mask() & POWER_FAULT_SEVERE_MASK) != 0U) &&
     ((power_status & POWER_STATUS_CAN_CLEAR_BIT) == 0UL))
  {
    return 0U;
  }
  power_clear_requested = 1U;
  return 1U;
}

void power_supervisor_get_snapshot(power_supervisor_snapshot_t *snapshot)
{
  if(snapshot != NULL)
  { *snapshot = power_data; }
}
