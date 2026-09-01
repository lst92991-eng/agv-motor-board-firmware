/**
 * @file power_supervisor.c
 * @brief 20ms母线/双电流换算、零点门和锁存保护状态机。
 *
 * 本模块不访问ADC、PWM或RTOS，也不构成电流环。它只消费一组三通道快照，
 * 产生允许ENABLE的ready状态和必须由底盘执行的锁存故障。
 */

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

/** @brief 将ready、可清除和故障位压缩为跨任务32位状态字。 */
static void power_publish(void)
{
  /* 32位对齐快照供跨任务读取；多字段诊断仍通过显式snapshot接口复制。 */
  power_status = (power_data.ready != 0U ? POWER_READY : 0U) |
      (power_data.can_clear != 0U ? POWER_CAN_CLEAR : 0U) |
      ((uint32_t)power_data.fault_mask << POWER_FAULT_SHIFT);
}

/** @brief 检查换算量程、迟滞关系、持续时间和批准标志是否合法。 */
static uint8_t power_config_ok(const power_supervisor_config_t *config)
{
  uint64_t left_max;
  uint64_t right_max;
  int64_t bus_max;

  /* 阈值必须能由12位ADC量程实际观测，禁止不可达保护制造虚假安全感。 */
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

/** @brief 返回当前生产电气配置；函数名保留历史兼容性。 */
const power_supervisor_config_t *power_supervisor_provisional_config(void)
{
  /* API名称沿用历史；当前返回生产动态验收配置，后续清洗可统一改名。 */
  return &power_default;
}

/** @brief 按零点和uA/count增益把ADC码换算成非负mA。 */
static uint32_t power_current(uint16_t raw, uint16_t zero, uint32_t gain)
{
  return (raw > zero) ?
      (uint32_t)(((uint64_t)(raw - zero) * gain + 500ULL) / 1000ULL) : 0U;
}

/** @brief 按uV/count和偏置把母线ADC码换算成mV。 */
static uint32_t power_bus(uint16_t raw)
{
  int64_t value = (int64_t)(((uint64_t)raw * power_config.bus_uv_per_count +
      500ULL) / 1000ULL) + power_config.bus_offset_mv;
  if(value <= 0) return 0U;
  return (value > UINT32_MAX) ? UINT32_MAX : (uint32_t)value;
}

/** @brief 清空尚未完成的静止零点累计窗口。 */
static void power_reset_zero(void)
{
  power_left_sum = power_right_sum = 0U;
  power_data.zero_samples = 0U;
}

/** @brief 初始化电气状态机并以配置非法/未批准方式失败关闭。 */
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

/** @brief 消费一组三通道样本并更新零点、故障迟滞和ready。 */
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
    /* 连续N次无效才锁存；无效周期绝不能更新零点或重新形成ready。 */
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
    /* 零点只在底盘禁用、PWM为零、编码器无变化且母线正常时累计。 */
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
  /* 清故障分两阶段：本周期先证明电压/电流恢复，下周期再重新形成ready。 */
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

/** @brief 返回零点完成且无任何故障时的ENABLE准入状态。 */
uint8_t power_supervisor_is_ready(void) { return (power_status & POWER_READY) != 0U; }
/** @brief 返回当前锁存电气故障位图。 */
uint8_t power_supervisor_get_fault_mask(void)
{
  return (uint8_t)(power_status >> POWER_FAULT_SHIFT);
}

/** @brief 在迟滞恢复条件满足时提交下一周期清故障请求。 */
uint8_t power_supervisor_clear_fault(void)
{
  /* 请求只排队，不在命令上下文直接清除保护位。 */
  if(((power_supervisor_get_fault_mask() & POWER_FAULT_SEVERE_MASK) != 0U) &&
     ((power_status & POWER_CAN_CLEAR) == 0U)) return 0U;
  power_clear_requested = 1U;
  return 1U;
}

/** @brief 复制最近一次电气换算和保护状态。 */
void power_supervisor_get_snapshot(power_supervisor_snapshot_t *snapshot)
{
  if(snapshot != NULL) *snapshot = power_data;
}
