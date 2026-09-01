/**
 * @file safety.c
 * @brief 简洁的电压、电流和本地故障锁存；故障只能显式清除。
 */
#include "safety.h"

#include "board_config.h"

static safety_status_t safety_status;
static uint16_t safety_left_zero;
static uint16_t safety_right_zero;
static uint32_t safety_left_sum;
static uint32_t safety_right_sum;
static uint32_t safety_last_sample_ms;
static uint32_t safety_last_sequence;
static uint8_t safety_zero_count;
static uint8_t safety_peak_ticks;

static uint16_t safety_u16(uint32_t value)
{
  return (value > 65535UL) ? 65535U : (uint16_t)value;
}

static uint16_t safety_current(uint16_t raw, uint16_t zero)
{
  return (raw > zero) ? safety_u16(((uint32_t)(raw - zero) *
      BOARD_CURRENT_UA_PER_COUNT) / 1000UL) : 0U;
}

void safety_init(void)
{
  safety_status.bus_mv = 0U;
  safety_status.left_current_ma = 0U;
  safety_status.right_current_ma = 0U;
  safety_status.fault = SAFETY_FAULT_NONE;
  safety_status.ready = 0U;
  safety_left_zero = safety_right_zero = 0U;
  safety_left_sum = safety_right_sum = 0UL;
  safety_last_sequence = safety_last_sample_ms = 0UL;
  safety_zero_count = safety_peak_ticks = 0U;
}

void safety_update(uint32_t now_ms, const adc_dma_sample_t *sample,
                   uint8_t zero_allowed)
{
  uint8_t zero_sample;

  if((sample == 0) || (sample->sequence == 0U))
  {
    if(((safety_last_sequence == 0U) && (now_ms >= BOARD_ADC_STALE_MS)) ||
       ((safety_last_sequence != 0U) &&
        ((now_ms - safety_last_sample_ms) >= BOARD_ADC_STALE_MS)))
      safety_raise(SAFETY_FAULT_POWER);
    return;
  }
  if(sample->sequence == safety_last_sequence)
  {
    if((now_ms - safety_last_sample_ms) >= BOARD_ADC_STALE_MS)
      safety_raise(SAFETY_FAULT_POWER);
    return;
  }
  safety_last_sequence = sample->sequence;
  safety_last_sample_ms = now_ms;
  safety_status.bus_mv = safety_u16(((uint32_t)sample->bus_raw *
      BOARD_BUS_UV_PER_COUNT) / 1000UL + BOARD_BUS_OFFSET_MV);
  zero_sample = ((zero_allowed != 0U) &&
      (sample->left_raw <= BOARD_ADC_ZERO_MAX_RAW) &&
      (sample->right_raw <= BOARD_ADC_ZERO_MAX_RAW) &&
      (safety_status.bus_mv >= BOARD_BUS_UV_TRIP_MV) &&
      (safety_status.bus_mv <= BOARD_BUS_OV_TRIP_MV)) ? 1U : 0U;
  if((safety_zero_count < BOARD_ADC_ZERO_SAMPLES) && (zero_sample != 0U))
  {
    safety_left_sum += sample->left_raw;
    safety_right_sum += sample->right_raw;
    if(++safety_zero_count == BOARD_ADC_ZERO_SAMPLES)
    {
      safety_left_zero = (uint16_t)(safety_left_sum / BOARD_ADC_ZERO_SAMPLES);
      safety_right_zero = (uint16_t)(safety_right_sum / BOARD_ADC_ZERO_SAMPLES);
    }
  }
  else if(safety_zero_count < BOARD_ADC_ZERO_SAMPLES)
  {
    safety_left_sum = safety_right_sum = 0UL;
    safety_zero_count = 0U;
  }
  safety_status.left_current_ma = safety_current(sample->left_raw, safety_left_zero);
  safety_status.right_current_ma = safety_current(sample->right_raw, safety_right_zero);
  if((safety_status.bus_mv < BOARD_BUS_UV_TRIP_MV) ||
     (safety_status.bus_mv > BOARD_BUS_OV_TRIP_MV)) safety_raise(SAFETY_FAULT_POWER);
  if((safety_status.left_current_ma >= BOARD_PEAK_CURRENT_MA) ||
     (safety_status.right_current_ma >= BOARD_PEAK_CURRENT_MA))
  {
    if(++safety_peak_ticks >= BOARD_PEAK_CURRENT_TICKS) safety_raise(SAFETY_FAULT_POWER);
  }
  else safety_peak_ticks = 0U;
  safety_status.ready = ((safety_zero_count == BOARD_ADC_ZERO_SAMPLES) &&
                         (safety_status.fault == SAFETY_FAULT_NONE)) ? 1U : 0U;
}

void safety_raise(uint8_t fault)
{
  safety_status.fault |= fault;
  safety_status.ready = 0U;
}

uint8_t safety_can_enable(void)
{
  return safety_status.ready;
}

uint8_t safety_fault(void)
{
  return safety_status.fault;
}

uint8_t safety_clear(void)
{
  if((safety_status.bus_mv < BOARD_BUS_UV_RECOVER_MV) ||
     (safety_status.bus_mv > BOARD_BUS_OV_RECOVER_MV) ||
     (safety_status.left_current_ma >= BOARD_PEAK_CURRENT_MA) ||
     (safety_status.right_current_ma >= BOARD_PEAK_CURRENT_MA)) return 0U;
  safety_status.fault = SAFETY_FAULT_NONE;
  safety_status.ready = (safety_zero_count == BOARD_ADC_ZERO_SAMPLES) ? 1U : 0U;
  return 1U;
}

void safety_get(safety_status_t *status)
{
  if(status != 0) *status = safety_status;
}
