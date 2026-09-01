/** @file safety.h @brief 车辆本地故障、ADC样本和ENABLE准入。 */
#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>

#include "adc_dma.h"

enum
{
  SAFETY_FAULT_NONE = 0U,
  SAFETY_FAULT_BUMPER = 0x01U,
  SAFETY_FAULT_ESTOP = 0x02U,
  SAFETY_FAULT_TIMEOUT = 0x04U,
  SAFETY_FAULT_CAN = 0x08U,
  SAFETY_FAULT_IMU = 0x10U,
  SAFETY_FAULT_DRIVE = 0x20U,
  SAFETY_FAULT_POWER = 0x40U
};

typedef struct
{
  uint16_t bus_mv;
  uint16_t left_current_ma;
  uint16_t right_current_ma;
  uint8_t fault;
  uint8_t ready;
} safety_status_t;

void safety_init(void);
void safety_update(uint32_t now_ms, const adc_dma_sample_t *sample,
                   uint8_t zero_allowed);
void safety_raise(uint8_t fault);
uint8_t safety_can_enable(void);
uint8_t safety_fault(void);
uint8_t safety_clear(void);
void safety_get(safety_status_t *status);

#endif
