/** @file adc_dma.h @brief ADC1三通道扫描DMA最新样本。 */
#ifndef ADC_DMA_H
#define ADC_DMA_H

#include <stdint.h>

typedef struct
{
  uint16_t bus_raw;
  uint16_t left_raw;
  uint16_t right_raw;
  uint32_t sequence;
} adc_dma_sample_t;

uint8_t adc_dma_init(void);
void adc_dma_start(void);
void adc_dma_irq(void);
uint8_t adc_dma_take(adc_dma_sample_t *sample);

#endif
