/**
 * @file adc_dma.c
 * @brief ADC1扫描PA3/PA4/PA5，并由DMA一次搬运完整三通道样本。
 */
#include "adc_dma.h"

#include "at32f45x.h"
#include "at32f45x_adc.h"
#include "at32f45x_crm.h"
#include "at32f45x_dma.h"
#include "at32f45x_gpio.h"
#include "at32f45x_misc.h"

static volatile uint16_t adc_dma_raw[3];
static volatile uint32_t adc_dma_sequence;
static volatile uint8_t adc_dma_busy;
static uint8_t adc_dma_ready;

/* AT32 ADC标志函数不能直接作为回调；初始化阶段使用显式有界轮询。 */
static uint8_t adc_dma_calibrate(void)
{
  uint32_t limit = 100000UL;
  while((adc_flag_get(ADC1, ADC_RDY_FLAG) == RESET) && (limit-- > 0U)) {}
  if(limit == 0U) return 0U;
  adc_calibration_init(ADC1);
  limit = 100000UL;
  while(adc_calibration_init_status_get(ADC1) && (limit-- > 0U)) {}
  if(limit == 0U) return 0U;
  adc_calibration_start(ADC1);
  limit = 100000UL;
  while(adc_calibration_status_get(ADC1) && (limit-- > 0U)) {}
  return (limit > 0U) ? 1U : 0U;
}

uint8_t adc_dma_init(void)
{
  gpio_init_type gpio;
  adc_common_config_type common;
  adc_base_config_type base;
  dma_init_type dma;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio);
  gpio.gpio_mode = GPIO_MODE_ANALOG;
  gpio.gpio_pins = GPIO_PINS_3 | GPIO_PINS_4 | GPIO_PINS_5;
  gpio_init(GPIOA, &gpio);
  adc_reset();
  adc_common_default_para_init(&common);
  common.combine_mode = ADC_INDEPENDENT_MODE;
  common.div = ADC_HCLK_DIV_6;
  common.common_dma_mode = ADC_COMMON_DMAMODE_DISABLE;
  adc_common_config(&common);
  adc_base_default_para_init(&base);
  base.sequence_mode = TRUE;
  base.repeat_mode = FALSE;
  base.data_align = ADC_RIGHT_ALIGNMENT;
  base.ordinary_channel_length = 3U;
  adc_base_config(ADC1, &base);
  adc_resolution_set(ADC1, ADC_RESOLUTION_12B);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_3, 1U, ADC_SAMPLETIME_92_5);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_4, 2U, ADC_SAMPLETIME_92_5);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_5, 3U, ADC_SAMPLETIME_92_5);
  adc_ordinary_conversion_trigger_set(ADC1, ADC_ORDINARY_TRIG_TMR1CH1,
                                      ADC_ORDINARY_TRIG_EDGE_NONE);
  dma_reset(DMA1_CHANNEL1);
  dma_default_para_init(&dma);
  dma.peripheral_base_addr = (uint32_t)&ADC1->odt;
  dma.memory_base_addr = (uint32_t)adc_dma_raw;
  dma.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  dma.buffer_size = 3U;
  dma.peripheral_inc_enable = FALSE;
  dma.memory_inc_enable = TRUE;
  dma.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  dma.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  dma.loop_mode_enable = TRUE;
  dma.priority = DMA_PRIORITY_HIGH;
  dma_init(DMA1_CHANNEL1, &dma);
  dmamux_enable(DMA1, TRUE);
  dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_ADC1);
  dma_flag_clear(DMA1_GL1_FLAG);
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT | DMA_DTERR_INT, TRUE);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);
  adc_dma_mode_enable(ADC1, TRUE);
  adc_dma_request_repeat_enable(ADC1, TRUE);
  adc_enable(ADC1, TRUE);
  if(adc_dma_calibrate() == 0U) return 0U;
  adc_dma_sequence = 0U;
  adc_dma_busy = 0U;
  adc_dma_ready = 1U;
  nvic_irq_enable(DMA1_Channel1_IRQn, 2U, 3U);
  return 1U;
}

void adc_dma_start(void)
{
  if((adc_dma_ready == 0U) || (adc_dma_busy != 0U)) return;
  adc_dma_busy = 1U;
  adc_ordinary_software_trigger_enable(ADC1, TRUE);
}

void adc_dma_irq(void)
{
  if(dma_flag_get(DMA1_DTERR1_FLAG) != RESET)
  {
    dma_flag_clear(DMA1_GL1_FLAG);
    adc_dma_busy = 0U;
    adc_dma_ready = 0U;
    return;
  }
  if(dma_flag_get(DMA1_FDT1_FLAG) != RESET)
  {
    dma_flag_clear(DMA1_GL1_FLAG);
    adc_dma_sequence++;
    adc_dma_busy = 0U;
  }
}

uint8_t adc_dma_take(adc_dma_sample_t *sample)
{
  uint32_t irq;
  if((sample == 0) || (adc_dma_ready == 0U) || (adc_dma_sequence == 0U)) return 0U;
  irq = __get_PRIMASK();
  __disable_irq();
  sample->bus_raw = adc_dma_raw[0];
  sample->left_raw = adc_dma_raw[1];
  sample->right_raw = adc_dma_raw[2];
  sample->sequence = adc_dma_sequence;
  if(irq == 0U) __enable_irq();
  return 1U;
}
