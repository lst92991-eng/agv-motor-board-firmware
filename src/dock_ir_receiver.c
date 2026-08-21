/**
  ****************************************************************************
  * @file     dock_ir_receiver.c
  * @brief    充电基站38kHz红外接收头硬件接口
  ****************************************************************************
  */

#include "dock_ir_receiver.h"

#include "at32f45x.h"
#include "at32f45x_crm.h"
#include "at32f45x_exint.h"
#include "at32f45x_gpio.h"
#include "at32f45x_misc.h"
#include "at32f45x_scfg.h"

#define DOCK_IR_PORT GPIOA
#define DOCK_IR_PIN  GPIO_PINS_8
#define DOCK_IR_WINDOW_SAMPLES             50U
#define DOCK_IR_DETECT_MIN_ACTIVE_SAMPLES   2U

static uint8_t dock_ir_window_samples;
static uint8_t dock_ir_active_samples;
static volatile uint8_t dock_ir_activity;
static volatile uint8_t dock_ir_activity_percent;
static volatile uint16_t dock_ir_falling_edges;
static volatile uint16_t dock_ir_last_window_edges;

void dock_ir_receiver_init(void)
{
  exint_init_type exint_config;
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = DOCK_IR_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(DOCK_IR_PORT, &gpio_init_struct);

  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE8);
  exint_default_para_init(&exint_config);
  exint_config.line_enable = TRUE;
  exint_config.line_mode = EXINT_LINE_INTERRUPT;
  exint_config.line_select = EXINT_LINE_8;
  exint_config.line_polarity = EXINT_TRIGGER_FALLING_EDGE;
  exint_init(&exint_config);
  exint_flag_clear(EXINT_LINE_8);

  dock_ir_window_samples = 0U;
  dock_ir_active_samples = 0U;
  dock_ir_activity = 0U;
  dock_ir_activity_percent = 0U;
  dock_ir_falling_edges = 0U;
  dock_ir_last_window_edges = 0U;

  /* EXINT8与编码器、防撞右侧共享入口和相同中断优先级。 */
  nvic_irq_enable(EXINT9_5_IRQn, 2U, 1U);
}

uint8_t dock_ir_receiver_get_raw_level(void)
{
  return (gpio_input_data_bit_read(DOCK_IR_PORT, DOCK_IR_PIN) != RESET) ?
         1U : 0U;
}

uint8_t dock_ir_receiver_is_carrier_active(void)
{
  return (dock_ir_receiver_get_raw_level() == 0U) ? 1U : 0U;
}

void dock_ir_receiver_sample_2ms(void)
{
  uint16_t falling_edges;
  uint32_t primask;

  if(dock_ir_receiver_is_carrier_active() != 0U)
  {
    if(dock_ir_active_samples < DOCK_IR_WINDOW_SAMPLES)
    {
      dock_ir_active_samples++;
    }
  }
  dock_ir_window_samples++;

  if(dock_ir_window_samples >= DOCK_IR_WINDOW_SAMPLES)
  {
    primask = __get_PRIMASK();
    __disable_irq();
    falling_edges = dock_ir_falling_edges;
    dock_ir_falling_edges = 0U;
    if(primask == 0U)
    {
      __enable_irq();
    }

    dock_ir_last_window_edges = falling_edges;
    dock_ir_activity_percent = (uint8_t)(
        ((uint16_t)dock_ir_active_samples * 100U +
         (DOCK_IR_WINDOW_SAMPLES / 2U)) /
        DOCK_IR_WINDOW_SAMPLES);
    dock_ir_activity =
        ((dock_ir_active_samples >= DOCK_IR_DETECT_MIN_ACTIVE_SAMPLES) ||
         (falling_edges >= DOCK_IR_DETECT_MIN_ACTIVE_SAMPLES)) ?
        1U : 0U;
    dock_ir_window_samples = 0U;
    dock_ir_active_samples = 0U;
  }
}

uint8_t dock_ir_receiver_get_activity(void)
{
  return dock_ir_activity;
}

uint8_t dock_ir_receiver_get_activity_percent(void)
{
  return dock_ir_activity_percent;
}

uint16_t dock_ir_receiver_get_falling_edges(void)
{
  return dock_ir_last_window_edges;
}

void dock_ir_receiver_falling_edge_irq_handler(void)
{
  if(dock_ir_falling_edges < UINT16_MAX)
  {
    dock_ir_falling_edges++;
  }
}
