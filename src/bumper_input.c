#include "bumper_input.h"

#include "at32f45x.h"
#include "at32f45x_crm.h"
#include "at32f45x_exint.h"
#include "at32f45x_gpio.h"
#include "at32f45x_misc.h"
#include "at32f45x_scfg.h"

static volatile uint8_t bumper_pressed;

static void bumper_line(uint32_t line)
{
  exint_init_type config;
  exint_default_para_init(&config);
  config.line_enable = TRUE;
  config.line_mode = EXINT_LINE_INTERRUPT;
  config.line_select = line;
  config.line_polarity = EXINT_TRIGGER_BOTH_EDGE;
  exint_init(&config);
  exint_flag_clear(line);
}

void bumper_input_init(void)
{
  gpio_init_type gpio;

  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio);
  gpio.gpio_mode = GPIO_MODE_INPUT;
  gpio.gpio_pull = GPIO_PULL_NONE;
  gpio.gpio_pins = GPIO_PINS_9;
  gpio_init(GPIOB, &gpio);
  gpio.gpio_pins = GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15;
  gpio_init(GPIOC, &gpio);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOB, SCFG_PINS_SOURCE9);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE13);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE14);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE15);
  bumper_line(EXINT_LINE_9);
  bumper_line(EXINT_LINE_13);
  bumper_line(EXINT_LINE_14);
  bumper_line(EXINT_LINE_15);
  bumper_pressed = 0U;
  nvic_irq_enable(EXINT9_5_IRQn, 2U, 1U);
  nvic_irq_enable(EXINT15_10_IRQn, 2U, 2U);
}

uint8_t bumper_input_read_active_mask(void)
{
  uint8_t mask = 0U;
  if(gpio_input_data_bit_read(GPIOB, GPIO_PINS_9) == RESET)
    mask |= BUMPER_INPUT_RIGHT_MASK;
  if(gpio_input_data_bit_read(GPIOC, GPIO_PINS_13) == RESET)
    mask |= BUMPER_INPUT_REAR_MASK;
  if(gpio_input_data_bit_read(GPIOC, GPIO_PINS_14) == RESET)
    mask |= BUMPER_INPUT_LEFT_MASK;
  if(gpio_input_data_bit_read(GPIOC, GPIO_PINS_15) == RESET)
    mask |= BUMPER_INPUT_FRONT_MASK;
  return mask;
}

uint8_t bumper_input_take_pressed_latch_mask(void)
{
  uint32_t irq = __get_PRIMASK();
  uint8_t mask;
  __disable_irq();
  mask = bumper_pressed;
  bumper_pressed = 0U;
  if(irq == 0U) __enable_irq();
  return mask;
}

uint8_t bumper_input_has_pressed_latch(void)
{
  return (bumper_pressed != 0U) ? 1U : 0U;
}

void bumper_input_edge_irq_handler(uint8_t changed_mask)
{
  bumper_pressed |= bumper_input_read_active_mask() &
      changed_mask & BUMPER_INPUT_ALL_MASK;
}
