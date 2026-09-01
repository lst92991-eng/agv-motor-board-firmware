/** @file bumper.c @brief 防撞条EXINT短脉冲锁存和100ms释放门。 */
#include "bumper.h"

#include "at32f45x.h"
#include "at32f45x_crm.h"
#include "at32f45x_exint.h"
#include "at32f45x_gpio.h"
#include "at32f45x_misc.h"
#include "at32f45x_scfg.h"

static volatile uint8_t bumper_irq_latch;
static uint8_t bumper_latch;
static uint8_t bumper_release_ticks;

static uint8_t bumper_read(void)
{
  uint8_t mask = 0U;
  if(gpio_input_data_bit_read(GPIOC, GPIO_PINS_15) == RESET) mask |= BUMPER_FRONT;
  if(gpio_input_data_bit_read(GPIOC, GPIO_PINS_13) == RESET) mask |= BUMPER_REAR;
  if(gpio_input_data_bit_read(GPIOC, GPIO_PINS_14) == RESET) mask |= BUMPER_LEFT;
  if(gpio_input_data_bit_read(GPIOB, GPIO_PINS_9) == RESET) mask |= BUMPER_RIGHT;
  return mask;
}

static void bumper_line(uint32_t line)
{
  exint_init_type exti;
  exint_default_para_init(&exti);
  exti.line_enable = TRUE;
  exti.line_mode = EXINT_LINE_INTERRUPT;
  exti.line_select = line;
  exti.line_polarity = EXINT_TRIGGER_BOTH_EDGE;
  exint_init(&exti);
  exint_flag_clear(line);
}

void bumper_init(void)
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
  bumper_irq_latch = bumper_latch = bumper_read();
  bumper_release_ticks = (bumper_latch == 0U) ? 10U : 0U;
  nvic_irq_enable(EXINT9_5_IRQn, 2U, 1U);
  nvic_irq_enable(EXINT15_10_IRQn, 2U, 2U);
}

void bumper_edge_irq(uint8_t changed_mask)
{
  bumper_irq_latch |= bumper_read() & changed_mask;
}

void bumper_update(void)
{
  uint32_t irq = __get_PRIMASK();
  uint8_t edge;
  uint8_t active = bumper_read();
  __disable_irq();
  edge = bumper_irq_latch;
  bumper_irq_latch = 0U;
  if(irq == 0U) __enable_irq();
  if((active | edge) != 0U)
  {
    bumper_latch |= active | edge;
    bumper_release_ticks = 0U;
  }
  else if(bumper_release_ticks < 10U) bumper_release_ticks++;
}

uint8_t bumper_latched(void) { return bumper_latch; }
uint8_t bumper_can_clear(void) { return (bumper_read() == 0U) && (bumper_release_ticks >= 10U); }
uint8_t bumper_clear(void)
{
  if(bumper_can_clear() == 0U) return 0U;
  bumper_latch = 0U;
  return 1U;
}
