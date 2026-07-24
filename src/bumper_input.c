/**
  *****************************************************************************
  * @file     bumper_input.c
  * @brief    四路防撞条数字输入驱动
  *
  * 左、后、前、右防撞条分别连接PC13、PC14、PC15、PB9。开发板每路
  * 已配置10k外部上拉、100欧姆串联电阻和100nF滤波电容，因此MCU内部
  * 上下拉均关闭。驱动使用双边沿中断，测试阶段不预设碰撞有效电平。
  *****************************************************************************
  */

#include "bumper_input.h"

#include "at32f45x.h"
#include "at32f45x_crm.h"
#include "at32f45x_exint.h"
#include "at32f45x_gpio.h"
#include "at32f45x_misc.h"
#include "at32f45x_scfg.h"

#define BUMPER_RIGHT_PORT        GPIOB
#define BUMPER_RIGHT_PIN         GPIO_PINS_9
#define BUMPER_LEFT_PORT         GPIOC
#define BUMPER_LEFT_PIN          GPIO_PINS_13
#define BUMPER_REAR_PORT         GPIOC
#define BUMPER_REAR_PIN          GPIO_PINS_14
#define BUMPER_FRONT_PORT        GPIOC
#define BUMPER_FRONT_PIN         GPIO_PINS_15

static volatile uint8_t bumper_input_changed_mask;

static void bumper_input_exint_line_init(uint32_t line)
{
  exint_init_type exint_config;

  exint_default_para_init(&exint_config);
  exint_config.line_enable = TRUE;
  exint_config.line_mode = EXINT_LINE_INTERRUPT;
  exint_config.line_select = line;
  exint_config.line_polarity = EXINT_TRIGGER_BOTH_EDGE;
  exint_init(&exint_config);
  exint_flag_clear(line);
}

void bumper_input_init(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);

  /* 板上已有10k外部上拉，禁止内部上下拉，避免改变输入分压和时间常数。 */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_pins = BUMPER_RIGHT_PIN;
  gpio_init(BUMPER_RIGHT_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins =
    BUMPER_LEFT_PIN | BUMPER_REAR_PIN | BUMPER_FRONT_PIN;
  gpio_init(BUMPER_LEFT_PORT, &gpio_init_struct);

  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOB, SCFG_PINS_SOURCE9);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE13);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE14);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE15);

  bumper_input_exint_line_init(EXINT_LINE_9);
  bumper_input_exint_line_init(EXINT_LINE_13);
  bumper_input_exint_line_init(EXINT_LINE_14);
  bumper_input_exint_line_init(EXINT_LINE_15);

  bumper_input_changed_mask = 0U;

  /* EXINT9_5与编码器共享入口，优先级保持与现有编码器配置一致。 */
  nvic_irq_enable(EXINT9_5_IRQn, 2U, 1U);
  nvic_irq_enable(EXINT15_10_IRQn, 2U, 2U);
}

uint8_t bumper_input_read_raw_mask(void)
{
  uint8_t raw_mask = 0U;

  if(gpio_input_data_bit_read(BUMPER_RIGHT_PORT, BUMPER_RIGHT_PIN) != RESET)
  {
    raw_mask |= BUMPER_INPUT_RIGHT_MASK;
  }
  if(gpio_input_data_bit_read(BUMPER_LEFT_PORT, BUMPER_LEFT_PIN) != RESET)
  {
    raw_mask |= BUMPER_INPUT_LEFT_MASK;
  }
  if(gpio_input_data_bit_read(BUMPER_REAR_PORT, BUMPER_REAR_PIN) != RESET)
  {
    raw_mask |= BUMPER_INPUT_REAR_MASK;
  }
  if(gpio_input_data_bit_read(BUMPER_FRONT_PORT, BUMPER_FRONT_PIN) != RESET)
  {
    raw_mask |= BUMPER_INPUT_FRONT_MASK;
  }

  return raw_mask;
}

uint8_t bumper_input_take_changed_mask(void)
{
  uint32_t interrupt_state = __get_PRIMASK();
  uint8_t changed_mask;

  __disable_irq();
  changed_mask = bumper_input_changed_mask;
  bumper_input_changed_mask = 0U;
  if(interrupt_state == 0U)
  {
    __enable_irq();
  }

  return changed_mask;
}

void bumper_input_edge_irq_handler(uint8_t changed_mask)
{
  bumper_input_changed_mask |= changed_mask & BUMPER_INPUT_ALL_MASK;
}
