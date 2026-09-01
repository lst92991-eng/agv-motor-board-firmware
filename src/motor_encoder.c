#include "motor_encoder.h"

#include <string.h>

#include "at32f45x.h"
#include "at32f45x_crm.h"
#include "at32f45x_exint.h"
#include "at32f45x_gpio.h"
#include "at32f45x_misc.h"
#include "at32f45x_scfg.h"

typedef struct
{
  volatile int32_t count;
  volatile uint32_t invalid;
  uint8_t state;
  int8_t direction;
} motor_encoder_axis_t;

static const int8_t motor_encoder_step[16] =
{
   0,  1, -1,  0, -1,  0,  0,  1,
   1,  0,  0, -1,  0, -1,  1,  0
};
static motor_encoder_axis_t motor_encoder_axis[2];

static uint8_t motor_encoder_read(motor_id_t motor)
{
  gpio_type *port = (motor == MOTOR_LEFT) ? GPIOA : GPIOC;
  uint16_t a = (motor == MOTOR_LEFT) ? GPIO_PINS_6 : GPIO_PINS_4;
  uint16_t b = (motor == MOTOR_LEFT) ? GPIO_PINS_7 : GPIO_PINS_5;
  return (uint8_t)(((gpio_input_data_bit_read(port, a) != RESET) ? 2U : 0U) |
                   ((gpio_input_data_bit_read(port, b) != RESET) ? 1U : 0U));
}

static void motor_encoder_line(uint32_t line)
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

void motor_encoder_init(void)
{
  gpio_init_type gpio;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio);
  gpio.gpio_mode = GPIO_MODE_INPUT;
  gpio.gpio_pull = GPIO_PULL_NONE;
  gpio.gpio_pins = GPIO_PINS_4 | GPIO_PINS_5;
  gpio_init(GPIOC, &gpio);
  gpio.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7;
  gpio_init(GPIOA, &gpio);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE4);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE5);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE6);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE7);
  motor_encoder_line(EXINT_LINE_4);
  motor_encoder_line(EXINT_LINE_5);
  motor_encoder_line(EXINT_LINE_6);
  motor_encoder_line(EXINT_LINE_7);
  memset(motor_encoder_axis, 0, sizeof(motor_encoder_axis));
  motor_encoder_axis[MOTOR_LEFT].state = motor_encoder_read(MOTOR_LEFT);
  motor_encoder_axis[MOTOR_RIGHT].state = motor_encoder_read(MOTOR_RIGHT);
  motor_encoder_axis[MOTOR_LEFT].direction = 1;
  motor_encoder_axis[MOTOR_RIGHT].direction = -1;
  nvic_irq_enable(EXINT4_IRQn, 2U, 0U);
  nvic_irq_enable(EXINT9_5_IRQn, 2U, 1U);
}

static void motor_encoder_edge(motor_id_t motor)
{
  motor_encoder_axis_t *axis = &motor_encoder_axis[motor];
  uint8_t state = motor_encoder_read(motor);
  int8_t step = motor_encoder_step[(axis->state << 2) | state];
  if((step == 0) && (state != axis->state)) axis->invalid++;
  axis->count += step * axis->direction;
  axis->state = state;
}

void motor_encoder_left_edge_handler(void) { motor_encoder_edge(MOTOR_LEFT); }
void motor_encoder_right_edge_handler(void) { motor_encoder_edge(MOTOR_RIGHT); }

int32_t motor_encoder_get_count(motor_id_t motor)
{
  return (motor <= MOTOR_RIGHT) ? motor_encoder_axis[motor].count : 0;
}

uint32_t motor_encoder_get_invalid_transition_count(motor_id_t motor)
{
  return (motor <= MOTOR_RIGHT) ? motor_encoder_axis[motor].invalid : 0U;
}

void motor_encoder_reset_all(void)
{
  uint32_t irq = __get_PRIMASK();
  __disable_irq();
  motor_encoder_axis[MOTOR_LEFT].count = 0;
  motor_encoder_axis[MOTOR_RIGHT].count = 0;
  motor_encoder_axis[MOTOR_LEFT].invalid = 0U;
  motor_encoder_axis[MOTOR_RIGHT].invalid = 0U;
  if(irq == 0U) __enable_irq();
}
