/**
  *****************************************************************************
  * @file     motor_encoder.c
  * @brief    双路增量式正交编码器计数
  *
 * 左编码器 A/B 相连接 PA6/PA7，右编码器 A/B 相连接 PC4/PC5。
  * 每个信号的上升沿和下降沿都会触发中断，因此一组完整 AB 周期得到 4 个计数。
  *****************************************************************************
  */

#include "motor_encoder.h"

#include "at32f45x.h"
#include "at32f45x_crm.h"
#include "at32f45x_exint.h"
#include "at32f45x_gpio.h"
#include "at32f45x_misc.h"
#include "at32f45x_scfg.h"

/* 若实车前进时某一侧计数为负，可把对应方向修正值改为 -1。 */
#define MOTOR_LEFT_ENCODER_DIRECTION     1
/* 右电机镜像安装，反相后左右编码器正计数都表示小车向前。 */
#define MOTOR_RIGHT_ENCODER_DIRECTION   -1

static volatile int32_t left_encoder_count;
static volatile int32_t right_encoder_count;
static volatile uint32_t left_invalid_transition_count;
static volatile uint32_t right_invalid_transition_count;
static uint8_t left_encoder_state;
static uint8_t right_encoder_state;

/*
 * 正交解码查找表。索引由“上次 AB 状态 + 当前 AB 状态”组成。
 * 合法相邻状态产生 +1 或 -1，同时变化等非法跳变记为 0，可抑制部分毛刺。
 */
static const int8_t quadrature_step_table[16] =
{
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0
};

static uint8_t motor_encoder_read_left_state(void)
{
  uint8_t phase_a = (gpio_input_data_bit_read(GPIOA, GPIO_PINS_6) != RESET) ? 1U : 0U;
  uint8_t phase_b = (gpio_input_data_bit_read(GPIOA, GPIO_PINS_7) != RESET) ? 1U : 0U;

  return (uint8_t)((phase_a << 1) | phase_b);
}

static uint8_t motor_encoder_read_right_state(void)
{
  uint8_t phase_a = (gpio_input_data_bit_read(GPIOC, GPIO_PINS_4) != RESET) ? 1U : 0U;
  uint8_t phase_b = (gpio_input_data_bit_read(GPIOC, GPIO_PINS_5) != RESET) ? 1U : 0U;

  return (uint8_t)((phase_a << 1) | phase_b);
}

static void motor_encoder_exint_line_config(uint32_t line)
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

void motor_encoder_init(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);

  /* 当前编码器为5V推挽输出，GPIO不启用内部上拉，避免向3.3V电源回灌。 */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_pins = GPIO_PINS_4 | GPIO_PINS_5;
  gpio_init(GPIOC, &gpio_init_struct);

  gpio_init_struct.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7;
  gpio_init(GPIOA, &gpio_init_struct);

  /* 右编码器使用 PC4/PC5，左编码器使用 PA6/PA7。 */
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE4);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE5);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE6);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE7);

  motor_encoder_exint_line_config(EXINT_LINE_4);
  motor_encoder_exint_line_config(EXINT_LINE_5);
  motor_encoder_exint_line_config(EXINT_LINE_6);
  motor_encoder_exint_line_config(EXINT_LINE_7);

  left_encoder_state = motor_encoder_read_left_state();
  right_encoder_state = motor_encoder_read_right_state();
  left_encoder_count = 0;
  right_encoder_count = 0;
  left_invalid_transition_count = 0U;
  right_invalid_transition_count = 0U;

  nvic_irq_enable(EXINT4_IRQn, 2, 0);
  nvic_irq_enable(EXINT9_5_IRQn, 2, 1);
}

void motor_encoder_left_edge_handler(void)
{
  uint8_t current_state = motor_encoder_read_left_state();
  uint8_t table_index = (uint8_t)((left_encoder_state << 2) | current_state);
  int8_t step = quadrature_step_table[table_index];

  if((step == 0) && (current_state != left_encoder_state))
  {
    left_invalid_transition_count++;
  }
  left_encoder_count += step * MOTOR_LEFT_ENCODER_DIRECTION;
  left_encoder_state = current_state;
}

void motor_encoder_right_edge_handler(void)
{
  uint8_t current_state = motor_encoder_read_right_state();
  uint8_t table_index = (uint8_t)((right_encoder_state << 2) | current_state);
  int8_t step = quadrature_step_table[table_index];

  if((step == 0) && (current_state != right_encoder_state))
  {
    right_invalid_transition_count++;
  }
  right_encoder_count += step * MOTOR_RIGHT_ENCODER_DIRECTION;
  right_encoder_state = current_state;
}

int32_t motor_encoder_get_count(motor_id_t motor)
{
  if(motor == MOTOR_LEFT)
  {
    return left_encoder_count;
  }
  if(motor == MOTOR_RIGHT)
  {
    return right_encoder_count;
  }

  return 0;
}

uint32_t motor_encoder_get_invalid_transition_count(motor_id_t motor)
{
  if(motor == MOTOR_LEFT)
  {
    return left_invalid_transition_count;
  }
  if(motor == MOTOR_RIGHT)
  {
    return right_invalid_transition_count;
  }

  return 0U;
}

void motor_encoder_reset(motor_id_t motor)
{
  uint32_t interrupt_state = __get_PRIMASK();

  __disable_irq();
  if(motor == MOTOR_LEFT)
  {
    left_encoder_count = 0;
    left_invalid_transition_count = 0U;
  }
  else if(motor == MOTOR_RIGHT)
  {
    right_encoder_count = 0;
    right_invalid_transition_count = 0U;
  }

  if(interrupt_state == 0U)
  {
    __enable_irq();
  }
}

void motor_encoder_reset_all(void)
{
  uint32_t interrupt_state = __get_PRIMASK();

  __disable_irq();
  left_encoder_count = 0;
  right_encoder_count = 0;
  left_invalid_transition_count = 0U;
  right_invalid_transition_count = 0U;
  if(interrupt_state == 0U)
  {
    __enable_irq();
  }
}
