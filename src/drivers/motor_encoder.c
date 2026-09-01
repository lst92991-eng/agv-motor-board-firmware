/**
 * @file motor_encoder.c
 * @brief 左右AB编码器双边沿正交解码与异常跳变统计。
 *
 * 左轮PA6/PA7、右轮PC4/PC5依赖外部10kΩ上拉到5V，MCU内部不上下拉。边沿
 * 在EXINT ISR中解码，任务只读取32位快照或在停车状态下原子清零。
 */

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

/** @brief 读取指定轮两相电平并编码为2位AB状态。 */
static uint8_t motor_encoder_read(motor_id_t motor)
{
  gpio_type *port = (motor == MOTOR_LEFT) ? GPIOA : GPIOC;
  uint16_t a = (motor == MOTOR_LEFT) ? GPIO_PINS_6 : GPIO_PINS_4;
  uint16_t b = (motor == MOTOR_LEFT) ? GPIO_PINS_7 : GPIO_PINS_5;
  return (uint8_t)(((gpio_input_data_bit_read(port, a) != RESET) ? 2U : 0U) |
                   ((gpio_input_data_bit_read(port, b) != RESET) ? 1U : 0U));
}

/** @brief 为一个编码器相位配置双边沿EXINT并清除旧标志。 */
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

/** @brief 初始化四个编码器输入、方向约定和中断入口。 */
void motor_encoder_init(void)
{
  gpio_init_type gpio;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio);
  gpio.gpio_mode = GPIO_MODE_INPUT;
  /* 外部上拉已定义电平，内部上下拉会改变5V接口的边沿特性。 */
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

/** @brief 用Gray码状态表处理指定轮的一次AB边沿。 */
static void motor_encoder_edge(motor_id_t motor)
{
  motor_encoder_axis_t *axis = &motor_encoder_axis[motor];
  uint8_t state = motor_encoder_read(motor);
  int8_t step = motor_encoder_step[(axis->state << 2) | state];
  /* 两位同时变化不是合法Gray码相邻状态，计数保持但记录诊断。 */
  if((step == 0) && (state != axis->state)) axis->invalid++;
  axis->count += step * axis->direction;
  axis->state = state;
}

/** @brief 左编码器EXINT统一入口。 */
void motor_encoder_left_edge_handler(void) { motor_encoder_edge(MOTOR_LEFT); }
/** @brief 右编码器EXINT统一入口。 */
void motor_encoder_right_edge_handler(void) { motor_encoder_edge(MOTOR_RIGHT); }

/** @brief 返回指定轮累计有符号计数。 */
int32_t motor_encoder_get_count(motor_id_t motor)
{
  return (motor <= MOTOR_RIGHT) ? motor_encoder_axis[motor].count : 0;
}

/** @brief 返回指定轮非法Gray码跳变累计值。 */
uint32_t motor_encoder_get_invalid_transition_count(motor_id_t motor)
{
  return (motor <= MOTOR_RIGHT) ? motor_encoder_axis[motor].invalid : 0U;
}

/** @brief 在短临界区内同时清零两轮计数和异常统计。 */
void motor_encoder_reset_all(void)
{
  uint32_t irq = __get_PRIMASK();
  __disable_irq();
  /* count与invalid由ISR写入，成组清零期间必须屏蔽对应边沿。 */
  motor_encoder_axis[MOTOR_LEFT].count = 0;
  motor_encoder_axis[MOTOR_RIGHT].count = 0;
  motor_encoder_axis[MOTOR_LEFT].invalid = 0U;
  motor_encoder_axis[MOTOR_RIGHT].invalid = 0U;
  if(irq == 0U) __enable_irq();
}
