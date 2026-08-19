/**
  ****************************************************************************
  * @file     dock_ir_receiver.c
  * @brief    充电基站38kHz红外接收头硬件接口
  ****************************************************************************
  */

#include "dock_ir_receiver.h"

#include "at32f45x_conf.h"

#define DOCK_IR_PORT GPIOA
#define DOCK_IR_PIN  GPIO_PINS_8

void dock_ir_receiver_init(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = DOCK_IR_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(DOCK_IR_PORT, &gpio_init_struct);
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
