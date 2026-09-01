/**
 * @file board_port.c
 * @brief 仅保留硬件中断分发、CPU异常制动和100ms独立看门狗。
 */
#include "board_port.h"

#include "at32f45x.h"
#include "at32f45x_exint.h"
#include "at32f45x_wdt.h"
#include "adc_dma.h"
#include "bumper.h"
#include "canfd_driver.h"
#include "motor_driver.h"
#include "motor_encoder.h"

static void board_safe_halt(void)
{
  __disable_irq();
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
  __DSB();
  __ISB();
  for(;;) {}
}

void NMI_Handler(void) { board_safe_halt(); }
void HardFault_Handler(void) { board_safe_halt(); }
void MemManage_Handler(void) { board_safe_halt(); }
void BusFault_Handler(void) { board_safe_halt(); }
void UsageFault_Handler(void) { board_safe_halt(); }

void CAN3_RX_IRQHandler(void) { canfd_driver_rx_irq(); }
void DMA1_Channel1_IRQHandler(void) { adc_dma_irq(); }

void EXINT4_IRQHandler(void)
{
  if(exint_interrupt_flag_get(EXINT_LINE_4) != RESET)
  {
    exint_flag_clear(EXINT_LINE_4);
    motor_encoder_right_edge_handler();
  }
}

void EXINT9_5_IRQHandler(void)
{
  uint8_t left = 0U;
  uint8_t right = 0U;
  if(exint_interrupt_flag_get(EXINT_LINE_5) != RESET)
  {
    exint_flag_clear(EXINT_LINE_5);
    right = 1U;
  }
  if(exint_interrupt_flag_get(EXINT_LINE_6) != RESET)
  {
    exint_flag_clear(EXINT_LINE_6);
    left = 1U;
  }
  if(exint_interrupt_flag_get(EXINT_LINE_7) != RESET)
  {
    exint_flag_clear(EXINT_LINE_7);
    left = 1U;
  }
  if(exint_interrupt_flag_get(EXINT_LINE_9) != RESET)
  {
    exint_flag_clear(EXINT_LINE_9);
    bumper_edge_irq(BUMPER_RIGHT);
  }
  if(left != 0U) motor_encoder_left_edge_handler();
  if(right != 0U) motor_encoder_right_edge_handler();
}

void EXINT15_10_IRQHandler(void)
{
  uint8_t changed = 0U;
  if(exint_interrupt_flag_get(EXINT_LINE_13) != RESET)
  {
    exint_flag_clear(EXINT_LINE_13);
    changed |= BUMPER_REAR;
  }
  if(exint_interrupt_flag_get(EXINT_LINE_14) != RESET)
  {
    exint_flag_clear(EXINT_LINE_14);
    changed |= BUMPER_LEFT;
  }
  if(exint_interrupt_flag_get(EXINT_LINE_15) != RESET)
  {
    exint_flag_clear(EXINT_LINE_15);
    changed |= BUMPER_FRONT;
  }
  if(changed != 0U) bumper_edge_irq(changed);
}

uint8_t board_port_watchdog_init(void)
{
  uint32_t timeout = 100000UL;
  wdt_register_write_enable(TRUE);
  wdt_divider_set(WDT_CLK_DIV_64);
  wdt_reload_value_set(62U);
  while(((wdt_flag_get(WDT_DIVF_UPDATE_FLAG) != RESET) ||
         (wdt_flag_get(WDT_RLDF_UPDATE_FLAG) != RESET)) && (timeout-- > 0U)) {}
  wdt_register_write_enable(FALSE);
  if(timeout == 0U) return 0U;
  wdt_counter_reload();
  wdt_enable();
  return 1U;
}

void board_port_watchdog_feed(void) { wdt_counter_reload(); }
