#include "at32f45x_int.h"

#include "bumper_input.h"
#include "canfd_protocol.h"
#include "motor_driver.h"
#include "motor_encoder.h"

static void motor_exception_safe_halt(void)
{
  __disable_irq();
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
  __DSB();
  __ISB();
  for(;;) {}
}

void NMI_Handler(void) { motor_exception_safe_halt(); }
void HardFault_Handler(void) { motor_exception_safe_halt(); }
void MemManage_Handler(void) { motor_exception_safe_halt(); }
void BusFault_Handler(void) { motor_exception_safe_halt(); }
void UsageFault_Handler(void) { motor_exception_safe_halt(); }

#if !defined(USE_FREERTOS)
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) {}
#endif

void CAN3_RX_IRQHandler(void)
{
  canfd_protocol_rx_irq_handler();
}

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
  confirm_state left = FALSE;
  confirm_state right = FALSE;
  uint8_t bumper = 0U;

  if(exint_interrupt_flag_get(EXINT_LINE_5) != RESET)
  {
    exint_flag_clear(EXINT_LINE_5);
    right = TRUE;
  }
  if(exint_interrupt_flag_get(EXINT_LINE_6) != RESET)
  {
    exint_flag_clear(EXINT_LINE_6);
    left = TRUE;
  }
  if(exint_interrupt_flag_get(EXINT_LINE_7) != RESET)
  {
    exint_flag_clear(EXINT_LINE_7);
    left = TRUE;
  }
  if(exint_interrupt_flag_get(EXINT_LINE_9) != RESET)
  {
    exint_flag_clear(EXINT_LINE_9);
    bumper = BUMPER_INPUT_RIGHT_MASK;
  }
  if(left != FALSE) motor_encoder_left_edge_handler();
  if(right != FALSE) motor_encoder_right_edge_handler();
  if(bumper != 0U) bumper_input_edge_irq_handler(bumper);
}

void EXINT15_10_IRQHandler(void)
{
  uint8_t bumper = 0U;

  if(exint_interrupt_flag_get(EXINT_LINE_13) != RESET)
  {
    exint_flag_clear(EXINT_LINE_13);
    bumper |= BUMPER_INPUT_REAR_MASK;
  }
  if(exint_interrupt_flag_get(EXINT_LINE_14) != RESET)
  {
    exint_flag_clear(EXINT_LINE_14);
    bumper |= BUMPER_INPUT_LEFT_MASK;
  }
  if(exint_interrupt_flag_get(EXINT_LINE_15) != RESET)
  {
    exint_flag_clear(EXINT_LINE_15);
    bumper |= BUMPER_INPUT_FRONT_MASK;
  }
  if(bumper != 0U) bumper_input_edge_irq_handler(bumper);
}
