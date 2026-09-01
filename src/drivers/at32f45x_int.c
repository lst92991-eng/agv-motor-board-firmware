/**
 * @file at32f45x_int.c
 * @brief CPU异常、CAN3、编码器和防撞条的最小ISR入口。
 *
 * ISR只清标志、采样或投递；运动状态机和协议解析留在任务上下文。CPU异常
 * 不尝试日志或恢复，直接关中断并让AT8236保持主动制动。
 */

#include "at32f45x_int.h"

#include "bumper_input.h"
#include "canfd_protocol.h"
#include "motor_driver.h"
#include "motor_encoder.h"

/** @brief CPU异常共用的不可恢复主动制动停机路径。 */
static void motor_exception_safe_halt(void)
{
  __disable_irq();
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
  /* 确保PWM寄存器写入在进入不可返回循环前完成。 */
  __DSB();
  __ISB();
  for(;;) {}
}

/** @brief NMI入口，禁止尝试依赖RTOS或通信的恢复。 */
void NMI_Handler(void) { motor_exception_safe_halt(); }
/** @brief HardFault入口，立即切断正常运动输出。 */
void HardFault_Handler(void) { motor_exception_safe_halt(); }
/** @brief 内存管理Fault入口。 */
void MemManage_Handler(void) { motor_exception_safe_halt(); }
/** @brief 总线Fault入口。 */
void BusFault_Handler(void) { motor_exception_safe_halt(); }
/** @brief 未定义指令或非法状态Fault入口。 */
void UsageFault_Handler(void) { motor_exception_safe_halt(); }

#if !defined(USE_FREERTOS)
/** @brief 无RTOS构建的空SVC占位入口。 */
void SVC_Handler(void) {}
/** @brief 无RTOS构建的空DebugMonitor入口。 */
void DebugMon_Handler(void) {}
/** @brief 无RTOS构建的空PendSV入口。 */
void PendSV_Handler(void) {}
/** @brief 无RTOS构建的空SysTick入口。 */
void SysTick_Handler(void) {}
#endif

/** @brief CAN3 RX中断转交协议层，只复制硬件帧。 */
void CAN3_RX_IRQHandler(void)
{
  canfd_protocol_rx_irq_handler();
}

/** @brief 处理右编码器PC4相位边沿。 */
void EXINT4_IRQHandler(void)
{
  if(exint_interrupt_flag_get(EXINT_LINE_4) != RESET)
  {
    exint_flag_clear(EXINT_LINE_4);
    motor_encoder_right_edge_handler();
  }
}

/** @brief 处理右编码器PC5、左编码器PA6/PA7和右防撞共享中断。 */
void EXINT9_5_IRQHandler(void)
{
  confirm_state left = FALSE;
  confirm_state right = FALSE;
  uint8_t bumper = 0U;

  /* EXINT5为右编码器，6/7为左编码器，9为右防撞。 */
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

/** @brief 处理PC13后、PC14左、PC15前三路防撞共享中断。 */
void EXINT15_10_IRQHandler(void)
{
  uint8_t bumper = 0U;

  /* 物理安装冻结为PC13后、PC14左、PC15前，位图遵循整车V0.2。 */
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
