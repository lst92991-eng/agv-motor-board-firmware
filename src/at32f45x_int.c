/**
  **************************************************************************
  * @file     at32f45x_int.c
  * @brief    main interrupt service routines.
  **************************************************************************
  *
  * Copyright (c) 2025, Artery Technology, All rights reserved.
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "at32f45x_int.h"
#include "bumper_input.h"
#include "canfd_protocol.h"
#include "debug_uart_transport.h"
#include "dock_ir_receiver.h"
#include "motor_driver.h"
#include "motor_encoder.h"

/** @addtogroup AT32F455_periph_examples
  * @{
  */

/** @addtogroup 455_USART_printf
  * @{
  */

static void motor_exception_safe_halt(void)
{
  __disable_irq();
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
  __DSB();
  __ISB();
  while(1)
  {
  }
}

/**
  * @brief  this function handles nmi exception.
  * @param  none
  * @retval none
  */
void NMI_Handler(void)
{
  motor_exception_safe_halt();
}

/**
  * @brief  this function handles hard fault exception.
  * @param  none
  * @retval none
  */
void HardFault_Handler(void)
{
  motor_exception_safe_halt();
}

/**
  * @brief  this function handles memory manage exception.
  * @param  none
  * @retval none
  */
void MemManage_Handler(void)
{
  motor_exception_safe_halt();
}

/**
  * @brief  this function handles bus fault exception.
  * @param  none
  * @retval none
  */
void BusFault_Handler(void)
{
  motor_exception_safe_halt();
}

/**
  * @brief  this function handles usage fault exception.
  * @param  none
  * @retval none
  */
void UsageFault_Handler(void)
{
  motor_exception_safe_halt();
}

#if !defined(USE_FREERTOS)
/* 未启用FreeRTOS时保留空的系统异常入口。 */
void SVC_Handler(void)
{
}

/**
  * @brief  this function handles debug monitor exception.
  * @param  none
  * @retval none
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  this function handles pendsv_handler exception.
  * @param  none
  * @retval none
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  this function handles systick handler.
  * @param  none
  * @retval none
  */
void SysTick_Handler(void)
{
}
#endif

/* USART1接收中断只搬运字节，协议解析在主循环中完成。 */
void USART1_IRQHandler(void)
{
  debug_uart_transport_irq_handler();
}

/* CAN3接收中断只搬运硬件帧，协议解析在CANFD任务或测试循环中完成。 */
void CAN3_RX_IRQHandler(void)
{
  canfd_protocol_rx_irq_handler();
}

/* PC4对应右编码器A相。 */
void EXINT4_IRQHandler(void)
{
  if(exint_interrupt_flag_get(EXINT_LINE_4) != RESET)
  {
    exint_flag_clear(EXINT_LINE_4);
    motor_encoder_right_edge_handler();
  }
}

/* EXINT5对应右B相，EXINT6/7对应左A/B相。 */
void EXINT9_5_IRQHandler(void)
{
  confirm_state left_changed = FALSE;
  confirm_state right_changed = FALSE;
  uint8_t bumper_changed = 0U;

  if(exint_interrupt_flag_get(EXINT_LINE_8) != RESET)
  {
    exint_flag_clear(EXINT_LINE_8);
    dock_ir_receiver_falling_edge_irq_handler();
  }

  if(exint_interrupt_flag_get(EXINT_LINE_5) != RESET)
  {
    exint_flag_clear(EXINT_LINE_5);
    right_changed = TRUE;
  }

  if(exint_interrupt_flag_get(EXINT_LINE_6) != RESET)
  {
    exint_flag_clear(EXINT_LINE_6);
    left_changed = TRUE;
  }

  if(exint_interrupt_flag_get(EXINT_LINE_7) != RESET)
  {
    exint_flag_clear(EXINT_LINE_7);
    left_changed = TRUE;
  }

  if(exint_interrupt_flag_get(EXINT_LINE_9) != RESET)
  {
    exint_flag_clear(EXINT_LINE_9);
    bumper_changed |= BUMPER_INPUT_RIGHT_MASK;
  }

  if(left_changed != FALSE)
  {
    motor_encoder_left_edge_handler();
  }

  if(right_changed != FALSE)
  {
    motor_encoder_right_edge_handler();
  }

  if(bumper_changed != 0U)
  {
    bumper_input_edge_irq_handler(bumper_changed);
  }
}

/* PC13、PC14和PC15分别对应左、后、前防撞条。 */
void EXINT15_10_IRQHandler(void)
{
  uint8_t bumper_changed = 0U;

  if(exint_interrupt_flag_get(EXINT_LINE_13) != RESET)
  {
    exint_flag_clear(EXINT_LINE_13);
    bumper_changed |= BUMPER_INPUT_LEFT_MASK;
  }

  if(exint_interrupt_flag_get(EXINT_LINE_14) != RESET)
  {
    exint_flag_clear(EXINT_LINE_14);
    bumper_changed |= BUMPER_INPUT_REAR_MASK;
  }

  if(exint_interrupt_flag_get(EXINT_LINE_15) != RESET)
  {
    exint_flag_clear(EXINT_LINE_15);
    bumper_changed |= BUMPER_INPUT_FRONT_MASK;
  }

  if(bumper_changed != 0U)
  {
    bumper_input_edge_irq_handler(bumper_changed);
  }
}

/**
  * @}
  */

/**
  * @}
  */
