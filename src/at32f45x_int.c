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
#include "debug_uart_transport.h"
#include "motor_encoder.h"

/** @addtogroup AT32F455_periph_examples
  * @{
  */

/** @addtogroup 455_USART_printf
  * @{
  */

/**
  * @brief  this function handles nmi exception.
  * @param  none
  * @retval none
  */
void NMI_Handler(void)
{
}

/**
  * @brief  this function handles hard fault exception.
  * @param  none
  * @retval none
  */
void HardFault_Handler(void)
{
  /* go to infinite loop when hard fault exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles memory manage exception.
  * @param  none
  * @retval none
  */
void MemManage_Handler(void)
{
  /* go to infinite loop when memory manage exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles bus fault exception.
  * @param  none
  * @retval none
  */
void BusFault_Handler(void)
{
  /* go to infinite loop when bus fault exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles usage fault exception.
  * @param  none
  * @retval none
  */
void UsageFault_Handler(void)
{
  /* go to infinite loop when usage fault exception occurs */
  while(1)
  {
  }
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

/* PC4 对应左编码器 A 相。 */
void EXINT4_IRQHandler(void)
{
  if(exint_interrupt_flag_get(EXINT_LINE_4) != RESET)
  {
    exint_flag_clear(EXINT_LINE_4);
    motor_encoder_left_edge_handler();
  }
}

/* EXINT5~7分别对应左B相、右A相和右B相。 */
void EXINT9_5_IRQHandler(void)
{
  confirm_state left_changed = FALSE;
  confirm_state right_changed = FALSE;

  if(exint_interrupt_flag_get(EXINT_LINE_5) != RESET)
  {
    exint_flag_clear(EXINT_LINE_5);
    left_changed = TRUE;
  }

  if(exint_interrupt_flag_get(EXINT_LINE_6) != RESET)
  {
    exint_flag_clear(EXINT_LINE_6);
    right_changed = TRUE;
  }

  if(exint_interrupt_flag_get(EXINT_LINE_7) != RESET)
  {
    exint_flag_clear(EXINT_LINE_7);
    right_changed = TRUE;
  }

  if(left_changed != FALSE)
  {
    motor_encoder_left_edge_handler();
  }

  if(right_changed != FALSE)
  {
    motor_encoder_right_edge_handler();
  }
}

/**
  * @}
  */

/**
  * @}
  */
