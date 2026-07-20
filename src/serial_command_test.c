/**
  *****************************************************************************
  * @file     serial_command_test.c
  * @brief    上位机运动命令安全接收测试
  *
  * 本测试不会初始化电机驱动，也不会输出PWM。所有合法命令只打印解析结果，
  * 并返回RECEIVED应答，方便先验证Python和MCU之间的双向串口通信。
  *****************************************************************************
  */

#include "serial_command_test.h"

#include <stdio.h>

#include "at32f45x_board.h"
#include "debug_uart_transport.h"
#include "motion_command.h"
#include "serial_motion_protocol.h"

static motion_command_handle_result_t serial_command_test_handle(
    const motion_command_t *command,
    void *context)
{
  UNUSED(context);

  if(command->type == MOTION_CMD_GOTO_LOCAL)
  {
    printf("RXCMD,%u,GOTO,%ld,%ld,%ld,%u,NO_MOTION\r\n",
           (uint32_t)command->sequence,
           command->data.goto_local.x_mm,
           command->data.goto_local.y_mm,
           command->data.goto_local.yaw_cdeg,
           (uint32_t)command->data.goto_local.speed_mm_s);
  }
  else
  {
    printf("RXCMD,%u,%s,NO_MOTION\r\n",
           (uint32_t)command->sequence,
           motion_command_get_name(command->type));
  }

  return MOTION_HANDLE_RECEIVED;
}

void serial_command_test_init(void)
{
  debug_uart_transport_init();
  serial_motion_protocol_init(serial_command_test_handle, 0);
  printf("\r\nSERIAL_CMD_TEST,READY,NO_MOTOR_OUTPUT\r\n");
  printf("SERIAL_CMD_TEST,PROTOCOL_VERSION,%u\r\n",
         (uint32_t)MOTION_PROTOCOL_VERSION);
}

void serial_command_test_loop(void)
{
  serial_motion_protocol_poll();
  delay_ms(1U);
}
