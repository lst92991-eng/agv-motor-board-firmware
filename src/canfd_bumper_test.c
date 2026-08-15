/**
  ****************************************************************************
  * @file     canfd_bumper_test.c
  * @brief    防撞条本地锁存与CAN FD事件上报安全联调
  *
  * 本测试不初始化电机。它复用正式CAN FD协议和防撞安全模块，用于验证租约、
  * STOP、ESTOP、FAULT_RESET、命令结果以及四路防撞事件上报。
  ****************************************************************************
  */

#include "canfd_bumper_test.h"

#include <stdint.h>
#include <stdio.h>

#include "at32f45x_board.h"
#include "bumper_safety.h"
#include "canfd_protocol.h"
#include "motion_command.h"

#define CANFD_BUMPER_TEST_LOOP_MS                  1U
#define CANFD_BUMPER_TEST_SAFETY_STEP_MS          10U
#define CANFD_BUMPER_TEST_REPORT_MS              100U

#define CANFD_BUMPER_TEST_FAULT_ESTOP          0x0001U
#define CANFD_BUMPER_TEST_FAULT_BUMPER         0x0008U
#define CANFD_BUMPER_TEST_FAULT_COMM           0x0010U
#define CANFD_BUMPER_TEST_FAULT_BUSOFF         0x0020U

static uint32_t canfd_bumper_test_uptime_ms;
static uint16_t canfd_bumper_test_fault;
static uint8_t canfd_bumper_test_safety_ticks;
static uint8_t canfd_bumper_test_report_ticks;

static uint8_t canfd_bumper_test_source_mask(void)
{
  uint8_t mask = 0U;

  if((canfd_bumper_test_fault & CANFD_BUMPER_TEST_FAULT_BUMPER) != 0U)
  {
    mask |= 0x01U;
  }
  if((canfd_bumper_test_fault & CANFD_BUMPER_TEST_FAULT_ESTOP) != 0U)
  {
    mask |= 0x02U;
  }
  if((canfd_bumper_test_fault & (CANFD_BUMPER_TEST_FAULT_COMM |
                                 CANFD_BUMPER_TEST_FAULT_BUSOFF)) != 0U)
  {
    mask |= 0x04U;
  }
  return mask;
}

static void canfd_bumper_test_send_safety(uint8_t urgent)
{
  bumper_safety_snapshot_t bumper;
  canfd_safety_status_t status;

  bumper_safety_get_snapshot(&bumper);
  status.uptime_ms = canfd_bumper_test_uptime_ms;
  status.fault_mask = (uint32_t)canfd_bumper_test_fault;
  status.warning_mask = 0UL;
  status.bumper_mask = (uint8_t)(bumper.active_mask | bumper.latched_mask);
  status.estop_source_mask = canfd_bumper_test_source_mask();
  status.safety_state = (canfd_bumper_test_fault == 0U) ? 0U :
                        (((canfd_bumper_test_fault &
                           (CANFD_BUMPER_TEST_FAULT_ESTOP |
                            CANFD_BUMPER_TEST_FAULT_BUMPER)) != 0U) ?
                         2U : 3U);
  canfd_protocol_send_safety_status(&status, urgent);
}

static motion_command_handle_result_t canfd_bumper_test_handle_command(
    const motion_command_t *command,
    void *context)
{
  (void)context;

  if(command == NULL)
  {
    return MOTION_HANDLE_REJECTED;
  }
  if(command->type == MOTION_CMD_ESTOP)
  {
    canfd_bumper_test_fault |= CANFD_BUMPER_TEST_FAULT_ESTOP;
    canfd_bumper_test_send_safety(1U);
    return MOTION_HANDLE_ACCEPTED;
  }
  if(command->type == MOTION_CMD_STOP)
  {
    canfd_bumper_test_send_safety(1U);
    return MOTION_HANDLE_ACCEPTED;
  }
  if(command->type == MOTION_CMD_CLEAR_FAULT)
  {
    if(bumper_safety_clear_latch() == 0U)
    {
      return MOTION_HANDLE_REJECTED;
    }
    canfd_bumper_test_fault = 0U;
    canfd_bumper_test_send_safety(1U);
    return MOTION_HANDLE_ACCEPTED;
  }
  if(command->type == MOTION_CMD_RESET_ODOM)
  {
    return (canfd_bumper_test_fault == 0U) ?
           MOTION_HANDLE_ACCEPTED : MOTION_HANDLE_REJECTED;
  }
  if((command->type == MOTION_CMD_ENABLE) ||
     (command->type == MOTION_CMD_DISABLE))
  {
    return MOTION_HANDLE_ACCEPTED;
  }
  if(command->type == MOTION_CMD_GOTO_LOCAL)
  {
    /* 仅为验证运动会话的租约超时，不初始化电机，也不输出PWM。 */
    if(canfd_bumper_test_fault != 0U)
    {
      return MOTION_HANDLE_REJECTED;
    }
    canfd_bumper_test_send_safety(1U);
    return MOTION_HANDLE_ACCEPTED;
  }
  if(command->type == MOTION_CMD_COMM_TIMEOUT)
  {
    canfd_bumper_test_fault |= CANFD_BUMPER_TEST_FAULT_COMM;
    canfd_bumper_test_send_safety(1U);
    return MOTION_HANDLE_ACCEPTED;
  }
  if(command->type == MOTION_CMD_CAN_BUSOFF)
  {
    canfd_bumper_test_fault |= CANFD_BUMPER_TEST_FAULT_BUSOFF;
    canfd_bumper_test_send_safety(1U);
    return MOTION_HANDLE_ACCEPTED;
  }
  return MOTION_HANDLE_REJECTED;
}

void canfd_bumper_test_init(void)
{
  bumper_safety_snapshot_t bumper;

  canfd_bumper_test_uptime_ms = 0UL;
  canfd_bumper_test_fault = 0U;
  canfd_bumper_test_safety_ticks = 0U;
  canfd_bumper_test_report_ticks = 0U;

  bumper_safety_init();
  bumper_safety_get_snapshot(&bumper);
  if(bumper.latched_mask != 0U)
  {
    canfd_bumper_test_fault |= CANFD_BUMPER_TEST_FAULT_BUMPER;
  }
  canfd_protocol_init(canfd_bumper_test_handle_command, NULL);

  printf("\r\nCANFD_BUMPER_TEST,NO_MOTOR\r\n");
  printf("CANFD_PROTOCOL,version=0.1,node=1,nominal=500kbps,data=4Mbps,BRS=1\r\n");
  printf("BUMPER_ACTIVE_LEVEL,LOW,raw=0x%02X,active=0x%02X,latched=0x%02X\r\n",
         bumper.raw_mask, bumper.active_mask, bumper.latched_mask);
  canfd_bumper_test_send_safety(1U);
}

void canfd_bumper_test_loop(void)
{
  bumper_safety_snapshot_t bumper;

  delay_ms(CANFD_BUMPER_TEST_LOOP_MS);
  canfd_bumper_test_uptime_ms += CANFD_BUMPER_TEST_LOOP_MS;
  canfd_protocol_poll(canfd_bumper_test_uptime_ms);

  canfd_bumper_test_safety_ticks++;
  if(canfd_bumper_test_safety_ticks >= CANFD_BUMPER_TEST_SAFETY_STEP_MS)
  {
    canfd_bumper_test_safety_ticks = 0U;
    if(bumper_safety_step_10ms() != 0U)
    {
      bumper_safety_get_snapshot(&bumper);
      if(bumper.latched_mask != 0U)
      {
        canfd_bumper_test_fault |= CANFD_BUMPER_TEST_FAULT_BUMPER;
      }
      printf("BUMPER_EVENT,raw=0x%02X,active=0x%02X,latched=0x%02X\r\n",
             bumper.raw_mask, bumper.active_mask, bumper.latched_mask);
      canfd_bumper_test_send_safety(1U);
    }
  }

  canfd_bumper_test_report_ticks++;
  if(canfd_bumper_test_report_ticks >= CANFD_BUMPER_TEST_REPORT_MS)
  {
    canfd_bumper_test_report_ticks = 0U;
    canfd_bumper_test_send_safety(0U);
  }
}
