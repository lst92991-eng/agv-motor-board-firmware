/**
  ****************************************************************************
  * @file     canfd_protocol.c
  * @brief    AGV电机板CAN FD驱动、协议解析和状态上报
  *
  * CAN3使用PB3/PB4的MUX11，仲裁段500kbps、数据段4Mbps并开启BRS。
  * 接收中断只复制帧到静态队列；字段解析、会话去重和发送均在任务轮询中
  * 执行。PDO逐字段序列化，不叠加私有头或应用层CRC。
  ****************************************************************************
  */

#include "canfd_protocol.h"

#include <stddef.h>
#include <string.h>

#include "at32f45x.h"
#include "at32f45x_can.h"
#include "at32f45x_crm.h"
#include "at32f45x_gpio.h"
#include "at32f45x_misc.h"

#define CANFD_BITTIME_DIV                         2U
/* 192 MHz PCLK CAN clock: both phases sample at 75%; arbitration/data SJW
 * are 250 ns and 62.5 ns respectively. */
#define CANFD_AC_BTS1                           144U
#define CANFD_AC_BTS2                            48U
#define CANFD_AC_RSAW                            24U
#define CANFD_FD_BTS1                            18U
#define CANFD_FD_BTS2                             6U
#define CANFD_FD_RSAW                             6U
#define CANFD_FD_SSP                             19U

#define CANFD_RX_QUEUE_LENGTH                     8U
#define CANFD_TX_URGENT_QUEUE_LENGTH             16U
#define CANFD_TX_NORMAL_QUEUE_LENGTH              8U
#define CANFD_LEASE_MIN_MS                      100U
#define CANFD_LEASE_DEFAULT_MS                  200U
#define CANFD_LEASE_MAX_MS                     1000U
#define CANFD_TASK_TIMEOUT_MAX_MS            120000UL
#define CANFD_BUSOFF_RECOVERY_PERIOD_MS        1000UL
#define CANFD_TX_COMPLETE_TIMEOUT_MS              50UL

#define CANFD_SAFETY_SOURCE_BUMPER              0x01U
#define CANFD_BUS_STATE_BUSOFF                  0x01U
#define CANFD_BUS_STATE_ERROR_PASSIVE           0x02U
#define CANFD_BUS_STATE_ERROR_WARNING           0x04U
#define CANFD_BUS_STATE_RX_OVERFLOW             0x08U

#define CANFD_RESULT_STAGE_RECEIVED               0U
#define CANFD_RESULT_STAGE_ACCEPTED               1U
#define CANFD_RESULT_STAGE_EXECUTING              2U
#define CANFD_RESULT_STAGE_SUCCEEDED              3U
#define CANFD_RESULT_STAGE_REJECTED               4U
#define CANFD_RESULT_STAGE_ABORTED                5U
#define CANFD_RESULT_STAGE_TIMEOUT                6U
#define CANFD_RESULT_STAGE_FAILED                 7U

#define CANFD_RESULT_OK                          0x00U
#define CANFD_RESULT_UNSUPPORTED_COMMAND         0x01U
#define CANFD_RESULT_INVALID_STATE               0x02U
#define CANFD_RESULT_OUT_OF_RANGE                0x03U
#define CANFD_RESULT_BUSY                        0x04U
#define CANFD_RESULT_NOT_CONTROL_OWNER           0x05U
#define CANFD_RESULT_DATA_INVALID                0x08U
#define CANFD_RESULT_DUPLICATE_MISMATCH           0x09U
#define CANFD_RESULT_CONTROL_LOST                0x0AU
#define CANFD_RESULT_INTERNAL_ERROR              0x0BU
#define CANFD_RESULT_UNSUPPORTED_PARAMETER       0x0CU

typedef struct
{
  uint32_t id;
  can_data_length_type data_length;
  uint8_t data_size;
  uint8_t data[64];
} canfd_frame_t;

typedef struct
{
  canfd_frame_t *frames;
  uint8_t capacity;
  volatile uint8_t head;
  volatile uint8_t tail;
} canfd_tx_queue_t;

static canfd_frame_t canfd_rx_queue[CANFD_RX_QUEUE_LENGTH];
static canfd_frame_t
  canfd_tx_urgent_frames[CANFD_TX_URGENT_QUEUE_LENGTH];
static canfd_frame_t
  canfd_tx_normal_frames[CANFD_TX_NORMAL_QUEUE_LENGTH];
static volatile uint8_t canfd_rx_head;
static volatile uint8_t canfd_rx_tail;
static canfd_tx_queue_t canfd_tx_urgent_queue;
static canfd_tx_queue_t canfd_tx_normal_queue;

static motion_command_handler_t canfd_motion_handler;
static void *canfd_motion_handler_context;
static uint32_t canfd_now_ms;
static uint32_t canfd_host_session;
static uint32_t canfd_last_lease_ms;
static uint32_t canfd_last_busoff_recovery_ms;
static uint32_t canfd_rx_count;
static uint32_t canfd_tx_count;
static uint32_t canfd_rx_drop_count;
static uint32_t canfd_tx_drop_count;
static uint32_t canfd_last_command_sequence;
static uint32_t canfd_last_command_session;
static uint32_t canfd_internal_sequence;
static uint16_t canfd_status_sequence;
static uint16_t canfd_lease_duration_ms;
static uint16_t canfd_last_lease_counter;
static uint8_t canfd_last_command_payload[32];
static uint8_t canfd_last_result_payload[16];
static uint8_t canfd_last_command_valid;
static uint8_t canfd_last_result_valid;
static uint8_t canfd_lease_counter_valid;
static uint8_t canfd_session_valid;
static uint8_t canfd_motion_session_active;
static uint8_t canfd_comm_timeout_reported;
static uint8_t canfd_busoff_reported;
static uint8_t canfd_rx_overflow_latched;
static uint8_t canfd_tx_handle;
static uint8_t canfd_tx_active_handle;
static uint8_t canfd_tx_inflight;
static uint8_t canfd_tx_link_fault_reported;
static uint32_t canfd_tx_started_ms;
static canfd_tx_queue_t *canfd_tx_active_queue;
static canfd_magnetic_sample_t canfd_magnetic_sample;
static volatile uint32_t canfd_magnetic_generation;

static uint8_t canfd_dlc_to_size(can_data_length_type data_length)
{
  static const uint8_t sizes[16] =
  {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
    8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
  };
  uint8_t dlc = (uint8_t)data_length;

  return (dlc < 16U) ? sizes[dlc] : 0U;
}

static can_data_length_type canfd_size_to_dlc(uint8_t data_size)
{
  switch(data_size)
  {
    case 12U:
      return CAN_DLC_BYTES_12;
    case 16U:
      return CAN_DLC_BYTES_16;
    case 24U:
      return CAN_DLC_BYTES_24;
    case 32U:
      return CAN_DLC_BYTES_32;
    default:
      return CAN_DLC_BYTES_0;
  }
}

static uint16_t canfd_get_u16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t canfd_get_u32(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

static int16_t canfd_get_i16(const uint8_t *data)
{
  return (int16_t)canfd_get_u16(data);
}

static int32_t canfd_get_i32(const uint8_t *data)
{
  return (int32_t)canfd_get_u32(data);
}

static void canfd_put_u16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
}

static void canfd_put_i16(uint8_t *data, int16_t value)
{
  canfd_put_u16(data, (uint16_t)value);
}

static void canfd_put_u32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
  data[2] = (uint8_t)(value >> 16U);
  data[3] = (uint8_t)(value >> 24U);
}

static void canfd_put_i32(uint8_t *data, int32_t value)
{
  canfd_put_u32(data, (uint32_t)value);
}

static uint8_t canfd_tx_queue_push(canfd_tx_queue_t *queue,
                                   const canfd_frame_t *frame)
{
  uint32_t interrupt_state;
  uint8_t next_head;

  if((queue == NULL) || (frame == NULL) || (queue->frames == NULL) ||
     (queue->capacity < 2U))
  {
    canfd_tx_drop_count++;
    return 0U;
  }

  interrupt_state = __get_PRIMASK();
  __disable_irq();
  next_head = (uint8_t)((queue->head + 1U) % queue->capacity);
  if(next_head == queue->tail)
  {
    if(interrupt_state == 0U)
    {
      __enable_irq();
    }
    canfd_tx_drop_count++;
    return 0U;
  }
  queue->frames[queue->head] = *frame;
  queue->head = next_head;
  if(interrupt_state == 0U)
  {
    __enable_irq();
  }
  return 1U;
}

static uint8_t canfd_queue_payload(uint32_t id,
                                   const uint8_t *data,
                                   uint8_t data_size,
                                   uint8_t urgent)
{
  canfd_frame_t frame;
  can_data_length_type data_length = canfd_size_to_dlc(data_size);

  if((data == NULL) || (data_length == CAN_DLC_BYTES_0))
  {
    return 0U;
  }

  memset(&frame, 0, sizeof(frame));
  frame.id = id;
  frame.data_length = data_length;
  frame.data_size = data_size;
  memcpy(frame.data, data, data_size);
  return canfd_tx_queue_push((urgent != 0U) ?
                             &canfd_tx_urgent_queue :
                             &canfd_tx_normal_queue,
                             &frame);
}

static uint8_t canfd_rx_queue_pop(canfd_frame_t *frame)
{
  uint32_t interrupt_state;

  interrupt_state = __get_PRIMASK();
  __disable_irq();
  if(canfd_rx_tail == canfd_rx_head)
  {
    if(interrupt_state == 0U)
    {
      __enable_irq();
    }
    return 0U;
  }
  *frame = canfd_rx_queue[canfd_rx_tail];
  canfd_rx_tail = (uint8_t)((canfd_rx_tail + 1U) %
                            CANFD_RX_QUEUE_LENGTH);
  if(interrupt_state == 0U)
  {
    __enable_irq();
  }
  return 1U;
}

static uint8_t canfd_result_code_from_error(motion_command_error_t error)
{
  if(error == MOTION_COMMAND_ERROR_NONE)
  {
    return CANFD_RESULT_OK;
  }
  if(error == MOTION_COMMAND_ERROR_RANGE)
  {
    return CANFD_RESULT_OUT_OF_RANGE;
  }
  if(error == MOTION_COMMAND_ERROR_BUSY)
  {
    return CANFD_RESULT_BUSY;
  }
  if(error == MOTION_COMMAND_ERROR_SESSION)
  {
    return CANFD_RESULT_NOT_CONTROL_OWNER;
  }
  if(error == MOTION_COMMAND_ERROR_UNSUPPORTED)
  {
    return CANFD_RESULT_UNSUPPORTED_COMMAND;
  }
  if(error == MOTION_COMMAND_ERROR_FAULT_LOCKED)
  {
    return CANFD_RESULT_INVALID_STATE;
  }
  if(error == MOTION_COMMAND_ERROR_TIMEOUT)
  {
    return CANFD_RESULT_CONTROL_LOST;
  }
  if((error == MOTION_COMMAND_ERROR_VERSION) ||
     (error == MOTION_COMMAND_ERROR_FORMAT) ||
     (error == MOTION_COMMAND_ERROR_CRC) ||
     (error == MOTION_COMMAND_ERROR_LENGTH) ||
     (error == MOTION_COMMAND_ERROR_SEQUENCE))
  {
    return CANFD_RESULT_DATA_INVALID;
  }
  return CANFD_RESULT_INTERNAL_ERROR;
}

static void canfd_send_result(uint32_t command_id,
                              uint32_t session_id,
                              uint8_t stage,
                              uint8_t result_code,
                              uint16_t detail_code)
{
  uint8_t data[16] = {0};

  canfd_put_u32(&data[0], command_id);
  canfd_put_u32(&data[4], session_id);
  data[8] = stage;
  data[9] = result_code;
  canfd_put_u16(&data[10], detail_code);
  canfd_put_u32(&data[12], canfd_now_ms);
  if((canfd_last_command_valid != 0U) &&
     (command_id == canfd_last_command_sequence) &&
     (session_id == canfd_last_command_session))
  {
    memcpy(canfd_last_result_payload, data, sizeof(data));
    canfd_last_result_valid = 1U;
  }
  (void)canfd_queue_payload(CANFD_ID_COMMAND_RESULT, data, 16U, 1U);
}

static void canfd_send_ack(uint32_t sequence,
                           uint32_t session_id,
                           motion_command_type_t type,
                           canfd_ack_status_t status,
                           motion_command_error_t error,
                           uint16_t fault_mask,
                           uint8_t state)
{
  uint8_t stage;
  uint8_t result_code = canfd_result_code_from_error(error);
  uint16_t detail_code = (error != MOTION_COMMAND_ERROR_NONE) ?
                         (uint16_t)error : fault_mask;

  (void)type;
  (void)state;
  if(status == CANFD_ACK_RECEIVED)
  {
    stage = CANFD_RESULT_STAGE_RECEIVED;
  }
  else if(status == CANFD_ACK_ACCEPTED)
  {
    stage = CANFD_RESULT_STAGE_ACCEPTED;
  }
  else if(status == CANFD_ACK_COMPLETED)
  {
    stage = CANFD_RESULT_STAGE_SUCCEEDED;
  }
  else if(status == CANFD_ACK_CANCELED)
  {
    stage = CANFD_RESULT_STAGE_ABORTED;
  }
  else if(status == CANFD_ACK_FAULT)
  {
    stage = CANFD_RESULT_STAGE_FAILED;
  }
  else
  {
    stage = CANFD_RESULT_STAGE_REJECTED;
  }
  canfd_send_result(sequence, session_id, stage, result_code, detail_code);
}

static uint8_t canfd_submit_command(motion_command_t *command,
                                    uint8_t session_required)
{
  motion_command_error_t error;
  motion_command_handle_result_t result;

  error = motion_command_validate(command);
  if(error != MOTION_COMMAND_ERROR_NONE)
  {
    canfd_send_ack(command->sequence, command->session_id, command->type,
                   CANFD_ACK_REJECTED, error, 0U, 0U);
    return 0U;
  }
  if((session_required != 0U) &&
     ((canfd_session_valid == 0U) ||
      (command->session_id != canfd_host_session)))
  {
    canfd_send_ack(command->sequence, command->session_id, command->type,
                   CANFD_ACK_REJECTED, MOTION_COMMAND_ERROR_SESSION,
                   0U, 0U);
    return 0U;
  }
  if(canfd_motion_handler == NULL)
  {
    canfd_send_ack(command->sequence, command->session_id, command->type,
                   CANFD_ACK_REJECTED, MOTION_COMMAND_ERROR_UNSUPPORTED,
                   0U, 0U);
    return 0U;
  }

  result = canfd_motion_handler(command, canfd_motion_handler_context);
  if((result == MOTION_HANDLE_RECEIVED) ||
     (result == MOTION_HANDLE_ACCEPTED))
  {
    if(command->type == MOTION_CMD_ENABLE)
    {
      canfd_motion_session_active = 1U;
      canfd_comm_timeout_reported = 0U;
    }
    else if((command->type == MOTION_CMD_ESTOP) ||
            (command->type == MOTION_CMD_DISABLE))
    {
      canfd_motion_session_active = 0U;
    }
    canfd_send_ack(command->sequence, command->session_id, command->type,
                   (result == MOTION_HANDLE_RECEIVED) ?
                   CANFD_ACK_RECEIVED : CANFD_ACK_ACCEPTED,
                   MOTION_COMMAND_ERROR_NONE, 0U, 0U);
    return 1U;
  }
  canfd_protocol_send_motion_result(command, result, 0U, 0U);
  return 0U;
}

static uint8_t canfd_command_is_duplicate(const canfd_frame_t *frame,
                                          uint32_t command_id,
                                          uint32_t session_id)
{
  if((canfd_last_command_valid == 0U) ||
     (session_id != canfd_last_command_session))
  {
    return 0U;
  }
  if(command_id < canfd_last_command_sequence)
  {
    canfd_send_result(command_id, session_id,
                      CANFD_RESULT_STAGE_REJECTED,
                      CANFD_RESULT_DATA_INVALID,
                      MOTION_COMMAND_ERROR_SEQUENCE);
    return 1U;
  }
  if(command_id != canfd_last_command_sequence)
  {
    return 0U;
  }
  if(memcmp(frame->data, canfd_last_command_payload, 32U) != 0)
  {
    canfd_send_result(command_id, session_id,
                      CANFD_RESULT_STAGE_REJECTED,
                      CANFD_RESULT_DUPLICATE_MISMATCH, 0U);
  }
  else if(canfd_last_result_valid != 0U)
  {
    (void)canfd_queue_payload(CANFD_ID_COMMAND_RESULT,
                              canfd_last_result_payload, 16U, 1U);
  }
  else
  {
    canfd_send_result(command_id, session_id,
                      CANFD_RESULT_STAGE_RECEIVED,
                      CANFD_RESULT_OK, 0U);
  }
  return 1U;
}

static uint8_t canfd_session_matches(uint32_t session_id)
{
  return ((canfd_session_valid != 0U) &&
          (session_id != 0UL) &&
          (session_id == canfd_host_session)) ? 1U : 0U;
}

static void canfd_accept_claim(uint32_t command_id,
                               uint32_t session_id,
                               int32_t requested_lease_ms)
{
  if((session_id == 0UL) ||
     (requested_lease_ms < (int32_t)CANFD_LEASE_MIN_MS) ||
     (requested_lease_ms > (int32_t)CANFD_LEASE_MAX_MS))
  {
    canfd_send_result(command_id, session_id,
                      CANFD_RESULT_STAGE_REJECTED,
                      CANFD_RESULT_OUT_OF_RANGE, 0U);
    return;
  }
  if((canfd_session_valid != 0U) && (session_id != canfd_host_session))
  {
    canfd_send_result(command_id, session_id,
                      CANFD_RESULT_STAGE_REJECTED,
                      CANFD_RESULT_BUSY, 0U);
    return;
  }
  if(session_id != canfd_host_session)
  {
    canfd_last_command_sequence = command_id;
    canfd_last_command_session = session_id;
  }
  canfd_host_session = session_id;
  canfd_session_valid = 1U;
  canfd_lease_duration_ms = (uint16_t)requested_lease_ms;
  canfd_last_lease_ms = canfd_now_ms;
  canfd_lease_counter_valid = 0U;
  canfd_comm_timeout_reported = 0U;
  canfd_send_result(command_id, session_id,
                    CANFD_RESULT_STAGE_ACCEPTED, CANFD_RESULT_OK, 0U);
  canfd_send_result(command_id, session_id,
                    CANFD_RESULT_STAGE_SUCCEEDED, CANFD_RESULT_OK, 0U);
}

static void canfd_parse_command(const canfd_frame_t *frame)
{
  motion_command_t command;
  uint32_t command_id;
  uint32_t session_id;
  uint8_t command_code;
  uint8_t session_required = 1U;
  int32_t arg1;
  int32_t arg3;

  if(frame->data_size != 32U)
  {
    return;
  }
  command_id = canfd_get_u32(&frame->data[0]);
  session_id = canfd_get_u32(&frame->data[4]);
  command_code = frame->data[8];
  if(command_id == 0UL)
  {
    canfd_send_result(command_id, session_id,
                      CANFD_RESULT_STAGE_REJECTED,
                      CANFD_RESULT_DATA_INVALID,
                      MOTION_COMMAND_ERROR_SEQUENCE);
    return;
  }
  if(canfd_command_is_duplicate(frame, command_id, session_id) != 0U)
  {
    return;
  }
  if((command_code == CANFD_COMMAND_CLAIM_CONTROL) &&
     (canfd_session_valid != 0U) &&
     (session_id != canfd_host_session))
  {
    canfd_send_result(command_id, session_id,
                      CANFD_RESULT_STAGE_REJECTED,
                      CANFD_RESULT_BUSY, 0U);
    return;
  }
  canfd_last_command_sequence = command_id;
  canfd_last_command_session = session_id;
  memcpy(canfd_last_command_payload, frame->data, 32U);
  canfd_last_command_valid = 1U;
  canfd_last_result_valid = 0U;

  if((frame->data[9] != 0U) || (canfd_get_u16(&frame->data[10]) != 0U))
  {
    canfd_send_result(command_id, session_id,
                      CANFD_RESULT_STAGE_REJECTED,
                      CANFD_RESULT_DATA_INVALID, 0U);
    return;
  }
  if(command_code == CANFD_COMMAND_CLAIM_CONTROL)
  {
    canfd_accept_claim(command_id, session_id,
                       canfd_get_i32(&frame->data[12]));
    return;
  }

  memset(&command, 0, sizeof(command));
  command.version = MOTION_PROTOCOL_VERSION;
  command.sequence = command_id;
  command.session_id = session_id;
  command.source = MOTION_SOURCE_CANFD;
  if(command_code == CANFD_COMMAND_RELEASE_CONTROL)
  {
    command.type = MOTION_CMD_DISABLE;
  }
  else if(command_code == CANFD_COMMAND_ENABLE_MOTION)
  {
    command.type = MOTION_CMD_ENABLE;
  }
  else if(command_code == CANFD_COMMAND_DISABLE_MOTION)
  {
    command.type = MOTION_CMD_DISABLE;
  }
  else if(command_code == CANFD_COMMAND_GOTO_REL)
  {
    arg3 = canfd_get_i32(&frame->data[24]);
    if((arg3 < 0L) || (arg3 > 65535L) ||
       (canfd_get_u32(&frame->data[28]) == 0UL) ||
       (canfd_get_u32(&frame->data[28]) > CANFD_TASK_TIMEOUT_MAX_MS))
    {
      canfd_send_result(command_id, session_id,
                        CANFD_RESULT_STAGE_REJECTED,
                        CANFD_RESULT_OUT_OF_RANGE, 0U);
      return;
    }
    command.type = MOTION_CMD_GOTO_LOCAL;
    command.data.goto_local.x_mm = canfd_get_i32(&frame->data[12]);
    command.data.goto_local.y_mm = canfd_get_i32(&frame->data[16]);
    command.data.goto_local.yaw_cdeg = canfd_get_i32(&frame->data[20]);
    command.data.goto_local.speed_mm_s = (uint16_t)arg3;
    command.data.goto_local.timeout_ms = canfd_get_u32(&frame->data[28]);
  }
  else if(command_code == CANFD_COMMAND_FOLLOW_MAG_START)
  {
    arg1 = canfd_get_i32(&frame->data[16]);
    arg3 = canfd_get_i32(&frame->data[24]);
    if((arg1 < -32768L) || (arg1 > 32767L) ||
       (arg3 < 0L) || (arg3 > 65535L) ||
       (canfd_get_i32(&frame->data[20]) != 0L) ||
       (canfd_get_u32(&frame->data[28]) == 0UL) ||
       (canfd_get_u32(&frame->data[28]) > CANFD_TASK_TIMEOUT_MAX_MS))
    {
      canfd_send_result(command_id, session_id,
                        CANFD_RESULT_STAGE_REJECTED,
                        CANFD_RESULT_OUT_OF_RANGE, 0U);
      return;
    }
    command.type = MOTION_CMD_FOLLOW_MAG;
    command.data.follow_mag.distance_mm = canfd_get_i32(&frame->data[12]);
    command.data.follow_mag.target_offset_x10mm = (int16_t)arg1;
    command.data.follow_mag.speed_mm_s = (uint16_t)arg3;
    command.data.follow_mag.timeout_ms = canfd_get_u32(&frame->data[28]);
  }
  else if(command_code == CANFD_COMMAND_STOP)
  {
    command.type = MOTION_CMD_STOP;
    session_required = 0U;
  }
  else if(command_code == CANFD_COMMAND_ESTOP)
  {
    command.type = MOTION_CMD_ESTOP;
    session_required = 0U;
  }
  else if(command_code == CANFD_COMMAND_FAULT_RESET)
  {
    command.type = MOTION_CMD_CLEAR_FAULT;
  }
  else if(command_code == CANFD_COMMAND_RESET_ODOM)
  {
    command.type = MOTION_CMD_RESET_ODOM;
  }
  else
  {
    canfd_send_result(command_id, session_id,
                      CANFD_RESULT_STAGE_REJECTED,
                      CANFD_RESULT_UNSUPPORTED_COMMAND,
                      command_code);
    return;
  }

  if((session_required != 0U) &&
     (canfd_session_matches(session_id) == 0U))
  {
    canfd_send_result(command_id, session_id,
                      CANFD_RESULT_STAGE_REJECTED,
                      CANFD_RESULT_NOT_CONTROL_OWNER, 0U);
    return;
  }
  (void)canfd_submit_command(&command, session_required);
  if((command_code == CANFD_COMMAND_RELEASE_CONTROL) &&
     (session_id == canfd_host_session))
  {
    canfd_session_valid = 0U;
    canfd_motion_session_active = 0U;
  }
}

static void canfd_parse_lease(const canfd_frame_t *frame)
{
  uint32_t session_id;
  uint16_t counter;
  uint16_t duration_ms;
  uint16_t delta;

  if(frame->data_size != 12U)
  {
    return;
  }
  session_id = canfd_get_u32(&frame->data[0]);
  counter = canfd_get_u16(&frame->data[4]);
  duration_ms = canfd_get_u16(&frame->data[6]);
  if((canfd_session_matches(session_id) == 0U) ||
     (duration_ms < CANFD_LEASE_MIN_MS) ||
     (duration_ms > CANFD_LEASE_MAX_MS))
  {
    return;
  }
  if(canfd_lease_counter_valid != 0U)
  {
    delta = (uint16_t)(counter - canfd_last_lease_counter);
    if((delta == 0U) || (delta >= 0x8000U))
    {
      return;
    }
  }
  canfd_last_lease_counter = counter;
  canfd_lease_counter_valid = 1U;
  canfd_lease_duration_ms = duration_ms;
  canfd_last_lease_ms = canfd_now_ms;
  canfd_comm_timeout_reported = 0U;
}

static void canfd_parse_velocity(const canfd_frame_t *frame)
{
  uint32_t session_id;
  uint16_t validity_ms;

  if(frame->data_size != 16U)
  {
    return;
  }
  session_id = canfd_get_u32(&frame->data[0]);
  validity_ms = canfd_get_u16(&frame->data[6]);
  if((canfd_session_matches(session_id) == 0U) ||
     (validity_ms < 50U) || (validity_ms > 500U))
  {
    return;
  }
  /* 当前固件不公布速度流能力，收到预留RPDO时保持现有安全输出。 */
}

static void canfd_parse_magnetic(const canfd_frame_t *frame)
{
  canfd_magnetic_sample_t sample;
  uint32_t interrupt_state;
  uint8_t index;

  if(frame->data_size != 48U)
  {
    return;
  }

  memset(&sample, 0, sizeof(sample));
  sample.sample_time_ms = canfd_get_u32(&frame->data[0]);
  sample.receive_time_ms = canfd_now_ms;
  sample.sequence = canfd_get_u16(&frame->data[4]);
  sample.status = frame->data[6];
  sample.flags = frame->data[7];
  sample.active_mask = canfd_get_u16(&frame->data[8]);
  sample.offset_x10mm = canfd_get_i16(&frame->data[10]);
  sample.strength = canfd_get_u16(&frame->data[12]);
  sample.width_x10mm = canfd_get_u16(&frame->data[14]);
  for(index = 0U; index < 8U; index++)
  {
    sample.signal[index] = canfd_get_u16(&frame->data[32U + (index * 2U)]);
  }
  sample.protocol_valid = ((sample.status <= 3U) &&
                           ((sample.flags & 0xF8U) == 0U) &&
                           ((sample.active_mask & 0xFF00U) == 0U)) ? 1U : 0U;
  sample.received = 1U;

  /*
   * CAN任务优先级低于控制任务，发布半帧时若被抢占，读端无法等待写端完成。
   * 极短临界区只覆盖RAM快照复制，避免把合法磁帧误判为持续丢失。
   */
  interrupt_state = __get_PRIMASK();
  __disable_irq();
  canfd_magnetic_generation++;
  __DMB();
  canfd_magnetic_sample = sample;
  __DMB();
  canfd_magnetic_generation++;
  if(interrupt_state == 0UL)
  {
    __enable_irq();
  }
}

static void canfd_parse_frame(const canfd_frame_t *frame)
{
  if(frame->id == CANFD_ID_MAGNETIC_TPDO1)
  {
    canfd_parse_magnetic(frame);
  }
  else if(frame->id == CANFD_ID_CHASSIS_COMMAND)
  {
    canfd_parse_command(frame);
  }
  else if(frame->id == CANFD_ID_CONTROL_LEASE)
  {
    canfd_parse_lease(frame);
  }
  else if(frame->id == CANFD_ID_VELOCITY)
  {
    canfd_parse_velocity(frame);
  }
}

static void canfd_gpio_init(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_3 | GPIO_PINS_4;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE3, GPIO_MUX_11);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE4, GPIO_MUX_11);
}

static void canfd_hardware_init(void)
{
  can_bittime_type bittime;

  canfd_gpio_init();
  crm_periph_clock_enable(CRM_CAN3_PERIPH_CLOCK, TRUE);
  crm_can_clock_select(CRM_CAN3, CRM_CAN_CLOCK_SOURCE_PCLK);
  can_software_reset(CAN3, TRUE);
  can_bittime_default_para_init(&bittime);
  bittime.bittime_div = CANFD_BITTIME_DIV;
  bittime.ac_bts1_size = CANFD_AC_BTS1;
  bittime.ac_bts2_size = CANFD_AC_BTS2;
  bittime.ac_rsaw_size = CANFD_AC_RSAW;
  bittime.fd_bts1_size = CANFD_FD_BTS1;
  bittime.fd_bts2_size = CANFD_FD_BTS2;
  bittime.fd_rsaw_size = CANFD_FD_RSAW;
  bittime.fd_ssp_offset = CANFD_FD_SSP;
  can_bittime_set(CAN3, &bittime);
  can_fd_iso_mode_enable(CAN3, TRUE);
  can_stb_transmit_mode_set(CAN3, CAN_STB_TRANSMIT_BY_FIFO);
  can_software_reset(CAN3, FALSE);
  can_retransmission_limit_set(CAN3, CAN_RE_TRANS_TIMES_UNLIMIT);
  can_rearbitration_limit_set(CAN3, CAN_RE_ARBI_TIMES_UNLIMIT);
  can_rxbuf_overflow_mode_set(CAN3, CAN_RXBUF_OVERFLOW_BE_LOSE);
  can_receive_all_enable(CAN3, FALSE);
  can_mode_set(CAN3, CAN_MODE_COMMUNICATE);
  can_flag_clear(CAN3, CAN_ALL_FLAG);
  can_interrupt_enable(CAN3, CAN_RIE_INT | CAN_ROIE_INT, TRUE);
  nvic_irq_enable(CAN3_RX_IRQn, 3U, 0U);
}

static void canfd_hardware_recover_from_link_fault(void)
{
  uint32_t interrupt_state = __get_PRIMASK();

  nvic_irq_disable(CAN3_RX_IRQn);
  __disable_irq();
  canfd_rx_tail = canfd_rx_head;
  if(interrupt_state == 0U)
  {
    __enable_irq();
  }

  /* 仅写BUSOFF清除位在实测中不能可靠恢复发送，完整重置CAN3状态机。 */
  canfd_tx_inflight = 0U;
  canfd_tx_active_queue = NULL;
  canfd_hardware_init();
}

static void canfd_submit_internal_fault(motion_command_type_t type)
{
  motion_command_t command;

  if(canfd_motion_handler == NULL)
  {
    return;
  }
  memset(&command, 0, sizeof(command));
  command.version = MOTION_PROTOCOL_VERSION;
  command.sequence = ++canfd_internal_sequence;
  if(command.sequence == 0U)
  {
    command.sequence = ++canfd_internal_sequence;
  }
  command.source = MOTION_SOURCE_LOCAL_SAFETY;
  command.type = type;
  (void)canfd_motion_handler(&command, canfd_motion_handler_context);
}

static void canfd_check_timeouts_and_bus(void)
{
  uint8_t busoff = (CAN3->ctrlstat_bit.busoff != 0U) ? 1U : 0U;

  if(busoff != 0U)
  {
    if(canfd_busoff_reported == 0U)
    {
      canfd_busoff_reported = 1U;
      canfd_submit_internal_fault(MOTION_CMD_CAN_BUSOFF);
    }
    if((canfd_now_ms - canfd_last_busoff_recovery_ms) >=
       CANFD_BUSOFF_RECOVERY_PERIOD_MS)
    {
      canfd_last_busoff_recovery_ms = canfd_now_ms;
      can_busoff_reset(CAN3);
      canfd_hardware_recover_from_link_fault();
    }
  }
  else
  {
    canfd_busoff_reported = 0U;
  }

  if((canfd_session_valid != 0U) &&
     ((canfd_now_ms - canfd_last_lease_ms) >
      (uint32_t)canfd_lease_duration_ms))
  {
    canfd_session_valid = 0U;
    canfd_lease_counter_valid = 0U;
    if((canfd_motion_session_active != 0U) &&
       (canfd_comm_timeout_reported == 0U))
    {
      canfd_comm_timeout_reported = 1U;
      canfd_motion_session_active = 0U;
      canfd_submit_internal_fault(MOTION_CMD_COMM_TIMEOUT);
    }
  }
}

static uint8_t canfd_tx_queue_peek(canfd_tx_queue_t *queue,
                                   canfd_frame_t *frame)
{
  uint32_t interrupt_state = __get_PRIMASK();

  if((queue == NULL) || (frame == NULL) || (queue->frames == NULL) ||
     (queue->capacity < 2U))
  {
    return 0U;
  }

  __disable_irq();
  if(queue->tail == queue->head)
  {
    if(interrupt_state == 0U)
    {
      __enable_irq();
    }
    return 0U;
  }
  *frame = queue->frames[queue->tail];
  if(interrupt_state == 0U)
  {
    __enable_irq();
  }
  return 1U;
}

static void canfd_tx_queue_pop(canfd_tx_queue_t *queue)
{
  uint32_t interrupt_state = __get_PRIMASK();

  if((queue == NULL) || (queue->frames == NULL) ||
     (queue->capacity < 2U))
  {
    return;
  }

  __disable_irq();
  if(queue->tail != queue->head)
  {
    queue->tail = (uint8_t)((queue->tail + 1U) %
                            queue->capacity);
  }
  if(interrupt_state == 0U)
  {
    __enable_irq();
  }
}

static void canfd_transmit_one(void)
{
  canfd_frame_t frame;
  can_txbuf_type tx;
  can_transmit_status_type transmit_status;
  canfd_tx_queue_t *queue = &canfd_tx_urgent_queue;

  if(CAN3->ctrlstat_bit.busoff != 0U)
  {
    return;
  }
  if(canfd_tx_inflight != 0U)
  {
    if(can_flag_get(CAN3, CAN_TPIF_FLAG) != RESET)
    {
      can_flag_clear(CAN3, CAN_TPIF_FLAG);
      can_transmit_status_get(CAN3, &transmit_status);
      if((transmit_status.final_handle == canfd_tx_active_handle) &&
         (transmit_status.final_tstat == CAN_TSTAT_TRANSMITTED))
      {
        canfd_tx_queue_pop(canfd_tx_active_queue);
        canfd_tx_count++;
        canfd_tx_inflight = 0U;
        canfd_tx_active_queue = NULL;
        canfd_tx_link_fault_reported = 0U;
        return;
      }
    }

    if((canfd_now_ms - canfd_tx_started_ms) <
       CANFD_TX_COMPLETE_TIMEOUT_MS)
    {
      return;
    }

    can_transmit_abort(CAN3, CAN_ABORT_TRANSMIT_PTB);
    if(canfd_tx_link_fault_reported == 0U)
    {
      canfd_tx_link_fault_reported = 1U;
      canfd_submit_internal_fault(MOTION_CMD_CAN_BUSOFF);
    }
    canfd_hardware_recover_from_link_fault();
    return;
  }
  if(canfd_tx_queue_peek(queue, &frame) == 0U)
  {
    queue = &canfd_tx_normal_queue;
    if(canfd_tx_queue_peek(queue, &frame) == 0U)
    {
      return;
    }
  }

  memset(&tx, 0, sizeof(tx));
  tx.id = frame.id;
  tx.id_type = CAN_ID_STANDARD;
  tx.frame_type = CAN_FRAME_DATA;
  tx.data_length = frame.data_length;
  tx.fd_format = CAN_FORMAT_FD;
  tx.fd_rate_switch = CAN_BRS_ON;
  tx.handle = canfd_tx_handle++;
  memcpy(tx.data, frame.data, frame.data_size);
  if(can_txbuf_write(CAN3, CAN_TXBUF_PTB, &tx) == SUCCESS)
  {
    can_flag_clear(CAN3, CAN_TPIF_FLAG);
    if(can_txbuf_transmit(CAN3, CAN_TRANSMIT_PTB) == SUCCESS)
    {
      canfd_tx_active_handle = tx.handle;
      canfd_tx_active_queue = queue;
      canfd_tx_started_ms = canfd_now_ms;
      canfd_tx_inflight = 1U;
    }
  }
}

void canfd_protocol_init(motion_command_handler_t handler, void *context)
{
  canfd_rx_head = 0U;
  canfd_rx_tail = 0U;
  memset(canfd_tx_urgent_frames, 0, sizeof(canfd_tx_urgent_frames));
  memset(canfd_tx_normal_frames, 0, sizeof(canfd_tx_normal_frames));
  memset(&canfd_tx_urgent_queue, 0, sizeof(canfd_tx_urgent_queue));
  memset(&canfd_tx_normal_queue, 0, sizeof(canfd_tx_normal_queue));
  canfd_tx_urgent_queue.frames = canfd_tx_urgent_frames;
  canfd_tx_urgent_queue.capacity = CANFD_TX_URGENT_QUEUE_LENGTH;
  canfd_tx_normal_queue.frames = canfd_tx_normal_frames;
  canfd_tx_normal_queue.capacity = CANFD_TX_NORMAL_QUEUE_LENGTH;
  canfd_motion_handler = handler;
  canfd_motion_handler_context = context;
  canfd_now_ms = 0UL;
  canfd_host_session = 0UL;
  canfd_last_lease_ms = 0UL;
  canfd_last_busoff_recovery_ms = 0UL;
  canfd_rx_count = 0UL;
  canfd_tx_count = 0UL;
  canfd_rx_drop_count = 0UL;
  canfd_tx_drop_count = 0UL;
  canfd_last_command_sequence = 0U;
  canfd_last_command_session = 0U;
  canfd_internal_sequence = 0U;
  canfd_status_sequence = 0U;
  canfd_lease_duration_ms = CANFD_LEASE_DEFAULT_MS;
  canfd_last_lease_counter = 0U;
  memset(canfd_last_command_payload, 0, sizeof(canfd_last_command_payload));
  memset(canfd_last_result_payload, 0, sizeof(canfd_last_result_payload));
  canfd_last_command_valid = 0U;
  canfd_last_result_valid = 0U;
  canfd_lease_counter_valid = 0U;
  canfd_session_valid = 0U;
  canfd_motion_session_active = 0U;
  canfd_comm_timeout_reported = 0U;
  canfd_busoff_reported = 0U;
  canfd_rx_overflow_latched = 0U;
  canfd_tx_handle = 0U;
  canfd_tx_active_handle = 0U;
  canfd_tx_inflight = 0U;
  canfd_tx_link_fault_reported = 0U;
  canfd_tx_started_ms = 0UL;
  canfd_tx_active_queue = NULL;
  memset(&canfd_magnetic_sample, 0, sizeof(canfd_magnetic_sample));
  canfd_magnetic_generation = 0UL;
  canfd_hardware_init();
}

void canfd_protocol_poll(uint32_t now_ms)
{
  canfd_frame_t frame;
  uint8_t processed = 0U;

  canfd_now_ms = now_ms;
  while((processed < CANFD_RX_QUEUE_LENGTH) &&
        (canfd_rx_queue_pop(&frame) != 0U))
  {
    canfd_parse_frame(&frame);
    processed++;
  }
  canfd_check_timeouts_and_bus();
  canfd_transmit_one();
}

void canfd_protocol_rx_irq_handler(void)
{
  can_rxbuf_type rx;
  uint8_t next_head;
  uint8_t data_size;
  uint8_t index;
  uint8_t guard = 0U;

  while((can_rxbuf_status_get(CAN3) != CAN_RXBUF_STATUS_EMPTY) &&
        (guard < CANFD_RX_QUEUE_LENGTH))
  {
    memset(&rx, 0, sizeof(rx));
    if(can_rxbuf_read(CAN3, &rx) != SUCCESS)
    {
      break;
    }
    data_size = canfd_dlc_to_size(rx.data_length);
    next_head = (uint8_t)((canfd_rx_head + 1U) % CANFD_RX_QUEUE_LENGTH);
    if((next_head == canfd_rx_tail) ||
       (rx.id_type != CAN_ID_STANDARD) ||
       (rx.frame_type != CAN_FRAME_DATA) ||
       (rx.fd_format != CAN_FORMAT_FD) ||
       (rx.fd_rate_switch != CAN_BRS_ON) ||
       (data_size == 0U))
    {
      canfd_rx_drop_count++;
    }
    else
    {
      canfd_rx_queue[canfd_rx_head].id = rx.id;
      canfd_rx_queue[canfd_rx_head].data_length = rx.data_length;
      canfd_rx_queue[canfd_rx_head].data_size = data_size;
      for(index = 0U; index < data_size; index++)
      {
        canfd_rx_queue[canfd_rx_head].data[index] = rx.data[index];
      }
      __DMB();
      canfd_rx_head = next_head;
      canfd_rx_count++;
    }
    guard++;
  }

  if(can_flag_get(CAN3, CAN_ROIF_FLAG) != RESET)
  {
    canfd_rx_overflow_latched = 1U;
    canfd_rx_drop_count++;
  }
  can_flag_clear(CAN3, CAN_RIF_FLAG | CAN_RAFIF_FLAG |
                       CAN_RFIF_FLAG | CAN_ROIF_FLAG);
}

void canfd_protocol_send_motion_result(
    const motion_command_t *command,
    motion_command_handle_result_t result,
    uint16_t fault_mask,
    uint8_t state)
{
  canfd_ack_status_t status;
  motion_command_error_t error = MOTION_COMMAND_ERROR_NONE;

  if((command == NULL) || (command->source != MOTION_SOURCE_CANFD))
  {
    return;
  }
  if(result == MOTION_HANDLE_RECEIVED)
  {
    status = CANFD_ACK_RECEIVED;
  }
  else if(result == MOTION_HANDLE_ACCEPTED)
  {
    status = CANFD_ACK_ACCEPTED;
  }
  else
  {
    status = CANFD_ACK_REJECTED;
    error = (result == MOTION_HANDLE_BUSY) ?
            MOTION_COMMAND_ERROR_BUSY :
            ((fault_mask != 0U) ? MOTION_COMMAND_ERROR_FAULT_LOCKED :
                                  MOTION_COMMAND_ERROR_UNSUPPORTED);
    if(command->type == MOTION_CMD_ENABLE)
    {
      canfd_motion_session_active = 0U;
    }
  }
  canfd_send_ack(command->sequence, command->session_id,
                 command->type, status, error,
                 fault_mask, state);
  if(result == MOTION_HANDLE_ACCEPTED)
  {
    if((command->type == MOTION_CMD_GOTO_LOCAL) ||
       (command->type == MOTION_CMD_FOLLOW_MAG))
    {
      canfd_send_result(command->sequence, command->session_id,
                        CANFD_RESULT_STAGE_EXECUTING,
                        CANFD_RESULT_OK, 0U);
    }
    else
    {
      canfd_send_result(command->sequence, command->session_id,
                        CANFD_RESULT_STAGE_SUCCEEDED,
                        CANFD_RESULT_OK, 0U);
    }
  }
}

void canfd_protocol_send_command_event(
    uint32_t sequence,
    uint32_t session_id,
    motion_command_type_t type,
    canfd_ack_status_t status,
    motion_command_error_t error,
    uint16_t fault_mask,
    uint8_t state)
{
  canfd_send_ack(sequence, session_id, type, status, error,
                 fault_mask, state);
}

void canfd_protocol_send_drive_status(const canfd_drive_status_t *status)
{
  uint8_t data[32] = {0};

  if(status == NULL)
  {
    return;
  }
  data[0] = status->application_state;
  data[1] = status->motion_state;
  data[2] = status->motion_mode;
  data[3] = status->flags;
  canfd_put_u32(&data[4], status->command_id);
  canfd_put_u32(&data[8], status->session_id);
  canfd_put_u32(&data[12], status->fault_mask);
  canfd_put_u32(&data[16], status->warning_mask);
  canfd_put_i32(&data[20], status->x_mm);
  canfd_put_i32(&data[24], status->y_mm);
  canfd_put_i32(&data[28], status->yaw_cdeg);
  (void)canfd_queue_payload(CANFD_ID_CHASSIS_STATE, data, 32U, 0U);
}

void canfd_protocol_send_odom_status(const canfd_odom_status_t *status)
{
  uint8_t data[24] = {0};

  if(status == NULL)
  {
    return;
  }
  canfd_put_u32(&data[0], status->uptime_ms);
  canfd_put_u16(&data[4], ++canfd_status_sequence);
  canfd_put_u16(&data[6], status->valid_flags);
  canfd_put_i32(&data[8], status->x_mm);
  canfd_put_i32(&data[12], status->y_mm);
  canfd_put_i32(&data[16], status->yaw_cdeg);
  canfd_put_i16(&data[20], status->linear_mm_s);
  canfd_put_i16(&data[22], status->yaw_rate_cdeg_s);
  (void)canfd_queue_payload(CANFD_ID_ODOM_STATUS, data, 24U, 0U);
}

void canfd_protocol_send_motor_status(const canfd_motor_status_t *status)
{
  uint8_t data[32] = {0};

  if(status == NULL)
  {
    return;
  }
  canfd_put_u32(&data[0], status->uptime_ms);
  canfd_put_i16(&data[4], status->left_target_rpm_x100);
  canfd_put_i16(&data[6], status->right_target_rpm_x100);
  canfd_put_i16(&data[8], status->left_measured_rpm_x100);
  canfd_put_i16(&data[10], status->right_measured_rpm_x100);
  canfd_put_i16(&data[12], status->left_pwm);
  canfd_put_i16(&data[14], status->right_pwm);
  canfd_put_i32(&data[16], status->left_encoder_count);
  canfd_put_i32(&data[20], status->right_encoder_count);
  data[24] = status->left_invalid_count;
  data[25] = status->right_invalid_count;
  data[26] = status->state;
  (void)canfd_queue_payload(CANFD_ID_MOTOR_DIAGNOSTIC, data, 32U, 0U);
}

void canfd_protocol_send_safety_status(
    const canfd_safety_status_t *status,
    uint8_t urgent)
{
  uint8_t data[16] = {0};

  if(status == NULL)
  {
    return;
  }
  canfd_put_u32(&data[0], status->uptime_ms);
  data[4] = status->bumper_mask;
  data[5] = status->estop_source_mask;
  data[6] = status->safety_state;
  canfd_put_u32(&data[8], status->fault_mask);
  canfd_put_u32(&data[12], status->warning_mask);
  (void)canfd_queue_payload(CANFD_ID_SAFETY_STATUS, data, 16U, urgent);
}

uint16_t canfd_protocol_get_link_age_ms(uint32_t now_ms)
{
  uint32_t age;

  if(canfd_session_valid == 0U)
  {
    return 0xFFFFU;
  }
  age = now_ms - canfd_last_lease_ms;
  return (age > 0xFFFFUL) ? 0xFFFFU : (uint16_t)age;
}

uint32_t canfd_protocol_get_session_id(void)
{
  return (canfd_session_valid != 0U) ? canfd_host_session : 0UL;
}

uint8_t canfd_protocol_is_session_valid(void)
{
  return canfd_session_valid;
}

uint8_t canfd_protocol_get_bus_state_flags(void)
{
  uint8_t flags = 0U;

  if(CAN3->ctrlstat_bit.busoff != 0U)
  {
    flags |= CANFD_BUS_STATE_BUSOFF;
  }
  if(can_flag_get(CAN3, CAN_EPASS_FLAG) != RESET)
  {
    flags |= CANFD_BUS_STATE_ERROR_PASSIVE;
  }
  if(can_flag_get(CAN3, CAN_EWARN_FLAG) != RESET)
  {
    flags |= CANFD_BUS_STATE_ERROR_WARNING;
  }
  if(canfd_rx_overflow_latched != 0U)
  {
    flags |= CANFD_BUS_STATE_RX_OVERFLOW;
  }
  return flags;
}

uint8_t canfd_protocol_is_bus_healthy(void)
{
  if((CAN3->ctrlstat_bit.busoff != 0U) ||
     (can_flag_get(CAN3, CAN_EPASS_FLAG) != RESET) ||
     (can_flag_get(CAN3, CAN_EWARN_FLAG) != RESET))
  {
    return 0U;
  }
  return 1U;
}

uint32_t canfd_protocol_get_rx_count(void)
{
  return canfd_rx_count;
}

uint32_t canfd_protocol_get_tx_count(void)
{
  return canfd_tx_count;
}

uint32_t canfd_protocol_get_rx_drop_count(void)
{
  return canfd_rx_drop_count;
}

uint32_t canfd_protocol_get_tx_drop_count(void)
{
  return canfd_tx_drop_count;
}

uint8_t canfd_protocol_get_magnetic_sample(
    canfd_magnetic_sample_t *sample)
{
  canfd_magnetic_sample_t snapshot;
  uint32_t generation_before;
  uint32_t generation_after;
  uint8_t attempt;

  if(sample == NULL)
  {
    return 0U;
  }
  for(attempt = 0U; attempt < 3U; attempt++)
  {
    generation_before = canfd_magnetic_generation;
    if((generation_before & 1UL) != 0UL)
    {
      continue;
    }
    __DMB();
    snapshot = canfd_magnetic_sample;
    __DMB();
    generation_after = canfd_magnetic_generation;
    if((generation_before == generation_after) &&
       ((generation_after & 1UL) == 0UL))
    {
      if(snapshot.received == 0U)
      {
        return 0U;
      }
      snapshot.age_ms = (uint16_t)(((canfd_now_ms -
                                     snapshot.receive_time_ms) > 65535UL) ?
                                   65535UL :
                                   (canfd_now_ms - snapshot.receive_time_ms));
      *sample = snapshot;
      return 1U;
    }
  }
  return 0U;
}
