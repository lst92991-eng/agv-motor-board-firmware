/**
 * @file canfd_protocol.c
 * @brief CAN3链路、底盘线协议和控制租约
 */

#include "canfd_protocol.h"

#include <stddef.h>
#include <string.h>

#include "at32f45x.h"
#include "at32f45x_can.h"
#include "at32f45x_crm.h"
#include "at32f45x_gpio.h"
#include "at32f45x_misc.h"
#include "canfd_status_codec.h"
#include "motion_reference_receiver.h"

#define CANFD_BITTIME_DIV                 2U
#define CANFD_AC_BTS1                   144U
#define CANFD_AC_BTS2                    48U
#define CANFD_AC_RSAW                    24U
#define CANFD_FD_BTS1                    18U
#define CANFD_FD_BTS2                     6U
#define CANFD_FD_RSAW                     6U
#define CANFD_FD_SSP                     19U
#define CANFD_RX_CAPACITY                 8U
#define CANFD_TX_CAPACITY                12U
#define CANFD_LEASE_MIN_MS              100U
#define CANFD_LEASE_DEFAULT_MS          200U
#define CANFD_LEASE_MAX_MS             1000U
#define CANFD_BUSOFF_RECOVERY_MS       1000U
#define CANFD_TX_TIMEOUT_MS              50U

#define CANFD_STAGE_RECEIVED              0U
#define CANFD_STAGE_ACCEPTED              1U
#define CANFD_STAGE_SUCCEEDED             3U
#define CANFD_STAGE_REJECTED              4U
#define CANFD_RESULT_OK                  0x00U
#define CANFD_RESULT_UNSUPPORTED         0x01U
#define CANFD_RESULT_INVALID_STATE       0x02U
#define CANFD_RESULT_OUT_OF_RANGE        0x03U
#define CANFD_RESULT_BUSY                0x04U
#define CANFD_RESULT_NOT_OWNER           0x05U
#define CANFD_RESULT_DATA_INVALID        0x08U
#define CANFD_RESULT_DUPLICATE_MISMATCH  0x09U

typedef struct
{
  uint32_t order;
  uint32_t receive_ms;
  uint32_t id;
  uint8_t size;
  uint8_t data[32];
} canfd_frame_t;

typedef struct
{
  canfd_frame_t frame[CANFD_TX_CAPACITY];
  volatile uint8_t head;
  volatile uint8_t tail;
} canfd_tx_queue_t;

typedef struct
{
  uint32_t command_id;
  uint32_t session_id;
  uint8_t command[32];
  uint8_t result[16];
  uint8_t valid;
  uint8_t result_valid;
} canfd_replay_t;

typedef struct
{
  uint8_t code;
  motion_command_type_t type;
  uint8_t session_required;
} canfd_command_map_t;

static const canfd_command_map_t canfd_commands[] =
{
  {CANFD_COMMAND_RELEASE_CONTROL, MOTION_CMD_RELEASE, 1U},
  {CANFD_COMMAND_ENABLE_MOTION, MOTION_CMD_ENABLE, 1U},
  {CANFD_COMMAND_DISABLE_MOTION, MOTION_CMD_DISABLE, 1U},
  {CANFD_COMMAND_STOP, MOTION_CMD_STOP, 0U},
  {CANFD_COMMAND_ESTOP, MOTION_CMD_ESTOP, 0U},
  {CANFD_COMMAND_FAULT_RESET, MOTION_CMD_CLEAR_FAULT, 1U},
  {CANFD_COMMAND_RESET_ODOM, MOTION_CMD_RESET_ODOM, 1U}
};

static canfd_frame_t canfd_rx[CANFD_RX_CAPACITY];
static canfd_tx_queue_t canfd_tx_urgent;
static canfd_tx_queue_t canfd_tx_normal;
static volatile uint8_t canfd_rx_head;
static volatile uint8_t canfd_rx_tail;
static volatile uint32_t canfd_arrival_order;
static volatile uint32_t canfd_last_command_order;
static volatile uint32_t canfd_processed_command_order;
static volatile uint8_t canfd_rx_overflow;
static motion_command_handler_t canfd_handler;
static void *canfd_handler_context;
static motion_reference_receiver_t canfd_reference;
static motion_reference_ingress_mailbox_t canfd_reference_mailbox;
static canfd_replay_t canfd_replay;
static uint32_t canfd_now_ms;
static uint32_t canfd_session_id;
static uint32_t canfd_last_lease_ms;
static uint32_t canfd_last_busoff_recovery_ms;
static uint32_t canfd_internal_sequence;
static uint32_t canfd_tx_started_ms;
static uint16_t canfd_lease_ms;
static uint16_t canfd_last_lease_counter;
static uint16_t canfd_status_sequence;
static uint8_t canfd_session_valid;
static uint8_t canfd_motion_enabled;
static uint8_t canfd_lease_counter_valid;
static uint8_t canfd_timeout_reported;
static uint8_t canfd_busoff_reported;
static uint8_t canfd_tx_inflight;
static uint8_t canfd_tx_handle;
static uint8_t canfd_tx_active_handle;
static canfd_tx_queue_t *canfd_tx_active_queue;

static uint8_t canfd_dlc_size(can_data_length_type dlc)
{
  static const uint8_t sizes[16] =
  {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
    8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
  };
  return ((uint8_t)dlc < 16U) ? sizes[(uint8_t)dlc] : 0U;
}

static can_data_length_type canfd_dlc(uint8_t size)
{
  if(size == 12U) return CAN_DLC_BYTES_12;
  if(size == 16U) return CAN_DLC_BYTES_16;
  if(size == 32U) return CAN_DLC_BYTES_32;
  return CAN_DLC_BYTES_0;
}

static uint16_t canfd_get_u16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t canfd_get_u32(const uint8_t *data)
{
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
      ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static void canfd_put_u16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
}

static void canfd_put_u32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
  data[2] = (uint8_t)(value >> 16U);
  data[3] = (uint8_t)(value >> 24U);
}

static uint32_t canfd_lock(void)
{
  uint32_t state = __get_PRIMASK();
  __disable_irq();
  return state;
}

static void canfd_unlock(uint32_t state)
{
  if(state == 0U) __enable_irq();
}

static uint8_t canfd_tx_push(canfd_tx_queue_t *queue,
                             uint32_t id,
                             const uint8_t *data,
                             uint8_t size)
{
  uint32_t irq;
  uint8_t next;
  can_data_length_type dlc = canfd_dlc(size);

  if((queue == NULL) || (data == NULL) || (dlc == CAN_DLC_BYTES_0)) return 0U;
  irq = canfd_lock();
  next = (uint8_t)((queue->head + 1U) % CANFD_TX_CAPACITY);
  if(next == queue->tail)
  {
    canfd_unlock(irq);
    return 0U;
  }
  queue->frame[queue->head].id = id;
  queue->frame[queue->head].size = size;
  memcpy(queue->frame[queue->head].data, data, size);
  queue->head = next;
  canfd_unlock(irq);
  return 1U;
}

static uint8_t canfd_tx_peek(canfd_tx_queue_t *queue, canfd_frame_t *frame)
{
  uint32_t irq = canfd_lock();
  if(queue->tail == queue->head)
  {
    canfd_unlock(irq);
    return 0U;
  }
  *frame = queue->frame[queue->tail];
  canfd_unlock(irq);
  return 1U;
}

static void canfd_tx_pop(canfd_tx_queue_t *queue)
{
  uint32_t irq = canfd_lock();
  if(queue->tail != queue->head)
  {
    queue->tail = (uint8_t)((queue->tail + 1U) % CANFD_TX_CAPACITY);
  }
  canfd_unlock(irq);
}

static uint8_t canfd_rx_pop(canfd_frame_t *frame)
{
  uint32_t irq = canfd_lock();
  if(canfd_rx_tail == canfd_rx_head)
  {
    canfd_unlock(irq);
    return 0U;
  }
  *frame = canfd_rx[canfd_rx_tail];
  canfd_rx_tail = (uint8_t)((canfd_rx_tail + 1U) % CANFD_RX_CAPACITY);
  canfd_unlock(irq);
  return 1U;
}

static void canfd_cache_command(const canfd_frame_t *frame,
                                uint32_t command_id,
                                uint32_t session_id)
{
  uint32_t irq = canfd_lock();
  canfd_replay.command_id = command_id;
  canfd_replay.session_id = session_id;
  memcpy(canfd_replay.command, frame->data, 32U);
  canfd_replay.valid = 1U;
  canfd_replay.result_valid = 0U;
  canfd_unlock(irq);
}

static void canfd_emit_result(uint32_t command_id,
                              uint32_t session_id,
                              uint8_t stage,
                              uint8_t result,
                              uint16_t detail,
                              uint8_t cache)
{
  uint8_t payload[16] = {0};
  uint32_t irq;

  canfd_put_u32(&payload[0], command_id);
  canfd_put_u32(&payload[4], session_id);
  payload[8] = stage;
  payload[9] = result;
  canfd_put_u16(&payload[10], detail);
  canfd_put_u32(&payload[12], canfd_now_ms);
  if(cache != 0U)
  {
    irq = canfd_lock();
    if((canfd_replay.valid != 0U) &&
       (canfd_replay.command_id == command_id) &&
       (canfd_replay.session_id == session_id))
    {
      memcpy(canfd_replay.result, payload, sizeof(payload));
      canfd_replay.result_valid = 1U;
    }
    canfd_unlock(irq);
  }
  (void)canfd_tx_push(&canfd_tx_urgent, CANFD_ID_COMMAND_RESULT,
                      payload, sizeof(payload));
}

static uint8_t canfd_error_code(motion_command_error_t error)
{
  if(error == MOTION_COMMAND_ERROR_NONE) return CANFD_RESULT_OK;
  if(error == MOTION_COMMAND_ERROR_BUSY) return CANFD_RESULT_BUSY;
  if(error == MOTION_COMMAND_ERROR_UNSUPPORTED) return CANFD_RESULT_UNSUPPORTED;
  if(error == MOTION_COMMAND_ERROR_FAULT_LOCKED) return CANFD_RESULT_INVALID_STATE;
  return CANFD_RESULT_DATA_INVALID;
}

static uint8_t canfd_duplicate(const canfd_frame_t *frame,
                               uint32_t command_id,
                               uint32_t session_id)
{
  canfd_replay_t snapshot;
  uint32_t irq = canfd_lock();
  snapshot = canfd_replay;
  canfd_unlock(irq);

  if((snapshot.valid == 0U) || (snapshot.session_id != session_id)) return 0U;
  if(command_id < snapshot.command_id)
  {
    canfd_emit_result(command_id, session_id, CANFD_STAGE_REJECTED,
        CANFD_RESULT_DATA_INVALID, MOTION_COMMAND_ERROR_SEQUENCE, 0U);
    return 1U;
  }
  if(command_id != snapshot.command_id) return 0U;
  if(memcmp(frame->data, snapshot.command, 32U) != 0)
  {
    canfd_emit_result(command_id, session_id, CANFD_STAGE_REJECTED,
        CANFD_RESULT_DUPLICATE_MISMATCH, 0U, 0U);
  }
  else if(snapshot.result_valid != 0U)
  {
    (void)canfd_tx_push(&canfd_tx_urgent, CANFD_ID_COMMAND_RESULT,
                        snapshot.result, sizeof(snapshot.result));
  }
  else
  {
    canfd_emit_result(command_id, session_id, CANFD_STAGE_RECEIVED,
                      CANFD_RESULT_OK, 0U, 0U);
  }
  return 1U;
}

static const canfd_command_map_t *canfd_find_command(uint8_t code)
{
  uint8_t index;
  for(index = 0U; index < sizeof(canfd_commands) / sizeof(canfd_commands[0]);
      index++)
  {
    if(canfd_commands[index].code == code) return &canfd_commands[index];
  }
  return NULL;
}

static void canfd_accept_claim(uint32_t command_id,
                               uint32_t session_id,
                               uint32_t lease_ms)
{
  if((session_id == 0U) || (lease_ms < CANFD_LEASE_MIN_MS) ||
     (lease_ms > CANFD_LEASE_MAX_MS))
  {
    canfd_emit_result(command_id, session_id, CANFD_STAGE_REJECTED,
                      CANFD_RESULT_OUT_OF_RANGE, 0U, 1U);
    return;
  }
  if((canfd_session_valid != 0U) && (session_id != canfd_session_id))
  {
    canfd_emit_result(command_id, session_id, CANFD_STAGE_REJECTED,
                      CANFD_RESULT_BUSY, 0U, 1U);
    return;
  }
  canfd_session_id = session_id;
  canfd_session_valid = 1U;
  canfd_motion_enabled = 0U;
  canfd_lease_ms = (uint16_t)lease_ms;
  canfd_last_lease_ms = canfd_now_ms;
  canfd_lease_counter_valid = 0U;
  canfd_timeout_reported = 0U;
  motion_reference_receiver_reset(&canfd_reference, session_id);
  canfd_emit_result(command_id, session_id, CANFD_STAGE_ACCEPTED,
                    CANFD_RESULT_OK, 0U, 1U);
  canfd_emit_result(command_id, session_id, CANFD_STAGE_SUCCEEDED,
                    CANFD_RESULT_OK, 0U, 1U);
}

static void canfd_parse_command(const canfd_frame_t *frame)
{
  const canfd_command_map_t *map;
  motion_command_t command = {0};
  motion_command_handle_result_t handled;
  uint32_t command_id;
  uint32_t session_id;
  uint8_t code;

  if(frame->size != 32U) return;
  command_id = canfd_get_u32(&frame->data[0]);
  session_id = canfd_get_u32(&frame->data[4]);
  code = frame->data[8];
  if(command_id == 0U) return;
  if(canfd_duplicate(frame, command_id, session_id) != 0U) return;
  canfd_cache_command(frame, command_id, session_id);
  if((frame->data[9] != 0U) || (canfd_get_u16(&frame->data[10]) != 0U))
  {
    canfd_emit_result(command_id, session_id, CANFD_STAGE_REJECTED,
                      CANFD_RESULT_DATA_INVALID, 0U, 1U);
    return;
  }
  if(code == CANFD_COMMAND_CLAIM_CONTROL)
  {
    canfd_accept_claim(command_id, session_id,
                       canfd_get_u32(&frame->data[12]));
    return;
  }
  map = canfd_find_command(code);
  if(map == NULL)
  {
    canfd_emit_result(command_id, session_id, CANFD_STAGE_REJECTED,
                      CANFD_RESULT_UNSUPPORTED, code, 1U);
    return;
  }
  if((map->session_required != 0U) &&
     ((canfd_session_valid == 0U) || (session_id != canfd_session_id)))
  {
    canfd_emit_result(command_id, session_id, CANFD_STAGE_REJECTED,
                      CANFD_RESULT_NOT_OWNER, 0U, 1U);
    return;
  }
  command.version = MOTION_PROTOCOL_VERSION;
  command.sequence = command_id;
  command.session_id = session_id;
  command.source = MOTION_SOURCE_CANFD;
  command.type = map->type;
  if(canfd_handler == NULL)
  {
    canfd_emit_result(command_id, session_id, CANFD_STAGE_REJECTED,
        CANFD_RESULT_UNSUPPORTED, MOTION_COMMAND_ERROR_UNSUPPORTED, 1U);
    return;
  }
  handled = canfd_handler(&command, canfd_handler_context);
  if((handled == MOTION_HANDLE_RECEIVED) || (handled == MOTION_HANDLE_ACCEPTED))
  {
    if((command.type == MOTION_CMD_ESTOP) ||
       (command.type == MOTION_CMD_DISABLE) ||
       (command.type == MOTION_CMD_RELEASE))
    {
      canfd_motion_enabled = 0U;
    }
    if(command.type == MOTION_CMD_STOP)
    {
      motion_reference_receiver_invalidate_latest(&canfd_reference);
    }
    canfd_emit_result(command_id, session_id,
        (handled == MOTION_HANDLE_RECEIVED) ? CANFD_STAGE_RECEIVED :
                                              CANFD_STAGE_ACCEPTED,
        CANFD_RESULT_OK, 0U, 1U);
  }
  else
  {
    canfd_protocol_send_motion_result(&command, handled, 0U, 0U);
  }
}

static void canfd_parse_lease(const canfd_frame_t *frame)
{
  uint32_t session_id;
  uint16_t counter;
  uint16_t lease_ms;
  uint16_t delta;

  if(frame->size != 12U) return;
  session_id = canfd_get_u32(&frame->data[0]);
  counter = canfd_get_u16(&frame->data[4]);
  lease_ms = canfd_get_u16(&frame->data[6]);
  if((canfd_session_valid == 0U) || (session_id != canfd_session_id) ||
     (lease_ms < CANFD_LEASE_MIN_MS) || (lease_ms > CANFD_LEASE_MAX_MS)) return;
  if(canfd_lease_counter_valid != 0U)
  {
    delta = (uint16_t)(counter - canfd_last_lease_counter);
    if((delta == 0U) || (delta >= 0x8000U)) return;
  }
  canfd_last_lease_counter = counter;
  canfd_lease_counter_valid = 1U;
  canfd_lease_ms = lease_ms;
  canfd_last_lease_ms = canfd_now_ms;
  canfd_timeout_reported = 0U;
}

static void canfd_parse_reference(const motion_reference_ingress_t *ingress)
{
  uint32_t elapsed;

  if((canfd_session_valid == 0U) || (canfd_motion_enabled == 0U) ||
     (motion_reference_is_after(ingress->arrival_order,
                                canfd_last_command_order) == 0U)) return;
  elapsed = canfd_now_ms - canfd_last_lease_ms;
  if(elapsed >= canfd_lease_ms) return;
  (void)motion_reference_receiver_accept(&canfd_reference, ingress->payload,
      AGV_MOTION_REFERENCE_PAYLOAD_SIZE, ingress->receive_ms,
      (uint16_t)(canfd_lease_ms - elapsed));
}

static void canfd_gpio_init(void)
{
  gpio_init_type gpio;
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio);
  gpio.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio.gpio_mode = GPIO_MODE_MUX;
  gpio.gpio_pins = GPIO_PINS_3 | GPIO_PINS_4;
  gpio.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE3, GPIO_MUX_11);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE4, GPIO_MUX_11);
}

static void canfd_hardware_init(void)
{
  can_bittime_type timing;
  canfd_gpio_init();
  crm_periph_clock_enable(CRM_CAN3_PERIPH_CLOCK, TRUE);
  crm_can_clock_select(CRM_CAN3, CRM_CAN_CLOCK_SOURCE_PCLK);
  can_software_reset(CAN3, TRUE);
  can_bittime_default_para_init(&timing);
  timing.bittime_div = CANFD_BITTIME_DIV;
  timing.ac_bts1_size = CANFD_AC_BTS1;
  timing.ac_bts2_size = CANFD_AC_BTS2;
  timing.ac_rsaw_size = CANFD_AC_RSAW;
  timing.fd_bts1_size = CANFD_FD_BTS1;
  timing.fd_bts2_size = CANFD_FD_BTS2;
  timing.fd_rsaw_size = CANFD_FD_RSAW;
  timing.fd_ssp_offset = CANFD_FD_SSP;
  can_bittime_set(CAN3, &timing);
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

static void canfd_recover(void)
{
  uint32_t irq;
  nvic_irq_disable(CAN3_RX_IRQn);
  irq = canfd_lock();
  canfd_rx_tail = canfd_rx_head;
  canfd_unlock(irq);
  canfd_tx_inflight = 0U;
  canfd_tx_active_queue = NULL;
  canfd_hardware_init();
}

static void canfd_internal_fault(motion_command_type_t type)
{
  motion_command_t command = {0};
  if(canfd_handler == NULL) return;
  command.version = MOTION_PROTOCOL_VERSION;
  command.sequence = ++canfd_internal_sequence;
  if(command.sequence == 0U) command.sequence = ++canfd_internal_sequence;
  command.source = MOTION_SOURCE_LOCAL_SAFETY;
  command.type = type;
  (void)canfd_handler(&command, canfd_handler_context);
}

static void canfd_check_link(void)
{
  uint8_t busoff = (CAN3->ctrlstat_bit.busoff != 0U) ? 1U : 0U;

  if((busoff != 0U) || (canfd_rx_overflow != 0U))
  {
    if(canfd_busoff_reported == 0U)
    {
      canfd_busoff_reported = 1U;
      canfd_motion_enabled = 0U;
      canfd_session_valid = 0U;
      motion_reference_receiver_reset(&canfd_reference, 0U);
      canfd_internal_fault(MOTION_CMD_CAN_BUSOFF);
    }
    if((canfd_now_ms - canfd_last_busoff_recovery_ms) >=
       CANFD_BUSOFF_RECOVERY_MS)
    {
      canfd_last_busoff_recovery_ms = canfd_now_ms;
      canfd_rx_overflow = 0U;
      can_busoff_reset(CAN3);
      canfd_recover();
    }
  }
  else
  {
    canfd_busoff_reported = 0U;
  }

  if((canfd_session_valid != 0U) &&
     ((canfd_now_ms - canfd_last_lease_ms) >= canfd_lease_ms))
  {
    uint8_t was_enabled = canfd_motion_enabled;
    canfd_session_valid = 0U;
    canfd_motion_enabled = 0U;
    canfd_lease_counter_valid = 0U;
    motion_reference_receiver_reset(&canfd_reference, 0U);
    if((was_enabled != 0U) && (canfd_timeout_reported == 0U))
    {
      canfd_timeout_reported = 1U;
      canfd_internal_fault(MOTION_CMD_COMM_TIMEOUT);
    }
  }
}

static void canfd_transmit(void)
{
  canfd_frame_t frame;
  can_txbuf_type tx;
  can_transmit_status_type status;
  canfd_tx_queue_t *queue = &canfd_tx_urgent;

  if(CAN3->ctrlstat_bit.busoff != 0U) return;
  if(canfd_tx_inflight != 0U)
  {
    if(can_flag_get(CAN3, CAN_TPIF_FLAG) != RESET)
    {
      can_flag_clear(CAN3, CAN_TPIF_FLAG);
      can_transmit_status_get(CAN3, &status);
      if((status.final_handle == canfd_tx_active_handle) &&
         (status.final_tstat == CAN_TSTAT_TRANSMITTED))
      {
        canfd_tx_pop(canfd_tx_active_queue);
        canfd_tx_inflight = 0U;
        canfd_tx_active_queue = NULL;
        return;
      }
    }
    if((canfd_now_ms - canfd_tx_started_ms) < CANFD_TX_TIMEOUT_MS) return;
    can_transmit_abort(CAN3, CAN_ABORT_TRANSMIT_PTB);
    canfd_motion_enabled = 0U;
    canfd_internal_fault(MOTION_CMD_CAN_BUSOFF);
    canfd_recover();
    return;
  }
  if(canfd_tx_peek(queue, &frame) == 0U)
  {
    queue = &canfd_tx_normal;
    if(canfd_tx_peek(queue, &frame) == 0U) return;
  }
  memset(&tx, 0, sizeof(tx));
  tx.id = frame.id;
  tx.id_type = CAN_ID_STANDARD;
  tx.frame_type = CAN_FRAME_DATA;
  tx.data_length = canfd_dlc(frame.size);
  tx.fd_format = CAN_FORMAT_FD;
  tx.fd_rate_switch = CAN_BRS_ON;
  tx.handle = canfd_tx_handle++;
  memcpy(tx.data, frame.data, frame.size);
  if((can_txbuf_write(CAN3, CAN_TXBUF_PTB, &tx) == SUCCESS) &&
     (can_txbuf_transmit(CAN3, CAN_TRANSMIT_PTB) == SUCCESS))
  {
    canfd_tx_active_handle = tx.handle;
    canfd_tx_active_queue = queue;
    canfd_tx_started_ms = canfd_now_ms;
    canfd_tx_inflight = 1U;
  }
}

void canfd_protocol_init(motion_command_handler_t handler, void *context)
{
  memset(canfd_rx, 0, sizeof(canfd_rx));
  memset(&canfd_tx_urgent, 0, sizeof(canfd_tx_urgent));
  memset(&canfd_tx_normal, 0, sizeof(canfd_tx_normal));
  memset(&canfd_replay, 0, sizeof(canfd_replay));
  canfd_rx_head = canfd_rx_tail = 0U;
  canfd_handler = handler;
  canfd_handler_context = context;
  canfd_now_ms = 0U;
  canfd_arrival_order = 0U;
  canfd_last_command_order = 0U;
  canfd_processed_command_order = 0U;
  canfd_session_id = 0U;
  canfd_session_valid = 0U;
  canfd_motion_enabled = 0U;
  canfd_lease_ms = CANFD_LEASE_DEFAULT_MS;
  canfd_lease_counter_valid = 0U;
  canfd_timeout_reported = 0U;
  canfd_busoff_reported = 0U;
  canfd_rx_overflow = 0U;
  canfd_tx_inflight = 0U;
  canfd_tx_active_queue = NULL;
  canfd_status_sequence = 0U;
  motion_reference_receiver_reset(&canfd_reference, 0U);
  motion_reference_ingress_mailbox_reset(&canfd_reference_mailbox);
  canfd_hardware_init();
}

void canfd_protocol_poll(uint32_t now_ms)
{
  canfd_frame_t frame;
  motion_reference_ingress_t reference;
  uint8_t reference_available;

  canfd_now_ms = now_ms;
  reference_available = motion_reference_ingress_mailbox_take(
      &canfd_reference_mailbox, &reference);
  while(canfd_rx_pop(&frame) != 0U)
  {
    if(frame.id == CANFD_ID_CHASSIS_COMMAND)
    {
      motion_reference_receiver_invalidate_latest(&canfd_reference);
      canfd_parse_command(&frame);
      canfd_processed_command_order = frame.order;
    }
    else if(frame.id == CANFD_ID_CONTROL_LEASE)
    {
      canfd_parse_lease(&frame);
    }
  }
  if((reference_available != 0U) &&
     (canfd_processed_command_order == canfd_last_command_order))
  {
    canfd_parse_reference(&reference);
  }
  canfd_check_link();
  canfd_transmit();
}

void canfd_protocol_rx_irq_handler(void)
{
  can_rxbuf_type rx;
  uint8_t guard = 0U;

  while((can_rxbuf_status_get(CAN3) != CAN_RXBUF_STATUS_EMPTY) &&
        (guard++ < CANFD_RX_CAPACITY))
  {
    uint8_t size;
    uint32_t order;
    uint8_t next;

    memset(&rx, 0, sizeof(rx));
    if(can_rxbuf_read(CAN3, &rx) != SUCCESS) break;
    size = canfd_dlc_size(rx.data_length);
    if((rx.id_type != CAN_ID_STANDARD) ||
       (rx.frame_type != CAN_FRAME_DATA) ||
       (rx.fd_format != CAN_FORMAT_FD) ||
       (rx.fd_rate_switch != CAN_BRS_ON) || (size == 0U)) continue;
    if((rx.id != CANFD_ID_CHASSIS_COMMAND) &&
       (rx.id != CANFD_ID_CONTROL_LEASE) &&
       (rx.id != CANFD_ID_MOTION_REFERENCE)) continue;
    order = ++canfd_arrival_order;
    if(order == 0U) order = ++canfd_arrival_order;
    if(rx.id == CANFD_ID_MOTION_REFERENCE)
    {
      if(size == AGV_MOTION_REFERENCE_PAYLOAD_SIZE)
      {
        motion_reference_ingress_mailbox_publish(&canfd_reference_mailbox,
            rx.data, canfd_now_ms, order);
      }
      continue;
    }
    if(rx.id == CANFD_ID_CHASSIS_COMMAND) canfd_last_command_order = order;
    next = (uint8_t)((canfd_rx_head + 1U) % CANFD_RX_CAPACITY);
    if(next == canfd_rx_tail)
    {
      canfd_rx_overflow = 1U;
      continue;
    }
    canfd_rx[canfd_rx_head].order = order;
    canfd_rx[canfd_rx_head].receive_ms = canfd_now_ms;
    canfd_rx[canfd_rx_head].id = rx.id;
    canfd_rx[canfd_rx_head].size = size;
    memcpy(canfd_rx[canfd_rx_head].data, rx.data, size);
    __DMB();
    canfd_rx_head = next;
  }
  if(can_flag_get(CAN3, CAN_ROIF_FLAG) != RESET) canfd_rx_overflow = 1U;
  can_flag_clear(CAN3, CAN_RIF_FLAG | CAN_RAFIF_FLAG |
                       CAN_RFIF_FLAG | CAN_ROIF_FLAG);
}

void canfd_protocol_send_motion_result(const motion_command_t *command,
    motion_command_handle_result_t result, uint16_t fault_mask, uint8_t state)
{
  motion_command_error_t error;
  (void)state;
  if((command == NULL) || (command->source != MOTION_SOURCE_CANFD)) return;
  error = (result == MOTION_HANDLE_BUSY) ? MOTION_COMMAND_ERROR_BUSY :
      ((fault_mask != 0U) ? MOTION_COMMAND_ERROR_FAULT_LOCKED :
                            MOTION_COMMAND_ERROR_UNSUPPORTED);
  if((result == MOTION_HANDLE_RECEIVED) || (result == MOTION_HANDLE_ACCEPTED))
  {
    error = MOTION_COMMAND_ERROR_NONE;
  }
  canfd_emit_result(command->sequence, command->session_id,
      (result == MOTION_HANDLE_ACCEPTED) ? CANFD_STAGE_ACCEPTED :
      ((result == MOTION_HANDLE_RECEIVED) ? CANFD_STAGE_RECEIVED :
                                           CANFD_STAGE_REJECTED),
      canfd_error_code(error), (error == MOTION_COMMAND_ERROR_NONE) ?
      fault_mask : (uint16_t)error, 1U);
  if(result != MOTION_HANDLE_ACCEPTED) return;
  canfd_emit_result(command->sequence, command->session_id,
                    CANFD_STAGE_SUCCEEDED, CANFD_RESULT_OK, 0U, 1U);
  if(command->type == MOTION_CMD_ENABLE) canfd_motion_enabled = 1U;
  if((command->type == MOTION_CMD_DISABLE) ||
     (command->type == MOTION_CMD_ESTOP)) canfd_motion_enabled = 0U;
  if(command->type == MOTION_CMD_RELEASE)
  {
    canfd_motion_enabled = 0U;
    canfd_session_valid = 0U;
    canfd_lease_counter_valid = 0U;
    motion_reference_receiver_reset(&canfd_reference, 0U);
  }
}

static void canfd_send_status(uint32_t id, const uint8_t *payload,
                              uint8_t size, uint8_t urgent)
{
  (void)canfd_tx_push((urgent != 0U) ? &canfd_tx_urgent : &canfd_tx_normal,
                      id, payload, size);
}

void canfd_protocol_send_drive_status(const canfd_drive_status_t *status)
{
  uint8_t data[CANFD_DRIVE_STATUS_PAYLOAD_SIZE];
  if(canfd_status_encode_drive(status, data, sizeof(data)) != 0U)
    canfd_send_status(CANFD_ID_CHASSIS_STATE, data, sizeof(data), 0U);
}

void canfd_protocol_send_odom_status(const canfd_odom_status_t *status)
{
  uint8_t data[CANFD_ODOM_STATUS_PAYLOAD_SIZE];
  if(canfd_status_encode_odom(status, ++canfd_status_sequence,
                              data, sizeof(data)) != 0U)
    canfd_send_status(CANFD_ID_ODOM_STATUS, data, sizeof(data), 0U);
}

void canfd_protocol_send_safety_status(const canfd_safety_status_t *status,
                                       uint8_t urgent)
{
  uint8_t data[CANFD_SAFETY_STATUS_PAYLOAD_SIZE];
  if(canfd_status_encode_safety(status, data, sizeof(data)) != 0U)
    canfd_send_status(CANFD_ID_SAFETY_STATUS, data, sizeof(data), urgent);
}

uint32_t canfd_protocol_get_session_id(void)
{
  return (canfd_session_valid != 0U) ? canfd_session_id : 0U;
}

uint8_t canfd_protocol_is_session_valid(void)
{
  return canfd_session_valid;
}

uint8_t canfd_protocol_is_bus_healthy(void)
{
  return ((CAN3->ctrlstat_bit.busoff == 0U) &&
          (can_flag_get(CAN3, CAN_EPASS_FLAG) == RESET) &&
          (canfd_rx_overflow == 0U)) ? 1U : 0U;
}

uint8_t canfd_protocol_get_motion_reference(agv_motion_reference_t *reference)
{
  if((reference == NULL) || (canfd_session_valid == 0U) ||
     (canfd_motion_enabled == 0U) ||
     (canfd_processed_command_order != canfd_last_command_order) ||
     (CAN3->ctrlstat_bit.busoff != 0U)) return 0U;
  return motion_reference_receiver_get_latest(&canfd_reference,
                                               canfd_now_ms, reference);
}
