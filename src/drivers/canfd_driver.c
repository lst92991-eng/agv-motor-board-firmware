/**
 * @file canfd_driver.c
 * @brief CAN3的500k/4M BRS配置、短RX队列和非阻塞发送。
 */
#include "canfd_driver.h"

#include <string.h>

#include "at32f45x.h"
#include "at32f45x_can.h"
#include "at32f45x_crm.h"
#include "at32f45x_gpio.h"
#include "at32f45x_misc.h"

#define CANFD_RX_COUNT 4U
#define CANFD_TX_TIMEOUT_MS 50U

static canfd_frame_t canfd_rx[CANFD_RX_COUNT];
static volatile uint8_t canfd_head;
static volatile uint8_t canfd_tail;
static volatile uint8_t canfd_overflow;
static uint8_t canfd_tx_busy;
static uint8_t canfd_tx_handle;
static uint32_t canfd_tx_started_ms;
static uint32_t canfd_now_ms;

static uint8_t canfd_dlc_size(can_data_length_type dlc)
{
  static const uint8_t size[16] = {0U,1U,2U,3U,4U,5U,6U,7U,8U,12U,16U,20U,24U,32U,48U,64U};
  return ((uint8_t)dlc < 16U) ? size[(uint8_t)dlc] : 0U;
}

static can_data_length_type canfd_dlc(uint8_t size)
{
  if(size <= 8U) return (can_data_length_type)size;
  if(size == 12U) return CAN_DLC_BYTES_12;
  if(size == 16U) return CAN_DLC_BYTES_16;
  if(size == 20U) return CAN_DLC_BYTES_20;
  if(size == 24U) return CAN_DLC_BYTES_24;
  return CAN_DLC_BYTES_32;
}

void canfd_driver_init(void)
{
  gpio_init_type gpio;
  can_bittime_type timing;

  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_CAN3_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio);
  gpio.gpio_pins = GPIO_PINS_3 | GPIO_PINS_4;
  gpio.gpio_mode = GPIO_MODE_MUX;
  gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio.gpio_pull = GPIO_PULL_NONE;
  gpio.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOB, &gpio);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE3, GPIO_MUX_11);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE4, GPIO_MUX_11);
  crm_can_clock_select(CRM_CAN3, CRM_CAN_CLOCK_SOURCE_PCLK);
  can_software_reset(CAN3, TRUE);
  can_bittime_default_para_init(&timing);
  timing.bittime_div = 2U;
  timing.ac_bts1_size = 144U;
  timing.ac_bts2_size = 48U;
  timing.ac_rsaw_size = 24U;
  timing.fd_bts1_size = 18U;
  timing.fd_bts2_size = 6U;
  timing.fd_rsaw_size = 6U;
  timing.fd_ssp_offset = 19U;
  can_bittime_set(CAN3, &timing);
  can_fd_iso_mode_enable(CAN3, TRUE);
  can_stb_transmit_mode_set(CAN3, CAN_STB_TRANSMIT_BY_FIFO);
  can_software_reset(CAN3, FALSE);
  can_receive_all_enable(CAN3, FALSE);
  can_mode_set(CAN3, CAN_MODE_COMMUNICATE);
  can_interrupt_enable(CAN3, CAN_RIE_INT | CAN_ROIE_INT, TRUE);
  can_flag_clear(CAN3, CAN_ALL_FLAG);
  canfd_head = canfd_tail = canfd_overflow = 0U;
  canfd_tx_busy = 0U;
  nvic_irq_enable(CAN3_RX_IRQn, 3U, 0U);
}

void canfd_driver_poll(uint32_t now_ms)
{
  canfd_now_ms = now_ms;
  if((canfd_tx_busy != 0U) && (can_flag_get(CAN3, CAN_TPIF_FLAG) != RESET))
  {
    can_flag_clear(CAN3, CAN_TPIF_FLAG);
    canfd_tx_busy = 0U;
  }
  if((canfd_tx_busy != 0U) && ((now_ms - canfd_tx_started_ms) >= CANFD_TX_TIMEOUT_MS))
  {
    can_transmit_abort(CAN3, CAN_ABORT_TRANSMIT_PTB);
    canfd_tx_busy = 0U;
    canfd_overflow = 1U;
  }
}

void canfd_driver_rx_irq(void)
{
  can_rxbuf_type rx;
  uint8_t next;
  uint8_t size;

  while(can_rxbuf_status_get(CAN3) != CAN_RXBUF_STATUS_EMPTY)
  {
    memset(&rx, 0, sizeof(rx));
    if(can_rxbuf_read(CAN3, &rx) != SUCCESS) break;
    size = canfd_dlc_size(rx.data_length);
    if((rx.id_type != CAN_ID_STANDARD) || (rx.frame_type != CAN_FRAME_DATA) ||
       (rx.fd_format != CAN_FORMAT_FD) || (rx.fd_rate_switch != CAN_BRS_ON) ||
       (size == 0U) || (size > sizeof(canfd_rx[0].data))) continue;
    next = (uint8_t)((canfd_head + 1U) % CANFD_RX_COUNT);
    if(next == canfd_tail)
    {
      canfd_overflow = 1U;
      continue;
    }
    canfd_rx[canfd_head].id = rx.id;
    canfd_rx[canfd_head].size = size;
    memcpy(canfd_rx[canfd_head].data, rx.data, size);
    __DMB();
    canfd_head = next;
  }
  if(can_flag_get(CAN3, CAN_ROIF_FLAG) != RESET) canfd_overflow = 1U;
  can_flag_clear(CAN3, CAN_RIF_FLAG | CAN_RAFIF_FLAG | CAN_RFIF_FLAG | CAN_ROIF_FLAG);
}

uint8_t canfd_driver_take(canfd_frame_t *frame)
{
  uint32_t irq;
  if(frame == 0) return 0U;
  irq = __get_PRIMASK();
  __disable_irq();
  if(canfd_tail == canfd_head)
  {
    if(irq == 0U) __enable_irq();
    return 0U;
  }
  *frame = canfd_rx[canfd_tail];
  canfd_tail = (uint8_t)((canfd_tail + 1U) % CANFD_RX_COUNT);
  if(irq == 0U) __enable_irq();
  return 1U;
}

uint8_t canfd_driver_send(uint32_t id, const uint8_t *data, uint8_t size)
{
  can_txbuf_type tx;
  if((data == 0) || (size == 0U) || (size > 32U) || (canfd_tx_busy != 0U) ||
     (canfd_driver_healthy() == 0U)) return 0U;
  memset(&tx, 0, sizeof(tx));
  tx.id = id;
  tx.id_type = CAN_ID_STANDARD;
  tx.frame_type = CAN_FRAME_DATA;
  tx.data_length = canfd_dlc(size);
  tx.fd_format = CAN_FORMAT_FD;
  tx.fd_rate_switch = CAN_BRS_ON;
  tx.handle = canfd_tx_handle++;
  memcpy(tx.data, data, size);
  if((can_txbuf_write(CAN3, CAN_TXBUF_PTB, &tx) != SUCCESS) ||
     (can_txbuf_transmit(CAN3, CAN_TRANSMIT_PTB) != SUCCESS)) return 0U;
  canfd_tx_busy = 1U;
  canfd_tx_started_ms = canfd_now_ms;
  return 1U;
}

uint8_t canfd_driver_healthy(void)
{
  return ((CAN3->ctrlstat_bit.busoff == 0U) &&
          (can_flag_get(CAN3, CAN_EPASS_FLAG) == RESET) &&
          (canfd_overflow == 0U)) ? 1U : 0U;
}

void canfd_driver_recover(void)
{
  canfd_driver_init();
}
