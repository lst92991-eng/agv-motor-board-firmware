/**
  *****************************************************************************
  * @file     canfd_test.c
  * @brief    FDCAN3 CAN FD持续发送测试
  *
  * 开发板实际连接为PB3=FDCAN3_RX、PB4=FDCAN3_TX，外接TJA1051T/3。程序
  * 只使用正常通信模式持续发送16字节CAN FD帧，不启动电机，也不处理接收。
  *****************************************************************************
  */

#include "canfd_test.h"

#include <stdint.h>
#include <stdio.h>

#include "at32f45x_board.h"
#include "at32f45x_can.h"
#include "at32f45x_crm.h"
#include "at32f45x_gpio.h"

#define CANFD_TEST_TIMEOUT                  2000000UL
#define CANFD_TEST_PERIOD_MS                      100U

/* APB1为192MHz，仲裁段为1Mbps，数据段为4Mbps，适配TJA1051T/3。 */
#define CANFD_TEST_BITTIME_DIV                    2U
#define CANFD_TEST_AC_BTS1                       72U
#define CANFD_TEST_AC_BTS2                       24U
#define CANFD_TEST_AC_RSAW                       24U
#define CANFD_TEST_FD_BTS1                       18U
#define CANFD_TEST_FD_BTS2                        6U
#define CANFD_TEST_FD_RSAW                        6U
#define CANFD_TEST_FD_SSP                        19U

/* 临时硬件隔离测试：1表示把PB4改成普通GPIO方波，不启动CAN3。 */
#define CANFD_TEST_GPIO_PROBE                     0U

static uint32_t canfd_test_time_ms;
static uint32_t canfd_test_tx_count;
static uint32_t canfd_test_tx_ok_count;

static void canfd_test_gpio_init(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = (CANFD_TEST_GPIO_PROBE != 0U) ?
                                GPIO_MODE_OUTPUT : GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = (CANFD_TEST_GPIO_PROBE != 0U) ?
                                GPIO_PINS_4 : (GPIO_PINS_3 | GPIO_PINS_4);
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

#if (CANFD_TEST_GPIO_PROBE == 0U)
  /* AT32F456的CAN3在PB3/PB4使用MUX11。 */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE3, GPIO_MUX_11);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE4, GPIO_MUX_11);
#else
  /* 方波测试先把PB4拉高，避免上电瞬间出现不确定电平。 */
  gpio_bits_set(GPIOB, GPIO_PINS_4);
#endif
}

static void canfd_test_configure(can_mode_type mode)
{
  can_bittime_type bittime;

  crm_periph_clock_enable(CRM_CAN3_PERIPH_CLOCK, TRUE);
  crm_can_clock_select(CRM_CAN3, CRM_CAN_CLOCK_SOURCE_PCLK);
  can_software_reset(CAN3, TRUE);

  can_bittime_default_para_init(&bittime);
  bittime.bittime_div = CANFD_TEST_BITTIME_DIV;
  bittime.ac_bts1_size = CANFD_TEST_AC_BTS1;
  bittime.ac_bts2_size = CANFD_TEST_AC_BTS2;
  bittime.ac_rsaw_size = CANFD_TEST_AC_RSAW;
  bittime.fd_bts1_size = CANFD_TEST_FD_BTS1;
  bittime.fd_bts2_size = CANFD_TEST_FD_BTS2;
  bittime.fd_rsaw_size = CANFD_TEST_FD_RSAW;
  bittime.fd_ssp_offset = CANFD_TEST_FD_SSP;
  can_bittime_set(CAN3, &bittime);
  can_fd_iso_mode_enable(CAN3, TRUE);
  can_stb_transmit_mode_set(CAN3, CAN_STB_TRANSMIT_BY_FIFO);
  can_software_reset(CAN3, FALSE);
  can_retransmission_limit_set(CAN3, CAN_RE_TRANS_TIMES_0);
  can_rearbitration_limit_set(CAN3, CAN_RE_ARBI_TIMES_0);
  can_mode_set(CAN3, mode);
  can_flag_clear(CAN3, CAN_ALL_FLAG);
}

static void canfd_test_make_frame(can_txbuf_type *frame, uint32_t sequence)
{
  uint8_t index;

  frame->id = 0x321U;
  frame->id_type = CAN_ID_STANDARD;
  frame->frame_type = CAN_FRAME_DATA;
  frame->data_length = CAN_DLC_BYTES_16;
  frame->fd_format = CAN_FORMAT_FD;
  frame->fd_rate_switch = CAN_BRS_ON;
  frame->handle = (uint8_t)sequence;
  for(index = 0U; index < 16U; index++)
  {
    frame->data[index] = (uint8_t)(sequence + index);
  }
}

static void canfd_test_external_send(void)
{
  can_txbuf_type tx = {0};
  can_transmit_status_type status = {0};
  uint32_t timeout;

  canfd_test_make_frame(&tx, canfd_test_tx_count);
  can_flag_clear(CAN3, CAN_TPIF_FLAG | CAN_BEIF_FLAG | CAN_EIF_FLAG |
                       CAN_EPASS_FLAG | CAN_EWARN_FLAG);

  if(can_txbuf_write(CAN3, CAN_TXBUF_PTB, &tx) != SUCCESS)
  {
    printf("CANFD_TX,seq=%lu,write=FAIL,err=0x%08lX\r\n",
           canfd_test_tx_count,
           CAN3->err);
    return;
  }

  (void)can_txbuf_transmit(CAN3, CAN_TRANSMIT_PTB);
  timeout = CANFD_TEST_TIMEOUT;
  while((can_flag_get(CAN3, CAN_TPIF_FLAG) == RESET) &&
        (timeout > 0UL))
  {
    timeout--;
  }

  can_transmit_status_get(CAN3, &status);
  canfd_test_tx_count++;
  if((timeout > 0UL) && (status.final_tstat == CAN_TSTAT_TRANSMITTED))
  {
    canfd_test_tx_ok_count++;
  }

  printf("CANFD_TX,seq=%lu,done=%u,tstat=%u,ok=%lu,err=0x%08lX,"
         "busoff=%u,tecnt=%u\r\n",
         canfd_test_tx_count - 1UL,
         (unsigned int)(timeout > 0UL),
         (unsigned int)status.final_tstat,
         canfd_test_tx_ok_count,
         CAN3->err,
         (unsigned int)CAN3->ctrlstat_bit.busoff,
         (unsigned int)CAN3->err_bit.tecnt);

}

void canfd_test_init(void)
{
  canfd_test_gpio_init();
  canfd_test_time_ms = 0UL;
  canfd_test_tx_count = 0UL;
  canfd_test_tx_ok_count = 0UL;
#if (CANFD_TEST_GPIO_PROBE == 0U)
  printf("\r\nCANFD_TX_ONLY,FDCAN3,PB3_RX,PB4_TX,TJA1051T3\r\n");
  printf("CANFD_BITRATE,arbitration=1Mbps,data=4Mbps,frame=16bytes,BRS=ON\r\n");

  /* 仅配置正常通信模式，外部接收设备需要与本板共地并正确终端。 */
  canfd_test_configure(CAN_MODE_COMMUNICATE);
  printf("CANFD_TX_READY,ID=0x321,PERIOD_MS=100\r\n");
#else
  printf("\r\nPB4_GPIO_PROBE,frequency=1kHz,duty=50%%\r\n");
#endif
}

void canfd_test_loop(void)
{
#if (CANFD_TEST_GPIO_PROBE != 0U)
  gpio_bits_toggle(GPIOB, GPIO_PINS_4);
  delay_us(500U);
#else
  delay_ms(CANFD_TEST_PERIOD_MS);
  canfd_test_time_ms += CANFD_TEST_PERIOD_MS;
  canfd_test_external_send();
#endif
}
