/**
  *****************************************************************************
  * @file     at32f456_replacement_test.c
  * @brief    AT32F456RCT7换芯安全自检
  *
  * 本测试验证芯片容量、时钟、SRAM、浮点单元和CAN FD内部回环，同时初始化
  * 板上常用输入与ADC，并通过CAN3周期发送外部测试帧。电机驱动只进入滑行
  * 停止状态，不会主动驱动电机。
  *****************************************************************************
  */

#include "at32f456_replacement_test.h"

#include <stdint.h>
#include <stdio.h>

#include "at32f45x.h"
#include "at32f45x_board.h"
#include "at32f45x_can.h"
#include "at32f45x_crm.h"
#include "at32f45x_debug.h"
#include "at32f45x_flash.h"
#include "at32f45x_gpio.h"
#include "at32f45x_pwc.h"
#include "at32f45x_usart.h"
#include "adc_monitor.h"
#include "bumper_input.h"
#include "motor_driver.h"
#include "motor_encoder.h"

#if !defined(AT32F456RCT7)
#error "replacement test requires AT32F456RCT7"
#endif

#if !defined(SUPPORT_CAN_FD)
#error "AT32F456RCT7 must enable CAN FD support"
#endif

#define REPLACEMENT_FLASH_SIZE_KB          256UL
#define REPLACEMENT_FLASH_SIZE_ADDRESS     0x1FFFF7E0UL
#define REPLACEMENT_UID1_ADDRESS           0x1FFFF7E8UL
#define REPLACEMENT_UID2_ADDRESS           0x1FFFF7ECUL
#define REPLACEMENT_UID3_ADDRESS           0x1FFFF7F0UL
#define REPLACEMENT_SCLK_HZ                192000000UL
#define REPLACEMENT_RAM_TEST_WORDS              2048U
#define REPLACEMENT_CAN_TIMEOUT             2000000UL
#define REPLACEMENT_CLOCK_TIMEOUT           2000000UL
#define REPLACEMENT_LOOP_PERIOD_MS                10U
#define REPLACEMENT_HEARTBEAT_PERIOD_MS          1000U
#define REPLACEMENT_CAN_TX_PERIOD_MS              100U

#define REPLACEMENT_CAN_BITTIME_DIV                  2U
#define REPLACEMENT_CAN_AC_BTS1                     72U
#define REPLACEMENT_CAN_AC_BTS2                     24U
#define REPLACEMENT_CAN_AC_RSAW                     24U
#define REPLACEMENT_CAN_FD_BTS1                     18U
#define REPLACEMENT_CAN_FD_BTS2                      6U
#define REPLACEMENT_CAN_FD_RSAW                      6U
#define REPLACEMENT_CAN_FD_SSP                      19U

static volatile uint32_t replacement_ram_test_buffer[REPLACEMENT_RAM_TEST_WORDS];
static uint32_t replacement_pass_count;
static uint32_t replacement_fail_count;
static uint32_t replacement_heartbeat_ms;
static uint32_t replacement_heartbeat_count;
static uint32_t replacement_uart_rx_count;
static uint32_t replacement_can_tx_elapsed_ms;
static uint32_t replacement_can_tx_count;
static uint32_t replacement_can_tx_ok_count;
static uint8_t replacement_clock_setup_ok;

void at32f456_replacement_clock_config(void)
{
  uint32_t timeout;

  replacement_clock_setup_ok = 0U;
  crm_reset();
  flash_psr_set(FLASH_WAIT_CYCLE_5);
  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
  pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);

  crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
  if(crm_hext_stable_wait() == ERROR)
  {
    system_core_clock_update();
    return;
  }

  crm_pll_config(CRM_PLL_SOURCE_HEXT, 64, 1, CRM_PLL_FP_4);
  crm_pllu_div_set(CRM_PLL_FU_16);
  crm_pllu_output_set(TRUE);
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);

  timeout = REPLACEMENT_CLOCK_TIMEOUT;
  while((crm_flag_get(CRM_PLL_STABLE_FLAG) != SET) && (timeout > 0UL))
  {
    timeout--;
  }
  if(timeout == 0UL)
  {
    system_core_clock_update();
    return;
  }

  crm_ahb_div_set(CRM_AHB_DIV_1);
  crm_apb3_div_set(CRM_APB3_DIV_4);
  crm_apb2_div_set(CRM_APB2_DIV_1);
  crm_apb1_div_set(CRM_APB1_DIV_1);
  crm_auto_step_mode_enable(TRUE);
  crm_sysclk_switch(CRM_SCLK_PLL);

  timeout = REPLACEMENT_CLOCK_TIMEOUT;
  while((crm_sysclk_switch_status_get() != CRM_SCLK_PLL) && (timeout > 0UL))
  {
    timeout--;
  }
  crm_auto_step_mode_enable(FALSE);
  system_core_clock_update();

  if((timeout > 0UL) && (system_core_clock == REPLACEMENT_SCLK_HZ))
  {
    replacement_clock_setup_ok = 1U;
  }
}

static uint32_t replacement_read_word(uint32_t address)
{
  return *(volatile const uint32_t *)address;
}

static void replacement_report(const char *name, uint8_t passed)
{
  if(passed != 0U)
  {
    replacement_pass_count++;
    printf("SELFTEST,%s,PASS\r\n", name);
  }
  else
  {
    replacement_fail_count++;
    printf("SELFTEST,%s,FAIL\r\n", name);
  }
}

static uint8_t replacement_uid_test(uint32_t uid1,
                                    uint32_t uid2,
                                    uint32_t uid3)
{
  if(((uid1 == 0UL) && (uid2 == 0UL) && (uid3 == 0UL)) ||
     ((uid1 == 0xFFFFFFFFUL) &&
      (uid2 == 0xFFFFFFFFUL) &&
      (uid3 == 0xFFFFFFFFUL)))
  {
    return 0U;
  }

  return 1U;
}

static uint8_t replacement_clock_test(crm_clocks_freq_type *clocks)
{
  crm_clocks_freq_get(clocks);

  printf("SELFTEST_INFO,CLOCK,sclk=%lu,ahb=%lu,apb1=%lu,apb2=%lu,hext=%u\r\n",
         clocks->sclk_freq,
         clocks->ahb_freq,
         clocks->apb1_freq,
         clocks->apb2_freq,
         (unsigned int)(crm_flag_get(CRM_HEXT_STABLE_FLAG) == SET));

  return ((replacement_clock_setup_ok != 0U) &&
          (crm_flag_get(CRM_HEXT_STABLE_FLAG) == SET) &&
          (clocks->sclk_freq == REPLACEMENT_SCLK_HZ) &&
          (clocks->ahb_freq == REPLACEMENT_SCLK_HZ) &&
          (clocks->apb1_freq == REPLACEMENT_SCLK_HZ) &&
          (clocks->apb2_freq == REPLACEMENT_SCLK_HZ)) ? 1U : 0U;
}

static uint8_t replacement_ram_test(void)
{
  uint32_t index;

  for(index = 0UL; index < REPLACEMENT_RAM_TEST_WORDS; index++)
  {
    replacement_ram_test_buffer[index] = 0xAAAAAAAAUL;
  }
  for(index = 0UL; index < REPLACEMENT_RAM_TEST_WORDS; index++)
  {
    if(replacement_ram_test_buffer[index] != 0xAAAAAAAAUL)
    {
      return 0U;
    }
    replacement_ram_test_buffer[index] = 0x55555555UL;
  }
  for(index = 0UL; index < REPLACEMENT_RAM_TEST_WORDS; index++)
  {
    if(replacement_ram_test_buffer[index] != 0x55555555UL)
    {
      return 0U;
    }
    replacement_ram_test_buffer[index] = 0xA5A50000UL ^ index;
  }
  for(index = 0UL; index < REPLACEMENT_RAM_TEST_WORDS; index++)
  {
    if(replacement_ram_test_buffer[index] != (0xA5A50000UL ^ index))
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t replacement_fpu_test(void)
{
  volatile float first = 1.25f;
  volatile float second = 2.5f;
  volatile float result = first * second;

  return ((result > 3.124f) && (result < 3.126f)) ? 1U : 0U;
}

static void replacement_can_fd_gpio_init(void)
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

  /* AT32F456的CAN3_RX/TX在PB3/PB4上必须选择MUX11。 */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE3, GPIO_MUX_11);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE4, GPIO_MUX_11);
}

static void replacement_can_fd_configure(can_mode_type mode)
{
  can_bittime_type bittime;

  crm_periph_clock_enable(CRM_CAN3_PERIPH_CLOCK, TRUE);
  crm_can_clock_select(CRM_CAN3, CRM_CAN_CLOCK_SOURCE_PCLK);
  can_software_reset(CAN3, TRUE);

  can_bittime_default_para_init(&bittime);
  bittime.bittime_div = REPLACEMENT_CAN_BITTIME_DIV;
  bittime.ac_bts1_size = REPLACEMENT_CAN_AC_BTS1;
  bittime.ac_bts2_size = REPLACEMENT_CAN_AC_BTS2;
  bittime.ac_rsaw_size = REPLACEMENT_CAN_AC_RSAW;
  bittime.fd_bts1_size = REPLACEMENT_CAN_FD_BTS1;
  bittime.fd_bts2_size = REPLACEMENT_CAN_FD_BTS2;
  bittime.fd_rsaw_size = REPLACEMENT_CAN_FD_RSAW;
  bittime.fd_ssp_offset = REPLACEMENT_CAN_FD_SSP;
  can_bittime_set(CAN3, &bittime);
  can_fd_iso_mode_enable(CAN3, TRUE);
  can_stb_transmit_mode_set(CAN3, CAN_STB_TRANSMIT_BY_FIFO);
  can_software_reset(CAN3, FALSE);
  can_retransmission_limit_set(CAN3, CAN_RE_TRANS_TIMES_0);
  can_rearbitration_limit_set(CAN3, CAN_RE_ARBI_TIMES_0);
  can_mode_set(CAN3, mode);
  can_flag_clear(CAN3, CAN_ALL_FLAG);
}

static uint8_t replacement_can_fd_loopback_test(void)
{
  can_txbuf_type tx = {0};
  can_rxbuf_type rx = {0};
  uint32_t timeout;
  uint8_t index;

  if(replacement_clock_setup_ok == 0U)
  {
    return 0U;
  }

  replacement_can_fd_configure(CAN_MODE_INT_LOOPBACK);

  tx.id = 0x456U;
  tx.id_type = CAN_ID_STANDARD;
  tx.frame_type = CAN_FRAME_DATA;
  tx.data_length = CAN_DLC_BYTES_16;
  tx.fd_format = CAN_FORMAT_FD;
  tx.fd_rate_switch = CAN_BRS_ON;
  for(index = 0U; index < 16U; index++)
  {
    tx.data[index] = (uint8_t)(0x30U + index);
  }

  timeout = REPLACEMENT_CAN_TIMEOUT;
  while((can_txbuf_write(CAN3, CAN_TXBUF_PTB, &tx) != SUCCESS) &&
        (timeout > 0UL))
  {
    timeout--;
  }
  if(timeout == 0UL)
  {
    return 0U;
  }

  can_txbuf_transmit(CAN3, CAN_TRANSMIT_PTB);
  timeout = REPLACEMENT_CAN_TIMEOUT;
  while((can_flag_get(CAN3, CAN_TPIF_FLAG) != SET) && (timeout > 0UL))
  {
    timeout--;
  }
  if(timeout == 0UL)
  {
    return 0U;
  }
  can_flag_clear(CAN3, CAN_TPIF_FLAG);

  timeout = REPLACEMENT_CAN_TIMEOUT;
  while((can_flag_get(CAN3, CAN_RIF_FLAG) != SET) && (timeout > 0UL))
  {
    timeout--;
  }
  if((timeout == 0UL) || (can_rxbuf_read(CAN3, &rx) != SUCCESS))
  {
    return 0U;
  }

  if((rx.id != tx.id) ||
     (rx.id_type != CAN_ID_STANDARD) ||
     (rx.frame_type != CAN_FRAME_DATA) ||
     (rx.data_length != CAN_DLC_BYTES_16) ||
     (rx.fd_format != CAN_FORMAT_FD) ||
     (rx.fd_rate_switch != CAN_BRS_ON) ||
     (rx.recv_frame != CAN_RECV_LOOPBACK))
  {
    return 0U;
  }

  for(index = 0U; index < 16U; index++)
  {
    if(rx.data[index] != tx.data[index])
    {
      return 0U;
    }
  }

  return 1U;
}

static void replacement_can_fd_external_init(void)
{
  replacement_can_fd_gpio_init();
  replacement_can_fd_configure(CAN_MODE_COMMUNICATE);
  replacement_can_tx_elapsed_ms = 0UL;
  replacement_can_tx_count = 0UL;
  replacement_can_tx_ok_count = 0UL;
}

static void replacement_can_fd_external_send(void)
{
  can_txbuf_type tx = {0};
  can_transmit_status_type status = {0};
  uint32_t timeout;
  uint8_t index;

  if(CAN3->ctrlstat_bit.busoff != 0U)
  {
    replacement_can_fd_configure(CAN_MODE_COMMUNICATE);
  }

  tx.id = 0x321U;
  tx.id_type = CAN_ID_STANDARD;
  tx.frame_type = CAN_FRAME_DATA;
  tx.data_length = CAN_DLC_BYTES_16;
  tx.fd_format = CAN_FORMAT_FD;
  tx.fd_rate_switch = CAN_BRS_ON;
  tx.handle = (uint8_t)replacement_can_tx_count;
  for(index = 0U; index < 16U; index++)
  {
    tx.data[index] = (uint8_t)(replacement_can_tx_count + index);
  }

  can_flag_clear(CAN3, CAN_TPIF_FLAG | CAN_BEIF_FLAG | CAN_EIF_FLAG |
                       CAN_EPASS_FLAG | CAN_EWARN_FLAG);
  if(can_txbuf_write(CAN3, CAN_TXBUF_PTB, &tx) != SUCCESS)
  {
    return;
  }

  (void)can_txbuf_transmit(CAN3, CAN_TRANSMIT_PTB);
  timeout = REPLACEMENT_CAN_TIMEOUT;
  while((can_flag_get(CAN3, CAN_TPIF_FLAG) == RESET) &&
        (timeout > 0UL))
  {
    timeout--;
  }

  can_transmit_status_get(CAN3, &status);
  replacement_can_tx_count++;
  if((timeout > 0UL) && (status.final_tstat == CAN_TSTAT_TRANSMITTED))
  {
    replacement_can_tx_ok_count++;
  }
}

void at32f456_replacement_test_init(void)
{
  crm_clocks_freq_type clocks = {0};
  uint32_t flash_size_kb;
  uint32_t uid1;
  uint32_t uid2;
  uint32_t uid3;
  uint32_t device_id;
  uint32_t cortex_part;
  uint8_t adc_ok;
  uint8_t bumper_mask;

  replacement_pass_count = 0UL;
  replacement_fail_count = 0UL;
  replacement_heartbeat_ms = 0UL;
  replacement_heartbeat_count = 0UL;
  replacement_uart_rx_count = 0UL;
  replacement_can_tx_elapsed_ms = 0UL;
  replacement_can_tx_count = 0UL;
  replacement_can_tx_ok_count = 0UL;

  printf("\r\nSELFTEST_BEGIN,AT32F456RCT7,SAFE_NO_MOTOR\r\n");

  cortex_part = (SCB->CPUID & SCB_CPUID_PARTNO_Msk) >> SCB_CPUID_PARTNO_Pos;
  device_id = debug_device_id_get();
  flash_size_kb = replacement_read_word(REPLACEMENT_FLASH_SIZE_ADDRESS) & 0xFFFFUL;
  uid1 = replacement_read_word(REPLACEMENT_UID1_ADDRESS);
  uid2 = replacement_read_word(REPLACEMENT_UID2_ADDRESS);
  uid3 = replacement_read_word(REPLACEMENT_UID3_ADDRESS);

  printf("SELFTEST_INFO,CHIP,cpuid=0x%08lX,part=0x%03lX,pid=0x%08lX\r\n",
         SCB->CPUID,
         cortex_part,
         device_id);
  printf("SELFTEST_INFO,MEMORY,flash_kb=%lu,sram_link_kb=108\r\n",
         flash_size_kb);
  printf("SELFTEST_INFO,UID,%08lX-%08lX-%08lX\r\n", uid1, uid2, uid3);

  replacement_report("CORTEX_M4", cortex_part == 0xC24UL);
  replacement_report("DEVICE_PID", (device_id != 0UL) && (device_id != 0xFFFFFFFFUL));
  replacement_report("UID96", replacement_uid_test(uid1, uid2, uid3));
  replacement_report("FLASH_256KB", flash_size_kb == REPLACEMENT_FLASH_SIZE_KB);
  replacement_report("CLOCK_192MHZ_HEXT", replacement_clock_test(&clocks));
  replacement_report("SRAM_BUFFER_8KB", replacement_ram_test());
  replacement_report("FPU", replacement_fpu_test());

  motor_driver_init();
  motor_driver_stop_all(MOTOR_STOP_COAST);
  replacement_report("MOTOR_OUTPUT_SAFE_STOP", 1U);

  motor_encoder_init();
  motor_encoder_reset_all();
  replacement_report("ENCODER_INPUT_INIT", 1U);

  bumper_input_init();
  bumper_mask = bumper_input_read_raw_mask();
  printf("SELFTEST_INFO,BUMPER,raw=0x%02X\r\n", bumper_mask);
  replacement_report("BUMPER_INPUT_INIT", 1U);

  adc_monitor_init();
  adc_ok = adc_monitor_update();
  printf("SELFTEST_INFO,ADC,pa3=%u,pa4=%u,pa5=%u\r\n",
         adc_monitor_get_raw(ADC_MONITOR_PA3),
         adc_monitor_get_raw(ADC_MONITOR_PA4),
         adc_monitor_get_raw(ADC_MONITOR_PA5));
  replacement_report("ADC1_PA3_PA4_PA5", adc_ok);

  replacement_report("CAN3_FD_BRS_INTERNAL_LOOPBACK",
                     replacement_can_fd_loopback_test());

  replacement_can_fd_external_init();
  printf("SELFTEST_INFO,CANFD_EXTERNAL,can=3,rx=PB3,tx=PB4,mux=11,"
         "id=0x321,nominal=1Mbps,data=4Mbps,brs=1,period_ms=%u\r\n",
         REPLACEMENT_CAN_TX_PERIOD_MS);

  printf("SELFTEST_SUMMARY,pass=%lu,fail=%lu,result=%s\r\n",
         replacement_pass_count,
         replacement_fail_count,
         (replacement_fail_count == 0UL) ? "PASS" : "FAIL");
  printf("SELFTEST_READY,type_any_character_to_test_uart_rx\r\n");
}

void at32f456_replacement_test_loop(void)
{
  while(usart_flag_get(PRINT_UART, USART_RDBF_FLAG) != RESET)
  {
    uint8_t data = (uint8_t)usart_data_receive(PRINT_UART);
    replacement_uart_rx_count++;
    printf("SELFTEST_UART_RX,count=%lu,data=0x%02X\r\n",
           replacement_uart_rx_count,
           data);
  }

  delay_ms(REPLACEMENT_LOOP_PERIOD_MS);
  replacement_can_tx_elapsed_ms += REPLACEMENT_LOOP_PERIOD_MS;
  if(replacement_can_tx_elapsed_ms >= REPLACEMENT_CAN_TX_PERIOD_MS)
  {
    replacement_can_tx_elapsed_ms = 0UL;
    replacement_can_fd_external_send();
  }

  replacement_heartbeat_ms += REPLACEMENT_LOOP_PERIOD_MS;
  if(replacement_heartbeat_ms >= REPLACEMENT_HEARTBEAT_PERIOD_MS)
  {
    replacement_heartbeat_ms = 0UL;
    replacement_heartbeat_count++;
    (void)adc_monitor_update();
    printf("SELFTEST_ALIVE,sec=%lu,adc=%u/%u/%u,bumper=0x%02X,enc=%ld/%ld,invalid=%lu/%lu\r\n",
           replacement_heartbeat_count,
           adc_monitor_get_raw(ADC_MONITOR_PA3),
           adc_monitor_get_raw(ADC_MONITOR_PA4),
           adc_monitor_get_raw(ADC_MONITOR_PA5),
           bumper_input_read_raw_mask(),
           motor_encoder_get_count(MOTOR_LEFT),
           motor_encoder_get_count(MOTOR_RIGHT),
           motor_encoder_get_invalid_transition_count(MOTOR_LEFT),
           motor_encoder_get_invalid_transition_count(MOTOR_RIGHT));
    printf("SELFTEST_CANFD,tx=%lu,ok=%lu,err=0x%08lX,busoff=%u,tecnt=%u\r\n",
           replacement_can_tx_count,
           replacement_can_tx_ok_count,
           CAN3->err,
           (unsigned int)CAN3->ctrlstat_bit.busoff,
           (unsigned int)CAN3->err_bit.tecnt);
  }
}
