/**
  *****************************************************************************
  * @file     pn532_uart5.c
  * @brief    PN532 HSU协议和AT32 USART5轮询驱动
  *
  * PN532模块固定在HSU串口模式，主控PB5为USART5_RX，PB6为USART5_TX。
  * 当前版本用于独立读卡测试，后续可把底层收发替换成中断或DMA环形缓冲。
  *****************************************************************************
  */

#include "pn532_uart5.h"

#include <string.h>

#include "at32f45x_board.h"
#include "at32f45x_conf.h"

#define PN532_PREAMBLE                         0x00U
#define PN532_START_CODE_1                     0x00U
#define PN532_START_CODE_2                     0xFFU
#define PN532_HOST_TO_PN532                    0xD4U
#define PN532_PN532_TO_HOST                    0xD5U
#define PN532_COMMAND_GET_FIRMWARE             0x02U
#define PN532_COMMAND_SAM_CONFIGURATION        0x14U
#define PN532_COMMAND_IN_LIST_PASSIVE_TARGET   0x4AU
#define PN532_COMMAND_IN_DATA_EXCHANGE         0x40U
#define PN532_MIFARE_AUTHENTICATE_A            0x60U
#define PN532_MIFARE_READ                      0x30U
#define PN532_UART5_BAUDRATE                   115200U
#define PN532_UART_BYTE_TIMEOUT_MS             20U
#define PN532_RESPONSE_TIMEOUT_MS              1200U
#define PN532_FRAME_MAX_SIZE                   64U

static uint8_t pn532_uart5_receive_byte(uint8_t *data, uint32_t timeout_ms)
{
  uint32_t loops = timeout_ms * 100U;

  while(loops > 0U)
  {
    if(usart_flag_get(USART5, USART_RDBF_FLAG) != RESET)
    {
      *data = (uint8_t)usart_data_receive(USART5);
      return 1U;
    }
    delay_us(10U);
    loops--;
  }

  return 0U;
}

static void pn532_uart5_flush_rx(void)
{
  while(usart_flag_get(USART5, USART_RDBF_FLAG) != RESET)
  {
    (void)usart_data_receive(USART5);
  }
}

static void pn532_uart5_send(const uint8_t *data, uint16_t length)
{
  uint16_t index;

  for(index = 0U; index < length; index++)
  {
    while(usart_flag_get(USART5, USART_TDBE_FLAG) == RESET)
    {
    }
    usart_data_transmit(USART5, data[index]);
  }
  while(usart_flag_get(USART5, USART_TDC_FLAG) == RESET)
  {
  }
}

static pn532_result_t pn532_uart5_read_frame(uint8_t *data,
                                              uint8_t *length,
                                              uint32_t timeout_ms)
{
  uint8_t value;
  uint8_t state = 0U;
  uint8_t frame_length;
  uint8_t length_checksum;
  uint8_t data_checksum;
  uint8_t index;

  for(;;)
  {
    if(pn532_uart5_receive_byte(&value, timeout_ms) == 0U)
    {
      return PN532_RESULT_TIMEOUT;
    }
    if(state == 0U)
    {
      state = (value == PN532_PREAMBLE) ? 1U : 0U;
    }
    else if(state == 1U)
    {
      if(value == PN532_START_CODE_1)
      {
        state = 2U;
      }
      else
      {
        state = (value == PN532_PREAMBLE) ? 1U : 0U;
      }
    }
    else
    {
      if(value == PN532_START_CODE_2)
      {
        break;
      }
      state = (value == PN532_PREAMBLE) ? 1U : 0U;
    }
  }

  if(pn532_uart5_receive_byte(&frame_length, PN532_UART_BYTE_TIMEOUT_MS) == 0U ||
     pn532_uart5_receive_byte(&length_checksum,
                              PN532_UART_BYTE_TIMEOUT_MS) == 0U)
  {
    return PN532_RESULT_TIMEOUT;
  }

  /* PN532 ACK帧为00 FF 00，NACK帧为FF 00 00。 */
  if((frame_length == 0U) && (length_checksum == 0xFFU))
  {
    if(pn532_uart5_receive_byte(&value, PN532_UART_BYTE_TIMEOUT_MS) == 0U)
    {
      return PN532_RESULT_TIMEOUT;
    }
    *length = 0U;
    return PN532_RESULT_OK;
  }
  if((frame_length == 0xFFU) && (length_checksum == 0x00U))
  {
    if(pn532_uart5_receive_byte(&value, PN532_UART_BYTE_TIMEOUT_MS) == 0U ||
       pn532_uart5_receive_byte(&value, PN532_UART_BYTE_TIMEOUT_MS) == 0U)
    {
      return PN532_RESULT_TIMEOUT;
    }
    return PN532_RESULT_NACK;
  }
  if((uint8_t)(frame_length + length_checksum) != 0U)
  {
    return PN532_RESULT_FRAME;
  }
  if(frame_length == 0U)
  {
    if(pn532_uart5_receive_byte(&value, PN532_UART_BYTE_TIMEOUT_MS) == 0U)
    {
      return PN532_RESULT_TIMEOUT;
    }
    *length = 0U;
    return PN532_RESULT_OK;
  }
  if(frame_length > PN532_FRAME_MAX_SIZE)
  {
    return PN532_RESULT_FRAME;
  }

  for(index = 0U; index < frame_length; index++)
  {
    if(pn532_uart5_receive_byte(&data[index], PN532_UART_BYTE_TIMEOUT_MS) == 0U)
    {
      return PN532_RESULT_TIMEOUT;
    }
  }
  if(pn532_uart5_receive_byte(&data_checksum,
                              PN532_UART_BYTE_TIMEOUT_MS) == 0U ||
     pn532_uart5_receive_byte(&value, PN532_UART_BYTE_TIMEOUT_MS) == 0U)
  {
    return PN532_RESULT_TIMEOUT;
  }

  value = 0U;
  for(index = 0U; index < frame_length; index++)
  {
    value = (uint8_t)(value + data[index]);
  }
  if((uint8_t)(value + data_checksum) != 0U)
  {
    return PN532_RESULT_FRAME;
  }

  *length = frame_length;
  return PN532_RESULT_OK;
}

static pn532_result_t pn532_uart5_command(uint8_t command,
                                           const uint8_t *parameters,
                                           uint8_t parameter_length,
                                           uint8_t *response,
                                           uint8_t *response_length)
{
  uint8_t frame[PN532_FRAME_MAX_SIZE + 8U];
  uint8_t response_frame[PN532_FRAME_MAX_SIZE];
  uint8_t frame_length;
  uint8_t response_frame_length;
  uint8_t data_sum;
  uint8_t index;
  pn532_result_t result;

  if(parameter_length > (PN532_FRAME_MAX_SIZE - 3U))
  {
    return PN532_RESULT_ARGUMENT;
  }

  frame_length = (uint8_t)(2U + parameter_length);
  frame[0] = PN532_PREAMBLE;
  frame[1] = PN532_START_CODE_1;
  frame[2] = PN532_START_CODE_2;
  frame[3] = frame_length;
  frame[4] = (uint8_t)(0U - frame_length);
  frame[5] = PN532_HOST_TO_PN532;
  frame[6] = command;
  data_sum = (uint8_t)(PN532_HOST_TO_PN532 + command);
  for(index = 0U; index < parameter_length; index++)
  {
    frame[7U + index] = parameters[index];
    data_sum = (uint8_t)(data_sum + parameters[index]);
  }
  frame[7U + parameter_length] = (uint8_t)(0U - data_sum);
  frame[8U + parameter_length] = 0U;

  pn532_uart5_flush_rx();
  pn532_uart5_send(frame, (uint16_t)(9U + parameter_length));

  result = pn532_uart5_read_frame(response_frame,
                                  &response_frame_length,
                                  PN532_RESPONSE_TIMEOUT_MS);
  if(result != PN532_RESULT_OK || response_frame_length != 0U)
  {
    return (result == PN532_RESULT_OK) ? PN532_RESULT_PROTOCOL : result;
  }

  result = pn532_uart5_read_frame(response_frame,
                                  &response_frame_length,
                                  PN532_RESPONSE_TIMEOUT_MS);
  if(result != PN532_RESULT_OK)
  {
    return result;
  }
  if(response_frame_length < 2U ||
     response_frame[0] != PN532_PN532_TO_HOST ||
     response_frame[1] != (uint8_t)(command + 1U))
  {
    return PN532_RESULT_PROTOCOL;
  }
  *response_length = (uint8_t)(response_frame_length - 2U);
  memcpy(response, &response_frame[2], *response_length);
  return PN532_RESULT_OK;
}

static void pn532_uart5_wakeup(void)
{
  static const uint8_t wakeup_sequence[] = {0x55U, 0x55U, 0x00U, 0x00U, 0x00U};

  pn532_uart5_flush_rx();
  pn532_uart5_send(wakeup_sequence, sizeof(wakeup_sequence));
  delay_ms(50U);
  pn532_uart5_flush_rx();
}

static void pn532_uart5_gpio_init(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_USART5_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_5 | GPIO_PINS_6;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE5, GPIO_MUX_8);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE6, GPIO_MUX_8);

  usart_init(USART5,
             PN532_UART5_BAUDRATE,
             USART_DATA_8BITS,
             USART_STOP_1_BIT);
  usart_transmitter_enable(USART5, TRUE);
  usart_receiver_enable(USART5, TRUE);
  usart_enable(USART5, TRUE);
}

pn532_result_t pn532_uart5_init(uint8_t *ic,
                                uint8_t *version,
                                uint8_t *revision,
                                uint8_t *support)
{
  uint8_t response[16];
  uint8_t response_length;
  pn532_result_t result;

  pn532_uart5_gpio_init();
  pn532_uart5_wakeup();
  result = pn532_uart5_command(PN532_COMMAND_GET_FIRMWARE,
                                0,
                                0U,
                                response,
                                &response_length);
  if((result != PN532_RESULT_OK) || (response_length < 4U))
  {
    return (result == PN532_RESULT_OK) ? PN532_RESULT_PROTOCOL : result;
  }
  *ic = response[0];
  *version = response[1];
  *revision = response[2];
  *support = response[3];

  return pn532_uart5_reinitialize();
}

pn532_result_t pn532_uart5_reinitialize(void)
{
  static const uint8_t sam_parameters[] = {0x01U, 0x14U, 0x01U};
  uint8_t response[4];
  uint8_t response_length;

  pn532_uart5_wakeup();
  return pn532_uart5_command(PN532_COMMAND_SAM_CONFIGURATION,
                             sam_parameters,
                             sizeof(sam_parameters),
                             response,
                             &response_length);
}

pn532_result_t pn532_list_passive_target(pn532_target_t *target)
{
  static const uint8_t parameters[] = {0x01U, 0x00U};
  uint8_t response[32];
  uint8_t response_length;
  pn532_result_t result;
  uint8_t uid_length;

  if(target == 0)
  {
    return PN532_RESULT_ARGUMENT;
  }
  result = pn532_uart5_command(PN532_COMMAND_IN_LIST_PASSIVE_TARGET,
                               parameters,
                               sizeof(parameters),
                               response,
                               &response_length);
  if(result != PN532_RESULT_OK)
  {
    return result;
  }
  if((response_length == 0U) || (response[0] == 0U))
  {
    return PN532_RESULT_NO_TARGET;
  }
  if(response_length < 6U)
  {
    return PN532_RESULT_PROTOCOL;
  }
  uid_length = response[5];
  if((uid_length == 0U) || (uid_length > PN532_UID_MAX_SIZE) ||
     (response_length < (uint8_t)(6U + uid_length)))
  {
    return PN532_RESULT_PROTOCOL;
  }

  memset(target, 0, sizeof(*target));
  target->target_number = response[1];
  target->atqa = ((uint16_t)response[2] << 8) | response[3];
  target->sak = response[4];
  target->uid_length = uid_length;
  memcpy(target->uid, &response[6], uid_length);
  return PN532_RESULT_OK;
}

pn532_result_t pn532_mifare_authenticate(const pn532_target_t *target,
                                         uint8_t block,
                                         const uint8_t key_a[6])
{
  uint8_t parameters[13];
  uint8_t response[4];
  uint8_t response_length;

  if((target == 0) || (key_a == 0) || (target->uid_length < 4U))
  {
    return PN532_RESULT_ARGUMENT;
  }
  parameters[0] = target->target_number;
  parameters[1] = PN532_MIFARE_AUTHENTICATE_A;
  parameters[2] = block;
  memcpy(&parameters[3], key_a, 6U);
  memcpy(&parameters[9], &target->uid[target->uid_length - 4U], 4U);
  if(pn532_uart5_command(PN532_COMMAND_IN_DATA_EXCHANGE,
                         parameters,
                         sizeof(parameters),
                         response,
                         &response_length) != PN532_RESULT_OK)
  {
    return PN532_RESULT_CARD;
  }
  if((response_length == 0U) || (response[0] != 0U))
  {
    return PN532_RESULT_CARD;
  }
  return PN532_RESULT_OK;
}

pn532_result_t pn532_mifare_read_block(const pn532_target_t *target,
                                       uint8_t block,
                                       const uint8_t key_a[6],
                                       uint8_t data[16])
{
  uint8_t parameters[3];
  uint8_t response[24];
  uint8_t response_length;
  pn532_result_t result;

  if((target == 0) || (key_a == 0) || (data == 0))
  {
    return PN532_RESULT_ARGUMENT;
  }
  result = pn532_mifare_authenticate(target, block, key_a);
  if(result != PN532_RESULT_OK)
  {
    return result;
  }

  parameters[0] = target->target_number;
  parameters[1] = PN532_MIFARE_READ;
  parameters[2] = block;
  result = pn532_uart5_command(PN532_COMMAND_IN_DATA_EXCHANGE,
                               parameters,
                               sizeof(parameters),
                               response,
                               &response_length);
  if((result != PN532_RESULT_OK) || (response_length < 17U) ||
     (response[0] != 0U))
  {
    return (result == PN532_RESULT_OK) ? PN532_RESULT_CARD : result;
  }
  memcpy(data, &response[1], 16U);
  return PN532_RESULT_OK;
}
