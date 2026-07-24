/**
  *****************************************************************************
  * @file     rfid_record.c
  * @brief    AGV RFID记录格式与CRC校验
  *****************************************************************************
  */

#include "rfid_record.h"

#include <string.h>

#define RFID_RECORD_VERSION 1U

uint16_t rfid_record_crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint16_t index;
  uint8_t bit;

  for(index = 0U; index < length; index++)
  {
    crc ^= (uint16_t)data[index] << 8;
    for(bit = 0U; bit < 8U; bit++)
    {
      if((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc = (uint16_t)(crc << 1);
      }
    }
  }

  return crc;
}

uint8_t rfid_record_decode(const uint8_t *data, rfid_record_t *record)
{
  uint16_t stored_crc;
  uint16_t calculated_crc;

  if((data == 0) || (record == 0))
  {
    return 0U;
  }
  if((data[0] != 'A') || (data[1] != 'G') ||
     (data[2] != 'V') || (data[3] != '1') ||
     (data[4] != RFID_RECORD_VERSION))
  {
    return 0U;
  }

  stored_crc = (uint16_t)data[14] | ((uint16_t)data[15] << 8);
  calculated_crc = rfid_record_crc16(data, 14U);
  if(stored_crc != calculated_crc)
  {
    return 0U;
  }

  memset(record, 0, sizeof(*record));
  record->tag_type = data[5];
  record->station_id = (uint16_t)data[6] | ((uint16_t)data[7] << 8);
  record->map_version = (uint16_t)data[8] | ((uint16_t)data[9] << 8);
  record->flags = (uint16_t)data[10] | ((uint16_t)data[11] << 8);
  record->sequence = (uint16_t)data[12] | ((uint16_t)data[13] << 8);
  return 1U;
}
