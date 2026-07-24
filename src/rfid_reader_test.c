/**
  *****************************************************************************
  * @file     rfid_reader_test.c
  * @brief    PN532车载读卡独立测试
  *
  * 当前测试只读MIFARE Classic的块4和块5，不输出任何电机控制命令。
  *****************************************************************************
  */

#include "rfid_reader_test.h"

#include <stdio.h>
#include <string.h>

#include "at32f45x_board.h"
#include "pn532_uart5.h"
#include "rfid_record.h"

#define RFID_PRIMARY_BLOCK                 4U
#define RFID_BACKUP_BLOCK                  5U
#define RFID_RETRY_DELAY_MS             500U
#define RFID_POLL_DELAY_MS               100U
#define RFID_ABSENT_CONFIRM_COUNT          3U

static const uint8_t rfid_default_key_a[6] =
  {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
static pn532_target_t rfid_last_target;
static uint8_t rfid_target_reported;
static uint8_t rfid_absent_count;
static pn532_result_t rfid_init_result;

static uint8_t rfid_target_same(const pn532_target_t *first,
                                const pn532_target_t *second)
{
  return (first->uid_length == second->uid_length) &&
         (memcmp(first->uid, second->uid, first->uid_length) == 0) ? 1U : 0U;
}

static void rfid_print_target(const pn532_target_t *target)
{
  uint8_t index;

  printf("RFID_TAG,atqa=0x%04X,sak=0x%02X,uid=",
         target->atqa,
         target->sak);
  for(index = 0U; index < target->uid_length; index++)
  {
    printf("%02X%s", target->uid[index],
           (index + 1U == target->uid_length) ? "" : ":");
  }
  printf("\r\n");
}

static void rfid_print_record(const char *copy_name,
                              const rfid_record_t *record)
{
  printf("RFID_RECORD,copy=%s,type=%u,station=%u,map=%u,flags=0x%04X,seq=%u\r\n",
         copy_name,
         record->tag_type,
         record->station_id,
         record->map_version,
         record->flags,
         record->sequence);
}

static void rfid_read_record(const pn532_target_t *target)
{
  uint8_t primary_data[16];
  uint8_t backup_data[16];
  rfid_record_t primary_record;
  rfid_record_t backup_record;
  uint8_t primary_valid;
  uint8_t backup_valid;
  pn532_result_t primary_result;
  pn532_result_t backup_result;

  if((target->sak != 0x08U) && (target->sak != 0x09U) &&
     (target->sak != 0x18U))
  {
    printf("RFID_UNSUPPORTED,only_mifare_classic,SAK=0x%02X\r\n",
           target->sak);
    return;
  }

  primary_result = pn532_mifare_read_block(target,
                                           RFID_PRIMARY_BLOCK,
                                           rfid_default_key_a,
                                           primary_data);
  backup_result = pn532_mifare_read_block(target,
                                          RFID_BACKUP_BLOCK,
                                          rfid_default_key_a,
                                          backup_data);
  primary_valid = (primary_result == PN532_RESULT_OK) &&
                  (rfid_record_decode(primary_data, &primary_record) != 0U);
  backup_valid = (backup_result == PN532_RESULT_OK) &&
                 (rfid_record_decode(backup_data, &backup_record) != 0U);

  if(primary_valid != 0U)
  {
    rfid_print_record("PRIMARY", &primary_record);
  }
  if(backup_valid != 0U)
  {
    rfid_print_record("BACKUP", &backup_record);
  }
  if((primary_valid == 0U) && (backup_valid == 0U))
  {
    printf("RFID_RECORD_INVALID,primary_result=%d,backup_result=%d\r\n",
           primary_result,
           backup_result);
  }
  else if((primary_valid != 0U) && (backup_valid != 0U))
  {
    if(primary_record.sequence >= backup_record.sequence)
    {
      printf("RFID_POSITION,station=%u,type=%u,source=PRIMARY\r\n",
             primary_record.station_id,
             primary_record.tag_type);
    }
    else
    {
      printf("RFID_POSITION,station=%u,type=%u,source=BACKUP\r\n",
             backup_record.station_id,
             backup_record.tag_type);
    }
  }
  else if(primary_valid != 0U)
  {
    printf("RFID_POSITION,station=%u,type=%u,source=PRIMARY_ONLY\r\n",
           primary_record.station_id,
           primary_record.tag_type);
  }
  else
  {
    printf("RFID_POSITION,station=%u,type=%u,source=BACKUP_ONLY\r\n",
           backup_record.station_id,
           backup_record.tag_type);
  }
}

void rfid_reader_test_init(void)
{
  uint8_t ic = 0U;
  uint8_t version = 0U;
  uint8_t revision = 0U;
  uint8_t support = 0U;

  memset(&rfid_last_target, 0, sizeof(rfid_last_target));
  rfid_target_reported = 0U;
  rfid_absent_count = 0U;
  rfid_init_result = pn532_uart5_init(&ic, &version, &revision, &support);
  printf("\r\nRFID_READER_TEST,USART5,PB5_RX,PB6_TX\r\n");
  if(rfid_init_result == PN532_RESULT_OK)
  {
    printf("RFID_READY,ic=0x%02X,version=%u.%u,support=0x%02X\r\n",
           ic,
           version,
           revision,
           support);
  }
  else
  {
    printf("RFID_INIT_ERROR,result=%d\r\n", rfid_init_result);
  }
}

void rfid_reader_test_loop(void)
{
  pn532_target_t target;
  pn532_result_t result;

  if(rfid_init_result != PN532_RESULT_OK)
  {
    delay_ms(RFID_RETRY_DELAY_MS);
    rfid_init_result = pn532_uart5_reinitialize();
    printf("RFID_INIT_RETRY,result=%d\r\n", rfid_init_result);
    return;
  }

  result = pn532_list_passive_target(&target);
  if(result == PN532_RESULT_NO_TARGET)
  {
    if(rfid_absent_count < RFID_ABSENT_CONFIRM_COUNT)
    {
      rfid_absent_count++;
    }
    if(rfid_absent_count >= RFID_ABSENT_CONFIRM_COUNT)
    {
      rfid_target_reported = 0U;
    }
  }
  else if(result == PN532_RESULT_OK)
  {
    rfid_absent_count = 0U;
    if((rfid_target_reported == 0U) ||
       (rfid_target_same(&target, &rfid_last_target) == 0U))
    {
      rfid_print_target(&target);
      rfid_read_record(&target);
      rfid_last_target = target;
      rfid_target_reported = 1U;
    }
  }
  else
  {
    printf("RFID_POLL_ERROR,result=%d\r\n", result);
    rfid_init_result = pn532_uart5_reinitialize();
  }

  delay_ms(RFID_POLL_DELAY_MS);
}
