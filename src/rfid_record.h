#ifndef RFID_RECORD_H
#define RFID_RECORD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RFID_RECORD_SIZE 16U

typedef struct
{
  uint8_t tag_type;
  uint16_t station_id;
  uint16_t map_version;
  uint16_t flags;
  uint16_t sequence;
} rfid_record_t;

/* 校验并解析MIFARE数据块中的AGV记录。 */
uint8_t rfid_record_decode(const uint8_t *data, rfid_record_t *record);

/* 计算AGV记录使用的CRC16-CCITT-FALSE。 */
uint16_t rfid_record_crc16(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
