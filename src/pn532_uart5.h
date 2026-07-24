#ifndef PN532_UART5_H
#define PN532_UART5_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PN532_UID_MAX_SIZE 10U

typedef struct
{
  uint8_t target_number;
  uint16_t atqa;
  uint8_t sak;
  uint8_t uid_length;
  uint8_t uid[PN532_UID_MAX_SIZE];
} pn532_target_t;

typedef enum
{
  PN532_RESULT_OK = 0,
  PN532_RESULT_NO_TARGET = 1,
  PN532_RESULT_TIMEOUT = -1,
  PN532_RESULT_FRAME = -2,
  PN532_RESULT_NACK = -3,
  PN532_RESULT_PROTOCOL = -4,
  PN532_RESULT_CARD = -5,
  PN532_RESULT_ARGUMENT = -6
} pn532_result_t;

/* 初始化PB5/PB6上的USART5并唤醒PN532。 */
pn532_result_t pn532_uart5_init(uint8_t *ic,
                                uint8_t *version,
                                uint8_t *revision,
                                uint8_t *support);

/* 重新发送唤醒和SAM配置命令。 */
pn532_result_t pn532_uart5_reinitialize(void);

/* 搜索一个ISO14443-A标签。 */
pn532_result_t pn532_list_passive_target(pn532_target_t *target);

/* 使用Key A认证MIFARE Classic数据块。 */
pn532_result_t pn532_mifare_authenticate(const pn532_target_t *target,
                                         uint8_t block,
                                         const uint8_t key_a[6]);

/* 读取一个MIFARE Classic数据块。 */
pn532_result_t pn532_mifare_read_block(const pn532_target_t *target,
                                       uint8_t block,
                                       const uint8_t key_a[6],
                                       uint8_t data[16]);

#ifdef __cplusplus
}
#endif

#endif
