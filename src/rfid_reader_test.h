#ifndef RFID_READER_TEST_H
#define RFID_READER_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化USART5和PN532，并输出固件版本。 */
void rfid_reader_test_init(void);

/* 轮询标签并输出UID及有效AGV记录。 */
void rfid_reader_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif
