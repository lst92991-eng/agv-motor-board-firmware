/**
 * @file canfd_status_codec.h
 * @brief 三类底盘状态PDO的固定长度小端编码接口。
 *
 * codec不访问CAN硬件，便于主机黄金字节测试锁定线格式。
 */

#ifndef CANFD_STATUS_CODEC_H
#define CANFD_STATUS_CODEC_H

#include <stddef.h>
#include <stdint.h>

#include "canfd_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CANFD_DRIVE_STATUS_PAYLOAD_SIZE       32U
#define CANFD_ODOM_STATUS_PAYLOAD_SIZE        32U
#define CANFD_SAFETY_STATUS_PAYLOAD_SIZE      16U

/** 编码32字节底盘状态；容量不足或空指针返回0。 */
uint8_t canfd_status_encode_drive(
    const canfd_drive_status_t *status,
    uint8_t *payload,
    size_t capacity);

/** 编码32字节里程计，sequence由通信层统一递增。 */
uint8_t canfd_status_encode_odom(
    const canfd_odom_status_t *status,
    uint16_t sequence,
    uint8_t *payload,
    size_t capacity);

/** 编码16字节安全状态；保留字节始终清零。 */
uint8_t canfd_status_encode_safety(
    const canfd_safety_status_t *status,
    uint8_t *payload,
    size_t capacity);

#ifdef __cplusplus
}
#endif

#endif
