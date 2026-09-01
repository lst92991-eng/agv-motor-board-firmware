/**
 * @file motion_reference_receiver.h
 * @brief MotionReference latest-only接收器和ISR覆盖邮箱。
 */

#ifndef MOTION_REFERENCE_RECEIVER_H
#define MOTION_REFERENCE_RECEIVER_H

#include <stdint.h>

#include "agv/motion_contract.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  MOTION_REFERENCE_RECEIVER_REJECTED = 0,
  MOTION_REFERENCE_RECEIVER_ACCEPTED,
  MOTION_REFERENCE_RECEIVER_DUPLICATE,
  MOTION_REFERENCE_RECEIVER_OLD
} motion_reference_receiver_result_t;

typedef struct
{
  /* generation为奇数时写者尚未完成，读者不得消费latest。 */
  agv_motion_reference_t latest;
  volatile uint32_t generation;
  uint32_t session_id;
  uint16_t last_sequence;
  uint8_t sequence_valid;
  uint8_t latest_valid;
} motion_reference_receiver_t;

typedef struct
{
  uint8_t payload[AGV_MOTION_REFERENCE_PAYLOAD_SIZE];
  uint32_t receive_ms;
  uint32_t arrival_order;
} motion_reference_ingress_t;

typedef struct
{
  motion_reference_ingress_t latest;
  volatile uint32_t generation;
  uint32_t last_taken_order;
  uint8_t valid;
} motion_reference_ingress_mailbox_t;

/* 首次reset前结构必须清零；reset同时建立新的唯一控制会话。 */
void motion_reference_receiver_reset(motion_reference_receiver_t *receiver,
                                     uint32_t session_id);

/* 清除可见Reference但保留序号历史，STOP后旧帧不能重新生效。 */
void motion_reference_receiver_invalidate_latest(
    motion_reference_receiver_t *receiver);

/* 只接受活动会话内更新的Reference，并将TTL收紧到剩余租约。 */
motion_reference_receiver_result_t motion_reference_receiver_accept(
    motion_reference_receiver_t *receiver,
    const uint8_t *payload,
    uint16_t payload_size,
    uint32_t receive_ms,
    uint16_t lease_remaining_ms);

/* 仅在generation一致且有效TTL未过期时复制最新Reference。 */
uint8_t motion_reference_receiver_get_latest(
    const motion_reference_receiver_t *receiver,
    uint32_t now_ms,
    agv_motion_reference_t *reference);

/* CAN RX ISR单写者覆盖邮箱；拥塞时保留最新值而不是排队旧运动量。 */
void motion_reference_ingress_mailbox_reset(
    motion_reference_ingress_mailbox_t *mailbox);
/** ISR单写者覆盖发布最新payload、接收时刻和到达顺序。 */
void motion_reference_ingress_mailbox_publish(
    motion_reference_ingress_mailbox_t *mailbox,
    const uint8_t *payload,
    uint32_t receive_ms,
    uint32_t arrival_order);
/** 任务侧有界复制尚未消费的最新输入。 */
uint8_t motion_reference_ingress_mailbox_take(
    motion_reference_ingress_mailbox_t *mailbox,
    motion_reference_ingress_t *ingress);

/* 两个到达序号的间隔必须小于2^31。 */
uint8_t motion_reference_is_after(uint32_t value, uint32_t barrier);

#ifdef __cplusplus
}
#endif

#endif
