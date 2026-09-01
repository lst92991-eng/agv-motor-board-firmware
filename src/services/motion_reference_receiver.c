/**
 * @file motion_reference_receiver.c
 * @brief 0x401会话/序号/TTL校验与ISR到任务的latest-only快照。
 *
 * CAN RX ISR只覆盖最新payload，协议任务负责解码；控制任务通过generation奇偶
 * 序列读取一致快照。该机制不是多写者锁，两个结构都必须保持单写者。
 */

#include "motion_reference_receiver.h"

#include <string.h>

#if defined(AT32F456RCT7)
#include "at32f45x.h"
#define MOTION_REFERENCE_MEMORY_BARRIER() __DMB()
#elif defined(_MSC_VER)
#include <intrin.h>
#define MOTION_REFERENCE_MEMORY_BARRIER() _ReadWriteBarrier()
#elif defined(__GNUC__) || defined(__clang__)
#define MOTION_REFERENCE_MEMORY_BARRIER() \
  __asm__ volatile("" ::: "memory")
#else
#define MOTION_REFERENCE_MEMORY_BARRIER() ((void)0)
#endif

/** @brief 将generation推进为奇数并发布写入开始屏障。 */
static void motion_reference_receiver_write_begin(
    motion_reference_receiver_t *receiver)
{
  receiver->generation++;
  MOTION_REFERENCE_MEMORY_BARRIER();
}

/** @brief 发布写入完成屏障并将generation恢复为偶数。 */
static void motion_reference_receiver_write_end(
    motion_reference_receiver_t *receiver)
{
  MOTION_REFERENCE_MEMORY_BARRIER();
  receiver->generation++;
}

/** @brief 清空接收状态并建立新的活动会话ID。 */
void motion_reference_receiver_reset(motion_reference_receiver_t *receiver,
                                     uint32_t session_id)
{
  if(receiver == NULL)
  {
    return;
  }
  motion_reference_receiver_write_begin(receiver);
  memset(&receiver->latest, 0, sizeof(receiver->latest));
  receiver->session_id = session_id;
  receiver->last_sequence = 0U;
  receiver->sequence_valid = 0U;
  receiver->latest_valid = 0U;
  motion_reference_receiver_write_end(receiver);
}

/** @brief 使最新Reference不可见但保留最后序号防止旧帧重放。 */
void motion_reference_receiver_invalidate_latest(
    motion_reference_receiver_t *receiver)
{
  if(receiver == NULL)
  {
    return;
  }
  motion_reference_receiver_write_begin(receiver);
  memset(&receiver->latest, 0, sizeof(receiver->latest));
  receiver->latest_valid = 0U;
  motion_reference_receiver_write_end(receiver);
}

/** @brief 解码并接受活动会话中的更新Reference，同时收紧有效TTL。 */
motion_reference_receiver_result_t motion_reference_receiver_accept(
    motion_reference_receiver_t *receiver,
    const uint8_t *payload,
    uint16_t payload_size,
    uint32_t receive_ms,
    uint16_t lease_remaining_ms)
{
  agv_motion_reference_t decoded;
  uint32_t decoded_session;
  uint16_t decoded_sequence;
  uint16_t sequence_delta;

  if((receiver == NULL) || (receiver->session_id == 0U) ||
     (lease_remaining_ms == 0U) ||
     (agv_motion_reference_decode_payload(
          payload, payload_size, receive_ms, &decoded_session,
          &decoded_sequence, &decoded) == 0U) ||
     (decoded_session != receiver->session_id))
  {
    return MOTION_REFERENCE_RECEIVER_REJECTED;
  }

  if(receiver->sequence_valid != 0U)
  {
    /* 16位模差小于半量程表示更新，兼容0xFFFF到0的正常回绕。 */
    sequence_delta = (uint16_t)(decoded_sequence -
                                receiver->last_sequence);
    if(sequence_delta == 0U)
    {
      return MOTION_REFERENCE_RECEIVER_DUPLICATE;
    }
    if(sequence_delta >= 0x8000U)
    {
      return MOTION_REFERENCE_RECEIVER_OLD;
    }
  }

  if(decoded.valid_for_ms > lease_remaining_ms)
  {
    /* Reference绝不能比控制租约活得更久。 */
    decoded.valid_for_ms = lease_remaining_ms;
  }
  motion_reference_receiver_write_begin(receiver);
  receiver->latest = decoded;
  receiver->last_sequence = decoded_sequence;
  receiver->sequence_valid = 1U;
  receiver->latest_valid = 1U;
  motion_reference_receiver_write_end(receiver);
  return MOTION_REFERENCE_RECEIVER_ACCEPTED;
}

/** @brief 有界重试复制一致且仍新鲜的最新Reference。 */
uint8_t motion_reference_receiver_get_latest(
    const motion_reference_receiver_t *receiver,
    uint32_t now_ms,
    agv_motion_reference_t *reference)
{
  agv_motion_reference_t snapshot;
  uint32_t generation_before;
  uint32_t generation_after;
  uint8_t latest_valid;
  uint8_t attempt;

  if((receiver == NULL) || (reference == NULL))
  {
    return 0U;
  }
  for(attempt = 0U; attempt < 3U; attempt++)
  {
    /* 有界重试保证控制任务不会因ISR持续更新而无界等待。 */
    generation_before = receiver->generation;
    if((generation_before & 1UL) != 0UL)
    {
      continue;
    }
    MOTION_REFERENCE_MEMORY_BARRIER();
    snapshot = receiver->latest;
    latest_valid = receiver->latest_valid;
    MOTION_REFERENCE_MEMORY_BARRIER();
    generation_after = receiver->generation;
    if((generation_before == generation_after) &&
       ((generation_after & 1UL) == 0UL))
    {
      if((latest_valid == 0U) ||
         (agv_motion_reference_is_fresh(&snapshot, now_ms) == 0U))
      {
        return 0U;
      }
      *reference = snapshot;
      return 1U;
    }
  }
  return 0U;
}

/** @brief 清空ISR覆盖邮箱及其已消费到达序号。 */
void motion_reference_ingress_mailbox_reset(
    motion_reference_ingress_mailbox_t *mailbox)
{
  if(mailbox != NULL)
  {
    memset(mailbox, 0, sizeof(*mailbox));
  }
}

/** @brief ISR单写者覆盖发布最新0x401 payload和到达元数据。 */
void motion_reference_ingress_mailbox_publish(
    motion_reference_ingress_mailbox_t *mailbox,
    const uint8_t *payload,
    uint32_t receive_ms,
    uint32_t arrival_order)
{
  if((mailbox == NULL) || (payload == NULL) || (arrival_order == 0U))
  {
    return;
  }
  /* ISR单写者：先标记写入中，再发布payload与到达顺序。 */
  mailbox->generation++;
  MOTION_REFERENCE_MEMORY_BARRIER();
  memcpy(mailbox->latest.payload, payload,
         AGV_MOTION_REFERENCE_PAYLOAD_SIZE);
  mailbox->latest.receive_ms = receive_ms;
  mailbox->latest.arrival_order = arrival_order;
  mailbox->valid = 1U;
  MOTION_REFERENCE_MEMORY_BARRIER();
  mailbox->generation++;
}

/** @brief 任务侧有界复制尚未消费的最新ISR输入。 */
uint8_t motion_reference_ingress_mailbox_take(
    motion_reference_ingress_mailbox_t *mailbox,
    motion_reference_ingress_t *ingress)
{
  motion_reference_ingress_t snapshot;
  uint32_t generation_before;
  uint32_t generation_after;
  uint8_t valid;
  uint8_t attempt;

  if((mailbox == NULL) || (ingress == NULL))
  {
    return 0U;
  }
  for(attempt = 0U; attempt < 3U; attempt++)
  {
    generation_before = mailbox->generation;
    if((generation_before & 1UL) != 0UL)
    {
      continue;
    }
    MOTION_REFERENCE_MEMORY_BARRIER();
    snapshot = mailbox->latest;
    valid = mailbox->valid;
    MOTION_REFERENCE_MEMORY_BARRIER();
    generation_after = mailbox->generation;
    if((generation_before == generation_after) &&
       ((generation_after & 1UL) == 0UL))
    {
      if((valid == 0U) ||
         (snapshot.arrival_order == mailbox->last_taken_order))
      {
        return 0U;
      }
      mailbox->last_taken_order = snapshot.arrival_order;
      *ingress = snapshot;
      return 1U;
    }
  }
  return 0U;
}

/** @brief 用32位半量程模差判断到达序号是否越过屏障。 */
uint8_t motion_reference_is_after(uint32_t value, uint32_t barrier)
{
  /* 有符号模差要求两个序号真实距离小于2^31。 */
  return (((int32_t)(value - barrier)) > 0) ? 1U : 0U;
}
