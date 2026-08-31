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

static void motion_reference_receiver_write_begin(
    motion_reference_receiver_t *receiver)
{
  receiver->generation++;
  MOTION_REFERENCE_MEMORY_BARRIER();
}

static void motion_reference_receiver_write_end(
    motion_reference_receiver_t *receiver)
{
  MOTION_REFERENCE_MEMORY_BARRIER();
  receiver->generation++;
}

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

void motion_reference_ingress_mailbox_reset(
    motion_reference_ingress_mailbox_t *mailbox)
{
  if(mailbox != NULL)
  {
    memset(mailbox, 0, sizeof(*mailbox));
  }
}

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

uint8_t motion_reference_arrived_before(uint32_t first,
                                        uint32_t second)
{
  return (((int32_t)(first - second)) < 0) ? 1U : 0U;
}

uint8_t motion_reference_is_after_barrier(uint32_t ingress_order,
                                          uint32_t barrier_order,
                                          uint8_t barrier_valid)
{
  if(barrier_valid == 0U)
  {
    return 1U;
  }
  return motion_reference_arrived_before(barrier_order, ingress_order);
}
