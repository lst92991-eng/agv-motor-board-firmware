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
  agv_motion_reference_t latest;
  volatile uint32_t generation;
  uint32_t session_id;
  uint16_t last_sequence;
  uint8_t sequence_valid;
  uint8_t latest_valid;
} motion_reference_receiver_t;

/* State must be zero-initialized before its first reset. */
void motion_reference_receiver_reset(motion_reference_receiver_t *receiver,
                                     uint32_t session_id);

/* Clear the visible reference without allowing its sequence to replay. */
void motion_reference_receiver_invalidate_latest(
    motion_reference_receiver_t *receiver);

/* Decode and retain only a newer reference owned by the active session. */
motion_reference_receiver_result_t motion_reference_receiver_accept(
    motion_reference_receiver_t *receiver,
    const uint8_t *payload,
    uint16_t payload_size,
    uint32_t receive_ms,
    uint16_t lease_remaining_ms);

/* Copy the latest reference only while its effective TTL remains fresh. */
uint8_t motion_reference_receiver_get_latest(
    const motion_reference_receiver_t *receiver,
    uint32_t now_ms,
    agv_motion_reference_t *reference);

#ifdef __cplusplus
}
#endif

#endif
