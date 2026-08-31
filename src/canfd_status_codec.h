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
#define CANFD_MOTOR_STATUS_PAYLOAD_SIZE       32U
#define CANFD_SAFETY_STATUS_PAYLOAD_SIZE      16U
#define CANFD_DOCK_STATUS_PAYLOAD_SIZE        32U
#define CANFD_DOCK_SCAN_PAYLOAD_SIZE          16U

uint8_t canfd_status_encode_drive(
    const canfd_drive_status_t *status,
    uint8_t *payload,
    size_t capacity);

uint8_t canfd_status_encode_odom(
    const canfd_odom_status_t *status,
    uint16_t sequence,
    uint8_t *payload,
    size_t capacity);

uint8_t canfd_status_encode_motor(
    const canfd_motor_status_t *status,
    uint8_t *payload,
    size_t capacity);

uint8_t canfd_status_encode_safety(
    const canfd_safety_status_t *status,
    uint8_t *payload,
    size_t capacity);

uint8_t canfd_status_encode_dock(
    const canfd_dock_status_t *status,
    uint8_t *payload,
    size_t capacity);

uint8_t canfd_status_encode_dock_scan(
    const canfd_dock_scan_t *status,
    uint8_t *payload,
    size_t capacity);

#ifdef __cplusplus
}
#endif

#endif
