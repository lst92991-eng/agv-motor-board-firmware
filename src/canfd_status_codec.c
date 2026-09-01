/**
  ****************************************************************************
  * @file     canfd_status_codec.c
  * @brief    电机板状态PDO的固定小端字节布局
  *
  * 本模块不访问CAN外设和队列，便于在主机上对协议字段、符号和保留字节回归。
  ****************************************************************************
  */

#include "canfd_status_codec.h"

#include <string.h>

static void canfd_status_put_u16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
}

static void canfd_status_put_i16(uint8_t *data, int16_t value)
{
  canfd_status_put_u16(data, (uint16_t)value);
}

static void canfd_status_put_u32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
  data[2] = (uint8_t)(value >> 16U);
  data[3] = (uint8_t)(value >> 24U);
}

static void canfd_status_put_i32(uint8_t *data, int32_t value)
{
  canfd_status_put_u32(data, (uint32_t)value);
}

uint8_t canfd_status_encode_drive(
    const canfd_drive_status_t *status,
    uint8_t *payload,
    size_t capacity)
{
  if((status == NULL) || (payload == NULL) ||
     (capacity < CANFD_DRIVE_STATUS_PAYLOAD_SIZE))
  {
    return 0U;
  }
  memset(payload, 0, CANFD_DRIVE_STATUS_PAYLOAD_SIZE);
  payload[0] = status->application_state;
  payload[1] = status->motion_state;
  payload[2] = status->motion_mode;
  payload[3] = status->flags;
  canfd_status_put_u32(&payload[4], status->command_id);
  canfd_status_put_u32(&payload[8], status->session_id);
  canfd_status_put_u32(&payload[12], status->fault_mask);
  canfd_status_put_u32(&payload[16], status->warning_mask);
  canfd_status_put_i32(&payload[20], status->x_mm);
  canfd_status_put_i32(&payload[24], status->y_mm);
  canfd_status_put_i32(&payload[28], status->yaw_cdeg);
  return 1U;
}

uint8_t canfd_status_encode_odom(
    const canfd_odom_status_t *status,
    uint16_t sequence,
    uint8_t *payload,
    size_t capacity)
{
  if((status == NULL) || (payload == NULL) ||
     (capacity < CANFD_ODOM_STATUS_PAYLOAD_SIZE))
  {
    return 0U;
  }
  memset(payload, 0, CANFD_ODOM_STATUS_PAYLOAD_SIZE);
  canfd_status_put_u32(&payload[0], status->uptime_ms);
  canfd_status_put_u16(&payload[4], sequence);
  canfd_status_put_u16(&payload[6], status->valid_flags);
  canfd_status_put_i32(&payload[8], status->x_mm);
  canfd_status_put_i32(&payload[12], status->y_mm);
  canfd_status_put_i32(&payload[16], status->yaw_cdeg);
  canfd_status_put_i16(&payload[20], status->linear_mm_s);
  canfd_status_put_i16(&payload[22], status->yaw_rate_cdeg_s);
  canfd_status_put_i16(&payload[24], status->accel_x_mg);
  canfd_status_put_i16(&payload[26], status->accel_y_mg);
  canfd_status_put_i16(&payload[28], status->accel_z_mg);
  return 1U;
}

uint8_t canfd_status_encode_safety(
    const canfd_safety_status_t *status,
    uint8_t *payload,
    size_t capacity)
{
  if((status == NULL) || (payload == NULL) ||
     (capacity < CANFD_SAFETY_STATUS_PAYLOAD_SIZE))
  {
    return 0U;
  }
  memset(payload, 0, CANFD_SAFETY_STATUS_PAYLOAD_SIZE);
  canfd_status_put_u32(&payload[0], status->uptime_ms);
  payload[4] = status->bumper_mask;
  payload[5] = status->estop_source_mask;
  payload[6] = status->safety_state;
  canfd_status_put_u32(&payload[8], status->fault_mask);
  canfd_status_put_u32(&payload[12], status->warning_mask);
  return 1U;
}
