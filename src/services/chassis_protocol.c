/**
 * @file chassis_protocol.c
 * @brief 固定四报文教学协议：控制、Reference、状态和里程计。
 */
#include "chassis_protocol.h"

#include <string.h>

#include "board_config.h"
#include "canfd_driver.h"

#define PROTOCOL_MIN_TTL_MS 50U
#define PROTOCOL_MAX_TTL_MS 300U

static drive_reference_t protocol_reference;
static chassis_command_t protocol_command;
static uint16_t protocol_reference_sequence;
static uint16_t protocol_command_sequence;
static uint16_t protocol_ttl_ms;
static uint32_t protocol_reference_ms;
static uint8_t protocol_reference_valid;

static uint16_t protocol_u16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static int16_t protocol_i16(const uint8_t *data)
{
  return (int16_t)protocol_u16(data);
}

static uint32_t protocol_u32(const uint8_t *data)
{
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
      ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static void protocol_put_u16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
}

static void protocol_put_u32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
  data[2] = (uint8_t)(value >> 16U);
  data[3] = (uint8_t)(value >> 24U);
}

static uint8_t protocol_after(uint16_t value, uint16_t previous)
{
  uint16_t delta = (uint16_t)(value - previous);
  return ((delta != 0U) && (delta < 0x8000U)) ? 1U : 0U;
}

static void protocol_parse_control(const canfd_frame_t *frame)
{
  chassis_command_t command;
  if((frame->size != 8U) || (frame->data[1] != 0U) ||
     (protocol_u32(&frame->data[4]) != 0U)) return;
  command.type = (chassis_command_type_t)frame->data[0];
  command.sequence = protocol_u16(&frame->data[2]);
  if((command.type <= CHASSIS_CMD_NONE) ||
     (command.type > CHASSIS_CMD_RESET_ODOMETRY) ||
     ((protocol_command_sequence != 0U) &&
      (protocol_after(command.sequence, protocol_command_sequence) == 0U))) return;
  protocol_command_sequence = command.sequence;
  /* ESTOP覆盖其他待处理命令；其余命令只保留最新一条。 */
  if((protocol_command.type != CHASSIS_CMD_ESTOP) ||
     (command.type == CHASSIS_CMD_ESTOP)) protocol_command = command;
}

static void protocol_parse_reference(const canfd_frame_t *frame, uint32_t now_ms)
{
  uint16_t sequence;
  uint16_t ttl;
  drive_reference_t reference;

  if(frame->size != BOARD_CAN_REFERENCE_SIZE) return;
  sequence = protocol_u16(&frame->data[0]);
  ttl = protocol_u16(&frame->data[2]);
  reference.linear_mm_s = protocol_i16(&frame->data[4]);
  reference.angular_mrad_s = protocol_i16(&frame->data[6]);
  reference.accel_mm_s2 = protocol_u16(&frame->data[8]);
  reference.decel_mm_s2 = protocol_u16(&frame->data[10]);
  if((protocol_u32(&frame->data[12]) != 0U) ||
     (ttl < PROTOCOL_MIN_TTL_MS) || (ttl > PROTOCOL_MAX_TTL_MS) ||
     ((protocol_reference_valid != 0U) &&
      (protocol_after(sequence, protocol_reference_sequence) == 0U))) return;
  if((reference.linear_mm_s > (int16_t)BOARD_MAX_LINEAR_MM_S) ||
     (reference.linear_mm_s < -(int16_t)BOARD_MAX_LINEAR_MM_S) ||
     (reference.angular_mrad_s > (int16_t)BOARD_MAX_ANGULAR_MRAD_S) ||
     (reference.angular_mrad_s < -(int16_t)BOARD_MAX_ANGULAR_MRAD_S)) return;
  protocol_reference = reference;
  protocol_reference_sequence = sequence;
  protocol_ttl_ms = ttl;
  protocol_reference_ms = now_ms;
  protocol_reference_valid = 1U;
}

void chassis_protocol_init(void)
{
  memset(&protocol_reference, 0, sizeof(protocol_reference));
  memset(&protocol_command, 0, sizeof(protocol_command));
  protocol_reference_sequence = protocol_command_sequence = 0U;
  protocol_reference_valid = 0U;
  canfd_driver_init();
}

void chassis_protocol_poll(uint32_t now_ms)
{
  canfd_frame_t frame;
  canfd_driver_poll(now_ms);
  while(canfd_driver_take(&frame) != 0U)
  {
    if(frame.id == BOARD_CAN_ID_CONTROL) protocol_parse_control(&frame);
    else if(frame.id == BOARD_CAN_ID_REFERENCE) protocol_parse_reference(&frame, now_ms);
  }
}

uint8_t chassis_protocol_take_command(chassis_command_t *command)
{
  if((command == 0) || (protocol_command.type == CHASSIS_CMD_NONE)) return 0U;
  *command = protocol_command;
  protocol_command.type = CHASSIS_CMD_NONE;
  return 1U;
}

uint8_t chassis_protocol_get_reference(uint32_t now_ms, drive_reference_t *reference)
{
  if((reference == 0) || (protocol_reference_valid == 0U) ||
     ((now_ms - protocol_reference_ms) >= protocol_ttl_ms)) return 0U;
  *reference = protocol_reference;
  return 1U;
}

void chassis_protocol_clear_reference(void)
{
  protocol_reference_valid = 0U;
}

uint8_t chassis_protocol_bus_healthy(void)
{
  return canfd_driver_healthy();
}

void chassis_protocol_send_status(const chassis_status_t *status, uint32_t uptime_ms)
{
  uint8_t data[24] = {0};
  if(status == 0) return;
  data[0] = status->state;
  data[1] = status->fault;
  data[2] = status->bumper_mask;
  data[3] = status->flags;
  protocol_put_u16(&data[4], status->command_sequence);
  data[6] = status->command_result;
  protocol_put_u32(&data[8], uptime_ms);
  protocol_put_u16(&data[12], (uint16_t)status->left_pwm);
  protocol_put_u16(&data[14], (uint16_t)status->right_pwm);
  protocol_put_u16(&data[16], status->bus_mv);
  protocol_put_u16(&data[18], status->left_current_ma);
  protocol_put_u16(&data[20], status->right_current_ma);
  (void)canfd_driver_send(BOARD_CAN_ID_STATUS, data, sizeof(data));
}

void chassis_protocol_send_odometry(const odometry_t *odometry,
                                    int16_t linear_mm_s,
                                    int16_t yaw_rate_cdeg_s,
                                    uint32_t uptime_ms)
{
  uint8_t data[20] = {0};
  if(odometry == 0) return;
  protocol_put_u32(&data[0], (uint32_t)(int32_t)odometry->x_mm);
  protocol_put_u32(&data[4], (uint32_t)(int32_t)odometry->y_mm);
  protocol_put_u32(&data[8], (uint32_t)(int32_t)(odometry->yaw_deg * 100.0f));
  protocol_put_u16(&data[12], (uint16_t)linear_mm_s);
  protocol_put_u16(&data[14], (uint16_t)yaw_rate_cdeg_s);
  protocol_put_u32(&data[16], uptime_ms);
  (void)canfd_driver_send(BOARD_CAN_ID_ODOMETRY, data, sizeof(data));
}
