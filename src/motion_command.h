#ifndef MOTION_COMMAND_H
#define MOTION_COMMAND_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTION_PROTOCOL_VERSION                 1U

typedef enum
{
  MOTION_SOURCE_SERIAL = 1,
  MOTION_SOURCE_CANFD = 2,
  MOTION_SOURCE_LOCAL_SAFETY = 3
} motion_command_source_t;

typedef enum
{
  MOTION_CMD_GOTO_LOCAL = 1,
  MOTION_CMD_STOP,
  MOTION_CMD_ESTOP,
  MOTION_CMD_RESET_ODOM,
  MOTION_CMD_CLEAR_FAULT,
  MOTION_CMD_SET_VELOCITY,
  MOTION_CMD_COMM_TIMEOUT,
  MOTION_CMD_CAN_BUSOFF
} motion_command_type_t;

typedef enum
{
  MOTION_COMMAND_ERROR_NONE = 0,
  MOTION_COMMAND_ERROR_VERSION = 1,
  MOTION_COMMAND_ERROR_SEQUENCE = 2,
  MOTION_COMMAND_ERROR_RANGE = 3,
  MOTION_COMMAND_ERROR_FORMAT = 4,
  MOTION_COMMAND_ERROR_UNSUPPORTED = 5,
  MOTION_COMMAND_ERROR_BUSY = 6,
  MOTION_COMMAND_ERROR_CRC = 7,
  MOTION_COMMAND_ERROR_LENGTH = 8,
  MOTION_COMMAND_ERROR_SESSION = 9,
  MOTION_COMMAND_ERROR_FAULT_LOCKED = 10,
  MOTION_COMMAND_ERROR_TIMEOUT = 11,
  MOTION_COMMAND_ERROR_CAN_BUSOFF = 12
} motion_command_error_t;

typedef enum
{
  MOTION_HANDLE_RECEIVED = 0,
  MOTION_HANDLE_ACCEPTED,
  MOTION_HANDLE_BUSY,
  MOTION_HANDLE_REJECTED
} motion_command_handle_result_t;

typedef struct
{
  int32_t x_mm;
  int32_t y_mm;
  int32_t yaw_cdeg;
  uint16_t speed_mm_s;
} motion_goto_local_data_t;

typedef struct
{
  int16_t linear_mm_s;
  int16_t angular_mrad_s;
  uint16_t accel_mm_s2;
  uint16_t decel_mm_s2;
  uint16_t valid_time_ms;
} motion_velocity_data_t;

typedef struct
{
  uint8_t version;
  uint16_t sequence;
  motion_command_source_t source;
  motion_command_type_t type;
  union
  {
    motion_goto_local_data_t goto_local;
    motion_velocity_data_t velocity;
  } data;
} motion_command_t;

typedef motion_command_handle_result_t (*motion_command_handler_t)(
    const motion_command_t *command,
    void *context);

/* 检查统一运动命令的版本、序号和参数范围。 */
motion_command_error_t motion_command_validate(const motion_command_t *command);

/* 返回统一命令的固定英文名称，供串口ACK和日志使用。 */
const char *motion_command_get_name(motion_command_type_t type);

#ifdef __cplusplus
}
#endif

#endif
