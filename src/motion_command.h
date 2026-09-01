#ifndef MOTION_COMMAND_H
#define MOTION_COMMAND_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTION_PROTOCOL_VERSION                 1U

typedef enum
{
  MOTION_SOURCE_CANFD = 2,
  MOTION_SOURCE_LOCAL_SAFETY = 3
} motion_command_source_t;

typedef enum
{
  MOTION_CMD_STOP = 3,
  MOTION_CMD_ESTOP = 4,
  MOTION_CMD_RESET_ODOM = 5,
  MOTION_CMD_CLEAR_FAULT = 6,
  MOTION_CMD_COMM_TIMEOUT = 8,
  MOTION_CMD_CAN_BUSOFF = 9,
  MOTION_CMD_ENABLE = 10,
  MOTION_CMD_DISABLE = 11,
  MOTION_CMD_RELEASE = 12,
  MOTION_CMD_POWER_FAULT = 13
} motion_command_type_t;

typedef enum
{
  MOTION_COMMAND_ERROR_NONE = 0,
  MOTION_COMMAND_ERROR_VERSION = 1,
  MOTION_COMMAND_ERROR_SEQUENCE = 2,
  MOTION_COMMAND_ERROR_FORMAT = 4,
  MOTION_COMMAND_ERROR_UNSUPPORTED = 5,
  MOTION_COMMAND_ERROR_BUSY = 6,
  MOTION_COMMAND_ERROR_FAULT_LOCKED = 10
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
  uint8_t version;
  uint32_t sequence;
  uint32_t session_id;
  motion_command_source_t source;
  motion_command_type_t type;
} motion_command_t;

typedef motion_command_handle_result_t (*motion_command_handler_t)(
    const motion_command_t *command,
    void *context);

#ifdef __cplusplus
}
#endif

#endif
