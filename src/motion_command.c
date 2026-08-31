/**
  *****************************************************************************
  * @file     motion_command.c
  * @brief    串口和CAN FD共用的运动命令定义与校验
  *****************************************************************************
  */

#include "motion_command.h"

#include <stddef.h>

motion_command_error_t motion_command_validate(const motion_command_t *command)
{
  if(command == NULL)
  {
    return MOTION_COMMAND_ERROR_FORMAT;
  }
  if(command->version != MOTION_PROTOCOL_VERSION)
  {
    return MOTION_COMMAND_ERROR_VERSION;
  }
  if(command->sequence == 0U)
  {
    return MOTION_COMMAND_ERROR_SEQUENCE;
  }

  switch(command->type)
  {
    case MOTION_CMD_STOP:
    case MOTION_CMD_ESTOP:
    case MOTION_CMD_RESET_ODOM:
    case MOTION_CMD_CLEAR_FAULT:
    case MOTION_CMD_COMM_TIMEOUT:
    case MOTION_CMD_CAN_BUSOFF:
    case MOTION_CMD_ENABLE:
    case MOTION_CMD_DISABLE:
    case MOTION_CMD_RELEASE:
    case MOTION_CMD_POWER_FAULT:
      return MOTION_COMMAND_ERROR_NONE;
    default:
      return MOTION_COMMAND_ERROR_UNSUPPORTED;
  }
}

const char *motion_command_get_name(motion_command_type_t type)
{
  switch(type)
  {
    case MOTION_CMD_STOP:
      return "STOP";
    case MOTION_CMD_ESTOP:
      return "ESTOP";
    case MOTION_CMD_RESET_ODOM:
      return "RESET_ODOM";
    case MOTION_CMD_CLEAR_FAULT:
      return "CLEAR_FAULT";
    case MOTION_CMD_COMM_TIMEOUT:
      return "COMM_TIMEOUT";
    case MOTION_CMD_CAN_BUSOFF:
      return "CAN_BUSOFF";
    case MOTION_CMD_ENABLE:
      return "ENABLE";
    case MOTION_CMD_DISABLE:
      return "DISABLE";
    case MOTION_CMD_RELEASE:
      return "RELEASE";
    case MOTION_CMD_POWER_FAULT:
      return "POWER_FAULT";
    default:
      return "UNKNOWN";
  }
}

uint8_t motion_command_releases_session(
    motion_command_type_t type,
    motion_command_handle_result_t result)
{
  return ((type == MOTION_CMD_RELEASE) &&
          (result == MOTION_HANDLE_ACCEPTED)) ? 1U : 0U;
}
