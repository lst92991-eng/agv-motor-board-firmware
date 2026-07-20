/**
  *****************************************************************************
  * @file     motion_command.c
  * @brief    串口和CAN FD共用的运动命令定义与校验
  *****************************************************************************
  */

#include "motion_command.h"

#include <stddef.h>

#define MOTION_GOTO_POSITION_LIMIT_MM       100000L
#define MOTION_GOTO_YAW_LIMIT_CDEG           18000L
#define MOTION_GOTO_SPEED_MIN_MM_S              10U
#define MOTION_GOTO_SPEED_MAX_MM_S            1000U

motion_command_error_t motion_command_validate(const motion_command_t *command)
{
  const motion_goto_local_data_t *goto_data;

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

  if(command->type == MOTION_CMD_GOTO_LOCAL)
  {
    goto_data = &command->data.goto_local;
    if((goto_data->x_mm < -MOTION_GOTO_POSITION_LIMIT_MM) ||
       (goto_data->x_mm > MOTION_GOTO_POSITION_LIMIT_MM) ||
       (goto_data->y_mm < -MOTION_GOTO_POSITION_LIMIT_MM) ||
       (goto_data->y_mm > MOTION_GOTO_POSITION_LIMIT_MM) ||
       (goto_data->yaw_cdeg < -MOTION_GOTO_YAW_LIMIT_CDEG) ||
       (goto_data->yaw_cdeg > MOTION_GOTO_YAW_LIMIT_CDEG) ||
       (goto_data->speed_mm_s < MOTION_GOTO_SPEED_MIN_MM_S) ||
       (goto_data->speed_mm_s > MOTION_GOTO_SPEED_MAX_MM_S))
    {
      return MOTION_COMMAND_ERROR_RANGE;
    }
  }
  else if((command->type != MOTION_CMD_STOP) &&
          (command->type != MOTION_CMD_ESTOP) &&
          (command->type != MOTION_CMD_RESET_ODOM))
  {
    return MOTION_COMMAND_ERROR_UNSUPPORTED;
  }

  return MOTION_COMMAND_ERROR_NONE;
}

const char *motion_command_get_name(motion_command_type_t type)
{
  switch(type)
  {
    case MOTION_CMD_GOTO_LOCAL:
      return "GOTO";
    case MOTION_CMD_STOP:
      return "STOP";
    case MOTION_CMD_ESTOP:
      return "ESTOP";
    case MOTION_CMD_RESET_ODOM:
      return "RESET_ODOM";
    default:
      return "UNKNOWN";
  }
}
