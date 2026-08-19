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
#define MOTION_FOLLOW_MAG_DISTANCE_MIN_MM       300L
#define MOTION_FOLLOW_MAG_DISTANCE_MAX_MM      6000L
#define MOTION_FOLLOW_MAG_OFFSET_LIMIT_X10MM    100L
#define MOTION_FOLLOW_MAG_SPEED_MIN_MM_S          50U
#define MOTION_FOLLOW_MAG_SPEED_MAX_MM_S         330U
#define MOTION_VELOCITY_LINEAR_LIMIT_MM_S       100L
#define MOTION_VELOCITY_ANGULAR_LIMIT_MRAD_S    400L
#define MOTION_VELOCITY_ACCEL_MIN_MM_S2          10U
#define MOTION_VELOCITY_ACCEL_MAX_MM_S2         500U
#define MOTION_VELOCITY_DECEL_MIN_MM_S2          10U
#define MOTION_VELOCITY_DECEL_MAX_MM_S2        1000U
#define MOTION_VELOCITY_VALID_MIN_MS             50U
#define MOTION_VELOCITY_VALID_MAX_MS            500U

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
  else if(command->type == MOTION_CMD_FOLLOW_MAG)
  {
    const motion_follow_mag_data_t *follow = &command->data.follow_mag;

    if((follow->distance_mm < MOTION_FOLLOW_MAG_DISTANCE_MIN_MM) ||
       (follow->distance_mm > MOTION_FOLLOW_MAG_DISTANCE_MAX_MM) ||
       (follow->target_offset_x10mm <
        -MOTION_FOLLOW_MAG_OFFSET_LIMIT_X10MM) ||
       (follow->target_offset_x10mm >
        MOTION_FOLLOW_MAG_OFFSET_LIMIT_X10MM) ||
       (follow->allow_multi_peak > 1U) ||
       (follow->speed_mm_s < MOTION_FOLLOW_MAG_SPEED_MIN_MM_S) ||
       (follow->speed_mm_s > MOTION_FOLLOW_MAG_SPEED_MAX_MM_S) ||
       (follow->timeout_ms == 0UL) ||
       (follow->timeout_ms > 120000UL))
    {
      return MOTION_COMMAND_ERROR_RANGE;
    }
  }
  else if(command->type == MOTION_CMD_SET_VELOCITY)
  {
    const motion_velocity_data_t *velocity = &command->data.velocity;

    if((velocity->linear_mm_s < -MOTION_VELOCITY_LINEAR_LIMIT_MM_S) ||
       (velocity->linear_mm_s > MOTION_VELOCITY_LINEAR_LIMIT_MM_S) ||
       (velocity->angular_mrad_s <
        -MOTION_VELOCITY_ANGULAR_LIMIT_MRAD_S) ||
       (velocity->angular_mrad_s >
        MOTION_VELOCITY_ANGULAR_LIMIT_MRAD_S) ||
       (velocity->accel_mm_s2 < MOTION_VELOCITY_ACCEL_MIN_MM_S2) ||
       (velocity->accel_mm_s2 > MOTION_VELOCITY_ACCEL_MAX_MM_S2) ||
       (velocity->decel_mm_s2 < MOTION_VELOCITY_DECEL_MIN_MM_S2) ||
       (velocity->decel_mm_s2 > MOTION_VELOCITY_DECEL_MAX_MM_S2) ||
       (velocity->valid_time_ms < MOTION_VELOCITY_VALID_MIN_MS) ||
       (velocity->valid_time_ms > MOTION_VELOCITY_VALID_MAX_MS))
    {
      return MOTION_COMMAND_ERROR_RANGE;
    }
  }
  else if((command->type != MOTION_CMD_STOP) &&
          (command->type != MOTION_CMD_ESTOP) &&
          (command->type != MOTION_CMD_RESET_ODOM) &&
          (command->type != MOTION_CMD_CLEAR_FAULT) &&
          (command->type != MOTION_CMD_COMM_TIMEOUT) &&
          (command->type != MOTION_CMD_CAN_BUSOFF) &&
          (command->type != MOTION_CMD_ENABLE) &&
          (command->type != MOTION_CMD_DISABLE))
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
    case MOTION_CMD_FOLLOW_MAG:
      return "FOLLOW_MAG";
    case MOTION_CMD_STOP:
      return "STOP";
    case MOTION_CMD_ESTOP:
      return "ESTOP";
    case MOTION_CMD_RESET_ODOM:
      return "RESET_ODOM";
    case MOTION_CMD_CLEAR_FAULT:
      return "CLEAR_FAULT";
    case MOTION_CMD_SET_VELOCITY:
      return "VELOCITY";
    case MOTION_CMD_COMM_TIMEOUT:
      return "COMM_TIMEOUT";
    case MOTION_CMD_CAN_BUSOFF:
      return "CAN_BUSOFF";
    case MOTION_CMD_ENABLE:
      return "ENABLE";
    case MOTION_CMD_DISABLE:
      return "DISABLE";
    default:
      return "UNKNOWN";
  }
}
