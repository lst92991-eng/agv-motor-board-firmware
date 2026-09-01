/** @file chassis_protocol.h @brief 单上位机CAN FD运动协议。 */
#ifndef CHASSIS_PROTOCOL_H
#define CHASSIS_PROTOCOL_H

#include <stdint.h>

#include "drive_control.h"
#include "odometry.h"

typedef enum
{
  CHASSIS_CMD_NONE = 0,
  CHASSIS_CMD_ENABLE = 1,
  CHASSIS_CMD_DISABLE = 2,
  CHASSIS_CMD_STOP = 3,
  CHASSIS_CMD_ESTOP = 4,
  CHASSIS_CMD_FAULT_RESET = 5,
  CHASSIS_CMD_RESET_ODOMETRY = 6
} chassis_command_type_t;

typedef struct
{
  chassis_command_type_t type;
  uint16_t sequence;
} chassis_command_t;

typedef struct
{
  uint8_t state;
  uint8_t fault;
  uint8_t bumper_mask;
  uint8_t flags;
  uint16_t command_sequence;
  uint8_t command_result;
  int16_t left_pwm;
  int16_t right_pwm;
  uint16_t bus_mv;
  uint16_t left_current_ma;
  uint16_t right_current_ma;
} chassis_status_t;

void chassis_protocol_init(void);
void chassis_protocol_poll(uint32_t now_ms);
uint8_t chassis_protocol_take_command(chassis_command_t *command);
uint8_t chassis_protocol_get_reference(uint32_t now_ms, drive_reference_t *reference);
void chassis_protocol_clear_reference(void);
uint8_t chassis_protocol_bus_healthy(void);
void chassis_protocol_send_status(const chassis_status_t *status, uint32_t uptime_ms);
void chassis_protocol_send_odometry(const odometry_t *odometry,
                                    int16_t linear_mm_s,
                                    int16_t yaw_rate_cdeg_s,
                                    uint32_t uptime_ms);

#endif
