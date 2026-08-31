#ifndef CANFD_PROTOCOL_H
#define CANFD_PROTOCOL_H

#include <stdint.h>

#include "agv/motion_contract.h"
#include "motion_command.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CANFD_ID_CHASSIS_STATE               0x181U
#define CANFD_ID_CHASSIS_COMMAND             0x201U
#define CANFD_ID_COMMAND_RESULT              0x281U
#define CANFD_ID_CONTROL_LEASE               0x301U
#define CANFD_ID_ODOM_STATUS                 0x381U
#define CANFD_ID_MOTION_REFERENCE            AGV_CAN_ID_MOTION_REFERENCE
#define CANFD_ID_SAFETY_STATUS               0x481U
#define CANFD_ID_MOTOR_DIAGNOSTIC             0x581U
#define CANFD_ID_DOCK_STATUS                  0x582U
#define CANFD_ID_DOCK_SCAN                    0x583U

typedef enum
{
  CANFD_COMMAND_CLAIM_CONTROL = 0x01,
  CANFD_COMMAND_RELEASE_CONTROL = 0x02,
  CANFD_COMMAND_ENABLE_MOTION = 0x10,
  CANFD_COMMAND_DISABLE_MOTION = 0x11,
  CANFD_COMMAND_STOP = 0x30,
  CANFD_COMMAND_ESTOP = 0x31,
  CANFD_COMMAND_FAULT_RESET = 0x32,
  CANFD_COMMAND_RESET_ODOM = 0x40
} canfd_command_code_t;

typedef struct
{
  uint32_t command_id;
  uint32_t session_id;
  uint32_t fault_mask;
  uint32_t warning_mask;
  int32_t x_mm;
  int32_t y_mm;
  int32_t yaw_cdeg;
  uint8_t application_state;
  uint8_t motion_state;
  uint8_t motion_mode;
  uint8_t flags;
} canfd_drive_status_t;

typedef struct
{
  uint32_t uptime_ms;
  int32_t x_mm;
  int32_t y_mm;
  int32_t yaw_cdeg;
  int16_t linear_mm_s;
  int16_t yaw_rate_cdeg_s;
  int16_t accel_x_mg;
  int16_t accel_y_mg;
  int16_t accel_z_mg;
  uint16_t valid_flags;
} canfd_odom_status_t;

typedef struct
{
  uint32_t uptime_ms;
  int16_t left_target_rpm_x100;
  int16_t right_target_rpm_x100;
  int16_t left_measured_rpm_x100;
  int16_t right_measured_rpm_x100;
  int16_t left_pwm;
  int16_t right_pwm;
  int32_t left_encoder_count;
  int32_t right_encoder_count;
  uint8_t left_invalid_count;
  uint8_t right_invalid_count;
  uint8_t state;
  uint8_t ir_activity;
  uint8_t ir_activity_percent;
  uint8_t ir_raw_level;
  uint16_t ir_falling_edges;
} canfd_motor_status_t;

typedef struct
{
  uint32_t uptime_ms;
  uint32_t sample_sequence;
  uint16_t left_mm;
  uint16_t right_mm;
  int16_t distance_error_mm;
  int16_t alignment_error_mm;
  uint16_t ir_falling_edges;
  uint8_t flags;
  uint8_t ir_activity_percent;
  int8_t left_api_status;
  int8_t right_api_status;
  uint8_t left_range_status;
  uint8_t right_range_status;
  int32_t bus_error;
  uint16_t left_raw_mm;
  uint16_t right_raw_mm;
} canfd_dock_status_t;

typedef struct
{
  uint32_t sample_sequence;
  uint32_t sample_time_ms;
  uint16_t left_raw_mm;
  uint16_t right_raw_mm;
  uint8_t flags;
  uint8_t left_range_status;
  uint8_t right_range_status;
} canfd_dock_scan_t;

typedef struct
{
  uint32_t uptime_ms;
  uint32_t fault_mask;
  uint32_t warning_mask;
  uint8_t bumper_mask;
  uint8_t estop_source_mask;
  uint8_t safety_state;
} canfd_safety_status_t;

/* 初始化CAN3、接收中断、协议队列和统一运动命令回调。 */
void canfd_protocol_init(motion_command_handler_t handler, void *context);

/* 通信任务或无RTOS测试循环调用，处理接收、超时、bus-off恢复和发送队列。 */
void canfd_protocol_poll(uint32_t now_ms);

/* CAN3接收中断入口，只复制硬件帧到静态环形缓冲区。 */
void canfd_protocol_rx_irq_handler(void);

/* 发送命令接收/接受/拒绝结果。 */
void canfd_protocol_send_motion_result(
    const motion_command_t *command,
    motion_command_handle_result_t result,
    uint16_t fault_mask,
    uint8_t state);

void canfd_protocol_send_drive_status(const canfd_drive_status_t *status);
void canfd_protocol_send_odom_status(const canfd_odom_status_t *status);
void canfd_protocol_send_motor_status(const canfd_motor_status_t *status);
void canfd_protocol_send_dock_status(const canfd_dock_status_t *status);
void canfd_protocol_send_dock_scan(const canfd_dock_scan_t *status);
void canfd_protocol_send_safety_status(
    const canfd_safety_status_t *status,
    uint8_t urgent);

uint16_t canfd_protocol_get_link_age_ms(uint32_t now_ms);
uint32_t canfd_protocol_get_session_id(void);
uint8_t canfd_protocol_is_session_valid(void);
uint8_t canfd_protocol_get_bus_state_flags(void);
uint8_t canfd_protocol_is_bus_healthy(void);
uint32_t canfd_protocol_get_rx_count(void);
uint32_t canfd_protocol_get_tx_count(void);
uint32_t canfd_protocol_get_rx_drop_count(void);
uint32_t canfd_protocol_get_tx_drop_count(void);

/*
 * Return the newest accepted 0x401 reference while its effective TTL is
 * fresh and no newer control-command barrier is pending.
 */
uint8_t canfd_protocol_get_motion_reference(
    agv_motion_reference_t *reference);

#ifdef __cplusplus
}
#endif

#endif
