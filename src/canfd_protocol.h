#ifndef CANFD_PROTOCOL_H
#define CANFD_PROTOCOL_H

#include <stdint.h>

#include "motion_command.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CANFD_PROTOCOL_VERSION                 1U
#define CANFD_PROTOCOL_NODE_ID                 1U

#define CANFD_ID_CHASSIS_STATE               0x181U
#define CANFD_ID_CHASSIS_COMMAND             0x201U
#define CANFD_ID_COMMAND_RESULT              0x281U
#define CANFD_ID_CONTROL_LEASE               0x301U
#define CANFD_ID_ODOM_STATUS                 0x381U
#define CANFD_ID_VELOCITY                    0x401U
#define CANFD_ID_SAFETY_STATUS               0x481U

typedef enum
{
  CANFD_COMMAND_CLAIM_CONTROL = 0x01,
  CANFD_COMMAND_RELEASE_CONTROL = 0x02,
  CANFD_COMMAND_ENABLE_MOTION = 0x10,
  CANFD_COMMAND_DISABLE_MOTION = 0x11,
  CANFD_COMMAND_GOTO_REL = 0x20,
  CANFD_COMMAND_FOLLOW_MAG_START = 0x21,
  CANFD_COMMAND_STOP = 0x30,
  CANFD_COMMAND_ESTOP = 0x31,
  CANFD_COMMAND_FAULT_RESET = 0x32,
  CANFD_COMMAND_RESET_ODOM = 0x40
} canfd_command_code_t;

typedef enum
{
  CANFD_ACK_RECEIVED = 0,
  CANFD_ACK_ACCEPTED = 1,
  CANFD_ACK_COMPLETED = 2,
  CANFD_ACK_CANCELED = 3,
  CANFD_ACK_DUPLICATE = 4,
  CANFD_ACK_REJECTED = 5,
  CANFD_ACK_FAULT = 6
} canfd_ack_status_t;

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
} canfd_motor_status_t;

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

/* 发送任务完成、取消或运行故障事件。 */
void canfd_protocol_send_command_event(
    uint32_t sequence,
    uint32_t session_id,
    motion_command_type_t type,
    canfd_ack_status_t status,
    motion_command_error_t error,
    uint16_t fault_mask,
    uint8_t state);

void canfd_protocol_send_drive_status(const canfd_drive_status_t *status);
void canfd_protocol_send_odom_status(const canfd_odom_status_t *status);
void canfd_protocol_send_motor_status(const canfd_motor_status_t *status);
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

#ifdef __cplusplus
}
#endif

#endif
