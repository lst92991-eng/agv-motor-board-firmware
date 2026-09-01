/**
 * @file canfd_protocol.h
 * @brief 电机板CAN FD业务PDO、控制会话和状态发布接口。
 *
 * 非安全运动只允许来自0x401；管理命令不得携带轮速或PWM。
 */

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
  /* 所有多字节字段由codec按小端逐字段编码，禁止直接发送结构体内存。 */
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

/** 排队32字节底盘状态PDO。 */
void canfd_protocol_send_drive_status(const canfd_drive_status_t *status);
/** 排队32字节里程计PDO。 */
void canfd_protocol_send_odom_status(const canfd_odom_status_t *status);
/* urgent非零时进入紧急TX队列，优先于周期状态。 */
void canfd_protocol_send_safety_status(
    const canfd_safety_status_t *status,
    uint8_t urgent);

/** 返回活动会话ID，无有效会话返回0。 */
uint32_t canfd_protocol_get_session_id(void);
/** 判断当前控制会话是否仍有效。 */
uint8_t canfd_protocol_is_session_valid(void);
/** 判断CAN3未bus-off且RX队列未溢出。 */
uint8_t canfd_protocol_is_bus_healthy(void);

/*
 * 仅在有效TTL、租约、ENABLE、bus-off和控制命令屏障全部通过时返回最新0x401。
 */
uint8_t canfd_protocol_get_motion_reference(
    agv_motion_reference_t *reference);

#ifdef __cplusplus
}
#endif

#endif
