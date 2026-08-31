#ifndef CHASSIS_MOTION_CONTROLLER_H
#define CHASSIS_MOTION_CONTROLLER_H

#include "motion_command.h"

/* 对齐32位跨任务快照：低两位为状态，高30位为ChassisCtrl发布epoch。 */
#define CHASSIS_POWER_STATUS_ZERO_SAFE          0x01UL
#define CHASSIS_POWER_STATUS_FAULT_LATCHED      0x02UL

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化速度环、编码器、BMI270、累计里程计和本地安全状态。 */
void chassis_motion_controller_init(void);

/* 只处理使能/停车/释放/复位和本地故障；运动量唯一来自0x401。 */
motion_command_handle_result_t chassis_motion_controller_handle_command(
    const motion_command_t *command);

/* 由最高优先级任务每10ms更新安全、IMU、里程计和参考执行器。 */
void chassis_motion_controller_step_10ms(void);

/* CAN FD应答和状态上报读取的只读状态。 */
uint8_t chassis_motion_controller_get_state(void);
uint16_t chassis_motion_controller_get_fault_mask(void);
uint32_t chassis_motion_controller_get_uptime_ms(void);
uint32_t chassis_motion_controller_get_power_status_snapshot(void);
uint8_t chassis_motion_controller_has_power_fault(void);

#ifdef __cplusplus
}
#endif

#endif
