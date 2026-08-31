#ifndef CHASSIS_MOTION_CONTROLLER_H
#define CHASSIS_MOTION_CONTROLLER_H

#include "motion_command.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化双电机速度环、编码器、BMI270和底盘状态。 */
void chassis_motion_controller_init(void);

/* 由底盘控制任务处理一条已从队列取出的统一运动命令。 */
motion_command_handle_result_t chassis_motion_controller_handle_command(
    const motion_command_t *command);

/* 由最高优先级控制任务每10ms调用一次。 */
void chassis_motion_controller_step_10ms(void);

/* CAN FD应答和状态上报读取的只读状态。 */
uint8_t chassis_motion_controller_get_state(void);
uint16_t chassis_motion_controller_get_fault_mask(void);
uint32_t chassis_motion_controller_get_uptime_ms(void);

#ifdef __cplusplus
}
#endif

#endif
