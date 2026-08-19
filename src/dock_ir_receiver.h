#ifndef DOCK_IR_RECEIVER_H
#define DOCK_IR_RECEIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化连接IRM-H238T数字输出的PA8，不启用MCU内部上下拉。 */
void dock_ir_receiver_init(void);

/* 返回PA8原始电平：高电平为1，低电平为0。 */
uint8_t dock_ir_receiver_get_raw_level(void);

/*
 * IRM-H238T检测到38kHz载波时输出低电平。
 * 单次电平不包含方向和距离，禁止由本函数直接触发底盘运动。
 */
uint8_t dock_ir_receiver_is_carrier_active(void);

#ifdef __cplusplus
}
#endif

#endif
