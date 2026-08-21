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

/* 由2ms通信任务调用，形成100ms低电平命中率窗口。 */
void dock_ir_receiver_sample_2ms(void);

/* 最近完整100ms窗口内是否检测到至少两个载波命中样本。 */
uint8_t dock_ir_receiver_get_activity(void);

/* 最近完整窗口的低电平采样占比，范围0..100。 */
uint8_t dock_ir_receiver_get_activity_percent(void);

/* 最近完整100ms窗口内PA8下降沿数量，饱和到65535。 */
uint16_t dock_ir_receiver_get_falling_edges(void);

/* EXINT9_5中断入口调用，只做饱和计数。 */
void dock_ir_receiver_falling_edge_irq_handler(void);

#ifdef __cplusplus
}
#endif

#endif
