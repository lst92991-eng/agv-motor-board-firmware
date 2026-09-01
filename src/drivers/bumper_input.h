/**
 * @file bumper_input.h
 * @brief 四路防撞条原始输入与ISR锁存接口。
 *
 * 位图固定为Bit0左、Bit1后、Bit2前、Bit3右，与CAN安全PDO一致。
 */

#ifndef BUMPER_INPUT_H
#define BUMPER_INPUT_H

#include <stdint.h>

#define BUMPER_INPUT_LEFT_MASK   0x01U
#define BUMPER_INPUT_REAR_MASK   0x02U
#define BUMPER_INPUT_FRONT_MASK  0x04U
#define BUMPER_INPUT_RIGHT_MASK  0x08U
#define BUMPER_INPUT_ALL_MASK    0x0FU

/** 初始化低有效GPIO、双边沿EXINT和按下锁存。 */
void bumper_input_init(void);
/** 返回当前低有效输入转换后的有效位图。 */
uint8_t bumper_input_read_active_mask(void);
/** 原子取出并清除ISR累计的按下位图。 */
uint8_t bumper_input_take_pressed_latch_mask(void);
/** 清故障前检查是否仍有尚未消费的ISR按下证据。 */
uint8_t bumper_input_has_pressed_latch(void);
/** 由对应EXINT ISR调用；不得在此执行制动或RTOS操作。 */
void bumper_input_edge_irq_handler(uint8_t changed_mask);

#endif
