#ifndef BUMPER_INPUT_H
#define BUMPER_INPUT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 四个方向在状态字中的位定义，原始值1表示GPIO高，有效值1表示防撞条按下。 */
#define BUMPER_INPUT_RIGHT_MASK    0x01U
#define BUMPER_INPUT_LEFT_MASK     0x02U
#define BUMPER_INPUT_REAR_MASK     0x04U
#define BUMPER_INPUT_FRONT_MASK    0x08U
#define BUMPER_INPUT_ALL_MASK      0x0FU

/* 初始化四路防撞条GPIO和双边沿外部中断，GPIO不启用内部上下拉。 */
void bumper_input_init(void);

/* 读取四路GPIO原始电平，返回值位定义见上面的状态掩码。 */
uint8_t bumper_input_read_raw_mask(void);

/* 防撞条闭合后信号被拉低，返回当前低电平有效的方向。 */
uint8_t bumper_input_read_active_mask(void);

/* 原子读取并清除中断期间累计的变化方向。 */
uint8_t bumper_input_take_changed_mask(void);

/* 原子读取并清除中断期间锁存的按下方向，短促碰撞也不会被释放沿覆盖。 */
uint8_t bumper_input_take_pressed_latch_mask(void);

/* 由外部中断入口调用，只记录变化方向，不在中断中打印或延时。 */
void bumper_input_edge_irq_handler(uint8_t changed_mask);

#ifdef __cplusplus
}
#endif

#endif
