/** @file board_port.h @brief 中断入口和独立硬件看门狗。 */
#ifndef BOARD_PORT_H
#define BOARD_PORT_H

#include <stdint.h>

uint8_t board_port_watchdog_init(void);
void board_port_watchdog_feed(void);

#endif
