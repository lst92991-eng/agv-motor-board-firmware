/** @file canfd_driver.h @brief CAN3 FD硬件收发和链路健康状态。 */
#ifndef CANFD_DRIVER_H
#define CANFD_DRIVER_H

#include <stdint.h>

typedef struct
{
  uint32_t id;
  uint8_t size;
  uint8_t data[32];
} canfd_frame_t;

void canfd_driver_init(void);
void canfd_driver_poll(uint32_t now_ms);
void canfd_driver_rx_irq(void);
uint8_t canfd_driver_take(canfd_frame_t *frame);
uint8_t canfd_driver_send(uint32_t id, const uint8_t *data, uint8_t size);
uint8_t canfd_driver_healthy(void);
void canfd_driver_recover(void);

#endif
