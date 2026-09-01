/** @file chassis_app.h @brief 唯一底盘状态机和10ms控制任务。 */
#ifndef CHASSIS_APP_H
#define CHASSIS_APP_H

#include <stdint.h>

uint8_t chassis_app_init(void);
void chassis_app_step(void);

#endif
