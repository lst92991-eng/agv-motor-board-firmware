/**
 * @file app_freertos.h
 * @brief 电机板静态RTOS组合根和失败入口。
 */

#ifndef APP_FREERTOS_H
#define APP_FREERTOS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 创建静态队列和四个关键任务；任一资源创建失败返回0。 */
uint8_t app_freertos_init(void);

/* 启动调度器；正常情况下本函数不会返回。 */
void app_freertos_start(void);

/* FreeRTOS断言失败入口；实现必须立即主动制动且不返回。 */
void app_freertos_assert_failed(const char *file, int line);

#ifdef __cplusplus
}
#endif

#endif
