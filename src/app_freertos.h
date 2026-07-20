#ifndef APP_FREERTOS_H
#define APP_FREERTOS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 创建静态命令队列、控制任务和通信任务，成功返回1。 */
uint8_t app_freertos_init(void);

/* 启动调度器；正常情况下本函数不会返回。 */
void app_freertos_start(void);

/* FreeRTOS断言失败入口。 */
void app_freertos_assert_failed(const char *file, int line);

#ifdef __cplusplus
}
#endif

#endif
