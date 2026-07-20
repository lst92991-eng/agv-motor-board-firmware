#ifndef SERIAL_MOTION_PROTOCOL_H
#define SERIAL_MOTION_PROTOCOL_H

#include "motion_command.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化串口运动协议，并注册统一运动命令处理器。 */
void serial_motion_protocol_init(motion_command_handler_t handler,
                                 void *context);

/* 非阻塞处理接收缓冲区中的所有完整命令行。 */
void serial_motion_protocol_poll(void);

/* 由控制任务在真正处理队列命令后发送最终接受或拒绝结果。 */
void serial_motion_protocol_send_result(
    const motion_command_t *command,
    motion_command_handle_result_t result);

#ifdef __cplusplus
}
#endif

#endif
