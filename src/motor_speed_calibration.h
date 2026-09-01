#ifndef MOTOR_SPEED_CALIBRATION_H
#define MOTOR_SPEED_CALIBRATION_H

/*
 * 已通过整车落地验证的速度环标定参数。
 *
 * 依据：20ms控制周期、2640 count/输出轴圈，以及实际37R电机和85mm主动轮。
 * 参数变更必须重新完成架空方向检查、落地四方向检查和循迹回归，不能在普通
 * 运行固件中提供无边界在线改写入口。
 */

#define MOTOR_SPEED_CONTROL_PERIOD_MS                20U
#define MOTOR_SPEED_ENCODER_COUNTS_PER_OUTPUT_REV   2640.0f
#define MOTOR_SPEED_TARGET_LIMIT_RPM                  75.0f
#define MOTOR_SPEED_RAW_RPM_LIMIT                    120.0f

/* 前馈克服低速静摩擦；PID补偿负载和个体差异，D当前保持0等待实车曲线。 */
#define MOTOR_SPEED_FEED_FORWARD_PER_RPM               9.4f
#define MOTOR_SPEED_FEED_FORWARD_STATIC_PWM          100.0f
/* 20ms、2640 count/rev时单个计数约为1.14RPM。 */
#define MOTOR_SPEED_FILTER_ALPHA                       0.20f

#define MOTOR_SPEED_LEFT_PID_KP                        4.0f
#define MOTOR_SPEED_LEFT_PID_KI                        8.0f
#define MOTOR_SPEED_LEFT_PID_KD                        0.0f
#define MOTOR_SPEED_RIGHT_PID_KP                       4.0f
#define MOTOR_SPEED_RIGHT_PID_KI                       8.0f
#define MOTOR_SPEED_RIGHT_PID_KD                       0.0f
#define MOTOR_SPEED_PID_CORRECTION_LIMIT             250.0f

/* 目标存在但200ms无反馈时只锁存停车，不执行PWM脱困。 */
#define MOTOR_SPEED_STALL_TARGET_MIN_RPM                 8.0f
#define MOTOR_SPEED_STALL_MEASURED_MAX_RPM               1.0f
#define MOTOR_SPEED_STALL_DETECT_CYCLES                 10U
#define MOTOR_SPEED_REVERSE_ZERO_RPM                      1.0f
#define MOTOR_SPEED_REVERSE_ZERO_SAMPLES                    3U

/*
 * 左右反馈符号来自实车方向验收。两个电机安装方向相反，正式控制层已经统一为
 * “正值表示车辆向前”，禁止通过试错同时修改驱动极性和反馈符号。
 */
#define MOTOR_SPEED_LEFT_FEEDBACK_SIGN                 1.0f
#define MOTOR_SPEED_RIGHT_FEEDBACK_SIGN               -1.0f

#endif
