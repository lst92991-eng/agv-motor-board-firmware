#ifndef MOTOR_SPEED_TUNING_H
#define MOTOR_SPEED_TUNING_H

/*
 * 速度环首轮整定参数。
 *
 * 依据：20ms 控制周期、2640 count/输出轴圈，以及旧板开环标定
 * （约 PWM=200/300/400/500/700/1000 对应 17/27/37/47/67/97RPM）。
 * 两侧电机共用同一套初值；整车联调时应以 PID_TEST 的串口记录分别微调。
 */

#define MOTOR_SPEED_CONTROL_PERIOD_MS                20U
#define MOTOR_SPEED_ENCODER_COUNTS_PER_OUTPUT_REV   2640.0f
#define MOTOR_SPEED_TARGET_LIMIT_RPM                  75.0f
#define MOTOR_SPEED_RAW_RPM_LIMIT                    120.0f

/* 前馈保留实车低速克服静摩擦所需的偏置，PI 只补偿负载和个体差异。 */
#define MOTOR_SPEED_FEED_FORWARD_PER_RPM               9.4f
#define MOTOR_SPEED_FEED_FORWARD_STATIC_PWM           50.0f
#define MOTOR_SPEED_PWM_SLEW_PER_CYCLE                12.0f

/* 20ms、2640 count/rev 时单个计数约为 1.14RPM；0.20 兼顾量化噪声与响应。 */
#define MOTOR_SPEED_FILTER_ALPHA                       0.20f

/* Kd 保持为零，避免正交编码器量化噪声被微分项放大。 */
#define MOTOR_SPEED_LEFT_PID_KP                        4.0f
#define MOTOR_SPEED_LEFT_PID_KI                        8.0f
#define MOTOR_SPEED_LEFT_PID_KD                        0.0f
#define MOTOR_SPEED_RIGHT_PID_KP                       4.0f
#define MOTOR_SPEED_RIGHT_PID_KI                       8.0f
#define MOTOR_SPEED_RIGHT_PID_KD                       0.0f
#define MOTOR_SPEED_PID_CORRECTION_LIMIT             250.0f

/* 启动助推只用于克服静摩擦；检测到转动后尽快退出，最长 400ms。 */
#define MOTOR_SPEED_START_BOOST_MIN_PWM              180.0f
#define MOTOR_SPEED_START_BOOST_EXTRA_PWM             60.0f
#define MOTOR_SPEED_START_BOOST_MAX_PWM              350.0f
#define MOTOR_SPEED_START_BOOST_TOTAL_CYCLES           20U
#define MOTOR_SPEED_START_BOOST_MIN_CYCLES              5U
#define MOTOR_SPEED_START_BOOST_READY_CYCLES            2U
#define MOTOR_SPEED_START_BOOST_EXIT_RPM                5.0f

/*
 * 反馈符号沿用已完成的单轮测试结论。新板接线后，架空车轮以正向 12RPM
 * 测试一次；若某侧 measured_rpm 为负，仅翻转对应宏，不改 PID 参数。
 */
#define MOTOR_SPEED_LEFT_FEEDBACK_SIGN                 1.0f
#define MOTOR_SPEED_RIGHT_FEEDBACK_SIGN               -1.0f

#endif
