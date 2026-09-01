/**
 * @file board_config.h
 * @brief 底盘板唯一硬件、运动和保护标定源。
 *
 * 所有数值均是本板已经验收或需要人工复验的物理限制。上位机只能发送更保守的
 * Reference，不能通过通信字段放宽这些限制。
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* 固定控制节拍。 */
#define BOARD_CONTROL_PERIOD_MS             10U
#define BOARD_SPEED_PERIOD_MS               20U
#define BOARD_STATUS_PERIOD_MS             100U
#define BOARD_ADC_PERIOD_MS                 20U
#define BOARD_REFERENCE_TIMEOUT_MS         120U
#define BOARD_IMU_STALE_MS                  50U

/* 差速底盘几何和车辆坐标：X向前、Y向左、左转为正。 */
#define BOARD_WHEEL_CIRCUMFERENCE_MM    267.035f
#define BOARD_TRACK_WIDTH_MM            500.0f
#define BOARD_ENCODER_COUNTS_PER_REV   2640.0f
#define BOARD_MAX_LINEAR_MM_S             300.0f
#define BOARD_MAX_ANGULAR_MRAD_S         1200.0f
#define BOARD_MAX_WHEEL_RPM                75.0f
#define BOARD_YAW_RATE_GAIN                 0.35f

/* 速度环：PWM单位为千分比。Kd保留为0，便于课堂说明完整PID结构。 */
#define BOARD_PID_KP                         4.0f
#define BOARD_PID_KI                         8.0f
#define BOARD_PID_KD                         0.0f
#define BOARD_PID_LIMIT                    250.0f
#define BOARD_FF_STATIC_PWM                100.0f
#define BOARD_FF_PER_RPM                     9.4f
#define BOARD_RPM_FILTER_ALPHA               0.20f
#define BOARD_RAW_RPM_LIMIT                120.0f
#define BOARD_STALL_TARGET_RPM               8.0f
#define BOARD_STALL_MEASURED_RPM             1.0f
#define BOARD_STALL_TICKS                   10U

/* 左右硬件安装方向冻结于实车方向验收。 */
#define BOARD_LEFT_TARGET_SIGN              -1.0f
#define BOARD_RIGHT_TARGET_SIGN              1.0f
#define BOARD_LEFT_FEEDBACK_SIGN             1.0f
#define BOARD_RIGHT_FEEDBACK_SIGN           -1.0f

/* 电气监督：PA3母线，PA4/PA5分别为左右INA180。 */
#define BOARD_ADC_FULL_SCALE               4095U
#define BOARD_BUS_UV_PER_COUNT           8850UL
#define BOARD_BUS_OFFSET_MV                  0L
#define BOARD_CURRENT_UA_PER_COUNT       8060UL
#define BOARD_BUS_UV_TRIP_MV            18000UL
#define BOARD_BUS_UV_RECOVER_MV         19200UL
#define BOARD_BUS_OV_TRIP_MV            25800UL
#define BOARD_BUS_OV_RECOVER_MV         25200UL
#define BOARD_PEAK_CURRENT_MA             800U
#define BOARD_PEAK_CURRENT_TICKS            5U
#define BOARD_ADC_STALE_MS                  60U
#define BOARD_ADC_ZERO_SAMPLES              20U
#define BOARD_ADC_ZERO_MAX_RAW              19U

/* 简化CAN FD教学协议。 */
#define BOARD_CAN_ID_CONTROL             0x201U
#define BOARD_CAN_ID_STATUS              0x181U
#define BOARD_CAN_ID_ODOMETRY            0x381U
#define BOARD_CAN_ID_REFERENCE           0x401U
#define BOARD_CAN_REFERENCE_SIZE           16U

#endif
