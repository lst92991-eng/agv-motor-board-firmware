# 教学架构与人工验收

## 一条控制链

```text
上位机 v/ω
  → CAN FD驱动
  → 协议解析
  → 10ms底盘状态机
  → 差速与20ms速度PID
  → AT8236 PWM
  → 电机与编码器
```

`chassis_app`是唯一状态机：

```text
DISABLED → ENABLED → RUNNING
     ↑                 ↓
     └──── FAULT ←─────┘
```

## CAN FD教学协议

| 方向 | ID | 内容 |
| --- | ---: | --- |
| 上位机→底盘 | `0x201` | 控制命令和递增sequence |
| 上位机→底盘 | `0x401` | sequence、TTL、`v`、`ω`、加速度、减速度 |
| 底盘→上位机 | `0x181` | 状态、故障、防撞、电压、电流和命令结果 |
| 底盘→上位机 | `0x381` | x、y、相对Yaw、实际速度 |

`0x401`持续刷新就是通信租约。Reference TTL到期后，本板主动制动并锁存FAULT。

`0x201`固定8字节：`[命令码, 0, sequence_u16, 0_u32]`。命令码依次为1=ENABLE、
2=DISABLE、3=STOP、4=ESTOP、5=FAULT_RESET、6=RESET_ODOMETRY。

`0x401`固定16字节小端布局：`sequence_u16, ttl_ms_u16, v_mm_s_i16,
omega_mrad_s_i16, accel_mm_s2_u16, decel_mm_s2_u16, reserved_u32`。TTL范围50～300ms；
本板限制`|v|≤300mm/s`、`|omega|≤1200mrad/s`。

## 模块阅读顺序

1. `drivers/motor_driver.c`：20kHz PWM、方向、Coast和Brake。
2. `drivers/motor_encoder.c`：左右AB编码器计数。
3. `drivers/adc_dma.c`、`bumper.c`、`imu_bmi270.c`：硬件输入。
4. `services/drive_control.c`：差速、共同限幅、斜率、前馈和PID。
5. `services/odometry.c`：编码器轮程和IMU航向融合。
6. `services/safety.c`：母线、电流和故障锁存。
7. `services/chassis_protocol.c`：四种CAN FD报文。
8. `app/chassis_app.c`：按固定顺序组合所有模块。

## 人工验收

1. 架空：ENABLE后前进、后退、左右转和原地旋转；STOP必须制动。
2. 通信：停止发送0x401，确认TTL到期后两电机主动制动并进入FAULT。
3. 防撞：逐个按下四路防撞条，确认立即制动且FAULT_RESET前不能重新ENABLE。
4. 电气：确认ADC DMA样本连续；母线越出18.0～25.8V不能ENABLE；0.8A持续100ms停车。
5. 里程计：直行、后退、左右360°旋转和方形闭环；据实车结果标定轮周长和有效轮距。

相对里程计用于两次RFID、磁条或视觉校正之间的连续估算，不能替代上位机绝对定位。
