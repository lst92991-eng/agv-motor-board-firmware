# AGV电机驱动板正式固件

AT32F456RCT7底盘控制固件。当前生产版本使用FreeRTOS，通过CAN FD接收Supervisor
任务，在电机板本地完成双电机速度闭环、IMU航向、里程计、磁条循迹、防撞锁存和
失联停车。

## 正式入口

`src/main.c`只有一个生产入口，不再通过`app_mode.h`切换独立测试程序。主要任务：

| 任务 | 周期 | 职责 |
| --- | ---: | --- |
| `ChassisCtrl` | 10ms | 运动状态机、安全处理、速度目标与遥测 |
| `SerialComm` | 2ms | 调试串口命令解析 |
| `CANFD` | 2ms | 整车CAN FD协议、控制租约和命令结果 |

## 已验收能力

- 前进、后退、左转、右转；
- 双编码器速度PI和BMI270/编码器航向融合；
- 已验收的直线磁条循迹闭环；圆滑L型仍待实车验收；
- 四路防撞条本地锁存停车；
- CAN FD 500k/4M BRS、200ms控制租约、bus-off与通信超时停车；
- 调试串口和整车CAN诊断帧。

关键生产模块边界：

- `chassis_motion_controller.*`：任务状态机、安全联动和最终轮速目标；
- `magnetic_follow_controller.*`：磁样本质量、偏移PD外环和动态限速；
- `canfd_protocol.*`：CAN硬件队列、命令、租约和bus-off恢复；
- `canfd_status_codec.*`：状态PDO固定小端字节布局，可在主机独立回归。

## 已沉淀但未接入运行任务

- `dock_tof_sensor.*`：双VL53L0X采集、滤波、距离和对准误差；
- `dock_ir_receiver.*`：IRM-H238T原始38kHz载波状态；
- `pn532_uart5.*`、`rfid_record.*`：RFID底层能力；
- `adc_monitor.*`、`motor_current.*`：ADC和电流采集基础能力。

这些模块不代表充电对准、RFID定位或电流保护已经完成。正式对接前仍需增加独立
应用状态机、CAN对象、故障策略和实车验收。

## 构建

```powershell
& .\tools\build_motor_firmware.ps1
```

脚本要求`0 Error(s), 0 Warning(s)`，随后生成`mdk_v5/objects/agv_motor.bin`并输出
大小和SHA-256。生成物位于忽略目录，不得提交。

主机算法回归：

```powershell
& .\tools\run_host_tests.ps1
```

当前覆盖航向融合、磁循迹控制和CAN FD状态PDO编码。

## 关键文档

- [能力基线](hardware/motor_board_capability_baseline_2026-08-19.md)
- [磁条循迹验收](hardware/chassis_magnetic_straight_validation_2026-08-18.md)
- [四方向验收](hardware/chassis_four_direction_ground_validation_2026-08-18.md)
- [安全与防撞回归](hardware/chassis_p0_safety_and_bumper_regression_2026-08-17.md)
- [整车CAN FD协议](hardware/canfd_protocol_v1.md)
