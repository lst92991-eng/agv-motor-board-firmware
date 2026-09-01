# AGV电机驱动板正式固件

AT32F456RCT7底盘控制固件。当前生产版本使用FreeRTOS，通过CAN FD接收Supervisor
仲裁后的短TTL `MotionReference(v, omega)`，在电机板本地完成差速解算、双电机
速度反馈、IMU/里程计、防撞、电气监督和失联停车。路线与磁跟踪算法不在电机板。
平台硬上限为直线`300 mm/s`、角速度`1200 mrad/s`和单轮`75 RPM`；曲线
`100 mm/s`属于Supervisor路线策略，只能进一步收紧这些上限。

## 正式入口

`src/main.c`只有一个生产入口，不再通过`app_mode.h`切换独立测试程序。主要任务：

| 任务 | 周期 | 职责 |
| --- | ---: | --- |
| `ChassisCtrl` | 10ms | 唯一参考执行器、本地安全、里程计与遥测 |
| `CANFD` | 2ms | 整车CAN FD协议、控制租约和命令结果 |
| `PowerMonitor` | 20ms | ADC、电流/母线监督与故障提交 |
| `Watchdog` | 20ms | Control/CAN/Power均推进时才喂狗 |

## 已验收能力

- 前进、后退、左转、右转；
- 双编码器速度PID（当前Kd为0）和BMI270/编码器航向融合；
- 磁条路线的历史实车证据仍有效；新架构由Supervisor磁producer生成统一参考，
  尚待重新烧录回归；
- 四路防撞条本地锁存停车；
- CAN FD 500k/4M BRS、200ms控制租约、bus-off与通信超时停车；
- 整车CAN状态、里程计和安全帧。

关键生产模块边界：

- `chassis_motion_controller.*`：唯一0x401执行器、本地安全、里程计和遥测；
- `motion_reference_receiver/control.*`：latest-only、TTL、差速、共同限幅和斜率；
- `canfd_protocol.*`：CAN硬件队列、命令、租约和bus-off恢复；
- `canfd_status_codec.*`：状态PDO固定小端字节布局，可在主机独立回归。

TOF和IR已经移出生产固件，后续属于独立对桩传感节点。电机板只保留
`power_supervisor.*`与`adc_monitor.*`的失效关闭电流/母线监督。PN532/RFID未来
只能作为上层事件producer接入，不能恢复电机板运动分支。

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

当前覆盖航向融合、CAN FD状态PDO、运动参考、看门狗、电气监督和速度环。
当前精简固件不保留调试UART和独立电气诊断PDO；首次电气标定只能通过SWD读取
原始ADC与监督状态。生产Target按开发动态验收要求启用保守的6串母线与0.8A/100ms
电流门；最终产品定版前仍必须用实测证据替换开发参数。测试顺序见
[电气监督首次标定](hardware/power_commissioning_acceptance.md)。

## 关键文档

- [能力基线](hardware/motor_board_capability_baseline_2026-08-19.md)
- [磁条循迹验收](hardware/chassis_magnetic_straight_validation_2026-08-18.md)
- [四方向验收](hardware/chassis_four_direction_ground_validation_2026-08-18.md)
- [安全与防撞回归](hardware/chassis_p0_safety_and_bumper_regression_2026-08-17.md)
- [整车CANopen FD协议](../../docs/AGV_CANopen_FD_Protocol_V0.2.md)
