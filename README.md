# AGV电机驱动板教学固件

AT32F456RCT7底盘板的最小正式控制实现。上位机负责路线、RFID、目标距离和
`v/ω`计算；本板只负责实时差速执行、速度闭环、相对里程计和本地停车保护。

## 阅读主线

```text
CAN FD Reference → chassis_protocol → chassis_app
→ drive_control → motor_driver → AT8236
                 ↑ encoder / BMI270 / ADC DMA / bumper
```

目录只保留三层：

```text
src/app/       唯一10ms底盘状态机
src/services/  协议、差速PID、里程计和安全判断
src/drivers/   CAN、PWM、编码器、BMI270、ADC DMA、防撞和WDT
src/config/    唯一板级标定源
```

## 最小CAN FD协议

| 方向 | ID | 用途 |
| --- | ---: | --- |
| 上位机→底盘 | `0x201` | ENABLE、DISABLE、STOP、ESTOP、FAULT_RESET、RESET_ODOMETRY |
| 上位机→底盘 | `0x401` | sequence、TTL、线速度、角速度、加速度、减速度 |
| 底盘→上位机 | `0x181` | 状态、故障、防撞、电气和命令结果 |
| 底盘→上位机 | `0x381` | x、y、相对Yaw和实际速度 |

`0x401`必须周期刷新。TTL过期、CAN bus-off、防撞、IMU陈旧、堵转或电气故障都会
由本板主动制动并锁存FAULT。

## 构建和测试

```powershell
& .\tools\run_host_tests.ps1
& .\tools\build_motor_firmware.ps1
```

构建要求0错误、0告警。烧录后的人工验收顺序见
[教学架构与验收](docs/teaching/current_architecture.md)。
