# AGV串口调试协议

该协议只用于开发阶段的串口调试和上位机可视化。正式CAN FD协议继续使用定点二进制格式，但命令语义保持一致。

## 基本约定

- 波特率：115200，8N1。
- 编码：ASCII。
- 每条消息以 `\r\n` 结束。
- 多字节数值不直接发送，统一转换成十进制文本。
- 距离单位为毫米，航向单位为0.01度，速度单位为毫米每秒。
- 每次发送新的GOTO任务时，上位机和主控都以接收指令时的车辆位置为原点、车头方向为X轴正方向，重新建立任务坐标系。
- 新任务开始后车辆位姿显示为 `(0,0,0)`，GOTO中的目标坐标已经转换到该任务坐标系。

## 上位机下发

### 本地目标点

```text
CMD,GOTO,version,sequence,x_mm,y_mm,yaw_cdeg,speed_mm_s
```

示例：移动到任务坐标系 `(2m, 1m)`，目标航向30度，速度0.1m/s：

```text
CMD,GOTO,1,25,2000,1000,3000,100
```

### 平滑停止

```text
CMD,STOP,version,sequence
```

### 急停

```text
CMD,ESTOP,version,sequence
```

### 本地里程清零

```text
CMD,RESET_ODOM,version,sequence
```

## 主控上报

### 实时位姿

```text
POSE,timestamp_ms,x_mm,y_mm,yaw_cdeg,v_mm_s,omega_cdeg_s,left_rpm_x100,right_rpm_x100,state,fault,left_target_rpm_x100,right_target_rpm_x100,left_pwm,right_pwm,left_count,right_count,left_invalid,right_invalid
```

示例：

```text
POSE,123456,1520,-35,120,98,-15,2210,2195,LINE_TRACK,0,2250,2250,280,276,-15000,14980,0,0
```

当前主控以10Hz上报 `POSE`。前11个字段保持兼容，后8个字段用于电机和编码器诊断；旧上位机可忽略追加字段。

运动故障发生时，主控在刹车前额外输出一帧：

```text
MOTION_FAULT,reason,left_count=...,right_count=...,left_target=...,right_target=...,left_rpm=...,right_rpm=...,left_pwm=...,right_pwm=...,left_invalid=...,right_invalid=...
```

### 指令应答

```text
ACK,sequence,command,status,error
```

状态包括：

```text
ACCEPTED
RUNNING
COMPLETED
REJECTED
CANCELED
FAULT
RECEIVED
```

`RECEIVED`表示当前处于安全收发测试阶段，主控已经正确解析命令，但不会初始化或驱动电机。

示例：

```text
ACK,25,GOTO,ACCEPTED,0
ACK,25,GOTO,COMPLETED,0
```

## 当前兼容性

当前底盘测试固件仍输出纯数字CSV，工具只会从旧格式中显示状态、航向和轮速，不会根据旧数据伪造 `x、y`。要使用真实二维位置和点击目标功能，主控需要增加本地里程计、串口接收和上述 `POSE/ACK` 上报。
