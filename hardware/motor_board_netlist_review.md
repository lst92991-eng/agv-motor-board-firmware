# 电机驱动板网表核对（2026-08-11）

来源：新版电机驱动板 `netlist.enet`，主控为 AT32F456RCT7。

| 功能 | 最终网络 | MCU/器件 | 固件结论 |
| --- | --- | --- | --- |
| 左电机驱动 | `L_IN1`、`L_IN2` | PC6/TMR8_CH1、PC7/TMR8_CH2 -> AT8236 U1 IN1/IN2 | 保持 20kHz 双 PWM 慢衰减控制 |
| 右电机驱动 | `R_IN1`、`R_IN2` | PC8/TMR8_CH3、PC9/TMR8_CH4 -> AT8236 U3 IN1/IN2 | 保持 20kHz 双 PWM 慢衰减控制 |
| 左编码器 | `L_ENA`、`L_ENB` | PA6、PA7；CN3 的 3、4 脚经 100Ω/1nF 滤波 | 保持双边沿正交解码 |
| 右编码器 | `R_ENA`、`R_ENB` | PC4、PC5；CN2 的 3、4 脚经 100Ω/1nF 滤波 | 保持双边沿正交解码 |
| 左电流 | U15 `OUT` 经 R41 | PA4 / ADC1_IN4 | 原软件映射正确 |
| 右电流 | U16 `OUT` 经 R40 | PA5 / ADC1_IN5 | 原软件映射正确 |
| 母线电压 | 24V 分压 | PA3 / ADC1_IN3 | 47k/4.7k，分压比约 11:1 |

## AT8236 保护说明

U1/U3 的 `VREF` 都采用 10k/1.8k 分压：约 0.503V。配合 10mΩ `ISEN`
电阻，理论斩波限流约为 5A；这高于 37R-520 电机约 0.7A 的堵转电流，
不能代替 MCU 的电流保护。整车联调前应在电机停转时校准 INA180 零点，
再以单轮受控加载记录峰值电流后确定软件过流阈值。

## 首轮 PID 基线

正式控制与 `APP_MODE_MOTOR_SPEED_TUNING_TEST` 共用 `motor_speed_tuning.h`：

- 周期 20ms，Kp=4.0，Ki=8.0，Kd=0，修正限幅 ±250 PWM。
- 速度滤波系数 0.20；前馈为 `120 + 10 × RPM`，启动助推 350 PWM、最长 600ms。
- 目标转速限制为额定 75RPM，低于约 100RPM 空载转速。

先架空两侧主动轮，切换到 `APP_MODE_MOTOR_SPEED_TUNING_TEST`。确认正向
12RPM 时两侧 `measured_rpm` 均为正；若有一侧为负，只翻转对应
`MOTOR_SPEED_*_FEEDBACK_SIGN`。串口 `PID_TEST` 行可直接用于比较超调、稳态
误差和两侧 PWM 差异。
