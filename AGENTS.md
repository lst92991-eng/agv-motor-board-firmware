# 回复格式（强制）

每次回复必须使用以下结构：

1. 做了什么
2. 有什么影响
3. 待确认的事
4. 下一步建议做什么

# 项目定位

本仓库是AGV电机驱动板的正式固件，目标MCU为AT32F456RCT7，外部晶振12MHz，
系统时钟192MHz，FreeRTOS静态任务。生产入口固定为`src/main.c`，不允许通过修改宏
切换成临时测试固件。

电机板负责实时且失联安全的底盘能力：

- AT8236双路PWM、电机方向和主动制动；
- 左右AB编码器、速度PI闭环和里程计；
- BMI270相对航向和Z轴角速度；
- 唯一短TTL车体速度参考的差速执行；
- 四路防撞条本地锁存停车；
- CAN FD控制租约、命令状态机、总线故障停车；
- 调试串口和CAN FD诊断遥测。

Linux/RK3576 Supervisor负责非实时决策，只能通过正式CAN FD运动接口下发任务，
不得绕开电机板本地防撞、速度上限、任务超时、磁条丢失和控制租约。

# 固定硬件事实

- 左电机编码器：PA6/PA7；右电机编码器：PC4/PC5；外部10kΩ上拉到5V，
  MCU内部上下拉关闭。
- TMR8四路PWM：PC6~PC9，20kHz。
- BMI270：PC10/PC11/PC12/PA15，车辆坐标X向前、Y向左、Z向上。
- CAN3：PB3/PB4，MUX11，仲裁500kbit/s、数据4Mbit/s、BRS。
- 防撞条：PC15前、PC13后、PC14左、PB9右。
- 双VL53L0X：PB13/PB14软件I2C，左/右XSHUT为PB11/PA11。
- IRM-H238T红外接收：PA8，38kHz，输出为单路数字电平。
- PN532硬件预留：USART5 PB5/PB6，3.3V HSU；当前无生产驱动。

# 正式运行结构

- `ChassisCtrl`：优先级5，10ms；唯一允许修改底盘运动状态。
- `SerialComm`：优先级3，2ms；调试串口协议入口。
- `CANFD`：优先级3，2ms；CAN帧收发和200ms控制租约。
- `PowerMonitor`：优先级2，20ms；电流/母线监督和本地故障提交。
- `DockSensors`：优先级2，阻塞式双TOF采样和`0x582`对接遥测。
- `Watchdog`：优先级1，20ms；Control/CAN/Power均有进展才喂狗。
- `MotionCmdQ`：8项静态队列；ESTOP、通信超时、bus-off、STOP走优先安全通道。
- ISR只采样和置位，不执行阻塞通信、日志或运动控制。

# 生产与预留能力边界

历史实车已验收电机、编码器、速度环、IMU、四方向、磁路线、CAN FD、防撞和失联
停车；本次统一参考重构目前只通过软件构建/主机门，仍需重新烧录回归。

正式控制边界：

- `chassis_motion_controller.*`只负责0x401执行、本地安全、里程计和遥测；
- 磁质量与偏移控制只属于Supervisor producer，电机板不得恢复第二算法；
- `canfd_protocol.*`负责CAN硬件、命令会话和租约；
- `canfd_status_codec.*`负责可主机测试的状态PDO字节布局。

已经接入正式低优先级采集，但尚未完成机械标定和整车验收：

- `dock_tof_sensor.*`：双TOF初始化、滤波、左右几何误差和`0x582`遥测；
- `dock_ir_receiver.*`：PA8下降沿、100ms活动窗口和`0x581/0x582`遥测。

已接入但仍保持失败关闭：`power_supervisor.*`、`adc_monitor.*`和
`motor_current.*`。未经现场标定时`limits_approved=0`，禁止ENABLE。

PN532/RFID当前不在生产树；以后只能产生带时间戳/序号的事件交给Supervisor，禁止
直接ENABLE、STOP、设轮速或PWM。充电对准、TOF闭环和电流环仍需单独实车验收。

# 安全红线

- 电机目标转速不超过75RPM；修改标定参数必须重新做架空、四方向和循迹回归。
- 防撞、ESTOP、CAN bus-off、租约/Reference超时、IMU或电气故障必须停车。
- 单个红外电平没有方向和距离信息，禁止直接触发车辆运动。
- 双TOF初始化和单次测距会阻塞，只能由未来低优先级传感器任务独占调用，禁止放入
  10ms底盘任务或中断。
- BMS为独立最终固件；电机板和Supervisor不得绕过其放电禁止状态。

# 代码与构建规则

- `src/`只放正式模块，不再放`*_test.c`、临时运动脚本或按宏切换的测试入口。
- 纯算法测试放`tests/`；硬件制造诊断如确需恢复，建立独立Keil Target，不得混入
  生产`main.c`。
- 保持驱动、协议、参考执行器分层；CAN/UART管理命令统一转成`motion_command_t`。
- 非安全运动只能来自0x401；维护串口只允许STOP/ESTOP。
- 协议字段、寄存器、时间、距离、速度和角度使用固定宽度类型并明确单位。
- 禁止提交`objects/`、`listings/`、`.bin/.hex/.map/.log`、缓存或SDK副本。
- Keil项目：`mdk_v5/agv_motor.uvprojx`，ARMCC 5.06 update 7，要求0错误0警告。
- 标准构建入口：`tools/build_motor_firmware.ps1`；该脚本只构建和校验，不烧录。
- 主机算法回归入口：`tools/run_host_tests.ps1`。

# 当前验证基线

- 四方向落地运动：通过。
- 四路防撞本地锁存：通过。
- CAN FD 500k/4M BRS与失联停车：通过。
- 2400mm直线、圆滑L型低速和L型往返：通过；完整回充模拟流程已按250mm/s
  直线峰值、75mm/s弯道完成实车验收。330mm/s不作为当前L型弯道验收速度。
- 38kHz红外接收和上位机有限角度峰值扫描：实车通过；正式Supervisor任务仍待接入。
- 双TOF基础测距历史测试通过，但当前安装和120度V形靶标必须重新标定。

详细能力边界见`hardware/motor_board_capability_baseline_2026-08-19.md`。
