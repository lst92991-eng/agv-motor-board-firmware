/**
  **************************************************************************
  * @file     bmi270_test.c
  * @brief    BMI270 测试模块
  *
  * 本文件完成三件事：
  * 1. 使用 GPIO 软件模拟 SPI，与 BMI270 通信；
  * 2. 调用 Bosch 官方 Sensor API 初始化 BMI270，并读取加速度计和陀螺仪数据；
  * 3. 根据加速度计和陀螺仪数据计算 roll、pitch、yaw，并用一行 printf 输出给 VOFA。
  *
  * 当前串口输出格式为：
  * roll_deg,pitch_deg,yaw_deg,side_state,pitch_state
  *
 * 注意：
 * 1. 输出单位是角度，适合直接观察车辆倾斜和翻转；
 * 2. 上电初始化完成后会做零偏校准，请保持车辆静止不动；
 * 3. yaw 目前只靠陀螺仪积分得到，没有磁力计或融合算法修正，所以长时间运行仍会慢慢漂移；
 * 4. side_state 和 pitch_state 是车辆姿态判断结果，用数字状态输出，避免影响 VOFA 解析；
 * 5. 本工程源码按 GBK 编码保存，后续中文注释也保持 GBK，避免 Keil/IDE 显示乱码。
  **************************************************************************
  */

#include "at32f45x_board.h"
#include "bmi270_test.h"
#include "bmi270.h"
#include "math.h"

/*
 * BMI270 软件 SPI 引脚定义。
 * 根据当前开发板连线：
 * PC10 -> SCK
 * PC11 -> MISO
 * PC12 -> MOSI
 * PA15 -> CS
 */
#define BMI270_SCK_PORT                  GPIOC
#define BMI270_SCK_PIN                   GPIO_PINS_10
#define BMI270_MISO_PORT                 GPIOC
#define BMI270_MISO_PIN                  GPIO_PINS_11
#define BMI270_MOSI_PORT                 GPIOC
#define BMI270_MOSI_PIN                  GPIO_PINS_12
#define BMI270_CS_PORT                   GPIOA
#define BMI270_CS_PIN                    GPIO_PINS_15

/*
 * 采样周期与换算参数。
 * BMI270_SPI_DELAY_US 控制软件 SPI 每个边沿之间的延时，数值越大通信越慢但越稳。
 * BMI270_SAMPLE_INTERVAL_MS 是主循环输出周期，50ms 对应约 20Hz 波形刷新。
 * BMI270_PI 用于角度和弧度之间的换算。
 */
#define BMI270_SPI_DELAY_US              1U
#define BMI270_SAMPLE_INTERVAL_MS        50U
#define BMI270_PI                        3.14159265358979323846f

/*
 * 上电零偏校准参数。
 * 校准期间需要让车保持静止，当前姿态会被当作 roll=0、pitch=0。
 * 陀螺仪 Y 轴平均值会作为 yaw 积分的零偏，减少静止时 yaw 慢慢漂移。
 */
#define BMI270_ZERO_CALIB_SAMPLE_COUNT   64U
#define BMI270_ZERO_CALIB_DELAY_MS       10U
#define BMI270_ZERO_CALIB_MAX_ATTEMPTS   400U

/*
 * 车辆姿态判断阈值，单位都是度。
 * 小车调试初期先用保守阈值，后面可以根据实际车架高度、轮距和重心再细调。
 */
#define VEHICLE_SIDE_TILT_WARN_DEG       10.0f
#define VEHICLE_SIDE_TILT_DANGER_DEG     35.0f
#define VEHICLE_PITCH_WARN_DEG           10.0f
#define VEHICLE_PITCH_DANGER_DEG         25.0f

/*
 * BMI270 运行状态。
 * bmi270_spi_mode3 用来记录当前使用 SPI mode0 还是 mode3。
 * bmi270_read_dummy 用来记录读取时是否需要 dummy 字节。
 * bmi270_dev 是 Bosch 官方 Sensor API 使用的设备对象，里面保存接口函数和芯片状态。
 */
static uint8_t bmi270_spi_mode3 = 0U;
static uint8_t bmi270_read_dummy = 1U;
static struct bmi2_dev bmi270_dev;
static int8_t bmi270_test_rslt = BMI2_E_COM_FAIL;
static float bmi270_roll_zero_deg = 0.0f;
static float bmi270_pitch_zero_deg = 0.0f;
static float bmi270_gyro_y_zero_mdps = 0.0f;
static float bmi270_yaw_deg = 0.0f;

/*
 * 本文件内部使用的函数声明。
 * 这些函数不对外暴露，所以全部使用 static，作用域只限制在 bmi270_test.c 内。
 */
static void bmi270_spi_gpio_init(void);
static uint8_t bmi270_spi_transfer(uint8_t tx_data);
static uint8_t bmi270_reg_read_with_dummy(uint8_t reg, uint8_t dummy_en);
static void bmi270_burst_read_raw(uint8_t reg, uint8_t *data, uint32_t len);
static void bmi270_burst_write_raw(uint8_t reg, const uint8_t *data, uint32_t len);
static BMI2_INTF_RETURN_TYPE bmi270_api_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
static BMI2_INTF_RETURN_TYPE bmi270_api_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
static void bmi270_api_delay_us(uint32_t period, void *intf_ptr);
static int8_t bmi270_config_accel_gyro(struct bmi2_dev *dev);
static int32_t gyro_lsb_to_mdps(int16_t val, int32_t dps, uint8_t bit_width);
static void bmi270_acc_to_euler_deg(const struct bmi2_sens_data *sensor_data, float *roll_deg, float *pitch_deg);
static int8_t bmi270_zero_calibrate(void);
static int8_t vehicle_get_side_state(float roll_deg);
static int8_t vehicle_get_pitch_state(float pitch_deg);
static int8_t bmi270_sensor_init(void);
static void bmi270_print_euler_data(void);

/*
 * CS 拉高。
 * BMI270 的 SPI 片选为低有效，拉高表示当前一次读写访问结束。
 */
static void bmi270_cs_high(void)
{
  gpio_bits_set(BMI270_CS_PORT, BMI270_CS_PIN);
}

/*
 * CS 拉低。
 * 拉低后 BMI270 会开始接收 SPI 命令字节，后续才能读写寄存器。
 */
static void bmi270_cs_low(void)
{
  gpio_bits_reset(BMI270_CS_PORT, BMI270_CS_PIN);
}

/* SCK 拉高，软件 SPI 通过控制这个引脚模拟时钟上升沿 */
static void bmi270_sck_high(void)
{
  gpio_bits_set(BMI270_SCK_PORT, BMI270_SCK_PIN);
}

/* SCK 拉低，软件 SPI 通过控制这个引脚模拟时钟下降沿 */
static void bmi270_sck_low(void)
{
  gpio_bits_reset(BMI270_SCK_PORT, BMI270_SCK_PIN);
}

/*
 * 设置 MOSI 电平。
 * bit_state 为 0 时输出低电平，为非 0 时输出高电平。
 */
static void bmi270_mosi_write(uint8_t bit_state)
{
  gpio_bits_write(BMI270_MOSI_PORT, BMI270_MOSI_PIN, bit_state ? TRUE : FALSE);
}

/*
 * 读取 MISO 电平。
 * 返回值只保留 0 或 1，便于后面按位拼成一个字节。
 */
static uint8_t bmi270_miso_read(void)
{
  return (gpio_input_data_bit_read(BMI270_MISO_PORT, BMI270_MISO_PIN) != RESET) ? 1U : 0U;
}

/*
 * 初始化软件模拟 SPI 所需的 GPIO。
 * CS、SCK、MOSI 配置为推挽输出，MISO 配置为输入。
 * 最后根据当前探测到的 SPI 模式，设置 SCK 的空闲电平。
 */
static void bmi270_spi_gpio_init(void)
{
  gpio_init_type gpio_init_struct;

  /* 先打开 GPIOA 和 GPIOC 的外设时钟，否则后面的 GPIO 配置不会生效。 */
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);

  /* 公共输出参数：强驱动、推挽输出、无上下拉。 */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

  /* CS 默认拉高，保证 BMI270 在初始化期间不被误选中。 */
  gpio_init_struct.gpio_pins = BMI270_CS_PIN;
  gpio_init(BMI270_CS_PORT, &gpio_init_struct);
  bmi270_cs_high();

  /* SCK 和 MOSI 都是主机输出引脚。 */
  gpio_init_struct.gpio_pins = BMI270_SCK_PIN | BMI270_MOSI_PIN;
  gpio_init(BMI270_SCK_PORT, &gpio_init_struct);
  bmi270_mosi_write(0U);

  /* MISO 是 BMI270 输出、单片机输入。 */
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = BMI270_MISO_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(BMI270_MISO_PORT, &gpio_init_struct);

  /* mode3 空闲时 SCK 为高，mode0 空闲时 SCK 为低。 */
  if(bmi270_spi_mode3 != 0U)
  {
    bmi270_sck_high();
  }
  else
  {
    bmi270_sck_low();
  }
}

/*
 * 软件模拟 SPI 收发一个字节。
 * tx_data 是主机发给 BMI270 的一个字节。
 * 返回值是同一时刻从 MISO 读回的一个字节。
 *
 * 这里按最高位先发送，每次循环处理一个 bit。
 * mode0：先拉高 SCK 后采样，再拉低 SCK；
 * mode3：先拉低 SCK 后拉高 SCK，中间采样。
 */
static uint8_t bmi270_spi_transfer(uint8_t tx_data)
{
  uint8_t rx_data = 0U;
  uint8_t bit_mask;

  /* 从 bit7 到 bit0 依次发送，BMI270 SPI 默认是高位在前。 */
  for(bit_mask = 0x80U; bit_mask != 0U; bit_mask >>= 1)
  {
    /* 先把当前要发送的 bit 放到 MOSI 上。 */
    bmi270_mosi_write((tx_data & bit_mask) ? 1U : 0U);

    if(bmi270_spi_mode3 != 0U)
    {
      /* SPI mode3：空闲高电平，先产生下降沿，再回到高电平。 */
      delay_us(BMI270_SPI_DELAY_US);
      bmi270_sck_low();
      delay_us(BMI270_SPI_DELAY_US);
      bmi270_sck_high();
      delay_us(BMI270_SPI_DELAY_US);
      if(bmi270_miso_read() != 0U)
      {
        rx_data |= bit_mask;
      }
    }
    else
    {
      /* SPI mode0：空闲低电平，先产生上升沿并采样，再回到低电平。 */
      delay_us(BMI270_SPI_DELAY_US);
      bmi270_sck_high();
      delay_us(BMI270_SPI_DELAY_US);
      if(bmi270_miso_read() != 0U)
      {
        rx_data |= bit_mask;
      }
      bmi270_sck_low();
      delay_us(BMI270_SPI_DELAY_US);
    }
  }

  return rx_data;
}

/*
 * 按 BMI270 时序读取一个寄存器。
 * reg 是寄存器地址，dummy_en 表示读地址之后是否额外丢弃一个 dummy 字节。
 * 这个函数主要用于启动时探测通信方式，不直接交给 Bosch 官方 API 使用。
 */
static uint8_t bmi270_reg_read_with_dummy(uint8_t reg, uint8_t dummy_en)
{
  uint8_t value;

  /* BMI270 SPI 读寄存器需要把地址最高位置 1。 */
  bmi270_cs_low();
  (void)bmi270_spi_transfer(reg | BMI2_SPI_RD_MASK);
  if(dummy_en != 0U)
  {
    /* 某些 BMI270 SPI 读操作会在地址之后先输出一个无效字节，需要丢弃。 */
    (void)bmi270_spi_transfer(0x00U);
  }
  value = bmi270_spi_transfer(0x00U);
  bmi270_cs_high();

  return value;
}

/*
 * 连续读取寄存器数据。
 * 注意：这里 reg 使用 Bosch API 传进来的原始地址，不再手动加读标志位。
 * Bosch 官方 API 内部会处理读标志位和 dummy 字节偏移，所以这里保持“原样传输”。
 */
static void bmi270_burst_read_raw(uint8_t reg, uint8_t *data, uint32_t len)
{
  uint32_t i;

  bmi270_cs_low();
  (void)bmi270_spi_transfer(reg);
  for(i = 0U; i < len; i++)
  {
    data[i] = bmi270_spi_transfer(0x00U);
  }
  bmi270_cs_high();
}

/*
 * 连续写入寄存器数据。
 * reg 是 Bosch API 传入的寄存器地址，data 是要写入的数据缓冲区，len 是写入长度。
 */
static void bmi270_burst_write_raw(uint8_t reg, const uint8_t *data, uint32_t len)
{
  uint32_t i;

  bmi270_cs_low();
  (void)bmi270_spi_transfer(reg);
  for(i = 0U; i < len; i++)
  {
    (void)bmi270_spi_transfer(data[i]);
  }
  bmi270_cs_high();
}

/*
 * 给 Bosch 官方 Sensor API 提供的读回调函数。
 * 官方 API 只知道要读哪个寄存器、读多少字节，真正的 SPI 读写由这里接到本工程的底层函数。
 */
static BMI2_INTF_RETURN_TYPE bmi270_api_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  UNUSED(intf_ptr);
  bmi270_burst_read_raw(reg_addr, reg_data, len);
  return BMI2_INTF_RET_SUCCESS;
}

/*
 * 给 Bosch 官方 Sensor API 提供的写回调函数。
 * BMI270 初始化、配置量程、开启传感器时，官方 API 会通过这个函数写寄存器。
 */
static BMI2_INTF_RETURN_TYPE bmi270_api_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  UNUSED(intf_ptr);
  bmi270_burst_write_raw(reg_addr, reg_data, len);
  return BMI2_INTF_RET_SUCCESS;
}

/*
 * 给 Bosch 官方 Sensor API 提供的延时函数。
 * period 单位是微秒。
 * 如果延时时间比较长，就拆成 delay_ms，避免一次 delay_us 传入过大的数。
 */
static void bmi270_api_delay_us(uint32_t period, void *intf_ptr)
{
  UNUSED(intf_ptr);

  /* 大于 60ms 的部分分段延时，避免 uint16_t 微秒延时范围不够。 */
  while(period > 60000U)
  {
    delay_ms(60);
    period -= 60000U;
  }

  /* 剩余时间如果超过 1ms，就向上取整使用毫秒延时。 */
  if(period >= 1000U)
  {
    delay_ms((uint16_t)((period + 999U) / 1000U));
  }
  else if(period != 0U)
  {
    delay_us(period);
  }
}

/*
 * 配置加速度计和陀螺仪。
 * 加速度计配置为 200Hz、正负 2g，用于观察静态倾角和姿态变化。
 * 陀螺仪配置为 200Hz、正负 2000dps，用于积分得到 yaw 变化。
 */
static int8_t bmi270_config_accel_gyro(struct bmi2_dev *dev)
{
  int8_t rslt;
  struct bmi2_sens_config config[2];
  uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };

  config[0].type = BMI2_ACCEL;
  config[1].type = BMI2_GYRO;

  /* 先读取当前配置，在当前配置基础上修改需要的字段。 */
  rslt = bmi2_get_sensor_config(config, 2, dev);
  if(rslt != BMI2_OK)
  {
    return rslt;
  }

  /* 加速度计：200Hz 输出，正负 2g，普通滤波。 */
  config[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
  config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
  config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
  config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

  /* 陀螺仪：200Hz 输出，正负 2000dps，普通滤波。 */
  config[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
  config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
  config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
  config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
  config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

  rslt = bmi2_set_sensor_config(config, 2, dev);
  if(rslt != BMI2_OK)
  {
    return rslt;
  }

  /* 最后真正使能加速度计和陀螺仪。 */
  return bmi2_sensor_enable(sensor_list, 2, dev);
}

/*
 * 将陀螺仪原始值换算成 mdps。
 * mdps 表示 milli-degree per second，也就是 0.001 度每秒。
 * 公式含义：原始值 / 半量程码值 * 当前量程 dps * 1000。
 */
static int32_t gyro_lsb_to_mdps(int16_t val, int32_t dps, uint8_t bit_width)
{
  int32_t half_scale = (int32_t)(1UL << (bit_width - 1U));
  return ((int32_t)val * dps * 1000L) / half_scale;
}

/*
 * 根据加速度计原始值计算 roll 和 pitch，输出单位是度。
 * 当前安装方向下，静止时 Y 轴接近重力方向，所以用 Y/Z 计算 roll。
 */
static void bmi270_acc_to_euler_deg(const struct bmi2_sens_data *sensor_data, float *roll_deg, float *pitch_deg)
{
  float ax;
  float ay;
  float az;

  ax = (float)sensor_data->acc.x;
  ay = (float)sensor_data->acc.y;
  az = (float)sensor_data->acc.z;

  *roll_deg = atan2f(az, ay) * (180.0f / BMI270_PI);
  *pitch_deg = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * (180.0f / BMI270_PI);
}

/*
 * 上电零偏校准。
 * 初始化完成后采集一段静止数据：
 * 1. roll 平均值作为 roll 零点；
 * 2. pitch 平均值作为 pitch 零点；
 * 3. 陀螺仪 Y 轴平均值作为 yaw 积分零偏。
 *
 * 校准时请不要移动小车，否则后面姿态判断会被带偏。
 */
static int8_t bmi270_zero_calibrate(void)
{
  int8_t rslt;
  struct bmi2_sens_data sensor_data;
  uint16_t valid_count = 0U;
  uint16_t attempt_count = 0U;
  float roll_deg;
  float pitch_deg;
  float roll_sum = 0.0f;
  float pitch_sum = 0.0f;
  float gyro_y_sum = 0.0f;

  printf("BMI270 zero calibration, keep vehicle still...\r\n");

  while((valid_count < BMI270_ZERO_CALIB_SAMPLE_COUNT) && (attempt_count < BMI270_ZERO_CALIB_MAX_ATTEMPTS))
  {
    delay_ms(BMI270_ZERO_CALIB_DELAY_MS);
    attempt_count++;

    rslt = bmi2_get_sensor_data(&sensor_data, &bmi270_dev);
    if((rslt == BMI2_OK) && ((sensor_data.status & BMI2_DRDY_ACC) != 0U) && ((sensor_data.status & BMI2_DRDY_GYR) != 0U))
    {
      bmi270_acc_to_euler_deg(&sensor_data, &roll_deg, &pitch_deg);
      roll_sum += roll_deg;
      pitch_sum += pitch_deg;
      gyro_y_sum += (float)gyro_lsb_to_mdps(sensor_data.gyr.y, 2000, bmi270_dev.resolution);
      valid_count++;
    }
  }

  if(valid_count == 0U)
  {
    printf("BMI270 zero calibration failed\r\n");
    return BMI2_E_COM_FAIL;
  }

  bmi270_roll_zero_deg = roll_sum / (float)valid_count;
  bmi270_pitch_zero_deg = pitch_sum / (float)valid_count;
  bmi270_gyro_y_zero_mdps = gyro_y_sum / (float)valid_count;
  bmi270_yaw_deg = 0.0f;

  printf("zero roll=%.2f pitch=%.2f gyro_y=%.2f samples=%u\r\n",
         bmi270_roll_zero_deg,
         bmi270_pitch_zero_deg,
         bmi270_gyro_y_zero_mdps,
         (uint32_t)valid_count);

  return BMI2_OK;
}

/*
 * 根据 roll 判断车身左右侧倾状态。
 * 返回值说明：
 *  0：基本水平；
 *  1：向右轻微侧倾；
 * -1：向左轻微侧倾；
 *  2：向右明显侧倾，有侧翻风险；
 * -2：向左明显侧倾，有侧翻风险。
 *
 * 如果实际车模方向和这里相反，后续只需要把 roll_deg 的正负号反过来。
 */
static int8_t vehicle_get_side_state(float roll_deg)
{
  if(roll_deg >= VEHICLE_SIDE_TILT_DANGER_DEG)
  {
    return 2;
  }
  else if(roll_deg >= VEHICLE_SIDE_TILT_WARN_DEG)
  {
    return 1;
  }
  else if(roll_deg <= -VEHICLE_SIDE_TILT_DANGER_DEG)
  {
    return -2;
  }
  else if(roll_deg <= -VEHICLE_SIDE_TILT_WARN_DEG)
  {
    return -1;
  }

  return 0;
}

/*
 * 根据 pitch 判断车身前后俯仰状态。
 * 返回值说明：
 *  0：基本水平；
 *  1：轻微翘头；
 * -1：轻微点头；
 *  2：明显翘头；
 * -2：明显点头。
 *
 * 如果实际车模表现相反，后续只需要把 pitch_deg 的正负号反过来。
 */
static int8_t vehicle_get_pitch_state(float pitch_deg)
{
  if(pitch_deg >= VEHICLE_PITCH_DANGER_DEG)
  {
    return 2;
  }
  else if(pitch_deg >= VEHICLE_PITCH_WARN_DEG)
  {
    return 1;
  }
  else if(pitch_deg <= -VEHICLE_PITCH_DANGER_DEG)
  {
    return -2;
  }
  else if(pitch_deg <= -VEHICLE_PITCH_WARN_DEG)
  {
    return -1;
  }

  return 0;
}

/*
 * 自动探测当前 SPI 模式和是否需要 dummy 字节。
 * 之前读取 chip_id 时出现过 0x21、0x00 等错误值，所以这里同时尝试：
 * 1. mode0 + dummy；
 * 2. mode0 + 无 dummy；
 * 3. mode3 + dummy；
 * 4. mode3 + 无 dummy。
 * 哪一种能读到 BMI270 正确 chip_id 0x24，就使用哪一种配置。
 */
static int8_t bmi270_probe_and_select_spi(void)
{
  uint8_t id_mode0_dummy;
  uint8_t id_mode0_nodummy;
  uint8_t id_mode3_dummy;
  uint8_t id_mode3_nodummy;

  /* 先尝试 SPI mode0，并分别测试带 dummy 和不带 dummy 的读法。 */
  bmi270_spi_mode3 = 0U;
  bmi270_read_dummy = 1U;
  bmi270_spi_gpio_init();
  delay_ms(10);
  (void)bmi270_reg_read_with_dummy(BMI2_CHIP_ID_ADDR, 1U);
  delay_ms(2);
  id_mode0_dummy = bmi270_reg_read_with_dummy(BMI2_CHIP_ID_ADDR, 1U);
  id_mode0_nodummy = bmi270_reg_read_with_dummy(BMI2_CHIP_ID_ADDR, 0U);

  /* 再尝试 SPI mode3，同样分别测试带 dummy 和不带 dummy 的读法。 */
  bmi270_spi_mode3 = 1U;
  bmi270_read_dummy = 1U;
  bmi270_spi_gpio_init();
  delay_ms(2);
  (void)bmi270_reg_read_with_dummy(BMI2_CHIP_ID_ADDR, 1U);
  delay_ms(2);
  id_mode3_dummy = bmi270_reg_read_with_dummy(BMI2_CHIP_ID_ADDR, 1U);
  id_mode3_nodummy = bmi270_reg_read_with_dummy(BMI2_CHIP_ID_ADDR, 0U);

  printf("chip_id probe: m0+d=0x%02X m0=0x%02X m3+d=0x%02X m3=0x%02X expected=0x%02X\r\n",
         id_mode0_dummy, id_mode0_nodummy, id_mode3_dummy, id_mode3_nodummy, BMI270_CHIP_ID);

  /* 根据探测结果选择后续正式通信使用的 SPI 模式。 */
  if(id_mode0_dummy == BMI270_CHIP_ID)
  {
    bmi270_spi_mode3 = 0U;
    bmi270_read_dummy = 1U;
  }
  else if(id_mode3_dummy == BMI270_CHIP_ID)
  {
    bmi270_spi_mode3 = 1U;
    bmi270_read_dummy = 1U;
  }
  else if(id_mode0_nodummy == BMI270_CHIP_ID)
  {
    bmi270_spi_mode3 = 0U;
    bmi270_read_dummy = 0U;
  }
  else if(id_mode3_nodummy == BMI270_CHIP_ID)
  {
    bmi270_spi_mode3 = 1U;
    bmi270_read_dummy = 0U;
  }
  else
  {
    return BMI2_E_COM_FAIL;
  }

  /* 按最终选择重新初始化 GPIO，保证 SCK 空闲电平正确。 */
  bmi270_spi_gpio_init();
  printf("selected spi: mode%d, read_dummy=%u\r\n", (bmi270_spi_mode3 != 0U) ? 3 : 0, bmi270_read_dummy);

  return BMI2_OK;
}

/*
 * 初始化 BMI270。
 * 流程：
 * 1. 先探测 SPI 模式，确保能读到 chip_id；
 * 2. 填充 bmi270_dev，把读、写、延时函数交给 Bosch 官方 API；
 * 3. 调用 bmi270_init 上传配置文件并初始化芯片；
 * 4. 配置并使能加速度计和陀螺仪。
 */
static int8_t bmi270_sensor_init(void)
{
  int8_t rslt;

  /* 如果连 chip_id 都读不到，直接返回错误，主循环会延时后重试。 */
  rslt = bmi270_probe_and_select_spi();
  if(rslt != BMI2_OK)
  {
    return rslt;
  }

  /* 填充 Bosch 官方 API 需要的设备结构体。 */
  bmi270_dev.intf = BMI2_SPI_INTF;
  bmi270_dev.read = bmi270_api_read;
  bmi270_dev.write = bmi270_api_write;
  bmi270_dev.delay_us = bmi270_api_delay_us;
  bmi270_dev.intf_ptr = NULL;
  bmi270_dev.read_write_len = 32U;

  /* 初始化 BMI270，内部会读取芯片 ID、上传配置文件、检查加载状态。 */
  rslt = bmi270_init(&bmi270_dev);
  printf("bmi270_init rslt=%d chip_id=0x%02X load_status=0x%02X\r\n",
         rslt, bmi270_dev.chip_id, bmi270_dev.load_status);
  if(rslt != BMI2_OK)
  {
    return rslt;
  }

  /* 配置加速度计和陀螺仪参数，然后等待数据稳定。 */
  rslt = bmi270_config_accel_gyro(&bmi270_dev);
  printf("bmi270_config_accel_gyro rslt=%d\r\n", rslt);
  if(rslt != BMI2_OK)
  {
    return rslt;
  }

  delay_ms(100);

  return bmi270_zero_calibrate();
}

/*
 * 读取传感器数据并输出成 VOFA 可直接解析的三路欧拉角。
 * 输出顺序：
 * roll,pitch,yaw,side_state,pitch_state
 *
 * roll 和 pitch 主要由加速度计计算，静态或低速运动时比较直观；
 * yaw 由陀螺仪 Y 轴积分得到，只能观察短时间转动趋势，不能当绝对航向角。
 * side_state 用于判断车辆是否左右倾斜；
 * pitch_state 用于判断车辆是否翘头或点头。
 *
 * 当前直接使用 %.2f 输出两位小数。
 * 如果后续发现串口输出变成空白或异常，需要检查工程是否启用了浮点 printf 支持。
 */
static void bmi270_print_euler_data(void)
{
  int8_t rslt;
  struct bmi2_sens_data sensor_data;
  float raw_roll_deg;
  float raw_pitch_deg;
  float roll_deg;
  float pitch_deg;
  int32_t gy_mdps;
  int8_t side_state;
  int8_t pitch_state;

  /* 从 BMI270 一次性读取加速度计和陀螺仪数据。 */
  rslt = bmi2_get_sensor_data(&sensor_data, &bmi270_dev);
  if((rslt == BMI2_OK) && ((sensor_data.status & BMI2_DRDY_ACC) != 0U) && ((sensor_data.status & BMI2_DRDY_GYR) != 0U))
  {
    /* 当前静态摆放下，Y 轴近似朝上，所以这里用 Y/Z 计算横滚角。 */
    gy_mdps = gyro_lsb_to_mdps(sensor_data.gyr.y, 2000, bmi270_dev.resolution);

    /*
     * roll_deg：绕车体前后方向翻滚的角度，单位是度。
     * pitch_deg：绕车体左右方向俯仰的角度，单位是度。
     * 这里的轴向映射和你的传感器安装方向有关，后期如果模型方向反了，只需要调整这里的符号或轴。
     */
    bmi270_acc_to_euler_deg(&sensor_data, &raw_roll_deg, &raw_pitch_deg);
    roll_deg = raw_roll_deg - bmi270_roll_zero_deg;
    pitch_deg = raw_pitch_deg - bmi270_pitch_zero_deg;

    /* yaw 使用陀螺仪积分，并减去上电静止时测出来的陀螺仪零偏。 */
    bmi270_yaw_deg += (((float)gy_mdps - bmi270_gyro_y_zero_mdps) * (float)BMI270_SAMPLE_INTERVAL_MS) / 1000000.0f;
    if(bmi270_yaw_deg > 180.0f)
    {
      bmi270_yaw_deg -= 360.0f;
    }
    else if(bmi270_yaw_deg < -180.0f)
    {
      bmi270_yaw_deg += 360.0f;
    }

    /* 根据当前欧拉角判断车辆姿态状态。 */
    side_state = vehicle_get_side_state(roll_deg);
    pitch_state = vehicle_get_pitch_state(pitch_deg);

    /* 一个 printf 同时输出角度和姿态状态，逗号分隔，换行结束。 */
    printf("%.2f,%.2f,%.2f,%d,%d\r\n", roll_deg, pitch_deg, bmi270_yaw_deg, side_state, pitch_state);
  }
}

/*
 * 初始化 BMI270 测试模块。
 * main.c 调用一次即可，后续由 bmi270_test_loop 持续输出或重试。
 */
void bmi270_test_init(void)
{
  printf("\r\nBMI270 official API test\r\n");
  bmi270_test_rslt = bmi270_sensor_init();
}

/*
 * BMI270 测试模块循环任务。
 * 初始化成功后按固定周期输出欧拉角；失败时每 1 秒重试一次初始化。
 */
void bmi270_test_loop(void)
{
  if(bmi270_test_rslt == BMI2_OK)
  {
    bmi270_print_euler_data();
    delay_ms(BMI270_SAMPLE_INTERVAL_MS);
  }
  else
  {
    printf("BMI270 init failed rslt=%d, retrying...\r\n", bmi270_test_rslt);
    delay_ms(1000);
    bmi270_test_rslt = bmi270_sensor_init();
  }
}
