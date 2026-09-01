/**
  *****************************************************************************
  * @file     imu_heading.c
  * @brief    BMI270 短时相对航向估计
  *
  * 当前开发板没有磁力计。模块在上电静止时用重力确定车辆竖直轴，将三轴
  * 陀螺仪投影到该轴后积分相对航向；结果仍不等价于长期绝对航向。
  *****************************************************************************
  */

#include "imu_heading.h"

#include <math.h>
#include <stddef.h>

#include "at32f45x_board.h"
#include "bmi270.h"

#define IMU_HEADING_SCK_PORT                    GPIOC
#define IMU_HEADING_SCK_PIN                     GPIO_PINS_10
#define IMU_HEADING_MISO_PORT                   GPIOC
#define IMU_HEADING_MISO_PIN                    GPIO_PINS_11
#define IMU_HEADING_MOSI_PORT                   GPIOC
#define IMU_HEADING_MOSI_PIN                    GPIO_PINS_12
#define IMU_HEADING_CS_PORT                     GPIOA
#define IMU_HEADING_CS_PIN                      GPIO_PINS_15

#define IMU_HEADING_SPI_DELAY_US                1U
#define IMU_HEADING_CALIB_SAMPLE_COUNT          128U
#define IMU_HEADING_CALIB_DELAY_MS              10U
#define IMU_HEADING_CALIB_MAX_ATTEMPTS          800U
#define IMU_HEADING_GYRO_RANGE_DPS              2000L
#define IMU_HEADING_ACCEL_RANGE_G                  2.0f
#define IMU_HEADING_MIN_GRAVITY_RATIO              0.70f
#define IMU_HEADING_MAX_GRAVITY_RATIO              1.30f
#define IMU_HEADING_FILTER_RC_S                  0.03183099f
#define IMU_HEADING_RATE_STEP_LIMIT_DPS          20.0f
#define IMU_HEADING_DEADBAND_SIGMA_MULTIPLIER     3.0f
#define IMU_HEADING_DEADBAND_MIN_DPS              0.08f
#define IMU_HEADING_DEADBAND_MAX_DPS              0.30f
#define IMU_HEADING_CALIB_MAX_SIGMA_DPS            1.00f
#define IMU_HEADING_YAW_SIGN                         1.0f
#define IMU_HEADING_GYRO_STALE_LIMIT_MS               50U

static uint8_t imu_heading_spi_mode3;
static uint8_t imu_heading_read_dummy = 1U;
static struct bmi2_dev imu_heading_dev;
static int8_t imu_heading_status = BMI2_E_COM_FAIL;
static float imu_heading_gyro_z_zero_dps;
static float imu_heading_gravity_x_raw;
static float imu_heading_gravity_y_raw;
static float imu_heading_gravity_z_raw;
static float imu_heading_accel_raw_per_g;
static int16_t imu_heading_linear_accel_x_mg;
static int16_t imu_heading_linear_accel_y_mg;
static int16_t imu_heading_linear_accel_z_mg;
static float imu_heading_rate_dps;
static float imu_heading_raw_rate_dps;
static float imu_heading_filtered_rate_dps;
static float imu_heading_noise_sigma_dps;
static float imu_heading_deadband_dps;
static uint16_t imu_heading_gyro_age_ms;

static float imu_heading_abs_float(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float imu_heading_limit(float value, float value_min, float value_max)
{
  if(value < value_min)
  {
    return value_min;
  }
  if(value > value_max)
  {
    return value_max;
  }
  return value;
}

static void imu_heading_cs_high(void)
{
  gpio_bits_set(IMU_HEADING_CS_PORT, IMU_HEADING_CS_PIN);
}

static void imu_heading_cs_low(void)
{
  gpio_bits_reset(IMU_HEADING_CS_PORT, IMU_HEADING_CS_PIN);
}

static void imu_heading_sck_high(void)
{
  gpio_bits_set(IMU_HEADING_SCK_PORT, IMU_HEADING_SCK_PIN);
}

static void imu_heading_sck_low(void)
{
  gpio_bits_reset(IMU_HEADING_SCK_PORT, IMU_HEADING_SCK_PIN);
}

static void imu_heading_mosi_write(uint8_t bit_state)
{
  gpio_bits_write(IMU_HEADING_MOSI_PORT,
                  IMU_HEADING_MOSI_PIN,
                  (bit_state != 0U) ? TRUE : FALSE);
}

static uint8_t imu_heading_miso_read(void)
{
  return (gpio_input_data_bit_read(IMU_HEADING_MISO_PORT,
                                   IMU_HEADING_MISO_PIN) != RESET) ? 1U : 0U;
}

/* 初始化软件 SPI 引脚，并根据探测结果设置时钟空闲电平。 */
static void imu_heading_spi_gpio_init(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

  gpio_init_struct.gpio_pins = IMU_HEADING_CS_PIN;
  gpio_init(IMU_HEADING_CS_PORT, &gpio_init_struct);
  imu_heading_cs_high();

  gpio_init_struct.gpio_pins = IMU_HEADING_SCK_PIN | IMU_HEADING_MOSI_PIN;
  gpio_init(IMU_HEADING_SCK_PORT, &gpio_init_struct);
  imu_heading_mosi_write(0U);

  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = IMU_HEADING_MISO_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(IMU_HEADING_MISO_PORT, &gpio_init_struct);

  if(imu_heading_spi_mode3 != 0U)
  {
    imu_heading_sck_high();
  }
  else
  {
    imu_heading_sck_low();
  }
}

/* 软件 SPI 按最高位优先收发一个字节，支持 mode0 和 mode3。 */
static uint8_t imu_heading_spi_transfer(uint8_t tx_data)
{
  uint8_t rx_data = 0U;
  uint8_t bit_mask;

  for(bit_mask = 0x80U; bit_mask != 0U; bit_mask >>= 1)
  {
    imu_heading_mosi_write(((tx_data & bit_mask) != 0U) ? 1U : 0U);

    if(imu_heading_spi_mode3 != 0U)
    {
      delay_us(IMU_HEADING_SPI_DELAY_US);
      imu_heading_sck_low();
      delay_us(IMU_HEADING_SPI_DELAY_US);
      imu_heading_sck_high();
      delay_us(IMU_HEADING_SPI_DELAY_US);
      if(imu_heading_miso_read() != 0U)
      {
        rx_data |= bit_mask;
      }
    }
    else
    {
      delay_us(IMU_HEADING_SPI_DELAY_US);
      imu_heading_sck_high();
      delay_us(IMU_HEADING_SPI_DELAY_US);
      if(imu_heading_miso_read() != 0U)
      {
        rx_data |= bit_mask;
      }
      imu_heading_sck_low();
      delay_us(IMU_HEADING_SPI_DELAY_US);
    }
  }

  return rx_data;
}

/* 启动阶段直接读取寄存器，用于探测 SPI 模式和读取空字节要求。 */
static uint8_t imu_heading_reg_read(uint8_t reg, uint8_t dummy_enable)
{
  uint8_t value;

  imu_heading_cs_low();
  (void)imu_heading_spi_transfer(reg | BMI2_SPI_RD_MASK);
  if(dummy_enable != 0U)
  {
    (void)imu_heading_spi_transfer(0x00U);
  }
  value = imu_heading_spi_transfer(0x00U);
  imu_heading_cs_high();

  return value;
}

static void imu_heading_burst_read(uint8_t reg, uint8_t *data, uint32_t len)
{
  uint32_t index;

  imu_heading_cs_low();
  (void)imu_heading_spi_transfer(reg);
  for(index = 0U; index < len; index++)
  {
    data[index] = imu_heading_spi_transfer(0x00U);
  }
  imu_heading_cs_high();
}

static void imu_heading_burst_write(uint8_t reg, const uint8_t *data, uint32_t len)
{
  uint32_t index;

  imu_heading_cs_low();
  (void)imu_heading_spi_transfer(reg);
  for(index = 0U; index < len; index++)
  {
    (void)imu_heading_spi_transfer(data[index]);
  }
  imu_heading_cs_high();
}

static BMI2_INTF_RETURN_TYPE imu_heading_api_read(uint8_t reg_addr,
                                                  uint8_t *reg_data,
                                                  uint32_t len,
                                                  void *intf_ptr)
{
  UNUSED(intf_ptr);
  imu_heading_burst_read(reg_addr, reg_data, len);
  return BMI2_INTF_RET_SUCCESS;
}

static BMI2_INTF_RETURN_TYPE imu_heading_api_write(uint8_t reg_addr,
                                                   const uint8_t *reg_data,
                                                   uint32_t len,
                                                   void *intf_ptr)
{
  UNUSED(intf_ptr);
  imu_heading_burst_write(reg_addr, reg_data, len);
  return BMI2_INTF_RET_SUCCESS;
}

static void imu_heading_api_delay_us(uint32_t period, void *intf_ptr)
{
  UNUSED(intf_ptr);

  while(period > 60000U)
  {
    delay_ms(60);
    period -= 60000U;
  }

  if(period >= 1000U)
  {
    delay_ms((uint16_t)((period + 999U) / 1000U));
  }
  else if(period != 0U)
  {
    delay_us(period);
  }
}

/* 同时尝试四种读法，避免不同上电状态下错误读到 0x21 或 0x00。 */
static int8_t imu_heading_probe_spi(void)
{
  uint8_t id_mode0_dummy;
  uint8_t id_mode0_no_dummy;
  uint8_t id_mode3_dummy;
  uint8_t id_mode3_no_dummy;

  imu_heading_spi_mode3 = 0U;
  imu_heading_spi_gpio_init();
  delay_ms(10);
  (void)imu_heading_reg_read(BMI2_CHIP_ID_ADDR, 1U);
  delay_ms(2);
  id_mode0_dummy = imu_heading_reg_read(BMI2_CHIP_ID_ADDR, 1U);
  id_mode0_no_dummy = imu_heading_reg_read(BMI2_CHIP_ID_ADDR, 0U);

  imu_heading_spi_mode3 = 1U;
  imu_heading_spi_gpio_init();
  delay_ms(2);
  (void)imu_heading_reg_read(BMI2_CHIP_ID_ADDR, 1U);
  delay_ms(2);
  id_mode3_dummy = imu_heading_reg_read(BMI2_CHIP_ID_ADDR, 1U);
  id_mode3_no_dummy = imu_heading_reg_read(BMI2_CHIP_ID_ADDR, 0U);

  if(id_mode0_dummy == BMI270_CHIP_ID)
  {
    imu_heading_spi_mode3 = 0U;
    imu_heading_read_dummy = 1U;
  }
  else if(id_mode3_dummy == BMI270_CHIP_ID)
  {
    imu_heading_spi_mode3 = 1U;
    imu_heading_read_dummy = 1U;
  }
  else if(id_mode0_no_dummy == BMI270_CHIP_ID)
  {
    imu_heading_spi_mode3 = 0U;
    imu_heading_read_dummy = 0U;
  }
  else if(id_mode3_no_dummy == BMI270_CHIP_ID)
  {
    imu_heading_spi_mode3 = 1U;
    imu_heading_read_dummy = 0U;
  }
  else
  {
    return BMI2_E_COM_FAIL;
  }

  imu_heading_spi_gpio_init();
  return BMI2_OK;
}

static int8_t imu_heading_config_sensor(void)
{
  int8_t result;
  struct bmi2_sens_config config[2];
  uint8_t sensors[2] = { BMI2_ACCEL, BMI2_GYRO };

  config[0].type = BMI2_ACCEL;
  config[1].type = BMI2_GYRO;
  result = bmi2_get_sensor_config(config, 2U, &imu_heading_dev);
  if(result != BMI2_OK)
  {
    return result;
  }

  /* 加速度计只在静止校准阶段确定车辆竖直轴，不参与 Yaw 绝对校正。 */
  config[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
  config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
  config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
  config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

  /* 三轴陀螺仪统一采用 200Hz、正负 2000dps。 */
  config[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
  config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
  config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
  config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
  config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

  result = bmi2_set_sensor_config(config, 2U, &imu_heading_dev);
  if(result != BMI2_OK)
  {
    return result;
  }

  return bmi2_sensor_enable(sensors, 2U, &imu_heading_dev);
}

static float imu_heading_raw_to_dps(int16_t raw_value)
{
  int32_t half_scale = (int32_t)(1UL << (imu_heading_dev.resolution - 1U));
  return ((float)raw_value * (float)IMU_HEADING_GYRO_RANGE_DPS) / (float)half_scale;
}

static int16_t imu_heading_linear_accel_to_mg(int16_t raw_value,
                                              float gravity_raw)
{
  float value_mg = (((float)raw_value - gravity_raw) * 1000.0f) /
                   imu_heading_accel_raw_per_g;

  value_mg = imu_heading_limit(value_mg, -2000.0f, 2000.0f);
  return (int16_t)value_mg;
}

/* 上电静止采样：估计三轴零偏，并由重力方向确定车辆竖直轴。 */
static int8_t imu_heading_calibrate(void)
{
  int8_t result = BMI2_E_COM_FAIL;
  struct bmi2_sens_data sensor_data;
  uint16_t valid_count = 0U;
  uint16_t attempt_count = 0U;
  float gyro_z_sum = 0.0f;
  float gyro_z_square_sum = 0.0f;
  float accel_x_sum = 0.0f;
  float accel_y_sum = 0.0f;
  float accel_z_sum = 0.0f;
  float gyro_z_dps;
  float accel_norm;
  float expected_one_g;
  float gravity_ratio;
  float noise_variance;

  while((valid_count < IMU_HEADING_CALIB_SAMPLE_COUNT) &&
        (attempt_count < IMU_HEADING_CALIB_MAX_ATTEMPTS))
  {
    delay_ms(IMU_HEADING_CALIB_DELAY_MS);
    attempt_count++;
    result = bmi2_get_sensor_data(&sensor_data, &imu_heading_dev);
    if((result == BMI2_OK) &&
       ((sensor_data.status & BMI2_DRDY_GYR) != 0U) &&
       ((sensor_data.status & BMI2_DRDY_ACC) != 0U))
    {
      gyro_z_dps = imu_heading_raw_to_dps(sensor_data.gyr.z);
      gyro_z_sum += gyro_z_dps;
      gyro_z_square_sum += gyro_z_dps * gyro_z_dps;
      accel_x_sum += (float)sensor_data.acc.x;
      accel_y_sum += (float)sensor_data.acc.y;
      accel_z_sum += (float)sensor_data.acc.z;
      valid_count++;
    }
  }

  if(valid_count != IMU_HEADING_CALIB_SAMPLE_COUNT)
  {
    return (result == BMI2_OK) ? BMI2_E_COM_FAIL : result;
  }

  imu_heading_gyro_z_zero_dps = gyro_z_sum / (float)valid_count;
  imu_heading_gravity_x_raw = accel_x_sum / (float)valid_count;
  imu_heading_gravity_y_raw = accel_y_sum / (float)valid_count;
  imu_heading_gravity_z_raw = accel_z_sum / (float)valid_count;
  accel_norm = sqrtf((imu_heading_gravity_x_raw * imu_heading_gravity_x_raw) +
                     (imu_heading_gravity_y_raw * imu_heading_gravity_y_raw) +
                     (imu_heading_gravity_z_raw * imu_heading_gravity_z_raw));
  expected_one_g = ((float)(1UL << (imu_heading_dev.resolution - 1U))) /
                   IMU_HEADING_ACCEL_RANGE_G;
  imu_heading_accel_raw_per_g = expected_one_g;
  gravity_ratio = accel_norm / expected_one_g;
  if((gravity_ratio < IMU_HEADING_MIN_GRAVITY_RATIO) ||
     (gravity_ratio > IMU_HEADING_MAX_GRAVITY_RATIO))
  {
    return BMI2_E_COM_FAIL;
  }
  noise_variance = gyro_z_square_sum / (float)valid_count -
      imu_heading_gyro_z_zero_dps * imu_heading_gyro_z_zero_dps;
  if(noise_variance < 0.0f)
  {
    noise_variance = 0.0f;
  }
  imu_heading_noise_sigma_dps = sqrtf(noise_variance);
  if(imu_heading_noise_sigma_dps > IMU_HEADING_CALIB_MAX_SIGMA_DPS)
  {
    return BMI2_E_COM_FAIL;
  }
  imu_heading_deadband_dps = imu_heading_limit(
      imu_heading_noise_sigma_dps *
      IMU_HEADING_DEADBAND_SIGMA_MULTIPLIER,
      IMU_HEADING_DEADBAND_MIN_DPS,
      IMU_HEADING_DEADBAND_MAX_DPS);
  imu_heading_rate_dps = 0.0f;
  imu_heading_raw_rate_dps = 0.0f;
  imu_heading_filtered_rate_dps = 0.0f;
  imu_heading_linear_accel_x_mg = 0;
  imu_heading_linear_accel_y_mg = 0;
  imu_heading_linear_accel_z_mg = 0;
  return BMI2_OK;
}

int8_t imu_heading_init(void)
{
  int8_t result;

  result = imu_heading_probe_spi();
  if(result != BMI2_OK)
  {
    imu_heading_status = result;
    return result;
  }

  imu_heading_dev.intf = BMI2_SPI_INTF;
  imu_heading_dev.read = imu_heading_api_read;
  imu_heading_dev.write = imu_heading_api_write;
  imu_heading_dev.delay_us = imu_heading_api_delay_us;
  imu_heading_dev.intf_ptr = NULL;
  imu_heading_dev.read_write_len = 32U;
  imu_heading_dev.dummy_byte = imu_heading_read_dummy;

  result = bmi270_init(&imu_heading_dev);
  if(result == BMI2_OK)
  {
    result = imu_heading_config_sensor();
  }
  if(result == BMI2_OK)
  {
    delay_ms(100);
    result = imu_heading_calibrate();
  }

  imu_heading_status = result;
  imu_heading_gyro_age_ms = 0U;
  return result;
}

int8_t imu_heading_update(float dt_s)
{
  int8_t result;
  struct bmi2_sens_data sensor_data;
  float gyro_z_dps;
  float rate_delta_dps;
  float limited_rate_dps;
  float filter_alpha;
  uint32_t elapsed_ms;

  if(imu_heading_status != BMI2_OK)
  {
    return imu_heading_status;
  }

  result = bmi2_get_sensor_data(&sensor_data, &imu_heading_dev);
  if(result != BMI2_OK)
  {
    imu_heading_status = result;
    return result;
  }

  if((sensor_data.status & BMI2_DRDY_GYR) != 0U)
  {
    imu_heading_gyro_age_ms = 0U;
    gyro_z_dps = imu_heading_raw_to_dps(sensor_data.gyr.z) -
                 imu_heading_gyro_z_zero_dps;
    /* BMI270安装坐标已冻结为Z向上，左转为正。 */
    imu_heading_raw_rate_dps = IMU_HEADING_YAW_SIGN * gyro_z_dps;

    /* 先限制单周期突变，再按实际采样周期执行 5Hz 一阶低通。 */
    rate_delta_dps = imu_heading_raw_rate_dps -
                     imu_heading_filtered_rate_dps;
    rate_delta_dps = imu_heading_limit(
        rate_delta_dps,
        -IMU_HEADING_RATE_STEP_LIMIT_DPS,
        IMU_HEADING_RATE_STEP_LIMIT_DPS);
    limited_rate_dps = imu_heading_filtered_rate_dps + rate_delta_dps;
    filter_alpha = dt_s / (IMU_HEADING_FILTER_RC_S + dt_s);
    filter_alpha = imu_heading_limit(filter_alpha, 0.0f, 1.0f);
    imu_heading_filtered_rate_dps += filter_alpha *
        (limited_rate_dps - imu_heading_filtered_rate_dps);

    /* 三倍静止噪声死区只作用于控制输出，低通内部状态保持连续。 */
    if(imu_heading_abs_float(imu_heading_filtered_rate_dps) <=
       imu_heading_deadband_dps)
    {
      imu_heading_rate_dps = 0.0f;
    }
    else
    {
      imu_heading_rate_dps = imu_heading_filtered_rate_dps;
    }

  }
  else
  {
    /* Never integrate or reuse a rate sample whose DRDY bit did not advance. */
    elapsed_ms = (uint32_t)((dt_s * 1000.0f) + 0.5f);
    if(elapsed_ms == 0U)
    {
      elapsed_ms = 1U;
    }
    if((uint32_t)imu_heading_gyro_age_ms + elapsed_ms > 65535UL)
    {
      imu_heading_gyro_age_ms = 65535U;
    }
    else
    {
      imu_heading_gyro_age_ms = (uint16_t)(
          (uint32_t)imu_heading_gyro_age_ms + elapsed_ms);
    }
    imu_heading_rate_dps = 0.0f;
  }

  if((sensor_data.status & BMI2_DRDY_ACC) != 0U)
  {
    imu_heading_linear_accel_x_mg = imu_heading_linear_accel_to_mg(
        sensor_data.acc.x, imu_heading_gravity_x_raw);
    imu_heading_linear_accel_y_mg = imu_heading_linear_accel_to_mg(
        sensor_data.acc.y, imu_heading_gravity_y_raw);
    imu_heading_linear_accel_z_mg = imu_heading_linear_accel_to_mg(
        sensor_data.acc.z, imu_heading_gravity_z_raw);
  }

  if(imu_heading_gyro_age_ms >= IMU_HEADING_GYRO_STALE_LIMIT_MS)
  {
    imu_heading_status = BMI2_E_DATA_RDY_INT_FAILED;
    return imu_heading_status;
  }
  return BMI2_OK;
}

float imu_heading_get_rate_dps(void)
{
  return imu_heading_rate_dps;
}

int16_t imu_heading_get_linear_accel_x_mg(void)
{
  return imu_heading_linear_accel_x_mg;
}

int16_t imu_heading_get_linear_accel_y_mg(void)
{
  return imu_heading_linear_accel_y_mg;
}

int16_t imu_heading_get_linear_accel_z_mg(void)
{
  return imu_heading_linear_accel_z_mg;
}

int8_t imu_heading_get_status(void)
{
  return imu_heading_status;
}

uint16_t imu_heading_get_gyro_age_ms(void)
{
  return imu_heading_gyro_age_ms;
}
