/**
  *****************************************************************************
  * @file     imu_heading.c
  * @brief    BMI270 短时相对航向估计
  *
  * 当前开发板没有磁力计，因此本模块使用陀螺仪 Y 轴积分得到相对航向角。
  * 该结果适合在一次前进、制动和后退测试中抑制偏航，不适合作为长期绝对航向。
  *****************************************************************************
  */

#include "imu_heading.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>

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
#define IMU_HEADING_FILTER_RC_S                  0.03183099f
#define IMU_HEADING_RATE_STEP_LIMIT_DPS          20.0f
#define IMU_HEADING_DEADBAND_SIGMA_MULTIPLIER     3.0f
#define IMU_HEADING_DEADBAND_MIN_DPS              0.08f
#define IMU_HEADING_DEADBAND_MAX_DPS              0.30f

static uint8_t imu_heading_spi_mode3;
static uint8_t imu_heading_read_dummy = 1U;
static struct bmi2_dev imu_heading_dev;
static int8_t imu_heading_status = BMI2_E_COM_FAIL;
static float imu_heading_gyro_zero_dps;
static float imu_heading_yaw_deg;
static float imu_heading_rate_dps;
static float imu_heading_raw_rate_dps;
static float imu_heading_filtered_rate_dps;
static float imu_heading_noise_sigma_dps;
static float imu_heading_deadband_dps;

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

  printf("chip_id probe: m0+d=0x%02X m0=0x%02X m3+d=0x%02X m3=0x%02X expected=0x%02X\r\n",
         id_mode0_dummy,
         id_mode0_no_dummy,
         id_mode3_dummy,
         id_mode3_no_dummy,
         BMI270_CHIP_ID);

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
  printf("selected spi: mode%d, read_dummy=%u\r\n",
         (imu_heading_spi_mode3 != 0U) ? 3 : 0,
         imu_heading_read_dummy);
  return BMI2_OK;
}

static int8_t imu_heading_config_sensor(void)
{
  int8_t result;
  struct bmi2_sens_config config;
  uint8_t sensor = BMI2_GYRO;

  config.type = BMI2_GYRO;
  result = bmi2_get_sensor_config(&config, 1U, &imu_heading_dev);
  if(result != BMI2_OK)
  {
    return result;
  }

  /* 航向控制采用 200Hz、正负 2000dps，并开启性能优先滤波。 */
  config.cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
  config.cfg.gyr.range = BMI2_GYR_RANGE_2000;
  config.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
  config.cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
  config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

  result = bmi2_set_sensor_config(&config, 1U, &imu_heading_dev);
  if(result != BMI2_OK)
  {
    return result;
  }

  return bmi2_sensor_enable(&sensor, 1U, &imu_heading_dev);
}

static float imu_heading_raw_to_dps(int16_t raw_value)
{
  int32_t half_scale = (int32_t)(1UL << (imu_heading_dev.resolution - 1U));
  return ((float)raw_value * (float)IMU_HEADING_GYRO_RANGE_DPS) / (float)half_scale;
}

/* 上电采集 128 个有效样本，车辆移动会直接造成后续航向持续偏移。 */
static int8_t imu_heading_calibrate(void)
{
  int8_t result = BMI2_E_COM_FAIL;
  struct bmi2_sens_data sensor_data;
  uint16_t valid_count = 0U;
  uint16_t attempt_count = 0U;
  float gyro_sum = 0.0f;
  float gyro_square_sum = 0.0f;
  float gyro_sample_dps;
  float noise_variance;

  printf("BMI270 heading calibration, keep vehicle still...\r\n");
  while((valid_count < IMU_HEADING_CALIB_SAMPLE_COUNT) &&
        (attempt_count < IMU_HEADING_CALIB_MAX_ATTEMPTS))
  {
    delay_ms(IMU_HEADING_CALIB_DELAY_MS);
    attempt_count++;
    result = bmi2_get_sensor_data(&sensor_data, &imu_heading_dev);
    if((result == BMI2_OK) && ((sensor_data.status & BMI2_DRDY_GYR) != 0U))
    {
      gyro_sample_dps = imu_heading_raw_to_dps(sensor_data.gyr.y);
      gyro_sum += gyro_sample_dps;
      gyro_square_sum += gyro_sample_dps * gyro_sample_dps;
      valid_count++;
    }
  }

  if(valid_count != IMU_HEADING_CALIB_SAMPLE_COUNT)
  {
    printf("BMI270 heading calibration failed samples=%u\r\n", valid_count);
    return (result == BMI2_OK) ? BMI2_E_COM_FAIL : result;
  }

  imu_heading_gyro_zero_dps = gyro_sum / (float)valid_count;
  noise_variance = (gyro_square_sum / (float)valid_count) -
                   (imu_heading_gyro_zero_dps *
                    imu_heading_gyro_zero_dps);
  if(noise_variance < 0.0f)
  {
    noise_variance = 0.0f;
  }
  imu_heading_noise_sigma_dps = sqrtf(noise_variance);
  imu_heading_deadband_dps = imu_heading_limit(
      imu_heading_noise_sigma_dps *
      IMU_HEADING_DEADBAND_SIGMA_MULTIPLIER,
      IMU_HEADING_DEADBAND_MIN_DPS,
      IMU_HEADING_DEADBAND_MAX_DPS);
  imu_heading_yaw_deg = 0.0f;
  imu_heading_rate_dps = 0.0f;
  imu_heading_raw_rate_dps = 0.0f;
  imu_heading_filtered_rate_dps = 0.0f;
  printf("heading gyro_y_zero=%.3f sigma=%.3f deadband=%.3f dps samples=%u\r\n",
         imu_heading_gyro_zero_dps,
         imu_heading_noise_sigma_dps,
         imu_heading_deadband_dps,
         valid_count);
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
  printf("bmi270_init rslt=%d chip_id=0x%02X load_status=0x%02X\r\n",
         result,
         imu_heading_dev.chip_id,
         imu_heading_dev.load_status);
  if(result == BMI2_OK)
  {
    result = imu_heading_config_sensor();
    printf("bmi270_heading_config rslt=%d\r\n", result);
  }
  if(result == BMI2_OK)
  {
    delay_ms(100);
    result = imu_heading_calibrate();
  }

  imu_heading_status = result;
  return result;
}

int8_t imu_heading_update(float dt_s)
{
  int8_t result;
  struct bmi2_sens_data sensor_data;
  float rate_delta_dps;
  float limited_rate_dps;
  float filter_alpha;

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
    imu_heading_raw_rate_dps =
        imu_heading_raw_to_dps(sensor_data.gyr.y) -
        imu_heading_gyro_zero_dps;

    /* 先限制单周期突变，再按实际采样周期执行5Hz一阶低通。 */
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

    imu_heading_yaw_deg += imu_heading_rate_dps * dt_s;
    while(imu_heading_yaw_deg > 180.0f)
    {
      imu_heading_yaw_deg -= 360.0f;
    }
    while(imu_heading_yaw_deg < -180.0f)
    {
      imu_heading_yaw_deg += 360.0f;
    }
  }

  return BMI2_OK;
}

void imu_heading_reset(void)
{
  imu_heading_yaw_deg = 0.0f;
}

float imu_heading_get_yaw_deg(void)
{
  return imu_heading_yaw_deg;
}

float imu_heading_get_rate_dps(void)
{
  return imu_heading_rate_dps;
}

float imu_heading_get_raw_rate_dps(void)
{
  return imu_heading_raw_rate_dps;
}

float imu_heading_get_filtered_rate_dps(void)
{
  return imu_heading_filtered_rate_dps;
}

float imu_heading_get_deadband_dps(void)
{
  return imu_heading_deadband_dps;
}

int8_t imu_heading_get_status(void)
{
  return imu_heading_status;
}
