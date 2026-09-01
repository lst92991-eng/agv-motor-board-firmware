/**
 * @file imu_bmi270.c
 * @brief SPI3传输、BMI270初始化、静止零偏和Z轴角速度。
 */
#include "imu_bmi270.h"

#include "at32f45x.h"
#include "at32f45x_board.h"
#include "at32f45x_crm.h"
#include "at32f45x_gpio.h"
#include "at32f45x_spi.h"
#include "bmi270.h"

#define IMU_SPI_TIMEOUT 10000UL
#define IMU_BIAS_SAMPLES 64U

static struct bmi2_dev imu_device;
static float imu_bias_dps;
static float imu_rate_dps;
static uint16_t imu_age_ms;
static uint8_t imu_ok;

static void imu_cs(uint8_t high)
{
  gpio_bits_write(GPIOA, GPIO_PINS_15, (high != 0U) ? TRUE : FALSE);
}

static uint8_t imu_transfer(uint8_t tx, uint8_t *rx)
{
  uint32_t timeout = IMU_SPI_TIMEOUT;
  while((spi_i2s_flag_get(SPI3, SPI_I2S_TDBE_FLAG) == RESET) && (timeout-- > 0U)) {}
  if(timeout == 0U) return 0U;
  spi_i2s_data_transmit(SPI3, tx);
  timeout = IMU_SPI_TIMEOUT;
  while((spi_i2s_flag_get(SPI3, SPI_I2S_RDBF_FLAG) == RESET) && (timeout-- > 0U)) {}
  if(timeout == 0U) return 0U;
  *rx = (uint8_t)spi_i2s_data_receive(SPI3);
  return 1U;
}

static BMI2_INTF_RETURN_TYPE imu_read(uint8_t reg, uint8_t *data,
                                      uint32_t length, void *context)
{
  uint32_t index;
  uint8_t value;
  UNUSED(context);
  imu_cs(0U);
  if(imu_transfer(reg, &value) == 0U) { imu_cs(1U); return BMI2_E_COM_FAIL; }
  for(index = 0U; index < length; index++)
  {
    if(imu_transfer(0U, &data[index]) == 0U) { imu_cs(1U); return BMI2_E_COM_FAIL; }
  }
  imu_cs(1U);
  return BMI2_INTF_RET_SUCCESS;
}

static BMI2_INTF_RETURN_TYPE imu_write(uint8_t reg, const uint8_t *data,
                                       uint32_t length, void *context)
{
  uint32_t index;
  uint8_t value;
  UNUSED(context);
  imu_cs(0U);
  if(imu_transfer(reg, &value) == 0U) { imu_cs(1U); return BMI2_E_COM_FAIL; }
  for(index = 0U; index < length; index++)
  {
    if(imu_transfer(data[index], &value) == 0U) { imu_cs(1U); return BMI2_E_COM_FAIL; }
  }
  imu_cs(1U);
  return BMI2_INTF_RET_SUCCESS;
}

static void imu_delay(uint32_t period_us, void *context)
{
  UNUSED(context);
  if(period_us >= 1000U) delay_ms((uint16_t)((period_us + 999U) / 1000U));
  else if(period_us != 0U) delay_us(period_us);
}

static void imu_spi_init(void)
{
  gpio_init_type gpio;
  spi_init_type spi;
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_SPI3_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio);
  gpio.gpio_pins = GPIO_PINS_10 | GPIO_PINS_11 | GPIO_PINS_12;
  gpio.gpio_mode = GPIO_MODE_MUX;
  gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio.gpio_pull = GPIO_PULL_NONE;
  gpio.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOC, &gpio);
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE10, GPIO_MUX_6);
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE11, GPIO_MUX_6);
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE12, GPIO_MUX_6);
  gpio.gpio_pins = GPIO_PINS_15;
  gpio.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init(GPIOA, &gpio);
  imu_cs(1U);
  spi_default_para_init(&spi);
  spi.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
  spi.master_slave_mode = SPI_MODE_MASTER;
  spi.mclk_freq_division = SPI_MCLK_DIV_32;
  spi.first_bit_transmission = SPI_FIRST_BIT_MSB;
  spi.frame_bit_num = SPI_FRAME_8BIT;
  spi.clock_polarity = SPI_CLOCK_POLARITY_LOW;
  spi.clock_phase = SPI_CLOCK_PHASE_1EDGE;
  spi.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
  spi_init(SPI3, &spi);
  spi_software_cs_internal_level_set(SPI3, SPI_SWCS_INTERNAL_LEVEL_HIGHT);
  spi_enable(SPI3, TRUE);
}

static uint8_t imu_configure(void)
{
  struct bmi2_sens_config config[2];
  uint8_t sensors[2] = {BMI2_ACCEL, BMI2_GYRO};
  if(bmi2_get_sensor_config(config, 2U, &imu_device) != BMI2_OK) return 0U;
  config[0].type = BMI2_ACCEL;
  config[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
  config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
  config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
  config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
  config[1].type = BMI2_GYRO;
  config[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
  config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
  config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
  config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
  if(bmi2_set_sensor_config(config, 2U, &imu_device) != BMI2_OK) return 0U;
  return (bmi2_sensor_enable(sensors, 2U, &imu_device) == BMI2_OK) ? 1U : 0U;
}

static uint8_t imu_calibrate(void)
{
  struct bmi2_sens_data data;
  uint16_t count = 0U;
  uint16_t tries = 0U;
  float sum = 0.0f;
  while((count < IMU_BIAS_SAMPLES) && (tries++ < 200U))
  {
    delay_ms(5U);
    if((bmi2_get_sensor_data(&data, &imu_device) == BMI2_OK) &&
       ((data.status & BMI2_DRDY_GYR) != 0U))
    {
      sum += (float)data.gyr.z * 2000.0f / 32768.0f;
      count++;
    }
  }
  if(count != IMU_BIAS_SAMPLES) return 0U;
  imu_bias_dps = sum / (float)count;
  imu_rate_dps = 0.0f;
  return 1U;
}

uint8_t imu_bmi270_init(void)
{
  imu_spi_init();
  imu_device.intf = BMI2_SPI_INTF;
  imu_device.read = imu_read;
  imu_device.write = imu_write;
  imu_device.delay_us = imu_delay;
  imu_device.read_write_len = 32U;
  imu_device.dummy_byte = 1U;
  imu_ok = ((bmi270_init(&imu_device) == BMI2_OK) &&
            (imu_configure() != 0U) && (imu_calibrate() != 0U)) ? 1U : 0U;
  imu_age_ms = 0U;
  return imu_ok;
}

uint8_t imu_bmi270_update(float dt_s)
{
  struct bmi2_sens_data data;
  uint16_t elapsed = (uint16_t)(dt_s * 1000.0f + 0.5f);
  float raw;
  if(imu_ok == 0U) return 0U;
  if(bmi2_get_sensor_data(&data, &imu_device) != BMI2_OK)
  {
    imu_ok = 0U;
    return 0U;
  }
  if((data.status & BMI2_DRDY_GYR) != 0U)
  {
    raw = (float)data.gyr.z * 2000.0f / 32768.0f - imu_bias_dps;
    imu_rate_dps += 0.2f * (raw - imu_rate_dps);
    imu_age_ms = 0U;
  }
  else if((uint32_t)imu_age_ms + elapsed < 65535UL) imu_age_ms += elapsed;
  else imu_age_ms = 65535U;
  return imu_bmi270_fresh();
}

float imu_bmi270_gyro_z_dps(void) { return imu_rate_dps; }
uint8_t imu_bmi270_fresh(void) { return (imu_ok != 0U) && (imu_age_ms < 50U); }
