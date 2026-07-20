/**
  *****************************************************************************
  * @file     tof_i2c3.c
  * @brief    VL53L0X共用I2C3引脚的软件I2C诊断接口
  *
  * PB13为SCL，PB14为SDA。当前诊断版本暂时不使用AT32硬件I2C3，直接
  * 通过开漏GPIO产生约100kHz时序，用于排除GPIO复用和I2C时序寄存器
  * 配置问题。传感器板上的4.7k电阻负责把两根总线拉到3.3V。
  *****************************************************************************
  */

#include "tof_i2c3.h"

#include "at32f45x_board.h"
#include "at32f45x_conf.h"

#define TOF_SCL_PIN                       GPIO_PINS_13
#define TOF_SDA_PIN                       GPIO_PINS_14
#define TOF_SOFT_I2C_HALF_PERIOD_US       5U
#define TOF_SOFT_I2C_STRETCH_TIMEOUT      2000U

static volatile int32_t tof_i2c3_last_error;

static void tof_scl_release(void)
{
  gpio_bits_set(GPIOB, TOF_SCL_PIN);
}

static void tof_scl_low(void)
{
  gpio_bits_reset(GPIOB, TOF_SCL_PIN);
}

static void tof_sda_release(void)
{
  gpio_bits_set(GPIOB, TOF_SDA_PIN);
}

static void tof_sda_low(void)
{
  gpio_bits_reset(GPIOB, TOF_SDA_PIN);
}

static uint8_t tof_scl_read(void)
{
  return (gpio_input_data_bit_read(GPIOB, TOF_SCL_PIN) != RESET) ? 1U : 0U;
}

static uint8_t tof_sda_read(void)
{
  return (gpio_input_data_bit_read(GPIOB, TOF_SDA_PIN) != RESET) ? 1U : 0U;
}

static int32_t tof_scl_wait_high(void)
{
  uint32_t timeout = TOF_SOFT_I2C_STRETCH_TIMEOUT;

  tof_scl_release();
  while(tof_scl_read() == 0U)
  {
    if((timeout--) == 0U)
    {
      tof_i2c3_last_error = -13;
      return -13;
    }
    delay_us(1U);
  }
  return 0;
}

static int32_t tof_i2c_start(void)
{
  tof_sda_release();
  if(tof_scl_wait_high() != 0)
  {
    return -13;
  }
  delay_us(TOF_SOFT_I2C_HALF_PERIOD_US);
  if(tof_sda_read() == 0U)
  {
    tof_i2c3_last_error = -14;
    return -14;
  }
  tof_sda_low();
  delay_us(TOF_SOFT_I2C_HALF_PERIOD_US);
  tof_scl_low();
  return 0;
}

static void tof_i2c_stop(void)
{
  tof_scl_low();
  tof_sda_low();
  delay_us(TOF_SOFT_I2C_HALF_PERIOD_US);
  tof_scl_release();
  delay_us(TOF_SOFT_I2C_HALF_PERIOD_US);
  tof_sda_release();
  delay_us(TOF_SOFT_I2C_HALF_PERIOD_US);
}

static int32_t tof_i2c_write_byte(uint8_t value)
{
  uint8_t bit_index;
  uint8_t ack;

  for(bit_index = 0U; bit_index < 8U; bit_index++)
  {
    tof_scl_low();
    if((value & 0x80U) != 0U)
    {
      tof_sda_release();
    }
    else
    {
      tof_sda_low();
    }
    delay_us(TOF_SOFT_I2C_HALF_PERIOD_US);
    if(tof_scl_wait_high() != 0)
    {
      return -13;
    }
    delay_us(TOF_SOFT_I2C_HALF_PERIOD_US);
    value <<= 1;
  }

  tof_scl_low();
  tof_sda_release();
  delay_us(TOF_SOFT_I2C_HALF_PERIOD_US);
  if(tof_scl_wait_high() != 0)
  {
    return -13;
  }
  delay_us(TOF_SOFT_I2C_HALF_PERIOD_US);
  ack = tof_sda_read();
  tof_scl_low();
  return (ack == 0U) ? 0 : -10;
}

static int32_t tof_i2c_read_byte(uint8_t *value, uint8_t send_ack)
{
  uint8_t bit_index;
  uint8_t data = 0U;

  tof_sda_release();
  for(bit_index = 0U; bit_index < 8U; bit_index++)
  {
    tof_scl_low();
    delay_us(TOF_SOFT_I2C_HALF_PERIOD_US);
    if(tof_scl_wait_high() != 0)
    {
      return -13;
    }
    data = (uint8_t)((data << 1) | tof_sda_read());
    delay_us(TOF_SOFT_I2C_HALF_PERIOD_US);
  }

  tof_scl_low();
  if(send_ack != 0U)
  {
    tof_sda_low();
  }
  else
  {
    tof_sda_release();
  }
  delay_us(TOF_SOFT_I2C_HALF_PERIOD_US);
  if(tof_scl_wait_high() != 0)
  {
    return -13;
  }
  delay_us(TOF_SOFT_I2C_HALF_PERIOD_US);
  tof_scl_low();
  tof_sda_release();
  *value = data;
  return 0;
}

static void tof_i2c_bus_recover(void)
{
  uint8_t pulse;

  tof_sda_release();
  tof_scl_release();
  delay_us(TOF_SOFT_I2C_HALF_PERIOD_US);
  if(tof_sda_read() != 0U)
  {
    return;
  }

  for(pulse = 0U; pulse < 9U; pulse++)
  {
    tof_scl_low();
    delay_us(TOF_SOFT_I2C_HALF_PERIOD_US);
    tof_scl_release();
    delay_us(TOF_SOFT_I2C_HALF_PERIOD_US);
  }
  tof_i2c_stop();
}

void tof_i2c3_init(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

  tof_scl_release();
  tof_sda_release();
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_pins = TOF_SCL_PIN | TOF_SDA_PIN;
  gpio_init(GPIOB, &gpio_init_struct);

  tof_i2c3_last_error = 0;
  tof_i2c_bus_recover();
}

void tof_i2c3_get_diagnostics(uint8_t *scl_level,
                              uint8_t *sda_level,
                              int32_t *last_error)
{
  if(scl_level != 0)
  {
    *scl_level = tof_scl_read();
  }
  if(sda_level != 0)
  {
    *sda_level = tof_sda_read();
  }
  if(last_error != 0)
  {
    *last_error = tof_i2c3_last_error;
  }
}

int32_t tof_i2c3_write(uint8_t address,
                       uint8_t reg,
                       const uint8_t *data,
                       uint32_t count)
{
  uint32_t index;
  int32_t result;

  if((count > 254U) || ((count > 0U) && (data == 0)))
  {
    return -3;
  }
  result = tof_i2c_start();
  if(result == 0)
  {
    result = tof_i2c_write_byte((uint8_t)(address & 0xFEU));
  }
  if(result == 0)
  {
    result = tof_i2c_write_byte(reg);
    if(result != 0)
    {
      result = -11;
    }
  }
  for(index = 0U; (index < count) && (result == 0); index++)
  {
    result = tof_i2c_write_byte(data[index]);
    if(result != 0)
    {
      result = -12;
    }
  }
  tof_i2c_stop();
  tof_i2c3_last_error = result;
  return result;
}

int32_t tof_i2c3_read(uint8_t address,
                      uint8_t reg,
                      uint8_t *data,
                      uint32_t count)
{
  uint32_t index;
  int32_t result;

  if((data == 0) || (count == 0U) || (count > 255U))
  {
    return -3;
  }

  result = tof_i2c_start();
  if(result == 0)
  {
    result = tof_i2c_write_byte((uint8_t)(address & 0xFEU));
  }
  if(result == 0)
  {
    result = tof_i2c_write_byte(reg);
    if(result != 0)
    {
      result = -11;
    }
  }
  if(result == 0)
  {
    result = tof_i2c_start();
  }
  if(result == 0)
  {
    result = tof_i2c_write_byte((uint8_t)(address | 0x01U));
  }
  for(index = 0U; (index < count) && (result == 0); index++)
  {
    result = tof_i2c_read_byte(&data[index],
                               (index + 1U < count) ? 1U : 0U);
  }
  tof_i2c_stop();
  tof_i2c3_last_error = result;
  return result;
}
