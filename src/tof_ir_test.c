/**
  *****************************************************************************
  * @file     tof_ir_test.c
  * @brief    双VL53L0X测距与简单滤波测试
  *
  * 两个VL53L0X共用软件I2C总线，默认8位地址均为0x52。
  * 通过XSHUT逐个唤醒并改为0x54和0x56，避免地址冲突。
  * 有效距离依次经过3点中值和一阶低通滤波，用于抑制偶发跳变。
  *****************************************************************************
  */

#include "tof_ir_test.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "at32f45x_board.h"
#include "at32f45x_conf.h"
#include "tof_i2c3.h"
#include "vl53l0x_api.h"

#define TOF_DEFAULT_ADDRESS                 0x52U
#define TOF_LEFT_ADDRESS                    0x54U
#define TOF_RIGHT_ADDRESS                   0x56U
#define TOF_MODEL_ID_REGISTER               0xC0U
#define TOF_EXPECTED_MODEL_ID               0xEEU
#define TOF_INVALID_DISTANCE_MM             8190U
#define TOF_DOCK_REFERENCE_LEFT_MM             43L
#define TOF_DOCK_REFERENCE_RIGHT_MM            26L
#define TOF_DOCK_ZERO_DEADBAND_MM                5L
#define TOF_FILTER_SAMPLE_COUNT                3U
#define TOF_FILTER_LOW_PASS_DIVISOR            4L
#define TOF_FILTER_FIXED_POINT_SHIFT            8
#define TOF_RETRY_DELAY_MS                   1000U
#define TOF_OUTPUT_DELAY_MS                    50U

typedef struct
{
  uint16_t samples[TOF_FILTER_SAMPLE_COUNT];
  uint8_t sample_count;
  uint8_t next_index;
  uint8_t initialized;
  int32_t filtered_q8;
} tof_distance_filter_t;

static VL53L0X_Dev_t tof_left_device;
static VL53L0X_Dev_t tof_right_device;
static VL53L0X_Error tof_left_init_status;
static VL53L0X_Error tof_right_init_status;
static tof_distance_filter_t tof_left_filter;
static tof_distance_filter_t tof_right_filter;

static uint16_t tof_median3(uint16_t first,
                            uint16_t second,
                            uint16_t third)
{
  if(first > second)
  {
    uint16_t temporary = first;
    first = second;
    second = temporary;
  }
  if(second > third)
  {
    uint16_t temporary = second;
    second = third;
    third = temporary;
  }
  if(first > second)
  {
    second = first;
  }

  return second;
}

static uint16_t tof_filter_update(tof_distance_filter_t *filter,
                                  uint16_t distance_mm)
{
  uint16_t median_mm;
  int32_t target_q8;

  filter->samples[filter->next_index] = distance_mm;
  filter->next_index++;
  if(filter->next_index >= TOF_FILTER_SAMPLE_COUNT)
  {
    filter->next_index = 0U;
  }
  if(filter->sample_count < TOF_FILTER_SAMPLE_COUNT)
  {
    filter->sample_count++;
  }

  if(filter->sample_count == 1U)
  {
    median_mm = filter->samples[0];
  }
  else if(filter->sample_count == 2U)
  {
    median_mm = (uint16_t)(((uint32_t)filter->samples[0] +
                            (uint32_t)filter->samples[1]) / 2U);
  }
  else
  {
    median_mm = tof_median3(filter->samples[0],
                            filter->samples[1],
                            filter->samples[2]);
  }

  target_q8 = (int32_t)median_mm << TOF_FILTER_FIXED_POINT_SHIFT;
  if(filter->initialized == 0U)
  {
    filter->filtered_q8 = target_q8;
    filter->initialized = 1U;
  }
  else
  {
    filter->filtered_q8 +=
      (target_q8 - filter->filtered_q8) / TOF_FILTER_LOW_PASS_DIVISOR;
  }

  return (uint16_t)((filter->filtered_q8 +
                     (1L << (TOF_FILTER_FIXED_POINT_SHIFT - 1))) >>
                    TOF_FILTER_FIXED_POINT_SHIFT);
}

static uint16_t tof_filter_last(const tof_distance_filter_t *filter)
{
  if(filter->initialized == 0U)
  {
    return 0U;
  }

  return (uint16_t)((filter->filtered_q8 +
                     (1L << (TOF_FILTER_FIXED_POINT_SHIFT - 1))) >>
                    TOF_FILTER_FIXED_POINT_SHIFT);
}

static int32_t tof_apply_zero_deadband(int32_t value_mm)
{
  if((value_mm >= -TOF_DOCK_ZERO_DEADBAND_MM) &&
     (value_mm <= TOF_DOCK_ZERO_DEADBAND_MM))
  {
    return 0L;
  }

  return value_mm;
}

static uint8_t tof_measurement_is_valid(
  const VL53L0X_RangingMeasurementData_t *measurement,
  VL53L0X_Error api_status)
{
  if((api_status != VL53L0X_ERROR_NONE) ||
     (measurement->RangeStatus != 0U) ||
     (measurement->RangeMilliMeter == 0U) ||
     (measurement->RangeMilliMeter >= TOF_INVALID_DISTANCE_MM))
  {
    return 0U;
  }

  return 1U;
}

static void tof_filter_reset_all(void)
{
  memset(&tof_left_filter, 0, sizeof(tof_left_filter));
  memset(&tof_right_filter, 0, sizeof(tof_right_filter));
}

static void tof_gpio_init(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOH_PERIPH_CLOCK, TRUE);

  /* 初始化时先拉低两个XSHUT，保证两个传感器不会同时占用默认地址。 */
  gpio_bits_reset(GPIOB, GPIO_PINS_11);
  gpio_bits_reset(GPIOA, GPIO_PINS_11);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_pins = GPIO_PINS_11;
  gpio_init(GPIOB, &gpio_init_struct);
  gpio_init(GPIOA, &gpio_init_struct);

  /* 两路GPIO1由传感器板外部上拉，MCU端不启用内部上拉。 */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_pins = GPIO_PINS_3;
  gpio_init(GPIOH, &gpio_init_struct);
  gpio_init_struct.gpio_pins = GPIO_PINS_12;
  gpio_init(GPIOA, &gpio_init_struct);
}

static void tof_set_shutdown(uint8_t left_enable, uint8_t right_enable)
{
  gpio_bits_write(GPIOB,
                  GPIO_PINS_11,
                  (left_enable != 0U) ? TRUE : FALSE);
  gpio_bits_write(GPIOA,
                  GPIO_PINS_11,
                  (right_enable != 0U) ? TRUE : FALSE);
}

static VL53L0X_Error tof_initialize_one(VL53L0X_Dev_t *device,
                                       uint8_t new_address,
                                       const char *name)
{
  VL53L0X_Error status;
  uint8_t model_id = 0U;
  uint8_t vhv_settings = 0U;
  uint8_t phase_cal = 0U;
  uint32_t ref_spad_count = 0U;
  uint8_t is_aperture_spads = 0U;

  memset(device, 0, sizeof(*device));
  device->I2cDevAddr = TOF_DEFAULT_ADDRESS;
  device->comms_type = 1U;
  device->comms_speed_khz = 100U;

  status = VL53L0X_RdByte(device, TOF_MODEL_ID_REGISTER, &model_id);
  printf("TOF_PROBE,%s,api=%d,model=0x%02X,expected=0x%02X\r\n",
         name,
         status,
         model_id,
         TOF_EXPECTED_MODEL_ID);
  if(status != VL53L0X_ERROR_NONE)
  {
    return status;
  }
  if(model_id != TOF_EXPECTED_MODEL_ID)
  {
    return VL53L0X_ERROR_NOT_SUPPORTED;
  }

  status = VL53L0X_DataInit(device);
  if(status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_SetDeviceAddress(device, new_address);
  }
  if(status == VL53L0X_ERROR_NONE)
  {
    device->I2cDevAddr = new_address;
    status = VL53L0X_StaticInit(device);
  }
  if(status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_PerformRefCalibration(device,
                                           &vhv_settings,
                                           &phase_cal);
  }
  if(status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_PerformRefSpadManagement(device,
                                              &ref_spad_count,
                                              &is_aperture_spads);
  }
  if(status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_SetDeviceMode(device,
                                   VL53L0X_DEVICEMODE_SINGLE_RANGING);
  }

  printf("TOF_INIT,%s,api=%d,address=0x%02X,spads=%lu,aperture=%u\r\n",
         name,
         status,
         device->I2cDevAddr,
         ref_spad_count,
         is_aperture_spads);
  return status;
}

static void tof_initialize_both(void)
{
  /* 每次重新初始化传感器时同步清空旧滤波状态。 */
  tof_filter_reset_all();
  tof_set_shutdown(0U, 0U);
  delay_ms(10U);

  tof_set_shutdown(1U, 0U);
  delay_ms(10U);
  tof_left_init_status = tof_initialize_one(&tof_left_device,
                                            TOF_LEFT_ADDRESS,
                                            "LEFT");

  /* 左侧改址完成后保持上电，再唤醒右侧并修改右侧地址。 */
  tof_set_shutdown((tof_left_init_status == VL53L0X_ERROR_NONE) ? 1U : 0U,
                   1U);
  delay_ms(10U);
  tof_right_init_status = tof_initialize_one(&tof_right_device,
                                             TOF_RIGHT_ADDRESS,
                                             "RIGHT");
}

static void tof_print_bus_diagnostics(void)
{
  uint8_t scl_level;
  uint8_t sda_level;
  uint8_t left_shutdown;
  uint8_t right_shutdown;
  int32_t last_error;

  tof_i2c3_get_diagnostics(&scl_level, &sda_level, &last_error);
  left_shutdown =
    (gpio_input_data_bit_read(GPIOB, GPIO_PINS_11) != RESET) ? 1U : 0U;
  right_shutdown =
    (gpio_input_data_bit_read(GPIOA, GPIO_PINS_11) != RESET) ? 1U : 0U;
  printf("TOF_BUS,scl=%u,sda=%u,xshut_l=%u,xshut_r=%u,i2c_error=%d\r\n",
         scl_level,
         sda_level,
         left_shutdown,
         right_shutdown,
         last_error);
}

void tof_ir_test_init(void)
{
  tof_gpio_init();
  tof_i2c3_init();
  tof_filter_reset_all();

  printf("\r\nTOF_TEST,VL53L0X_LEFT_RIGHT_FILTERED\r\n");
  printf("TOF_HEADER,left_raw_mm,left_filtered_mm,left_valid,"
         "right_raw_mm,right_filtered_mm,right_valid,diff_mm,pair_valid,"
         "dock_distance_error_mm,alignment_error_mm,"
         "left_range_status,right_range_status,left_api,right_api,"
         "gp_left,gp_right\r\n");
  printf("TOF_EXPECT,valid_requires_api_0_range_status_0_and_range_1_to_8189\r\n");
  printf("TOF_REFERENCE,left_mm=%ld,right_mm=%ld,diff_mm=%ld,deadband_mm=%ld\r\n",
         TOF_DOCK_REFERENCE_LEFT_MM,
         TOF_DOCK_REFERENCE_RIGHT_MM,
         TOF_DOCK_REFERENCE_LEFT_MM - TOF_DOCK_REFERENCE_RIGHT_MM,
         TOF_DOCK_ZERO_DEADBAND_MM);
  tof_print_bus_diagnostics();
  tof_initialize_both();
}

void tof_ir_test_loop(void)
{
  VL53L0X_RangingMeasurementData_t left_data;
  VL53L0X_RangingMeasurementData_t right_data;
  VL53L0X_Error left_status;
  VL53L0X_Error right_status;
  uint16_t left_filtered_mm;
  uint16_t right_filtered_mm;
  int32_t filtered_difference_mm = 0L;
  int32_t dock_distance_error_mm = 0L;
  int32_t alignment_error_mm = 0L;
  uint8_t left_valid;
  uint8_t right_valid;
  uint8_t pair_valid;
  uint8_t gp_left;
  uint8_t gp_right;

  if((tof_left_init_status != VL53L0X_ERROR_NONE) ||
     (tof_right_init_status != VL53L0X_ERROR_NONE))
  {
    printf("TOF_INIT_RETRY,left_api=%d,right_api=%d\r\n",
           tof_left_init_status,
           tof_right_init_status);
    tof_print_bus_diagnostics();
    delay_ms(TOF_RETRY_DELAY_MS);
    tof_i2c3_init();
    tof_initialize_both();
    return;
  }

  memset(&left_data, 0, sizeof(left_data));
  memset(&right_data, 0, sizeof(right_data));
  left_status = VL53L0X_PerformSingleRangingMeasurement(&tof_left_device,
                                                        &left_data);
  right_status = VL53L0X_PerformSingleRangingMeasurement(&tof_right_device,
                                                         &right_data);

  left_valid = tof_measurement_is_valid(&left_data, left_status);
  right_valid = tof_measurement_is_valid(&right_data, right_status);

  if(left_valid != 0U)
  {
    left_filtered_mm = tof_filter_update(&tof_left_filter,
                                         left_data.RangeMilliMeter);
  }
  else
  {
    /* 无效测量不进入滤波器，滤波值仅保留用于诊断。 */
    left_filtered_mm = tof_filter_last(&tof_left_filter);
  }

  if(right_valid != 0U)
  {
    right_filtered_mm = tof_filter_update(&tof_right_filter,
                                          right_data.RangeMilliMeter);
  }
  else
  {
    /* 业务层必须结合right_valid判断，不能把保留值当作新测量。 */
    right_filtered_mm = tof_filter_last(&tof_right_filter);
  }

  pair_valid = ((left_valid != 0U) && (right_valid != 0U)) ? 1U : 0U;
  if(pair_valid != 0U)
  {
    int32_t left_residual_mm;
    int32_t right_residual_mm;

    filtered_difference_mm = (int32_t)left_filtered_mm -
                             (int32_t)right_filtered_mm;
    left_residual_mm = (int32_t)left_filtered_mm -
                       TOF_DOCK_REFERENCE_LEFT_MM;
    right_residual_mm = (int32_t)right_filtered_mm -
                        TOF_DOCK_REFERENCE_RIGHT_MM;
    dock_distance_error_mm =
      tof_apply_zero_deadband((left_residual_mm + right_residual_mm) / 2L);
    alignment_error_mm =
      tof_apply_zero_deadband(left_residual_mm - right_residual_mm);
  }

  gp_left =
    (gpio_input_data_bit_read(GPIOH, GPIO_PINS_3) != RESET) ? 1U : 0U;
  gp_right =
    (gpio_input_data_bit_read(GPIOA, GPIO_PINS_12) != RESET) ? 1U : 0U;

  printf("TOF,%u,%u,%u,%u,%u,%u,%ld,%u,%ld,%ld,%u,%u,%d,%d,%u,%u\r\n",
         left_data.RangeMilliMeter,
         left_filtered_mm,
         left_valid,
         right_data.RangeMilliMeter,
         right_filtered_mm,
         right_valid,
         filtered_difference_mm,
         pair_valid,
         dock_distance_error_mm,
         alignment_error_mm,
         left_data.RangeStatus,
         right_data.RangeStatus,
         left_status,
         right_status,
         gp_left,
         gp_right);

  delay_ms(TOF_OUTPUT_DELAY_MS);
}
