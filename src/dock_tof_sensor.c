/**
  ****************************************************************************
  * @file     dock_tof_sensor.c
  * @brief    充电对准双VL53L0X采集、滤波与几何误差接口
  *
  * 本模块只提供传感器观测，不直接驱动电机。最终对准状态机必须位于应用层，
  * 并同时保留防撞、通信租约、任务超时和充电状态确认。
  ****************************************************************************
  */

#include "dock_tof_sensor.h"

#include <stddef.h>
#include <string.h>

#include "at32f45x_board.h"
#include "at32f45x_conf.h"
#include "tof_i2c3.h"
#include "vl53l0x_api.h"

#define DOCK_TOF_DEFAULT_ADDRESS                   0x52U
#define DOCK_TOF_LEFT_ADDRESS                      0x54U
#define DOCK_TOF_RIGHT_ADDRESS                     0x56U
#define DOCK_TOF_MODEL_ID_REGISTER                 0xC0U
#define DOCK_TOF_EXPECTED_MODEL_ID                 0xEEU
#define DOCK_TOF_INVALID_DISTANCE_MM               8190U
#define DOCK_TOF_FILTER_SAMPLE_COUNT                  3U
#define DOCK_TOF_FILTER_LOW_PASS_DIVISOR              4L
#define DOCK_TOF_FILTER_FIXED_POINT_SHIFT              8
#define DOCK_TOF_REFERENCE_LIMIT_MM                 2000L
#define DOCK_TOF_DEADBAND_LIMIT_MM                   100L

typedef struct
{
  uint16_t samples[DOCK_TOF_FILTER_SAMPLE_COUNT];
  uint8_t sample_count;
  uint8_t next_index;
  uint8_t initialized;
  int32_t filtered_q8;
} dock_tof_filter_t;

typedef struct
{
  VL53L0X_Dev_t left_device;
  VL53L0X_Dev_t right_device;
  VL53L0X_Error left_init_status;
  VL53L0X_Error right_init_status;
  dock_tof_filter_t left_filter;
  dock_tof_filter_t right_filter;
  dock_tof_calibration_t calibration;
  uint32_t sample_sequence;
  uint8_t initialized;
} dock_tof_context_t;

const dock_tof_calibration_t dock_tof_default_calibration =
{
  43L,
  26L,
  5L
};

static dock_tof_context_t dock_tof_context;

static uint8_t dock_tof_calibration_is_valid(
  const dock_tof_calibration_t *calibration)
{
  if(calibration == NULL)
  {
    return 0U;
  }
  if((calibration->left_reference_mm <= 0L) ||
     (calibration->left_reference_mm > DOCK_TOF_REFERENCE_LIMIT_MM) ||
     (calibration->right_reference_mm <= 0L) ||
     (calibration->right_reference_mm > DOCK_TOF_REFERENCE_LIMIT_MM) ||
     (calibration->zero_deadband_mm < 0L) ||
     (calibration->zero_deadband_mm > DOCK_TOF_DEADBAND_LIMIT_MM))
  {
    return 0U;
  }
  return 1U;
}

static uint16_t dock_tof_median3(uint16_t first,
                                 uint16_t second,
                                 uint16_t third)
{
  uint16_t temporary;

  if(first > second)
  {
    temporary = first;
    first = second;
    second = temporary;
  }
  if(second > third)
  {
    temporary = second;
    second = third;
    third = temporary;
  }
  if(first > second)
  {
    second = first;
  }
  return second;
}

static uint16_t dock_tof_filter_update(dock_tof_filter_t *filter,
                                       uint16_t distance_mm)
{
  uint16_t median_mm;
  int32_t target_q8;

  filter->samples[filter->next_index] = distance_mm;
  filter->next_index++;
  if(filter->next_index >= DOCK_TOF_FILTER_SAMPLE_COUNT)
  {
    filter->next_index = 0U;
  }
  if(filter->sample_count < DOCK_TOF_FILTER_SAMPLE_COUNT)
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
    median_mm = dock_tof_median3(filter->samples[0],
                                 filter->samples[1],
                                 filter->samples[2]);
  }

  target_q8 = (int32_t)median_mm << DOCK_TOF_FILTER_FIXED_POINT_SHIFT;
  if(filter->initialized == 0U)
  {
    filter->filtered_q8 = target_q8;
    filter->initialized = 1U;
  }
  else
  {
    filter->filtered_q8 +=
      (target_q8 - filter->filtered_q8) /
      DOCK_TOF_FILTER_LOW_PASS_DIVISOR;
  }

  return (uint16_t)((filter->filtered_q8 +
                     (1L << (DOCK_TOF_FILTER_FIXED_POINT_SHIFT - 1))) >>
                    DOCK_TOF_FILTER_FIXED_POINT_SHIFT);
}

static uint16_t dock_tof_filter_last(const dock_tof_filter_t *filter)
{
  if(filter->initialized == 0U)
  {
    return 0U;
  }
  return (uint16_t)((filter->filtered_q8 +
                     (1L << (DOCK_TOF_FILTER_FIXED_POINT_SHIFT - 1))) >>
                    DOCK_TOF_FILTER_FIXED_POINT_SHIFT);
}

static int32_t dock_tof_apply_deadband(int32_t value_mm)
{
  int32_t deadband_mm = dock_tof_context.calibration.zero_deadband_mm;

  if((value_mm >= -deadband_mm) && (value_mm <= deadband_mm))
  {
    return 0L;
  }
  return value_mm;
}

static uint8_t dock_tof_measurement_is_valid(
  const VL53L0X_RangingMeasurementData_t *measurement,
  VL53L0X_Error api_status)
{
  if((measurement == NULL) ||
     (api_status != VL53L0X_ERROR_NONE) ||
     (measurement->RangeStatus != 0U) ||
     (measurement->RangeMilliMeter == 0U) ||
     (measurement->RangeMilliMeter >= DOCK_TOF_INVALID_DISTANCE_MM))
  {
    return 0U;
  }
  return 1U;
}

static void dock_tof_gpio_init(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOH_PERIPH_CLOCK, TRUE);

  /* 初始化时先拉低两路XSHUT，避免同时占用默认地址。 */
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

  /* 两路GPIO1由模块提供电平，MCU端不启用内部上下拉。 */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_pins = GPIO_PINS_3;
  gpio_init(GPIOH, &gpio_init_struct);
  gpio_init_struct.gpio_pins = GPIO_PINS_12;
  gpio_init(GPIOA, &gpio_init_struct);
}

static void dock_tof_set_shutdown(uint8_t left_enable,
                                  uint8_t right_enable)
{
  gpio_bits_write(GPIOB, GPIO_PINS_11,
                  (left_enable != 0U) ? TRUE : FALSE);
  gpio_bits_write(GPIOA, GPIO_PINS_11,
                  (right_enable != 0U) ? TRUE : FALSE);
}

static VL53L0X_Error dock_tof_initialize_one(VL53L0X_Dev_t *device,
                                             uint8_t new_address)
{
  VL53L0X_Error status;
  uint8_t model_id = 0U;
  uint8_t vhv_settings = 0U;
  uint8_t phase_cal = 0U;
  uint32_t ref_spad_count = 0U;
  uint8_t is_aperture_spads = 0U;

  memset(device, 0, sizeof(*device));
  device->I2cDevAddr = DOCK_TOF_DEFAULT_ADDRESS;
  device->comms_type = 1U;
  device->comms_speed_khz = 100U;

  status = VL53L0X_RdByte(device, DOCK_TOF_MODEL_ID_REGISTER, &model_id);
  if(status != VL53L0X_ERROR_NONE)
  {
    return status;
  }
  if(model_id != DOCK_TOF_EXPECTED_MODEL_ID)
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
  return status;
}

static void dock_tof_initialize_both(void)
{
  memset(&dock_tof_context.left_filter, 0,
         sizeof(dock_tof_context.left_filter));
  memset(&dock_tof_context.right_filter, 0,
         sizeof(dock_tof_context.right_filter));

  dock_tof_set_shutdown(0U, 0U);
  delay_ms(10U);
  dock_tof_set_shutdown(1U, 0U);
  delay_ms(10U);
  dock_tof_context.left_init_status = dock_tof_initialize_one(
      &dock_tof_context.left_device, DOCK_TOF_LEFT_ADDRESS);

  dock_tof_set_shutdown(
      (dock_tof_context.left_init_status == VL53L0X_ERROR_NONE) ? 1U : 0U,
      1U);
  delay_ms(10U);
  dock_tof_context.right_init_status = dock_tof_initialize_one(
      &dock_tof_context.right_device, DOCK_TOF_RIGHT_ADDRESS);
}

uint8_t dock_tof_sensor_init(const dock_tof_calibration_t *calibration)
{
  dock_tof_calibration_t selected;

  if(calibration == NULL)
  {
    selected = dock_tof_default_calibration;
  }
  else
  {
    selected = *calibration;
  }
  if(dock_tof_calibration_is_valid(&selected) == 0U)
  {
    /* 无效标定必须撤销旧READY状态，避免调用者继续使用上一套参数。 */
    memset(&dock_tof_context, 0, sizeof(dock_tof_context));
    return 0U;
  }

  memset(&dock_tof_context, 0, sizeof(dock_tof_context));
  dock_tof_context.calibration = selected;
  dock_tof_gpio_init();
  tof_i2c3_init();
  dock_tof_initialize_both();
  dock_tof_context.initialized = 1U;
  return dock_tof_sensor_is_ready();
}

uint8_t dock_tof_sensor_reinitialize(void)
{
  if(dock_tof_context.initialized == 0U)
  {
    return 0U;
  }
  tof_i2c3_init();
  dock_tof_initialize_both();
  return dock_tof_sensor_is_ready();
}

uint8_t dock_tof_sensor_is_ready(void)
{
  return ((dock_tof_context.initialized != 0U) &&
          (dock_tof_context.left_init_status == VL53L0X_ERROR_NONE) &&
          (dock_tof_context.right_init_status == VL53L0X_ERROR_NONE)) ?
         1U : 0U;
}

uint8_t dock_tof_sensor_sample(dock_tof_snapshot_t *snapshot)
{
  VL53L0X_RangingMeasurementData_t left_data;
  VL53L0X_RangingMeasurementData_t right_data;
  VL53L0X_Error left_status;
  VL53L0X_Error right_status;
  dock_tof_snapshot_t next;
  int32_t left_residual_mm;
  int32_t right_residual_mm;

  if(snapshot == NULL)
  {
    return 0U;
  }

  memset(&next, 0, sizeof(next));
  dock_tof_context.sample_sequence++;
  if(dock_tof_context.sample_sequence == 0UL)
  {
    dock_tof_context.sample_sequence = 1UL;
  }
  next.sample_sequence = dock_tof_context.sample_sequence;
  next.left_api_status = (int8_t)dock_tof_context.left_init_status;
  next.right_api_status = (int8_t)dock_tof_context.right_init_status;

  if(dock_tof_sensor_is_ready() == 0U)
  {
    *snapshot = next;
    return 0U;
  }

  memset(&left_data, 0, sizeof(left_data));
  memset(&right_data, 0, sizeof(right_data));
  left_status = VL53L0X_PerformSingleRangingMeasurement(
      &dock_tof_context.left_device, &left_data);
  right_status = VL53L0X_PerformSingleRangingMeasurement(
      &dock_tof_context.right_device, &right_data);

  next.left_api_status = (int8_t)left_status;
  next.right_api_status = (int8_t)right_status;
  next.left_range_status = left_data.RangeStatus;
  next.right_range_status = right_data.RangeStatus;
  next.left_raw_mm = left_data.RangeMilliMeter;
  next.right_raw_mm = right_data.RangeMilliMeter;
  next.left_valid = dock_tof_measurement_is_valid(&left_data, left_status);
  next.right_valid = dock_tof_measurement_is_valid(&right_data, right_status);

  if(next.left_valid != 0U)
  {
    next.left_filtered_mm = dock_tof_filter_update(
        &dock_tof_context.left_filter, next.left_raw_mm);
  }
  else
  {
    next.left_filtered_mm = dock_tof_filter_last(
        &dock_tof_context.left_filter);
  }
  if(next.right_valid != 0U)
  {
    next.right_filtered_mm = dock_tof_filter_update(
        &dock_tof_context.right_filter, next.right_raw_mm);
  }
  else
  {
    next.right_filtered_mm = dock_tof_filter_last(
        &dock_tof_context.right_filter);
  }

  next.pair_valid = ((next.left_valid != 0U) &&
                     (next.right_valid != 0U)) ? 1U : 0U;
  if(next.pair_valid != 0U)
  {
    left_residual_mm = (int32_t)next.left_filtered_mm -
                       dock_tof_context.calibration.left_reference_mm;
    right_residual_mm = (int32_t)next.right_filtered_mm -
                        dock_tof_context.calibration.right_reference_mm;
    next.distance_error_mm = dock_tof_apply_deadband(
        (left_residual_mm + right_residual_mm) / 2L);
    next.alignment_error_mm = dock_tof_apply_deadband(
        left_residual_mm - right_residual_mm);
  }

  next.left_gpio_level =
      (gpio_input_data_bit_read(GPIOH, GPIO_PINS_3) != RESET) ? 1U : 0U;
  next.right_gpio_level =
      (gpio_input_data_bit_read(GPIOA, GPIO_PINS_12) != RESET) ? 1U : 0U;
  *snapshot = next;
  return next.pair_valid;
}

void dock_tof_sensor_get_bus_diagnostics(uint8_t *scl_level,
                                         uint8_t *sda_level,
                                         int32_t *last_error)
{
  tof_i2c3_get_diagnostics(scl_level, sda_level, last_error);
}
