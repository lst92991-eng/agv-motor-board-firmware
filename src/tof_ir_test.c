/**
  *****************************************************************************
  * @file     tof_ir_test.c
  * @brief    閸欏L53L0X濞村绐涘ù瀣槸
  *
  * 娑撱倓閲淰L53L0X娑撳﹦鏁搁崥搴ｆ畱姒涙顓?娴ｅ不2C閸︽澘娼冮柈鑺ユЦ0x52閵嗗倸鍨垫慨瀣閺冭泛鍘涢崥灞炬閹峰缍?  * XSHUT閿涘苯鍟€闁劒閲滈崬銈夊晪楠炴湹鎱ㄩ弨閫涜礋0x54閸?x56閿涘矂浼╅崗宥呮勾閸р偓閸愯尙鐛婇妴鍌涙拱濞村鐦弳鍌涙
  * 娑撳秴鍨垫慨瀣閹存牞顕伴崣鏍婢舵牗甯撮弨璺烘珤閿涘苯褰ф宀冪槈瀹革箑褰窽OF閻ㄥ嫰鈧矮淇婇崪宀冪獩缁傜粯鏆熼幑顔衡偓?  *****************************************************************************
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
#define TOF_RETRY_DELAY_MS                   1000U
#define TOF_OUTPUT_DELAY_MS                    50U

static VL53L0X_Dev_t tof_left_device;
static VL53L0X_Dev_t tof_right_device;
static VL53L0X_Error tof_left_init_status;
static VL53L0X_Error tof_right_init_status;

static void tof_gpio_init(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOH_PERIPH_CLOCK, TRUE);

  /* XSHUT娴ｅ海鏁搁獮鍐插彠闂傤厺绱堕幇鐔锋珤閿涘苯鍘涚拋鍓х枂鏉堟挸鍤崐鐓庡晙閸掑洦宕睪PIO濡€崇础閵?*/
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

  /* GP_L閵嗕笩P_R閺夊じ绗傚鑼病閺?.7k娑撳﹥濯洪敍灞肩箽閹镐椒璐熷ù顔锯敄閺佹澘鐡ф潏鎾冲弳閵?*/
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
  /* 娑撱倓閲滈崳銊ゆ韫囧懘銆忛崗鍫濇倱閺冭泛鍙ч梻顓ㄧ礉閸氾箑鍨妯款吇閸︽澘娼?x52娴兼艾褰傞悽鐔峰暱缁愪降鈧?*/
  tof_set_shutdown(0U, 0U);
  delay_ms(10U);

  tof_set_shutdown(1U, 0U);
  delay_ms(10U);
  tof_left_init_status = tof_initialize_one(&tof_left_device,
                                            TOF_LEFT_ADDRESS,
                                            "LEFT");

  /* 瀹革缚鏅舵径杈Е閺冨墎鎴风紒顓濈箽閹镐礁鍙鹃崗铏焽閿涘矂浼╅崗宥呯暊娴犮儵绮拋銈呮勾閸р偓楠炲弶澹堥崣鍏呮櫠閻欘剛鐝涢幒銏＄ゴ閵?*/
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
  left_shutdown = (gpio_input_data_bit_read(GPIOB, GPIO_PINS_11) != RESET) ? 1U : 0U;
  right_shutdown = (gpio_input_data_bit_read(GPIOA, GPIO_PINS_11) != RESET) ? 1U : 0U;
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

  printf("\r\nTOF_TEST,VL53L0X_LEFT_RIGHT\r\n");
  printf("TOF_HEADER,left_mm,left_range_status,left_api,"
         "right_mm,right_range_status,right_api,gp_left,gp_right\r\n");
  printf("TOF_EXPECT,api_0_and_range_status_0_are_valid\r\n");
  tof_print_bus_diagnostics();
  tof_initialize_both();
}

void tof_ir_test_loop(void)
{
  VL53L0X_RangingMeasurementData_t left_data;
  VL53L0X_RangingMeasurementData_t right_data;
  VL53L0X_Error left_status;
  VL53L0X_Error right_status;
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

  gp_left = (gpio_input_data_bit_read(GPIOH, GPIO_PINS_3) != RESET) ? 1U : 0U;
  gp_right = (gpio_input_data_bit_read(GPIOA, GPIO_PINS_12) != RESET) ? 1U : 0U;

  printf("TOF,%u,%u,%d,%u,%u,%d,%u,%u\r\n",
         left_data.RangeMilliMeter,
         left_data.RangeStatus,
         left_status,
         right_data.RangeMilliMeter,
         right_data.RangeStatus,
         right_status,
         gp_left,
         gp_right);

  delay_ms(TOF_OUTPUT_DELAY_MS);
}
