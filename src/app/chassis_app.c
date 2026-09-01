/**
 * @file chassis_app.c
 * @brief 一条可讲清的控制链：CAN→安全→里程计→差速PID→PWM。
 */
#include "chassis_app.h"

#include <string.h>

#include "adc_dma.h"
#include "board_config.h"
#include "board_port.h"
#include "bumper.h"
#include "canfd_driver.h"
#include "chassis_protocol.h"
#include "drive_control.h"
#include "imu_bmi270.h"
#include "motor_encoder.h"
#include "odometry.h"
#include "safety.h"

typedef enum
{
  CHASSIS_DISABLED = 0,
  CHASSIS_ENABLED,
  CHASSIS_RUNNING,
  CHASSIS_FAULT
} chassis_state_t;

static chassis_state_t chassis_state;
static uint8_t chassis_reference_seen;
static uint8_t chassis_speed_ticks;
static uint8_t chassis_adc_ticks;
static uint8_t chassis_status_ticks;
static uint8_t chassis_odom_ticks;
static uint16_t chassis_last_command_sequence;
static uint8_t chassis_last_command_result;
static uint32_t chassis_uptime_ms;

static int16_t chassis_i16(float value)
{
  if(value > 32767.0f) return 32767;
  if(value < -32768.0f) return -32768;
  return (int16_t)value;
}

static void chassis_stop_fault(uint8_t fault)
{
  safety_raise(fault);
  drive_control_stop();
  chassis_protocol_clear_reference();
  chassis_state = CHASSIS_FAULT;
  chassis_reference_seen = 0U;
}

static void chassis_handle_command(void)
{
  chassis_command_t command;
  if(chassis_protocol_take_command(&command) == 0U) return;
  chassis_last_command_sequence = command.sequence;
  chassis_last_command_result = 0U;
  switch(command.type)
  {
    case CHASSIS_CMD_ENABLE:
      if((chassis_state == CHASSIS_DISABLED) && (safety_can_enable() != 0U) &&
         (imu_bmi270_fresh() != 0U) && (bumper_can_clear() != 0U) &&
         (chassis_protocol_bus_healthy() != 0U)) chassis_state = CHASSIS_ENABLED;
      else chassis_last_command_result = 1U;
      break;
    case CHASSIS_CMD_DISABLE:
    case CHASSIS_CMD_STOP:
      drive_control_stop();
      chassis_protocol_clear_reference();
      chassis_reference_seen = 0U;
      if(chassis_state != CHASSIS_FAULT) chassis_state = CHASSIS_DISABLED;
      break;
    case CHASSIS_CMD_ESTOP:
      chassis_stop_fault(SAFETY_FAULT_ESTOP);
      break;
    case CHASSIS_CMD_FAULT_RESET:
      if(chassis_protocol_bus_healthy() == 0U) canfd_driver_recover();
      if((bumper_clear() != 0U) && (imu_bmi270_fresh() != 0U) &&
         (chassis_protocol_bus_healthy() != 0U) && (safety_clear() != 0U))
      {
        drive_control_clear_fault();
        chassis_state = CHASSIS_DISABLED;
      }
      else chassis_last_command_result = 1U;
      break;
    case CHASSIS_CMD_RESET_ODOMETRY:
      if(chassis_state == CHASSIS_DISABLED)
      {
        motor_encoder_reset_all();
        odometry_reset();
      }
      else chassis_last_command_result = 1U;
      break;
    default:
      chassis_last_command_result = 1U;
      break;
  }
}

static void chassis_send_status(void)
{
  chassis_status_t status;
  safety_status_t safety;
  odometry_t odometry;

  safety_get(&safety);
  odometry_get(&odometry);
  memset(&status, 0, sizeof(status));
  status.state = (uint8_t)chassis_state;
  status.fault = safety.fault;
  status.bumper_mask = bumper_latched();
  status.flags = ((imu_bmi270_fresh() != 0U) ? 0x01U : 0U) |
      ((odometry.valid != 0U) ? 0x02U : 0U);
  status.command_sequence = chassis_last_command_sequence;
  status.command_result = chassis_last_command_result;
  status.left_pwm = drive_control_pwm(MOTOR_LEFT);
  status.right_pwm = drive_control_pwm(MOTOR_RIGHT);
  status.bus_mv = safety.bus_mv;
  status.left_current_ma = safety.left_current_ma;
  status.right_current_ma = safety.right_current_ma;
  chassis_protocol_send_status(&status, chassis_uptime_ms);
}

static void chassis_send_odometry(void)
{
  odometry_t odometry;
  float left_body = drive_control_measured_rpm(MOTOR_LEFT) * BOARD_LEFT_TARGET_SIGN;
  float right_body = drive_control_measured_rpm(MOTOR_RIGHT) * BOARD_RIGHT_TARGET_SIGN;
  float linear = (left_body + right_body) * BOARD_WHEEL_CIRCUMFERENCE_MM / 120.0f;
  odometry_get(&odometry);
  chassis_protocol_send_odometry(&odometry, chassis_i16(linear),
      chassis_i16(imu_bmi270_gyro_z_dps() * 100.0f), chassis_uptime_ms);
}

void chassis_app_step(void)
{
  drive_reference_t reference;
  adc_dma_sample_t adc;
  odometry_t odometry;
  float dt_s = BOARD_CONTROL_PERIOD_MS / 1000.0f;

  chassis_uptime_ms += BOARD_CONTROL_PERIOD_MS;
  chassis_protocol_poll(chassis_uptime_ms);
  bumper_update();
  if(++chassis_adc_ticks >= (BOARD_ADC_PERIOD_MS / BOARD_CONTROL_PERIOD_MS))
  {
    chassis_adc_ticks = 0U;
    adc_dma_start();
  }
  if(adc_dma_take(&adc) != 0U)
    safety_update(chassis_uptime_ms, &adc,
        ((chassis_state == CHASSIS_DISABLED) &&
         (drive_control_pwm(MOTOR_LEFT) == 0) &&
         (drive_control_pwm(MOTOR_RIGHT) == 0)) ? 1U : 0U);
  else safety_update(chassis_uptime_ms, 0, 0U);
  if(bumper_latched() != 0U) safety_raise(SAFETY_FAULT_BUMPER);
  if(chassis_protocol_bus_healthy() == 0U) safety_raise(SAFETY_FAULT_CAN);
  if(imu_bmi270_update(dt_s) == 0U) safety_raise(SAFETY_FAULT_IMU);
  if(safety_fault() != SAFETY_FAULT_NONE) chassis_stop_fault(safety_fault());
  chassis_handle_command();

  if((chassis_state == CHASSIS_ENABLED) || (chassis_state == CHASSIS_RUNNING))
  {
    if(chassis_protocol_get_reference(chassis_uptime_ms, &reference) != 0U)
    {
      chassis_reference_seen = 1U;
      if((reference.linear_mm_s == 0) && (reference.angular_mrad_s == 0) &&
         (reference.accel_mm_s2 == 0U) && (reference.decel_mm_s2 == 0U))
      {
        drive_control_stop();
        chassis_state = CHASSIS_ENABLED;
      }
      else if(++chassis_speed_ticks >= (BOARD_SPEED_PERIOD_MS / BOARD_CONTROL_PERIOD_MS))
      {
        chassis_speed_ticks = 0U;
        if(drive_control_update(&reference,
            imu_bmi270_gyro_z_dps() * 17.45329252f, BOARD_SPEED_PERIOD_MS / 1000.0f) == 0U)
          chassis_stop_fault(SAFETY_FAULT_DRIVE);
        else chassis_state = CHASSIS_RUNNING;
      }
    }
    else if(chassis_reference_seen != 0U) chassis_stop_fault(SAFETY_FAULT_TIMEOUT);
  }

  odometry_update(-motor_encoder_get_count(MOTOR_LEFT),
                  -motor_encoder_get_count(MOTOR_RIGHT),
                  imu_bmi270_gyro_z_dps(), dt_s);
  odometry_get(&odometry);
  if(odometry.valid == 0U) chassis_stop_fault(SAFETY_FAULT_IMU);
  if(++chassis_status_ticks >= (BOARD_STATUS_PERIOD_MS / BOARD_CONTROL_PERIOD_MS))
  {
    chassis_status_ticks = 0U;
    chassis_send_status();
  }
  else if(++chassis_odom_ticks >= (BOARD_SPEED_PERIOD_MS / BOARD_CONTROL_PERIOD_MS))
  {
    chassis_odom_ticks = 0U;
    chassis_send_odometry();
  }
  board_port_watchdog_feed();
}

uint8_t chassis_app_init(void)
{
  drive_control_init();
  bumper_init();
  safety_init();
  odometry_init();
  chassis_protocol_init();
  chassis_state = CHASSIS_DISABLED;
  chassis_uptime_ms = 0U;
  if((adc_dma_init() == 0U) || (imu_bmi270_init() == 0U) ||
     (board_port_watchdog_init() == 0U)) chassis_stop_fault(SAFETY_FAULT_POWER);
  return 1U;
}
