/**
 * @file motor_driver.c
 * @brief AT8236双路H桥的20kHz双PWM底层驱动。
 *
 * 输入单位为千分比，正负号使用车辆统一轮向；右电机因物理安装在本层反相。
 * 本模块不做速度、电流或任务决策，只执行PWM、Coast和主动Brake。
 */

#include "motor_driver.h"

#include "at32f45x_crm.h"
#include "at32f45x_gpio.h"
#include "at32f45x_tmr.h"

#define MOTOR_PWM_HZ         20000UL
#define MOTOR_PWM_MAX            1000

static uint32_t motor_pwm_period;

/* TMR8 CH1/2驱动左桥，CH3/4驱动右桥。 */
static tmr_channel_select_type motor_channel(motor_id_t motor, uint8_t in2)
{
  if(motor == MOTOR_LEFT)
    return (in2 != 0U) ? TMR_SELECT_CHANNEL_2 : TMR_SELECT_CHANNEL_1;
  return (in2 != 0U) ? TMR_SELECT_CHANNEL_4 : TMR_SELECT_CHANNEL_3;
}

/** @brief 将PWM千分比换算为TMR8比较值并限制到有效范围。 */
static void motor_pwm(tmr_channel_select_type channel, uint16_t duty)
{
  if(duty > MOTOR_PWM_MAX) duty = MOTOR_PWM_MAX;
  tmr_channel_value_set(TMR8, channel,
      ((motor_pwm_period + 1UL) * duty) / MOTOR_PWM_MAX);
}

/** @brief 初始化PC6～PC9和TMR8四通道20kHz缓冲PWM。 */
void motor_driver_init(void)
{
  crm_clocks_freq_type clocks = {0};
  gpio_init_type gpio;
  tmr_output_config_type output;
  static const tmr_channel_select_type channels[4] =
  {
    TMR_SELECT_CHANNEL_1, TMR_SELECT_CHANNEL_2,
    TMR_SELECT_CHANNEL_3, TMR_SELECT_CHANNEL_4
  };
  uint8_t channel;

  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR8_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio);
  gpio.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9;
  gpio.gpio_mode = GPIO_MODE_MUX;
  gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio.gpio_pull = GPIO_PULL_DOWN;
  gpio.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOC, &gpio);
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE6, GPIO_MUX_3);
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE7, GPIO_MUX_3);
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE8, GPIO_MUX_3);
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE9, GPIO_MUX_3);

  crm_clocks_freq_get(&clocks);
  /* 周期从APB2时钟派生，避免系统时钟调整后PWM频率静默漂移。 */
  motor_pwm_period = clocks.apb2_freq / MOTOR_PWM_HZ - 1UL;
  tmr_base_init(TMR8, motor_pwm_period, 0U);
  tmr_cnt_dir_set(TMR8, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR8, TMR_CLOCK_DIV1);
  tmr_output_default_para_init(&output);
  output.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  output.oc_output_state = TRUE;
  output.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  output.oc_idle_state = FALSE;
  for(channel = 0U; channel < 4U; channel++)
  {
    tmr_output_channel_config(TMR8, channels[channel], &output);
    tmr_output_channel_buffer_enable(TMR8, channels[channel], TRUE);
  }
  tmr_period_buffer_enable(TMR8, TRUE);
  /* 开启复用输出前先写Coast，避免初始化期间出现短促驱动。 */
  motor_driver_stop_all(MOTOR_STOP_COAST);
  tmr_output_enable(TMR8, TRUE);
  tmr_counter_enable(TMR8, TRUE);
}

/** @brief 按车辆统一轮向设置单个AT8236的有符号PWM。 */
void motor_driver_set_speed(motor_id_t motor, int16_t speed)
{
  uint16_t duty;

  if((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT)) return;
  if(speed > MOTOR_PWM_MAX) speed = MOTOR_PWM_MAX;
  if(speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;
  if(motor == MOTOR_RIGHT) speed = (int16_t)-speed;
  if(speed == 0)
  {
    motor_driver_stop(motor, MOTOR_STOP_COAST);
    return;
  }
  duty = (uint16_t)((speed > 0) ? speed : -speed);
  /* 一侧保持100%、另一侧调制，形成AT8236慢衰减双PWM。 */
  motor_pwm(motor_channel(motor, 0U),
      (speed > 0) ? MOTOR_PWM_MAX : (uint16_t)(MOTOR_PWM_MAX - duty));
  motor_pwm(motor_channel(motor, 1U),
      (speed > 0) ? (uint16_t)(MOTOR_PWM_MAX - duty) : MOTOR_PWM_MAX);
}

/** @brief 将左右轮PWM依次交给对应H桥。 */
void motor_driver_set_speeds(int16_t left, int16_t right)
{
  motor_driver_set_speed(MOTOR_LEFT, left);
  motor_driver_set_speed(MOTOR_RIGHT, right);
}

/** @brief 设置单轮Coast或主动Brake电平组合。 */
void motor_driver_stop(motor_id_t motor, motor_stop_mode_t mode)
{
  uint16_t duty;
  if((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT)) return;
  /* 两输入同时高为主动制动，同时低为滑行。 */
  duty = (mode == MOTOR_STOP_BRAKE) ? MOTOR_PWM_MAX : 0U;
  motor_pwm(motor_channel(motor, 0U), duty);
  motor_pwm(motor_channel(motor, 1U), duty);
}

/** @brief 以同一停车模式同时处理左右H桥。 */
void motor_driver_stop_all(motor_stop_mode_t mode)
{
  motor_driver_stop(MOTOR_LEFT, mode);
  motor_driver_stop(MOTOR_RIGHT, mode);
}
