/**
  *****************************************************************************
  * @file     motor_driver.c
  * @brief    双路 AT8236 直流电机驱动
  *
  * 硬件连接：
  * 左电机：PC6 -> L_IN1，PC7 -> L_IN2；
  * 右电机：PC8 -> R_IN1，PC9 -> R_IN2。
  * 四个引脚使用 TMR8 的四个 PWM 通道，复用功能为 GPIO_MUX_3。
  *****************************************************************************
  */

#include "motor_driver.h"

#include "at32f45x.h"
#include "at32f45x_crm.h"
#include "at32f45x_gpio.h"
#include "at32f45x_tmr.h"

/* 20kHz 高于常见可听频率，电机低速运行时不会出现明显 PWM 啸叫。 */
#define MOTOR_PWM_FREQUENCY_HZ          20000UL
#define MOTOR_SPEED_MAX_PERMILLE        1000
#define MOTOR_LEFT_OUTPUT_DIRECTION         1
#define MOTOR_RIGHT_OUTPUT_DIRECTION       -1

static uint32_t motor_pwm_period;

/* 将电机输入端映射到 TMR8 的四个输出比较通道。 */
static tmr_channel_select_type motor_in1_channel(motor_id_t motor)
{
  return (motor == MOTOR_LEFT) ? TMR_SELECT_CHANNEL_1 : TMR_SELECT_CHANNEL_3;
}

static tmr_channel_select_type motor_in2_channel(motor_id_t motor)
{
  return (motor == MOTOR_LEFT) ? TMR_SELECT_CHANNEL_2 : TMR_SELECT_CHANNEL_4;
}

/* 写入单个 PWM 通道，占空比参数使用 0~1000 的千分比。 */
static void motor_pwm_set(tmr_channel_select_type channel, uint16_t duty_permille)
{
  uint32_t compare_value;

  if(duty_permille > MOTOR_SPEED_MAX_PERMILLE)
  {
    duty_permille = MOTOR_SPEED_MAX_PERMILLE;
  }

  compare_value = ((motor_pwm_period + 1UL) * duty_permille) / MOTOR_SPEED_MAX_PERMILLE;
  tmr_channel_value_set(TMR8, channel, compare_value);
}

void motor_driver_init(void)
{
  crm_clocks_freq_type clocks = {0};
  gpio_init_type gpio_init_struct;
  tmr_output_config_type output_config;

  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR8_PERIPH_CLOCK, TRUE);

  /* PC6~PC9 分别对应 TMR8_CH1~CH4。 */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOC, &gpio_init_struct);

  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE6, GPIO_MUX_3);
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE7, GPIO_MUX_3);
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE8, GPIO_MUX_3);
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE9, GPIO_MUX_3);

  crm_clocks_freq_get(&clocks);
  motor_pwm_period = (clocks.apb2_freq / MOTOR_PWM_FREQUENCY_HZ) - 1UL;

  tmr_base_init(TMR8, motor_pwm_period, 0);
  tmr_cnt_dir_set(TMR8, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR8, TMR_CLOCK_DIV1);

  tmr_output_default_para_init(&output_config);
  output_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  output_config.oc_output_state = TRUE;
  output_config.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  output_config.oc_idle_state = FALSE;

  tmr_output_channel_config(TMR8, TMR_SELECT_CHANNEL_1, &output_config);
  tmr_output_channel_config(TMR8, TMR_SELECT_CHANNEL_2, &output_config);
  tmr_output_channel_config(TMR8, TMR_SELECT_CHANNEL_3, &output_config);
  tmr_output_channel_config(TMR8, TMR_SELECT_CHANNEL_4, &output_config);

  tmr_output_channel_buffer_enable(TMR8, TMR_SELECT_CHANNEL_1, TRUE);
  tmr_output_channel_buffer_enable(TMR8, TMR_SELECT_CHANNEL_2, TRUE);
  tmr_output_channel_buffer_enable(TMR8, TMR_SELECT_CHANNEL_3, TRUE);
  tmr_output_channel_buffer_enable(TMR8, TMR_SELECT_CHANNEL_4, TRUE);
  tmr_period_buffer_enable(TMR8, TRUE);

  /* 先把比较值清零，再开启高级定时器主输出，防止初始化瞬间误转。 */
  motor_driver_stop_all(MOTOR_STOP_COAST);
  tmr_output_enable(TMR8, TRUE);
  tmr_counter_enable(TMR8, TRUE);
}

void motor_driver_set_speed(motor_id_t motor, int16_t speed_permille)
{
  int16_t output_direction;
  uint16_t duty_permille;
  uint16_t decay_duty_permille;

  if((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT))
  {
    return;
  }

  if(speed_permille > MOTOR_SPEED_MAX_PERMILLE)
  {
    speed_permille = MOTOR_SPEED_MAX_PERMILLE;
  }
  else if(speed_permille < -MOTOR_SPEED_MAX_PERMILLE)
  {
    speed_permille = -MOTOR_SPEED_MAX_PERMILLE;
  }

  /* 右电机镜像安装，统一补偿后，正值始终表示车轮推动小车向前。 */
  output_direction = (motor == MOTOR_LEFT) ? MOTOR_LEFT_OUTPUT_DIRECTION :
                                             MOTOR_RIGHT_OUTPUT_DIRECTION;
  speed_permille = (int16_t)(speed_permille * output_direction);

  if(speed_permille > 0)
  {
    duty_permille = (uint16_t)speed_permille;
    decay_duty_permille = MOTOR_SPEED_MAX_PERMILLE - duty_permille;

    /*
     * AT8236 正转慢衰减模式：IN1=1，IN2=PWM。
     * IN2 的高电平阶段为刹车，低电平阶段为正向驱动，因此写入 1-duty。
     */
    motor_pwm_set(motor_in1_channel(motor), MOTOR_SPEED_MAX_PERMILLE);
    motor_pwm_set(motor_in2_channel(motor), decay_duty_permille);
  }
  else if(speed_permille < 0)
  {
    duty_permille = (uint16_t)(-speed_permille);
    decay_duty_permille = MOTOR_SPEED_MAX_PERMILLE - duty_permille;

    /* AT8236 反转慢衰减模式：IN1=PWM，IN2=1。 */
    motor_pwm_set(motor_in1_channel(motor), decay_duty_permille);
    motor_pwm_set(motor_in2_channel(motor), MOTOR_SPEED_MAX_PERMILLE);
  }
  else
  {
    motor_driver_stop(motor, MOTOR_STOP_COAST);
  }
}

void motor_driver_set_speeds(int16_t left_speed_permille, int16_t right_speed_permille)
{
  motor_driver_set_speed(MOTOR_LEFT, left_speed_permille);
  motor_driver_set_speed(MOTOR_RIGHT, right_speed_permille);
}

void motor_driver_stop(motor_id_t motor, motor_stop_mode_t stop_mode)
{
  uint16_t duty_permille;

  if((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT))
  {
    return;
  }

  /* AT8236 两个输入都为低时滑行，都为高时进行短刹。 */
  duty_permille = (stop_mode == MOTOR_STOP_BRAKE) ? MOTOR_SPEED_MAX_PERMILLE : 0U;
  motor_pwm_set(motor_in1_channel(motor), duty_permille);
  motor_pwm_set(motor_in2_channel(motor), duty_permille);
}

void motor_driver_stop_all(motor_stop_mode_t stop_mode)
{
  motor_driver_stop(MOTOR_LEFT, stop_mode);
  motor_driver_stop(MOTOR_RIGHT, stop_mode);
}
