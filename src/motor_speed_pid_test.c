/**
  *****************************************************************************
  * @file     motor_speed_pid_test.c
  * @brief    双路电机速度闭环 PID 测试
  *
  * 电机参数：24V、60:1 减速比，输出轴空载约 100RPM，额定约 75RPM。
  * 开环标定显示 1000 PWM 时输出轴约 97RPM，与铭牌空载 100RPM 一致，
  * 因此编码器按 11 脉冲/电机轴/相、AB 四倍频计算，输出轴每圈为 2640 个计数。
  *
  * 串口输出顺序：
  * 左目标RPM,左实测RPM,左PWM,右目标RPM,右实测RPM,右PWM
  * 其中目标RPM是 PID 设定值，实测RPM由编码器计数增量计算得到，PWM是 PID 输出。
  *****************************************************************************
  */

#include "motor_speed_pid_test.h"

#include <stdint.h>
#include <stdio.h>

#include "at32f45x_board.h"
#include "motor_driver.h"
#include "motor_encoder.h"
#include "motor_pid.h"

/* 编码器和减速箱参数，后续只需修改这一组宏即可重新计算 RPM。 */
#define MOTOR_GEAR_RATIO                         60.0f
#define MOTOR_ENCODER_PULSES_PER_MOTOR_REV       11.0f
#define MOTOR_ENCODER_QUADRATURE_MULTIPLIER       4.0f
#define MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV      \
  (MOTOR_GEAR_RATIO * MOTOR_ENCODER_PULSES_PER_MOTOR_REV * MOTOR_ENCODER_QUADRATURE_MULTIPLIER)

/* PID 控制周期为 100ms，降低编码器量化噪声对控制输出的影响。 */
#define MOTOR_SPEED_PID_PERIOD_MS                100U
#define MOTOR_SPEED_PID_TARGET_RPM                40.0f
#define MOTOR_SPEED_PID_RUN_TIME_MS               8000U
#define MOTOR_SPEED_PID_STOP_TIME_MS              2000U
#define MOTOR_SPEED_PID_REVERSE_TIME_MS           8000U
#define MOTOR_SPEED_PID_CYCLE_STOP_TIME_MS        3000U

/*
 * 初始 PID 参数：
 * Kp 负责根据当前速度误差快速增加 PWM；
 * Ki 负责消除负载造成的稳态误差；
 * Kd 初始设为 0，避免编码器量化噪声放大。
 * 前馈项用于给电机一个接近目标速度的基础 PWM。
 */
#define MOTOR_SPEED_PID_KP                         1.5f
#define MOTOR_SPEED_PID_KI                         2.0f
#define MOTOR_SPEED_PID_KD                         0.0f
#define MOTOR_SPEED_PID_FEED_FORWARD_PWM_PER_RPM  10.0f
#define MOTOR_SPEED_PID_FEED_FORWARD_STATIC_PWM   30.0f

/* 速度反馈低通滤波系数，越小越平滑，但响应会变慢。 */
#define MOTOR_SPEED_FEEDBACK_FILTER_ALPHA          0.25f

/* 限制 PWM 每个控制周期的变化量，避免电机突然加减速引起机械抖动。 */
#define MOTOR_SPEED_PWM_SLEW_PER_CYCLE             40.0f

/* PID 校正量限制在前馈值附近，避免积分项把输出推到反向或满量程。 */
#define MOTOR_SPEED_PID_CORRECTION_LIMIT         350.0f

/* 100ms 采样下，超过此绝对值或相邻变化量的速度样本视为编码器毛刺。 */
#define MOTOR_SPEED_RAW_RPM_LIMIT                 120.0f
#define MOTOR_SPEED_RAW_RPM_STEP_LIMIT             80.0f

/* 若某侧“正 PWM”对应的编码器速度为负，把该侧反馈符号改为 -1。 */
#define MOTOR_LEFT_SPEED_FEEDBACK_SIGN            -1.0f
#define MOTOR_RIGHT_SPEED_FEEDBACK_SIGN           -1.0f

typedef enum
{
  MOTOR_SPEED_PID_STATE_FORWARD = 0,
  MOTOR_SPEED_PID_STATE_STOP_BEFORE_REVERSE,
  MOTOR_SPEED_PID_STATE_REVERSE,
  MOTOR_SPEED_PID_STATE_STOP_BEFORE_FORWARD
} motor_speed_pid_state_t;

static motor_pid_t left_speed_pid;
static motor_pid_t right_speed_pid;
static motor_speed_pid_state_t speed_pid_state;
static uint32_t speed_pid_state_elapsed_ms;
static int32_t last_left_count;
static int32_t last_right_count;
static float left_target_rpm;
static float right_target_rpm;
static float left_filtered_rpm;
static float right_filtered_rpm;
static float left_pwm_command;
static float right_pwm_command;
static uint8_t left_pid_active;
static uint8_t right_pid_active;
static float left_last_valid_raw_rpm;
static float right_last_valid_raw_rpm;

static float motor_speed_count_delta_to_rpm(int32_t count_delta)
{
  return ((float)count_delta * 60000.0f) /
         (MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV * (float)MOTOR_SPEED_PID_PERIOD_MS);
}

static float motor_speed_limit(float value, float value_min, float value_max)
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

/* 丢弃不符合电机物理加速度的单次跳变，避免编码器毛刺进入 PID。 */
static float motor_speed_filter_raw_rpm(float raw_rpm, float *last_valid_rpm)
{
  float step;

  step = raw_rpm - *last_valid_rpm;
  if((raw_rpm > MOTOR_SPEED_RAW_RPM_LIMIT) ||
     (raw_rpm < -MOTOR_SPEED_RAW_RPM_LIMIT) ||
     (step > MOTOR_SPEED_RAW_RPM_STEP_LIMIT) ||
     (step < -MOTOR_SPEED_RAW_RPM_STEP_LIMIT))
  {
    return *last_valid_rpm;
  }

  *last_valid_rpm = raw_rpm;
  return raw_rpm;
}

static int16_t motor_speed_pid_command(float target_rpm,
                                       float measured_rpm,
                                       motor_pid_t *pid,
                                       float *previous_command,
                                       uint8_t *pid_active)
{
  float feed_forward;
  float correction;
  float command;
  float command_delta;

  if(target_rpm == 0.0f)
  {
    motor_pid_reset(pid);
    *previous_command = 0.0f;
    *pid_active = 0U;
    return 0;
  }

  /* 开环实测模型：PWM 约等于 30 + 10 * RPM，负方向使用对称符号。 */
  if(target_rpm > 0.0f)
  {
    feed_forward = MOTOR_SPEED_PID_FEED_FORWARD_STATIC_PWM +
                   (target_rpm * MOTOR_SPEED_PID_FEED_FORWARD_PWM_PER_RPM);
  }
  else
  {
    feed_forward = -MOTOR_SPEED_PID_FEED_FORWARD_STATIC_PWM +
                   (target_rpm * MOTOR_SPEED_PID_FEED_FORWARD_PWM_PER_RPM);
  }
  /*
   * 启动斜坡阶段只使用前馈，不累积 PID 积分。
   * 到达前馈值以后再启用 PI，防止速度尚未建立时产生积分饱和。
   */
  if(*pid_active == 0U)
  {
    correction = 0.0f;
  }
  else
  {
    correction = motor_pid_update(pid,
                                  target_rpm,
                                  measured_rpm,
                                  (float)MOTOR_SPEED_PID_PERIOD_MS / 1000.0f);
    correction = motor_speed_limit(correction,
                                   -MOTOR_SPEED_PID_CORRECTION_LIMIT,
                                   MOTOR_SPEED_PID_CORRECTION_LIMIT);
  }
  command = feed_forward + correction;

  /* 正目标只输出正 PWM，负目标只输出负 PWM，避免过冲后反向抖动。 */
  if(target_rpm > 0.0f)
  {
    command = motor_speed_limit(command, 0.0f, 1000.0f);
  }
  else
  {
    command = motor_speed_limit(command, -1000.0f, 0.0f);
  }

  command_delta = command - *previous_command;
  command_delta = motor_speed_limit(command_delta,
                                    -MOTOR_SPEED_PWM_SLEW_PER_CYCLE,
                                    MOTOR_SPEED_PWM_SLEW_PER_CYCLE);
  *previous_command += command_delta;

  if((*pid_active == 0U) &&
     ((*previous_command - feed_forward) < 0.5f) &&
     ((*previous_command - feed_forward) > -0.5f))
  {
    motor_pid_reset(pid);
    *pid_active = 1U;
  }

  return (int16_t)(*previous_command);
}

static void motor_speed_pid_enter_state(motor_speed_pid_state_t state)
{
  speed_pid_state = state;
  speed_pid_state_elapsed_ms = 0;
  motor_pid_reset(&left_speed_pid);
  motor_pid_reset(&right_speed_pid);
  left_pwm_command = 0.0f;
  right_pwm_command = 0.0f;
  left_pid_active = 0U;
  right_pid_active = 0U;
  left_last_valid_raw_rpm = 0.0f;
  right_last_valid_raw_rpm = 0.0f;

  switch(state)
  {
    case MOTOR_SPEED_PID_STATE_FORWARD:
      left_target_rpm = MOTOR_SPEED_PID_TARGET_RPM;
      right_target_rpm = MOTOR_SPEED_PID_TARGET_RPM;
      break;

    case MOTOR_SPEED_PID_STATE_REVERSE:
      left_target_rpm = -MOTOR_SPEED_PID_TARGET_RPM;
      right_target_rpm = -MOTOR_SPEED_PID_TARGET_RPM;
      break;

    case MOTOR_SPEED_PID_STATE_STOP_BEFORE_REVERSE:
    case MOTOR_SPEED_PID_STATE_STOP_BEFORE_FORWARD:
    default:
      left_target_rpm = 0.0f;
      right_target_rpm = 0.0f;
      motor_driver_stop_all(MOTOR_STOP_COAST);
      break;
  }
}

static void motor_speed_pid_update_state(void)
{
  switch(speed_pid_state)
  {
    case MOTOR_SPEED_PID_STATE_FORWARD:
      if(speed_pid_state_elapsed_ms >= MOTOR_SPEED_PID_RUN_TIME_MS)
      {
        motor_speed_pid_enter_state(MOTOR_SPEED_PID_STATE_STOP_BEFORE_REVERSE);
      }
      break;

    case MOTOR_SPEED_PID_STATE_STOP_BEFORE_REVERSE:
      if(speed_pid_state_elapsed_ms >= MOTOR_SPEED_PID_STOP_TIME_MS)
      {
        motor_speed_pid_enter_state(MOTOR_SPEED_PID_STATE_REVERSE);
      }
      break;

    case MOTOR_SPEED_PID_STATE_REVERSE:
      if(speed_pid_state_elapsed_ms >= MOTOR_SPEED_PID_REVERSE_TIME_MS)
      {
        motor_speed_pid_enter_state(MOTOR_SPEED_PID_STATE_STOP_BEFORE_FORWARD);
      }
      break;

    case MOTOR_SPEED_PID_STATE_STOP_BEFORE_FORWARD:
    default:
      if(speed_pid_state_elapsed_ms >= MOTOR_SPEED_PID_CYCLE_STOP_TIME_MS)
      {
        motor_speed_pid_enter_state(MOTOR_SPEED_PID_STATE_FORWARD);
      }
      break;
  }
}

void motor_speed_pid_test_init(void)
{
  motor_encoder_init();
  motor_driver_init();
  motor_driver_stop_all(MOTOR_STOP_COAST);

  motor_pid_init(&left_speed_pid,
                 MOTOR_SPEED_PID_KP,
                 MOTOR_SPEED_PID_KI,
                 MOTOR_SPEED_PID_KD,
                 -MOTOR_SPEED_PID_CORRECTION_LIMIT,
                 MOTOR_SPEED_PID_CORRECTION_LIMIT);
  motor_pid_init(&right_speed_pid,
                 MOTOR_SPEED_PID_KP,
                 MOTOR_SPEED_PID_KI,
                 MOTOR_SPEED_PID_KD,
                 -MOTOR_SPEED_PID_CORRECTION_LIMIT,
                 MOTOR_SPEED_PID_CORRECTION_LIMIT);

  /* 给操作人员留出时间架空车轮。 */
  delay_sec(3);

  motor_encoder_reset_all();
  last_left_count = motor_encoder_get_count(MOTOR_LEFT);
  last_right_count = motor_encoder_get_count(MOTOR_RIGHT);
  left_filtered_rpm = 0.0f;
  right_filtered_rpm = 0.0f;
  motor_speed_pid_enter_state(MOTOR_SPEED_PID_STATE_FORWARD);
}

void motor_speed_pid_test_loop(void)
{
  int32_t current_left_count;
  int32_t current_right_count;
  int32_t left_count_delta;
  int32_t right_count_delta;
  float left_raw_rpm;
  float right_raw_rpm;
  float left_measured_rpm;
  float right_measured_rpm;
  int16_t left_pwm;
  int16_t right_pwm;

  delay_ms(MOTOR_SPEED_PID_PERIOD_MS);

  current_left_count = motor_encoder_get_count(MOTOR_LEFT);
  current_right_count = motor_encoder_get_count(MOTOR_RIGHT);
  left_count_delta = current_left_count - last_left_count;
  right_count_delta = current_right_count - last_right_count;
  last_left_count = current_left_count;
  last_right_count = current_right_count;

  left_raw_rpm = motor_speed_count_delta_to_rpm(left_count_delta) * MOTOR_LEFT_SPEED_FEEDBACK_SIGN;
  right_raw_rpm = motor_speed_count_delta_to_rpm(right_count_delta) * MOTOR_RIGHT_SPEED_FEEDBACK_SIGN;
  left_raw_rpm = motor_speed_filter_raw_rpm(left_raw_rpm, &left_last_valid_raw_rpm);
  right_raw_rpm = motor_speed_filter_raw_rpm(right_raw_rpm, &right_last_valid_raw_rpm);
  left_filtered_rpm += MOTOR_SPEED_FEEDBACK_FILTER_ALPHA * (left_raw_rpm - left_filtered_rpm);
  right_filtered_rpm += MOTOR_SPEED_FEEDBACK_FILTER_ALPHA * (right_raw_rpm - right_filtered_rpm);
  left_measured_rpm = left_filtered_rpm;
  right_measured_rpm = right_filtered_rpm;

  left_pwm = motor_speed_pid_command(left_target_rpm,
                                     left_measured_rpm,
                                     &left_speed_pid,
                                     &left_pwm_command,
                                     &left_pid_active);
  right_pwm = motor_speed_pid_command(right_target_rpm,
                                      right_measured_rpm,
                                      &right_speed_pid,
                                      &right_pwm_command,
                                      &right_pid_active);
  motor_driver_set_speeds(left_pwm, right_pwm);

  /* 输出目标 RPM、编码器实测 RPM 和 PID 输出 PWM，便于调节参数。 */
  printf("%.2f,%.2f,%d,%.2f,%.2f,%d\r\n",
         left_target_rpm,
         left_measured_rpm,
         left_pwm,
         right_target_rpm,
         right_measured_rpm,
         right_pwm);

  speed_pid_state_elapsed_ms += MOTOR_SPEED_PID_PERIOD_MS;
  motor_speed_pid_update_state();
}
