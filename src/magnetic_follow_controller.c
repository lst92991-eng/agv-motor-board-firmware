/**
  ****************************************************************************
  * @file     magnetic_follow_controller.c
  * @brief    磁条质量判定、横向偏移外环和循迹动态限速
  *
  * 本模块是纯控制逻辑，不读取CAN、不写PWM，也不处理安全锁存。调用者必须先
  * 保证样本时效，并在连续无效时执行本地停车和故障锁存。
  ****************************************************************************
  */

#include "magnetic_follow_controller.h"

#include <stddef.h>
#include <string.h>

#include "math_constants.h"

#define MAG_FOLLOW_REQUIRED_FLAGS                         0x07U
#define MAG_FOLLOW_MIN_STRENGTH                            180U
#define MAG_FOLLOW_MAX_AGE_MS                              100U
#define MAG_FOLLOW_MAX_OFFSET_X10MM                        350L
#define MAG_FOLLOW_SECONDARY_MAX_PERCENT                    50U
#define MAG_FOLLOW_OFFSET_DEADBAND_MM                         1.5f
#define MAG_FOLLOW_YAW_RATE_KP_DPS_PER_MM                     2.0f
#define MAG_FOLLOW_YAW_RATE_KD_DPS_PER_MM_S                   1.0f
#define MAG_FOLLOW_CURVE_YAW_RATE_KD_DPS_PER_MM_S             0.35f
#define MAG_FOLLOW_OFFSET_RATE_FILTER_ALPHA                    0.50f
#define MAG_FOLLOW_CURVE_OFFSET_RATE_FILTER_ALPHA              0.25f
#define MAG_FOLLOW_CURVE_RATE_TERM_LIMIT_DPS                   3.0f
#define MAG_FOLLOW_YAW_RATE_LIMIT_DPS                          8.0f
#define MAG_FOLLOW_CURVE_YAW_RATE_LIMIT_DPS                   12.0f
#define MAG_FOLLOW_FULL_SPEED_OFFSET_MM                        3.0f
#define MAG_FOLLOW_MIN_SPEED_OFFSET_MM                        15.0f
#define MAG_FOLLOW_MIN_SPEED_RPM                              20.0f
#define MAG_FOLLOW_MIN_SPEED_RATIO                             0.65f
#define MAG_FOLLOW_SPEED_LIMIT_RISE_RPM                        0.10f
#define MAG_FOLLOW_SPEED_LIMIT_FALL_RPM                        0.75f
#define MAG_FOLLOW_MIN_REQUEST_RPM                             8.0f
#define MAG_FOLLOW_ADAPTIVE_ENABLE_RPM                        30.0f
#define MAG_FOLLOW_ADAPTIVE_SUSPECT_CAP_RPM                   33.70f
#define MAG_FOLLOW_ADAPTIVE_CURVE_CAP_RPM                     16.85f
#define MAG_FOLLOW_ADAPTIVE_EMERGENCY_CAP_RPM                 11.23f
#define MAG_FOLLOW_ADAPTIVE_SPEED_RISE_RPM                     0.20f
#define MAG_FOLLOW_SUSPECT_OFFSET_MM                           4.0f
#define MAG_FOLLOW_SUSPECT_TREND_MM_S                          5.0f
#define MAG_FOLLOW_SUSPECT_ACTUAL_YAW_DPS                      4.0f
#define MAG_FOLLOW_CONFIRM_OFFSET_MM                           6.0f
#define MAG_FOLLOW_CONFIRM_ACTUAL_YAW_DPS                      6.0f
#define MAG_FOLLOW_EMERGENCY_OFFSET_MM                         9.0f
#define MAG_FOLLOW_EMERGENCY_ACTUAL_YAW_DPS                   12.0f
#define MAG_FOLLOW_EXIT_OFFSET_MM                              3.0f
#define MAG_FOLLOW_EXIT_ACTUAL_YAW_DPS                         2.5f
#define MAG_FOLLOW_SUSPECT_ENTER_TICKS                          3U
#define MAG_FOLLOW_CONFIRM_TICKS                                5U
#define MAG_FOLLOW_SUSPECT_TIMEOUT_TICKS                       20U
#define MAG_FOLLOW_CURVE_EXIT_TICKS                            30U
#define MAG_FOLLOW_EMERGENCY_RECOVER_TICKS                     20U
#define MAG_FOLLOW_TARGET_SIGN                                1.0f

static int32_t magnetic_follow_abs_i32(int32_t value)
{
  return (value < 0L) ? -value : value;
}

static float magnetic_follow_abs_float(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float magnetic_follow_limit(float value,
                                   float minimum,
                                   float maximum)
{
  if(value < minimum)
  {
    return minimum;
  }
  if(value > maximum)
  {
    return maximum;
  }
  return value;
}

static float magnetic_follow_minimum_speed_rpm(
    float requested_run_rpm)
{
  float minimum_speed_rpm = requested_run_rpm *
                            MAG_FOLLOW_MIN_SPEED_RATIO;

  minimum_speed_rpm = magnetic_follow_limit(
      minimum_speed_rpm,
      MAG_FOLLOW_MIN_REQUEST_RPM,
      MAG_FOLLOW_MIN_SPEED_RPM);
  if(minimum_speed_rpm > requested_run_rpm)
  {
    minimum_speed_rpm = requested_run_rpm;
  }
  return minimum_speed_rpm;
}

static float magnetic_follow_cap_speed(float requested_run_rpm,
                                       float cap_rpm)
{
  return (requested_run_rpm < cap_rpm) ? requested_run_rpm : cap_rpm;
}

static void magnetic_follow_update_adaptive_speed_phase(
    magnetic_follow_controller_t *controller,
    float offset_error_mm)
{
  float absolute_offset_mm = magnetic_follow_abs_float(offset_error_mm);
  float absolute_target_yaw_rate_dps = magnetic_follow_abs_float(
      controller->yaw_rate_target_dps);
  float absolute_actual_yaw_rate_dps = magnetic_follow_abs_float(
      controller->actual_yaw_rate_dps);
  uint8_t outward_trend;
  uint8_t yaw_evidence;
  uint8_t suspect;
  uint8_t confirmed;
  uint8_t emergency;
  uint8_t straight;

  if(controller->adaptive_high_speed == 0U)
  {
    return;
  }

  outward_trend = (((offset_error_mm >= MAG_FOLLOW_SUSPECT_OFFSET_MM) &&
                    (controller->offset_rate_mm_s >=
                     MAG_FOLLOW_SUSPECT_TREND_MM_S)) ||
                   ((offset_error_mm <= -MAG_FOLLOW_SUSPECT_OFFSET_MM) &&
                    (controller->offset_rate_mm_s <=
                     -MAG_FOLLOW_SUSPECT_TREND_MM_S))) ? 1U : 0U;
  yaw_evidence = ((absolute_offset_mm >= 2.5f) &&
                  (absolute_actual_yaw_rate_dps >=
                   MAG_FOLLOW_SUSPECT_ACTUAL_YAW_DPS) &&
                  ((controller->yaw_rate_target_dps *
                    controller->actual_yaw_rate_dps) > 0.0f)) ? 1U : 0U;
  suspect = ((outward_trend != 0U) || (yaw_evidence != 0U)) ? 1U : 0U;
  confirmed = ((absolute_offset_mm >= MAG_FOLLOW_CONFIRM_OFFSET_MM) ||
               ((absolute_actual_yaw_rate_dps >=
                 MAG_FOLLOW_CONFIRM_ACTUAL_YAW_DPS) &&
                (absolute_target_yaw_rate_dps >= 4.0f) &&
                ((controller->yaw_rate_target_dps *
                  controller->actual_yaw_rate_dps) > 0.0f))) ? 1U : 0U;
  emergency = ((absolute_offset_mm >= MAG_FOLLOW_EMERGENCY_OFFSET_MM) ||
               ((absolute_offset_mm >= 6.0f) &&
                (absolute_actual_yaw_rate_dps >=
                 MAG_FOLLOW_EMERGENCY_ACTUAL_YAW_DPS))) ? 1U : 0U;
  /* 0.1 mm量化在10 ms采样下会制造很大的瞬时偏移变化率，不能把它作为
   * 直线恢复的硬门槛；用偏移和实际所需转向量连续确认即可。
   */
  straight = ((absolute_offset_mm <= MAG_FOLLOW_EXIT_OFFSET_MM) &&
              (absolute_actual_yaw_rate_dps <=
               MAG_FOLLOW_EXIT_ACTUAL_YAW_DPS)) ? 1U : 0U;

  if(emergency != 0U)
  {
    controller->speed_phase = MAGNETIC_FOLLOW_SPEED_CURVE_EMERGENCY;
    controller->curve_evidence_ticks = 0U;
    controller->straight_evidence_ticks = 0U;
    return;
  }

  if(controller->speed_phase == MAGNETIC_FOLLOW_SPEED_FAST_STRAIGHT)
  {
    if(suspect != 0U)
    {
      if(controller->curve_evidence_ticks < UINT16_MAX)
      {
        controller->curve_evidence_ticks++;
      }
      if(controller->curve_evidence_ticks >=
         MAG_FOLLOW_SUSPECT_ENTER_TICKS)
      {
        controller->speed_phase = MAGNETIC_FOLLOW_SPEED_CURVE_SUSPECT;
        controller->straight_evidence_ticks = 0U;
      }
    }
    else
    {
      controller->curve_evidence_ticks = 0U;
    }
  }
  else if(controller->speed_phase == MAGNETIC_FOLLOW_SPEED_CURVE_SUSPECT)
  {
    if(suspect != 0U)
    {
      if(controller->curve_evidence_ticks < UINT16_MAX)
      {
        controller->curve_evidence_ticks++;
      }
      controller->straight_evidence_ticks = 0U;
      if(((confirmed != 0U) &&
          (controller->curve_evidence_ticks >=
           MAG_FOLLOW_CONFIRM_TICKS)) ||
         (controller->curve_evidence_ticks >=
          MAG_FOLLOW_SUSPECT_TIMEOUT_TICKS))
      {
        controller->speed_phase = MAGNETIC_FOLLOW_SPEED_CURVE_TRACK;
        controller->curve_evidence_ticks = 0U;
      }
    }
    else
    {
      controller->curve_evidence_ticks = 0U;
      if(controller->straight_evidence_ticks < UINT16_MAX)
      {
        controller->straight_evidence_ticks++;
      }
      if(controller->straight_evidence_ticks >=
         MAG_FOLLOW_SUSPECT_TIMEOUT_TICKS)
      {
        controller->speed_phase = MAGNETIC_FOLLOW_SPEED_FAST_STRAIGHT;
        controller->straight_evidence_ticks = 0U;
      }
    }
  }
  else if(controller->speed_phase == MAGNETIC_FOLLOW_SPEED_CURVE_TRACK)
  {
    if(straight != 0U)
    {
      if(controller->straight_evidence_ticks < UINT16_MAX)
      {
        controller->straight_evidence_ticks++;
      }
      if(controller->straight_evidence_ticks >=
         MAG_FOLLOW_CURVE_EXIT_TICKS)
      {
        controller->speed_phase = MAGNETIC_FOLLOW_SPEED_FAST_STRAIGHT;
        controller->straight_evidence_ticks = 0U;
      }
    }
    else
    {
      controller->straight_evidence_ticks = 0U;
    }
  }
  else if(controller->speed_phase ==
          MAGNETIC_FOLLOW_SPEED_CURVE_EMERGENCY)
  {
    if((absolute_offset_mm <= 6.0f) &&
       (absolute_actual_yaw_rate_dps <= 8.0f))
    {
      if(controller->straight_evidence_ticks < UINT16_MAX)
      {
        controller->straight_evidence_ticks++;
      }
      if(controller->straight_evidence_ticks >=
         MAG_FOLLOW_EMERGENCY_RECOVER_TICKS)
      {
        controller->speed_phase = MAGNETIC_FOLLOW_SPEED_CURVE_TRACK;
        controller->straight_evidence_ticks = 0U;
      }
    }
    else
    {
      controller->straight_evidence_ticks = 0U;
    }
  }
}

static float magnetic_follow_adaptive_speed_target_rpm(
    const magnetic_follow_controller_t *controller,
    float requested_run_rpm)
{
  if(controller->speed_phase == MAGNETIC_FOLLOW_SPEED_CURVE_SUSPECT)
  {
    return magnetic_follow_cap_speed(
        requested_run_rpm, MAG_FOLLOW_ADAPTIVE_SUSPECT_CAP_RPM);
  }
  if(controller->speed_phase == MAGNETIC_FOLLOW_SPEED_CURVE_TRACK)
  {
    return magnetic_follow_cap_speed(
        requested_run_rpm, MAG_FOLLOW_ADAPTIVE_CURVE_CAP_RPM);
  }
  if(controller->speed_phase == MAGNETIC_FOLLOW_SPEED_CURVE_EMERGENCY)
  {
    return magnetic_follow_cap_speed(
        requested_run_rpm, MAG_FOLLOW_ADAPTIVE_EMERGENCY_CAP_RPM);
  }
  return requested_run_rpm;
}

void magnetic_follow_controller_reset(
    magnetic_follow_controller_t *controller)
{
  if(controller != NULL)
  {
    memset(controller, 0, sizeof(*controller));
  }
}

uint8_t magnetic_follow_sample_is_trackable(
    const magnetic_follow_sample_t *sample)
{
  return magnetic_follow_sample_is_trackable_for_mode(sample, 0U);
}

uint8_t magnetic_follow_sample_is_trackable_for_mode(
    const magnetic_follow_sample_t *sample,
    uint8_t allow_multi_peak)
{
  uint32_t group_peak = 0UL;
  uint32_t dominant_peak = 0UL;
  uint32_t secondary_peak = 0UL;
  uint32_t total_strength = 0UL;
  uint8_t inactive_run = 0U;
  uint8_t index;

  if((sample == NULL) ||
     (sample->protocol_valid == 0U) ||
     (sample->age_ms > MAG_FOLLOW_MAX_AGE_MS) ||
     ((sample->flags & MAG_FOLLOW_REQUIRED_FLAGS) !=
      MAG_FOLLOW_REQUIRED_FLAGS) ||
     (sample->status == 0U) ||
     (sample->strength < MAG_FOLLOW_MIN_STRENGTH) ||
     (magnetic_follow_abs_i32((int32_t)sample->offset_x10mm) >
      MAG_FOLLOW_MAX_OFFSET_X10MM))
  {
    return 0U;
  }

  /* 单个阈值低谷仍属于同一宽峰；连续两个低谷才分隔独立磁峰。 */
  for(index = 0U; index < 8U; index++)
  {
    if((sample->active_mask & (uint16_t)(1U << index)) != 0U)
    {
      if(inactive_run >= 2U)
      {
        if(group_peak > dominant_peak)
        {
          secondary_peak = dominant_peak;
          dominant_peak = group_peak;
        }
        else if(group_peak > secondary_peak)
        {
          secondary_peak = group_peak;
        }
        group_peak = 0UL;
      }
      inactive_run = 0U;
      if(sample->signal[index] > group_peak)
      {
        group_peak = sample->signal[index];
      }
      total_strength += sample->signal[index];
    }
    else if((group_peak != 0UL) && (inactive_run < 2U))
    {
      inactive_run++;
    }
  }
  if(group_peak > dominant_peak)
  {
    secondary_peak = dominant_peak;
    dominant_peak = group_peak;
  }
  else if(group_peak > secondary_peak)
  {
    secondary_peak = group_peak;
  }

  if(total_strength == 0UL)
  {
    return 0U;
  }
  if((allow_multi_peak == 0U) && (sample->status == 3U) &&
     ((secondary_peak * 100UL) >
      (dominant_peak * MAG_FOLLOW_SECONDARY_MAX_PERCENT)))
  {
    return 0U;
  }
  return 1U;
}

void magnetic_follow_controller_start(
    magnetic_follow_controller_t *controller,
    int16_t target_offset_x10mm,
    uint8_t allow_multi_peak,
    const magnetic_follow_sample_t *sample,
    float requested_run_rpm)
{
  if((controller == NULL) || (sample == NULL))
  {
    return;
  }

  magnetic_follow_controller_reset(controller);
  controller->target_offset_x10mm = target_offset_x10mm;
  controller->allow_multi_peak = (allow_multi_peak != 0U) ? 1U : 0U;
  controller->offset_x10mm = sample->offset_x10mm;
  controller->strength = sample->strength;
  controller->age_ms = sample->age_ms;
  controller->last_sequence = sample->sequence;
  controller->sequence_valid = 1U;
  controller->last_receive_time_ms = sample->receive_time_ms;
  controller->last_offset_error_mm =
      ((float)sample->offset_x10mm - (float)target_offset_x10mm) / 10.0f;
  controller->adaptive_high_speed =
      ((controller->allow_multi_peak != 0U) &&
       (requested_run_rpm >= MAG_FOLLOW_ADAPTIVE_ENABLE_RPM)) ? 1U : 0U;
  if(controller->adaptive_high_speed != 0U)
  {
    /* 高速任务先以已验证的75mm/s捕获磁条，连续居中后再升速。 */
    controller->speed_phase = MAGNETIC_FOLLOW_SPEED_CURVE_TRACK;
    controller->speed_limit_rpm = magnetic_follow_cap_speed(
        requested_run_rpm, MAG_FOLLOW_ADAPTIVE_CURVE_CAP_RPM);
  }
  else
  {
    controller->speed_phase = MAGNETIC_FOLLOW_SPEED_STANDARD;
    controller->speed_limit_rpm = magnetic_follow_minimum_speed_rpm(
        requested_run_rpm);
  }
}

static void magnetic_follow_update_offset_rate(
    magnetic_follow_controller_t *controller,
    const magnetic_follow_sample_t *sample,
    float offset_error_mm)
{
  float offset_rate_filter_alpha;
  float raw_offset_rate_mm_s;
  uint32_t sample_interval_ms;

  if((controller->sequence_valid != 0U) &&
     (sample->sequence != controller->last_sequence))
  {
    sample_interval_ms = sample->receive_time_ms -
                         controller->last_receive_time_ms;
    if((sample_interval_ms >= 5UL) && (sample_interval_ms <= 200UL))
    {
      raw_offset_rate_mm_s =
          (offset_error_mm - controller->last_offset_error_mm) *
          1000.0f / (float)sample_interval_ms;
      offset_rate_filter_alpha =
          (controller->allow_multi_peak != 0U) ?
          MAG_FOLLOW_CURVE_OFFSET_RATE_FILTER_ALPHA :
          MAG_FOLLOW_OFFSET_RATE_FILTER_ALPHA;
      controller->offset_rate_mm_s +=
          offset_rate_filter_alpha *
          (raw_offset_rate_mm_s - controller->offset_rate_mm_s);
    }
    else
    {
      controller->offset_rate_mm_s = 0.0f;
    }
    controller->last_offset_error_mm = offset_error_mm;
    controller->last_sequence = sample->sequence;
    controller->last_receive_time_ms = sample->receive_time_ms;
  }
  else if(controller->sequence_valid == 0U)
  {
    controller->last_offset_error_mm = offset_error_mm;
    controller->last_sequence = sample->sequence;
    controller->last_receive_time_ms = sample->receive_time_ms;
    controller->offset_rate_mm_s = 0.0f;
    controller->sequence_valid = 1U;
  }
}

static void magnetic_follow_update_yaw_target(
    magnetic_follow_controller_t *controller,
    float offset_error_mm)
{
  float controlled_error_mm;
  float proportional_term_dps;
  float rate_term_dps;
  float yaw_rate_limit_dps;

  if(offset_error_mm > MAG_FOLLOW_OFFSET_DEADBAND_MM)
  {
    controlled_error_mm = offset_error_mm -
                          MAG_FOLLOW_OFFSET_DEADBAND_MM;
  }
  else if(offset_error_mm < -MAG_FOLLOW_OFFSET_DEADBAND_MM)
  {
    controlled_error_mm = offset_error_mm +
                          MAG_FOLLOW_OFFSET_DEADBAND_MM;
  }
  else
  {
    controlled_error_mm = 0.0f;
  }

  proportional_term_dps = MAG_FOLLOW_YAW_RATE_KP_DPS_PER_MM *
                          controlled_error_mm;
  rate_term_dps = ((controller->allow_multi_peak != 0U) ?
                   MAG_FOLLOW_CURVE_YAW_RATE_KD_DPS_PER_MM_S :
                   MAG_FOLLOW_YAW_RATE_KD_DPS_PER_MM_S) *
                  controller->offset_rate_mm_s;
  if(controller->allow_multi_peak != 0U)
  {
    /*
     * 弯道只让变化率提前增强同向转弯，不允许0.1 mm级重心噪声把轮差
     * 瞬间翻向。偏差已回到死区时也不保留微分转向。
     */
    rate_term_dps = magnetic_follow_limit(
        rate_term_dps,
        -MAG_FOLLOW_CURVE_RATE_TERM_LIMIT_DPS,
        MAG_FOLLOW_CURVE_RATE_TERM_LIMIT_DPS);
    if(controlled_error_mm == 0.0f)
    {
      rate_term_dps = 0.0f;
    }
    else if((proportional_term_dps > 0.0f) &&
            ((proportional_term_dps + rate_term_dps) <
             (0.5f * proportional_term_dps)))
    {
      rate_term_dps = -0.5f * proportional_term_dps;
    }
    else if((proportional_term_dps < 0.0f) &&
            ((proportional_term_dps + rate_term_dps) >
             (0.5f * proportional_term_dps)))
    {
      rate_term_dps = -0.5f * proportional_term_dps;
    }
  }
  controller->yaw_rate_target_dps = MAG_FOLLOW_TARGET_SIGN *
      (proportional_term_dps + rate_term_dps);
  yaw_rate_limit_dps = (controller->allow_multi_peak != 0U) ?
                       MAG_FOLLOW_CURVE_YAW_RATE_LIMIT_DPS :
                       MAG_FOLLOW_YAW_RATE_LIMIT_DPS;
  controller->yaw_rate_target_dps = magnetic_follow_limit(
      controller->yaw_rate_target_dps,
      -yaw_rate_limit_dps,
      yaw_rate_limit_dps);
}

static void magnetic_follow_update_speed_limit(
    magnetic_follow_controller_t *controller,
    float offset_error_mm,
    float requested_run_rpm)
{
  float absolute_offset_mm;
  float minimum_speed_rpm;
  float speed_limit_target_rpm;
  float speed_limit_step_rpm;

  absolute_offset_mm = magnetic_follow_abs_float(offset_error_mm);
  if(controller->adaptive_high_speed != 0U)
  {
    minimum_speed_rpm = magnetic_follow_cap_speed(
        requested_run_rpm, MAG_FOLLOW_ADAPTIVE_EMERGENCY_CAP_RPM);
    speed_limit_target_rpm =
        magnetic_follow_adaptive_speed_target_rpm(controller,
                                                  requested_run_rpm);
  }
  else
  {
    minimum_speed_rpm = magnetic_follow_minimum_speed_rpm(
        requested_run_rpm);
    if(absolute_offset_mm <= MAG_FOLLOW_FULL_SPEED_OFFSET_MM)
    {
      speed_limit_target_rpm = requested_run_rpm;
    }
    else if(absolute_offset_mm >= MAG_FOLLOW_MIN_SPEED_OFFSET_MM)
    {
      speed_limit_target_rpm = minimum_speed_rpm;
    }
    else
    {
      speed_limit_target_rpm = requested_run_rpm -
          ((requested_run_rpm - minimum_speed_rpm) *
           (absolute_offset_mm - MAG_FOLLOW_FULL_SPEED_OFFSET_MM) /
           (MAG_FOLLOW_MIN_SPEED_OFFSET_MM -
            MAG_FOLLOW_FULL_SPEED_OFFSET_MM));
    }
  }

  if((controller->adaptive_high_speed != 0U) &&
     (speed_limit_target_rpm < controller->speed_limit_rpm))
  {
    /*
     * 高速循迹一旦确认弯道，限速必须在本帧生效。旧的0.75 RPM/帧下降
     * 需要数秒才能从330 mm/s降到转弯速度，实车会在减速前冲出磁条。
     * 电机速度环自身仍负责物理减速；这里只取消错误的软件限速滞后。
     */
    controller->speed_limit_rpm = speed_limit_target_rpm;
  }
  else
  {
    speed_limit_step_rpm = magnetic_follow_limit(
        speed_limit_target_rpm - controller->speed_limit_rpm,
        -MAG_FOLLOW_SPEED_LIMIT_FALL_RPM,
        (controller->adaptive_high_speed != 0U) ?
        MAG_FOLLOW_ADAPTIVE_SPEED_RISE_RPM :
        MAG_FOLLOW_SPEED_LIMIT_RISE_RPM);
    controller->speed_limit_rpm = magnetic_follow_limit(
        controller->speed_limit_rpm + speed_limit_step_rpm,
        minimum_speed_rpm,
        requested_run_rpm);
  }
}

void magnetic_follow_controller_update(
    magnetic_follow_controller_t *controller,
    const magnetic_follow_sample_t *sample,
    float requested_run_rpm,
    float actual_yaw_rate_dps)
{
  float offset_error_mm;

  if((controller == NULL) || (sample == NULL))
  {
    return;
  }

  controller->offset_x10mm = sample->offset_x10mm;
  controller->strength = sample->strength;
  controller->age_ms = sample->age_ms;
  controller->actual_yaw_rate_dps = actual_yaw_rate_dps;
  offset_error_mm = ((float)sample->offset_x10mm -
                     (float)controller->target_offset_x10mm) / 10.0f;

  magnetic_follow_update_offset_rate(controller, sample, offset_error_mm);
  magnetic_follow_update_yaw_target(controller, offset_error_mm);
  magnetic_follow_update_adaptive_speed_phase(controller,
                                               offset_error_mm);
  magnetic_follow_update_speed_limit(controller,
                                     offset_error_mm,
                                     requested_run_rpm);
}

static int16_t magnetic_follow_round_i16(float value)
{
  if(value > 32767.0f)
  {
    return 32767;
  }
  if(value < -32768.0f)
  {
    return -32768;
  }
  return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

uint8_t magnetic_follow_controller_make_reference(
    const magnetic_follow_controller_t *controller,
    float base_rpm,
    float wheel_circumference_mm,
    int8_t travel_direction,
    uint32_t now_ms,
    uint16_t accel_mm_s2,
    uint16_t decel_mm_s2,
    agv_motion_reference_t *reference)
{
  float linear_mm_s;
  float angular_mrad_s;

  if((controller == NULL) || (reference == NULL) ||
     !(base_rpm > 0.0f) ||
     !(wheel_circumference_mm > 0.0f) ||
     ((travel_direction != 1) && (travel_direction != -1)) ||
     (accel_mm_s2 == 0U) || (decel_mm_s2 == 0U))
  {
    return 0U;
  }
  if(base_rpm > controller->speed_limit_rpm)
  {
    base_rpm = controller->speed_limit_rpm;
  }
  linear_mm_s = base_rpm *
                wheel_circumference_mm / 60.0f;
  angular_mrad_s = controller->yaw_rate_target_dps *
                   AGV_MATH_PI_F * 1000.0f / 180.0f;
  memset(reference, 0, sizeof(*reference));
  reference->reference_id = controller->last_sequence;
  reference->produced_ms = now_ms;
  reference->valid_for_ms = MAG_FOLLOW_MAX_AGE_MS;
  reference->linear_mm_s = magnetic_follow_round_i16(
      linear_mm_s * (float)travel_direction);
  reference->angular_mrad_s = magnetic_follow_round_i16(angular_mrad_s);
  reference->accel_mm_s2 = accel_mm_s2;
  reference->decel_mm_s2 = decel_mm_s2;
  reference->source = (uint8_t)AGV_MOTION_SOURCE_MAGNETIC;
  reference->kind = (uint8_t)AGV_MOTION_KIND_BODY_VELOCITY;
  return agv_motion_reference_is_valid(reference);
}
