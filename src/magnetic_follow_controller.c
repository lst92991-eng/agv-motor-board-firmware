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
#define MAG_FOLLOW_YAW_RATE_INNER_KP_RPM_PER_DPS               0.35f
#define MAG_FOLLOW_CURVE_YAW_RATE_INNER_KP_RPM_PER_DPS         0.45f
#define MAG_FOLLOW_CAPTURE_CORRECTION_LIMIT_RPM                6.0f
#define MAG_FOLLOW_TRACK_CORRECTION_LIMIT_RPM                  8.0f
#define MAG_FOLLOW_CURVE_CORRECTION_LIMIT_RPM                 11.0f
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
#define MAG_FOLLOW_SUSPECT_OFFSET_MM                           2.5f
#define MAG_FOLLOW_SUSPECT_YAW_RATE_DPS                        3.0f
#define MAG_FOLLOW_CONFIRM_OFFSET_MM                           4.0f
#define MAG_FOLLOW_CONFIRM_YAW_RATE_DPS                        5.0f
#define MAG_FOLLOW_EMERGENCY_OFFSET_MM                         8.0f
#define MAG_FOLLOW_EMERGENCY_YAW_RATE_DPS                     10.0f
#define MAG_FOLLOW_EXIT_OFFSET_MM                              2.5f
#define MAG_FOLLOW_EXIT_YAW_RATE_DPS                           3.5f
#define MAG_FOLLOW_CONFIRM_TICKS                                5U
#define MAG_FOLLOW_SUSPECT_TIMEOUT_TICKS                       20U
#define MAG_FOLLOW_CURVE_EXIT_TICKS                            30U
#define MAG_FOLLOW_EMERGENCY_RECOVER_TICKS                     20U
#define MAG_FOLLOW_TARGET_SIGN                                1.0f
#define MAG_FOLLOW_PI                                         3.14159265f

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
  float absolute_yaw_rate_dps = magnetic_follow_abs_float(
      controller->yaw_rate_target_dps);
  uint8_t suspect;
  uint8_t confirmed;
  uint8_t emergency;
  uint8_t straight;

  if(controller->adaptive_high_speed == 0U)
  {
    return;
  }

  suspect = ((absolute_offset_mm >= MAG_FOLLOW_SUSPECT_OFFSET_MM) ||
             (absolute_yaw_rate_dps >=
              MAG_FOLLOW_SUSPECT_YAW_RATE_DPS)) ? 1U : 0U;
  confirmed = ((absolute_offset_mm >= MAG_FOLLOW_CONFIRM_OFFSET_MM) ||
               (absolute_yaw_rate_dps >=
                MAG_FOLLOW_CONFIRM_YAW_RATE_DPS)) ? 1U : 0U;
  emergency = ((absolute_offset_mm >= MAG_FOLLOW_EMERGENCY_OFFSET_MM) ||
               ((absolute_offset_mm >= 6.0f) &&
                (absolute_yaw_rate_dps >=
                 MAG_FOLLOW_EMERGENCY_YAW_RATE_DPS))) ? 1U : 0U;
  /* 0.1 mm量化在10 ms采样下会制造很大的瞬时偏移变化率，不能把它作为
   * 直线恢复的硬门槛；用偏移和实际所需转向量连续确认即可。
   */
  straight = ((absolute_offset_mm <= MAG_FOLLOW_EXIT_OFFSET_MM) &&
              (absolute_yaw_rate_dps <=
               MAG_FOLLOW_EXIT_YAW_RATE_DPS)) ? 1U : 0U;

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
      controller->speed_phase = MAGNETIC_FOLLOW_SPEED_CURVE_SUSPECT;
      controller->curve_evidence_ticks = 1U;
      controller->straight_evidence_ticks = 0U;
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
       (absolute_yaw_rate_dps <= 8.0f))
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
    controller->speed_phase = MAGNETIC_FOLLOW_SPEED_FAST_STRAIGHT;
    controller->speed_limit_rpm = requested_run_rpm;
  }
  else
  {
    controller->speed_phase = MAGNETIC_FOLLOW_SPEED_STANDARD;
    controller->speed_limit_rpm = magnetic_follow_minimum_speed_rpm(
        requested_run_rpm);
  }
}

void magnetic_follow_controller_update(
    magnetic_follow_controller_t *controller,
    const magnetic_follow_sample_t *sample,
    float requested_run_rpm)
{
  float offset_error_mm;
  float controlled_error_mm;
  float absolute_offset_mm;
  float minimum_speed_rpm;
  float offset_rate_filter_alpha;
  float proportional_term_dps;
  float rate_term_dps;
  float raw_offset_rate_mm_s;
  float speed_limit_target_rpm;
  float speed_limit_step_rpm;
  float yaw_rate_limit_dps;
  uint32_t sample_interval_ms;

  if((controller == NULL) || (sample == NULL))
  {
    return;
  }

  controller->offset_x10mm = sample->offset_x10mm;
  controller->strength = sample->strength;
  controller->age_ms = sample->age_ms;
  offset_error_mm = ((float)sample->offset_x10mm -
                     (float)controller->target_offset_x10mm) / 10.0f;

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

  magnetic_follow_update_adaptive_speed_phase(controller,
                                               offset_error_mm);

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

float magnetic_follow_controller_wheel_correction_rpm(
    const magnetic_follow_controller_t *controller,
    float actual_yaw_rate_dps,
    float drive_track_mm,
    float wheel_circumference_mm)
{
  float feedforward_rpm;
  float inner_kp_rpm_per_dps;

  if((controller == NULL) || (drive_track_mm <= 0.0f) ||
     (wheel_circumference_mm <= 0.0f))
  {
    return 0.0f;
  }
  feedforward_rpm = controller->yaw_rate_target_dps * MAG_FOLLOW_PI *
                    drive_track_mm * 30.0f /
                    (180.0f * wheel_circumference_mm);
  inner_kp_rpm_per_dps = (controller->allow_multi_peak != 0U) ?
                         MAG_FOLLOW_CURVE_YAW_RATE_INNER_KP_RPM_PER_DPS :
                         MAG_FOLLOW_YAW_RATE_INNER_KP_RPM_PER_DPS;
  return feedforward_rpm +
         (inner_kp_rpm_per_dps *
          (controller->yaw_rate_target_dps - actual_yaw_rate_dps));
}

float magnetic_follow_controller_correction_limit_rpm(
    uint8_t capture_phase,
    uint8_t allow_multi_peak)
{
  if(capture_phase != 0U)
  {
    return MAG_FOLLOW_CAPTURE_CORRECTION_LIMIT_RPM;
  }
  return (allow_multi_peak != 0U) ?
         MAG_FOLLOW_CURVE_CORRECTION_LIMIT_RPM :
         MAG_FOLLOW_TRACK_CORRECTION_LIMIT_RPM;
}
