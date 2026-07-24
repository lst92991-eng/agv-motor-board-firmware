/**
  *****************************************************************************
  * @file     bumper_test.c
  * @brief    四路防撞条独立输入测试
  *
  * 本测试不初始化电机。串口首先输出中断后的原始GPIO电平，再在电平连续
  * 稳定10ms后输出确认值，用于判断防撞条真实有效电平和机械抖动情况。
  *****************************************************************************
  */

#include "bumper_test.h"

#include <stdint.h>
#include <stdio.h>

#include "at32f45x_board.h"
#include "bumper_input.h"

#define BUMPER_TEST_LOOP_PERIOD_MS       1U
#define BUMPER_TEST_STABLE_TIME_MS      10U

static uint8_t bumper_test_candidate_mask;
static uint8_t bumper_test_stable_mask;
static uint8_t bumper_test_stable_ticks;

static uint8_t bumper_test_level(uint8_t raw_mask, uint8_t direction_mask)
{
  return ((raw_mask & direction_mask) != 0U) ? 1U : 0U;
}

static void bumper_test_print(const char *event,
                              uint8_t raw_mask,
                              uint8_t changed_mask)
{
  printf("BUMPER_%s,front=%u,rear=%u,left=%u,right=%u,raw=0x%02X,changed=0x%02X\r\n",
         event,
         bumper_test_level(raw_mask, BUMPER_INPUT_FRONT_MASK),
         bumper_test_level(raw_mask, BUMPER_INPUT_REAR_MASK),
         bumper_test_level(raw_mask, BUMPER_INPUT_LEFT_MASK),
         bumper_test_level(raw_mask, BUMPER_INPUT_RIGHT_MASK),
         raw_mask,
         changed_mask);
}

void bumper_test_init(void)
{
  bumper_input_init();
  bumper_test_candidate_mask = bumper_input_read_raw_mask();
  bumper_test_stable_mask = bumper_test_candidate_mask;
  bumper_test_stable_ticks = 0U;
  (void)bumper_input_take_changed_mask();

  printf("\r\nBUMPER_TEST,NO_MOTOR,RAW_LEVEL_ONLY\r\n");
  printf("BUMPER_MAP,front=PC15,rear=PC14,left=PC13,right=PB9\r\n");
  printf("BUMPER_NOTE,1=HIGH,0=LOW,active_level_not_assumed\r\n");
  bumper_test_print("INITIAL", bumper_test_stable_mask, 0U);
}

void bumper_test_loop(void)
{
  uint8_t changed_mask = bumper_input_take_changed_mask();
  uint8_t raw_mask = bumper_input_read_raw_mask();

  if((changed_mask != 0U) || (raw_mask != bumper_test_candidate_mask))
  {
    bumper_test_candidate_mask = raw_mask;
    bumper_test_stable_ticks = 0U;
    bumper_test_print("EDGE", raw_mask, changed_mask);
  }
  else if(bumper_test_candidate_mask != bumper_test_stable_mask)
  {
    if(bumper_test_stable_ticks < BUMPER_TEST_STABLE_TIME_MS)
    {
      bumper_test_stable_ticks++;
    }
    if(bumper_test_stable_ticks >= BUMPER_TEST_STABLE_TIME_MS)
    {
      bumper_test_stable_mask = bumper_test_candidate_mask;
      bumper_test_print("STABLE", bumper_test_stable_mask, 0U);
    }
  }

  delay_ms(BUMPER_TEST_LOOP_PERIOD_MS);
}
