/**
 * @file adc_monitor.h
 * @brief 电气监督使用的三通道ADC原始采样接口。
 */

#ifndef ADC_MONITOR_H
#define ADC_MONITOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ADC监测通道：PA3母线、PA4左电流、PA5右电流。 */
typedef enum
{
  ADC_MONITOR_PA3 = 0,
  ADC_MONITOR_PA4,
  ADC_MONITOR_PA5,
  ADC_MONITOR_CHANNEL_COUNT
} adc_monitor_channel_t;

/** 初始化ADC1以及PA3、PA4、PA5；返回0表示校准或就绪等待超时。 */
uint8_t adc_monitor_init(void);

/** 依次采集三个通道；返回0时调用方不得使用本周期任何通道。 */
uint8_t adc_monitor_update(void);

/** 读取最近一次写入的12位ADC原始值；越界通道返回0。 */
uint16_t adc_monitor_get_raw(adc_monitor_channel_t channel);

#ifdef __cplusplus
}
#endif

#endif
