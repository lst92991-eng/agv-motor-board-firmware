#ifndef ADC_MONITOR_H
#define ADC_MONITOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ADC监测通道，与开发板上的PA3、PA4和PA5一一对应。 */
typedef enum
{
  ADC_MONITOR_PA3 = 0,
  ADC_MONITOR_PA4,
  ADC_MONITOR_PA5,
  ADC_MONITOR_CHANNEL_COUNT
} adc_monitor_channel_t;

/* 初始化ADC1以及PA3、PA4、PA5；返回0表示就绪或校准等待超时。 */
uint8_t adc_monitor_init(void);

/* 依次采集三个通道，返回1表示成功，返回0表示ADC等待超时。 */
uint8_t adc_monitor_update(void);

/* 读取最近一次采集的12位ADC原始值。 */
uint16_t adc_monitor_get_raw(adc_monitor_channel_t channel);

#ifdef __cplusplus
}
#endif

#endif
