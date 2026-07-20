#ifndef TOF_I2C3_H
#define TOF_I2C3_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化连接两个VL53L0X的I2C3总线。 */
void tof_i2c3_init(void);

/* 读取I2C空闲时的SCL、SDA电平和最近一次底层错误码。 */
void tof_i2c3_get_diagnostics(uint8_t *scl_level,
                              uint8_t *sda_level,
                              int32_t *last_error);

/* 向指定设备寄存器连续写入数据，地址使用ST API约定的8位格式。 */
int32_t tof_i2c3_write(uint8_t address,
                       uint8_t reg,
                       const uint8_t *data,
                       uint32_t count);

/* 从指定设备寄存器连续读取数据，地址使用ST API约定的8位格式。 */
int32_t tof_i2c3_read(uint8_t address,
                      uint8_t reg,
                      uint8_t *data,
                      uint32_t count);

#ifdef __cplusplus
}
#endif

#endif
