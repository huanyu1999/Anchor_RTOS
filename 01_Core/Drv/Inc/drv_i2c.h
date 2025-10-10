#ifndef __DRV_I2C_H__
#define __DRV_I2C_H__

#include "gpio.h"

#define I2C1_SDA_PORT      GPIOB
#define I2C1_SDA_PIN       GPIO_PIN_7

#define I2C1_SCL_PORT      GPIOB
#define I2C1_SCL_PIN       GPIO_PIN_6

typedef enum {
    STANDARD_MODE,
    FAST_MODE,
    FAST_MODE_Plus,
    MODE_NUMS
} i2c_mode_e;

typedef struct {
    uint8_t bus_mode;         // I2C总线模式
    uint32_t master_id;       // 主机地址
    uint32_t clk_speed;       // I2C时钟速率
} i2c_config_t ;

void drv_i2cInit(void);
void drv_i2cRead(uint8_t addr, uint8_t* buf, uint16_t size);
void drv_i2cWrite(uint8_t addr, uint8_t* data, uint16_t size);
void drv_i2cMemRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint16_t read_size);
void drv_i2cMemWrite(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t write_size);

void drv_i2cError(void);


#endif
