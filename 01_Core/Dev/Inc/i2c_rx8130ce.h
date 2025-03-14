#ifndef __RX8130CE_H_
#define __RX8130CE_H_

#include "main.h"

#define RX8130CE_RST_PORT   GPIOB
#define RX8130CE_RST_PIN    GPIO_PIN_8

#define RX8130CE_INT_PORT   GPIOB
#define RX8130CE_INT_PIN    GPIO_PIN_7

typedef struct {
    uint8_t scl_port;
    uint8_t scl_pin;
    uint8_t sda_port;
    uint8_t sda_pin;
    uint8_t i2c_mode;
    uint8_t i2c_clockSpeed;
    uint8_t i2c_address;
} i2c_device_t;

typedef enum {
    i2c1,
    i2c2,
    NUM_I2C_CHANNELS
} i2c_channel_e;

/* RX8130CE Structure for date/time */
typedef struct {
  uint8_t seconds; /*!< Seconds parameter, from 00 to 59 */
  uint8_t minutes; /*!< Minutes parameter, from 00 to 59 */
  uint8_t hours;   /*!< Hours parameter, 24Hour mode, 00 to 23 */
  uint8_t week;    /*!< Day in a week, from 0 to 6 (Sunday to Saturday) */
  uint8_t day;     /*!< Day in a month, 1 to 31 */
  uint8_t month;   /*!< Month in a year, 1 to 12 */
  uint8_t year;    /*!< Year parameter, 00 to 99, 00 is 2000 and 99 is 2099 */
} rx8130ce_time_t;

HAL_StatusTypeDef rx8130ce_write(uint16_t devAddr, uint8_t memAddr, uint8_t data, uint16_t length);
HAL_StatusTypeDef rx8130ce_read(uint16_t devAddr, uint8_t memAddr, uint8_t *data, uint16_t length);
uint8_t rx8130ce_Bcd2Bin(uint8_t bcd);
uint8_t rx8130ce_Bin2Bcd(uint8_t bin);

/******************************************************I2C HAL******************************************************/
void mx_rx8130ce_i2c2_init(void);
void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle);
#endif
