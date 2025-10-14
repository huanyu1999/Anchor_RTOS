#ifndef __DRV_EMMMC_H__
#define __DRV_EMMMC_H__

#include "stm32f4xx.h"   
#include "stm32f4xx_hal_mmc.h"

#define EMMC_OK        ((uint8_t)0x00)
#define EMMC_ERROR     ((uint8_t)0x01)

#define EMMC_TRANSFER_OK       (uint8_t)(0x00)
#define EMMC_TRANSFER_BUSY     (uint8_t)(0x01)


#define EMMC_D0_PORT    GPIOC
#define EMMC_D0_PIN     GPIO_PIN_8
#define EMMC_D1_PORT    GPIOC
#define EMMC_D1_PIN     GPIO_PIN_9
#define EMMC_D2_PORT    GPIOC
#define EMMC_D2_PIN     GPIO_PIN_10
#define EMMC_D3_PORT    GPIOC
#define EMMC_D3_PIN     GPIO_PIN_11
#define EMMC_D4_PORT    GPIOB
#define EMMC_D4_PIN     GPIO_PIN_8
#define EMMC_D5_PORT    GPIOB
#define EMMC_D5_PIN     GPIO_PIN_9
#define EMMC_D6_PORT    GPIOC
#define EMMC_D6_PIN     GPIO_PIN_6
#define EMMC_D7_PORT    GPIOC
#define EMMC_D7_PIN     GPIO_PIN_7
#define EMMC_CLK_PORT   GPIOC
#define EMMC_CLK_PIN    GPIO_PIN_12
#define EMMC_CMD_PORT   GPIOD
#define EMMC_CMD_PIN    GPIO_PIN_2

uint8_t drv_emmcInit(void);
void HAL_MMC_MspInit(MMC_HandleTypeDef *hmmc);
uint8_t drv_emmcGetInfo(HAL_MMC_CardInfoTypeDef* emmcInfo, HAL_MMC_CardCIDTypeDef* emmcCID);
uint8_t drv_emmcReadBlocks(uint32_t *pData, uint32_t readAddr, uint32_t numOfBlocks);
uint8_t drv_emmcWriteBlocks(uint32_t *pData, uint32_t writeAddr, uint32_t numOfBlocks);
uint8_t drv_emmcErase(uint32_t startAddr, uint32_t endAddr);
uint8_t drv_emmcGetState(void);
#endif
