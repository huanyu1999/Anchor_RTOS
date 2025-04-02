#ifndef __DRV_SDIO_H__
#define __DRV_SDIO_H__

#include "main.h"

#include "stm32f4xx_hal_sd.h"

// #define SDIO_ERROR 0x00
// #define SDIO_OK    0x01

#define   MSD_OK        ((uint8_t)0x00)
#define   MSD_ERROR     ((uint8_t)0x01)

#define SD_TRANSFER_OK   (uint8_t)(0x00)
#define SD_TRANSFER_BUSY (uint8_t)(0x01)

#define SDIO_PRESENT        ((uint8_t)0x01)
#define SDIO_NOT_PRESENT    ((uint8_t)0x00)

#define SD_CardInfo         HAL_SD_CardInfoTypeDef
#define SD_CardCID          HAL_SD_CardCIDTypedef

#define SDIO_CMD_PORT   GPIOD
#define SDIO_CMD_PIN    GPIO_PIN_2
#define SDIO_D0_PORT    GPIOC
#define SDIO_D0_PIN     GPIO_PIN_8
#define SDIO_D1_PORT    GPIOC
#define SDIO_D1_PIN     GPIO_PIN_9
#define SDIO_D2_PORT    GPIOC
#define SDIO_D2_PIN     GPIO_PIN_10
#define SDIO_D3_PORT    GPIOC
#define SDIO_D3_PIN     GPIO_PIN_11
#define SDIO_CLK_PORT   GPIOC
#define SDIO_CLK_PIN    GPIO_PIN_12
#define SDIO_DECT_PORT  GPIOA
#define SDIO_DECT_PIN   GPIO_PIN_15

uint8_t drv_sdioInit(void);
void drv_sdDetectInit(void);
uint8_t drv_sdIsDetect(void);
void HAL_SD_MspInit(SD_HandleTypeDef *hsd);
uint8_t drv_sdioGetCardInfo(HAL_SD_CardInfoTypeDef* cardInfo, HAL_SD_CardCIDTypedef* cardCID);
uint8_t drv_sdioGetCardState(void);
uint8_t drv_sdioReadBlocks(uint32_t *pData, uint32_t readAddr, uint32_t numOfBlocks, uint32_t timeOut);
uint8_t drv_sdioWriteBlocks(uint32_t *pData, uint32_t writeAddr, uint32_t numOfBlocks);
uint8_t drv_sdioReadBlocks_Dma(uint32_t *pData, uint32_t readAddr, uint32_t numOfBlocks);
uint8_t drv_sdioWriteBlocks_Dma(uint32_t *pData, uint32_t writeAddr, uint32_t numOfBlocks);
uint8_t drv_sdioCardErase(uint32_t startAddr, uint32_t endAddr);
void task7_sdCardReadTest(void *argument);

void dev_SD_WriteCpltCallback(void);
void dev_SD_ReadCpltCallback(void);
#endif
