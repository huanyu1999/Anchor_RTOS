#include "i2c_rx8130ce.h"
#include "main.h"
I2C_HandleTypeDef hi2c2;

/* rx8130ce register definitions */
#define RX8130_REG_SEC		0x10
#define RX8130_REG_MIN		0x11
#define RX8130_REG_HOUR		0x12
#define RX8130_REG_WDAY		0x13
#define RX8130_REG_MDAY		0x14
#define RX8130_REG_MONTH	0x15
#define RX8130_REG_YEAR		0x16

#define RX8130_REG_ALMIN	0x17
#define RX8130_REG_ALHOUR	0x18
#define RX8130_REG_ALWDAY	0x19
#define RX8130_REG_TCOUNT0	0x1A
#define RX8130_REG_TCOUNT1	0x1B
#define RX8130_REG_EXT		0x1C
#define RX8130_REG_FLAG		0x1D
#define RX8130_REG_CTRL0	0x1E
#define RX8130_REG_CTRL1	0x1F

#define RX8130_REG_END		0x23

/* Extension Register (1Ch) bit positions */ 
#define RX8130_BIT_EXT_TSEL		(7 << 0)
#define RX8130_BIT_EXT_WADA		(1 << 3)
#define RX8130_BIT_EXT_TE		(1 << 4)
#define RX8130_BIT_EXT_USEL		(1 << 5)
#define RX8130_BIT_EXT_FSEL		(3 << 6)

/* Flag Register (1Dh) bit positions */ 
#define RX8130_BIT_FLAG_VLF		(1 << 1)
#define RX8130_BIT_FLAG_AF		(1 << 3)
#define RX8130_BIT_FLAG_TF		(1 << 4)
#define RX8130_BIT_FLAG_UF		(1 << 5)

/* Control 0 Register (1Eh) bit positions */ 
#define RX8130_BIT_CTRL_TSTP	(1 << 2)
#define RX8130_BIT_CTRL_AIE		(1 << 3)
#define RX8130_BIT_CTRL_TIE		(1 << 4)
#define RX8130_BIT_CTRL_UIE		(1 << 5)
#define RX8130_BIT_CTRL_STOP	(1 << 6)
#define RX8130_BIT_CTRL_TEST	(1 << 7)

/* Control 1 Register (1Fh) bit positions */
#define RX8130_BIT_CTRL_INIEN   (1 << 4)
#define RX8130_BIT_CTRL_CHGEN   (1 << 5)

/* define rx8130ce device address */
#define RX8130_ADDRESS          0x3AA

#define MCU_ADDRESS             0x2AA


void rx8130ce_I2cInit(void)
{
    mx_rx8130ce_i2c2_init();
}

HAL_StatusTypeDef rx8130ce_setDateTime(volatile rx8130ce_time_t *time)
{
    uint8_t ctrl;
    
    rx8130ce_read(RX8130_ADDRESS, RX8130_REG_CTRL1, &ctrl, 1);      // enable battery switch
    ctrl = ctrl | RX8130_BIT_CTRL_INIEN;
    rx8130ce_write(RX8130_ADDRESS, RX8130_REG_CTRL1, ctrl, 1);
    
    rx8130ce_read(RX8130_ADDRESS, RX8130_REG_CTRL0, &ctrl, 1);      //set STOP bit before changing clock/calendar
    ctrl = ctrl | RX8130_BIT_CTRL_STOP;
    rx8130ce_write(RX8130_ADDRESS, RX8130_REG_CTRL0, ctrl, 1);
    
    
    if(HAL_I2C_IsDeviceReady(&hi2c2, RX8130_ADDRESS, 2, 5) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    uint8_t tempData[7] = { rx8130ce_Bin2Bcd(time->seconds), rx8130ce_Bin2Bcd(time->minutes),
                            rx8130ce_Bin2Bcd(time->hours), rx8130ce_Bin2Bcd(time->week),
                            rx8130ce_Bin2Bcd(time->day), rx8130ce_Bin2Bcd(time->month),
                            rx8130ce_Bin2Bcd(time->year) };
    
    /* 写入设置时间 */
    if (rx8130ce_write(RX8130_ADDRESS, RX8130_REG_SEC, *tempData, 7) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    rx8130ce_read(RX8130_ADDRESS, RX8130_REG_CTRL0, &ctrl, 1);      // clear STOP bit after changing clock/calendar
    ctrl = ctrl & ~RX8130_BIT_CTRL_STOP;
    rx8130ce_write(RX8130_ADDRESS, RX8130_REG_CTRL0, ctrl, 1);
    
    return HAL_OK;
}

void rx8130ce_getDateTime(volatile rx8130ce_time_t *time)
{
    uint8_t temp_data[7];
    rx8130ce_read(RX8130_ADDRESS, RX8130_REG_SEC, temp_data, 7);
    time->year  = rx8130ce_Bcd2Bin(temp_data[6]);
    time->month = rx8130ce_Bcd2Bin(temp_data[5]);
    time->day   = rx8130ce_Bcd2Bin(temp_data[4]);
    time->hours = rx8130ce_Bcd2Bin(temp_data[2]);
    time->minutes = rx8130ce_Bcd2Bin(temp_data[1]);
    time->seconds = rx8130ce_Bcd2Bin(temp_data[0]);

    time->week = 0;
    while ((temp_data[3] & 1) == 0)
    {
        time->week++;
        temp_data[3] >>= 1;
    }
}

/**
  * @brief  send a byte to RTC chip register
  * @param  
  * @retval None
  */
HAL_StatusTypeDef rx8130ce_write(uint16_t devAddr, uint8_t memAddr, uint8_t data, uint16_t length)
{
    uint8_t tempData[] = { memAddr, data };
    if(HAL_I2C_Master_Transmit(&hi2c2, devAddr, tempData, (length + 1), 1000) != HAL_OK)
    {
        return HAL_ERROR;       // 返回错误，需要重启芯片
    }
    
    return HAL_OK;
}

/**
  * @brief  read a byte from RTC chip register
  * @param  
  * @retval None
  */
HAL_StatusTypeDef rx8130ce_read(uint16_t devAddr, uint8_t memAddr, uint8_t *data, uint16_t length)
{
    uint8_t tempData[] = { memAddr };
    if(HAL_I2C_Master_Transmit(&hi2c2, devAddr, tempData, length, 1000) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    uint8_t tempData1[1];
    if(HAL_I2C_Master_Receive(&hi2c2, devAddr, tempData1, length, 1000) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    *data = tempData1[0];
    
    return HAL_OK;
}

uint8_t rx8130ce_Bcd2Bin(uint8_t bcd)
{
    uint8_t dec = 10 * (bcd >> 4);
    dec += bcd & 0xF;
    return dec;
}

uint8_t rx8130ce_Bin2Bcd(uint8_t bin)
{
    uint8_t low = 0;
    uint8_t high = 0;
    
    high = bin / 10;
    low = bin - (high * 10);
    
    return high << 4 | low;
}


/******************************************************I2C HAL******************************************************/
void mx_rx8130ce_i2c2_init(void)
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 400000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    hi2c2.Init.OwnAddress1 = MCU_ADDRESS;
    hi2c2.Init.OwnAddress2 = 0;

    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(i2cHandle->Instance == I2C2)
    {
        /* USER CODE BEGIN I2C1_MspInit 0 */

        /* USER CODE END I2C1_MspInit 0 */

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**I2C1 GPIO Configuration
        PB10     ------> I2C1_SCL
        PB11     ------> I2C1_SDA
        */
        GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        
        /* I2C1 clock enable */
        __HAL_RCC_I2C2_CLK_ENABLE();
        /* USER CODE BEGIN I2C1_MspInit 1 */

        /* USER CODE END I2C1_MspInit 1 */
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{
    if(i2cHandle->Instance==I2C2)
    {
        /* USER CODE BEGIN I2C1_MspDeInit 0 */

        /* USER CODE END I2C1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_I2C2_CLK_DISABLE();

        /**I2C1 GPIO Configuration
        PB10     ------> I2C1_SCL
        PB11     ------> I2C1_SDA
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

        /* USER CODE BEGIN I2C1_MspDeInit 1 */

        /* USER CODE END I2C1_MspDeInit 1 */
    }
}
