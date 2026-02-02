#include "drv_i2c.h"
#include "stm32f4xx_hal_i2c.h"
#include "cmsis_os.h"

#define I2C_MASTER_ADDRESS 0xFE       // 定义从设备地址 

// extern osSemaphoreId_t rx8130WriteSem;
// extern osSemaphoreId_t rx8130ReadSem;

I2C_HandleTypeDef i2c_handle;

const i2c_config_t i2c1_config = {
    .bus_mode = FAST_MODE, .clk_speed = 400000, .master_id = I2C_MASTER_ADDRESS,
};

void drv_i2cInit(void)
{
    i2c_handle.Instance = I2C1;
    i2c_handle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    i2c_handle.Init.ClockSpeed      = 400000;
    i2c_handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c_handle.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
    i2c_handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c_handle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    i2c_handle.Init.OwnAddress1     = I2C_MASTER_ADDRESS;           // 作为从设备时的值，不会影响作为主设备的读写
    i2c_handle.Init.OwnAddress2     = 0xFE;

    HAL_I2C_Init(&i2c_handle);
}

void drv_i2cRead(uint8_t addr, uint8_t* buf, uint16_t size)
{
    HAL_StatusTypeDef status = HAL_OK;  
    // HAL_I2C_Master_Receive(&i2c_handle, addr, buf, size, 0x1000);
    status = HAL_I2C_Master_Receive_DMA(&i2c_handle, addr, buf, size);
    if (status != HAL_OK)
    {
        drv_i2cInit();                  // 读写不成功，就重新初始化整个I2C接口
    }
}

void drv_i2cWrite(uint8_t addr, uint8_t* data, uint16_t size)
{
    HAL_StatusTypeDef status = HAL_OK;  
    HAL_I2C_Master_Transmit(&i2c_handle, addr, data, size, 0x1000);
    if (status != HAL_OK)
    {
        drv_i2cInit();                  // 读写不成功，就重新初始化整个I2C接口
    }
}

void drv_i2cMemRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint16_t read_size)
{
    HAL_StatusTypeDef status = HAL_OK;
    // osSemaphoreAcquire(rx8130ReadSem, osWaitForever);  
    // status = HAL_I2C_Mem_Read_DMA(&i2c_handle, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, read_size);
    status = HAL_I2C_Mem_Read(&i2c_handle, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, read_size, 1000);
    if (status != HAL_OK)
    {
        drv_i2cInit();                  // 读写不成功，就重新初始化整个I2C接口
    }
}

void drv_i2cMemWrite(uint8_t addr, uint8_t reg, uint8_t* buf, uint16_t write_size)
{
    HAL_StatusTypeDef status = HAL_OK;
    // osSemaphoreAcquire(rx8130WriteSem, osWaitForever);  
    // status = HAL_I2C_Mem_Write_DMA(&i2c_handle, addr, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, data, write_size);
    status = HAL_I2C_Mem_Write(&i2c_handle, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, write_size, 1000);
    if (status != HAL_OK)
    {
        drv_i2cInit();                  // 读写不成功，就重新初始化整个I2C接口
    }
}

void drv_i2cError(void)
{
    HAL_I2C_DeInit(&i2c_handle);
    drv_i2cInit();
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    static DMA_HandleTypeDef hdma_tx;
    static DMA_HandleTypeDef hdma_rx;

    GPIO_InitTypeDef GPIO_InitStructure = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStructure.Alternate = GPIO_AF4_I2C1;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin = I2C1_SDA_PIN | I2C1_SCL_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    // __HAL_RCC_DMA1_CLK_ENABLE();

    // hdma_tx.Instance = DMA1_Stream7;
    // hdma_tx.Init.Channel = DMA_CHANNEL_1;
    // hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    // hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    // hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
    // hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    // hdma_tx.Init.MemDataAlignment = DMA_PDATAALIGN_BYTE;
    // hdma_tx.Init.Mode = DMA_NORMAL;
    // hdma_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    // hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    // hdma_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    // hdma_tx.Init.MemBurst = DMA_MBURST_INC4;
    // hdma_tx.Init.PeriphBurst = DMA_PBURST_INC4;
    // HAL_DMA_Init(&hdma_tx);
    // __HAL_LINKDMA(hi2c, hdmatx, hdma_tx);

    // hdma_rx.Instance = DMA1_Stream0;
    // hdma_rx.Init.Channel = DMA_CHANNEL_1;
    // hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    // hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    // hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
    // hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    // hdma_rx.Init.MemDataAlignment = DMA_PDATAALIGN_BYTE;
    // hdma_rx.Init.Mode = DMA_NORMAL;
    // hdma_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    // hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    // hdma_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    // hdma_rx.Init.MemBurst = DMA_MBURST_INC4;
    // hdma_rx.Init.PeriphBurst = DMA_PBURST_INC4;
    // HAL_DMA_Init(&hdma_rx);
    // __HAL_LINKDMA(hi2c, hdmarx, hdma_rx);

    // /* 使能DMA中断 */
    // HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 7, 0);
    // HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

    // HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 7, 0);
    // HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

    // /* 使能I2C中断 */
    // HAL_NVIC_SetPriority(I2C1_EV_IRQn, 7, 0);
    // HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    // HAL_NVIC_SetPriority(I2C1_ER_IRQn, 7, 0);
    // HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

// void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
// {
//     UNUSED(hi2c);
//     osSemaphoreRelease(rx8130WriteSem);
// }

// void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
// {
//     UNUSED(hi2c);
//     osSemaphoreRelease(rx8130ReadSem);
// }
