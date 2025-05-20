#include "drv_i2c.h"
#include "stm32f4xx_hal_i2c.h"

#define I2C_MASTER_ADDRESS 0xFE       // 定义从设备地址 
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
    HAL_I2C_Master_Receive(&i2c_handle, addr, buf, size, 0x1000);
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
    HAL_I2C_Mem_Read(&i2c_handle, addr, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, buf, read_size, 0x1000);
    if (status != HAL_OK)
    {
        drv_i2cInit();                  // 读写不成功，就重新初始化整个I2C接口
    }
}

void drv_i2cMemWrite(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t write_size)
{
    HAL_StatusTypeDef status = HAL_OK;  
    status = HAL_I2C_Mem_Write(&i2c_handle, addr, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, data, write_size, 0x1000);
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
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStructure.Alternate = GPIO_AF4_I2C1;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin = I2C2_SDA_PIN | I2C2_SCL_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    /* 是否需要使能中断 */
}
