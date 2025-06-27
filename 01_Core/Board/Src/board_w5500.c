
#include "spi.h"
#include "gpio.h"
#include "board_w5500.h"

#include "wizchip_conf.h"

extern SPI_HandleTypeDef hspi2;

static gpio_config_t w5500_boardParam[] = {
    { .gpio_port = W5500_RST_PORT, .gpio_pin = W5500_RST_PIN, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_NOPULL, .gpio_speed = GPIO_SPEED_FAST },

    { .gpio_port = W5500_INT_PORT, .gpio_pin = W5500_INT_PIN, .gpio_mode = GPIO_MODE_IT_RISING, .gpio_pull = GPIO_NOPULL, .gpio_speed = GPIO_SPEED_FAST },
};

static exti_irq_t w5500_interruptConfig[] = {
    {.irq_name = W5500_EXTI_IRQ, .irq_priority = 8},
};

void board_w5500Init(void)
{
    board_w5500InterfaceInit();
    board_w5500ChipDeSel();
    board_w5500RstIntInit();
}

void board_w5500InterfaceInit(void)
{
    MX_SPI2_Init();
}

void board_w5500RstIntInit(void)
{
    drv_gpioInit(&w5500_boardParam[W5500_RST]);
    drv_gpioInit(&w5500_boardParam[W5500_INT]);
}

void board_w5500Reset(void)
{
    /* (Active low) RESET should be held low at least 500 us for W5500 */
    board_w5500RstHigh();
    HAL_Delay(10);
    board_w5500RstLow();
    HAL_Delay(10);
    board_w5500RstHigh();
    HAL_Delay(10);
}

void board_w5500ChipSel(void)
{
    drv_spiCSCtrl(&hspi2, GPIO_PIN_RESET);
}

void board_w5500ChipDeSel(void)
{
    drv_spiCSCtrl(&hspi2, GPIO_PIN_SET);
}

void board_w5500RstHigh(void)
{
    drv_gpioSetLevel(&w5500_boardParam[W5500_RST], GPIO_PIN_SET);
}

void board_w5500RstLow(void)
{    
    drv_gpioSetLevel(&w5500_boardParam[W5500_RST], GPIO_PIN_RESET);
}

void board_w5500WriteByte(uint8_t data)
{
    drv_spiWriteBytes(&data, 1);
}

uint8_t board_w5500ReadByte(void)
{
    return drv_spiReadByte();
}

void board_w5500WriteBytes(uint8_t* data, uint16_t length)
{
    drv_spiWriteBytes(data, length);
}

void board_w5500ReadBytes(uint8_t* data, uint16_t length)
{
    drv_spiReadBytes(data, length);
}

void board_w5500CallbackReg(void)
{
    reg_wizchip_cs_cbfunc(board_w5500ChipSel, board_w5500ChipDeSel);
    reg_wizchip_spi_cbfunc(board_w5500ReadByte, board_w5500WriteByte);
    reg_wizchip_spiburst_cbfunc(board_w5500ReadBytes, board_w5500WriteBytes);
}
