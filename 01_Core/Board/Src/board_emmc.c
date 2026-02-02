#include "board_emmc.h"
#include "gpio.h"
#include "cmsis_os.h"

static gpio_config_t emmc_boardParam[] = {
    {.gpio_port = EMMC_RST_PORT, .gpio_pin = EMMC_RST_PIN, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_PULLUP, .gpio_speed = GPIO_SPEED_FAST}
};

void board_emmcInit(void)
{
    drv_gpioInit(emmc_boardParam);
}

void board_emmcReset(void)
{
    drv_gpioSetLevel(emmc_boardParam, GPIO_PIN_RESET);
    HAL_Delay(1);
    // osDelay(1);
    drv_gpioSetLevel(emmc_boardParam, GPIO_PIN_SET);
    HAL_Delay(5);
    // osDelay(5);
}
