#include "board_gnss.h"
#include "gpio.h"

static gpio_config_t gnssBoardParam[] = {
    { .gpio_port = GNSS_RST_PORT, .gpio_pin = GNSS_RST_PIN, .gpio_mode = GPIO_MODE_OUTPUT_PP, 
      .gpio_pull = GPIO_NOPULL, .gpio_speed = GPIO_SPEED_FAST },
};

void board_gnssModInit(void)
{
    drv_gpioInit(gnssBoardParam);
    drv_gpioSetLevel(gnssBoardParam, GPIO_PIN_RESET);
}

void board_gnssModReset(void)
{
    drv_gpioSetLevel(gnssBoardParam, GPIO_PIN_SET);
    HAL_Delay(10);
    drv_gpioSetLevel(gnssBoardParam, GPIO_PIN_RESET);
}

