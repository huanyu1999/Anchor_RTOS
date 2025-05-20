#include "board_gpio.h"

/*
    如果有新的GPIO直连的设备，在此添加
    uwb_ok_led  PC2
    onside_led  PC5
    across_led  PB0
    can_rx_led  PB1
    switch_key  PC1
    pause_key   PC0
    buzzer      PC6
    sw0         PC14
    sw1         PC15
*/

static gpio_config_t gpio_boardParam[IO_NUM] = {
    {.gpio_port = UwbOK_LED_GPIO_Port, .clk_port = GPIO_PORT_C, .gpio_pin = UwbOK_LED_Pin, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_PULLUP, .gpio_speed = GPIO_SPEED_FREQ_VERY_HIGH},

    {.gpio_port = Onside_LED_GPIO_Port, .clk_port = GPIO_PORT_C,  .gpio_pin = Onside_LED_Pin, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_PULLUP, .gpio_speed = GPIO_SPEED_FREQ_VERY_HIGH},

    {.gpio_port = Across_LED_GPIO_Port, .clk_port = GPIO_PORT_B,  .gpio_pin = Across_LED_Pin, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_PULLUP, .gpio_speed = GPIO_SPEED_FREQ_VERY_HIGH},

    {.gpio_port = CAN_RX_LED_GPIO_Port, .clk_port = GPIO_PORT_B,  .gpio_pin = CAN_RX_LED_Pin, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_PULLUP, .gpio_speed = GPIO_SPEED_FREQ_VERY_HIGH},

    {.gpio_port = S_KEY_GPIO_Port, .clk_port = GPIO_PORT_C, .gpio_pin = S_KEY_Pin, .gpio_mode = GPIO_MODE_INPUT, .gpio_pull = GPIO_PULLUP, .gpio_speed = NULL},

    {.gpio_port = P_KEY_GPIO_Port, .clk_port = GPIO_PORT_C, .gpio_pin = P_KEY_Pin, .gpio_mode = GPIO_MODE_INPUT, .gpio_pull = GPIO_PULLUP, .gpio_speed = NULL},

    {.gpio_port = BUZZER_GPIO_Port, .clk_port = GPIO_PORT_C, .gpio_pin = BUZZER_Pin, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_PULLUP, .gpio_speed = GPIO_SPEED_FREQ_VERY_HIGH},

    {.gpio_port = SW0_GPIO_Port, .clk_port = GPIO_PORT_C, .gpio_pin = SW0_Pin, .gpio_mode = GPIO_MODE_INPUT, .gpio_pull = GPIO_PULLUP, .gpio_speed = NULL},

    {.gpio_port = SW1_GPIO_Port, .clk_port = GPIO_PORT_C, .gpio_pin = SW1_Pin, .gpio_mode = GPIO_MODE_INPUT, .gpio_pull = GPIO_PULLUP, .gpio_speed = NULL},
};

void board_gpioInit(gpioBoard_enum_t io_index)
{
    // 在板级这一层，传入打开gpio时钟的回调函数
    drv_gpioInit(&gpio_boardParam[io_index]);
}

void board_gpioSetLevel(gpioBoard_enum_t io_index, uint8_t level)
{
    drv_gpioSetLevel(&gpio_boardParam[io_index], level);
}

uint8_t board_gpioGetLevel(gpioBoard_enum_t io_index)
{
    return drv_gpioGetLevel(&gpio_boardParam[io_index]);
}

void board_gpioToggleLevel(gpioBoard_enum_t io_index)
{
    drv_gpioToggleLevel(&gpio_boardParam[io_index]);
}

void board_gpioClose(gpioBoard_enum_t io_index)
{
    drv_gpioClose(&gpio_boardParam[io_index]);
}

