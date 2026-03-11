#include "board_gpio.h"

/*
    如果有新的GPIO直连的设备，在此添加
    rt9013_en   PE8
    uwb_ok_led  PE0
    can_rx_led  PE1
    onside_led  PB1
    across_led  PE7
    switch_key  PB0
    pause_key   PC5
    buzzer      PE10
    anchor_id0  PE2
    anchor_id1  PE3
    anchor_id2  PE4
    anchor_id3  PE5
*/

static gpio_config_t gpio_boardParam[IO_NUM] = {
    { .gpio_port = RT9013_GPIO_PORT, .clk_port = GPIO_PORT_E, .gpio_pin = RT9013_EN_PIN, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_NOPULL, .gpio_speed = GPIO_SPEED_FREQ_VERY_HIGH },
    
    { .gpio_port = UWBOK_LED_GPIO_PORT, .clk_port = GPIO_PORT_E, .gpio_pin = UWBOK_LED_PIN, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_NOPULL, .gpio_speed = GPIO_SPEED_FREQ_VERY_HIGH },
    
    { .gpio_port = CAN_RX_LED_GPIO_PORT, .clk_port = GPIO_PORT_E, .gpio_pin = CAN_RX_LED_PIN, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_NOPULL, .gpio_speed = GPIO_SPEED_FREQ_VERY_HIGH },

    { .gpio_port = ONSIDE_LED_GPIO_PORT, .clk_port = GPIO_PORT_B, .gpio_pin = ONSIDE_LED_PIN, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_NOPULL, .gpio_speed = GPIO_SPEED_FREQ_VERY_HIGH },

    { .gpio_port = ACROSS_LED_GPIO_PORT, .clk_port = GPIO_PORT_E, .gpio_pin = ACROSS_LED_PIN, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_NOPULL, .gpio_speed = GPIO_SPEED_FREQ_VERY_HIGH },

    { .gpio_port = S_KEY_GPIO_PORT, .clk_port = GPIO_PORT_B, .gpio_pin = S_KEY_PIN, .gpio_mode = GPIO_MODE_INPUT, .gpio_pull = GPIO_PULLUP, .gpio_speed = 0 },

    { .gpio_port = P_KEY_GPIO_PORT, .clk_port = GPIO_PORT_C, .gpio_pin = P_KEY_PIN, .gpio_mode = GPIO_MODE_INPUT, .gpio_pull = GPIO_PULLUP, .gpio_speed = 0 },

    { .gpio_port = BUZZER_GPIO_PORT, .clk_port = GPIO_PORT_E, .gpio_pin = BUZZER_PIN, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_PULLUP, .gpio_speed = GPIO_SPEED_FREQ_VERY_HIGH },

    { .gpio_port = ANCHORID0_GPIO_PORT, .clk_port = GPIO_PORT_E, .gpio_pin = ANCHORID0_PIN, .gpio_mode = GPIO_MODE_INPUT, .gpio_pull = GPIO_PULLUP, .gpio_speed = 0 },

    { .gpio_port = ANCHORID1_GPIO_PORT, .clk_port = GPIO_PORT_E, .gpio_pin = ANCHORID1_PIN, .gpio_mode = GPIO_MODE_INPUT, .gpio_pull = GPIO_PULLUP, .gpio_speed = 0 },

    { .gpio_port = ANCHORID2_GPIO_PORT, .clk_port = GPIO_PORT_E, .gpio_pin = ANCHORID2_PIN, .gpio_mode = GPIO_MODE_INPUT, .gpio_pull = GPIO_PULLUP, .gpio_speed = 0 },

    { .gpio_port = ANCHORID3_GPIO_PORT, .clk_port = GPIO_PORT_E, .gpio_pin = ANCHORID3_PIN, .gpio_mode = GPIO_MODE_INPUT, .gpio_pull = GPIO_PULLUP, .gpio_speed = 0 }
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

