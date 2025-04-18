#include "dev_led_buzzer_dip.h"
#include "board_gpio.h"


/*************************************led************************************** */
void dev_ledAllInit(void)
{
    dev_ledInit(uwb_ok_led);
    dev_ledInit(onside_led);
    dev_ledInit(across_led);
    dev_ledInit(can_tx_led);
    dev_ledInit(can_rx_led);
    
    // 初始化阶段将所有指示灯关闭
    dev_ledOff(uwb_ok_led);
    dev_ledOff(onside_led);
    dev_ledOff(across_led);
    dev_ledOff(can_tx_led);
    dev_ledOff(uwb_ok_led);
}

void dev_ledInit(gpioBoard_enum_t led)
{
    board_gpioInit(led);
}

void dev_ledBlink(gpioBoard_enum_t led)
{
    board_gpioToggleLevel(led);
}

void dev_ledOn(gpioBoard_enum_t led)
{
    switch (led)
    {
    case uwb_ok_led:
    case can_rx_led:
    case can_tx_led:
        board_gpioSetLevel(led, io_LevelHigh);
        break;

    case across_led:
    case onside_led:
        board_gpioSetLevel(led, io_LevelLow);
        break;
    
    default:
        break;
    } 
}

void dev_ledOff(gpioBoard_enum_t led)
{
    switch (led)
    {
    case uwb_ok_led:
    case can_rx_led:
    case can_tx_led:
        board_gpioSetLevel(led, io_LevelLow);
        break;

    case across_led:
    case onside_led:
        board_gpioSetLevel(led, io_LevelHigh);
        break;
    
    default:
        break;
    } 
}

/*************************************buzzer************************************** */
void dev_buzzerInit(gpioBoard_enum_t buzzer)
{
    board_gpioInit(buzzer);
}

void dev_buzzerOpen(gpioBoard_enum_t buzzer)
{   
    board_gpioSetLevel(buzzer, io_LevelHigh);
}

void dev_buzzerClose(gpioBoard_enum_t buzzer)
{   
    board_gpioClose(buzzer);
    // board_gpioSetLevel(buzzer, io_LevelLow);
}

/*************************************DIP************************************** */
void dev_dipInit(gpioBoard_enum_t dip)
{
    board_gpioInit(dip);
}

uint8_t dev_dipRead(gpioBoard_enum_t dip)
{
    return board_gpioGetLevel(dip);
}

uint8_t dev_getDipVal(void)
{
    uint8_t switch_value = 0;

    switch_value = SWITCH_IS_ON(sw2) << 2
                    | SWITCH_IS_ON(sw1) << 1
                    | SWITCH_IS_ON(sw0) << 0;
    return switch_value;
}

