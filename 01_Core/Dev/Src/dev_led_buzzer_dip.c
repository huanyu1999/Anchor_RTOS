#include "dev_led_buzzer_dip.h"

/*************************************led&RT9013************************************** */
void dev_ledAndRT9013Init(void) {
    dev_ledInit(UWB_OK_LED);
    dev_ledInit(CAN_RX_LED);
    dev_ledInit(ONSIDE_LED);
    dev_ledInit(ACROSS_LED);
    board_gpioInit(RT9013_EN);
    board_gpioSetLevel(RT9013_EN, GPIO_PIN_SET);
    
    // 初始化阶段将所有指示灯关闭
    dev_ledOff(UWB_OK_LED);
    dev_ledOff(CAN_RX_LED);
    dev_ledOff(ONSIDE_LED);
    dev_ledOff(ACROSS_LED);
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
    switch (led) {
    case UWB_OK_LED:
    case CAN_RX_LED:
    case ACROSS_LED:
    case ONSIDE_LED:
        board_gpioSetLevel(led, io_LevelHigh);
        break;

    case RT9013_EN:
    case SWITCH_KEY:
    case PAUSE_KEY:
    case BUZZER:
    case ANCHOR_ID0:
    case ANCHOR_ID1:
    case ANCHOR_ID2:
    case IO_NUM:
    default:
        break;
    }
}

void dev_ledOff(gpioBoard_enum_t led) {
    switch (led) {
    case UWB_OK_LED:
    case CAN_RX_LED:
    case ACROSS_LED:
    case ONSIDE_LED:
        board_gpioSetLevel(led, io_LevelLow);
        break;

    case RT9013_EN:
    case SWITCH_KEY:
    case PAUSE_KEY:
    case BUZZER:
    case ANCHOR_ID0:
    case ANCHOR_ID1:
    case ANCHOR_ID2:
    case IO_NUM:
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
    board_gpioInit(buzzer);
    board_gpioSetLevel(buzzer, io_LevelHigh);
}

void dev_buzzerClose(gpioBoard_enum_t buzzer)
{
    // board_gpioSetLevel(buzzer, io_LevelLow);
    board_gpioClose(buzzer);
}

/*************************************DIP************************************** */
void dev_dipInit(gpioBoard_enum_t dip)
{
    board_gpioInit(dip);
    board_gpioSetLevel(dip, io_LevelHigh);  // 初始化后直接，内部拉高，拨码会将其拉低
}

uint8_t dev_dipRead(gpioBoard_enum_t dip)
{
    return board_gpioGetLevel(dip);
}

uint8_t dev_getDipVal(void)
{
    uint8_t switch_value = 0;

    switch_value = SWITCH_IS_ON(ANCHOR_ID2) << 2 |
                    SWITCH_IS_ON(ANCHOR_ID1) << 1 |
                    SWITCH_IS_ON(ANCHOR_ID0) << 0;
    return switch_value;
}

