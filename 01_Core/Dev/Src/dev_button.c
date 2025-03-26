#include "elog.h"
#include "board_gpio.h"
#include "dev.h"
#include "com_multiButton.h"

// 在此定义multibutton结构体变量
struct Button pause_key_s;
struct Button switch_key_s;

static void pauseButton_pressDownTask(void * btn);
static void pauseButton_longPressTask(void * btn);
static void switchButton_pressDownTask(void * btn);
static void switchButton_pressUpTask(void * btn);

void dev_buttonInit(gpioBoard_enum_t button)
{
    board_gpioInit(button);
}

uint8_t dev_buttonRead(uint8_t button_id)
{
    switch (button_id)
    {
    case BUTTON_ID_PAUSE:
        /* code */
        return board_gpioGetLevel(pause_key);

    case BUTTON_ID_SWITCH:
        /* code */
        return board_gpioGetLevel(switch_key);

    default:
        return 0;
    }
}

void dev_multiButtonInit(struct Button* button_handler, uint8_t button_id)
{
    button_init(button_handler, dev_buttonRead, 0, button_id);
}

void dev_multiButtonAddAndStart(struct Button* button_handler, PressEvent event, BtnCallback cb)
{
    button_attach(button_handler, event, cb);
}

/********************************key_task************************************ */
void dev_bottonTaskInit(void)
{
    dev_multiButtonInit(&pause_key_s, BUTTON_ID_PAUSE);
    dev_multiButtonAddAndStart(&pause_key_s, PRESS_DOWN, pauseButton_pressDownTask);
    dev_multiButtonAddAndStart(&pause_key_s, LONG_PRESS_START, pauseButton_longPressTask);
    button_start(&pause_key_s);

    dev_multiButtonInit(&switch_key_s, BUTTON_ID_SWITCH);
    dev_multiButtonAddAndStart(&switch_key_s, PRESS_DOWN, switchButton_pressDownTask);
    dev_multiButtonAddAndStart(&switch_key_s, PRESS_UP, switchButton_pressUpTask);
    button_start(&switch_key_s);

}

static void pauseButton_pressDownTask(void * btn)
{
    dev_buzzerClose(buzzer);        // 短暂按下，关闭蜂鸣器
}

static void pauseButton_longPressTask(void * btn)
{
    dev_buzzerInit(buzzer);             // 长按重新开启蜂鸣器
}

static void switchButton_pressDownTask(void * btn)
{
    // 发送对侧播报停止信息

}

static void switchButton_pressUpTask(void * btn)
{
    // 功能待开发
}
