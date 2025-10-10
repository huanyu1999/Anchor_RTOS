#include "dev_button.h"
#include "drv_timer.h"
#include "elog.h"
#include "cmsis_os.h"

// 在此定义multibutton结构体变量
struct Button pause_key_s;
struct Button switch_key_s;

TIM_HandleTypeDef button_tickHandler;

static void pauseButton_pressDownTask(void * btn);
static void pauseButton_longPressTask(void * btn);
static void switchButton_pressDownTask(void * btn);
static void switchButton_pressUpTask(void * btn);

void dev_buttonInit(gpioBoard_enum_t button) {
    board_gpioInit(button);
}

uint8_t dev_buttonRead(uint8_t button_id) {
    switch (button_id) {
    case BUTTON_ID_PAUSE:
        /* code */
        return board_gpioGetLevel(PAUSE_KEY);

    case BUTTON_ID_SWITCH:
        /* code */
        return board_gpioGetLevel(SWITCH_KEY);

    default:
        return 0;
    }
}

void dev_multiButtonInit(struct Button* button_handler, uint8_t button_id) {
    button_init(button_handler, dev_buttonRead, 0, button_id);
}

void dev_multiButtonAddAndStart(struct Button* button_handler, PressEvent event, BtnCallback cb) {
    button_attach(button_handler, event, cb);
}

/********************************key_task************************************ */
void dev_buttonTaskInit(void) {
    drv_setTimerForInt(&button_tickHandler, TIM3, 200, 7);
    
    dev_multiButtonInit(&pause_key_s, BUTTON_ID_PAUSE);
    dev_multiButtonAddAndStart(&pause_key_s, PRESS_DOWN, pauseButton_pressDownTask);
    dev_multiButtonAddAndStart(&pause_key_s, LONG_PRESS_START, pauseButton_longPressTask);
    button_start(&pause_key_s);

    dev_multiButtonInit(&switch_key_s, BUTTON_ID_SWITCH);
    dev_multiButtonAddAndStart(&switch_key_s, PRESS_DOWN, switchButton_pressDownTask);
    dev_multiButtonAddAndStart(&switch_key_s, PRESS_UP, switchButton_pressUpTask);
    button_start(&switch_key_s);
}

static void pauseButton_pressDownTask(void * btn) {
    dev_buzzerClose(BUZZER);        // 短暂按下，关闭蜂鸣器
    log_d("bee close.");
}

static void pauseButton_longPressTask(void * btn) {
    dev_buzzerInit(BUZZER);         // 长按重新开启蜂鸣器
    log_d("bee open.");
}

extern osThreadId_t task3_canSend_Handle;
static void switchButton_pressDownTask(void * btn) {
    osThreadFlagsSet(task3_canSend_Handle, 0x01);                 // 发送当前键值
    log_d("switchButton_pressDownTask 0x01");
}

static void switchButton_pressUpTask(void * btn) {
    osThreadFlagsSet(task3_canSend_Handle, 0x02);
    log_d("switchButton_pressUpTask 0x02");
}
