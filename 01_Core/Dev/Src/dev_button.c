#include "main.h"
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

void dev_buttonInit(gpioBoard_enum_t button) 
{
    board_gpioInit(button);
}

uint8_t dev_buttonRead(uint8_t button_id) 
{
    switch (button_id)
    {
    case BUTTON_ID_PAUSE:
        return board_gpioGetLevel(PAUSE_KEY);

    case BUTTON_ID_SWITCH:
        return board_gpioGetLevel(SWITCH_KEY);

    default:
        return 0;
    }
}

// void dev_multiButtonInit(struct Button* button_handler, uint8_t button_id)
// {
//     button_init(button_handler, dev_buttonRead, 0, button_id);
// }

// void dev_multiButtonAddAndStart(struct Button* button_handler, PressEvent event, BtnCallback cb)
// {
//     button_attach(button_handler, event, cb);
// }

/********************************key_task************************************ */
void dev_buttonMultiInit(void)
{
    drv_setTimerForInt(&button_tickHandler, TIM3, 200, 7);
    
    // dev_multiButtonInit(&pause_key_s, BUTTON_ID_PAUSE);
    // dev_multiButtonAddAndStart(&pause_key_s, PRESS_DOWN, pauseButton_pressDownTask);
    // dev_multiButtonAddAndStart(&pause_key_s, LONG_PRESS_START, pauseButton_longPressTask);
    button_init(&pause_key_s, dev_buttonRead, 0, BUTTON_ID_PAUSE);
    button_attach(&pause_key_s, PRESS_DOWN, pauseButton_pressDownTask);
    button_attach(&pause_key_s, LONG_PRESS_START, pauseButton_longPressTask);
    button_start(&pause_key_s);

    // dev_multiButtonInit(&switch_key_s, BUTTON_ID_SWITCH);
    // dev_multiButtonAddAndStart(&switch_key_s, PRESS_DOWN, switchButton_pressDownTask);
    // dev_multiButtonAddAndStart(&switch_key_s, PRESS_UP, switchButton_pressUpTask);
    button_init(&switch_key_s, dev_buttonRead, 0, BUTTON_ID_SWITCH);
    button_attach(&switch_key_s, PRESS_DOWN, switchButton_pressDownTask);
    button_attach(&switch_key_s, PRESS_DOWN, switchButton_pressUpTask);
    button_start(&switch_key_s);
}

extern osThreadId_t task10_emmcHandle;
extern osThreadId_t task2_voiceOutControl_handle;
static void pauseButton_pressDownTask(void * btn)
{
    UNUSED(btn);
    osThreadFlagsSet(task10_emmcHandle, 0x01);
    osThreadFlagsSet(task2_voiceOutControl_handle, FLAG_P_BUTTON_MUTE);
    log_d("bee close.");
}

static void pauseButton_longPressTask(void * btn)
{
    UNUSED(btn);
    osThreadFlagsSet(task2_voiceOutControl_handle, FLAG_P_BUTTON_ALARM);     
    log_d("bee open.");
}

extern osThreadId_t task_canSend_handle;
static void switchButton_pressDownTask(void * btn)
{
    UNUSED(btn);
    osThreadFlagsSet(task_canSend_handle, FLAG_S_BUTTON_TRIGGER);                
    // log_d("switchButton_pressDown");
}

static void switchButton_pressUpTask(void * btn)
{   
    UNUSED(btn);
    osThreadFlagsSet(task_canSend_handle, FLAG_S_BUTTON_TRIGGER);
    // log_d("switchButton_pressUp");
}
