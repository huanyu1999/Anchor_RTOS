#include "main.h"
#include "dev_button.h"
#include "drv_timer.h"
#include "os_event.h"
#include "elog.h"
#include "cmsis_os.h"


// 在此定义multibutton结构体变量
struct Button pause_key_s;
struct Button switch_key_s;

TIM_HandleTypeDef button_tickHandler;

static void pauseButton_pressDownTask(void * btn);
static void pauseButton_longPressTask(void * btn);
static void switchButton_leftRotationTask(void * btn);
static void switchButton_rightRotationTask(void * btn);

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

/********************************key_task************************************ */
void dev_buttonMultiInit(void)
{
    drv_setTimerForInt(&button_tickHandler, TIM3, 200, 7);
    
    button_init(&pause_key_s, dev_buttonRead, 0, BUTTON_ID_PAUSE);
    button_attach(&pause_key_s, PRESS_DOWN, pauseButton_pressDownTask);
    button_attach(&pause_key_s, LONG_PRESS_START, pauseButton_longPressTask);
    button_start(&pause_key_s);

    button_init(&switch_key_s, dev_buttonRead, 0, BUTTON_ID_SWITCH);   // 该旋钮可视作一个自锁按键
    button_attach(&switch_key_s, PRESS_DOWN, switchButton_rightRotationTask);
    button_attach(&switch_key_s, PRESS_UP, switchButton_leftRotationTask);
    button_start(&switch_key_s);
}

// extern osThreadId_t task10_emmcHandle;
extern osThreadId_t task2_voiceOutControl_handle;
static void pauseButton_pressDownTask(void * btn)
{
    UNUSED(btn);
    alarm_event_t evt = pauseButton_click_mute;
    osMessageQueuePut(queue_alarm, &evt, 0, 0);
}

static void pauseButton_longPressTask(void * btn)
{
    UNUSED(btn);
    alarm_event_t evt = pauseButton_longPress_alarm;
    osMessageQueuePut(queue_alarm, &evt, 0, 0);  
}

extern osThreadId_t task_canSend_handle;
static void switchButton_leftRotationTask(void * btn)
{
    UNUSED(btn);
    alarm_event_t evt = switchButton_left_rotation;
    osMessageQueuePut(queue_alarm, &evt, 0, 0);           
}

static void switchButton_rightRotationTask(void * btn)
{   
    UNUSED(btn);
    alarm_event_t evt = switchButton_right_rotation;
    osMessageQueuePut(queue_alarm, &evt, 0, 0);
}
