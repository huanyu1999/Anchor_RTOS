#ifndef __OS_EVNET_H__
#define __OS_EVNET_H__
#include "cmsis_os.h"

extern osMessageQueueId_t queue_alarm;

/**************************************************************alarm Output**************************************************************/
typedef enum {
    final_minDistance_get = 0,            // 基站获取到了当前的最小距离，准备播报
    pauseButton_longPress_alarm,          // long press the pause button, reopen the alarm
    pauseButton_click_mute,               // press the pause button, mute the alarm
    switchButton_alarm,                   // turn the switch button, open the alarm
    switchButton_mute,                    // turn the switch button, mute the alarm
    no_final_minDistance,                 // the anchor does not get the vaild distance, mute the alarm
    dis_changed_alarm,                    // the current min distance ,open the alarm
    switchButton_left_rotation,
    switchButton_right_rotation,
    can_received                          // can interface receive success
} alarm_event_t;

typedef enum {
    ALARM_IDLE,          // 无报警
    ALARM_ACTIVE,        // 报警激活中
    ALARM_MUTED,         // 被静音
    // ALARM_WAIT_FINAL     // 等待最终最小距离信号
} alarm_state_t;

#endif
