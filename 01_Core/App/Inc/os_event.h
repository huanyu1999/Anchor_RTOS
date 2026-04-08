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

/* 报警等级（按距离递减、紧迫度递增） */
typedef enum {
    ALARM_LEVEL_0,       // 安全：> 50m 或无有效距离
    ALARM_LEVEL_1,       // 注意：20~50m，间隔播报
    ALARM_LEVEL_2,       // 警告：10~20m，连续播报
    ALARM_LEVEL_3,       // 危险：< 10m，紧急语音 + 蜂鸣器
    ALARM_MUTED,         // 被静音
} alarm_state_t;

/* 分级报警距离阈值，单位 mm */
#define ALARM_DIST_FAR       50000    // 50m — 远距离预警门限
#define ALARM_DIST_NEAR      20000    // 20m — 近距离警告门限
#define ALARM_DIST_DANGER    10000    // 10m — 危险距离（安全绕行红线）

/* 语音播报间隔，单位 ms */
#define VOICE_INTERVAL_L1    3000     // LEVEL_1：3s 间隔
#define VOICE_INTERVAL_L2    1500     // LEVEL_2：1.5s 间隔
#define VOICE_INTERVAL_L3    800      // LEVEL_3：0.8s 连续紧急

#endif
