#ifndef __DW_INSTANCE_H__
#define __DW_INSTANCE_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#if defined(USE_DW3000)
#pragma message_1("Building for dw3000")
#include "board_dw3000.h"
#include "port_dw3000.h"
/* DW1000 deca_types.h 定义了 uint8/uint16/uint32/int32，DW3000 没有，补上兼容定义 */
typedef uint8_t  uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int32_t  int32;
/* DW3000 SDK 不提供以下常量/宏，值取自 DW1000 deca_device_api.h，TWR 算法共用 */
#ifndef UUS_TO_DWT_TIME
#define UUS_TO_DWT_TIME 65536
#endif
#ifndef FRAME_LEN_MAX
#define FRAME_LEN_MAX (127)
#endif
#ifndef SPEED_OF_LIGHT
#define SPEED_OF_LIGHT (299702547.0)
#endif
#ifndef FINAL_MSG_TS_LEN
#define FINAL_MSG_TS_LEN 4
#include "../01_Core/Dw3000/decadriver/deca_device_api.h"
#include "../01_Core/Dw3000/decadriver/deca_regs.h"
#include "../01_Core/Dw3000/decadriver/deca_types.h"
#include "../01_Core/Dw3000/platform/port_dw3000.h"
#include "../01_Core/Dw3000/platform/deca_spi.h"
#endif
#elif defined(USE_DW1000)
// #pragma message_2("Building for dw1000")
#include "board_dw1000.h"
#include "port_dw1000.h"
#include "../01_Core/Dw1000/decadriver/deca_device_api.h"
#include "../01_Core/Dw1000/decadriver/deca_regs.h"
#include "../01_Core/Dw1000/decadriver/deca_types.h"
#include "../01_Core/Dw1000/platform/port_dw1000.h"
#include "../01_Core/Dw1000/platform/deca_spi.h"
#else
#error "Please define USE_DW1000 or USE_DW3000"
#endif

#include "dev_led_buzzer_dip.h"
#include "drv_timer.h"
#include "iwdg.h"

typedef enum {
    UWB_TX_MODE_IMMEDIATE = 0,
    UWB_TX_MODE_DELAYED,
    UWB_TX_MODE_DELAYED_REF,
    UWB_TX_MODE_DELAYED_RX_TS,
    UWB_TX_MODE_DELAYED_TX_TS,
    UWB_TX_MODE_CCA,
} uwb_tx_mode_t;

#define SOFTWARE_VER                   "V1.0"

#define ANCRANGE                        1
#define MAX_AHCHOR_NUMBER               3       // 系统内最大基站数量，取4或者8，比如实际3个取4，实际6个取8
#define MAX_TAG_NUMBER                  50      // 设置最大标签个数

/* 天线延时
 * 计算距离结果比实际距离小，需要增大距离，则减小这个数
 * 计算距离结果比实际距离大，需要减小距离，则增大这个数
 */                                                                                                               
#define ANT_DLY_DW1000                 16549   // DW1000：已按标签链路标定，作为参考基准
#define ANT_DLY_DW3000                 16347   // DW3000：原照抄 16549，A2A 混合对偏小 ~0.95m，减 202 单位单独标定

#if defined(USE_DW3000)
#define ANT_DLY_DEFAULT                ANT_DLY_DW3000
#elif defined(USE_DW1000)
#define ANT_DLY_DEFAULT                ANT_DLY_DW1000
#endif

/* 发射功率，目前设定为最大值 */
#if defined(USE_DW3000)
#define TX_POWER                        0xfdfdfdfd
#elif defined(USE_DW1000)
#define TX_POWER                        0x1f1f1f1f
#endif

#define MAX_TAG_LIST_SIZE               (MAX_TAG_NUMBER)
#define MASK_40BIT                      (0x00FFFFFFFFFF)  // DW1000 counter is 40 bits
#define MASK_TXDTS                      (0x00FFFFFFFE00)  // The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.

/* 数据帧超时及延时时间*/
#define PRE_TIMEOUT                     5

#if (MAX_AHCHOR_NUMBER == 3)            // 每个时隙的持续时间，时隙时间过小，会导致相邻ID的标签，位于后方的标签无法测距，被上一个标签测距所影响，尝试增大该值，但是标签的时隙值不能相同，否则还是一样的情况？
#define ONE_SLOT_TIME_MS_110K           28
#define ONE_SLOT_TIME_MS_850K           12
#define ONE_SLOT_TIME_MS_6P8M           9
#elif (MAX_AHCHOR_NUMBER == 2)
#define ONE_SLOT_TIME_MS_110K           28
#define ONE_SLOT_TIME_MS_850K           8
#define ONE_SLOT_TIME_MS_6P8M           9
#elif (MAX_AHCHOR_NUMBER == 8)
#define ONE_SLOT_TIME_MS_110K           50
#define ONE_SLOT_TIME_MS_850K           20
#define ONE_SLOT_TIME_MS_6P8M           15
#endif

/************************************** 单位 microsecond ***************************************/
#define FINAL_RX_TIMEOUT_6P8M           600
#define RESP_RX_TIMEOUT_6P8M            450
#define FIRST_RESP_SEND_6P8M            900     // 6.8M通信速率下，第一个resp消息发送延时
#define DATA_INTERVAL_TIME_6P8M         1100    // 6.8M通信速率下，相邻消息间隔时间
#define ANC_RESP_SEND_BACK_6P8M         100     // 6.8M通信速率下，基站延后发送RESP消息时间
#define TAG_FINALE_SEND_BACK_6P8M       100     // 6.8M通信速率下，标签延后发送FINAL消息时间

#define FINAL_RX_TIMEOUT_110K           6000
#define RESP_RX_TIMEOUT_110K            3800
#define FIRST_RESP_SEND_110K            3000    // 110K通信速率下，第一个resp消息发送延时
#define DATA_INTERVAL_TIME_110K         3900    // 110K通信速率下，相邻消息间隔时间
#define ANC_RESP_SEND_BACK_110K         1080    // 110K通信速率下，基站延后发送RESP消息时间
#define TAG_FINALE_SEND_BACK_110K       1080    // 110K通信速率下，标签延后发送FINAL消息时间

#define FINAL_RX_TIMEOUT_850K           1300    // 850K通信速率下，基站接收final帧超时时间
#define RESP_RX_TIMEOUT_850K            1000    // 850K通信速率下，基站接收resp帧超时时间
/* 
    850K通信速率下，第一个resp消息处理（发送或者接收）延时，之前设定的为1250，
    考虑的是基站接收到poll帧记录时间戳后，还要做帧解析，处理后续resp帧，这些都是在dwt_isr中做完，需要留够时间，确保处理完成不出错。
*/ 
#define FIRST_RESP_SEND_850K            1300    
#define DATA_INTERVAL_TIME_850K         1600    // 850K通信速率下，一个resp帧的处理（发送或者接收）时间， 一个完整的时间间隔
#define ANC_RESP_SEND_BACK_850K         300     // 850K通信速率下，基站延后发送RESP消息时间，在前面接收处理完poll帧的基础上，可能还会有接收resp帧的情况，难道是guard time？
#define TAG_FINALE_SEND_BACK_850K       300     // 850K通信速率下，标签延后发送FINAL消息时间
#define MAX_POLL_SEND_SLEEP_COUNT       150     // MAX_POLL_SEND_SLEEP_COUNT次发送后无运动则进入休眠
#define ANC_RANGE_COUNT                 5       // 自标定时每个基站测距次数

/* PAN ID */
#define PAN_ID                          0xDECA

/* 中断状态标志 */
#define RX_WAIT                         0
#define TX_WAIT                         0
#define RX_OK                           1
#define TX_OK                           1
#define RX_TIMEOUT                      2
#define RX_ERROR                        3

/* 数据帧长度 */
#define POLL_MSG_LEN                    16
#define RESP_MSG_LEN                    19
#define FIANL_MSG_LEN                   (22 + 5 * MAX_AHCHOR_NUMBER + 10)
#define BLINK_MSG_LEN                   10
#define INIT_MSG_LEN                    12
#define ANCH_POLL_MSG_LEN               11      // header(9) + fcode(1) + range_nb(1)
#define ANCH_RESP2_MSG_LEN              15      // header(9) + fcode(1) + range_nb(1) + prev_dis(4)
#define ANCH_FINAL_MSG_LEN              (12 + FINAL_MSG_TS_LEN * 2 + FINAL_MSG_TS_LEN * (MAX_AHCHOR_NUMBER - 1))  // header(9)+fcode(1)+range_nb(1)+valid(1)+poll_tx(4)+final_tx(4)+resp_rx(4)×N

/* 数据帧数组索引 */
#define SEQ_NB_IDX                      2
#define PANID_IDX                       3
#define RECEIVER_SHORT_ADD_IDX          5
#define SENDER_SHORT_ADD_IDX            7
#define FUNC_CODE_IDX                   9
#define RANGE_NB_IDX                    10

#define POLL_MSG_SOS_IDX                11
#define POLL_MSG_ALARM_STA_IDX          12
#define POLL_MSG_BATTERY_IDX            13
#define POLL_MSG_USER_IDX               14

#define RESP_MSG_SLEEP_COR_IDX          11
#define RESP_MSG_PREV_DIS_IDX           13
#define RESP_MSG_ALARM_IDX              17
#define RESP_MSG_GROUP_IDX              18

// #define INIT_MSG_SLEEP_COR_IDX          10

#define FINAL_MSG_FINAL_VALID_IDX       11
#define FINAL_MSG_POLL_TX_TS_IDX        12
#define FINAL_MSG_FINAL_TX_TS_IDX       17
#define FINAL_MSG_A0_GROUP_ID_IDX       22
#define FINAL_MSG_RESP1_RX_TS_IDX       23
#define FINAL_MSG_RESP2_RX_TS_IDX       28
// #define FINAL_MSG_A0_GROUP_ID_IDX       20
// #define FINAL_MSG_RESP1_RX_TS_IDX       21
// #define FINAL_MSG_RESP2_RX_TS_IDX       26

// #define FINAL_MSG_RESP3_RX_TS_IDX       31
// #define FINAL_MSG_RESP4_RX_TS_IDX       36
// #define FINAL_MSG_RESP5_RX_TS_IDX       41
// #define FINAL_MSG_RESP6_RX_TS_IDX       46
// #define FINAL_MSG_RESP7_RX_TS_IDX       51
// #define FINAL_MSG_RESP8_RX_TS_IDX       56

// #define SYNC_MSG_TIME_IDX               10
// #define FINAL_MSG_A1_GROUP_ID_IDX       25
// #define FINAL_MSG_A2_GROUP_ID_IDX       30
// #define FINAL_MSG_A3_GROUP_ID_IDX       35
// #define FINAL_MSG_A4_GROUP_ID_IDX       40
// #define FINAL_MSG_A5_GROUP_ID_IDX       45
// #define FINAL_MSG_A6_GROUP_ID_IDX       50
// #define FINAL_MSG_A7_GROUP_ID_IDX       44

/* A2A FINAL 帧字段索引 */
#define A2A_RESP2_PREV_DIS_IDX          11

#define A2A_FINAL_VALID_IDX             11
#define A2A_FINAL_POLL_TX_TS_IDX        12
#define A2A_FINAL_FINAL_TX_TS_IDX       (A2A_FINAL_POLL_TX_TS_IDX + FINAL_MSG_TS_LEN)
#define A2A_FINAL_RESP_RX_TS_BASE       (A2A_FINAL_FINAL_TX_TS_IDX + FINAL_MSG_TS_LEN)

/* Function codes — naming follows TREK1000 RTLS convention */
#define RTLS_MSG_TAG_POLL               0x21    // Tag poll (broadcast)
#define RTLS_MSG_ANCH_RESP              0x10    // Anchor response to tag poll
#define RTLS_MSG_TAG_FINAL              0x23    // Tag final (with timestamps)
#define RTLS_MSG_TAG_BLINK              0x36    // Tag blink (discovery)
#define RTLS_MSG_RNG_INIT               0x38    // Ranging init (anchor → tag, discovery response)
#define RTLS_MSG_ANCH_POLL              0x7A    // Anchor-to-anchor poll
#define RTLS_MSG_ANCH_RESP2             0x7B    // Anchor response to anchor poll (A2A)
#define RTLS_MSG_ANCH_FINAL             0x7C    // Anchor final (A2A)

#define WAIT4TAGFINAL					2
#define WAIT4ANCFINAL					1

/* 状态机标志位 */
typedef enum {
    STA_IDLE,
    STA_SEND_POLL,
    STA_WAIT_RESP,
    STA_RECV_RESP,
    STA_SEND_FINAL,
    STA_A2A_SEND_POLL,
    STA_A2A_WAIT_RESP,
    STA_A2A_RECV_RESP,
    STA_SEND_RESP,
    STA_WAIT_FINAL,
    STA_RECV_FINAL,
    STA_SORR_RESP,
    STA_SEND_BLINK,
    STA_WAIT_INIT,
    STA_RECV_INIT,
    STA_A2A_SEND_FINAL
} instStatus;

/* TWR测距状态 */
typedef enum { RANGE_NULL, RANGE_TWR_OK, RANGE_ERROR } twrStatus;

/* UWB 芯片类型 */
typedef enum { UWB_CHIP_DW1000, UWB_CHIP_DW3000 } uwbChipType;

/* 系统运行角色 */
// Tag = Exchanges DecaRanging messages (Poll-Response-Final) with Anchor and enabling Anchor to calculate the range between the two instances
// Anchor = see above
// Anchor_Rng = the anchor (assumes a tag function) and ranges to another anchor - used in Anchor to Anchor TWR for auto positioning function
typedef enum { TAG, ANCHOR, ANCHOR_RNG, NUM_MODES } instanceModes;

// instance sending a poll (starting TWR) is INITIATOR
// instance which receives a poll (and will be involved in the TWR) is RESPONDER
// instance which does not receive a poll (default state) will be a LISTENER - will send no responses
// RESONDER_A = RESPONDER_Anchor_Poll
// RESPONDER_B = RESPONDER_Blink
// RESPONDER_T = RESPONDER_Tag_Poll
typedef enum { INITIATOR, RESPONDER_A, RESPONDER_B, RESPONDER_T, LISTENER, GREETER, ATWR_MODES } twrModes;

/******************************************************Dw1000 Device************************************************************/
struct dwDevice_s;

typedef void (*dwHandler_t)(struct dwDevice_s *dev);

typedef struct dwDevice_s {
    void* user_data;
    instanceModes device_mode;
    twrModes twr_mode;
    uint8_t device_id;
    uint8_t rxOtherResp;
    uint8_t respTxIndex;
    int8_t  remainingRespToRx;
    uint8_t wait4final;

    uint16_t indexDiff_poll;
    uint16_t indexDiff_final;
} dwDevice_t;

/******************************************************Distance Manage************************************************************/
#define ANCHOR_SELF_DIS  0
#define ANCHOR_OTHER_DIS 1
#define FINAL_DIS        2
#define MIN_DIS_QUEUE_LEN 3

// typedef struct {
//     uint8_t twr_successSign;          // 本次TWR测距是否成功的标志位，同标签编号进行绑定
//     int32_t tag_distance;
// } tagDistance_t;    

typedef struct {            
    int32_t dis_value;
    uint8_t dis_class;
    uint8_t dis_index;
} outDistance_t;

typedef struct {
    outDistance_t disMsg[MIN_DIS_QUEUE_LEN];               // 输出距离时使用的数组
    int32_t       min_dis;                                 // 最小的距离值
    uint8_t       dis_idx;                                 // 最小距离值对应的索引，也就是对应的标签ID                   
} dwDistance_t;
 
/******************************************************Uwb Event************************************************************/
typedef enum uwbEvent_e {
    eventPacketReceived,
    eventPacketSent,
    eventReceiveTimeout,
    eventReceiveFailed,
} uwbEvent_t;

// Callback for one uwb algorithm
typedef struct uwbAlgorithm_s {
    int (*init)(dwDevice_t *dev);
    uint32_t (*onEvent)(dwDevice_t *dev, uwbEvent_t event);
} uwbAlgorithm_t;

// 待后续优化测距模块代码再处理
extern uint8_t anc_id;
extern uint8_t tag_id;
extern uint8_t group_id;                                // 组ID
extern int32_t distance_report[8];
extern int32_t group_report[8];                         // 基站组ID数组，用于打包输出
extern uint32_t range_time;
extern uint8_t inst_ch;                                 // 信道号Channel number
extern uint8_t inst_prf;                                // PRF
extern uint8_t frame_seq_nb;                            // 每帧数据增加1
extern uint8_t range_nb;                                // 每次range增加1(poll resp1~4 fianl维护一套range_nb)
extern uint8_t recv_tag_id;
extern uint8_t recv_anc_id;
extern uint8_t range_status;
extern float rx_power;
extern uint16_t inst_slot_number;
extern uint8_t inst_dataRate; 
extern uint8_t inst_one_slot_time;
extern uint32_t inst_final_rx_timeout;
extern uint32_t inst_resp_rx_timeout;
extern uint64_t inst_poll2final_time;
extern uint32_t inst_data_interval;
extern uint16 ant_dly;
extern int32 distance_offset_cm;                        // 距离校准，单位cm
extern int user_data[10];

#if defined(ANCRANGE)
extern int32_t a2a_distance[MAX_AHCHOR_NUMBER];
extern uint32_t sframePeriod_ms;
extern uint32_t a2aStartTime_ms;
#endif

extern double dwt_getrangebias(uint8 chan, float range, uint8 prf);

/******************************************************dw_main.c************************************************************/
void dev_uwbInit(void);
void logOut_dw1000Config(void);
void task_uwb(void *arg);
void task_twrRun(void *arg);
void task_minHeapManage(void *arg);
void task_getMinDis(void *arg);
#if defined(USE_DW1000)
float uwb_isInNLOS_power(dwt_rxdiag_t *rx_diag, uint8_t receive_functionCode);
uint16_t uwb_isInNLOS_index(uint8_t receive_functionCode);
#endif
int check_twr_quality(uint16_t indexDiff_poll, uint16_t indexDiff_final);
dwDistance_t* get_the_local_structure_of_dis(void);
dwDevice_t* get_the_local_structure_of_dev(void);
uint8_t get_tagFinalRecvFlag(uint8_t tad_idx);
int uwb_starttx(uwb_tx_mode_t mode, bool response_expected);
#if defined(ANCRANGE)
void anch_checkA2ATrigger(dwDevice_t *dev);
#endif
#endif
