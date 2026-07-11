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

/************************************** TWR 时序常量（TREK1000 同源） ***************************************/
/* 所有 TWR 时序由 twr_set_replydelay()（dw_main.c，移植 TREK instance_set_replydelay）
 * 按帧长公式统一算出并装填 instance_data_t.timings，不再有按速率展开的时序宏。
 * 双端约定与参考数值见 docs/TWR_TIMING.md，Tag 工程须按同一公式适配。 */
#define DW_RX_ON_DELAY                  16      // us，DW 接收机使能到可收数据的开机延时
#define RX_RESPONSE_TURNAROUND          500     // us，帧间处理翻转余量（TREK 裸机原值 300；RTOS 回调路径保守取 500）
#define MAX_POLL_SEND_SLEEP_COUNT       150     // MAX_POLL_SEND_SLEEP_COUNT次发送后无运动则进入休眠

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

/* A2A(基站间测距) FINAL 帧字段索引 */
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

typedef enum {
    UWB_TX_MODE_IMMEDIATE = 0,
    UWB_TX_MODE_DELAYED,
    UWB_TX_MODE_DELAYED_REF,
    UWB_TX_MODE_DELAYED_RX_TS,
    UWB_TX_MODE_DELAYED_TX_TS,
    UWB_TX_MODE_CCA,
} uwb_tx_mode_t;

/* TWR测距状态 */
typedef enum {
    RANGE_NULL, RANGE_TWR_OK, RANGE_ERROR
} twrStatus;

/* UWB 芯片类型 */
typedef enum {
    UWB_CHIP_DW1000, UWB_CHIP_DW3000
} uwbChipType;

/* 系统运行角色 */
// Tag = Exchanges DecaRanging messages (Poll-Response-Final) with Anchor and enabling Anchor to calculate the range between the two instances
// Anchor = see above
// Anchor_Rng = the anchor (assumes a tag function) and ranges to another anchor - used in Anchor to Anchor TWR for auto positioning function
typedef enum {
    TAG, ANCHOR, ANCHOR_RNG, NUM_MODES
} instanceModes;

// instance sending a poll (starting TWR) is INITIATOR
// instance which receives a poll (and will be involved in the TWR) is RESPONDER
// instance which does not receive a poll (default state) will be a LISTENER - will send no responses
// RESONDER_A = RESPONDER_Anchor_Poll
// RESPONDER_B = RESPONDER_Blink
// RESPONDER_T = RESPONDER_Tag_Poll
typedef enum {
    INITIATOR,
    RESPONDER_A, RESPONDER_B, RESPONDER_T,
    LISTENER,
    GREETER,
    ATWR_MODES
} twrModes;

/******************************************************TWR Timings*************************************************************/
/* 统一时序参数集（TREK instance_set_replydelay 的输出字段），
 * init 时由 twr_set_replydelay() 装填一次，T2A/A2A 全部时序引用此结构。
 * 32h = 40bit DW 设备时间的高 32 位（即 >>8）；sy = symbol（1.0256us，dwt_setrxtimeout 原生单位）。
 * TREK 时序模型：统一槽间隔，首个 resp 槽 = poll RX + 1×fixedReplyDelay，逐槽 +1×；
 * delayed TX 编程的是 RMARKER 时刻，delayed RX 编程的是开机时刻，开窗须提前一个前导码。 */
typedef struct {
    uint32_t fixedReplyDelayAnc32h;   // 统一槽间隔 = devtime(resp整帧 + RX_RESPONSE_TURNAROUND) >> 8
    uint32_t preambleDuration32h;     // 前导码时长(devtime>>8) + DW_RX_ON_DELAY，delayed RX 开窗提前量
    uint32_t pollTx2FinalTxDelay32h;  // poll TX → final TX 总延时(devtime>>8) = (N+1)×fixedReplyDelay，Tag 端同公式
    uint16_t fixedReplyDelay_sy;      // 槽间隔的 symbol 表示，多槽接收超时窗算术用（A2A 容错）
    uint16_t fwto4RespFrame_sy;       // resp 帧接收超时（symbol）
    uint16_t fwto4FinalFrame_sy;      // final 帧接收超时（symbol，按较长的 T2A final 计算 + 余量）
} twrTimings_t;

/******************************************************TWR Instance************************************************************/
/* 统一实例管理结构（参照 TREK1000 instance_data_t），单例经 instance_get() 访问。
 *
 * T2A responder / A2A initiator / A2A responder 共用同一套交换状态：
 * 同一时刻设备只处于一个交换中，语义由 device_mode + twr_mode 区分（TREK1000 同款），
 * 不为 A2A 单独维护状态机或计数器副本。
 * 对外上报用的全局变量（anc_id/distance_report/range_status 等）不在此内，见文件尾 extern 区。 */
typedef struct instance_data_s {
    /* 角色管理 */
    instanceModes device_mode;      // ANCHOR / ANCHOR_RNG（TAG 预留）
    twrModes      twr_mode;         // 当前交换中的子角色
    uint8_t       device_id;        // 拨码读取的基站 ID
    uint8_t       gatewayAnchor;    // device_id == 0：A0，负责标签时隙校准 / A2A 发起 / discovery

    twrTimings_t  timings;          // 统一时序参数（twr_set_replydelay() 装填）

    /* 单次交换的轮转状态 */
    int8_t   remainingRespToRx;     // T2A 专用：还需接收的他站 resp 数；-1 = 空闲（LISTENER 不变式，A2A 期间保持 -1）
    int8_t   remainingRespToRxAnc;  // A2A 专用：发起端还需接收的 resp2 数（TREK 共用一个计数器，本工程按模式彻底分开）
    uint8_t  rxRespMaskAnc;         // A2A 专用：本轮已收到 resp2 的发送方位掩码，final 有效位（TREK rxResponseMaskAnc）；T2A 不用掩码
    uint8_t  wait4final;            // 0 / WAIT4TAGFINAL / WAIT4ANCFINAL
    uint8_t  lastTxFcode;           // 最近调度发送的功能码，TX-done 事件分流用（对应 TREK previousState 的作用）
    uint8_t  frame_seq_nb;          // 802.15.4 帧序号，每帧 +1
    uint8_t  range_nb;              // T2A 测距序号（随标签 poll 更新，resp 原样回带）
    uint8_t  a2a_range_nb;          // A2A 测距序号（发起端自增；TREK rangeNumAnc 同理独立）
    uint8_t  recv_tag_id;           // 当前测距标签 ID
    uint8_t  resp_valid;            // 标签 final 帧携带的有效 resp 掩码
    uint32_t nextSlotTime32h;       // 下一 resp 槽的绝对调度时间(32h)，逐事件累加（TREK delayedTRXTime32h 同款）

    /* responder 侧时间戳（T2A / A2A 共用，交换互斥所以安全） */
    uint64_t poll_rx_ts;
    uint64_t resp_tx_ts;
    uint64_t final_rx_ts;
    /* initiator 侧时间戳与调度（A2A 发起 / 未来 tag 角色复用） */
    uint64_t poll_tx_ts;
    uint64_t resp_rx_ts[MAX_AHCHOR_NUMBER];
    uint32_t final_tx_time32h;      // 发起端 final 预调度发送时间(32h，RMARKER)
    uint32_t final_rx_time32h;      // 应答端 final 延迟接收槽基准(32h，开窗时再减前导码)

    int32_t  prev_range[MAX_TAG_LIST_SIZE];     // 各标签上一轮测距值(mm)，下轮 resp 回传
#if defined(ANCRANGE)
    int32_t  a2a_distance[MAX_AHCHOR_NUMBER];   // 与各基站的 A2A 距离(mm)
    /* A2A 超帧调度（A0 专用） */
    uint32_t sframePeriod_ms;       // 超帧周期
    uint32_t a2aStartTime_ms;       // 下次 A2A 触发的绝对 tick
#endif
} instance_data_t;

/******************************************************Distance Manage************************************************************/
#define ANCHOR_SELF_DIS  0
#define ANCHOR_OTHER_DIS 1
#define FINAL_DIS        2
#define MIN_DIS_QUEUE_LEN 3

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

/******************************************************SuperFrame Config******************************************************/
typedef struct
{
    uint16 slotDuration_ms ; // slot duration (time for 1 tag to range to some anchors)
    uint16 numSlots ; 		 // number of slots in one superframe (number of tags supported)
    uint16 sfPeriod_ms ;	 // superframe period in ms
    uint16 tagPeriod_ms ; 	 // the time during which tag ranges to anchors and then sleeps, should be same as FRAME PERIOD so that tags don't interfere
    uint16 pollTxToFinalTxDly_us ; // response delay time (Poll to Final delay)
} sfConfig_t ;

/******************************************************Uwb Event************************************************************/
typedef enum uwbEvent_e {
    eventPacketReceived,
    eventPacketSent,
    eventReceiveTimeout,
    eventReceiveFailed,
} uwbEvent_t;

// Callback for one uwb algorithm
typedef struct uwbAlgorithm_s {
    int (*init)(instance_data_t *inst);
    uint32_t (*onEvent)(instance_data_t *inst, uwbEvent_t event);
} uwbAlgorithm_t;

/* 对外上报/配置接口使用的全局变量（net_protocol.c / app_network.c / dw_sort.c 消费），
 * TWR 内部轮转状态已收入 instance_data_t，不再对外暴露 */
extern uint8_t anc_id;
extern uint8_t tag_id;
extern uint8_t group_id;                                // 组ID
extern int32_t distance_report[8];
extern int32_t group_report[8];                         // 基站组ID数组，用于打包输出
extern uint32_t range_time;
extern uint8_t inst_ch;                                 // 信道号Channel number
extern uint8_t inst_prf;                                // PRF
extern uint8_t range_status;
extern float rx_power;
extern uint16_t inst_slot_number;
extern uint8_t inst_dataRate;
extern uint8_t inst_one_slot_time;
extern uint16 ant_dly;
extern int32 distance_offset_cm;                        // 距离校准，单位cm

extern double dwt_getrangebias(uint8 chan, float range, uint8 prf);

/******************************************************dw_main.c************************************************************/
void dev_uwbInit(void);
void logOut_dw1000Config(void);
void task_uwb(void *arg);
void task_minHeapManage(void *arg);
void task_getMinDis(void *arg);
#if defined(USE_DW1000)
float uwb_isInNLOS_power(dwt_rxdiag_t *rx_diag, uint8_t receive_functionCode);
uint16_t uwb_isInNLOS_index(uint8_t receive_functionCode);
#endif
int check_twr_quality(uint16_t indexDiff_poll, uint16_t indexDiff_final);
dwDistance_t* get_the_local_structure_of_dis(void);
instance_data_t* instance_get(void);
int dev_uwbStartTx(uwb_tx_mode_t mode, bool response_expected);
#if defined(ANCRANGE)
void anch_checkA2ATrigger(instance_data_t *inst);
#endif
#endif
