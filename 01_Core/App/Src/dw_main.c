#include "dw_sort.h"
#include "dw_instance.h"
#include "dev_can.h"
#include "uthash.h"
#include "cmsis_os.h"
#include "elog.h"
#include "SEGGER_RTT.h"
#include <math.h>

uint8_t group_id;                   // 组ID，后续会有用处，不同车务段的人员共同施工，各自跟各自的基站通信？？？？
uint8_t anc_id;                     // 如当前角色是基站，则表示当前基站ID
uint8_t tag_id;                     // 如当前角色是标签，则表示当前标签ID
int32_t distance_report[8];         // 基站测距值数组，用于打包输出
int32_t group_report[8];            // 基站组ID数组，用于打包输出
uint32_t range_time;                // 测距产生时间，串口打包发送
uint8_t range_status = RANGE_NULL;  // 测距成功标志位，用于打包输出
float rx_power;                     // 接收RSSI
uint16_t inst_slot_number;          // 系统内最大标签容量
uint8_t inst_dataRate;              // 通信速率，用于根据当前110K还是6.8M确定数据超时等通信过程相关参数
uint8_t inst_ch;                    // 信道号Channel number
uint8_t inst_prf;                   // PRF
uint8_t inst_one_slot_time;         // 一个slot的时间，根据通信速率不同而不同，单位ms
uint32_t inst_final_rx_timeout;     // 基站final接收超时时间，根据通信速率不同而不同，单位us
uint32_t inst_resp_rx_timeout;      // 标签发送poll后接收resp超时时间，根据通信速率不同而不同，单位us
uint64_t inst_poll2final_time;      // 单TWR周期poll起始到final结束的总时间
uint32_t inst_data_interval;        // 相邻两条数据的间隔，如poll和第一个resp的间隔，resp1和resp2的间隔，根据通信速率不同而不同，单位us
uint16 ant_dly = ANT_DLY_DEFAULT;   // 天线延时
uint32 tx_power;                    // 发射增益代码
int32 distance_offset_cm;           // 距离校准，单位cm

sfConfig_t sfConfig = {
    .numSlots = MAX_TAG_NUMBER + 2, // 最大slot数量，标签数量+2个基站slot
    .slotDuration_ms = 12,          // 单slot时间，单位ms
    .sfPeriod_ms = (MAX_TAG_NUMBER + 2) * 20,  // 整个superframe周期时间，单位ms
    .tagPeriod_ms = (MAX_TAG_NUMBER + 2) * 20, // 标签测距周期时间，单位ms,先设置为跟superframe周期时间一致，后续tag睡眠唤醒功能要使用
    .pollTxToFinalTxDly_us = 1300 + 3 * 1600,  // poll发送到final发送的延时，单位us
};  // super frame 配置，针对不同通信速率，不同的基站部署个数，选择不同的配置

#if defined(ANCRANGE)
uint32_t sframePeriod_ms;
uint32_t a2aStartTime_ms;
static uint32_t a2aSlotOffset_ms;
#endif
#if defined(USE_DW3000)
static dwt_txconfig_t txconfig_options = {
    .PGdly = 0x34,
    .power = TX_POWER,
    .PGcount = 0
};
#elif defined(USE_DW1000)
static dwt_txconfig_t txconfig_options = {
    .PGdly = 0xC2,
    .power = TX_POWER
};
#endif

/*
 * Configuration options for the following parameters:
 * Channel: 5, 9
 * PRF: 64
 * Preamble Length: 64, 128, 512, 1024
 * Preamble Code: 3/4 for 16MHz PRf, 9/10/11/12 for 64MHz PRF
 * Data Rate: 0.85, 6.8
 * STS: Length 64
 */
#if defined(USE_DW3000)
/* dw3000 rf 配置 channel9（符合 UWB 新国标）
 * DW3000 不支持 110K 速率；PRF 由 txCode/rxCode 隐含（code 9~24 = 64MHz）
 * sfdType: 0=IEEE 8bit, 1=DW 8bit, 2=DW 16bit, 3=4z BPRF
 */
static dwt_config_t uwb_config_channel9[] = {
    {   /* uwb_config0，channel9 PRF64M 前导码256 数据率 850K */
        .chan = 9,
        .txPreambLength = DWT_PLEN_256,
        .rxPAC = DWT_PAC16,
        .txCode = 10,
        .rxCode = 10,
        .sfdType = 1,
        .dataRate = DWT_BR_850K,
        .phrMode = DWT_PHRMODE_STD,
        .phrRate = DWT_PHRRATE_STD,
        .sfdTO = (257 + DWT_SFD_LEN8 - 16),
        .stsMode = DWT_STS_MODE_OFF,
        .stsLength = DWT_STS_LEN_64,
        .pdoaMode = DWT_PDOA_M0
    },
    {   /* uwb_config1，channel9 PRF64M 前导码512 数据率 850K */
        .chan = 9,
        .txPreambLength = DWT_PLEN_512,
        .rxPAC = DWT_PAC16,
        .txCode = 10,
        .rxCode = 10,
        .sfdType = 1,
        .dataRate = DWT_BR_850K,
        .phrMode = DWT_PHRMODE_STD,
        .phrRate = DWT_PHRRATE_STD,
        .sfdTO = (513 + DWT_SFD_LEN8 - 16),
        .stsMode = DWT_STS_MODE_OFF,
        .stsLength = DWT_STS_LEN_64,
        .pdoaMode = DWT_PDOA_M0
    },
    {   /* uwb_config2，channel9 PRF64M 前导码1024 数据率 850K，主要使用 */
        .chan = 9,
        .txPreambLength = DWT_PLEN_1024,
        .rxPAC = DWT_PAC32,
        .txCode = 10,
        .rxCode = 10,
        .sfdType = 1,
        .dataRate = DWT_BR_850K,
        .phrMode = DWT_PHRMODE_STD,
        .phrRate = DWT_PHRRATE_STD,
        .sfdTO = (1025 + DWT_SFD_LEN8 - 32),
        .stsMode = DWT_STS_MODE_OFF,
        .stsLength = DWT_STS_LEN_64,
        .pdoaMode = DWT_PDOA_M0
    },
};

/* dw3000 rf 配置 channel5（6.5GHz，与 DW1000 ch5 设备互通）
 * channel5 @ PRF64M 合法前导码为 9~12，取10
 * 其余参数与 channel9 保持一致
 */
static dwt_config_t uwb_config_channel5[] = {
    {   /* uwb_config0，channel5 PRF64M 前导码长度1024 数据率 850K，主要使用 */
        .chan = 5,
        .txPreambLength = DWT_PLEN_1024,
        .rxPAC = DWT_PAC32,
        .txCode = 10,
        .rxCode = 10,
        .sfdType = 2,      // choose 2 for DW 16-bit，non standard
        .dataRate = DWT_BR_850K,
        .phrMode = DWT_PHRMODE_STD,
        .phrRate = DWT_PHRRATE_STD,
        .sfdTO = (1025 + DWT_SFD_LEN16 - 32),
        .stsMode = DWT_STS_MODE_OFF,
        .stsLength = DWT_STS_LEN_64,
        .pdoaMode = DWT_PDOA_M0
    },
    {   /* uwb_config1，channel5 PRF64M 前导码长度1024 数据率 850K，主要使用 */
        .chan = 5,
        .txPreambLength = DWT_PLEN_1024,
        .rxPAC = DWT_PAC32,
        .txCode = 10,
        .rxCode = 10,
        .sfdType = 2,      // choose 2 for DW 16-bit，non standard
        .dataRate = DWT_BR_850K,
        .phrMode = DWT_PHRMODE_STD,
        .phrRate = DWT_PHRRATE_STD,
        .sfdTO = (1025 + DWT_SFD_LEN16 - 32),
        .stsMode = DWT_STS_MODE_OFF,
        .stsLength = DWT_STS_LEN_64,
        .pdoaMode = DWT_PDOA_M0
    },
};
#elif defined(USE_DW1000)
/* dw1000 rf 配置 channel5 */
static dwt_config_t uwb_config_channel5[7] = {
    {   /* uwb_config0，channel5 脉冲频率64M 前导码长度256 数据率 850K，该配置还算稳定，先前一直长期使用 */
        .chan = 5,
        .prf = DWT_PRF_64M,
        .txPreambLength = DWT_PLEN_256,
        .rxPAC = DWT_PAC16,
        .txCode = 10,
        .rxCode = 10,
        .nsSFD = 1,
        .dataRate = DWT_BR_850K,
        .phrMode = DWT_PHRMODE_STD,
        .sfdTO = (257 + DW_NS_SFD_LEN_850K - 16)
    },
    {    /* uwb_config1，channel5 脉冲频率64M 前导码长度512 数据率 850K */
        .chan = 5,
        .prf = DWT_PRF_64M,
        .txPreambLength = DWT_PLEN_512,
        .rxPAC = DWT_PAC16,
        .txCode = 10,
        .rxCode = 10,
        .nsSFD = 1,
        .dataRate = DWT_BR_850K,
        .phrMode = DWT_PHRMODE_STD,
        .sfdTO = (513 + DW_NS_SFD_LEN_850K - 16)
    },
    {   /* uwb_config2，channel5 脉冲频率64M 前导码长度1024 数据率 850K ，该配置人体遮挡情况下，表现较好，偶有距离大跳情况，考虑软件优化，主要使用 */
        .chan = 5,
        .prf = DWT_PRF_64M,
        .txPreambLength = DWT_PLEN_1024,
        .rxPAC = DWT_PAC32,
        .txCode = 10,
        .rxCode = 10,
        .nsSFD = 1,
        .dataRate = DWT_BR_850K,
        .phrMode = DWT_PHRMODE_STD,
        .sfdTO = (1025 + DW_NS_SFD_LEN_850K - 32)
    },
};
#endif

// Implemented UWB algoritm. The dummy one is at the end of this file.
static uwbAlgorithm_t dummy_Algorithm;
static uwbAlgorithm_t *current_Algorithm = &dummy_Algorithm;
extern uwbAlgorithm_t uwbTwr_AnchorAlgorithm;
static dwDevice_t   dw1000_dev;      // 定义dw1000设备，twr使用
static dwDistance_t distance_data;   // 定义距离管理

typedef struct {
    uwbAlgorithm_t *algorithm;
    uwbChipType     chipType;
    const char     *name;
} uwbAlgorithmEntry_t;

const uwbAlgorithmEntry_t availableAlgorithms[] = {
    {.algorithm = &uwbTwr_AnchorAlgorithm, .chipType = UWB_CHIP_DW1000, .name = "TWR ANCHOR DW1000"},
    {.algorithm = &uwbTwr_AnchorAlgorithm, .chipType = UWB_CHIP_DW3000, .name = "TWR ANCHOR DW3000"},
    {NULL, 0, NULL}
};

osSemaphoreId_t sema_uwbInt;                                   // 用于dw1000中断同步
StaticSemaphore_t sema_uwbInt_cb;
const osSemaphoreAttr_t sema_uwbInt_attr = {
    .name = "sema_uwbInt", .cb_mem = &sema_uwbInt_cb, .cb_size = sizeof(sema_uwbInt_cb)
};

osSemaphoreId_t sema_dw1000Write;
StaticQueue_t sema_dw1000Write_cb;                        
const osSemaphoreAttr_t sema_dw1000Write_attr = {
    .name = "sema_dw1000Write", .cb_mem  = &sema_dw1000Write_cb, .cb_size = sizeof(sema_dw1000Write_cb)
};

osSemaphoreId_t sema_dw1000Read;
StaticQueue_t sema_dw1000Read_cb;                    
const osSemaphoreAttr_t sema_dw1000Read_attr = {
    .name = "sema_dw1000Read", .cb_mem  = &sema_dw1000Read_cb, .cb_size = sizeof(sema_dw1000Read_cb)
};

osSemaphoreId_t   sema_tagDistClear;        // 该信号量用于周期性采集最小值处理与定时器中断同步
StaticSemaphore_t sema_tagDistClear_cb;
const osSemaphoreAttr_t sema_tagDistClear_attr = {
    .name = "sema_tagDistClear", .cb_mem = &sema_tagDistClear_cb, .cb_size = sizeof(sema_tagDistClear_cb)
};

osMessageQueueId_t queue_processDis;       // 该队列用于传递距离给排序处理用
tag_hashNode_t queue_processDis_buf[16];   // 初始化队列的存储空间
StaticQueue_t queue_processDis_cb;         // 初始化队列控制块存储空间
const osMessageQueueAttr_t queue_processDis_attr = {
    .name = "queue_processDis",
    .mq_mem = &queue_processDis_buf, .mq_size = sizeof(queue_processDis_buf), .cb_mem = &queue_processDis_cb, .cb_size = sizeof(queue_processDis_cb),
};

osMessageQueueId_t queue_uwbEvent;
uwbEvent_t queue_uwbEvent_buf[8];
StaticQueue_t queue_uwbEvent_cb;
const osMessageQueueAttr_t queue_uwbEvent_attr = {
    .name = "queue_uwbEvent",
    .mq_mem = &queue_uwbEvent_buf, .mq_size = sizeof(queue_uwbEvent_buf), .cb_mem = &queue_uwbEvent_cb, .cb_size = sizeof(queue_uwbEvent_cb),
};

TIM_HandleTypeDef timerForInvaildDistanceClearHandle;
/*******************************************************静态函数声明********************************************************/
static void distance_init(dwDistance_t *data);
static void dwDevice_init(dwDevice_t *dev);
static void txcallback(const dwt_cb_data_t *cb_data);
static void rxcallback(const dwt_cb_data_t *cb_data);
static void rxTimeoutCallback(const dwt_cb_data_t *cb_data);
static void rxfailedcallback(const dwt_cb_data_t *cb_data);

static uwbAlgorithm_t* findAlgorithmByChip(uwbChipType chip)
{
    for (int i = 0; availableAlgorithms[i].algorithm != NULL; i++)
    {
        if (availableAlgorithms[i].chipType == chip)
        {
            return availableAlgorithms[i].algorithm;
        }
    }
    return &dummy_Algorithm;
}

static uint16_t dev_GetDefaultAntDly(void)
{
#if defined(USE_DW3000)
    return ANT_DLY_DW3000;
#elif defined(USE_DW1000)
    return ANT_DLY_DW1000;
#else
    return ANT_DLY_DEFAULT;
#endif
}

/* Unify the app-facing TX start semantics.
 * DW1000 only supports immediate/delayed start plus wait-for-response.
 * DW3000 extends this with reference/RX/TX timestamp delayed modes and CCA. */
int dev_uwbStartTx(uwb_tx_mode_t mode, bool response_expected)
{
    uint8_t driver_mode = 0;

#if defined(USE_DW3000)
    switch (mode)
    {
    case UWB_TX_MODE_IMMEDIATE:
        driver_mode = DWT_START_TX_IMMEDIATE;
        break;
    case UWB_TX_MODE_DELAYED:
        driver_mode = DWT_START_TX_DELAYED;
        break;
    case UWB_TX_MODE_DELAYED_REF:
        driver_mode = DWT_START_TX_DLY_REF;
        break;
    case UWB_TX_MODE_DELAYED_RX_TS:
        driver_mode = DWT_START_TX_DLY_RS;
        break;
    case UWB_TX_MODE_DELAYED_TX_TS:
        driver_mode = DWT_START_TX_DLY_TS;
        break;
    case UWB_TX_MODE_CCA:
        driver_mode = DWT_START_TX_CCA;
        break;
    default:
        return DWT_ERROR;
    }
#elif defined(USE_DW1000)
    switch (mode)
    {
    case UWB_TX_MODE_IMMEDIATE:
        driver_mode = DWT_START_TX_IMMEDIATE;
        break;
    case UWB_TX_MODE_DELAYED:
        driver_mode = DWT_START_TX_DELAYED;
        break;
    default:
        return DWT_ERROR;
    }
#endif

    if (response_expected)
    {
        driver_mode |= DWT_RESPONSE_EXPECTED;
    }

    return dwt_starttx(driver_mode);
}

static float calc_length_data(float msgdatalen)
{
    int x = 0;

    /* 
        根据802.15.4 UWB PHY 规定,PSDU 数据要经过 RS(63,55) 编码:
        每 330 bits 数据为一块, 每块附加 48 bits 校验
    */
    x = (int)ceil(msgdatalen * 8.0f / 330.0f); // 不足330bits 的尾块也要按一整块加 48 bits，使用ceil函数向上取整
    msgdatalen = msgdatalen * 8.0f + x * 48.0f; // 计算总的编码后数据长度，单位bits，每个330bits数据块加上48bits校验码

    // Assume PHR length is 172308ns for 110k and 21539ns for 850k/6.8M.
    if (inst_dataRate == DWT_BR_110K)
    {
        msgdatalen *= 8205.13f;
        msgdatalen += 172308.0f;
    }
    else if (inst_dataRate == DWT_BR_850K)
    {
        msgdatalen *= 1025.64f; // 以850K为例，计算出来的总的bit数，乘上每个bit占用的时间
        msgdatalen += 21539.0f; // 加上PHR的占用时间
    }
    else
    {
        msgdatalen *= 128.21f;
        msgdatalen += 21539.0f;
    }

    return msgdatalen;         // 返回计算完成的的air time，单位ns
}

static void dev_uwbCommonPreInit(dwDevice_t *dev)
{
    distance_init(get_the_local_structure_of_dis());
    dwDevice_init(dev);

#if defined(USE_DW3000)
    current_Algorithm = findAlgorithmByChip(UWB_CHIP_DW3000);
#elif defined(USE_DW1000)
    current_Algorithm = findAlgorithmByChip(UWB_CHIP_DW1000);
#endif

    sema_uwbInt       = osSemaphoreNew(1, 0, &sema_uwbInt_attr);
    sema_dw1000Write  = osSemaphoreNew(1, 0, &sema_dw1000Write_attr);
    sema_dw1000Read   = osSemaphoreNew(1, 0, &sema_dw1000Read_attr);
    sema_tagDistClear = osSemaphoreNew(1, 0, &sema_tagDistClear_attr);
    queue_uwbEvent    = osMessageQueueNew(8, sizeof(uwbEvent_t), &queue_uwbEvent_attr);
    queue_processDis  = osMessageQueueNew(16, sizeof(tag_hashNode_t), &queue_processDis_attr);
}

static void dev_uwbCommonPostInit(dwDevice_t *dev)
{
    if (dev->device_mode == ANCHOR)
    {
        uint16_t anc_short_add = 0x8000 | dev->device_id;
        dwt_setaddress16(anc_short_add);
        anc_id = dev->device_id;
        if (anc_id == 0)
        {
            group_id = group_id | 0x80;
        }
        dwt_forcetrxoff();
    }
    else
    {
        dwt_setaddress16(dev->device_id);
        tag_id = dev->device_id;
        dwt_forcetrxoff();
    }

    drv_setTimerForInt(&timerForInvaildDistanceClearHandle, TIM2, 20, 5);
    current_Algorithm->init(&dw1000_dev);
    disManager_memPoolInit();

#if defined(ANCRANGE)
    sframePeriod_ms = (MAX_TAG_NUMBER + 1) * inst_one_slot_time;
    a2aSlotOffset_ms = MAX_TAG_NUMBER * inst_one_slot_time;
    uint32_t now = portGetTickCnt();
    a2aStartTime_ms = now - (now % sframePeriod_ms) + a2aSlotOffset_ms + 5 * sframePeriod_ms;
#endif

    logOut_dw1000Config();
}

#if defined(USE_DW3000)
static void dev_dw3000Init(dwDevice_t *dev)
{
    dwt_config_t *current_rfConfig = &uwb_config_channel5[0];

    port_set_dw_ic_spi_fastrate();
    uint32_t chip_id = dwt_readdevid();
    SEGGER_RTT_printf(0, "[DW3000] chip_id=0x%08lx (expect 0xDECA0302 or 0xDECA0312)\r\n", chip_id);    
    board_dw3000Rst();
    while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
    { };
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        Error_Handler();
    }
    dwt_configure(current_rfConfig);
    
    inst_slot_number = MAX_TAG_NUMBER;
    inst_dataRate = current_rfConfig->dataRate;
    inst_ch       = current_rfConfig->chan;

    /* 配置通信相关时序 */
    if (inst_dataRate == DWT_BR_6M8)
    {
        inst_one_slot_time    = ONE_SLOT_TIME_MS_6P8M;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_6P8M;
        inst_resp_rx_timeout  = RESP_RX_TIMEOUT_6P8M;
        inst_data_interval    = DATA_INTERVAL_TIME_6P8M;
        inst_poll2final_time  = ((FIRST_RESP_SEND_6P8M + MAX_AHCHOR_NUMBER * inst_data_interval) * UUS_TO_DWT_TIME);
    }
    else if (inst_dataRate == DWT_BR_850K)
    {
        inst_one_slot_time    = ONE_SLOT_TIME_MS_850K;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_850K;
        inst_resp_rx_timeout  = RESP_RX_TIMEOUT_850K;
        inst_data_interval    = DATA_INTERVAL_TIME_850K;
        inst_poll2final_time  = ((FIRST_RESP_SEND_850K + MAX_AHCHOR_NUMBER * inst_data_interval) * UUS_TO_DWT_TIME);
    }

    txconfig_options.power = TX_POWER;
    tx_power = txconfig_options.power;
    dwt_configuretxrf(&txconfig_options);

    ant_dly = dev_GetDefaultAntDly();
    dwt_setrxantennadelay(ant_dly);
    dwt_settxantennadelay(ant_dly);

    dwt_setpanid(PAN_ID);
    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN | DWT_FF_ACK_EN);

    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
    dwt_setfinegraintxseq(0);

    // dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT
    //                | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO, 0, DWT_ENABLE_INT);
    
    if (dev->device_mode == ANCHOR)
    {
        dwt_setcallbacks(&txcallback, &rxcallback, &rxTimeoutCallback, &rxfailedcallback, NULL, NULL);
    }
    /* 必须使能 ARFE（帧过滤拒绝）中断：与 DW1000 路径(DWT_INT_ARFE)对齐。
     * 空闲等 poll 时帧过滤是开的，若漏掉一次 poll、又收到另一基站发给标签的 resp 帧，
     * DW3000 会把该帧按 RX-error 拒绝并关闭接收机；不使能 ARFE 则没有任何回调，
     * 接收机被静默关闭、再也收不到后续 poll —— 这正是“双基站跑一阵后一个基站停收发”的根因。
     * ARFE 属于 SYS_STATUS_ALL_RX_ERR 且经 FINT_STAT_RXERR 路由到 rxfailedcallback，
     * 会被 dwt_isr 正常清除，不会造成中断线卡死。 */
    dwt_setinterrupt(SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK |
                     SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK |
                     SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK |
                     SYS_ENABLE_LO_ARFE_ENABLE_BIT_MASK, 0, DWT_ENABLE_INT);
    port_set_dwic_isr(dwt_isr);
}
#elif defined(USE_DW1000)
static void dev_dw1000Init(dwDevice_t *dev)
{
    dwt_config_t *current_rfConfig = &uwb_config_channel5[2];

    board_dw1000Rst();
    port_set_dw1000_slowrate();
    if (0xDECA0130 != dwt_readdevid())
    {
        board_dw1000SlowWakeup();
        dwt_softreset();
    }
    board_dw1000Rst();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        Error_Handler();
    }
    port_set_dw1000_fastrate();

    inst_slot_number = MAX_TAG_NUMBER;

    dwt_configure(current_rfConfig);
    inst_dataRate = current_rfConfig->dataRate;
    inst_ch       = current_rfConfig->chan;

    /* 配置通信相关时序 */
    if (inst_dataRate == DWT_BR_6M8)
    {
        inst_one_slot_time    = ONE_SLOT_TIME_MS_6P8M;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_6P8M;
        inst_resp_rx_timeout  = RESP_RX_TIMEOUT_6P8M;
        inst_data_interval    = DATA_INTERVAL_TIME_6P8M;
        inst_poll2final_time  = ((FIRST_RESP_SEND_6P8M + MAX_AHCHOR_NUMBER * inst_data_interval) * UUS_TO_DWT_TIME);
    }
    else if (inst_dataRate == DWT_BR_110K)
    {
        inst_one_slot_time    = ONE_SLOT_TIME_MS_110K;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_110K;
        inst_resp_rx_timeout  = RESP_RX_TIMEOUT_110K;
        inst_data_interval    = DATA_INTERVAL_TIME_110K;
        inst_poll2final_time  = ((FIRST_RESP_SEND_110K + MAX_AHCHOR_NUMBER * inst_data_interval) * UUS_TO_DWT_TIME);
    }
    else if (inst_dataRate == DWT_BR_850K)
    {
        inst_one_slot_time    = ONE_SLOT_TIME_MS_850K;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_850K;
        inst_resp_rx_timeout  = RESP_RX_TIMEOUT_850K;
        inst_data_interval    = DATA_INTERVAL_TIME_850K;
        inst_poll2final_time  = ((FIRST_RESP_SEND_850K + MAX_AHCHOR_NUMBER * inst_data_interval) * UUS_TO_DWT_TIME);
    }

    txconfig_options.power = TX_POWER;
    tx_power = txconfig_options.power;
    dwt_configuretxrf(&txconfig_options);

    ant_dly = dev_GetDefaultAntDly();
    dwt_setrxantennadelay(ant_dly);
    dwt_settxantennadelay(ant_dly);

    dwt_setpanid(PAN_ID);
    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);
    dwt_setlnapamode(1, 1);
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT
                   | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_RFTO | DWT_INT_RXPTO, 1);

    if (dev->device_mode == ANCHOR)
    {
        dwt_setcallbacks(&txcallback, &rxcallback, &rxTimeoutCallback, &rxfailedcallback);
    }
}
#endif

void dev_uwbInit(void)
{
    dwDevice_t *dev = get_the_local_structure_of_dev();
    dev_uwbCommonPreInit(dev);

#if defined(USE_DW3000)
    dev_dw3000Init(dev);
#elif defined(USE_DW1000)
    dev_dw1000Init(dev);
#endif

    dev_uwbCommonPostInit(dev);
}

void logOut_dw1000Config(void)
{
}

/**
 * @brief uwb中断处理任务
 * @param  void *arg RTOS要求参数为空指针类型
 * @retval none
 */
void task_uwb(void *arg)
{
    UNUSED(arg);
    dev_uwbInit();
#if defined(USE_DW3000)
    board_dw3000IRQInit();
#elif defined(USE_DW1000)
    board_dw1000IRQInit();
#endif
    for (;;)
    {   
        osStatus_t status = osSemaphoreAcquire(sema_uwbInt, osWaitForever); // 采用中断触发的方式执行，获取信号量
        if (status == osOK)
        {
            process_deca_irq();
        }
    }
}

/**
 * @brief uwb双边测距处理任务，中断处理发送任务通知，通过接收不同类型任务通知，执行对应的回调函数
 * @param  void *arg RTOS要求参数为空指针类型
 * @retval none
 */
void task_twrRun(void *arg)
{
    UNUSED(arg);
    uwbEvent_t evt;
    for (;;)
    {
#if defined(ANCRANGE)
        osStatus_t status = osMessageQueueGet(queue_uwbEvent, &evt, NULL, sframePeriod_ms);
#else
        osStatus_t status = osMessageQueueGet(queue_uwbEvent, &evt, NULL, osWaitForever);
#endif
        if (status == osOK)
        {
            switch (evt)
            {
            case eventPacketSent:
                (void) current_Algorithm->onEvent(&dw1000_dev, eventPacketSent);
                break;

            case eventPacketReceived:
                (void) current_Algorithm->onEvent(&dw1000_dev, eventPacketReceived);
                break;

            case eventReceiveTimeout:
                (void) current_Algorithm->onEvent(&dw1000_dev, eventReceiveTimeout);
                break;

            case eventReceiveFailed:
                (void) current_Algorithm->onEvent(&dw1000_dev, eventReceiveFailed);
                break;

            default:
                break;
            }
        }
#if defined(ANCRANGE)
        anch_checkA2ATrigger(&dw1000_dev);
#endif
    }
}

static tag_hashNode_t recv_processedDis = {0};
static distance_manager_t task_dis_manage = {0};

distance_manager_t *disManager_getHandle(void)
{
    return &task_dis_manage;
}

/**
 * @brief 最小堆管理任务，插入，更新，删除
 * @param  void *arg RTOS要求参数为空指针类型
 * @retval none
 */
void task_minHeapManage(void *arg)
{
    UNUSED(arg);
    
    for (;;)
    {
        osStatus_t status = osMessageQueueGet(queue_processDis, &recv_processedDis, 0, osWaitForever);
        // 接收到计算完成的距离，以及TWR完成时的tick数，进行插入以及更新，同时只要有标签TWR成功，就进行排序，最小距离发送
        if (status == osOK)    
        {
            uint32_t current_tick = osKernelGetTickCount();

            disManager_update(&task_dis_manage, recv_processedDis.tag_id, recv_processedDis.distance, recv_processedDis.last_updateTick);

            // 堆更新后，清除无效距离，获取当前tick，将超时的标签清除
            disManager_purgeExpired(&task_dis_manage, current_tick);
        }
    }
}

void task_getMinDis(void *arg)
{
    UNUSED(arg);
    static outDistance_t min_selfDis = {0};
    static tag_hashNode_t *min_dis_node = NULL;
    static uint32_t lastLogTick = 0;
    extern osMessageQueueId_t queue_minimalDis;

    for (;;)
    {
        osStatus_t status = osSemaphoreAcquire(sema_tagDistClear, osWaitForever);
        if (status == osOK)
        {
            uint32_t now = osKernelGetTickCount();
            disManager_purgeExpired(&task_dis_manage, now);
            min_dis_node = disManager_getMin(&task_dis_manage);

            if (min_dis_node == NULL)
            {
                min_selfDis.dis_value = 2000000;
                min_selfDis.dis_index = 0xFF;
                min_selfDis.dis_class = ANCHOR_SELF_DIS;
                osMessageQueuePut(queue_minimalDis, &min_selfDis, 0, 0);
            }
            else
            {
                min_selfDis.dis_value = min_dis_node->distance;
                min_selfDis.dis_index = min_dis_node->tag_id;
                min_selfDis.dis_class = ANCHOR_SELF_DIS;
                osMessageQueuePut(queue_minimalDis, &min_selfDis, 0, 0);
            }

            if ((now - lastLogTick) >= 2000)
            {
                lastLogTick = now;
                disManager_logSummary(&task_dis_manage);
            }
        }
    }
}

#if defined(USE_DW1000)
float uwb_isInNLOS_power(dwt_rxdiag_t *rx_diag, uint8_t receive_functionCode)
{
    // 先使用功率判别法
    uint8_t functionCode = receive_functionCode;
    float nlos_rssiThershold = 6.0; // NLOS判断阈值，单位dBm，根据实际情况调整？
    float rx_power = 0;
    float fpPower  = 0;
    float diff     = 0;

    dwt_readdiagnostics(rx_diag);
    rx_power = rx_diag->rxPower;
    fpPower  = rx_diag->fpPower;
    diff = rx_power - fpPower;
    log_d("receive %x, RX_POWER: %.2f dBm, FP_POWER: %.2f dBm, DIFF: %.2f dB", functionCode, rx_power, fpPower, diff);

    if (diff > nlos_rssiThershold)
    {
        return 1; // 接收功率跟第一路径功率的差值比阈值大，判断为NLOS情况
    }
    else
    {
        return 0; // LOS
    }
}

uint16_t uwb_isInNLOS_index(uint8_t receive_functionCode)
{
    uint8_t functionCode = receive_functionCode;
    uint16_t rx_time_fp = 0;
    uint16_t lde_fpIndex = 0;
    uint8_t peak_reg[2] = {0};
    uint16_t peak_index = 0;
    int sample_windowSize = 20;

    dwt_readfromdevice(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET, 2, (uint8_t*)&rx_time_fp);
    lde_fpIndex = rx_time_fp >> 6;

    int sample_startIndex = lde_fpIndex - (sample_windowSize / 2);

    // dwt_readdiagnostics(&rxDiag);

    if (sample_startIndex < 0)
    {
        sample_startIndex = 0; // 越界则从0开始读
    }

    dwt_readfromdevice(LDE_IF_ID, LDE_PPINDX_OFFSET, LDE_PPINDX_LEN, peak_reg);
    peak_index = peak_reg[0] | (peak_reg[1] << 8);
    uint16_t index_diff = abs(lde_fpIndex - peak_index);

    // log_d("receive %x, RX_PEAK_INDEX = %d, FP_INDEX = %d, Index diff = %d.",functionCode, peak_index, lde_fpIndex, index_diff);
    return index_diff;
}
#elif defined(USE_DW3000)
/* TODO: DW3000 NLOS 检测，寄存器和诊断结构体与 DW1000 完全不同，后续实现 */
#endif

int check_twr_quality(uint16_t indexDiff_poll, uint16_t indexDiff_final)
{
    uint16_t avg_indexDiff = (indexDiff_poll + indexDiff_final) / 2;
    uint16_t delta_indexDiff = abs(indexDiff_poll - indexDiff_final);

    // log_d("TWR quality check, poll indexDiff = %d, final indexDiff = %d, avg = %d, delta = %d.", indexDiff_poll, indexDiff_final, avg_indexDiff, delta_indexDiff);

    if (avg_indexDiff > 6 || delta_indexDiff > 4)
    {
        return -1; // 拒绝此次TWR测距结果，沿用上次距离
    }
    
    if (avg_indexDiff <= 2 || delta_indexDiff <= 1)
    {
        return 1; // 本次TWR测距结果良好
    }

    return 0;     // 本次TWR测距结果一般
}

dwDistance_t *get_the_local_structure_of_dis(void)
{
    return &distance_data;
}

dwDevice_t *get_the_local_structure_of_dev(void)
{
    return &dw1000_dev;
}

static void distance_init(dwDistance_t *data)
{
    data->dis_idx = 0;
    data->disMsg->dis_class = 0;
    data->min_dis = 2000000;
}

/******************************************************Dw1000 Device************************************************************/
static void dwDevice_init(dwDevice_t *dev)
{
    dev->device_mode = ANCHOR;
    dev->twr_mode = LISTENER;
    /* A2A/TWR 角色依赖真实基站 ID。
     * DW1000 也需要和 DW3000 一样从拨码读取，否则所有基站都会被当成 A0。 */
    dev->device_id = dev_getDipVal();
    dev->remainingRespToRx = -1;     // 初始化为 -1
    dev->rxOtherResp = 0;            // 接收其他基站resp帧计数
    dev->respTxIndex = 0;            // 该变量用于决定基站发送resp帧的位置，跟基站自身ID相关
#if defined(USE_DW3000)
    board_dw3000Init();
#elif defined(USE_DW1000)
    board_dw1000Init();
#endif
    SEGGER_RTT_printf(0, "\r\nThe Anchor ID : %d.\r\n", dev->device_id);  
}

/******************************************************interrupt use callback function************************************************************/
extern osThreadId_t task_twrRun_handle;
static void txcallback(const dwt_cb_data_t *cb_data)
{
    UNUSED(cb_data);
    uwbEvent_t evt = eventPacketSent;
    osMessageQueuePut(queue_uwbEvent, &evt, 0, 0);
}

static void rxcallback(const dwt_cb_data_t *cb_data)
{
    UNUSED(cb_data);
    uwbEvent_t evt = eventPacketReceived;
    osMessageQueuePut(queue_uwbEvent, &evt, 0, 0);
}

static void rxTimeoutCallback(const dwt_cb_data_t *cb_data)
{
    UNUSED(cb_data);
    uwbEvent_t evt = eventReceiveTimeout;
    osMessageQueuePut(queue_uwbEvent, &evt, 0, 0);
}

static void rxfailedcallback(const dwt_cb_data_t *cb_data)
{
    UNUSED(cb_data);
    uwbEvent_t evt = eventReceiveFailed;
    osMessageQueuePut(queue_uwbEvent, &evt, 0, 0);
}

