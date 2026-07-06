#include "dw_instance.h"
#include "dw_sort.h"
#include "cmsis_os.h"
#include "elog.h"

/* 保存当前ID标签的测距值，下次发送resp时发给标签 */

/* RESP数据帧格式 */
static uint8_t tx_resp_msg[RESP_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0x00, 0x80, RTLS_MSG_ANCH_RESP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#if defined(ANCRANGE)
static uint8_t tx_poll_anchor_msg[ANCH_POLL_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x00, 0x80, RTLS_MSG_ANCH_POLL, 0x00};
static uint8_t tx_anch_resp2_msg[ANCH_RESP2_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x00, 0x80, RTLS_MSG_ANCH_RESP2, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t tx_anch_final_msg[ANCH_FINAL_MSG_LEN];

#define A2A_RESPONDER_COUNT            (MAX_AHCHOR_NUMBER - 1)
#define A2A_FINAL_SCHEDULE_INDEX       (A2A_RESPONDER_COUNT + 2)

static void anch_start_a2a(instance_data_t *inst, uint8_t expectedResps);
static void anch_a2a_sendFinal(instance_data_t *inst);
static void rnganch_change_back_to_anchor(instance_data_t *inst);
#endif

/* 接收数据buffer */
static uint8_t rx_buffer[FRAME_LEN_MAX];

/* 单次TWR成功后的距离缓存，投递 queue_processDis 用 */
static float distance_now_m;
static tag_hashNode_t send_processeDis;

/* 速率相关时序参数集，Phase B 将由 twr_set_replydelay() 统一公式计算取代，
 * 当前数值与原 6 处 if-else 阶梯完全一致 */
typedef struct {
    uint32_t first_resp_us;   // poll 后首个 resp 槽的基准延时
    uint32_t anc_back_us;     // 基站 resp 发送在槽基准上的再延后量
    uint32_t final_back_us;   // final 发送在槽基准上的再延后量
} rateTiming_t;

static int twrAnchor_Init(instance_data_t *inst);
static uint32_t twrAnchor_onEvent(instance_data_t *inst, uwbEvent_t event);
static void twrAnchor_rxOkHandle(instance_data_t *inst);
static void twrAnchor_sentHandle(instance_data_t *inst);
static uint8_t twrAnchor_rxErrorOrTimeoutHandle(instance_data_t *inst);
static void anch_respSlotProcess(instance_data_t *inst);
static void anch_rxRenableImmdiate(instance_data_t *inst);
static void anch_perpareAnc2TagResp(instance_data_t *inst);
static rateTiming_t rate_timing(void);
static inline uint64_t get_tx_timestamp_u64(void);
static inline uint64_t get_rx_timestamp_u64(void);
static inline void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);
static inline void final_msg_set_ts(uint8_t *ts_field, uint64_t ts);

static rateTiming_t rate_timing(void)
{
    rateTiming_t rt;
#if defined(USE_DW1000)
    if (inst_dataRate == DWT_BR_110K)
    {
        rt.first_resp_us = FIRST_RESP_SEND_110K;
        rt.anc_back_us   = ANC_RESP_SEND_BACK_110K;
        rt.final_back_us = TAG_FINALE_SEND_BACK_110K;
        return rt;
    }
#endif
    if (inst_dataRate == DWT_BR_6M8)
    {
        rt.first_resp_us = FIRST_RESP_SEND_6P8M;
        rt.anc_back_us   = ANC_RESP_SEND_BACK_6P8M;
        rt.final_back_us = TAG_FINALE_SEND_BACK_6P8M;
    }
    else
    {
        rt.first_resp_us = FIRST_RESP_SEND_850K;
        rt.anc_back_us   = ANC_RESP_SEND_BACK_850K;
        rt.final_back_us = TAG_FINALE_SEND_BACK_850K;
    }
    return rt;
}

// 单套 TWR anchor 算法，DW1000/DW3000 通过函数内 USE_DW1000/USE_DW3000 条件编译区分
uwbAlgorithm_t uwbTwr_AnchorAlgorithm = { .init = twrAnchor_Init, .onEvent = twrAnchor_onEvent };

/*****************************************CCCC**DW1000 event function********************************************/
static int twrAnchor_Init(instance_data_t *inst)
{
#if defined(USE_DW3000)
    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN | DWT_FF_ACK_EN);
#else
    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);  // 设置帧过滤模式开启
#endif
    dwt_setpreambledetecttimeout(0);                        // 清除前导码超时，一直接收
    dwt_setrxtimeout(0);                                       // 清除接收数据超时，一直接收
    int ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);            // 打开接收机，等待接收数据，初始接收，进行一次buffer对齐
    if(ret == DWT_ERROR)                                            // 打开接收失败
    {
        return 0;
    }
    inst->twr_mode = LISTENER;                                      // 空闲状态，等待接收 poll (tag 或 anchor)
    return 1;
}

static uint32_t twrAnchor_onEvent(instance_data_t *inst, uwbEvent_t event)
{
    switch (event)
    {
    case eventPacketReceived:
        twrAnchor_rxOkHandle(inst);
        break;

    case eventPacketSent:
        twrAnchor_sentHandle(inst);
        break;

    case eventReceiveFailed:
    case eventReceiveTimeout:
        twrAnchor_rxErrorOrTimeoutHandle(inst);
        break;

    default:
        break;
    }

    return portMAX_DELAY;
}

/****************************************************DW3000 event function************************************************/
/* DW3000 算法与 DW1000 共享 rx_buffer、send_processeDis 等静态变量，
 * 后续实现时可直接复用，无需重复定义 */

/****************************************************interrupt handle function************************************************/

static void twrAnchor_rxOkHandle(void)
{
    dwDevice_t* dev = get_the_local_structure_of_dev();
    uint32 frame_len;

#if defined(USE_DW3000)
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
#else
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
#endif
    
    if (frame_len < FRAME_LEN_MAX)
    {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    uint8_t f_code = rx_buffer[FUNC_CODE_IDX];

#if defined(ANCRANGE)
    if (dev->device_mode == ANCHOR_RNG && dev->twr_mode == INITIATOR
        && f_code != RTLS_MSG_ANCH_RESP2)
    {
        dwt_setrxtimeout(inst_data_interval + inst_resp_rx_timeout);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }
    if (dev->twr_mode == RESPONDER_A && f_code != RTLS_MSG_ANCH_FINAL)
    {
        /* RESPONDER_A only waits for FINAL. */
        dwt_setrxtimeout(inst_final_rx_timeout);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }
#endif

    switch (f_code)
    {
    case RTLS_MSG_TAG_POLL:
#if defined(ANCRANGE)
        if (dev->device_mode == ANCHOR_RNG || dev->twr_mode == RESPONDER_A)
        {
            anch_rxRenableImmdiate(dev);
            break;
        }
#endif
        dev->twr_mode = RESPONDER_T;
        range_nb    = rx_buffer[RANGE_NB_IDX];             // 取range_nb，resp发送时发送相同的range_nb
        recv_tag_id = rx_buffer[SENDER_SHORT_ADD_IDX];     // 取发送标签的ID
        if(recv_tag_id >= inst_slot_number)                // 标签ID如果大于标签总容量则退出
        {
            anch_rxRenableImmdiate(dev);           // 直接开启下一轮接收poll，无需同步buffer，暂定没问题
            break;
        }

        range_time = portGetTickCnt();            // 取得twr刚开始接收到poll的Tick
        poll_rx_ts = get_rx_timestamp_u64();      // 获得poll_rx时间戳    
        dev->wait4final = 0;                      // 等待final标志位
        dev->remainingRespToRx = MAX_AHCHOR_NUMBER - 1;         // 基站需要接收的其他基站的resp帧的数量
        handleResp_times = MAX_AHCHOR_NUMBER;                   // 基站需要处理resp帧的次数
        dev->respTxIndex = 0x01 << anc_id;        // 该标志位标识当前基站发送resp帧的位置，先右移，0号基站接收poll帧后，直接发送，1号基站延后一位
        anch_perpareAnc2TagResp();                // 可以在这里就将要发送的resp帧写入发送缓存
        anch_txRespOrRxReEnable();                // 判断是否发送resp帧，还是重新打开接收，接收其他基站的resp帧，还是重新打开接收，接收标签final帧
        break;

    case RTLS_MSG_TAG_FINAL:                      // 基站每接收到一个标签的final帧，就进行一次TOF计算
        if (((rx_buffer[RANGE_NB_IDX] == range_nb) && (rx_buffer[SENDER_SHORT_ADD_IDX] == recv_tag_id))) // 验证final帧的range_nb和标签ID是否和之前的poll帧一致
        {
            dev->twr_mode = LISTENER ;            // 接收到final不用答复，故设置为listener模式
            dev->wait4final = 0; 
            resp_valid = rx_buffer[FINAL_MSG_FINAL_VALID_IDX];
            range_time = portGetTickCnt();         // 再次获取TWR成功，final帧接收的Tick
            resp_tx_ts = get_tx_timestamp_u64();   // 取得resp_tx时间戳
            final_rx_ts = get_rx_timestamp_u64();  // 取得final_rx时间戳

            if((resp_valid >> anc_id) & 0x01)      // final消息中，本基站发送的resp消息是有效的,则进行距离计算，或者发送时间戳
            {
                uint32_t poll_tx_ts_32, resp_rx_ts_32, final_tx_ts_32;
                uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                double Ra, Rb, Da, Db;
                int64_t tof_dtu;
                double tof;

                // /* 从final消息中，取得poll_tx时间戳，resp_rx时间戳，final_tx时间戳 */
                final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts_32);
                final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP1_RX_TS_IDX + anc_id * (FINAL_MSG_TS_LEN + 1)], &resp_rx_ts_32);
                final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts_32);

                /* 计算飞行时间 */
                poll_rx_ts_32  = (uint32_t)poll_rx_ts;
                resp_tx_ts_32  = (uint32_t)resp_tx_ts;
                final_rx_ts_32 = (uint32_t)final_rx_ts;
                Ra = (double)(resp_rx_ts_32 - poll_tx_ts_32);
                Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                Da = (double)(final_tx_ts_32 - resp_rx_ts_32);
                Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
                tof = (int32)tof_dtu; 
                if (tof > 0x7FFFFFFF)                       // 如果TOF值溢出
                {
                    tof -= 0x80000000;  
                }
                tof = tof_dtu * DWT_TIME_UNITS;
                distance_now_m = tof * SPEED_OF_LIGHT;      // 计算本次TWR测距的结果，单位为m
#if defined(USE_DW1000)
                distance_now_m = distance_now_m - dwt_getrangebias(inst_ch, (float)distance_now_m, inst_prf);
#endif
                if(distance_now_m > 20000.000)              // 如果测距结果大于20km，则认为测距失败
                {
                    distance_now_m = -1;
                }
                /* 保存本次测距值(mm)，下个周期在 resp 的 PREV_DIS 字段回传给标签 */
                prev_range[recv_tag_id] = (int32_t)(distance_now_m * 1000);
                send_processeDis.distance = distance_now_m * 1000;
                send_processeDis.tag_id   = recv_tag_id;
                send_processeDis.last_updateTick = range_time;
                extern osMessageQueueId_t queue_processDis;
                osMessageQueuePut(queue_processDis, &send_processeDis, 0, 0);

                range_status = RANGE_TWR_OK;  
                dev_ledBlink(UWB_OK_LED);
            }
            else     
            {
                range_status = RANGE_ERROR;     // 本基站发送的resp消息无效，测距失败
            }
        }   
        dwt_forcetrxoff();                      // 该函数重置了host side receive buffer pointer，导致双buffer模式下，TWR测距只能成功一次，后续TWR测距均失败
        anch_rxRenableImmdiate(dev);            // 直接开启下一轮接收poll，无需同步buffer，因为dwt_ihandleResp_times中已经处理过buffer切换    
        break;

    case RTLS_MSG_ANCH_RESP:
        if(rx_buffer[RANGE_NB_IDX] == range_nb)      // 和当前测距具有相同的range_nb
        {   
            dev->rxOtherResp++;                     // 接收到一次其他基站发送的resp帧  
            
            if((rx_buffer[RESP_MSG_GROUP_IDX] & 0x7f) == (group_id & 0x7f)) // 只取和自己组号相同的其他基站数据上报
            {
                uint8_t recv_anc_id = rx_buffer[SENDER_SHORT_ADD_IDX];     //取基站ID
                distance_report[recv_anc_id]  = (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX]   << 24;
                distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+1] << 16;
                distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+2] << 8;
                distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+3];
                group_report[recv_anc_id] = rx_buffer[RESP_MSG_GROUP_IDX] & 0x7f;  // 将最高bit置0，最高bit为校准基站标志位
            } 
        }
        // 不管校验有没有通过，都要去处理后续的操作
        dev->remainingRespToRx--;          // 剩余接收resp帧数量减一
        anch_txRespOrRxReEnable();         // 判断是否发送resp帧，还是重新打开接收，接收其他基站的resp帧
        break;

#if defined(ANCRANGE)
    case RTLS_MSG_ANCH_POLL:
    {
        if (anc_id == 0)
        {
            anch_rxRenableImmdiate(dev);
            break;
        }
        uint8_t initiator_id = rx_buffer[SENDER_SHORT_ADD_IDX];
        dev->device_mode = ANCHOR;
        dev->twr_mode = RESPONDER_A;
        a2a_poll_rx_ts = get_rx_timestamp_u64();
        a2a_range_nb = rx_buffer[RANGE_NB_IDX];
        a2a_state = A2A_RESP_SENT;
        // log_d("A2A A%d rxd poll from A%d", anc_id, initiator_id);

        uint8_t resp_position = anc_id - initiator_id - 1;
        uint32_t a2a_first_resp_us, a2a_anc_back_us;
#if defined(USE_DW1000)
        if (inst_dataRate == DWT_BR_110K)
        {
            a2a_first_resp_us = FIRST_RESP_SEND_110K;
            a2a_anc_back_us   = ANC_RESP_SEND_BACK_110K;
        }
        else
#endif
        if (inst_dataRate == DWT_BR_6M8)
        {
            a2a_first_resp_us = FIRST_RESP_SEND_6P8M;
            a2a_anc_back_us   = ANC_RESP_SEND_BACK_6P8M;
        }
        else
        {
            a2a_first_resp_us = FIRST_RESP_SEND_850K;
            a2a_anc_back_us   = ANC_RESP_SEND_BACK_850K;
        }
        /* A2A 的第一个 responder 原来直接占用首个响应槽。
         * 现在接收/发送流程经过 IRQ -> 队列 -> task_twrRun，DW3000 上这个槽太紧，
         * 容易在 dwt_starttx() 时已经“过点”。
         * 因此整体后移一个 data interval，让 A1 从第二个响应槽开始发。 */
        /* Shift A1/A2 response slots later to leave enough task-level scheduling margin. */
        uint64_t resp_tx_time = a2a_poll_rx_ts
            + ((uint64_t)(a2a_first_resp_us + (resp_position + 1) * inst_data_interval) * UUS_TO_DWT_TIME)
            + ((uint64_t)a2a_anc_back_us * UUS_TO_DWT_TIME);
        /* Open FINAL RX from the slot boundary, leaving guard time before the actual FINAL preamble arrives. */
        a2a_final_rx_time = a2a_poll_rx_ts
            + ((uint64_t)(a2a_first_resp_us + A2A_FINAL_SCHEDULE_INDEX * inst_data_interval) * UUS_TO_DWT_TIME);

        tx_anch_resp2_msg[SEQ_NB_IDX] = frame_seq_nb++;
        tx_anch_resp2_msg[PANID_IDX] = (uint8_t)PAN_ID;
        tx_anch_resp2_msg[PANID_IDX + 1] = (uint8_t)(PAN_ID >> 8);
        tx_anch_resp2_msg[SENDER_SHORT_ADD_IDX] = anc_id;
        tx_anch_resp2_msg[RECEIVER_SHORT_ADD_IDX] = 0xFF;
        tx_anch_resp2_msg[FUNC_CODE_IDX] = RTLS_MSG_ANCH_RESP2;
        tx_anch_resp2_msg[RANGE_NB_IDX] = a2a_range_nb;
        int32_t prev_dis = a2a_distance[initiator_id];
        tx_anch_resp2_msg[A2A_RESP2_PREV_DIS_IDX]     = (uint8_t)(prev_dis);
        tx_anch_resp2_msg[A2A_RESP2_PREV_DIS_IDX + 1] = (uint8_t)(prev_dis >> 8);
        tx_anch_resp2_msg[A2A_RESP2_PREV_DIS_IDX + 2] = (uint8_t)(prev_dis >> 16);
        tx_anch_resp2_msg[A2A_RESP2_PREV_DIS_IDX + 3] = (uint8_t)(prev_dis >> 24);

        dwt_writetxdata(ANCH_RESP2_MSG_LEN + FCS_LEN, tx_anch_resp2_msg, 0);
        dwt_writetxfctrl(ANCH_RESP2_MSG_LEN + FCS_LEN, 0, 1);
        dwt_setdelayedtrxtime((uint32)(resp_tx_time >> 8));
        if (dev_uwbStartTx(UWB_TX_MODE_DELAYED, false) == DWT_ERROR)
        {
            log_d("A2A A%d resp2 starttx failed, init=A%d pos=%d rn=%d",
                    anc_id, initiator_id, resp_position, a2a_range_nb);
            rnganch_change_back_to_anchor(dev);
        }
        break;
    }

    case RTLS_MSG_ANCH_RESP2:                               // 接收基站发送的resp帧，获取上一轮的a2a测距结果，判断是否要发送final帧或者继续接收其他基站的resp帧
    {
        if (dev->device_mode != ANCHOR_RNG)
        {
            anch_rxRenableImmdiate(dev);
            break;
        }
        uint8_t resp_anc_id = rx_buffer[SENDER_SHORT_ADD_IDX];
        a2a_resp_rx_ts[resp_anc_id] = get_rx_timestamp_u64();
        a2a_rxRespMask |= (1 << resp_anc_id);
        a2a_remainingResp--;
        // log_d("A2A A%d rxd resp2 from A%d, remain=%d", anc_id, resp_anc_id, a2a_remainingResp);

        int32_t prev_dis = (int32_t)(rx_buffer[A2A_RESP2_PREV_DIS_IDX]
            | ((uint32_t)rx_buffer[A2A_RESP2_PREV_DIS_IDX + 1] << 8)
            | ((uint32_t)rx_buffer[A2A_RESP2_PREV_DIS_IDX + 2] << 16)
            | ((uint32_t)rx_buffer[A2A_RESP2_PREV_DIS_IDX + 3] << 24));
        if (anc_id == 0)
        {
            log_i("A2A: A%d rxd resp2 from A%d prev=%.2f m", anc_id, resp_anc_id, prev_dis / 1000.0);
        }
        if (prev_dis > 0)
        {
            a2a_distance[resp_anc_id] = prev_dis;
        }

        if (a2a_remainingResp == 0)         // 期望接收的a2a测距基站resp帧全部接收完了，发送final帧
        {
            anch_a2a_sendFinal(dev);
        }
        else
        {
            dwt_setrxtimeout(inst_data_interval + inst_resp_rx_timeout);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        break;
    }

    case RTLS_MSG_ANCH_FINAL:
    {
        if (dev->twr_mode != RESPONDER_A)
        {
            anch_rxRenableImmdiate(dev);
            break;
        }
        // log_d("A2A A%d rxd final", anc_id);
        uint8_t initiator_id = rx_buffer[SENDER_SHORT_ADD_IDX];
        uint8_t valid = rx_buffer[A2A_FINAL_VALID_IDX];
        if (!((valid >> anc_id) & 0x01))
        {
            log_d("A2A A%d final valid mask miss, mask=0x%02X", anc_id, valid);
            rnganch_change_back_to_anchor(dev);
            break;
        }

        uint32_t poll_tx_ts_32, final_tx_ts_32, resp_rx_ts_32;
        uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
        double Ra, Rb, Da, Db;
        int64_t tof_dtu;
        double tof;
        double dist_m;
        final_msg_get_ts(&rx_buffer[A2A_FINAL_POLL_TX_TS_IDX], &poll_tx_ts_32);
        final_msg_get_ts(&rx_buffer[A2A_FINAL_FINAL_TX_TS_IDX], &final_tx_ts_32);
        final_msg_get_ts(&rx_buffer[A2A_FINAL_RESP_RX_TS_BASE + (anc_id - 1) * FINAL_MSG_TS_LEN], &resp_rx_ts_32);

        a2a_resp_tx_ts = get_tx_timestamp_u64();
        uint64_t a2a_final_rx_ts = get_rx_timestamp_u64();
        poll_rx_ts_32  = (uint32_t)a2a_poll_rx_ts;
        resp_tx_ts_32  = (uint32_t)a2a_resp_tx_ts;
        final_rx_ts_32 = (uint32_t)a2a_final_rx_ts;

        Ra = (double)(resp_rx_ts_32 - poll_tx_ts_32);
        Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
        Da = (double)(final_tx_ts_32 - resp_rx_ts_32);
        Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
        tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
        tof = tof_dtu * DWT_TIME_UNITS;
        dist_m = tof * SPEED_OF_LIGHT;

// #if defined(USE_DW1000)
//         dist_m = dist_m - dwt_getrangebias(inst_ch, (float)dist_m, inst_prf);
// #endif
        // if (dist_m > 0 && dist_m < 20000.0)
        {
            a2a_distance[initiator_id] = (int32_t)(dist_m * 1000);
            log_i("A2A: A%d-A%d = %.2f m (responder)", initiator_id, anc_id, dist_m);
        }
        rnganch_change_back_to_anchor(dev);
        break;
    }
#endif

    default:
        break;
    }
}

static void twrAnchor_sentHandle(void)
{
#if defined(ANCRANGE)
    dwDevice_t *dev = get_the_local_structure_of_dev();
    if (dev->device_mode == ANCHOR_RNG && dev->twr_mode == INITIATOR)
    {
        if (a2a_state == A2A_POLL_SENT)
        {
            // log_d("A2A A%d poll sent", anc_id);
            a2a_poll_tx_ts = get_tx_timestamp_u64();
            {
                uint32_t a2a_first_resp_us, a2a_final_back_us;
#if defined(USE_DW1000)
                if (inst_dataRate == DWT_BR_110K)
                {
                    a2a_first_resp_us = FIRST_RESP_SEND_110K;
                    a2a_final_back_us = TAG_FINALE_SEND_BACK_110K;
                }
                else
#endif
                if (inst_dataRate == DWT_BR_6M8)
                {
                    a2a_first_resp_us = FIRST_RESP_SEND_6P8M;
                    a2a_final_back_us = TAG_FINALE_SEND_BACK_6P8M;
                }
                else
                {
                    a2a_first_resp_us = FIRST_RESP_SEND_850K;
                    a2a_final_back_us = TAG_FINALE_SEND_BACK_850K;
                }
                /* FINAL is pre-scheduled from POLL TX, after all RESP2 slots plus one extra guard slot. */
                a2a_final_tx_time = a2a_poll_tx_ts
                    + ((uint64_t)(a2a_first_resp_us + A2A_FINAL_SCHEDULE_INDEX * inst_data_interval) * UUS_TO_DWT_TIME)
                    + ((uint64_t)a2a_final_back_us * UUS_TO_DWT_TIME);
            }
            return;
        }
        if (a2a_state == A2A_FINAL_SENT)
        {
            // log_d("A2A A%d final sent", anc_id);
            rnganch_change_back_to_anchor(dev);
            return;
        }
    }
    if (dev->twr_mode == RESPONDER_A)
    {
        // log_d("A2A A%d resp2 sent", anc_id);
        a2a_resp_tx_ts = get_tx_timestamp_u64();
        /* Arm delayed RX after RESP2 really leaves the air. */
        dwt_setdelayedtrxtime((uint32)(a2a_final_rx_time >> 8));
        dwt_setrxtimeout(inst_final_rx_timeout);
        dwt_setpreambledetecttimeout(PRE_TIMEOUT);
        if (dwt_rxenable(DWT_START_RX_DELAYED) == DWT_ERROR)
        {
            log_d("A2A A%d final rx enable failed", anc_id);
            rnganch_change_back_to_anchor(dev);
        }
        return;
    }
#endif
    anch_txRespOrRxReEnable();                      // 发送resp帧之后，触发中断
}

static uint8_t twrAnchor_rxErrorOrTimeoutHandle(void)
{
    dwDevice_t* dev = get_the_local_structure_of_dev();

#if defined(ANCRANGE)
    if (dev->device_mode == ANCHOR_RNG && dev->twr_mode == INITIATOR)
    {
        /* A0 发起 A0-A1-A2 轮询时，现场测试可能只有部分 responder 在线。
         * 只要已经收到过至少一个 resp2，就继续发 final，让已在线 responder
         * 能完成本轮 TWR；否则直接退回监听。 */
        if (a2a_remainingResp > 0)
        {
            a2a_remainingResp--;
            if (a2a_remainingResp > 0)
            {
                uint32_t a2a_first_resp_us, a2a_anc_back_us;
                uint64_t next_resp_rx_time;
                uint8_t a2a_expected_resp_count = A2A_RESPONDER_COUNT;
                uint8_t next_resp_slot;
#if defined(USE_DW1000)
                if (inst_dataRate == DWT_BR_110K)
                {
                    a2a_first_resp_us = FIRST_RESP_SEND_110K;
                    a2a_anc_back_us   = ANC_RESP_SEND_BACK_110K;
                }
                else
#endif
                if (inst_dataRate == DWT_BR_6M8)
                {
                    a2a_first_resp_us = FIRST_RESP_SEND_6P8M;
                    a2a_anc_back_us   = ANC_RESP_SEND_BACK_6P8M;
                }
                else
                {
                    a2a_first_resp_us = FIRST_RESP_SEND_850K;
                    a2a_anc_back_us   = ANC_RESP_SEND_BACK_850K;
                }

                next_resp_slot = a2a_expected_resp_count - a2a_remainingResp + 1;
                next_resp_rx_time = a2a_poll_tx_ts
                    + ((uint64_t)(a2a_first_resp_us + next_resp_slot * inst_data_interval) * UUS_TO_DWT_TIME);
                dwt_setdelayedtrxtime((uint32)(next_resp_rx_time >> 8));
                dwt_setrxtimeout(a2a_anc_back_us + inst_resp_rx_timeout);
                dwt_setpreambledetecttimeout(PRE_TIMEOUT);
                if (dwt_rxenable(DWT_START_RX_DELAYED) == DWT_ERROR)
                {
                    if (a2a_rxRespMask != 0)
                    {
                        anch_a2a_sendFinal(dev);
                    }
                    else
                    {
                        rnganch_change_back_to_anchor(dev);
                    }
                }
            }
            else if (a2a_rxRespMask != 0)
            {
                anch_a2a_sendFinal(dev);
            }
            else
            {
                rnganch_change_back_to_anchor(dev);
            }
        }
        else if (a2a_rxRespMask != 0)
        {
            anch_a2a_sendFinal(dev);
        }
        else
        {
            rnganch_change_back_to_anchor(dev);
        }
        return 0;
    }
    if (dev->twr_mode == RESPONDER_A)
    {
        log_d("A2A A%d wait final timeout", anc_id);
        rnganch_change_back_to_anchor(dev);
        return 0;
    }
#endif

    // 接收超时有3种情况，接收poll帧超时，接收resp帧超时，接收final帧超时
    // 接收poll异常，重新打开接收
    // 接收final异常，重新打开接收
    // 接收resp异常，继续后续的发送resp或者接收resp，或者接收final
    if (dev->remainingRespToRx == -1)           // 说明remainingRespToRx是初始值，只有在接收到poll帧的时候，才会设置为基站数量减一，poll帧接收失败
    {
        range_status = RANGE_ERROR;
        anch_rxRenableImmdiate(dev);
        return 0;
    }
    else if (dev->remainingRespToRx > 0)        // 说明是接收resp帧出现异常
    {
        dev->remainingRespToRx--;               // 剩余接收resp帧数量减一
        anch_txRespOrRxReEnable();
        return 1;
    }
    else if (dev->remainingRespToRx == 0)       // 说明是接收final帧出现异常
    {
        range_status = RANGE_ERROR; 
        anch_rxRenableImmdiate(dev);
        return 2;
    }

    return 3;
}

static void anch_txRespOrRxReEnable(void)
{
    dwDevice_t* dev = get_the_local_structure_of_dev();
    int send_resp = 0;
    
    if (dev->remainingRespToRx == 0)                // 接收处理完毕，等待接收final，设定延迟接收
    {
        dev->wait4final = WAIT4TAGFINAL;
    }

    if (dev->respTxIndex & 0x01)                    // 如果
    {
        dev->respTxIndex = 0;
        send_resp = 1;
    }

    if (send_resp == 1)
    {
        // 轮到该基站发送resp帧
        uint64_t resp_tx_time;
#if defined(USE_DW1000)
        if (inst_dataRate == DWT_BR_110K)
        {
            resp_tx_time = (poll_rx_ts + (FIRST_RESP_SEND_110K + anc_id * inst_data_interval) * UUS_TO_DWT_TIME + (ANC_RESP_SEND_BACK_110K * UUS_TO_DWT_TIME));
        }
        else if (inst_dataRate == DWT_BR_6M8)
#else
        if (inst_dataRate == DWT_BR_6M8)
#endif
        {
            resp_tx_time = (poll_rx_ts + (FIRST_RESP_SEND_6P8M + anc_id * inst_data_interval) * UUS_TO_DWT_TIME + (ANC_RESP_SEND_BACK_6P8M * UUS_TO_DWT_TIME));
        }
        else if(inst_dataRate == DWT_BR_850K)
        {
            resp_tx_time = (poll_rx_ts + (FIRST_RESP_SEND_850K + anc_id * inst_data_interval) * UUS_TO_DWT_TIME + (ANC_RESP_SEND_BACK_850K * UUS_TO_DWT_TIME));
        }
        else
        {
            return ;
        }

        resp_tx_time = resp_tx_time >> 8;
        dwt_setdelayedtrxtime((uint32)resp_tx_time);

        int ret = dev_uwbStartTx(UWB_TX_MODE_DELAYED, false);  // 延时发送
        if(ret == DWT_ERROR)
        {   
            range_status = RANGE_ERROR;  
            anch_rxRenableImmdiate(dev);                    // 发送resp失败，说明测距失败，重新使能立即接收，去接收poll帧,此处需要同步，可能先前有接收resp帧失败，导致buffer未对齐
        }  
        else
        {
            handleResp_times = handleResp_times - 1;        // 发送resp帧处理成功，计数减一
        }
    }
    else                                                    // 继续接收
    {
        if (dev->remainingRespToRx == 0)                    // 不再期望接收任何Resp帧，接收final
        {
            uint64_t final_rx_time = (poll_rx_ts + inst_poll2final_time);              
            final_rx_time = final_rx_time >> 8;
            dwt_setdelayedtrxtime((uint32)final_rx_time);   // 设置接收机开启延时时间
            dwt_setrxtimeout(inst_final_rx_timeout);        // 设置接收数据超时时间
            dwt_setpreambledetecttimeout(PRE_TIMEOUT);      // 设置接收前导码超时时间
            int ret = dwt_rxenable(DWT_START_RX_DELAYED);   // 延时开启接收机，进行buffer同步，因为之前可能接收resp帧失败，导致buffer未对齐
            if(ret == DWT_ERROR)                            // 打开失败，立即重新打开接收，相当于本次测距失败，重新接收poll帧
            {
                anch_rxRenableImmdiate(dev);                // 接收机开启失败，直接立即打开接收，重回测距开始阶段，并进行buffer同步，接收poll帧
            }
        }
        else                                                           // 打开延迟接收，用于接收resp帧
        {
#if defined(USE_DW3000)
            dwt_configureframefilter(0, 0);                            // 关闭帧过滤，能够接收所有数据
#else
            dwt_enableframefilter(DWT_FF_NOTYPE_EN);          // 关闭帧过滤，能够接收所有数据
#endif

            //设置resp数据接收机开启时间
            uint64_t resp_rx_time;
#if defined(USE_DW1000)
            if (inst_dataRate == DWT_BR_110K)
                resp_rx_time = (poll_rx_ts + ((FIRST_RESP_SEND_110K + (MAX_AHCHOR_NUMBER - handleResp_times) * inst_data_interval) * UUS_TO_DWT_TIME));
            else if (inst_dataRate == DWT_BR_6M8)
#else
            if (inst_dataRate == DWT_BR_6M8)
#endif
                resp_rx_time = (poll_rx_ts + ((FIRST_RESP_SEND_6P8M + (MAX_AHCHOR_NUMBER - handleResp_times) * inst_data_interval) * UUS_TO_DWT_TIME));

            else if (inst_dataRate == DWT_BR_850K)
                resp_rx_time = (poll_rx_ts + ((FIRST_RESP_SEND_850K + (MAX_AHCHOR_NUMBER - handleResp_times) * inst_data_interval) * UUS_TO_DWT_TIME));
            else
                return ;

            resp_rx_time = resp_rx_time >> 8;
            dwt_setdelayedtrxtime(resp_rx_time);          // 设置接收机开启延时时间
            dwt_setrxtimeout(inst_resp_rx_timeout);       // 设置接收数据超时时间
            dwt_setpreambledetecttimeout(PRE_TIMEOUT);    // 设置接收前导码超时时间
            int ret = dwt_rxenable(DWT_START_RX_DELAYED); // 延时开启接收机，之前可能情况接收resp帧失败，接收resp帧成功，第二种情况要进行buffer同步
            if (ret == DWT_ERROR)
            {
                anch_rxRenableImmdiate(dev);                         // 接收机开启失败，直接立即打开接收，重回测距开始阶段，接收poll帧 
            }
            handleResp_times = handleResp_times - 1;                 // 成功打开接收resp帧，处理次数减一
            dev->respTxIndex = dev->respTxIndex >> 1;                // 持续左移，直到能对应上基站自身ID，找到发送resp帧的位置
        }
    }
}

// 使能立即接收，基站TWR处理最开始的阶段，一般在TWR进行过程中出错调用
static void anch_rxRenableImmdiate(dwDevice_t *dev)
{
    dev = get_the_local_structure_of_dev();
#if defined(USE_DW3000)
    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN | DWT_FF_ACK_EN);
#else
    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);   // 设置帧过滤模式开启
#endif
    dwt_setpreambledetecttimeout(0);                         // 清除前导码超时，一直接收
    dwt_setrxtimeout(0);                                     // 清除接收数据超时，一直接收
    int ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);          // 打开接收机，等待接收数据    
    if (ret == DWT_ERROR)                                    // 打开接收失败
    {
        // anch_rxRenableImmdiate();                         // 处理重新打开接收
        // return 0;
    }
    dev->twr_mode = LISTENER;                               // 空闲状态，等待接收 poll (tag 或 anchor)
}

// 将要发送的resp帧打包好，存入发送缓存
static void anch_perpareAnc2TagResp(void)
{
    /* resp数据打包 */
    tx_resp_msg[SEQ_NB_IDX]    = frame_seq_nb++;
    tx_resp_msg[PANID_IDX]     = (uint8_t) PAN_ID; 
    tx_resp_msg[PANID_IDX + 1] = (uint8_t) (PAN_ID >> 8); 
    tx_resp_msg[RANGE_NB_IDX]           = range_nb;
    tx_resp_msg[SENDER_SHORT_ADD_IDX]   = anc_id;
    tx_resp_msg[RECEIVER_SHORT_ADD_IDX] = recv_tag_id;
    tx_resp_msg[FUNC_CODE_IDX]          = RTLS_MSG_ANCH_RESP;
    tx_resp_msg[RESP_MSG_GROUP_IDX]     = group_id;

    /* 把上一周期算出的测距值(mm)以大端写入 resp 的 PREV_DIS 字段，回传给标签显示。
     * 标签按大端读取(见 instance_tag.c)。首个周期 prev_range 为 0，标签显示 0，
     * 下一周期起即为真实距离。recv_tag_id 已在 POLL 处理中设好。 */
    tx_resp_msg[RESP_MSG_PREV_DIS_IDX]     = (uint8_t)(prev_range[recv_tag_id] >> 24);
    tx_resp_msg[RESP_MSG_PREV_DIS_IDX + 1] = (uint8_t)(prev_range[recv_tag_id] >> 16);
    tx_resp_msg[RESP_MSG_PREV_DIS_IDX + 2] = (uint8_t)(prev_range[recv_tag_id] >> 8);
    tx_resp_msg[RESP_MSG_PREV_DIS_IDX + 3] = (uint8_t)(prev_range[recv_tag_id]);

    if (anc_id == 0)                                      // A0负责校准标签时序，防冲突
    {
        tx_resp_msg[RESP_MSG_GROUP_IDX] = group_id | 0x80; // 参与时序校准
        int error = 0;
        int currentSlotTime = 0;
        int expectedSlotTime = 0;
        int sframePeriod_ms = inst_one_slot_time * inst_slot_number;    // sframePeriod_ms 为整个TWR周期的总时间= 单slot时间*slot个数(标签总容量)
        int slotDuration_ms = inst_one_slot_time;                       // slotDuration_ms 为单slot时间
        int tagSleepCorrection_ms = 0;
        
        currentSlotTime  = range_time % sframePeriod_ms;         // currentSlotTime 当前正在通信标签的实际slot
        expectedSlotTime = recv_tag_id * slotDuration_ms;        // expectedSlotTime 当前正在通信标签应该处于的slot
        error = expectedSlotTime - currentSlotTime;              // error 计算slot差异 用于校准

        if (error < (-(sframePeriod_ms >> 1))) // if error is more  than 0.5 period, add whole period to give up to 1.5 period sleep
        {
            tagSleepCorrection_ms = (sframePeriod_ms + error);
        }
        else // the minimum Sleep time will be 0.5 period
        {
            tagSleepCorrection_ms = error;
        }

        tx_resp_msg[RESP_MSG_SLEEP_COR_IDX] = (tagSleepCorrection_ms >> 8) & 0xFF; // 校准时间高8位存储在 index 11 
        tx_resp_msg[RESP_MSG_SLEEP_COR_IDX + 1] = tagSleepCorrection_ms & 0xFF;    // 校准时间低8位存储在 index 12
    }
    else
    {
        tx_resp_msg[RESP_MSG_GROUP_IDX] = group_id & 0x7f;  //不参与时序校准
        tx_resp_msg[RESP_MSG_SLEEP_COR_IDX] = 0;
        tx_resp_msg[RESP_MSG_SLEEP_COR_IDX + 1] = 0;
    }

    dwt_writetxdata(RESP_MSG_LEN + FCS_LEN, tx_resp_msg, 0); // 数据写入DW1000数据缓冲区
    dwt_writetxfctrl(RESP_MSG_LEN + FCS_LEN, 0, 1);
}

#if defined(ANCRANGE)
static void rnganch_change_back_to_anchor(dwDevice_t *dev)
{
    dev->device_mode = ANCHOR;
    dev->twr_mode = LISTENER;
    a2a_state = A2A_IDLE;
    a2a_final_rx_time = 0;
    dwt_setrxtimeout(0);
    dwt_setrxaftertxdelay(0);
    anch_rxRenableImmdiate(dev);
}

static void anch_start_a2a(dwDevice_t *dev, uint8_t expectedResps)
{
    dwt_forcetrxoff();
    dev->device_mode = ANCHOR_RNG;
    dev->twr_mode = INITIATOR;
    a2a_range_nb++;
    a2a_remainingResp = expectedResps;
    a2a_rxRespMask = 0;

    tx_poll_anchor_msg[SEQ_NB_IDX] = frame_seq_nb++;
    tx_poll_anchor_msg[PANID_IDX] = (uint8_t)PAN_ID;
    tx_poll_anchor_msg[PANID_IDX + 1] = (uint8_t)(PAN_ID >> 8);
    tx_poll_anchor_msg[SENDER_SHORT_ADD_IDX] = anc_id;
    tx_poll_anchor_msg[RECEIVER_SHORT_ADD_IDX] = 0xFF;
    tx_poll_anchor_msg[FUNC_CODE_IDX] = RTLS_MSG_ANCH_POLL;
    tx_poll_anchor_msg[RANGE_NB_IDX] = a2a_range_nb;

    dwt_writetxdata(ANCH_POLL_MSG_LEN + FCS_LEN, tx_poll_anchor_msg, 0);
    dwt_writetxfctrl(ANCH_POLL_MSG_LEN + FCS_LEN, 0, 1);

    {
        uint32_t a2a_first_resp_us, a2a_anc_back_us;
#if defined(USE_DW1000)
        if (inst_dataRate == DWT_BR_110K)
        {
            a2a_first_resp_us = FIRST_RESP_SEND_110K;
            a2a_anc_back_us   = ANC_RESP_SEND_BACK_110K;
        }
        else
#endif
        if (inst_dataRate == DWT_BR_6M8)
        {
            a2a_first_resp_us = FIRST_RESP_SEND_6P8M;
            a2a_anc_back_us   = ANC_RESP_SEND_BACK_6P8M;
        }
        else
        {
            a2a_first_resp_us = FIRST_RESP_SEND_850K;
            a2a_anc_back_us   = ANC_RESP_SEND_BACK_850K;
        }
        dwt_setrxtimeout(a2a_first_resp_us + inst_data_interval + a2a_anc_back_us + inst_resp_rx_timeout);
    }
    a2a_state = A2A_POLL_SENT;
    if (dev_uwbStartTx(UWB_TX_MODE_IMMEDIATE, true) == DWT_ERROR)       // 期待延迟接收
    {
        rnganch_change_back_to_anchor(dev);
    }
}

static void anch_a2a_sendFinal(dwDevice_t *dev)
{
    memset(tx_anch_final_msg, 0, ANCH_FINAL_MSG_LEN);
    tx_anch_final_msg[0] = 0x41;
    tx_anch_final_msg[1] = 0x88;
    tx_anch_final_msg[SEQ_NB_IDX] = frame_seq_nb++;
    tx_anch_final_msg[PANID_IDX] = (uint8_t)PAN_ID;
    tx_anch_final_msg[PANID_IDX + 1] = (uint8_t)(PAN_ID >> 8);
    tx_anch_final_msg[RECEIVER_SHORT_ADD_IDX] = 0xFF;
    tx_anch_final_msg[RECEIVER_SHORT_ADD_IDX + 1] = 0xFF;
    tx_anch_final_msg[SENDER_SHORT_ADD_IDX] = anc_id;
    tx_anch_final_msg[SENDER_SHORT_ADD_IDX + 1] = 0x80;
    tx_anch_final_msg[FUNC_CODE_IDX] = RTLS_MSG_ANCH_FINAL;
    tx_anch_final_msg[RANGE_NB_IDX] = a2a_range_nb;
    tx_anch_final_msg[A2A_FINAL_VALID_IDX] = a2a_rxRespMask;

    final_msg_set_ts(&tx_anch_final_msg[A2A_FINAL_POLL_TX_TS_IDX], a2a_poll_tx_ts);
    /* FINAL 是延时发送、时间戳由调度时间手工算出，不是硬件读取，
     * 必须补上 TX 天线延时才能和 poll_tx/resp_tx(硬件已含天线延时)保持一致。
     * 漏加会让 responder 侧 Da 偏小、TOF 偏大(实测 ~15m 误差)。 */
    uint64_t final_tx_ts_embed = (a2a_final_tx_time & MASK_TXDTS) + ant_dly;
    final_msg_set_ts(&tx_anch_final_msg[A2A_FINAL_FINAL_TX_TS_IDX], final_tx_ts_embed);
    for (uint8_t i = 1; i <= A2A_RESPONDER_COUNT; i++)
    {
        if ((a2a_rxRespMask >> i) & 0x01)
        {
            final_msg_set_ts(&tx_anch_final_msg[A2A_FINAL_RESP_RX_TS_BASE + (i - 1) * FINAL_MSG_TS_LEN], a2a_resp_rx_ts[i]);
        }
    }

    dwt_writetxdata(ANCH_FINAL_MSG_LEN + FCS_LEN, tx_anch_final_msg, 0);
    dwt_writetxfctrl(ANCH_FINAL_MSG_LEN + FCS_LEN, 0, 1);
    dwt_setdelayedtrxtime((uint32)(a2a_final_tx_time >> 8));
    a2a_state = A2A_FINAL_SENT;
    if (dev_uwbStartTx(UWB_TX_MODE_DELAYED, false) == DWT_ERROR)
    {
        log_d("A2A A%d final starttx failed, mask=0x%02X rn=%d",
                anc_id, a2a_rxRespMask, a2a_range_nb);
        rnganch_change_back_to_anchor(dev);
    }
}

void anch_checkA2ATrigger(dwDevice_t *dev)
{
    if (dev->device_mode != ANCHOR || dev->twr_mode != LISTENER)
        return;

    if (anc_id == 0 && portGetTickCnt() >= a2aStartTime_ms)
    {
        a2aStartTime_ms += sframePeriod_ms;
        anch_start_a2a(dev, A2A_RESPONDER_COUNT);
    }
}
#endif

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static inline uint64_t get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;

    dwt_readtxtimestamp(ts_tab);
    memcpy(&ts, ts_tab, 4); // 拷贝低32bit
    ts |= (uint64_t)ts_tab[4] << 32; // 添加上高8 bit
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static inline uint64_t get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;

    dwt_readrxtimestamp(ts_tab);
    memcpy(&ts, ts_tab, 4); // 拷贝低32bit
    ts |= (uint64_t)ts_tab[4] << 32; // 添加上高8 bit
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static inline void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts)
{
    memcpy(ts, ts_field, 4);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static inline void final_msg_set_ts(uint8_t *ts_field, uint64_t ts)
{
    memcpy(ts_field, &ts, 4);
}
