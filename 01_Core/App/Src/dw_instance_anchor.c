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

static int twrAnchor_Init(instance_data_t *inst);
static uint32_t twrAnchor_onEvent(instance_data_t *inst, uwbEvent_t event);
static void twrAnchor_rxOkHandle(instance_data_t *inst);
static void twrAnchor_sentHandle(instance_data_t *inst);
static uint8_t twrAnchor_rxErrorOrTimeoutHandle(instance_data_t *inst);
static void anch_respSlotProcess(instance_data_t *inst);
static void anch_rxRenableImmdiate(instance_data_t *inst);
static void anch_perpareAnc2TagResp(instance_data_t *inst);
static inline uint64_t get_tx_timestamp_u64(void);
static inline uint64_t get_rx_timestamp_u64(void);
static inline void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);
static inline void final_msg_set_ts(uint8_t *ts_field, uint64_t ts);

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

static void twrAnchor_rxOkHandle(instance_data_t *inst)
{
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
    if (inst->device_mode == ANCHOR_RNG && inst->twr_mode == INITIATOR
        && f_code != RTLS_MSG_ANCH_RESP2)
    {
        dwt_setrxtimeout(inst->timings.replyInterval_us + inst->timings.respRxTimeout_us);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }
    if (inst->twr_mode == RESPONDER_A && f_code != RTLS_MSG_ANCH_FINAL)
    {
        /* RESPONDER_A only waits for FINAL. */
        dwt_setrxtimeout(inst->timings.finalRxTimeout_us);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }
#endif

    switch (f_code)
    {
    case RTLS_MSG_TAG_POLL:
#if defined(ANCRANGE)
        if (inst->device_mode == ANCHOR_RNG || inst->twr_mode == RESPONDER_A)
        {
            anch_rxRenableImmdiate(inst);
            break;
        }
#endif
        inst->twr_mode = RESPONDER_T;
        inst->range_nb    = rx_buffer[RANGE_NB_IDX];             // 取range_nb，resp发送时发送相同的range_nb
        inst->recv_tag_id = rx_buffer[SENDER_SHORT_ADD_IDX];     // 取发送标签的ID
        if (inst->recv_tag_id >= inst_slot_number)               // 标签ID如果大于标签总容量则退出
        {
            anch_rxRenableImmdiate(inst);         // 直接开启下一轮接收poll，无需同步buffer，暂定没问题
            break;
        }

        range_time = portGetTickCnt();            // 取得twr刚开始接收到poll的Tick
        inst->poll_rx_ts = get_rx_timestamp_u64();// 获得poll_rx时间戳
        /* 初始化本轮交换状态：nextSlotTime 指向首个 resp 槽（TREK delayedTRXTime 机制），
         * 每消耗一槽（收到/超时/自己发完）累加一个 interval，取代旧 respTxIndex 位走位 + handleResp_times 双计数 */
        inst->wait4final = 0;
        inst->lastTxFcode = 0;
        inst->rxRespMask = 0;
        inst->remainingRespToRx = MAX_AHCHOR_NUMBER - 1;         // 还需接收的其他基站 resp 帧数量
        inst->nextSlotTime = inst->poll_rx_ts
            + (uint64_t)inst->timings.firstRespDly_us * UUS_TO_DWT_TIME;
        anch_perpareAnc2TagResp(inst);            // 可以在这里就将要发送的resp帧写入发送缓存
        anch_respSlotProcess(inst);               // 判断是发送resp帧、开窗接收其他基站resp帧，还是开窗接收标签final帧
        break;

    case RTLS_MSG_TAG_FINAL:                      // 基站每接收到一个标签的final帧，就进行一次TOF计算
        if (((rx_buffer[RANGE_NB_IDX] == inst->range_nb) && (rx_buffer[SENDER_SHORT_ADD_IDX] == inst->recv_tag_id))) // 验证final帧的range_nb和标签ID是否和之前的poll帧一致
        {
            inst->wait4final = 0;
            inst->resp_valid = rx_buffer[FINAL_MSG_FINAL_VALID_IDX];
            range_time = portGetTickCnt();               // 再次获取TWR成功，final帧接收的Tick
            inst->resp_tx_ts = get_tx_timestamp_u64();   // 取得resp_tx时间戳
            inst->final_rx_ts = get_rx_timestamp_u64();  // 取得final_rx时间戳

            if((inst->resp_valid >> anc_id) & 0x01)      // final消息中，本基站发送的resp消息是有效的,则进行距离计算，或者发送时间戳
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
                poll_rx_ts_32  = (uint32_t)inst->poll_rx_ts;
                resp_tx_ts_32  = (uint32_t)inst->resp_tx_ts;
                final_rx_ts_32 = (uint32_t)inst->final_rx_ts;
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
                inst->prev_range[inst->recv_tag_id] = (int32_t)(distance_now_m * 1000);
                send_processeDis.distance = distance_now_m * 1000;
                send_processeDis.tag_id   = inst->recv_tag_id;
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
        anch_rxRenableImmdiate(inst);           // 直接开启下一轮接收poll，无需同步buffer，因为dwt_isr中已经处理过buffer切换
        break;

    case RTLS_MSG_ANCH_RESP:
        if (rx_buffer[RANGE_NB_IDX] == inst->range_nb)  // 和当前测距具有相同的range_nb
        {
            inst->rxRespMask |= (uint8_t)(1 << rx_buffer[SENDER_SHORT_ADD_IDX]);  // 记录已收到该基站的 resp

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
        // 不管校验有没有通过，该 resp 槽都已消耗，推进到下一槽
        inst->remainingRespToRx--;
        inst->nextSlotTime += (uint64_t)inst->timings.replyInterval_us * UUS_TO_DWT_TIME;
        anch_respSlotProcess(inst);        // 判断是发送resp帧、继续开窗接收resp帧，还是开窗接收final帧
        break;

#if defined(ANCRANGE)
    case RTLS_MSG_ANCH_POLL:              // 基站接收到其他基站发送的poll帧
    {
        if (anc_id == 0)
        {
            anch_rxRenableImmdiate(inst);
            break;
        }
        uint8_t initiator_id = rx_buffer[SENDER_SHORT_ADD_IDX];
        inst->device_mode = ANCHOR;
        inst->twr_mode = RESPONDER_A;
        inst->poll_rx_ts = get_rx_timestamp_u64();
        inst->a2a_range_nb = rx_buffer[RANGE_NB_IDX];
        // log_d("A2A A%d rxd poll from A%d", anc_id, initiator_id);

        uint8_t resp_position = anc_id - initiator_id - 1;
        /* A2A 的第一个 responder 原来直接占用首个响应槽，DW3000 上这个槽太紧，
         * 容易在 dwt_starttx() 时已经“过点”。回调虽已直驱状态机（无队列转发），
         * 仍保留整体后移一个 interval 的调度余量，公式模式联调时再评估收紧。 */
        /* Shift A1/A2 response slots later to leave enough scheduling margin. */
        uint64_t resp_tx_time = inst->poll_rx_ts
            + ((uint64_t)(inst->timings.firstRespDly_us + (resp_position + 1) * inst->timings.replyInterval_us) * UUS_TO_DWT_TIME)
            + ((uint64_t)inst->timings.ancRespTxBack_us * UUS_TO_DWT_TIME);
        /* Open FINAL RX from the slot boundary, leaving guard time before the actual FINAL preamble arrives. */
        inst->final_rx_time = inst->poll_rx_ts
            + ((uint64_t)(inst->timings.firstRespDly_us + A2A_FINAL_SCHEDULE_INDEX * inst->timings.replyInterval_us) * UUS_TO_DWT_TIME);

        tx_anch_resp2_msg[SEQ_NB_IDX] = inst->frame_seq_nb++;
        tx_anch_resp2_msg[PANID_IDX] = (uint8_t)PAN_ID;
        tx_anch_resp2_msg[PANID_IDX + 1] = (uint8_t)(PAN_ID >> 8);
        tx_anch_resp2_msg[SENDER_SHORT_ADD_IDX] = anc_id;
        tx_anch_resp2_msg[RECEIVER_SHORT_ADD_IDX] = 0xFF;
        tx_anch_resp2_msg[FUNC_CODE_IDX] = RTLS_MSG_ANCH_RESP2;
        tx_anch_resp2_msg[RANGE_NB_IDX] = inst->a2a_range_nb;
        int32_t prev_dis = inst->a2a_distance[initiator_id];
        tx_anch_resp2_msg[A2A_RESP2_PREV_DIS_IDX]     = (uint8_t)(prev_dis);
        tx_anch_resp2_msg[A2A_RESP2_PREV_DIS_IDX + 1] = (uint8_t)(prev_dis >> 8);
        tx_anch_resp2_msg[A2A_RESP2_PREV_DIS_IDX + 2] = (uint8_t)(prev_dis >> 16);
        tx_anch_resp2_msg[A2A_RESP2_PREV_DIS_IDX + 3] = (uint8_t)(prev_dis >> 24);

        dwt_writetxdata(ANCH_RESP2_MSG_LEN + FCS_LEN, tx_anch_resp2_msg, 0);
        dwt_writetxfctrl(ANCH_RESP2_MSG_LEN + FCS_LEN, 0, 1);
        dwt_setdelayedtrxtime((uint32)(resp_tx_time >> 8));
        inst->lastTxFcode = RTLS_MSG_ANCH_RESP2;            // TX-done 事件按此分流
        if (dev_uwbStartTx(UWB_TX_MODE_DELAYED, false) == DWT_ERROR)
        {
            log_d("A2A A%d resp2 starttx failed, init=A%d pos=%d rn=%d",
                    anc_id, initiator_id, resp_position, inst->a2a_range_nb);
            rnganch_change_back_to_anchor(inst);
        }
        break;
    }

    case RTLS_MSG_ANCH_RESP2:   // 接收基站发送的resp帧，获取上一轮的a2a测距结果，判断是否要发送final帧或者继续接收其他基站的resp帧
    {
        if (inst->device_mode != ANCHOR_RNG)
        {
            anch_rxRenableImmdiate(inst);
            break;
        }
        uint8_t resp_anc_id = rx_buffer[SENDER_SHORT_ADD_IDX];
        inst->resp_rx_ts[resp_anc_id] = get_rx_timestamp_u64();
        inst->rxRespMask |= (1 << resp_anc_id);
        inst->remainingRespToRx--;
        // log_d("A2A A%d rxd resp2 from A%d, remain=%d", anc_id, resp_anc_id, inst->remainingRespToRx);

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
            inst->a2a_distance[resp_anc_id] = prev_dis;
        }

        /* resp2 槽按基站号时间排序：收到最后一台（MAX_AHCHOR_NUMBER-1）的 resp2 时，
         * 缺席基站的槽已全部过去，直接发 final，不再空等一个超时窗
         * （否则 final 延迟发送会因等待超时而“过点”） */
        if (inst->remainingRespToRx == 0 || resp_anc_id == MAX_AHCHOR_NUMBER - 1)
        {
            anch_a2a_sendFinal(inst);
        }
        else
        {
            dwt_setrxtimeout(inst->timings.replyInterval_us + inst->timings.respRxTimeout_us);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        break;
    }

    case RTLS_MSG_ANCH_FINAL:
    {
        if (inst->twr_mode != RESPONDER_A)
        {
            anch_rxRenableImmdiate(inst);
            break;
        }
        // log_d("A2A A%d rxd final", anc_id);
        uint8_t initiator_id = rx_buffer[SENDER_SHORT_ADD_IDX];
        uint8_t valid = rx_buffer[A2A_FINAL_VALID_IDX];
        if (!((valid >> anc_id) & 0x01))
        {
            log_d("A2A A%d final valid mask miss, mask=0x%02X", anc_id, valid);
            rnganch_change_back_to_anchor(inst);
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

        inst->resp_tx_ts = get_tx_timestamp_u64();
        inst->final_rx_ts = get_rx_timestamp_u64();
        poll_rx_ts_32  = (uint32_t)inst->poll_rx_ts;
        resp_tx_ts_32  = (uint32_t)inst->resp_tx_ts;
        final_rx_ts_32 = (uint32_t)inst->final_rx_ts;

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
        if (dist_m > 0 && dist_m < 20000.0)
        {
            inst->a2a_distance[initiator_id] = (int32_t)(dist_m * 1000);
            log_i("A2A: A%d-A%d = %.2f m (responder)", initiator_id, anc_id, dist_m);
        }
        rnganch_change_back_to_anchor(inst);
        break;
    }
#endif

    default:
        /* 未知功能码：接收完成后接收机已关闭，必须重新开窗，否则停收（Phase C blink 在 switch 前特判） */
        anch_rxRenableImmdiate(inst);
        break;
    }
}

/* TX-done 事件按 lastTxFcode 分流（对应 TREK previousState 的作用） */
static void twrAnchor_sentHandle(instance_data_t *inst)
{
    switch (inst->lastTxFcode)
    {
#if defined(ANCRANGE)
    case RTLS_MSG_ANCH_POLL:      // A2A 发起端：poll 已发出，预排 final 发送时刻
        // log_d("A2A A%d poll sent", anc_id);
        inst->poll_tx_ts = get_tx_timestamp_u64();
        /* FINAL is pre-scheduled from POLL TX, after all RESP2 slots plus one extra guard slot. */
        inst->final_tx_time = inst->poll_tx_ts
            + ((uint64_t)(inst->timings.firstRespDly_us + A2A_FINAL_SCHEDULE_INDEX * inst->timings.replyInterval_us) * UUS_TO_DWT_TIME)
            + ((uint64_t)inst->timings.finalTxBack_us * UUS_TO_DWT_TIME);
        break;
    case RTLS_MSG_ANCH_RESP2:    // A2A 应答端：resp2 已发出，开窗等 final
        // log_d("A2A A%d resp2 sent", anc_id);
        inst->resp_tx_ts = get_tx_timestamp_u64();
        /* Arm delayed RX after RESP2 really leaves the air. */
        dwt_setdelayedtrxtime((uint32)(inst->final_rx_time >> 8));
        dwt_setrxtimeout(inst->timings.finalRxTimeout_us);
        dwt_setpreambledetecttimeout(PRE_TIMEOUT);
        if (dwt_rxenable(DWT_START_RX_DELAYED) == DWT_ERROR)
        {
            log_d("A2A A%d final rx enable failed", anc_id);
            rnganch_change_back_to_anchor(inst);
        }
        break;
    case RTLS_MSG_ANCH_FINAL:                       // A2A 发起端：final 已发出，本轮结束
        // log_d("A2A A%d final sent", anc_id);
        rnganch_change_back_to_anchor(inst);
        break;
#endif
    case RTLS_MSG_ANCH_RESP:                        // T2A：自己的 resp 槽已消耗，推进到下一槽
    default:
        inst->nextSlotTime += (uint64_t)inst->timings.replyInterval_us * UUS_TO_DWT_TIME;
        anch_respSlotProcess(inst);
        break;
    }
}

static uint8_t twrAnchor_rxErrorOrTimeoutHandle(instance_data_t *inst)
{
#if defined(ANCRANGE)
    if (inst->device_mode == ANCHOR_RNG && inst->twr_mode == INITIATOR)
    {
        /* A0 发起 A0-A1-A2 轮询时，现场可能只有部分 responder 在线。
         * 某槽超时/接收出错不终止本轮：立即重开接收兜住后续槽
         * （immediate 开窗无延迟“过点”风险；resp2 是广播帧，帧过滤不挡）。
         * 槽数耗尽后只要收到过 resp2 就发 final，让在线 responder 完成本轮 TWR。 */
        if (inst->remainingRespToRx > 0)
        {
            inst->remainingRespToRx--;
        }
        if (inst->remainingRespToRx > 0)
        {
            dwt_setrxtimeout(inst->timings.replyInterval_us + inst->timings.respRxTimeout_us);
            dwt_setpreambledetecttimeout(0);
            if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_ERROR)
            {
                return 0;
            }
        }
        if (inst->rxRespMask != 0)
        {
            anch_a2a_sendFinal(inst);
        }
        else
        {
            rnganch_change_back_to_anchor(inst);
        }
        return 0;
    }
    if (inst->twr_mode == RESPONDER_A)
    {
        log_d("A2A A%d wait final timeout", anc_id);
        rnganch_change_back_to_anchor(inst);
        return 0;
    }
#endif

    // 接收超时有3种情况：等poll期间杂散错误、resp槽超时、final接收超时
    if (inst->remainingRespToRx == -1)          // 空闲/等poll期间（不变式：LISTENER ⇔ remaining == -1），杂散事件不得开窗
    {
        range_status = RANGE_ERROR;
        anch_rxRenableImmdiate(inst);
        return 0;
    }
    else if (inst->wait4final != 0)             // 接收final帧出现异常
    {
        range_status = RANGE_ERROR;
        anch_rxRenableImmdiate(inst);
        return 2;
    }
    else if (inst->remainingRespToRx > 0)       // 接收resp帧出现异常：该槽已消耗，推进到下一槽
    {
        inst->remainingRespToRx--;
        inst->nextSlotTime += (uint64_t)inst->timings.replyInterval_us * UUS_TO_DWT_TIME;
        anch_respSlotProcess(inst);
        return 1;
    }

    range_status = RANGE_ERROR;                 // remaining==0 但未进入等final（异常路径），回到监听
    anch_rxRenableImmdiate(inst);
    return 3;
}

/* T2A resp 槽统一推进引擎（TREK anch_txresponse_or_rx_reenable 同款思路）：
 * 由 poll 接收、resp 接收成功/接收超时、自身 resp 发完各事件驱动，nextSlotTime 始终指向
 * 下一个未处理 resp 槽的绝对 dwt 时间（TREK delayedTRXTime 机制），调用前由事件方累加。
 * 三选一：轮到本基站则延时发送 resp；还有他站 resp 槽则延时开窗接收；否则开窗等 final。 */
static void anch_respSlotProcess(instance_data_t *inst)
{
    if (inst->remainingRespToRx < 0)   // 不变式：LISTENER ⇔ remaining == -1，杂散事件不得开窗
    {
        anch_rxRenableImmdiate(inst);
        return;
    }

    /* 已消耗槽数 == anc_id 时轮到本基站发送（TREK: remainingRespToRx + anc_id == N-1），
     * lastTxFcode 挡住发送完成后同一条件再次成立的重入 */
    if ((inst->remainingRespToRx + anc_id == MAX_AHCHOR_NUMBER - 1)
        && inst->lastTxFcode != RTLS_MSG_ANCH_RESP)
    {
        uint64_t resp_tx_time = inst->nextSlotTime
            + (uint64_t)inst->timings.ancRespTxBack_us * UUS_TO_DWT_TIME;
        dwt_setdelayedtrxtime((uint32)(resp_tx_time >> 8));
        inst->lastTxFcode = RTLS_MSG_ANCH_RESP;
        if (dev_uwbStartTx(UWB_TX_MODE_DELAYED, false) == DWT_ERROR)
        {
            range_status = RANGE_ERROR;
            anch_rxRenableImmdiate(inst);           // 发送resp失败，说明测距失败，重新使能立即接收，去接收poll帧
        }
    }
    else if (inst->remainingRespToRx > 0)           // 还有其他基站的resp槽，延时开窗接收
    {
#if defined(USE_DW3000)
        dwt_configureframefilter(0, 0);             // 关闭帧过滤：resp帧目的地址是标签，需旁听
#else
        dwt_enableframefilter(DWT_FF_NOTYPE_EN);    // 关闭帧过滤，能够接收所有数据
#endif
        dwt_setdelayedtrxtime((uint32)(inst->nextSlotTime >> 8));   // 从槽起点开窗
        dwt_setrxtimeout(inst->timings.respRxTimeout_us);           // 设置接收数据超时时间
        dwt_setpreambledetecttimeout(PRE_TIMEOUT);  // 设置接收前导码超时时间
        if (dwt_rxenable(DWT_START_RX_DELAYED) == DWT_ERROR)
        {
            anch_rxRenableImmdiate(inst);           // 接收机开启失败，重回测距开始阶段，接收poll帧
        }
    }
    else                                            // resp阶段结束，延时开窗等标签final
    {
        inst->wait4final = WAIT4TAGFINAL;
        uint64_t final_rx_time = inst->poll_rx_ts + inst->timings.pollRx2FinalRx_dwt;
        dwt_setdelayedtrxtime((uint32)(final_rx_time >> 8));
        dwt_setrxtimeout(inst->timings.finalRxTimeout_us);
        dwt_setpreambledetecttimeout(PRE_TIMEOUT);
        if (dwt_rxenable(DWT_START_RX_DELAYED) == DWT_ERROR)
        {
            anch_rxRenableImmdiate(inst);           // 打开失败，相当于本次测距失败，重新接收poll帧
        }
    }
}

// 使能立即接收，基站TWR处理最开始的阶段，一般在TWR进行过程中出错调用
static void anch_rxRenableImmdiate(instance_data_t *inst)
{
#if defined(USE_DW3000)
    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN | DWT_FF_ACK_EN);
#else
    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);   // 设置帧过滤模式开启
#endif
    dwt_setpreambledetecttimeout(0);                         // 清除前导码超时，一直接收
    dwt_setrxtimeout(0);                                     // 清除接收数据超时，一直接收
    (void)dwt_rxenable(DWT_START_RX_IMMEDIATE);              // 打开接收机，等待接收数据
    inst->twr_mode = LISTENER;                               // 空闲状态，等待接收 poll (tag 或 anchor)
    /* 不变式：LISTENER ⇔ remainingRespToRx == -1，交换状态一并复位 */
    inst->remainingRespToRx = -1;
    inst->wait4final = 0;
    inst->lastTxFcode = 0;
}

// 将要发送的resp帧打包好，存入发送缓存
static void anch_perpareAnc2TagResp(instance_data_t *inst)
{
    /* resp数据打包 */
    tx_resp_msg[SEQ_NB_IDX]    = inst->frame_seq_nb++;
    tx_resp_msg[PANID_IDX]     = (uint8_t) PAN_ID;
    tx_resp_msg[PANID_IDX + 1] = (uint8_t) (PAN_ID >> 8);
    tx_resp_msg[RANGE_NB_IDX]           = inst->range_nb;
    tx_resp_msg[SENDER_SHORT_ADD_IDX]   = anc_id;
    tx_resp_msg[RECEIVER_SHORT_ADD_IDX] = inst->recv_tag_id;
    tx_resp_msg[FUNC_CODE_IDX]          = RTLS_MSG_ANCH_RESP;
    tx_resp_msg[RESP_MSG_GROUP_IDX]     = group_id;

    /* 把上一周期算出的测距值(mm)以大端写入 resp 的 PREV_DIS 字段，回传给标签显示。
     * 标签按大端读取(见 instance_tag.c)。首个周期 prev_range 为 0，标签显示 0，
     * 下一周期起即为真实距离。recv_tag_id 已在 POLL 处理中设好。 */
    tx_resp_msg[RESP_MSG_PREV_DIS_IDX]     = (uint8_t)(inst->prev_range[inst->recv_tag_id] >> 24);
    tx_resp_msg[RESP_MSG_PREV_DIS_IDX + 1] = (uint8_t)(inst->prev_range[inst->recv_tag_id] >> 16);
    tx_resp_msg[RESP_MSG_PREV_DIS_IDX + 2] = (uint8_t)(inst->prev_range[inst->recv_tag_id] >> 8);
    tx_resp_msg[RESP_MSG_PREV_DIS_IDX + 3] = (uint8_t)(inst->prev_range[inst->recv_tag_id]);

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
        expectedSlotTime = inst->recv_tag_id * slotDuration_ms;  // expectedSlotTime 当前正在通信标签应该处于的slot
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
static void rnganch_change_back_to_anchor(instance_data_t *inst)
{
    inst->device_mode = ANCHOR;
    inst->final_rx_time = 0;
    dwt_setrxtimeout(0);
    dwt_setrxaftertxdelay(0);
    anch_rxRenableImmdiate(inst);       // 内部复位 twr_mode/remainingRespToRx/lastTxFcode
}

static void anch_start_a2a(instance_data_t *inst, uint8_t expectedResps)
{
    dwt_forcetrxoff();
    inst->device_mode = ANCHOR_RNG;
    inst->twr_mode = INITIATOR;
    inst->a2a_range_nb++;
    inst->remainingRespToRx = expectedResps;
    inst->rxRespMask = 0;

    tx_poll_anchor_msg[SEQ_NB_IDX] = inst->frame_seq_nb++;
    tx_poll_anchor_msg[PANID_IDX] = (uint8_t)PAN_ID;
    tx_poll_anchor_msg[PANID_IDX + 1] = (uint8_t)(PAN_ID >> 8);
    tx_poll_anchor_msg[SENDER_SHORT_ADD_IDX] = anc_id;
    tx_poll_anchor_msg[RECEIVER_SHORT_ADD_IDX] = 0xFF;
    tx_poll_anchor_msg[FUNC_CODE_IDX] = RTLS_MSG_ANCH_POLL;
    tx_poll_anchor_msg[RANGE_NB_IDX] = inst->a2a_range_nb;

    dwt_writetxdata(ANCH_POLL_MSG_LEN + FCS_LEN, tx_poll_anchor_msg, 0);
    dwt_writetxfctrl(ANCH_POLL_MSG_LEN + FCS_LEN, 0, 1);

    /* 接收窗一次性覆盖全部 responder 槽（槽序号 1..A2A_RESPONDER_COUNT，整体后移一槽）。
     * 只预算一个槽的话，中间某台不在线（如关掉 A1）时超时会掐断正在接收的后续 resp2
     * （帧等待超时不因前导码检测停表），且延迟重开窗已过点，导致整轮报废。 */
    dwt_setrxtimeout(inst->timings.firstRespDly_us
                     + A2A_RESPONDER_COUNT * inst->timings.replyInterval_us
                     + inst->timings.ancRespTxBack_us + inst->timings.respRxTimeout_us);
    inst->lastTxFcode = RTLS_MSG_ANCH_POLL;         // TX-done 事件按此分流
    if (dev_uwbStartTx(UWB_TX_MODE_IMMEDIATE, true) == DWT_ERROR)       // 期待延迟接收
    {
        rnganch_change_back_to_anchor(inst);
    }
}

static void anch_a2a_sendFinal(instance_data_t *inst)
{
    memset(tx_anch_final_msg, 0, ANCH_FINAL_MSG_LEN);
    tx_anch_final_msg[0] = 0x41;
    tx_anch_final_msg[1] = 0x88;
    tx_anch_final_msg[SEQ_NB_IDX] = inst->frame_seq_nb++;
    tx_anch_final_msg[PANID_IDX] = (uint8_t)PAN_ID;
    tx_anch_final_msg[PANID_IDX + 1] = (uint8_t)(PAN_ID >> 8);
    tx_anch_final_msg[RECEIVER_SHORT_ADD_IDX] = 0xFF;
    tx_anch_final_msg[RECEIVER_SHORT_ADD_IDX + 1] = 0xFF;
    tx_anch_final_msg[SENDER_SHORT_ADD_IDX] = anc_id;
    tx_anch_final_msg[SENDER_SHORT_ADD_IDX + 1] = 0x80;
    tx_anch_final_msg[FUNC_CODE_IDX] = RTLS_MSG_ANCH_FINAL;
    tx_anch_final_msg[RANGE_NB_IDX] = inst->a2a_range_nb;
    tx_anch_final_msg[A2A_FINAL_VALID_IDX] = inst->rxRespMask;

    final_msg_set_ts(&tx_anch_final_msg[A2A_FINAL_POLL_TX_TS_IDX], inst->poll_tx_ts);
    /* FINAL 是延时发送、时间戳由调度时间手工算出，不是硬件读取，
     * 必须补上 TX 天线延时才能和 poll_tx/resp_tx(硬件已含天线延时)保持一致。
     * 漏加会让 responder 侧 Da 偏小、TOF 偏大(实测 ~15m 误差)。 */
    uint64_t final_tx_ts_embed = (inst->final_tx_time & MASK_TXDTS) + ant_dly;
    final_msg_set_ts(&tx_anch_final_msg[A2A_FINAL_FINAL_TX_TS_IDX], final_tx_ts_embed);
    for (uint8_t i = 1; i <= A2A_RESPONDER_COUNT; i++)
    {
        if ((inst->rxRespMask >> i) & 0x01)
        {
            final_msg_set_ts(&tx_anch_final_msg[A2A_FINAL_RESP_RX_TS_BASE + (i - 1) * FINAL_MSG_TS_LEN], inst->resp_rx_ts[i]);
        }
    }

    dwt_writetxdata(ANCH_FINAL_MSG_LEN + FCS_LEN, tx_anch_final_msg, 0);
    dwt_writetxfctrl(ANCH_FINAL_MSG_LEN + FCS_LEN, 0, 1);
    dwt_setdelayedtrxtime((uint32)(inst->final_tx_time >> 8));
    inst->lastTxFcode = RTLS_MSG_ANCH_FINAL;        // TX-done 事件按此分流
    if (dev_uwbStartTx(UWB_TX_MODE_DELAYED, false) == DWT_ERROR)
    {
        log_d("A2A A%d final starttx failed, mask=0x%02X rn=%d",
                anc_id, inst->rxRespMask, inst->a2a_range_nb);
        rnganch_change_back_to_anchor(inst);
    }
}

void anch_checkA2ATrigger(instance_data_t *inst)
{
    if (inst->device_mode != ANCHOR || inst->twr_mode != LISTENER)
        return;

    if (anc_id == 0 && portGetTickCnt() >= inst->a2aStartTime_ms)
    {
        inst->a2aStartTime_ms += inst->sframePeriod_ms;
        anch_start_a2a(inst, A2A_RESPONDER_COUNT);
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
