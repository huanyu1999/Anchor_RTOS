#include "dw_instance.h"
#include "dw_sort.h"
#include "app_config.h"
#include "os_event.h"
#include "cmsis_os.h"
#include "elog.h"
#include <stdio.h>

#define ANCH_FRAME_TRACE_ENABLE 1

#if ANCH_FRAME_TRACE_ENABLE
#define ANCH_FRAME_TRACE_LOG(...) log_d(__VA_ARGS__)
#else
#define ANCH_FRAME_TRACE_LOG(...) ((void)0)
#endif

/* 距离打印开关：连 A0 的 RTT 即可看到测距结果，T2A/A2A 各每轮一行：
 *   DIST rn=17  T0->A0:   12.34 | T0->A1:   12.50 | T0->A2:   13.01 (m)
 *   A2A  rn=5   | A0-A1:   30.12 | A0-A2:   28.45 (m)
 * T2A 行：A0 为本轮实测值，A1/A2 为其 resp 帧回传的上一轮值（慢一拍）；
 * A2A 行：均为 responder resp2 回传的上一轮值。缺数据的显示 ----。
 * 仅 A0 打印，其他基站保持安静 */
#define ANCH_DIST_PRINT_ENABLE 1

#if ANCH_DIST_PRINT_ENABLE
/* 仅打印用：本轮已旁听到 resp 的他站位掩码（非 TWR 引擎状态，T2A 引擎仍不用掩码） */
static uint8_t dist_print_rxMask;
#endif

/* 保存当前ID标签的测距值，下次发送resp时发给标签 */

/* RESP数据帧格式 */
static uint8_t tx_resp_msg[RESP_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0x00, 0x80, RTLS_MSG_ANCH_RESP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/* RNG_INIT 数据帧格式（discovery 应答，A0 → 标签）：帧头 0x41 0x8C = 64bit 目的 + 16bit 源，
 * 目的地址 = blink 携带的标签 EUI64，载荷 = fcode + sleepCorrection + 分配的 tag_id（TREK srd_msg_dlss 同款） */
static uint8_t tx_rng_init_msg[RNG_INIT_MSG_LEN] = {0x41, 0x8C, 0, 0xCA, 0xDE,
                                                    0, 0, 0, 0, 0, 0, 0, 0,      /* 目的 EUI64，发送前填 */
                                                    0x00, 0x80, RTLS_MSG_RNG_INIT, 0, 0, 0, 0};

#if defined(ANCRANGE)
#define A2A_RESPONDER_COUNT            (MAX_AHCHOR_NUMBER - 1)
#define A2A_FINAL_SCHEDULE_INDEX       (A2A_RESPONDER_COUNT + 2)

static uint8_t tx_poll_anchor_msg[ANCH_POLL_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x00, 0x80, RTLS_MSG_ANCH_POLL, 0x00};
static uint8_t tx_anch_resp2_msg[ANCH_RESP2_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x00, 0x80, RTLS_MSG_ANCH_RESP2, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t tx_anch_final_msg[ANCH_FINAL_MSG_LEN];

static void rnganch_start_a2a(instance_data_t *inst, uint8_t expectedResps);
static void rnganch_sendFinal(instance_data_t *inst);
static void rnganch_changeBackToAnchor(instance_data_t *inst);
static void anch_perpareAnc2AncResp(instance_data_t *inst, uint8_t initiator_id);
static void rnganch_txResponseOrRxReenable(instance_data_t *inst, uint8_t initiator_id);
static void rnganch_rxRespSendfinal_or_rxReenable(instance_data_t *inst, bool lastRespRxd);
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
static void anch_txresponse_or_rx_reenable(instance_data_t *inst);
static void anch_rxRenableImmdiate(instance_data_t *inst);
static void anch_perpareAnc2TagResp(instance_data_t *inst);
static void anch_setFrameFilter(const instance_data_t *inst);
static int32_t calc_tag_sleep_correction(uint8_t slot, uint32_t nowMs);
static void anch_processTagBlink(instance_data_t *inst);
static int anch_add_tag_to_list(instance_data_t *inst, const uint8_t *eui64);
static void anch_prepareRngInitResp(instance_data_t *inst, const uint8_t *tagEui64, uint16_t slot);
static int16_t anch_get_rssi_cdbm(void);
static int32_t read_i32_le(const uint8_t *src);
static void write_i32_le(uint8_t *dst, int32_t value);
static void write_i16_le(uint8_t *dst, int16_t value);
static void anch_publish_tag_tof(instance_data_t *inst, uint8_t tag_id, uint8_t range_nb);
#if defined(ANCRANGE)
static void anch_publish_a2a_tof(instance_data_t *inst);
#endif

static inline uint64_t get_tx_timestamp_u64(void);
static inline uint64_t get_rx_timestamp_u64(void);
static inline void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);
static inline void final_msg_set_ts(uint8_t *ts_field, uint64_t ts);

// 单套 TWR anchor 算法，DW1000/DW3000 通过函数内 USE_DW1000/USE_DW3000 条件编译区分
uwbAlgorithm_t uwbTwr_AnchorAlgorithm = { .init = twrAnchor_Init, .onEvent = twrAnchor_onEvent };

/* 常规帧过滤设置：data/ack 帧；A0/gateway 额外放开 reserved 帧类型（ISO 0xC5 blink，discovery 用）。
 * DW3000 上放开 reserved 后必须真机确认非法 reserved 帧走 rxfailedcallback 且接收机被正确重开（ARFE 教训） */
static void anch_setFrameFilter(const instance_data_t *inst)
{
    uint16_t ff_mode = DWT_FF_DATA_EN | DWT_FF_ACK_EN;
#if UWB_DISCOVERY_ENABLE
    if (inst->gatewayAnchor)
    {
        ff_mode |= DWT_FF_RSVD_EN;
    }
#else
    (void)inst;
#endif
#if defined(USE_DW3000)
    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, ff_mode);
#else
    dwt_enableframefilter(ff_mode);
#endif
}

static int16_t anch_get_rssi_cdbm(void)
{
#if defined(USE_DW1000)
    dwt_rxdiag_t diag;
    dwt_readdiagnostics(&diag);
    float rssi = diag.rxPower * 100.0f;
    if (rssi > 32766.0f)
    {
        return 32766;
    }
    if (rssi < -32768.0f)
    {
        return -32768;
    }
    return (int16_t)rssi;
#else
    return INT16_MAX;
#endif
}

static int32_t read_i32_le(const uint8_t *src)
{
    return (int32_t)((uint32_t)src[0]
        | ((uint32_t)src[1] << 8)
        | ((uint32_t)src[2] << 16)
        | ((uint32_t)src[3] << 24));
}

static void write_i32_le(uint8_t *dst, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    dst[0] = (uint8_t)raw;
    dst[1] = (uint8_t)(raw >> 8);
    dst[2] = (uint8_t)(raw >> 16);
    dst[3] = (uint8_t)(raw >> 24);
}

static void write_i16_le(uint8_t *dst, int16_t value)
{
    uint16_t raw = (uint16_t)value;
    dst[0] = (uint8_t)raw;
    dst[1] = (uint8_t)(raw >> 8);
}

/*****************************************CCCC**DW1000 event function********************************************/
static int twrAnchor_Init(instance_data_t *inst)
{
    anch_setFrameFilter(inst);                              // 设置帧过滤模式开启
    dwt_setpreambledetecttimeout(0);                        // 清除前导码超时，一直接收
    dwt_setrxtimeout(0);                                    // 清除接收数据超时，一直接收
    int ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);         // 打开接收机，等待接收数据，初始接收，进行一次buffer对齐
    if(ret == DWT_ERROR)                                    // 打开接收失败
    {
        return 0;
    }
    inst->twr_mode = LISTENER;                              // 空闲状态，等待接收 poll (tag 或 anchor)
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
    /*
     *当前设备模式为A2A模式
     *并且当前TWR状态为已经发送poll帧，twr引擎模式为INITIATOR
     *并且当前接收到的帧不为基站发送的resp帧
     *则设置接收超时时间（相邻resp槽位间隔+resp帧接收超时），重新立即使能接收
     */
    if (inst->device_mode == ANCHOR_RNG && 
        inst->twr_mode == INITIATOR &&
        f_code != RTLS_MSG_ANCH_RESP2)
    {
        dwt_setrxtimeout(inst->timings.fixedReplyDelay_sy + inst->timings.fwto4RespFrame_sy);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }
    /*
     *当前设备模式为A2A模式
     *并且当前TWR状态为已经发送了基站resp帧，twr引擎模式为RESPONDER_A
     *并且当前接收到的帧不为基站发送的final
     *则设置接收超时时间（final帧接收超时），重新立即使能接收
     */
    if (inst->twr_mode == RESPONDER_A && f_code != RTLS_MSG_ANCH_FINAL)
    {
        /* RESPONDER_A only waits for FINAL. */
        dwt_setrxtimeout(inst->timings.fwto4FinalFrame_sy);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }
#endif

    /* ISO 0xC5 blink（discovery）：无 fcode/PAN，必须在 switch(f_code) 前按帧控字节特判。
     * 仅 A0/gateway 空闲监听时处理（帧过滤只对 A0 放开 reserved 帧；resp 旁听窗关过滤时
     * 其他基站也可能收到，与未知帧同样处理——重新开窗）。注意须放在 A2A 特判之后，
     * A2A 交换中收到 blink 走上面的容错重开窗，不得打断交换 */
    if (UWB_DISCOVERY_ENABLE && rx_buffer[0] == 0xC5 && frame_len == BLINK_MSG_LEN + FCS_LEN)
    {
        if (inst->gatewayAnchor && inst->twr_mode == LISTENER)
        {
            anch_processTagBlink(inst);
        }
        else
        {
            anch_rxRenableImmdiate(inst);
        }
        return;
    }

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
        inst->twr_mode = RESPONDER_T;                         // 准备答复TAG的Poll帧
        inst->range_nb = rx_buffer[RANGE_NB_IDX];             // 取range_nb，resp发送时发送相同的range_nb
        inst->recv_tag_id = rx_buffer[SENDER_SHORT_ADD_IDX];  // 取发送标签的ID
        if (inst->recv_tag_id >= inst_slot_number)            // 标签ID如果大于标签总容量则退出
        {
            anch_rxRenableImmdiate(inst);         // 直接开启下一轮接收poll，无需同步buffer，暂定没问题
            break;
        }

        range_time = portGetTickCnt();            // 取得twr刚开始接收到poll的Tick
        inst->poll_rx_ts = get_rx_timestamp_u64();// 获得poll_rx时间戳
        inst->current_poll_rssi_cdbm = anch_get_rssi_cdbm();
        /* 初始化本轮交换状态：nextSlotTime32h 基准 = poll RX，槽引擎每次进入先推进一槽
         * （TREK anch_txresponse_or_rx_reenable 同款：delayedTRXTime32h 由函数内累加），
         * 首个 resp 槽即 poll RX + 1×槽间隔 */
        // ANCH_FRAME_TRACE_LOG("T2A rx poll: A%d <- T%d rn=%d", anc_id, inst->recv_tag_id, inst->range_nb);
        inst->wait4final = 0;
        inst->lastTxFcode = 0;
        inst->remainingRespToRx = MAX_AHCHOR_NUMBER - 1;         // 还需接收的其他基站 resp 帧数量
#if ANCH_DIST_PRINT_ENABLE
        dist_print_rxMask = 0;                    // 仅打印用，逐轮清零
#endif
        inst->nextSlotTime32h = (uint32_t)(inst->poll_rx_ts >> 8); // 设置后续resp的发送时间
        anch_perpareAnc2TagResp(inst);            // 可以在这里就将要发送的resp帧写入发送缓存
        anch_txresponse_or_rx_reenable(inst);     // 判断是发送resp帧、开窗接收其他基站resp帧，还是开窗接收标签final帧
        break;

    case RTLS_MSG_TAG_FINAL:                      // 基站每接收到一个标签的final帧，就进行一次TOF计算
        if (((rx_buffer[RANGE_NB_IDX] == inst->range_nb) && (rx_buffer[SENDER_SHORT_ADD_IDX] == inst->recv_tag_id))) // 验证final帧的range_nb和标签ID是否和之前的poll帧一致
        {
            inst->wait4final = 0;
            inst->resp_valid = rx_buffer[FINAL_MSG_FINAL_VALID_IDX];
            // ANCH_FRAME_TRACE_LOG("T2A rx final: A%d <- T%d rn=%d valid=0x%02X",
            //                      anc_id, inst->recv_tag_id, inst->range_nb, inst->resp_valid);
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
                /* A0 在写入本轮结果前，聚合上一轮四 Anchor ToF。远端值在本轮 RESP 中
                 * 回传；按 result_seq 严格匹配，避免将丢帧后的旧值混进定位组。 */
                if (anc_id == 0 && inst->tag_result_valid[inst->recv_tag_id][0])
                {
                    anch_publish_tag_tof(inst, inst->recv_tag_id,
                                         inst->tag_result_seq[inst->recv_tag_id][0]);
                }

                /* 保存本次测距值(mm)供本地报警，并保存原始 ToF 供下一轮 A0 聚合。 */
                inst->prev_range[inst->recv_tag_id] = (int32_t)(distance_now_m * 1000);
                inst->tag_tof_dtu[inst->recv_tag_id][anc_id] = (int32_t)tof_dtu;
                inst->tag_rssi_cdbm[inst->recv_tag_id][anc_id] = inst->current_poll_rssi_cdbm;
                inst->tag_result_seq[inst->recv_tag_id][anc_id] = inst->range_nb;
                inst->tag_result_valid[inst->recv_tag_id][anc_id] = 1;
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
#if ANCH_DIST_PRINT_ENABLE
            /* 每轮拼一行打印：final 已收完、交换结束，此处不影响时序。
             * A0 为本轮实测；A1/A2 为其 resp 回传的上一轮值；没数据的显示 ---- */
            if (anc_id == 0)
            {
                char line[96];
                int len;
                if (range_status == RANGE_TWR_OK)
                {
                    len = snprintf(line, sizeof(line), "DIST rn=%-3u T%u->A0: %7.2f",
                                   inst->range_nb, inst->recv_tag_id, distance_now_m);
                }
                else
                {
                    len = snprintf(line, sizeof(line), "DIST rn=%-3u T%u->A0:    ----",
                                   inst->range_nb, inst->recv_tag_id);
                }
                for (uint8_t a = 1; a < MAX_AHCHOR_NUMBER; a++)
                {
                    if (dist_print_rxMask & (1 << a))
                    {
                        len += snprintf(line + len, sizeof(line) - len, " | T%u->A%u: %7.2f",
                                        inst->recv_tag_id, a, distance_report[a] / 1000.0);
                    }
                    else
                    {
                        len += snprintf(line + len, sizeof(line) - len, " | T%u->A%u:    ----",
                                        inst->recv_tag_id, a);
                    }
                }
                log_i("%s (m)", line);
            }
#endif
        }
        dwt_forcetrxoff();                      // 该函数重置了host side receive buffer pointer，导致双buffer模式下，TWR测距只能成功一次，后续TWR测距均失败
        anch_rxRenableImmdiate(inst);           // 直接开启下一轮接收poll，无需同步buffer，因为dwt_isr中已经处理过buffer切换
        break;

    case RTLS_MSG_ANCH_RESP:                    // 接收到T2A阶段的，其他基站发送的resp帧
        // ANCH_FRAME_TRACE_LOG("T2A rx resp: A%d <- A%d rn=%d current=%d",
        //                      anc_id, rx_buffer[SENDER_SHORT_ADD_IDX], rx_buffer[RANGE_NB_IDX], inst->range_nb);
        if (frame_len >= RESP_MSG_LEN + FCS_LEN && rx_buffer[RANGE_NB_IDX] == inst->range_nb)  // 和当前测距具有相同的range_nb
        {
            if((rx_buffer[RESP_MSG_GROUP_IDX] & 0x7f) == (group_id & 0x7f)) // 只取和自己组号相同的其他基站数据上报
            {
                uint8_t recv_anc_id = rx_buffer[SENDER_SHORT_ADD_IDX];     //取基站ID
                if (recv_anc_id < MAX_AHCHOR_NUMBER && recv_anc_id != anc_id)
                {
                    int32_t prev_tof_dtu = read_i32_le(&rx_buffer[RESP_MSG_PREV_TOF_IDX]);
                    uint8_t result_seq = rx_buffer[RESP_MSG_RESULT_SEQ_IDX];
                    int16_t result_rssi = (int16_t)((uint16_t)rx_buffer[RESP_MSG_RSSI_IDX]
                        | ((uint16_t)rx_buffer[RESP_MSG_RSSI_IDX + 1] << 8));

                    inst->tag_tof_dtu[inst->recv_tag_id][recv_anc_id] = prev_tof_dtu;
                    inst->tag_rssi_cdbm[inst->recv_tag_id][recv_anc_id] = result_rssi;
                    inst->tag_result_seq[inst->recv_tag_id][recv_anc_id] = result_seq;
                    inst->tag_result_valid[inst->recv_tag_id][recv_anc_id] = (prev_tof_dtu > 0);
                    distance_report[recv_anc_id] = (int32_t)((double)prev_tof_dtu
                        * DWT_TIME_UNITS * SPEED_OF_LIGHT * 1000.0);
                    group_report[recv_anc_id] = rx_buffer[RESP_MSG_GROUP_IDX] & 0x7f;
                }
#if ANCH_DIST_PRINT_ENABLE
                dist_print_rxMask |= (uint8_t)(1 << recv_anc_id);
#endif
            }
        }
        // 不管校验有没有通过，该 resp 槽都已消耗（槽推进在引擎内完成）
        inst->remainingRespToRx--;
        anch_txresponse_or_rx_reenable(inst);    // 判断是发送resp帧、继续开窗接收resp帧，还是开窗接收final帧
        break;

#if defined(ANCRANGE)
    case RTLS_MSG_ANCH_POLL:              // 基站接收到其他基站发送的poll帧，准备发送resp帧
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
        ANCH_FRAME_TRACE_LOG("A2A A%d rxd poll from A%d", anc_id, initiator_id);

        anch_perpareAnc2AncResp(inst, initiator_id);          // resp2 帧打包写入发送缓存
        rnganch_txResponseOrRxReenable(inst, initiator_id);   // 排槽并延时发送 resp2，失败回 ANCHOR
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
        inst->rxRespMaskAnc |= (1 << resp_anc_id);
        inst->remainingRespToRxAnc--;

        // ANCH_FRAME_TRACE_LOG("A2A A%d rxd resp2 from A%d, remain=%d",
        //                      anc_id, resp_anc_id, inst->remainingRespToRxAnc);

        int32_t prev_dis = (int32_t)(rx_buffer[A2A_RESP2_PREV_DIS_IDX]
            | ((uint32_t)rx_buffer[A2A_RESP2_PREV_DIS_IDX + 1] << 8)
            | ((uint32_t)rx_buffer[A2A_RESP2_PREV_DIS_IDX + 2] << 16)
            | ((uint32_t)rx_buffer[A2A_RESP2_PREV_DIS_IDX + 3] << 24));
        /* 逐帧打印已合并到 final 发送完成处的单行输出（sentHandle 的 ANCH_FINAL 分支） */
        // if (anc_id == 0)
        // {
        //     log_i("A2A: A%d rxd resp2 from A%d prev=%.2f m", anc_id, resp_anc_id, prev_dis / 1000.0);
        // }
        if (prev_dis > 0)
        {
            inst->a2a_distance[resp_anc_id] = prev_dis;
        }

        /* resp2 槽按基站号时间排序：收到最后一台（MAX_AHCHOR_NUMBER-1）的 resp2 时，
         * 缺席基站的槽已全部过去，直接发 final，不再空等一个超时窗
         * （否则 final 延迟发送会因等待超时而“过点”） */
        rnganch_rxRespSendfinal_or_rxReenable(inst, resp_anc_id == MAX_AHCHOR_NUMBER - 1);
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
        ANCH_FRAME_TRACE_LOG("A2A A%d rxd final", anc_id);
        if (!((valid >> anc_id) & 0x01))
        {
            ANCH_FRAME_TRACE_LOG("A2A A%d final valid mask miss, mask=0x%02X", anc_id, valid);
            rnganch_changeBackToAnchor(inst);
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

#if defined(USE_DW1000)
        dist_m = dist_m - dwt_getrangebias(inst_ch, (float)dist_m, inst_prf);
#endif
        if (dist_m > 0 && dist_m < 20000.0)
        {
            inst->a2a_distance[initiator_id] = (int32_t)(dist_m * 1000);
            log_i("A2A  rn=%-3u A%d-A%d = %.2f m (responder)", inst->a2a_range_nb, initiator_id, anc_id, dist_m);
        }
        rnganch_changeBackToAnchor(inst);
        break;
    }
#endif

    default:
        /* 未知功能码：接收完成后接收机已关闭，必须重新开窗，否则停收（Phase C blink 在 switch 前特判） */
        anch_rxRenableImmdiate(inst);
        break;
    }
}

/* 基站TX-done 事件按 lastTxFcode 分流（对应 TREK previousState 的作用），每次发送成功，就跳转该函数 */
static void twrAnchor_sentHandle(instance_data_t *inst)
{
    switch (inst->lastTxFcode)
    {
#if defined(ANCRANGE)
    case RTLS_MSG_ANCH_POLL:      // A2A 发起端：poll 已发出，预排 final 发送时刻
        inst->poll_tx_ts = get_tx_timestamp_u64();
        /* FINAL is pre-scheduled from POLL TX, after all RESP2 slots plus one extra guard slot. */
        inst->final_tx_time32h = (uint32_t)(inst->poll_tx_ts >> 8)
            + (uint32_t)(A2A_FINAL_SCHEDULE_INDEX + 1) * inst->timings.fixedReplyDelayAnc32h;
        break;

    case RTLS_MSG_ANCH_RESP2:    // A2A 应答端：resp2 已发出，开窗等 final
        // ANCH_FRAME_TRACE_LOG("A2A A%d resp2 sent", anc_id);
        inst->resp_tx_ts = get_tx_timestamp_u64();
        /* Arm delayed RX after RESP2 really leaves the air, open one preamble early. */
        dwt_setdelayedtrxtime(inst->final_rx_time32h - inst->timings.preambleDuration32h);
        dwt_setrxtimeout(inst->timings.fwto4FinalFrame_sy);
        dwt_setpreambledetecttimeout(PRE_TIMEOUT);
        if (dwt_rxenable(DWT_START_RX_DELAYED) == DWT_ERROR)
        {
            log_d("A2A A%d final rx enable failed", anc_id);
            rnganch_changeBackToAnchor(inst);
        }
        break;
    case RTLS_MSG_ANCH_FINAL:                       // A2A 发起端：final 已发出，本轮结束
        // ANCH_FRAME_TRACE_LOG("A2A A%d final sent", anc_id);
#if ANCH_DIST_PRINT_ENABLE
        /* A2A 每轮拼一行打印：值为各 responder resp2 回传的上一轮 A0-Ax 距离（慢一拍），
         * 本轮没收到谁的 resp2 就显示 ----。final 已发出、交换结束，此处不影响时序 */
        if (anc_id == 0)
        {
            char line[80];
            int len = snprintf(line, sizeof(line), "A2A  rn=%-3u", inst->a2a_range_nb);
            for (uint8_t a = 1; a <= A2A_RESPONDER_COUNT; a++)
            {
                if ((inst->rxRespMaskAnc >> a) & 0x01)
                {
                    len += snprintf(line + len, sizeof(line) - len, " | A0-A%u: %7.2f",
                                    a, inst->a2a_distance[a] / 1000.0);
                }
                else
                {
                    len += snprintf(line + len, sizeof(line) - len, " | A0-A%u:    ----", a);
                }
            }
            log_i("%s (m)", line);
        }
#endif
        rnganch_changeBackToAnchor(inst);
        break;
#endif
    case RTLS_MSG_RNG_INIT:                         // discovery：RNG_INIT 已发出，回 LISTENER 等标签首个 poll
        ANCH_FRAME_TRACE_LOG("DISC A%d rng_init sent", anc_id);
        anch_rxRenableImmdiate(inst);               // 内部复位 twr_mode(RESPONDER_B→LISTENER)/lastTxFcode
        break;

    case RTLS_MSG_ANCH_RESP:                        // T2A：自己的 resp 槽已消耗（槽推进在引擎内完成）
        // ANCH_FRAME_TRACE_LOG("T2A tx resp: A%d -> T%d rn=%d", anc_id, inst->recv_tag_id, inst->range_nb);
    default:
        anch_txresponse_or_rx_reenable(inst);
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
        if (inst->remainingRespToRxAnc > 0)
        {
            // log_d("A2A A%d wait resp2 timeout, remain=%d mask=0x%02X rn=%d",
                    // anc_id, inst->remainingRespToRxAnc, inst->rxRespMaskAnc, inst->a2a_range_nb);
            inst->remainingRespToRxAnc--;
        }
        rnganch_rxRespSendfinal_or_rxReenable(inst, false);
        return 0;
    }
    if (inst->twr_mode == RESPONDER_A)      // 发送A2A resp帧的基站，接收final超时
    {
        log_d("A2A A%d wait final timeout", anc_id);
        rnganch_changeBackToAnchor(inst);
        return 0;
    }
#endif

    // T2A模式，接收超时有3种情况：等poll期间杂散错误、resp槽超时、final接收超时
    if (inst->remainingRespToRx == -1)          // 空闲/等poll期间（不变式：LISTENER ⇔ remaining == -1），杂散事件不得开窗
    {
        range_status = RANGE_ERROR;
        anch_rxRenableImmdiate(inst);
        return 0;
    }
    else if (inst->wait4final != 0)             // T2A模式，接收final帧出现异常
    {
        range_status = RANGE_ERROR;
        anch_rxRenableImmdiate(inst);
        return 2;
    }
    else if (inst->remainingRespToRx > 0)       // T2A模式，接收resp帧出现异常：该槽已消耗（槽推进在引擎内完成）
    {
        inst->remainingRespToRx--;
        anch_txresponse_or_rx_reenable(inst);
        return 1;
    }

    range_status = RANGE_ERROR;                 // remaining==0 但未进入等final（异常路径），回到监听
    anch_rxRenableImmdiate(inst);
    return 3;
}

/* T2A resp 槽统一推进引擎（TREK anch_txresponse_or_rx_reenable 同名移植，仅 T2A 使用，
 * A2A 走 rnganch_* / rnganchrxresp_* 两个函数，两条路径彻底分开）：
 * 由 poll 接收、resp 接收成功/接收超时、自身 resp 发完各事件驱动。nextSlotTime32h 基准为
 * poll RX 时刻，每次进入本函数先推进一槽（TREK 在函数内累加 delayedTRXTime32h 同款），
 * 推进后即指向当前待处理 resp 槽的 RMARKER 时刻。
 * 三选一：轮到本基站则延时发送 resp；还有他站 resp 槽则延时开窗接收（提前一个前导码）；
 * 否则开窗等 final（Tag final TX = poll TX + pollTx2FinalTxDelay，双端同值）。
 */
static void anch_txresponse_or_rx_reenable(instance_data_t *inst)
{
    if (inst->remainingRespToRx < 0)   // 不变式：LISTENER ⇔ remaining == -1，杂散事件不得开窗
    {
        anch_rxRenableImmdiate(inst);
        return;
    }

    inst->nextSlotTime32h += inst->timings.fixedReplyDelayAnc32h;   // 每消耗一槽推进一次

    /* 已消耗槽数 == anc_id 时轮到本基站发送（TREK: remainingRespToRx + anc_id == N-1），
     * lastTxFcode 挡住发送完成后同一条件再次成立的重入 */
    if ((inst->remainingRespToRx + anc_id == MAX_AHCHOR_NUMBER - 1)
        && inst->lastTxFcode != RTLS_MSG_ANCH_RESP)
    {
        dwt_setdelayedtrxtime(inst->nextSlotTime32h);   // resp RMARKER 落在槽时刻（翻转量已含在槽间隔内）
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
        /* 他站 resp RMARKER 在槽时刻，前导码在其之前飞完，开窗须提前一个前导码（TREK anch_enable_rx） */
        dwt_setdelayedtrxtime(inst->nextSlotTime32h - inst->timings.preambleDuration32h);
        dwt_setrxtimeout(inst->timings.fwto4RespFrame_sy);          // 设置接收数据超时时间
        dwt_setpreambledetecttimeout(PRE_TIMEOUT);  // 设置接收前导码超时时间
        if (dwt_rxenable(DWT_START_RX_DELAYED) == DWT_ERROR)
        {
            anch_rxRenableImmdiate(inst);           // 接收机开启失败，重回测距开始阶段，接收poll帧
        }
    }
    else                                            // resp阶段结束，延时开窗等标签final
    {
        inst->wait4final = WAIT4TAGFINAL;
        uint32_t final_rx_time32h = (uint32_t)(inst->poll_rx_ts >> 8)
            + inst->timings.pollTx2FinalTxDelay32h - inst->timings.preambleDuration32h;
        dwt_setdelayedtrxtime(final_rx_time32h);
        dwt_setrxtimeout(inst->timings.fwto4FinalFrame_sy * 2);     // TREK 同款：final 窗超时加倍余量
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
    anch_setFrameFilter(inst);                               // 设置帧过滤模式开启
    dwt_setpreambledetecttimeout(0);                         // 清除前导码超时，一直接收
    dwt_setrxtimeout(0);                                     // 清除接收数据超时，一直接收
    dwt_rxenable(DWT_START_RX_IMMEDIATE);                    // 打开接收机，等待接收数据
    inst->twr_mode = LISTENER;                               // 空闲状态，等待接收 poll (tag 或 anchor)
    /* T2A 不变式：LISTENER ⇔ remainingRespToRx == -1，交换状态一并复位
     * （remainingRespToRx 为 T2A 专用，A2A 期间保持 -1，A2A 用 remainingRespToRxAnc） */
    inst->remainingRespToRx = -1;
    inst->wait4final = 0;
    inst->lastTxFcode = 0;
}

// T2A模式，基站将要发送给标签的resp帧打包好，存入发送缓存
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

    /* 上一轮本 Anchor 的 ToF 结果：A0 在下一轮旁听其它 RESP 后按 result_seq 聚合。
     * Tag 固件需将旧 PREV_DIS 解释改为这一小端 ToF 字段，并采用新的 RESP_MSG_LEN。 */
    if (inst->tag_result_valid[inst->recv_tag_id][anc_id])
    {
        write_i32_le(&tx_resp_msg[RESP_MSG_PREV_TOF_IDX],
                     inst->tag_tof_dtu[inst->recv_tag_id][anc_id]);
        tx_resp_msg[RESP_MSG_RESULT_SEQ_IDX] = inst->tag_result_seq[inst->recv_tag_id][anc_id];
        write_i16_le(&tx_resp_msg[RESP_MSG_RSSI_IDX],
                     inst->tag_rssi_cdbm[inst->recv_tag_id][anc_id]);
    }
    else
    {
        write_i32_le(&tx_resp_msg[RESP_MSG_PREV_TOF_IDX], 0);
        tx_resp_msg[RESP_MSG_RESULT_SEQ_IDX] = 0;
        write_i16_le(&tx_resp_msg[RESP_MSG_RSSI_IDX], INT16_MAX);
    }

    if (anc_id == 0)                                      // A0负责校准标签时序，防冲突
    {
        tx_resp_msg[RESP_MSG_GROUP_IDX] = group_id | 0x80; // 参与时序校准
        int32_t tagSleepCorrection_ms = calc_tag_sleep_correction(inst->recv_tag_id, range_time);

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

/* 标签睡眠校准量(ms)：把标签下次 poll 引导到它自己的时隙（TREK 同款算法）。
 * nowMs 取当前事件（poll/blink 接收）的 tick；返回值区间 (-0.5 周期, +1 周期)，
 * T2A resp 直接下发，RNG_INIT 路径再额外 +1 周期（TREK：注册后至少睡 1.5 周期） */
static int32_t calc_tag_sleep_correction(uint8_t slot, uint32_t nowMs)
{
    int32_t sframePeriod_ms = (int32_t)inst_one_slot_time * inst_slot_number;  // 整个TWR周期的总时间 = 单slot时间 × slot个数(标签总容量)
    int32_t currentSlotTime  = (int32_t)(nowMs % (uint32_t)sframePeriod_ms);   // 当前正在通信标签的实际slot时刻
    int32_t expectedSlotTime = (int32_t)slot * inst_one_slot_time;             // 该标签应该处于的slot时刻
    int32_t error = expectedSlotTime - currentSlotTime;

    if (error < (-(sframePeriod_ms >> 1))) // if error is more than 0.5 period, add whole period to give up to 1.5 period sleep
    {
        return sframePeriod_ms + error;
    }
    return error;
}

/* discovery：A0 收到标签 ISO 0xC5 blink（rxOkHandle 特判入口，仅 gateway+LISTENER）。
 * 注册进 tagList 拿到 slot（即分配的 tag_id），随后在 blinkRx + 1×槽间隔处延时发送
 * RNG_INIT（TREK rx_ok_cb_anch 的 DWT_SIG_RX_BLINK 路径同款）；表满或发送过点则重开接收 */
static void anch_processTagBlink(instance_data_t *inst)
{
    int slot = anch_add_tag_to_list(inst, &rx_buffer[BLINK_TAG_EUI64_IDX]);
    if (slot < 0)                                   // tagList 已满，无法接纳新标签
    {
        ANCH_FRAME_TRACE_LOG("DISC A%d tagList full, blink dropped", anc_id);
        anch_rxRenableImmdiate(inst);
        return;
    }
    inst->twr_mode = RESPONDER_B;
    anch_prepareRngInitResp(inst, &rx_buffer[BLINK_TAG_EUI64_IDX], (uint16_t)slot);

    /* RNG_INIT RMARKER = blink RX + 1×槽间隔（TREK 用 fixedReplyDelayAnc32h 同款） */
    uint64_t blink_rx_ts = get_rx_timestamp_u64();
    dwt_setdelayedtrxtime((uint32_t)(blink_rx_ts >> 8) + inst->timings.fixedReplyDelayAnc32h);
    inst->lastTxFcode = RTLS_MSG_RNG_INIT;
    if (dev_uwbStartTx(UWB_TX_MODE_DELAYED, false) == DWT_ERROR)
    {
        ANCH_FRAME_TRACE_LOG("DISC A%d rng_init starttx failed, slot=%d", anc_id, slot);
        anch_rxRenableImmdiate(inst);               // 过点即本次注册应答失败，标签会再 blink
        return;
    }
    /* 日志放在延时发送排定之后，EUI 格式化不占 blink→RNG_INIT 时间窗 */
    log_i("DISC: A%d rxd blink, tag eui %02X%02X%02X%02X%02X%02X%02X%02X -> slot %d",
          anc_id,
          rx_buffer[BLINK_TAG_EUI64_IDX + 7], rx_buffer[BLINK_TAG_EUI64_IDX + 6],
          rx_buffer[BLINK_TAG_EUI64_IDX + 5], rx_buffer[BLINK_TAG_EUI64_IDX + 4],
          rx_buffer[BLINK_TAG_EUI64_IDX + 3], rx_buffer[BLINK_TAG_EUI64_IDX + 2],
          rx_buffer[BLINK_TAG_EUI64_IDX + 1], rx_buffer[BLINK_TAG_EUI64_IDX],
          slot);
}

/* discovery：标签 EUI64 注册表顺序扫描（TREK anch_add_tag_to_list 移植），
 * 已注册返回原 slot，否则占用首个空槽；返回 -1 = 表满。下标即分配的 tag_id/时隙号 */
static int anch_add_tag_to_list(instance_data_t *inst, const uint8_t *eui64)
{
    static const uint8_t blank[8] = {0};

    for (int i = 0; i < MAX_TAG_LIST_SIZE; i++)
    {
        if (memcmp(inst->tagList[i], eui64, 8) == 0)    // 该标签已注册（重启/漏收 RNG_INIT 后重新 blink）
        {
            return i;
        }
        if (memcmp(inst->tagList[i], blank, 8) == 0)    // 首个空槽，接纳新标签
        {
            memcpy(inst->tagList[i], eui64, 8);
            inst->tagListLen = i + 1;
            return i;
        }
    }
    return -1;
}

/* discovery：RNG_INIT 帧打包写入发送缓存（TREK anch_prepare_anc2tag_rangeinitresponse 对应）。
 * sleepCorrection 额外 +1 周期（叠加 calc 结果后最小 1.5 周期，TREK 同款），
 * 载荷小端（与本工程 RESP 帧 sleepCorr 大端不同，见 dw_instance.h 索引区注释） */
static void anch_prepareRngInitResp(instance_data_t *inst, const uint8_t *tagEui64, uint16_t slot)
{
    int32_t sleepCorrection_ms = calc_tag_sleep_correction((uint8_t)slot, portGetTickCnt())
                               + (int32_t)inst_one_slot_time * inst_slot_number;

    tx_rng_init_msg[SEQ_NB_IDX] = inst->frame_seq_nb++;
    memcpy(&tx_rng_init_msg[RNG_INIT_DEST_ADDR_IDX], tagEui64, 8);
    tx_rng_init_msg[RNG_INIT_SRC_ADDR_IDX]     = anc_id;    // 源短地址 0x8000|anc_id 小端
    tx_rng_init_msg[RNG_INIT_SRC_ADDR_IDX + 1] = 0x80;
    tx_rng_init_msg[RNG_INIT_SLP_IDX]     = sleepCorrection_ms & 0xFF;
    tx_rng_init_msg[RNG_INIT_SLP_IDX + 1] = (sleepCorrection_ms >> 8) & 0xFF;
    tx_rng_init_msg[RNG_INIT_TAG_ADD_IDX]     = slot & 0xFF;
    tx_rng_init_msg[RNG_INIT_TAG_ADD_IDX + 1] = (slot >> 8) & 0xFF;

    dwt_writetxdata(RNG_INIT_MSG_LEN + FCS_LEN, tx_rng_init_msg, 0);
    dwt_writetxfctrl(RNG_INIT_MSG_LEN + FCS_LEN, 0, 1);
}

#if defined(ANCRANGE)
/* A2A模式下的基站切换为T2A模式下基站，同Tag进行TWR */
static void rnganch_changeBackToAnchor(instance_data_t *inst)
{
    inst->device_mode = ANCHOR;
    inst->final_rx_time32h = 0;
    inst->remainingRespToRxAnc = 0;     // A2A 专用计数器随本轮交换结束复位
    dwt_setrxtimeout(0);
    dwt_setrxaftertxdelay(0);
    anch_rxRenableImmdiate(inst);       // 内部复位 twr_mode/remainingRespToRx(T2A)/lastTxFcode
}

/* A2A 应答端 resp2 帧打包写入发送缓存（TREK anch_prepare_anc2anc_response 对应），
 * 收到发起端 poll 后调用，随后由 rnganch_txResponseOrRxReenable 排槽发送 */
static void anch_perpareAnc2AncResp(instance_data_t *inst, uint8_t initiator_id)
{
    tx_anch_resp2_msg[SEQ_NB_IDX] = inst->frame_seq_nb++;
    tx_anch_resp2_msg[PANID_IDX] = (uint8_t)PAN_ID;
    tx_anch_resp2_msg[PANID_IDX + 1] = (uint8_t)(PAN_ID >> 8);
    tx_anch_resp2_msg[SENDER_SHORT_ADD_IDX] = anc_id;
    tx_anch_resp2_msg[RECEIVER_SHORT_ADD_IDX] = 0xFF;
    tx_anch_resp2_msg[FUNC_CODE_IDX] = RTLS_MSG_ANCH_RESP2;
    tx_anch_resp2_msg[RANGE_NB_IDX] = inst->a2a_range_nb;
    int32_t prev_dis = inst->a2a_distance[initiator_id];    // 上一轮与发起端的测距结果回传
    tx_anch_resp2_msg[A2A_RESP2_PREV_DIS_IDX]     = (uint8_t)(prev_dis);
    tx_anch_resp2_msg[A2A_RESP2_PREV_DIS_IDX + 1] = (uint8_t)(prev_dis >> 8);
    tx_anch_resp2_msg[A2A_RESP2_PREV_DIS_IDX + 2] = (uint8_t)(prev_dis >> 16);
    tx_anch_resp2_msg[A2A_RESP2_PREV_DIS_IDX + 3] = (uint8_t)(prev_dis >> 24);

    dwt_writetxdata(ANCH_RESP2_MSG_LEN + FCS_LEN, tx_anch_resp2_msg, 0);
    dwt_writetxfctrl(ANCH_RESP2_MSG_LEN + FCS_LEN, 0, 1);
}

/* A2A 应答端排槽并延时发送 resp2（TREK rnganch_txResponseOrRxReenable 对应）：
 * 发送失败直接回 ANCHOR（TREK 是重开接收，本工程 resp2 过点即本轮报废，等下轮 poll）。
 * final 接收窗在 TX-done（sentHandle 的 RESP2 分支）里再开，不用 TREK 的 W4R 机制 */
static void rnganch_txResponseOrRxReenable(instance_data_t *inst, uint8_t initiator_id)
{
    uint8_t resp_position = anc_id - initiator_id - 1;
    /* A2A 的第一个 responder 原来直接占用首个响应槽，DW3000 上这个槽太紧，
     * 容易在 dwt_starttx() 时已经“过点”。回调虽已直驱状态机（无队列转发），
     * 仍保留整体后移一个槽的调度余量，SWO 实测收紧 RX_RESPONSE_TURNAROUND 时再评估。
     * 槽 k 的 RMARKER 时刻 = poll RX + (k+1)×fixedReplyDelay，responder 占槽 pos+1 */
    uint32_t resp_tx_time32h = (uint32_t)(inst->poll_rx_ts >> 8)
                             + (uint32_t)(resp_position + 1) * inst->timings.fixedReplyDelayAnc32h; // A1 resp帧发送的时间，安排在poll接收后，加上一个resp帧槽位，推测poll帧处理需要时间，函数调度也需要，要留出一个槽位的时间给MCU处理

    /* final 槽基准（开窗时再提前一个前导码） */
    inst->final_rx_time32h = (uint32_t)(inst->poll_rx_ts >> 8)
                            + (uint32_t)(A2A_FINAL_SCHEDULE_INDEX + 1) * inst->timings.fixedReplyDelayAnc32h;

    dwt_setdelayedtrxtime(resp_tx_time32h);
    inst->lastTxFcode = RTLS_MSG_ANCH_RESP2;            // TX-done 事件按此分流
    if (dev_uwbStartTx(UWB_TX_MODE_DELAYED, false) == DWT_ERROR)
    {
        // log_d("A2A A%d resp2 starttx failed, init=A%d pos=%d rn=%d",
        //         anc_id, initiator_id, resp_position, inst->a2a_range_nb);
        rnganch_changeBackToAnchor(inst);
    }
}

/* A2A 发起端 resp2 收到/槽超时后的三选一（TREK rnganch_rxRespSendfinal_or_rxReenable
 * ：还有槽且未到末台则继续开窗收 resp2；否则收到过 resp2 就发 final，一台都没收到
 * 回 ANCHOR。lastRespRxd = 收到的是末台（MAX_AHCHOR_NUMBER-1）的 resp2，缺席基站的槽
 * 已全部过去，直接发 final 不再空等（继续开窗失败时同样收尾，不让接收机停摆） */
static void rnganch_rxRespSendfinal_or_rxReenable(instance_data_t *inst, bool lastRespRxd)
{
    if (inst->remainingRespToRxAnc > 0 && !lastRespRxd)
    {
        dwt_setrxtimeout(inst->timings.fixedReplyDelay_sy + inst->timings.fwto4RespFrame_sy);
        dwt_setpreambledetecttimeout(0);
        if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_ERROR)
        {
            return;
        }
    }
    
    if (inst->rxRespMaskAnc != 0)   // A2A 发起端，resp标志不为0，代表至少有一次A2A resp帧接收成功，那么可以发送final帧，完成A2A TWR
    {
        rnganch_sendFinal(inst);
    }
    else
    {
        rnganch_changeBackToAnchor(inst);
    }
}

static void rnganch_start_a2a(instance_data_t *inst, uint8_t expectedResps)
{
    dwt_forcetrxoff();
    inst->device_mode = ANCHOR_RNG;
    inst->twr_mode = INITIATOR;
    inst->a2a_range_nb++;
    inst->remainingRespToRxAnc = expectedResps;
    inst->rxRespMaskAnc = 0;

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
    dwt_setrxtimeout((A2A_RESPONDER_COUNT + 1) * inst->timings.fixedReplyDelay_sy + inst->timings.fwto4RespFrame_sy);
    inst->lastTxFcode = RTLS_MSG_ANCH_POLL;         // TX-done 事件按此分流
    if (dev_uwbStartTx(UWB_TX_MODE_IMMEDIATE, true) == DWT_ERROR)  // 期待延迟接收
    {
        rnganch_changeBackToAnchor(inst);
    }
}

static void rnganch_sendFinal(instance_data_t *inst)
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
    tx_anch_final_msg[A2A_FINAL_VALID_IDX] = inst->rxRespMaskAnc;

    final_msg_set_ts(&tx_anch_final_msg[A2A_FINAL_POLL_TX_TS_IDX], inst->poll_tx_ts);
    /* FINAL 是延时发送、时间戳由调度时间手工算出，不是硬件读取，
     * 必须补上 TX 天线延时才能和 poll_tx/resp_tx(硬件已含天线延时)保持一致。
     * 漏加会让 responder 侧 Da 偏小、TOF 偏大(实测 ~15m 误差)。
     * 32h<<8 还原 40bit：MASK_TXDTS 本就丢弃低 9 位，bit9 以上无损。 */
    uint64_t final_tx_ts_embed = ((((uint64_t)inst->final_tx_time32h) << 8) & MASK_TXDTS) + ant_dly;
    final_msg_set_ts(&tx_anch_final_msg[A2A_FINAL_FINAL_TX_TS_IDX], final_tx_ts_embed);
    for (uint8_t i = 1; i <= A2A_RESPONDER_COUNT; i++)
    {
        if ((inst->rxRespMaskAnc >> i) & 0x01)
        {
            final_msg_set_ts(&tx_anch_final_msg[A2A_FINAL_RESP_RX_TS_BASE + (i - 1) * FINAL_MSG_TS_LEN], inst->resp_rx_ts[i]);
        }
    }

    dwt_writetxdata(ANCH_FINAL_MSG_LEN + FCS_LEN, tx_anch_final_msg, 0);
    dwt_writetxfctrl(ANCH_FINAL_MSG_LEN + FCS_LEN, 0, 1);
    dwt_setdelayedtrxtime(inst->final_tx_time32h);
    inst->lastTxFcode = RTLS_MSG_ANCH_FINAL;        // TX-done 事件按此分流
    if (dev_uwbStartTx(UWB_TX_MODE_DELAYED, false) == DWT_ERROR)
    {
        // ANCH_FRAME_TRACE_LOG("A2A A%d final starttx failed, mask=0x%02X rn=%d",
        //         anc_id, inst->rxRespMaskAnc, inst->a2a_range_nb);
        rnganch_changeBackToAnchor(inst);
    }
}

void anch_checkA2ATrigger(instance_data_t *inst)
{
    if (inst->device_mode != ANCHOR || inst->twr_mode != LISTENER)
        return;

    if (anc_id == 0 && portGetTickCnt() >= inst->a2aStartTime_ms)
    {
        inst->a2aStartTime_ms += inst->sframePeriod_ms;
        rnganch_start_a2a(inst, A2A_RESPONDER_COUNT);
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
