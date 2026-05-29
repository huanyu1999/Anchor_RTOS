#include "dw_instance.h"
#include "dw_sort.h"
#include "cmsis_os.h"
#include "elog.h"

// #define TWR_DEBUG
// extern double dwt_getrangebias(uint8 chan, float range, uint8 prf);

/* 保存当前ID标签的测距值，下次发送resp时发给标签 */

/* RESP数据帧格式 */
static uint8_t tx_resp_msg[RESP_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0x00, 0x80, FUNC_CODE_RESP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#if defined (ANCRANGE)
static uint8_t tx_sync_msg[SYNC_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x80, 0x00, 0x80, FUNC_CODE_SYNC};
#endif

/* 接收数据buffer */
static uint8_t rx_buffer[FRAME_LEN_MAX];

/* TWR时间戳，用于计算飞行时间 */
static uint64_t poll_rx_ts;    
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;
static uint16_t indexDiff_poll;
static uint16_t indexDiff_final;
static int32_t prev_range[MAX_TAG_LIST_SIZE];
static float distance_now_m;
static tag_hashNode_t send_processeDis;
static uint8_t resp_valid = 0x00;                    // 基站数据有效标志 
static uint8_t handleResp_times;                     // 对基站发送或者接收resp帧计数，初始化为当前测距的基站个数

static int twrAnchor_Init(dwDevice_t *dev);
static uint32_t twrAnchor_onEvent(dwDevice_t *dev, uwbEvent_t event);
static void twrAnchor_rxOkHandle(void);
static void twrAnchor_sentHandle(void);
static uint8_t twrAnchor_rxErrorOrTimeoutHandle(void);
static void anch_txRespOrRxReEnable(void);
static void anch_rxRenableImmdiate(dwDevice_t *dev);
static void anch_perpareAnc2TagResp(void);

static inline uint64_t get_tx_timestamp_u64(void);
static inline uint64_t get_rx_timestamp_u64(void);
static inline void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);
static inline void final_msg_set_ts(uint8_t *ts_field, uint64_t ts);

// 单套 TWR anchor 算法，DW1000/DW3000 通过函数内 USE_DW1000/USE_DW3000 条件编译区分
uwbAlgorithm_t uwbTwr_AnchorAlgorithm = { .init = twrAnchor_Init, .onEvent = twrAnchor_onEvent };

/*******************************************DW1000 event function********************************************/
static int twrAnchor_Init(dwDevice_t *dev)
{
    dev = get_the_local_structure_of_dev();
#if defined(USE_DW3000)
    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN | DWT_FF_ACK_EN);
#else
    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);  // 设置帧过滤模式开启
#endif
    dwt_setpreambledetecttimeout(0);                        // 清除前导码超时，一直接收
    dwt_setrxtimeout(0);                                    // 清除接收数据超时，一直接收
    int ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);         // 打开接收机，等待接收数据，初始接收，进行一次buffer对齐
    if(ret == DWT_ERROR)                                    // 打开接收失败
    {
        return 0;
    }
    dev->twr_mode = RESPONDER_T;                            // twr模式设置，表示同基站测距，接收poll
    return 1;
}

static uint32_t twrAnchor_onEvent(dwDevice_t *dev, uwbEvent_t event)
{
    UNUSED(dev);
    switch (event)
    {
    case eventPacketReceived:
        twrAnchor_rxOkHandle();
        break;

    case eventPacketSent:
        twrAnchor_sentHandle();
        break;

    case eventReceiveFailed:
    case eventReceiveTimeout:
        twrAnchor_rxErrorOrTimeoutHandle();
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
    dwDistance_t* distance =  get_the_local_structure_of_dis();
    
    uint32 frame_len;
#if defined(USE_DW3000)
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
#else
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);                          /* Clear good RX frame event in the DW1000 status register. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;          /* A frame has been received, read it into the local buffer. */
#endif
    
    if (frame_len < FRAME_LEN_MAX)
    {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    uint8_t f_code = rx_buffer[FUNC_CODE_IDX];

    switch (f_code)
    {
    case FUNC_CODE_POLL:
        range_nb    = rx_buffer[RANGE_NB_IDX];             // 取range_nb，resp发送时发送相同的range_nb
        recv_tag_id = rx_buffer[SENDER_SHORT_ADD_IDX];     // 取发送标签的ID
        if(recv_tag_id >= inst_slot_number)                // 标签ID如果大于标签总容量则退出
        {
            anch_rxRenableImmdiate(dev);             // 直接开启下一轮接收poll，无需同步buffer，暂定没问题
            break;
        }

        range_time = portGetTickCnt();            // 取得twr刚开始接收到poll的Tick
        poll_rx_ts = get_rx_timestamp_u64();      // 获得poll_rx时间戳      // 使用memcpy 直接获取？？
        dev->wait4final = 0;                      // 等待final标志位
        dev->remainingRespToRx = MAX_AHCHOR_NUMBER - 1;         // 基站需要接收的其他基站的resp帧的数量
        handleResp_times = MAX_AHCHOR_NUMBER;                   // 基站需要处理resp帧的次数
        dev->respTxIndex = 0x01 << anc_id;        // 该标志位标识当前基站发送resp帧的位置，先右移，0号基站接收poll帧后，直接发送，1号基站延后一位
        anch_perpareAnc2TagResp();                // 可以在这里就将要发送的resp帧写入发送缓存
        anch_txRespOrRxReEnable();                // 判断是否发送resp帧，还是重新打开接收，接收其他基站的resp帧，还是重新打开接收，接收标签final帧
#if defined(USE_DW1000)
        dev->indexDiff_poll = uwb_isInNLOS_index(FUNC_CODE_POLL);
#endif
        break;

    case FUNC_CODE_FINAL:                        // 基站每接收到一个标签的final帧，就进行一次TOF计算
        if (((rx_buffer[RANGE_NB_IDX] == range_nb) && (rx_buffer[SENDER_SHORT_ADD_IDX] == recv_tag_id))) // 验证final帧的range_nb和标签ID是否和之前的poll帧一致
        {
            dev->twr_mode = LISTENER ;           // 接收到final不用答复，故设置为listener模式
            dev->wait4final = 0; 
            resp_valid = rx_buffer[FINAL_MSG_FINAL_VALID_IDX];
            range_time = portGetTickCnt();         // 再次获取TWR成功，final帧接收的Tick
            resp_tx_ts = get_tx_timestamp_u64();   // 取得resp_tx时间戳
            final_rx_ts = get_rx_timestamp_u64();  // 取得final_rx时间戳
            // if(rx_buffer[FINAL_MSG_A0_GROUP_ID_IDX + anc_id * 5] != (group_id & 0x7f))   // 验证标签final内的基站组号和当前组号相同
            // {
            //     printf("recv = %x, me = %x\n", rx_buffer[FINAL_MSG_A0_GROUP_ID_IDX + anc_id * 5], group_id & 0x7f);
            //     resp_valid = resp_valid & (uint8_t)(~(0x01 << anc_id));                  // 设置该基站无效
            // }

            if((resp_valid >> anc_id) & 0x01)                                               // final消息中，本基站发送的resp消息是有效的,则进行距离计算，或者发送时间戳
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
                if (tof > 0x7FFFFFFF)                     // 如果TOF值溢出
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
#if defined(USE_DW1000)
                dev->indexDiff_final = uwb_isInNLOS_index(FUNC_CODE_FINAL);
                int twr_quality = check_twr_quality(dev->indexDiff_poll, dev->indexDiff_final);
                dev->indexDiff_poll = 0;
                dev->indexDiff_final = 0;
                // float dis_toss =  fabsf(distance_now_m -((float)prev_range[recv_tag_id] / 1000.0f));
                // if (prev_range[recv_tag_id] > 0 && dis_toss >= 5)
                // {
                //     distance_now_m = (float)(prev_range[recv_tag_id]) / 1000.0f; // 如果测距结果出现不符合人体正常移动速度的波动，就滤掉本次结果，use last distance.
                // }
#ifdef TWR_DEBUG
                if (twr_quality == -1)             // TWR质量不合格，拒绝此次测距结果，沿用上次测距结果，这里质量差的判别存在问题
                {
                    log_d("TWR C, tag %d dis : %.2f", recv_tag_id, (float)(distance_now_m));
                }
                else if (twr_quality == 0)
                {
                    prev_range[recv_tag_id] = distance_now_m * 1000;    // 使用本次TWR结果，更新prev_range，并将单位转换为m
                    log_d("TWR A, tag %d dis : %.2f", recv_tag_id, (float)(prev_range[recv_tag_id]) / 1000.0);
                }
                else if (twr_quality == 1)
                {
                    prev_range[recv_tag_id] = distance_now_m * 1000;    // 使用本次TWR结果，更新prev_range，并将单位转换为m，
                    log_d("TWR B, tag %d dis : %.2f", recv_tag_id, (float)(prev_range[recv_tag_id]) / 1000.0);
                }
#endif
#endif  // USE_DW1000
                // send_processeDis.distance = prev_range[recv_tag_id];
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

    case FUNC_CODE_RESP:
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

    default:
        break;
    }
}

static void twrAnchor_sentHandle(void)
{
    anch_txRespOrRxReEnable();                      // 发送resp帧之后，触发中断
}

static uint8_t twrAnchor_rxErrorOrTimeoutHandle(void)
{
    dwDevice_t* dev = get_the_local_structure_of_dev();

    // 面对接收超时，分为3种，接收poll帧，接收resp帧，接收final帧
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

        int ret = dwt_starttx(DWT_START_TX_DELAYED);  // 延时发送
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
        // else if (dev->remainingRespToRx == -1)
        // {
        //     range_status = RANGE_ERROR; 
        //     anch_rxRenableImmdiate(dev);
        // }
        else                                                           // 打开延迟接收，用于接收resp帧
        {
#if defined(USE_DW3000)
            dwt_configureframefilter(0, 0);                     // 关闭帧过滤，能够接收所有数据
#else
            dwt_enableframefilter(DWT_FF_NOTYPE_EN);            // 关闭帧过滤，能够接收所有数据
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
            // dev->remainingRespToRx--;                             // 不管是否接收成功，剩余接收的resp帧数量减一
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
    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);  // 设置帧过滤模式开启
#endif
    dwt_setpreambledetecttimeout(0);                        // 清除前导码超时，一直接收
    dwt_setrxtimeout(0);                                       // 清除接收数据超时，一直接收
    int ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);            // 打开接收机，等待接收数据    
    if (ret == DWT_ERROR)                                            // 打开接收失败
    {
        // anch_rxRenableImmdiate();                                 // 处理重新打开接收
        // return 0;
    }
    dev->twr_mode = RESPONDER_T;                                     // twr模式设置，表示同基站测距，接收poll
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
    tx_resp_msg[FUNC_CODE_IDX]          = FUNC_CODE_RESP;
    tx_resp_msg[RESP_MSG_GROUP_IDX]     = group_id;

    // 将上次的测距值打包在resp中发给标签, 暂时先不用，目前方案不太适合
    // if(range_nb == prev_range[recv_tag_id].range_nb + 1)
    // {
    //     tx_resp_msg[RESP_MSG_PREV_DIS_IDX]   = prev_range[recv_tag_id] >> 24;
    //     tx_resp_msg[RESP_MSG_PREV_DIS_IDX+1] = prev_range[recv_tag_id] >> 16;
    //     tx_resp_msg[RESP_MSG_PREV_DIS_IDX+2] = prev_range[recv_tag_id] >> 8;
    //     tx_resp_msg[RESP_MSG_PREV_DIS_IDX+3] = prev_range[recv_tag_id];
    // }

    if (anc_id == 0)                                      // A0负责校准标签时序，防冲突
    {
        tx_resp_msg[RESP_MSG_GROUP_IDX] = group_id | 0x80; // 参与时序校准
        int error = 0;
        int currentSlotTime = 0;
        int expectedSlotTime = 0;
        int sframePeriod_ms = inst_one_slot_time * inst_slot_number;    // sframePeriod_ms 为整个TWR周期的总时间= 单slot时间*slot个数(标签总容量)
        int slotDuration_ms = inst_one_slot_time;                      // slotDuration_ms 为单slot时间
        int tagSleepCorrection_ms = 0;
        
        currentSlotTime  = range_time % sframePeriod_ms;        // currentSlotTime 当前正在通信标签的实际slot
        expectedSlotTime = recv_tag_id * slotDuration_ms;        // expectedSlotTime 当前正在通信标签应该处于的slot
        error = expectedSlotTime - currentSlotTime;             // error 计算slot差异 用于校准

        if (error < (-(sframePeriod_ms >> 1))) // if error is more  than 0.5 period, add whole period to give up to 1.5 period sleep
        {
            tagSleepCorrection_ms = (sframePeriod_ms + error);
        }
        else //the minimum Sleep time will be 0.5 period
        {
            tagSleepCorrection_ms = error;
        }

        tx_resp_msg[RESP_MSG_SLEEP_COR_IDX] = (tagSleepCorrection_ms >> 8) & 0xFF; // 校准时间高8位存储在 index 11 
        tx_resp_msg[RESP_MSG_SLEEP_COR_IDX + 1] = tagSleepCorrection_ms & 0xFF;// 校准时间低8位存储在 index 12
    }
    else
    {
        tx_resp_msg[RESP_MSG_GROUP_IDX] = group_id & 0x7f;  //不参与时序校准
        tx_resp_msg[RESP_MSG_SLEEP_COR_IDX] = 0;
        tx_resp_msg[RESP_MSG_SLEEP_COR_IDX + 1] = 0;
    }

    dwt_writetxdata(RESP_MSG_LEN + FCS_LEN, tx_resp_msg, 0); //数据写入DW1000数据缓冲区
    dwt_writetxfctrl(RESP_MSG_LEN + FCS_LEN, 0, 1); 
}

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
    // int8_t i;
    dwt_readtxtimestamp(ts_tab);
    // for (i = 4; i >= 0; i--)
    // {
    //     ts <<= 8;
    //     ts |= ts_tab[i];
    // }

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
    // int8_t i;
    dwt_readrxtimestamp(ts_tab);
    // for (i = 4; i >= 0; i--)
    // {
    //     ts <<= 8;
    //     ts |= ts_tab[i];
    // }

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
    // uint8_t i;
    // *ts = 0;
    // for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    // {
    //     *ts += ((uint32_t)ts_field[i] << (i * 8));
    // }
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
    // uint8_t i;
    // for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    // {
    //     ts_field[i] = (uint8_t)ts;
    //     ts >>= 8;
    // }
    memcpy(ts_field, &ts, 4);
}
