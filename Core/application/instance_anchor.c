#include "instance.h"

extern double dwt_getrangebias(uint8 chan, float range, uint8 prf);

/* 保存当前ID标签的测距值，下次发送resp时发给标签 */
typedef struct {
    uint8_t range_nb;
    int32_t distance;
} prev_range_t;

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

/* 发送和接收数据中断标志 */
static volatile uint8_t rx_status = RX_WAIT;
static volatile uint8_t tx_status = TX_WAIT;

static prev_range_t prev_range[MAX_TAG_LIST_SIZE];
static uint8_t resp_valid = 0x00;                    //基站数据有效标志 
static uint8_t sr, rr;                               //用于控制当前基站处于resp时是发送还是接收,sr用于确认基站发送resp后，
static dwt_rxdiag_t rx_diag;                         // 計算接收功率

static void twrAnchor_rxOkHandle(void);
static uint8_t twrAnchor_rxErrorOrTimeout(void);
static void twrAnchor_sentHandler(void);
static void anch_txRespOrRxReenale(void);
static int twrAnchor_Init(dwDevice_t *dev);
static uint32_t twrAnchor_onEvent(dwDevice_t *dev, uwbEvent_t event);
static void anch_calcTof(uint8_t *data, uint64_t anchor_respTxTime, uint64_t tag_finalRxTime, uint64_t tag_pollRxTime, uint8_t add_idx);
static void anch_rxRenableImmdiate(dwDevice_t *dev);
static void anch_perpareAnc2TagResp(void);

/**
  * @brief  uwb 基站状态机处理
  * @param  none 
  * @retval none
  */
// void anchor_app(void)
// {
//     dwDistance_t* distance =  get_the_local_structure_of_dis();

//     switch (state)
//     {
//         case STA_INIT_POLL_SYNC: //初始化接收机，接收poll消息
//         {
//             dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);  //设置帧过滤模式开启
//             dwt_setpreambledetecttimeout(0);                        //清除前导码超时，一直接收
//             dwt_setrxtimeout(0);                                    //清除接收数据超时，一直接收
//             int ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);         //打开接收机，等待接收数据    
//             if(ret == DWT_ERROR)
//             {
//                 state = STA_INIT_POLL_SYNC;
//                 break;
//             }

//             rx_status = RX_WAIT;                                    //清rx标志位，中断服务函数更改其状态
//             state = STA_WAIT_POLL_SYNC;
//             break;
//         }
        
//         case STA_WAIT_POLL_SYNC:                                   //等待poll消息，中断回调函数状态变更
//         {
//             if(rx_status == RX_OK)                                 //接收到数据
//             {
//                 state = STA_RECV_POLL_SYNC;
//             }
//             else if((rx_status == RX_TIMEOUT) || (rx_status == RX_ERROR))  //接收数据错误，重新开启接收
//             {
//                 state = STA_INIT_POLL_SYNC;
//             }

// #if defined (ANCRANGE) //设置基站间测距模式开启 1=开始 2=运行中 0=停止
//             static uint32_t next_sync_time = 0;                     //基站A0下次广播并启动测距的时间

//             if(dev_id == 0)                                         //基站A0定时发送数据，轮发其他基站启动测距
//             {
//                 if((portGetTickCnt() > (next_sync_time)) && (ancrange_flag == 2)) //通知下一个基站
//                 {
//                     ancrange_flag = 1;
//                     target_ancid++;
//                     if(target_ancid >= MAX_AHCHOR_NUMBER)
//                     {
//                         target_ancid = 1;
//                     }
//                 }
//                 else if((ancrange_flag == 1) && (anc_id == 0))
//                 {
//                     next_sync_time = portGetTickCnt() + inst_one_slot_time * inst_slot_number * (ANC_RANGE_COUNT + 0);
//                     ancrange_flag = 2;                              //设置基站间测距模式开启 1=开始 2=运行中 0=停止
//                     dwt_forcetrxoff();
//                     state = STA_SEND_SYNC;
//                     //printf("send sync\r\n");
//                     //led_toggle(LED1); 
//                 }
//             }
// #endif
//             break;
//         }

// #if defined (ANCRANGE) 
//         case STA_SEND_SYNC:  //A0负责发送时间戳并通知其他基站启动测距  todo 指定对方地址模式，非广播
//         {
//             /* sync数据打包 */
//             tx_sync_msg[SEQ_NB_IDX] = frame_seq_nb++;  
//             tx_sync_msg[PANID_IDX] = (uint8_t)PAN_ID; 
//             tx_sync_msg[PANID_IDX + 1] = (uint8_t)(PAN_ID>>8); 
//             tx_sync_msg[SENDER_SHORT_ADD_IDX] = anc_id;
//             tx_sync_msg[RECEIVER_SHORT_ADD_IDX] = target_ancid;
//             tx_sync_msg[FUNC_CODE_IDX] = FUNC_CODE_SYNC;
//             uint32_t now_time = portGetTickCnt();

//             tx_sync_msg[SYNC_MSG_TIME_IDX + 0] = now_time;
//             tx_sync_msg[SYNC_MSG_TIME_IDX + 1] = now_time >> 8;
//             tx_sync_msg[SYNC_MSG_TIME_IDX + 2] = now_time >> 16;
//             tx_sync_msg[SYNC_MSG_TIME_IDX + 3] = now_time >> 24;

//             dwt_writetxdata(SYNC_MSG_LEN + FCS_LEN, tx_sync_msg, 0);    //数据写入DW3000数据缓冲区
//             dwt_writetxfctrl(SYNC_MSG_LEN + FCS_LEN, 0, 1);             //配置TX帧控制寄存器
//             tx_status = TX_WAIT;                                        //发送状态标志，在中断回调函数变更
//             int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);              //立即发送SYNC消息
//             if(ret == DWT_ERROR)
//             {
//                 dwt_forcetrxoff();
//                 state = STA_INIT_POLL_SYNC;
//                 break;
//             }             
//             while(tx_status == TX_WAIT);                             //等待发送成功
//             tx_status = TX_WAIT;                                     //清标志     
//             state = STA_INIT_POLL_SYNC;     
//             break;
//         }
// #endif

//         case STA_RECV_POLL_SYNC://接收处理poll消息
//             if(rx_buffer[FUNC_CODE_IDX] == FUNC_CODE_POLL)      //判断收到的数据是POLL
//             {
//                 range_nb    = rx_buffer[RANGE_NB_IDX];             //取range_nb，resp发送时发送相同的range_nb
//                 recv_tag_id = rx_buffer[SENDER_SHORT_ADD_IDX];  //取发送标签的ID
//                 sos         = rx_buffer[POLL_MSG_SOS_IDX];
//                 if(sos > 1) { sos = 0; }
                
//                 if(recv_tag_id >= inst_slot_number)             //标签ID如果大于标签总容量则退出
//                 {
//                     state = STA_INIT_POLL_SYNC;
//                     break;
//                 }
                
//                 for(int i = 0; i < 10; i++)             //接收用户字节
//                 {
//                     user_data[i] = rx_buffer[POLL_MSG_USER_IDX + i];
//                 }

//                 range_time = portGetTickCnt();          // 取得测距时间
//                 poll_rx_ts = get_rx_timestamp_u64();    // 获得poll_rx时间戳
//                 distance->sort_distance1[recv_tag_id].poll_receiveSign = 0x01; // 获取到该标签的poll帧，设置为有效
                
//                 sr = MAX_AHCHOR_NUMBER;
//                 rr = 0x01 << anc_id;
//                 rx_status = RX_WAIT;
//                 state = STA_SORR_RESP;
//                 // led_on(RUN_LED1);
//             }
// #if defined (ANCRANGE)
//             else if(rx_buffer[FUNC_CODE_IDX] == FUNC_CODE_SYNC)      //其他基站接收到基站A0的时间同步消息
//             {
//                 uint32_t sync_time    = (uint32_t)rx_buffer[SYNC_MSG_TIME_IDX + 3] << 24;
//                 sync_time = sync_time | (uint32_t)rx_buffer[SYNC_MSG_TIME_IDX + 2] << 16;
//                 sync_time = sync_time | (uint32_t)rx_buffer[SYNC_MSG_TIME_IDX + 1] << 8;
//                 sync_time = sync_time | (uint32_t)rx_buffer[SYNC_MSG_TIME_IDX + 0];

//                 if(inst_dataRate == DWT_BR_110K)
//                 {
//                     sync_time = sync_time + 2;//110k发送耗时约2ms补偿
//                 }
//                 sys_time_diff = HAL_GetTick() - sync_time;  //时间同步
//                 range_nb = 0;
//                 ancrange_flag = 1; //设置基站间测距模式开启 1=开始 2=运行中 0=停止
//                 instance_mode = TAG;  //切换当前角色控制为标签
//                 temp_dev_id = dev_id; //备份当前ID
//                 dev_id = 0;
//                 ancrange_count = ANC_RANGE_COUNT;
//                 set_instance();
//                 break;
//                 //state = STA_INIT_POLL_SYNC;    
//             }
// #endif
//             else //非POLL数据重新开启接收POLL
//             {
//                 state = STA_INIT_POLL_SYNC;
//             }

//             break;

//         case STA_SORR_RESP://根据基站ID按顺序进行发送或接收resp消息
//             if(sr > 0)// sr>0 处于resp阶段，按基站ID判断接收resp或发送resp
//             {
//                 if(rr & 0x01)//当前该基站需发送resp
//                 {
//                     rr = 0;
//                     state = STA_SEND_RESP;
//                 }
//                 else//当前该基站需接收resp
//                 {
//                    dwt_enableframefilter(DWT_FF_NOTYPE_EN); //关闭帧过滤，能够接收所有数据

//                     //设置resp数据接收机开启时间
//                     uint64_t resp_rx_time;
//                     if(inst_dataRate == DWT_BR_110K)
//                         resp_rx_time = (poll_rx_ts + ((FIRST_RESP_SEND_110K + (MAX_AHCHOR_NUMBER - sr) * inst_data_interval) * UUS_TO_DWT_TIME));

//                     else if(inst_dataRate == DWT_BR_6M8)
//                         resp_rx_time = (poll_rx_ts + ((FIRST_RESP_SEND_6P8M + (MAX_AHCHOR_NUMBER - sr) * inst_data_interval) * UUS_TO_DWT_TIME));

//                     else if(inst_dataRate == DWT_BR_850K)
//                         resp_rx_time = (poll_rx_ts + ((FIRST_RESP_SEND_850K + (MAX_AHCHOR_NUMBER - sr) * inst_data_interval) * UUS_TO_DWT_TIME));

//                     resp_rx_time = resp_rx_time >> 8;
//                     dwt_setdelayedtrxtime(resp_rx_time);                    //设置接收机开启延时时间
//                     dwt_setrxtimeout(inst_resp_rx_timeout);                 //设置接收数据超时时间
//                     dwt_setpreambledetecttimeout(PRE_TIMEOUT);               //设置接收前导码超时时间
//                     int ret = dwt_rxenable(DWT_START_RX_DELAYED);          //延时开启接收机
//                     if(ret == DWT_ERROR)
//                     {
//                         state = STA_INIT_POLL_SYNC;
//                         break;
//                     }   
//                     rr = rr >> 1;
//                     state = STA_WAIT_RESP;
//                 }
//                 sr = sr - 1; 
//             }
//             else//准备接收final
//             {      
//                //final数据的接收机开启时间，提前100us开启
//                 uint64_t final_rx_time = (poll_rx_ts + inst_poll2final_time);              
//                 final_rx_time = final_rx_time >> 8;
//                 dwt_setdelayedtrxtime((uint32)final_rx_time);  //设置接收机开启延时时间
//                 dwt_setrxtimeout(inst_final_rx_timeout);       //设置接收数据超时时间
//                 dwt_setpreambledetecttimeout(PRE_TIMEOUT);     //设置接收前导码超时时间
//                 int ret = dwt_rxenable(DWT_START_RX_DELAYED);            //延时开启接收机
//                 if(ret == DWT_ERROR)
//                 {
//                     state = STA_INIT_POLL_SYNC;
//                     break;
//                 }
//                 state = STA_WAIT_FINAL;
//             }
//             break;

//         case STA_SEND_RESP: //打包发送resp消息
//         {
//             //设置resp消息发送时间
//             uint64_t resp_tx_time;
//             if(inst_dataRate == DWT_BR_110K)
//                 resp_tx_time = (poll_rx_ts + (FIRST_RESP_SEND_110K + anc_id * inst_data_interval) * UUS_TO_DWT_TIME + (ANC_RESP_SEND_BACK_110K * UUS_TO_DWT_TIME));
//             else if(inst_dataRate == DWT_BR_6M8)
//                 resp_tx_time = (poll_rx_ts + (FIRST_RESP_SEND_6P8M + anc_id * inst_data_interval) * UUS_TO_DWT_TIME + (ANC_RESP_SEND_BACK_6P8M * UUS_TO_DWT_TIME));
//             else if(inst_dataRate == DWT_BR_850K)   
//                 resp_tx_time = (poll_rx_ts + (FIRST_RESP_SEND_850K + anc_id * inst_data_interval) * UUS_TO_DWT_TIME + (ANC_RESP_SEND_BACK_850K * UUS_TO_DWT_TIME));
//             resp_tx_time = resp_tx_time >> 8;
//             dwt_setdelayedtrxtime((uint32)resp_tx_time);

//             /* resp数据打包 */
//             tx_resp_msg[SEQ_NB_IDX] = frame_seq_nb++;
//             tx_resp_msg[PANID_IDX] = (uint8_t)PAN_ID; 
//             tx_resp_msg[PANID_IDX + 1] = (uint8_t)(PAN_ID>>8); 
//             tx_resp_msg[RANGE_NB_IDX] = range_nb;
//             tx_resp_msg[SENDER_SHORT_ADD_IDX] = anc_id;
//             tx_resp_msg[RECEIVER_SHORT_ADD_IDX] = recv_tag_id;
//             tx_resp_msg[FUNC_CODE_IDX] = FUNC_CODE_RESP;
//             tx_resp_msg[RESP_MSG_GROUP_IDX] = group_id;

//             //将上次的测距值打包在resp中发给标签
//             //if(range_nb == prev_range[recv_tag_id].range_nb + 1)
//             {
//                 tx_resp_msg[RESP_MSG_PREV_DIS_IDX]   = prev_range[recv_tag_id].distance >> 24;
//                 tx_resp_msg[RESP_MSG_PREV_DIS_IDX+1] = prev_range[recv_tag_id].distance >> 16;
//                 tx_resp_msg[RESP_MSG_PREV_DIS_IDX+2] = prev_range[recv_tag_id].distance >> 8;
//                 tx_resp_msg[RESP_MSG_PREV_DIS_IDX+3] = prev_range[recv_tag_id].distance;
//             }

//             if(anc_id == 0)//A0负责校准标签时序，防冲突
//             {
//                 tx_resp_msg[RESP_MSG_GROUP_IDX] = group_id | 0x80; //参与时序校准
//                 int error = 0;
//                 int currentSlotTime = 0;
//                 int expectedSlotTime = 0;
//                 int sframePeriod_ms = inst_one_slot_time * inst_slot_number;    //sframePeriod_ms 为整个TWR周期的总时间= 单slot时间*slot个数(标签总容量)
//                 int slotDuration_ms = inst_one_slot_time;                       //slotDuration_ms 为单slot时间
//                 int tagSleepCorrection_ms = 0;
                
//                 currentSlotTime = range_time % sframePeriod_ms;     //currentSlotTime 当前正在通信标签的实际slot
//                 expectedSlotTime = recv_tag_id * slotDuration_ms;   //expectedSlotTime 当前正在通信标签应该处于的slot
//                 error = expectedSlotTime - currentSlotTime;         //error 计算slot差异 用于校准

//                 if(error < (-(sframePeriod_ms>>1))) //if error is more  than 0.5 period, add whole period to give up to 1.5 period sleep
//                 {
//                     tagSleepCorrection_ms = (sframePeriod_ms + error);
//                 }
//                 else //the minimum Sleep time will be 0.5 period
//                 {
//                     tagSleepCorrection_ms = error;
//                 }

//                 tx_resp_msg[RESP_MSG_SLEEP_COR_IDX] = (tagSleepCorrection_ms >> 8) & 0xFF;//高8位存11 
//                 tx_resp_msg[RESP_MSG_SLEEP_COR_IDX + 1] = tagSleepCorrection_ms & 0xFF;//低8位存12
//             }
//             else
//             {
//                 tx_resp_msg[RESP_MSG_GROUP_IDX] = group_id & 0x7f;  //不参与时序校准
//                 tx_resp_msg[RESP_MSG_SLEEP_COR_IDX] = 0;
//                 tx_resp_msg[RESP_MSG_SLEEP_COR_IDX + 1] = 0;
//             }

//             dwt_writetxdata(RESP_MSG_LEN + FCS_LEN, tx_resp_msg, 0); //数据写入DW1000数据缓冲区
//             dwt_writetxfctrl(RESP_MSG_LEN + FCS_LEN, 0, 1); 
//             tx_status = TX_WAIT; 
//             int ret = dwt_starttx(DWT_START_TX_DELAYED);  //延时发送
//             if(ret == DWT_ERROR)
//             {
//                 dwt_forcetrxoff();
//                 state = STA_INIT_POLL_SYNC;
//                 break;
//             }  
//             while(tx_status == TX_WAIT);//等待发送结束，状态在中断回调函数中变更
//             tx_status = TX_WAIT;
//             state = STA_SORR_RESP;
//             break;
//         }

//         case STA_WAIT_RESP:  //等待接收resp消息，在中断回调函数内rx_status状态变更
//             if(rx_status == RX_OK)
//             {
//                 rx_status = RX_WAIT;
//                 state = STA_RECV_RESP;
//             }
//             else if((rx_status == RX_TIMEOUT)||(rx_status == RX_ERROR))
//             {
//                 rx_status = RX_WAIT;
//                 state = STA_SORR_RESP;
//             }   
//             break;

//         case STA_RECV_RESP: //接收到其他基站的resp消息
//             {
//                 if(rx_buffer[FUNC_CODE_IDX] == FUNC_CODE_RESP)//正确接收到resp消息
//                 {
//                     if(rx_buffer[RANGE_NB_IDX] == range_nb)//和当前测距具有相同的range_nb
//                     {
//                         if((rx_buffer[RESP_MSG_GROUP_IDX] & 0x7f) == (group_id & 0x7f))//只取和自己组号相同的其他基站数据上报
//                         {
//                             uint8_t recv_anc_id = rx_buffer[SENDER_SHORT_ADD_IDX]; //取基站ID
//                             distance_report[recv_anc_id]  = (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX]   << 24;
//                             distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+1] << 16;
//                             distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+2] << 8;
//                             distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+3];

//                             group_report[recv_anc_id] = rx_buffer[RESP_MSG_GROUP_IDX] & 0x7f;  //将最高bit置0，最高bit为校准基站标志位
//                         } 
//                     }
//                 }
//                 state = STA_SORR_RESP;
//             }
//                 break;

//         case STA_WAIT_FINAL:  //等待接收final消息，在中断回调函数内rx_status状态变更
//             if(rx_status == RX_OK)
//             {   
//                 rx_status = RX_WAIT;
//                 state = STA_RECV_FINAL;
//             }
//             else if((rx_status == RX_TIMEOUT) || (rx_status == RX_ERROR))
//             {
//                 rx_status = RX_WAIT;
//                 state = STA_INIT_POLL_SYNC;
//                 range_status = RANGE_ERROR; 
//             }
//             break;

//         case STA_RECV_FINAL:  // 接收到final消息，数据处理
//             // 验证function code为final，与poll相同的range number 和相同的tag id 则进行TWR测距计算 
//             if ((rx_buffer[FUNC_CODE_IDX] == FUNC_CODE_FINAL) && (rx_buffer[RANGE_NB_IDX] == range_nb) && (rx_buffer[SENDER_SHORT_ADD_IDX] == recv_tag_id))  
//             {
//                 // distance_report[anc_id] = prev_range[recv_tag_id].distance;  //将上次的测距值写入distance_report用于串口输出
//                 // group_report[anc_id] = group_id & 0x7f;
//                 resp_valid = rx_buffer[FINAL_MSG_FINAL_VALID_IDX];
//                 distance->sort_distance1[recv_tag_id].final_receiveSign = 0x01;           // 获取到该标签的final帧，设置为有效
//                 // if(rx_buffer[FINAL_MSG_A0_GROUP_ID_IDX + anc_id * 5] != (group_id & 0x7f)) // 验证标签final内的基站组号和当前组号相同
//                 // {
//                 //     printf("recv = %x, me = %x\n", rx_buffer[FINAL_MSG_A0_GROUP_ID_IDX + anc_id * 5], group_id & 0x7f);
//                 //     resp_valid = resp_valid & (uint8_t)(~(0x01 << anc_id));     //设置该基站无效
//                 // }

//                 if((resp_valid >> anc_id) & 0x01)                               //final消息中，本基站发送的resp消息是有效的,则进行距离计算
//                 {
//                     uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
//                     uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
//                     double Ra, Rb, Da, Db;
//                     int64_t tof_dtu;
//                     double tof;

//                     resp_tx_ts = get_tx_timestamp_u64();   //取得resp_tx时间戳
//                     final_rx_ts = get_rx_timestamp_u64();  //取得final_rx时间戳

//                     /* 从final消息中，取得poll_tx时间戳，resp_rx时间戳，final_tx时间戳 */
//                     final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
//                     final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP1_RX_TS_IDX + anc_id * (FINAL_MSG_TS_LEN + 1)], &resp_rx_ts);
//                     final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

//                     /* 计算飞行时间 */
//                     poll_rx_ts_32 = (uint32_t)poll_rx_ts;
//                     resp_tx_ts_32 = (uint32_t)resp_tx_ts;
//                     final_rx_ts_32 = (uint32_t)final_rx_ts;
//                     Ra = (double)(resp_rx_ts - poll_tx_ts);
//                     Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
//                     Da = (double)(final_tx_ts - resp_rx_ts);
//                     Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
//                     tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

//                     tof = (int32)tof_dtu; 
//                     if (tof > 0x7FFFFFFF) 
//                     {
//                         tof -= 0x80000000;  
//                     }

//                     tof = tof * DWT_TIME_UNITS;
//                     distance_now_m = tof * SPEED_OF_LIGHT;
//                     distance_now_m = distance_now_m - dwt_getrangebias(inst_ch, (float)distance_now_m, inst_prf);
//                     if(distance_now_m > 20000.000)  
//                     {
//                         distance_now_m = -1;
//                     }
                    
//                     // distance_now_m = distance_now_m - (float)distance_offset_cm / 100.0f;         //校准（PDOA模式）
                    
//                     //更新prev_range为本次测距值
//                     prev_range[recv_tag_id].distance = distance_now_m * 1000;//单位转换为mm
//                     prev_range[recv_tag_id].range_nb = range_nb;
//                     // double rx_power =  calculate_RSSI(&rx_diag);
//                     // printf_use_dma("POWER : %.4f dBM\r\n", rx_power);
//                     // printf("poll_tx_ts %lu resp_rx_ts %lu final_tx_ts %lu\n", poll_tx_ts, resp_rx_ts, final_tx_ts);
//                     // printf("poll_rx_ts %d resp_tx_ts %d final_rx_ts %d\n", poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32);
//                     // ("Ra %.lf Rb %.lf Da %.lf Db %.lf\n", Ra, Rb, Da, Db);
//                     // printf_use_dma("tag %d dis : %.2f \n", recv_tag_id, (float)(prev_range[recv_tag_id].distance) / 1000.0);
                    
//                     /* 将获取到的各标签最新距离存入排序数组 */
//                     distance->sort_distance1[recv_tag_id].tag_distance = prev_range[recv_tag_id].distance;

//                     range_status = RANGE_TWR_OK;            // 设置TWR成功测距标志，在dw_main.c里判断打包串口输出
//                     distance->newRange = 0x01;
//                 }
//                 else
//                 {
//                     prev_range[recv_tag_id].distance = -1;
//                 }

//             }
//             dwt_forcetrxoff();
//             state = STA_INIT_POLL_SYNC;
//             break;
        
//         default:
//             break;
//     }
// }

/*******************************************event function********************************************/
static void twrAnchor_rxOkHandle(void)
{
    dwDevice_t* dev = get_the_local_structure_of_dev();
    dwDistance_t* distance =  get_the_local_structure_of_dis();
    
    uint32 frame_len;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);                                 /* Clear good RX frame event in the DW1000 status register. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;                /* A frame has been received, read it into the local buffer. */
    
    if (frame_len < FRAME_LEN_MAX)
    {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    uint8_t f_code = rx_buffer[FUNC_CODE_IDX];

    switch (f_code)
    {
    case FUNC_CODE_POLL:
        range_nb    = rx_buffer[RANGE_NB_IDX];             //取range_nb，resp发送时发送相同的range_nb
        recv_tag_id = rx_buffer[SENDER_SHORT_ADD_IDX];     //取发送标签的ID
        sos         = rx_buffer[POLL_MSG_SOS_IDX];
        if(sos > 1) { sos = 0; }
        
        if(recv_tag_id >= inst_slot_number)                //标签ID如果大于标签总容量则退出
        {
            anch_rxRenableImmdiate(dev);                      // 直接开启下一轮接收poll
            break;
        }
        
        range_time = portGetTickCnt();          // 取得测距时间
        poll_rx_ts = get_rx_timestamp_u64();    // 获得poll_rx时间戳
        distance->sort_distance1[recv_tag_id].poll_receiveSign = 0x01; // 获取到该标签的poll帧，设置为有效
        dev->wait4final = 0;
        dev->remainingRespToRx = 2;
        sr = MAX_AHCHOR_NUMBER;
        dev->rxEnIndex = 0x01 << anc_id;
        // dev->twr_mode = RESPONDER_T;
        // rr = 0x01 << anc_id;
        anch_perpareAnc2TagResp();              // 可以在这里就将要发送的resp帧写入发送缓存
        anch_txRespOrRxReenale();               // 判断是否发送resp帧，还是重新打开接收，接收其他基站的resp帧
        break;
    
    case FUNC_CODE_FINAL:
        if (((rx_buffer[RANGE_NB_IDX] == range_nb) && (rx_buffer[SENDER_SHORT_ADD_IDX] == recv_tag_id)))
        {
            dev->twr_mode = LISTENER ;              // 接收到final不用答复，故设置为listener模式
            dev->wait4final = 0; 
            resp_valid = rx_buffer[FINAL_MSG_FINAL_VALID_IDX];
            distance->sort_distance1[recv_tag_id].final_receiveSign = 0x01;           // 获取到该标签的final帧，设置为有效
            // if(rx_buffer[FINAL_MSG_A0_GROUP_ID_IDX + anc_id * 5] != (group_id & 0x7f)) // 验证标签final内的基站组号和当前组号相同
            // {
            //     printf("recv = %x, me = %x\n", rx_buffer[FINAL_MSG_A0_GROUP_ID_IDX + anc_id * 5], group_id & 0x7f);
            //     resp_valid = resp_valid & (uint8_t)(~(0x01 << anc_id));     //设置该基站无效
            // }

            if((resp_valid >> anc_id) & 0x01)                               //final消息中，本基站发送的resp消息是有效的,则进行距离计算
            {
                uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                double Ra, Rb, Da, Db;
                int64_t tof_dtu;
                double tof;

                resp_tx_ts = get_tx_timestamp_u64();   //取得resp_tx时间戳
                final_rx_ts = get_rx_timestamp_u64();  //取得final_rx时间戳

                /* 从final消息中，取得poll_tx时间戳，resp_rx时间戳，final_tx时间戳 */
                final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP1_RX_TS_IDX + anc_id * (FINAL_MSG_TS_LEN + 1)], &resp_rx_ts);
                final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                /* 计算飞行时间 */
                poll_rx_ts_32 = (uint32_t)poll_rx_ts;
                resp_tx_ts_32 = (uint32_t)resp_tx_ts;
                final_rx_ts_32 = (uint32_t)final_rx_ts;
                Ra = (double)(resp_rx_ts - poll_tx_ts);
                Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                Da = (double)(final_tx_ts - resp_rx_ts);
                Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                tof = (int32)tof_dtu; 
                if (tof > 0x7FFFFFFF) 
                {
                    tof -= 0x80000000;  
                }

                tof = tof * DWT_TIME_UNITS;
                distance_now_m = tof * SPEED_OF_LIGHT;
                distance_now_m = distance_now_m - dwt_getrangebias(inst_ch, (float)distance_now_m, inst_prf);
                if(distance_now_m > 20000.000)  
                {
                }
                
                // distance_now_m = distance_now_m - (float)distance_offset_cm / 100.0f;         //校准（PDOA模式）
                
                //更新prev_range为本次测距值
                prev_range[recv_tag_id].distance = distance_now_m * 1000;//单位转换为mm
                prev_range[recv_tag_id].range_nb = range_nb;
                // double rx_power =  calculate_RSSI(&rx_diag);
                // printf_use_dma("POWER : %.4f dBM\r\n", rx_power);
                // printf("poll_tx_ts %lu resp_rx_ts %lu final_tx_ts %lu\n", poll_tx_ts, resp_rx_ts, final_tx_ts);
                // printf("poll_rx_ts %d resp_tx_ts %d final_rx_ts %d\n", poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32);
                // ("Ra %.lf Rb %.lf Da %.lf Db %.lf\n", Ra, Rb, Da, Db);
                // printf_use_dma("tag %d dis : %.2f \n", recv_tag_id, (float)(prev_range[recv_tag_id].distance) / 1000.0);
                
                    distance_now_m = -1;
                /* 将获取到的各标签最新距离存入排序数组 */
                distance->sort_distance1[recv_tag_id].tag_distance = prev_range[recv_tag_id].distance;

                range_status = RANGE_TWR_OK;            // 设置TWR成功测距标志，在dw_main.c里判断打包串口输出
                led_toggle(uwb_ok_led);
                distance->newRange = 0x01;
            }
            else
            {
                prev_range[recv_tag_id].distance = -1;
            }
        }   
        dwt_forcetrxoff();                              // 本次测距结束，重新使能立即接收
        anch_rxRenableImmdiate(dev);
        break;

    case FUNC_CODE_RESP:
        if(rx_buffer[RANGE_NB_IDX] == range_nb)   // 和当前测距具有相同的range_nb
        {   
            dev->rxResp++;                     // 接收到一次其他基站发送的resp帧  
            
            if((rx_buffer[RESP_MSG_GROUP_IDX] & 0x7f) == (group_id & 0x7f))//只取和自己组号相同的其他基站数据上报
            {
                uint8_t recv_anc_id = rx_buffer[SENDER_SHORT_ADD_IDX]; //取基站ID
                distance_report[recv_anc_id]  = (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX]   << 24;
                distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+1] << 16;
                distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+2] << 8;
                distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+3];

                group_report[recv_anc_id] = rx_buffer[RESP_MSG_GROUP_IDX] & 0x7f;  //将最高bit置0，最高bit为校准基站标志位
            } 
        }
        // 不管校验有没有通过，都要去处理后续的操作
        dev->remainingRespToRx--;          // 剩余接收resp帧数量减一
        anch_txRespOrRxReenale();          // 判断是否发送resp帧，还是重新打开接收，接收其他基站的resp帧

        break;

    default:
        break;
    }
}

static uint8_t twrAnchor_rxErrorOrTimeout(void)
{
    dwDevice_t* dev = get_the_local_structure_of_dev();

    // 面对接收超时，分为3种，接收poll帧，接收resp帧，接收final帧
    // 接收poll异常，重新打开接收
    // 接收final异常，重新打开接收
    // 接收resp异常，继续后续的发送resp或者接收resp，或者接收final
    if (dev->remainingRespToRx == -1)           // 说明是接收poll帧出现异常
    {
        anch_rxRenableImmdiate(dev);
        return 0;
    }
    
    if (dev->remainingRespToRx >= 0)             // 说明是接收resp帧出现异常
    {
        dev->remainingRespToRx--;
        anch_txRespOrRxReenale();
        return 1;
    }

    return 3;
}

static void twrAnchor_sentHandler(void)
{
    // printf_use_dma("Base ID %d sending RESP at %lu\n", anc_id, portGetTickCnt());
    anch_txRespOrRxReenale();                      // 发送resp帧之后，触发中断
}

static void anch_txRespOrRxReenale(void)
{
    dwDevice_t* dev = get_the_local_structure_of_dev();
    int send_resp = 0;
    
    if (dev->remainingRespToRx == 0)                // 接收处理完毕，等待接收final，设定延迟接收
    {
        dev->wait4final = WAIT4TAGFINAL;
    }

    // if ((dev->remainingRespToRx + dev->device_id) == 2) 
    if (dev->rxEnIndex & 0x01)
    {
        dev->rxEnIndex = 0;
        send_resp = 1;
    }

    if (send_resp == 1)
    {
        // 轮到该基站发送resp帧
        uint64_t resp_tx_time;
        if(inst_dataRate == DWT_BR_110K)
            resp_tx_time = (poll_rx_ts + (FIRST_RESP_SEND_110K + anc_id * inst_data_interval) * UUS_TO_DWT_TIME + (ANC_RESP_SEND_BACK_110K * UUS_TO_DWT_TIME));
        else if(inst_dataRate == DWT_BR_6M8)
            resp_tx_time = (poll_rx_ts + (FIRST_RESP_SEND_6P8M + anc_id * inst_data_interval) * UUS_TO_DWT_TIME + (ANC_RESP_SEND_BACK_6P8M * UUS_TO_DWT_TIME));
        else if(inst_dataRate == DWT_BR_850K)   
            resp_tx_time = (poll_rx_ts + (FIRST_RESP_SEND_850K + anc_id * inst_data_interval) * UUS_TO_DWT_TIME + (ANC_RESP_SEND_BACK_850K * UUS_TO_DWT_TIME));
        
        resp_tx_time = resp_tx_time >> 8;
        dwt_setdelayedtrxtime((uint32)resp_tx_time);

        int ret = dwt_starttx(DWT_START_TX_DELAYED);  //延时发送
        if(ret == DWT_ERROR)
        {    
            anch_rxRenableImmdiate(dev);                 // 发送resp失败，说明测距失败，重新使能立即接收，接收poll帧,是否直接return？
        } 
        else
        {
            sr = sr - 1;
        }
    }
    else      // 继续接收
    {
        if (dev->remainingRespToRx == 0)                   // 不再期望接收任何Resp帧，接收final
        {
            uint64_t final_rx_time = (poll_rx_ts + inst_poll2final_time);              
            final_rx_time = final_rx_time >> 8;
            dwt_setdelayedtrxtime((uint32)final_rx_time);  // 设置接收机开启延时时间
            dwt_setrxtimeout(inst_final_rx_timeout);       // 设置接收数据超时时间
            dwt_setpreambledetecttimeout(PRE_TIMEOUT);     // 设置接收前导码超时时间
            int ret = dwt_rxenable(DWT_START_RX_DELAYED);  // 延时开启接收机
            if(ret == DWT_ERROR)                           // 打开失败，立即重新打开接收，相当于本次测距失败，重新接收poll帧
            {
                anch_rxRenableImmdiate(dev);
            }
        }
        else if (dev->remainingRespToRx == -1)
        {
            anch_rxRenableImmdiate(dev);
        }
        else                                               // 打开延迟接收，用于接收resp帧
        {
            dwt_enableframefilter(DWT_FF_NOTYPE_EN);        // 关闭帧过滤，能够接收所有数据

            //设置resp数据接收机开启时间
            uint64_t resp_rx_time;
            if(inst_dataRate == DWT_BR_110K)
                resp_rx_time = (poll_rx_ts + ((FIRST_RESP_SEND_110K + (MAX_AHCHOR_NUMBER - sr) * inst_data_interval) * UUS_TO_DWT_TIME));

            else if(inst_dataRate == DWT_BR_6M8)
                resp_rx_time = (poll_rx_ts + ((FIRST_RESP_SEND_6P8M + (MAX_AHCHOR_NUMBER - sr) * inst_data_interval) * UUS_TO_DWT_TIME));

            else if(inst_dataRate == DWT_BR_850K)
                resp_rx_time = (poll_rx_ts + ((FIRST_RESP_SEND_850K + (MAX_AHCHOR_NUMBER - sr) * inst_data_interval) * UUS_TO_DWT_TIME));

            resp_rx_time = resp_rx_time >> 8;
            dwt_setdelayedtrxtime(resp_rx_time);                   // 设置接收机开启延时时间
            dwt_setrxtimeout(inst_resp_rx_timeout);                // 设置接收数据超时时间
            dwt_setpreambledetecttimeout(PRE_TIMEOUT);             // 设置接收前导码超时时间
            int ret = dwt_rxenable(DWT_START_RX_DELAYED);          // 延时开启接收机
            if(ret == DWT_ERROR)
            {
                anch_rxRenableImmdiate(dev);                          // 接收机开启失败，直接立即打开接收，重回测距开始阶段，接收poll帧 
            }
            sr = sr - 1;
            dev->rxEnIndex = dev->rxEnIndex >> 1;
        }
    }
}

static int twrAnchor_Init(dwDevice_t *dev)
{
    dev = get_the_local_structure_of_dev();
    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);  // 设置帧过滤模式开启
    dwt_setpreambledetecttimeout(0);                        // 清除前导码超时，一直接收
    dwt_setrxtimeout(0);                                    // 清除接收数据超时，一直接收
    int ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);         // 打开接收机，等待接收数据    
    if(ret == DWT_ERROR)                                    // 打开接收失败
    {
        // anch_rxRenableImmdiate();                           // 处理重新打开接收
        return 0;
    }
    dev->twr_mode = RESPONDER_T;                            // twr模式设置，表示同基站测距，接收poll
    return 1;
}

static uint32_t twrAnchor_onEvent(dwDevice_t *dev, uwbEvent_t event)
{
    switch (event)
    {
    case eventPacketReceived:
        twrAnchor_rxOkHandle();
        break;

    case eventPacketSent:
        twrAnchor_sentHandler();
        break;

    case eventReceiveFailed:
    case eventReceiveTimeout:
        twrAnchor_rxErrorOrTimeout();
        break;    
    
    default:

        break;
    }

    return portMAX_DELAY;
}

uwbAlgorithm_t uwbTwr_AnchorAlgorithm = {
    .init = twrAnchor_Init,
    .onEvent = twrAnchor_onEvent
};

static void anch_calcTof(uint8_t *data, uint64_t anchor_respTxTime, uint64_t tag_finalRxTime, uint64_t tag_pollRxTime, uint8_t add_idx)
{
    // double Ra, Rb, Da, Db;
    // uint64_t tag_finalTxTime  = 0;          // 标签的final帧发送时间
    // uint64_t tag_pollTxTime  = 0;           // 标签的poll帧发送时间
    // uint64_t anchor_respRxTime  = 0;        // 标签接收到基站的resp帧的接收时间
}

// 使能立即接收
static void anch_rxRenableImmdiate(dwDevice_t *dev)
{
    dev = get_the_local_structure_of_dev();
    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);  // 设置帧过滤模式开启
    dwt_setpreambledetecttimeout(0);                        // 清除前导码超时，一直接收
    dwt_setrxtimeout(0);                                    // 清除接收数据超时，一直接收
    int ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);         // 打开接收机，等待接收数据    
    if(ret == DWT_ERROR)                                    // 打开接收失败
    {
        // anch_rxRenableImmdiate();                           // 处理重新打开接收
        // return 0;
    }
    dev->twr_mode = RESPONDER_T;                            // twr模式设置，表示同基站测距，接收poll
    // return 1;
}

// 使能立即接收
static void anch_rxRenableDelayed(uint32_t dly_time)
{
    
    
    
}

// 将要发送的resp帧做好，存入发送缓存
static void anch_perpareAnc2TagResp(void)
{
    /* resp数据打包 */
    tx_resp_msg[SEQ_NB_IDX] = frame_seq_nb++;
    tx_resp_msg[PANID_IDX] = (uint8_t)PAN_ID; 
    tx_resp_msg[PANID_IDX + 1] = (uint8_t)(PAN_ID>>8); 
    tx_resp_msg[RANGE_NB_IDX] = range_nb;
    tx_resp_msg[SENDER_SHORT_ADD_IDX] = anc_id;
    tx_resp_msg[RECEIVER_SHORT_ADD_IDX] = recv_tag_id;
    tx_resp_msg[FUNC_CODE_IDX] = FUNC_CODE_RESP;
    tx_resp_msg[RESP_MSG_GROUP_IDX] = group_id;

    //将上次的测距值打包在resp中发给标签
    //if(range_nb == prev_range[recv_tag_id].range_nb + 1)
    {
        tx_resp_msg[RESP_MSG_PREV_DIS_IDX]   = prev_range[recv_tag_id].distance >> 24;
        tx_resp_msg[RESP_MSG_PREV_DIS_IDX+1] = prev_range[recv_tag_id].distance >> 16;
        tx_resp_msg[RESP_MSG_PREV_DIS_IDX+2] = prev_range[recv_tag_id].distance >> 8;
        tx_resp_msg[RESP_MSG_PREV_DIS_IDX+3] = prev_range[recv_tag_id].distance;
    }

    if(anc_id == 0)                                        // A0负责校准标签时序，防冲突
    {
        tx_resp_msg[RESP_MSG_GROUP_IDX] = group_id | 0x80; // 参与时序校准
        int error = 0;
        int currentSlotTime = 0;
        int expectedSlotTime = 0;
        int sframePeriod_ms = inst_one_slot_time * inst_slot_number;    // sframePeriod_ms 为整个TWR周期的总时间= 单slot时间*slot个数(标签总容量)
        int slotDuration_ms = inst_one_slot_time;                       // slotDuration_ms 为单slot时间
        int tagSleepCorrection_ms = 0;
        
        currentSlotTime = range_time % sframePeriod_ms;     // currentSlotTime 当前正在通信标签的实际slot
        expectedSlotTime = recv_tag_id * slotDuration_ms;   // expectedSlotTime 当前正在通信标签应该处于的slot
        error = expectedSlotTime - currentSlotTime;         // error 计算slot差异 用于校准

        if(error < (-(sframePeriod_ms>>1))) //if error is more  than 0.5 period, add whole period to give up to 1.5 period sleep
        {
            tagSleepCorrection_ms = (sframePeriod_ms + error);
        }
        else //the minimum Sleep time will be 0.5 period
        {
            tagSleepCorrection_ms = error;
        }

        tx_resp_msg[RESP_MSG_SLEEP_COR_IDX] = (tagSleepCorrection_ms >> 8) & 0xFF;//高8位存11 
        tx_resp_msg[RESP_MSG_SLEEP_COR_IDX + 1] = tagSleepCorrection_ms & 0xFF;//低8位存12
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
