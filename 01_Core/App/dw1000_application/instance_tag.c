#include "instance.h"


/* poll数据帧格式 */
static uint8_t tx_poll_msg[POLL_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x00, 0x00, FUNC_CODE_POLL, 0x00, 0x00, 0x00, 0x00};
/* final数据帧格式 */
static uint8_t tx_final_msg[FIANL_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x00, 0x00, FUNC_CODE_FINAL, 0X00};
/* 接收数据buffer */
static uint8_t rx_buffer[FRAME_LEN_MAX];

static uint8_t resp_expect = MAX_AHCHOR_NUMBER;         //resp消息接收个数
static int tagSleepCorrection_ms = 0;                   //标签时序校准，用于slot分配防冲突管理
uint32_t volatile next_period_time = 0;                          //标签下次测距周期开始时间，用于防冲突管理
uint8_t Correction_flag = 0;                            //该标签已经被时序校准的标志位

/* TWR时间戳，用于计算飞行时间 */
static uint64_t poll_tx_ts;                     
static uint64_t resp_rx_ts[MAX_AHCHOR_NUMBER];
static uint64_t final_tx_ts;

/* 发送和接收数据中断标志 */
static volatile uint8_t rx_status = RX_WAIT;
static volatile uint8_t tx_status = TX_WAIT;

static uint8_t resp_valid = 0x00;               //基站数据有效标志
uint32 diff_time;                               //标签增加发起测距随机时间，避免冲突


static uint8_t led_flag = 0;
static uint32_t led_time = 0;     

void tag_app(void)
{
    switch (state)
    {
        case STA_SEND_POLL:  //打包和发送poll消息
        {
            if(inst_slot_number > 1)
            {
                diff_time += (tag_id + 1); //产生随机时间避免一直冲突     
                if(diff_time > inst_one_slot_time * inst_slot_number / 2)
                {
                    diff_time = 0;
                }
            }
            else
            {
                diff_time = 0;
            }
                     
            range_nb++;
            /* poll数据打包 */
            tx_poll_msg[SEQ_NB_IDX] = frame_seq_nb++;  
            tx_poll_msg[PANID_IDX] = (uint8_t)PAN_ID; 
            tx_poll_msg[PANID_IDX + 1] = (uint8_t)(PAN_ID>>8); 
            tx_poll_msg[RANGE_NB_IDX] = range_nb;  
            tx_poll_msg[SENDER_SHORT_ADD_IDX] = tag_id;
            tx_poll_msg[FUNC_CODE_IDX] = FUNC_CODE_POLL;
            tx_poll_msg[POLL_MSG_SOS_IDX] = sos;
            if(alarm > 0)
                alarm = 1;
            tx_poll_msg[POLL_MSG_ALARM_STA_IDX] = alarm;
#if defined(ANCRANGE)  
            if(ancrange_flag > 0)
            {
                battery = 0xff;
            }
#endif
            // tx_poll_msg[POLL_MSG_BATTERY_IDX] = battery;

            for(int i = 0; i < 10; i++)  //打包用户字节
            {
                tx_poll_msg[POLL_MSG_USER_IDX + i] = user_data[i];
            }

            range_time = portGetTickCnt();                           //获取测距时间，用于dw_main.c中串口打包
            dwt_writetxdata(POLL_MSG_LEN + FCS_LEN, tx_poll_msg, 0); //数据写入DW3000数据缓冲区
            dwt_writetxfctrl(POLL_MSG_LEN + FCS_LEN, 0, 1);          //配置TX帧控制寄存器
            tx_status = TX_WAIT;                                     //发送状态标志，在中断回调函数变更
            int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);           //立即发送POLL消息
            if(ret == DWT_ERROR)
            {
                next_period_time = range_time + inst_one_slot_time * inst_slot_number + ((Correction_flag == 1)?0:(diff_time));  //如该标签未被时序校准，初次上电，增加随机时间，避免一直冲突      
                state = STA_IDLE;
                break;
            }
            while(tx_status == TX_WAIT);                        //等待发送成功，tx_status在发送成功中断内变更状态
            tx_status = TX_WAIT;                                //清标志
            
            poll_tx_ts = get_tx_timestamp_u64();                //取得poll_tx时间戳

            resp_expect = MAX_AHCHOR_NUMBER;                    //发送poll消息后，等待最大基站数量个resp消息回复

            resp_valid = 0;                                     //resp有效校验初始化
            memset(rx_buffer, 0, sizeof(rx_buffer));
            //发送POLL之后，延时开启接收等待resp
            dwt_setrxtimeout(inst_resp_rx_timeout);     //设置接收超时时间
            dwt_setpreambledetecttimeout(PRE_TIMEOUT);  //设置前导码超时
            uint32_t resp_rx_time;
            if(inst_dataRate == DWT_BR_110K)
                resp_rx_time = (poll_tx_ts + (FIRST_RESP_SEND_110K * UUS_TO_DWT_TIME)) >> 8;
            else if(inst_dataRate == DWT_BR_6M8)
                resp_rx_time = (poll_tx_ts + (FIRST_RESP_SEND_6P8M * UUS_TO_DWT_TIME)) >> 8;
            else if(inst_dataRate == DWT_BR_850K)
                resp_rx_time = (poll_tx_ts + (FIRST_RESP_SEND_850K * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(resp_rx_time);        //设置接收机开启延时时间
            ret = dwt_rxenable(DWT_START_RX_DELAYED);   //延时开启接收机
            if(ret == DWT_ERROR)
            {
                next_period_time = range_time + inst_one_slot_time * inst_slot_number + ((Correction_flag == 1)?0:(diff_time));  //如该标签未被时序校准，初次上电，增加随机时间，避免一直冲突      
                state = STA_IDLE;
                break;
            }
            alarm = 0;
            for(int i = 0; i < 10; i++)  //清除用户字节
            {
                user_data[i] = 0;
            }
#if defined(ANCRANGE)        
            if(ancrange_flag == 2)
            {
                ancrange_count--;
            }
#endif
            rx_status = RX_WAIT;
            state = STA_WAIT_RESP;
            printf_use_dma("tx poll \r\n");
        }
            break;

        case STA_WAIT_RESP: //等待resp数据接收
        {
            if(rx_status == RX_OK)  //接收成功
            {
                state = STA_RECV_RESP;
            }
            else if((rx_status == RX_TIMEOUT) || (rx_status == RX_ERROR))//接收超时或接收错误
            {
                state = STA_RECV_RESP;
                // printf_use_dma("rx reception.\n");
            }
            if(portGetTickCnt() >= (range_time + 50))  //超时没有中断信号，故障，重启
            {
                HAL_NVIC_SystemReset();  //重启
            }
        }
            break;

        case STA_RECV_RESP:
        {
            static uint8_t resp_recved = 0;     //单slot内接收到的resp个数，用于判定如果1个resp没收到则不发送final
            static uint8_t no_resp_count = 0;   //未接收到任何基站回复的周期计数
            if((rx_status == RX_OK) && (rx_buffer[FUNC_CODE_IDX] != FUNC_CODE_RESP))    //接收消息成功，但是消息不是resp消息则直接进入IDLE
            {
                next_period_time = range_time + inst_one_slot_time * inst_slot_number + ((Correction_flag == 1)?0:(diff_time));  //如该标签未被时序校准，初次上电，增加随机时间，避免一直冲突      
                state = STA_IDLE;
            }
            else
            {
                if(rx_buffer[FUNC_CODE_IDX] == FUNC_CODE_RESP)      //正确接收到resp消息
                {
                    recv_anc_id = rx_buffer[SENDER_SHORT_ADD_IDX];  //取发送方基站ID
                    if(rx_buffer[RANGE_NB_IDX] == range_nb)         //和poll相同的range_nb
                    {
                        resp_valid = resp_valid | (0x01 << recv_anc_id);  //设置该基站resp有效，
                        resp_rx_ts[recv_anc_id] = get_rx_timestamp_u64(); //取该基站resp_rx时间戳，用于tof计算
                        resp_recved++;
                        /* 将resp消息内的测距信息取出，用于串口数据打包输出 */
                        distance_report[recv_anc_id]  = (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX]   << 24;
                        distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+1] << 16;
                        distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+2] << 8;
                        distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+3];

                        group_report[recv_anc_id] = rx_buffer[RESP_MSG_GROUP_IDX] & 0x7f; //将最高bit置0，最高bit为校准基站标志位
                        led_toggle(RUN_LED1);
                    }
                    alarm += rx_buffer[RESP_MSG_ALARM_IDX];
                    //当收到A0基站的resp时，取tagSleepCorrection用于校准
                    //if(recv_anc_id == 0)
                    //当基站组ID的最高bit = 1时，为时序校准基站，取tagSleepCorrection用于校准
                    if((rx_buffer[RESP_MSG_GROUP_IDX] != 0xff) && ((rx_buffer[RESP_MSG_GROUP_IDX] & 0x80) == 0x80))
                    {
                        tagSleepCorrection_ms = (int16) (((uint16) rx_buffer[RESP_MSG_SLEEP_COR_IDX] << 8) + rx_buffer[RESP_MSG_SLEEP_COR_IDX+1]);//高8位存11  低8位存12
                        Correction_flag = 1;
                        dwt_rxdiag_t rx_diag;
                        //dwt_readdiagnostics(&rx_diag);//读取信号强度等诊断信息
                        rx_power = rx_diag.rxPower;


                    }
                }
                resp_expect--;
                if(resp_expect == 0)//所有resp消息接收完成，发送final
                {
                    if(resp_recved == 0)//如果一个resp也没收到，则不发final，直接进入IDLE
                    {
                        if((sos == 0) && (alarm == 0))
                        {
//                            led_off(LED3);//绿
//                            led_off(LED2);//蓝
//                            led_toggle(LED1); //红灯闪烁
                            //motor_off();
                        }
                        no_resp_count++;
                        if(no_resp_count > 10)
                        {
                            Correction_flag = 0;
                            no_resp_count = 0;
                        }
                        next_period_time = range_time + inst_one_slot_time * inst_slot_number + ((Correction_flag == 1)?0:(diff_time));  //如该标签未被时序校准，初次上电，增加随机时间，避免一直冲突
                     
                        state = STA_IDLE;
                        range_status = RANGE_ERROR; 
                        break;
                        
                    }
                    else //收到1个及以上resp消息
                    {
                        if((sos == 0) && (alarm == 0))
                        {
//                            led_off(LED1);//红
//                            led_off(LED2);//蓝
//                            led_toggle(LED3); //绿灯闪烁
                            //motor_off();

                        }
                        state = STA_SEND_FINAL;
                        resp_recved = 0;
                        no_resp_count = 0;
                    }
                }
                else//继续接收其他resp消息
                {
                    //设置resp数据接收机开启时间
                    uint32_t resp_rx_time;
                    if(inst_dataRate == DWT_BR_110K)

                        resp_rx_time = (poll_tx_ts + ((FIRST_RESP_SEND_110K + (MAX_AHCHOR_NUMBER - resp_expect) * inst_data_interval) * UUS_TO_DWT_TIME)) >> 8;

                    else if(inst_dataRate == DWT_BR_6M8)

                        resp_rx_time = (poll_tx_ts + ((FIRST_RESP_SEND_6P8M + (MAX_AHCHOR_NUMBER - resp_expect) * inst_data_interval) * UUS_TO_DWT_TIME)) >> 8;

                    else if(inst_dataRate == DWT_BR_850K)

                        resp_rx_time = (poll_tx_ts + ((FIRST_RESP_SEND_850K + (MAX_AHCHOR_NUMBER - resp_expect) * inst_data_interval) * UUS_TO_DWT_TIME)) >> 8;

                    dwt_setdelayedtrxtime(resp_rx_time);                //设置接收机开启延时时间
                    int ret = dwt_rxenable(DWT_START_RX_DELAYED);       //延时开启接收机
                    if(ret == DWT_ERROR)
                    {
                        //next_period_time = range_time + inst_one_slot_time * inst_slot_number;//设置下个周期开始时间
                        next_period_time = range_time + inst_one_slot_time * inst_slot_number + ((Correction_flag == 1)?0:(diff_time));  //如该标签未被时序校准，初次上电，增加随机时间，避免一直冲突
                        state = STA_IDLE;
                        break;
                    }
                    state = STA_WAIT_RESP;
                }
            }
            rx_status = RX_WAIT;
        }
            break;

        case STA_SEND_FINAL:
        {
            uint64_t final_tx_time;  //设置final发送时间
            if(inst_dataRate == DWT_BR_110K)
                final_tx_time = (poll_tx_ts + inst_poll2final_time + TAG_FINALE_SEND_BACK_110K * UUS_TO_DWT_TIME);  //设置final发送时间
            else if(inst_dataRate == DWT_BR_6M8)
                final_tx_time = (poll_tx_ts + inst_poll2final_time + TAG_FINALE_SEND_BACK_6P8M * UUS_TO_DWT_TIME);  //设置final发送时间
            else if(inst_dataRate == DWT_BR_850K)
                final_tx_time = (poll_tx_ts + inst_poll2final_time + TAG_FINALE_SEND_BACK_850K * UUS_TO_DWT_TIME);  //设置final发送时间
            final_tx_time = final_tx_time >> 8;
            dwt_setdelayedtrxtime((uint32)final_tx_time); //在final_tx_time这个时间发送数据
            final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + ant_dly;  //final发送时间戳

            //final数据包内写入poll_tx时间戳，final_tx时间戳，4个resp_rx时间戳
            final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
            final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);
            for(int i = 0; i < MAX_AHCHOR_NUMBER; i++)
            {
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP1_RX_TS_IDX + i * (FINAL_MSG_TS_LEN + 1)], resp_rx_ts[i]);
            }

            //final数据打包
            tx_final_msg[SEQ_NB_IDX] = frame_seq_nb++;
            tx_final_msg[PANID_IDX] = (uint8_t)PAN_ID; 
            tx_final_msg[PANID_IDX + 1] = (uint8_t)(PAN_ID>>8); 
            tx_final_msg[RANGE_NB_IDX] = range_nb;
            tx_final_msg[SENDER_SHORT_ADD_IDX] = tag_id;
            tx_final_msg[FINAL_MSG_FINAL_VALID_IDX] = resp_valid;
            tx_final_msg[FUNC_CODE_IDX] = FUNC_CODE_FINAL;

            for(int i = 0; i < MAX_AHCHOR_NUMBER; i++)
            {
                tx_final_msg[FINAL_MSG_A0_GROUP_ID_IDX + i*5] = group_report[i];
            }

            dwt_writetxdata(FIANL_MSG_LEN + FCS_LEN, tx_final_msg, 0); //数据写入数据缓冲区
            dwt_writetxfctrl(FIANL_MSG_LEN + FCS_LEN, 0, 1); 

            tx_status = TX_WAIT;                              //发送状态标志，在中断回调函数变更
            int ret = dwt_starttx(DWT_START_TX_DELAYED);      //延时发送
            if(ret == DWT_ERROR)
            {
                next_period_time = range_time + inst_one_slot_time * inst_slot_number;//设置下个周期开始时间
                state = STA_IDLE;
                break;
            }
            while(tx_status == TX_WAIT);            //等待发送成功，tx_status在发送成功中断内变更状态
            tx_status = TX_WAIT;                    //清标志
            range_status = RANGE_TWR_OK;            //设置TWR成功测距标志，在dw_main.c里判断打包串口输出       
            next_period_time = range_time + inst_one_slot_time * inst_slot_number + tagSleepCorrection_ms;  //设置下个周期开始时间
            tagSleepCorrection_ms = 0;
            state = STA_IDLE;

            break;
        }
        
        case STA_IDLE:
        {

            static unsigned long nowtime;
            nowtime = portGetTickCnt();
#if defined(ANCRANGE)
            if((ancrange_count == 0) && (ancrange_flag == 2)) //本次切换T0测距结束，切回基站
            {
                dev_id = temp_dev_id;
                instance_mode = ANCHOR; 
                ancrange_flag = 0;
                set_instance();
                break;
            }
            
            if(ancrange_flag == 1) //基站切换tag，第一次发送poll按同步时间slot发送
            {
                if(nowtime % (inst_one_slot_time * inst_slot_number) == 0) //用T0 slot发送
                {
                    ancrange_flag = 2; //设置基站间测距模式开启 1=开始 2=运行中 0=停止
                    dwt_forcetrxoff();
                    state = STA_SEND_POLL;
                    break;
                }
            }
            else
#endif
            {
                if(nowtime >= next_period_time)  //下个周期发送时间到
                {
                    dwt_forcetrxoff();
                    // battery = 0;
//                    if(USE_CW2015 == 1)
//                    {
//                        battery = cw2015_read_battery();
//                    }
                    state = STA_SEND_POLL;
                    break;
                }
            }

            //LED控制
            if((portGetTickCnt()%200 == 0) && (portGetTickCnt() > (led_time + 5)) && ((sos == 1) || (alarm > 0)))
            {
                led_time = portGetTickCnt();
                if(led_flag)
                {
//                    led_on(LED1); //红灯
//                    led_off(LED2); //绿灯
//                    led_off(LED3); //蓝
                }
                else
                {
//                    led_off(LED1); //红灯
//                    led_on(LED2); //绿灯
//                    led_off(LED3); //蓝
                }
                led_flag = !led_flag;
                //motor_on();
            }
        }
            break;
        
        default:
            break;
    }

}



/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tag_rx_ok_cb()
 *
 * @brief Callback to process RX good frame events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void tag_rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    rx_status = RX_OK;
    if (cb_data->datalength <= FRAME_LEN_MAX)
    {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
    }

    UNUSED(cb_data);

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tag_rx_to_cb()
 *
 * @brief Callback to process RX timeout events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void tag_rx_to_cb(const dwt_cb_data_t *cb_data)
{
    rx_status = RX_TIMEOUT;
    UNUSED(cb_data);

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tag_rx_err_cb()
 *
 * @brief Callback to process RX error events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void tag_rx_err_cb(const dwt_cb_data_t *cb_data)
{
    rx_status = RX_ERROR;
    UNUSED(cb_data);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tag_tx_conf_cb()
 *
 * @brief Callback to process TX confirmation events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void tag_tx_conf_cb(const dwt_cb_data_t *cb_data)
{
    tx_status = TX_OK;
    UNUSED(cb_data);
}

