#include "instance.h"

uint8_t switch8 = 0;                                    //拨码开关键值
uint8_t instance_mode = ANCHOR;                         //设备运行角色
uint8_t dev_id;                                         //设备ID
uint8_t group_id;                                       //组ID
uint8_t anc_id;                                         //如当前角色是基站，则表示当前基站ID
uint8_t tag_id;                                         //如当前角色是标签，则表示当前标签ID
uint8_t state = STA_IDLE;                               //状态机状态控制
int32_t distance_report[8];                             //基站测距值数组，用于打包输出
int32_t previous_sort_distance[MAX_TAG_LIST_SIZE] = {-1}; // 
// int32_t sort_distance[MAX_TAG_LIST_SIZE] = {-1};        //用于标签距离排序
// tagDistance_t sort_distance1[MAX_TAG_LIST_SIZE];
int32_t group_report[8];                                //基站组ID数组，用于打包输出
uint32_t range_time;                                    //测距产生时间，串口打包发送
uint8_t frame_seq_nb = 0;                               //每帧数据增加1
uint8_t range_nb = 0;                                   //每次range增加1(poll resp1~4 fianl维护一套range_nb)
uint8_t recv_tag_id;                                    //如当前角色是基站，则表示当前基站收到标签发送过来数据的标签ID
uint8_t recv_anc_id;                                    //如当前角色是标签，则表示当前标签收到基站发送过来数据的基站ID
uint8_t range_status = RANGE_NULL;                      //测距成功标志位，用于打包输出
float rx_power;                                         //接收RSSI
uint16_t inst_slot_number;                              //系统内最大标签容量
uint8_t inst_dataRate;                                  //通信速率，用于根据当前110K还是6.8M确定数据超时等通信过程相关参数
uint8_t inst_ch;                                        //信道号Channel number
uint8_t inst_prf;                                       //PRF
uint8_t inst_one_slot_time;                             //一个slot的时间，根据通信速率不同而不同，单位ms
uint32_t inst_final_rx_timeout;                         //基站final接收超时时间，根据通信速率不同而不同，单位us
uint32_t inst_resp_rx_timeout;                          //标签发送poll后接收resp超时时间，根据通信速率不同而不同，单位us
// uint32_t inst_init_rx_timeout;                       //标签发送blink后接收init超时时间，根据通信速率不同而不同，单位us
uint64_t inst_poll2final_time;                          //单TWR周期poll起始到final结束的总时间
uint32_t inst_data_interval;                            //相邻两条数据的间隔，如poll和第一个resp的间隔，resp1和resp2的间隔，根据通信速率不同而不同，单位us
uint16 ant_dly = ANT_DLY;                               //天线延时
uint32 tx_power;                                        //发射增益代码
uint8_t UART_RX_BUF[200];                               //串口接收BUF
uint32_t uart_rx_len;                                   //串口接收数据长度
// vec3d anchorArray[8];                                //基站坐标，用于标签解算自身位置
double distance_now_m;                                  //基站计算本周期测距结果，单位米
int32 distance_offset_cm;                               //距离校准，单位cm
uint8_t sos = 0;
uint8_t alarm = 0;
int user_data[10];

/* 計算接收功率 */
static double   RX_level = 0;       // 接收功率
static int      RX_level_C = 0;	    //0x12 CIR_PWR 接收功率参数
static int      RX_level_N = 0;     //0x10 RXPACC  接收功率参数
static float    RX_level_A=0;       //
uint32_t D17F = 0;

static distance_type distance_data;

min_heap sort_heap = {0};
MultiTimer sort_timer;

/* dw1000 rf 配置  */
static dwt_config_t uwb_config[CONFIG_BR_NUM] = {
    {
        .chan = 2, .prf = DWT_PRF_64M, .txPreambLength = DWT_PLEN_1024, .rxPAC = DWT_PAC32, .txCode = 10, .rxCode = 10, .nsSFD = 1, .dataRate = DWT_BR_110K, .phrMode = DWT_PHRMODE_STD, .sfdTO = (1025 + DW_NS_SFD_LEN_110K - 32) 
    }, /* uwb_config1 */
    {
        .chan = 2, .prf = DWT_PRF_64M, .txPreambLength = DWT_PLEN_128, .rxPAC = DWT_PAC8, .txCode = 10, .rxCode = 10, .nsSFD = 1, .dataRate = DWT_BR_6M8, .phrMode = DWT_PHRMODE_STD, .sfdTO = (129 + DW_NS_SFD_LEN_6M8 - 8)
    }, /* uwb_config2 */
    {
        .chan = 5, .prf = DWT_PRF_64M, .txPreambLength = DWT_PLEN_128, .rxPAC = DWT_PAC8, .txCode = 10, .rxCode = 10, .nsSFD = 1, .dataRate = DWT_BR_6M8, .phrMode = DWT_PHRMODE_STD, .sfdTO = (129 + DW_NS_SFD_LEN_6M8 - 8)
    }, /* uwb_config3 */
    {
        .chan = 2, .prf = DWT_PRF_64M, .txPreambLength = DWT_PLEN_256, .rxPAC = DWT_PAC16, .txCode = 9, .rxCode = 9, .nsSFD = 1, .dataRate = DWT_BR_850K, .phrMode = DWT_PHRMODE_STD, .sfdTO = (257 + DW_NS_SFD_LEN_850K - 16)
    }, /* uwb_config4 */
    {
        .chan = 5, .prf = DWT_PRF_64M, .txPreambLength = DWT_PLEN_256, .rxPAC = DWT_PAC16, .txCode = 10, .rxCode = 10, .nsSFD = 1, .dataRate = DWT_BR_850K, .phrMode = DWT_PHRMODE_STD, .sfdTO = (257 + DW_NS_SFD_LEN_850K - 16) 
    }, /* uwb_config5 当前使用的配置，channel 5，baudrate 850K */
};

/*******************************************************函数声明********************************************************/
static void distance_init(distance_type* data);
static void sort_timer_callBack(MultiTimer* timer, void* userData);
static void print_config(void);

void uwb_init(void)
{
    static dwt_txconfig_t txconfig_options = {
        .PGdly = 0XC2,            /* PG delay */
        .power = TX_POWER         /* TX power */
    };
    distance_type* distance = get_the_local_structure_of_dis();
    reset_DW1000();               /* Target specific drive of RSTn line into DW1000 low for a period. */
    port_set_dw1000_slowrate();
    if(DWT_DEVICE_ID != dwt_readdevid())    // 若读取ID失败，先执行唤醒
    {
        port_wakeup_IC();                   // 使用SPI-NS管脚唤醒DW1000
        dwt_softreset();                    // 软件复位
    }
    reset_DW1000();                         // 复位

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) // dw1000初始化失败
    {
        while (1);
    }
    port_set_dw1000_fastrate();

    inst_slot_number = MAX_TAG_NUMBER;
    dwt_configure(&uwb_config[4]);
    inst_dataRate = uwb_config[4].dataRate;
    inst_ch       = uwb_config[4].chan;

    /* 配置通信相关时序 */
    if(inst_dataRate == DWT_BR_6M8)
    {
        inst_one_slot_time    = ONE_SLOT_TIME_MS_6P8M;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_6P8M;
        inst_resp_rx_timeout  = RESP_RX_TIMEOUT_6P8M;
        inst_data_interval    = DATA_INTERVAL_TIME_6P8M;
        inst_poll2final_time  = ((FIRST_RESP_SEND_6P8M +  MAX_AHCHOR_NUMBER * inst_data_interval) * UUS_TO_DWT_TIME);
    }
    else if(inst_dataRate == DWT_BR_110K)
    {
        inst_one_slot_time    = ONE_SLOT_TIME_MS_110K;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_110K;
        inst_resp_rx_timeout  = RESP_RX_TIMEOUT_110K;
        inst_data_interval    = DATA_INTERVAL_TIME_110K;
        inst_poll2final_time  = ((FIRST_RESP_SEND_110K +  MAX_AHCHOR_NUMBER * inst_data_interval) * UUS_TO_DWT_TIME);
    }
    else if(inst_dataRate == DWT_BR_850K) 
    {
        inst_one_slot_time    = ONE_SLOT_TIME_MS_850K;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_850K;
        inst_resp_rx_timeout  = RESP_RX_TIMEOUT_850K;
        inst_data_interval    = DATA_INTERVAL_TIME_850K;
        inst_poll2final_time  = ((FIRST_RESP_SEND_850K +  MAX_AHCHOR_NUMBER * inst_data_interval) * UUS_TO_DWT_TIME);
    }

    if(uwb_config[4].prf == DWT_PRF_64M) 
    { 
        RX_level_A = 121.74;
    }
    else if (uwb_config[4].prf == DWT_PRF_16M)
    {
        RX_level_A = 113.77;
    }
    D17F = pow(2, 17);
    
    txconfig_options.power = TX_POWER;

    tx_power = txconfig_options.power;
    dwt_configuretxrf(&txconfig_options);//设置发射功率和pg值

    ant_dly = ANT_DLY;

    dwt_setrxantennadelay(ant_dly);                 //设置天线延时
    dwt_settxantennadelay(ant_dly);

    dwt_setpanid(PAN_ID);                                   //设置PAN ID 组号
    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);  //设置帧过滤模式开启

    dwt_setlnapamode(1, 1);//设置外置PA和LNA控制开启
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);//设置DW3000控制的收发指示灯开启，低功耗时可注释掉
    
    // 配置角色 
    instance_mode = ANCHOR; //当前角色控制为标签

    // 配置设备ID
    dev_id = read_SwitchValue();

    //设置中断标志
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO), 1);

    if(instance_mode == ANCHOR)
    {
        //设置基站的中断回调函数
        dwt_setcallbacks(&anc_tx_conf_cb, &anc_rx_ok_cb, &anc_rx_to_cb, &anc_rx_err_cb);
    }
    else 
    {   //设置标签的中断回调函数
        // dwt_setcallbacks(&tag_tx_conf_cb, &tag_rx_ok_cb, &tag_rx_to_cb, &tag_rx_err_cb);
    }

    //按角色初始化设备短地址，设置状态机初始状态
    if(instance_mode == ANCHOR)
    {
        /* 设备短地址为2个字节，为了区分A0和T0短地址，基站的最高位为1
         * 如A1短地址=0x8001，T1短地址=0x0001
         */
        uint16_t anc_short_add = 0x8000 | dev_id;
        dwt_setaddress16(anc_short_add);
        anc_id = dev_id;

        if(anc_id == 0)             // A0的group ID最高bit设置为1，为时序校准基站
        {
            group_id = group_id | 0x80;
        }
        dwt_forcetrxoff();
        state = STA_INIT_POLL_SYNC;
    }
    else
    {
        dwt_setaddress16(dev_id);
        tag_id = dev_id;
        dwt_forcetrxoff();
        state = STA_IDLE;
    }
    // 因为当前板子引脚分配，dw1000中断脚为PC13，复位引脚为PC14，共用一个中断处理，在setup_DW1000RSTnIRQ中会关闭该中断，因此在这里重新打开。
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    distance_init(distance);        // 初始化距离结构体

    multiTimerStart(&sort_timer, 400, sort_timer_callBack, NULL);   // 维护一个定时任务，用于堆中更新数据。
}

void dw_main(void)
{
    print_config();                                     // 打印系统参数信息
    while(1)                                            // 测距功能实现，按角色执行基站状态机或标签状态机
    {
        distance_type* distance = get_the_local_structure_of_dis();
        anchor_app();
        
        if(range_status == RANGE_TWR_OK)            // TWR测距有效，进行数据滤波和打包输出、屏显或者其他处理
        {        
            range_status = RANGE_NULL;              // 清空标志位
            led_toggle(uwb_ok_led);                 // PCB闪烁LED，说明标签同基站本次测距成功
            // HAL_IWDG_Refresh(&hiwdg);
        }
        else if(range_status == RANGE_ERROR) 
        {
            range_status = RANGE_NULL;              // 清空标志位
            for(uint8_t i = 0; i < 8; i++)          // 清空distance_report数组，设置无效值
            {
                distance_report[i] = -1; 
                group_report[i] = -1; 
            }
            printf_use_dma("RANGE_ERROR, ID = %d, rb = %d, range_time = %d\r\n", dev_id, range_nb, range_time);
        }
    }
}

double calculate_RSSI(dwt_rxdiag_t* rx_diag)
{
    dwt_readdiagnostics(rx_diag);
    RX_level_C = (int)rx_diag->maxGrowthCIR;
    RX_level_N = (int)rx_diag->rxPreamCount;

    /* 計算接收功率 */
    RX_level = RX_level_C * D17F;
    RX_level = RX_level / (RX_level_N * RX_level_N);
    RX_level = 10 * log10(RX_level);
    RX_level = RX_level - RX_level_A;

    return RX_level;
}

void tag_distance_handler(void)
{
    distance_type* distance =  get_the_local_structure_of_dis();
    int a;
    int rx = 0;
    int cur_size = distance->dis_min_heap.heap_current_size;
    uint8_t array[cur_size];
    int sign;

    rx = get_newrange();    
    a = heap_find_index(&distance->dis_min_heap, recv_tag_id);

    if (rx == 0x01)                 // 基站标签TWR成功，针对当前成功的标签进行处理
    {   
        if (a == -1)                // 当前有效的标签距离之前没有入堆，插入
        {
            heap_insert(&distance->dis_min_heap, distance->sort_distance1[recv_tag_id].tag_distance, recv_tag_id);
        }
        else                        // 当前有效的标签距离之前已经入堆，更新
        {
            heap_update(&distance->dis_min_heap, recv_tag_id, distance->sort_distance1[recv_tag_id].tag_distance);          // 有效距离，将对应的索引存储的数据，更新到堆里面
        }
    }
    // 更新完有效距离之后，遍历当前堆，移除无效的节点
    // 获取当前堆的节点对应的索引，也就是获取当前堆存储了哪些标签的距离
    // 针对这些索引进行final包的标志位判断（当前TWR成功的标签无需判断），无效就从堆中删除。
    for (int i = 0; i < cur_size; i++)  // 获取当前堆中所有节点对应的索引
    {
        array[i] = heap_getNodeIndex(&distance->dis_min_heap, i);
    }
    
    for (int i = 0; i < cur_size; i++ )
    {
        if(array[i] != recv_tag_id)     // 跳过当前TWR成功的标签
        {
            sign = get_sign(array[i]);
            if (sign == 0)              // 该节点数据无效
            {
                heap_remove(&distance->dis_min_heap, array[i]);     // 寻找无效距离，从堆中删除
            }
        }
    }
}

distance_type* get_the_local_structure_of_dis(void)
{
    return &distance_data;
}

int get_newrange(void)
{
    distance_type* distance = get_the_local_structure_of_dis();
    int x = distance->newRange;
    distance->newRange = 0x00;
    return x;
}

// 获取标签的有效位
uint8_t get_sign(uint8_t tad_idx)
{
    distance_type* distance = get_the_local_structure_of_dis();

    uint8_t x = distance->sort_distance1[tad_idx].final_receiveSign;
    distance->sort_distance1[tad_idx].final_receiveSign = 0;
    return x;
}   

static void distance_init(distance_type* data)
{
    data->dis_idx = 0;
    data->disMsg->dis_class = 0; 
    data->min_dis = 2000000;
    data->newRange = 0x00;
    for (int i = 0; i < MAX_TAG_LIST_SIZE; i++)
    {
        data->sort_distance1[i].final_receiveSign = 0x00;
        data->sort_distance1[i].poll_receiveSign = 0x00;
        data->sort_distance1[i].tag_distance = 2000000;
    }
    heap_init(&data->dis_min_heap);
}

static void sort_timer_callBack(MultiTimer* timer, void* userData)
{
    tag_distance_handler();
    multiTimerStart(&sort_timer, 400, sort_timer_callBack, NULL); 
}

static void print_config(void)
{
    int len;
    uint8_t UART_TX_DATA[512];
    len = sprintf((char*)&UART_TX_DATA[0], "\r\n***************************************************\r\n");
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* firmware = %s\r\n* role = %s\r\n* addr = %x\r\n", SOFTWARE_VER, (instance_mode == TAG)?"TAG":"AHCHOR", dev_id);
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* max_anc_num = %d\r\n* max_tag_num = %d\r\n* sync = 0\r\n", MAX_AHCHOR_NUMBER, inst_slot_number);
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* baud_rate = %s\r\n* channel = CH%d\r\n", (inst_dataRate == DWT_BR_110K)? "110K" : "6.8M", inst_ch);
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* data_rate = %dHz\r\n* update_time = %dms\r\n* kalmanfilter = %d\r\n", 1000 / (inst_slot_number * inst_one_slot_time), inst_slot_number * inst_one_slot_time, (switch8 & SWS1_KAM_MODE)? 1:0);
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* ant_dly  = %d\r\n* tx_power = %08lx\r\n", ant_dly, tx_power);
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "***************************************************\r\n");
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);
}
