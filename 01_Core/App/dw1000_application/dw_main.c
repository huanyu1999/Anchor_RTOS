
#include "instance.h"
#include "elog.h"

uint8_t group_id;           // 组ID
uint8_t anc_id;             // 如当前角色是基站，则表示当前基站ID
uint8_t tag_id;             // 如当前角色是标签，则表示当前标签ID
int32_t distance_report[8]; // 基站测距值数组，用于打包输出
int32_t group_report[8];    // 基站组ID数组，用于打包输出
uint32_t range_time;        // 测距产生时间，串口打包发送
uint8_t frame_seq_nb = 0;   // 每帧数据增加1
uint8_t range_nb = 0;       // 每次range增加1(poll resp1~4 fianl维护一套range_nb)
uint8_t recv_tag_id;        // 如当前角色是基站，则表示当前基站收到标签发送过来数据的标签ID
uint8_t recv_anc_id;        // 如当前角色是标签，则表示当前标签收到基站发送过来数据的基站ID
uint8_t range_status = RANGE_NULL; // 测距成功标志位，用于打包输出
float rx_power;                    // 接收RSSI
uint16_t inst_slot_number;         // 系统内最大标签容量
uint8_t inst_dataRate; // 通信速率，用于根据当前110K还是6.8M确定数据超时等通信过程相关参数
uint8_t inst_ch;       // 信道号Channel number
uint8_t inst_prf;      // PRF
uint8_t inst_one_slot_time;     // 一个slot的时间，根据通信速率不同而不同，单位ms
uint32_t inst_final_rx_timeout; // 基站final接收超时时间，根据通信速率不同而不同，单位us
uint32_t inst_resp_rx_timeout; // 标签发送poll后接收resp超时时间，根据通信速率不同而不同，单位us
// uint32_t inst_init_rx_timeout; //标签发送blink后接收init超时时间，根据通信速率不同而不同，单位us
uint64_t inst_poll2final_time; // 单TWR周期poll起始到final结束的总时间
uint32_t
    inst_data_interval; // 相邻两条数据的间隔，如poll和第一个resp的间隔，resp1和resp2的间隔，根据通信速率不同而不同，单位us
uint16 ant_dly = ANT_DLY; // 天线延时
uint32 tx_power;          // 发射增益代码
double distance_now_m;    // 基站计算本周期测距结果，单位米
int32 distance_offset_cm; // 距离校准，单位cm
int user_data[10];

/* 計算接收功率 */
static double RX_level = 0; // 接收功率
static int RX_level_C = 0;  // 0x12 CIR_PWR 接收功率参数
static int RX_level_N = 0;  // 0x10 RXPACC  接收功率参数
static float RX_level_A = 0;
uint32_t D17F = 0;

/* dw1000 rf 配置  */
static dwt_config_t uwb_config[CONFIG_BR_NUM] = {
    {.chan = 2,
     .prf = DWT_PRF_64M,
     .txPreambLength = DWT_PLEN_1024,
     .rxPAC = DWT_PAC32,
     .txCode = 10,
     .rxCode = 10,
     .nsSFD = 1,
     .dataRate = DWT_BR_110K,
     .phrMode = DWT_PHRMODE_STD,
     .sfdTO = (1025 + DW_NS_SFD_LEN_110K - 32)}, /* uwb_config1 */
    {.chan = 2,
     .prf = DWT_PRF_64M,
     .txPreambLength = DWT_PLEN_128,
     .rxPAC = DWT_PAC8,
     .txCode = 10,
     .rxCode = 10,
     .nsSFD = 1,
     .dataRate = DWT_BR_6M8,
     .phrMode = DWT_PHRMODE_STD,
     .sfdTO = (129 + DW_NS_SFD_LEN_6M8 - 8)}, /* uwb_config2 */
    {.chan = 5,
     .prf = DWT_PRF_64M,
     .txPreambLength = DWT_PLEN_128,
     .rxPAC = DWT_PAC8,
     .txCode = 10,
     .rxCode = 10,
     .nsSFD = 1,
     .dataRate = DWT_BR_6M8,
     .phrMode = DWT_PHRMODE_STD,
     .sfdTO = (129 + DW_NS_SFD_LEN_6M8 - 8)}, /* uwb_config3 */
    {.chan = 2,
     .prf = DWT_PRF_64M,
     .txPreambLength = DWT_PLEN_256,
     .rxPAC = DWT_PAC16,
     .txCode = 9,
     .rxCode = 9,
     .nsSFD = 1,
     .dataRate = DWT_BR_850K,
     .phrMode = DWT_PHRMODE_STD,
     .sfdTO = (257 + DW_NS_SFD_LEN_850K - 16)}, /* uwb_config4 */
    {.chan = 5,
     .prf = DWT_PRF_64M,
     .txPreambLength = DWT_PLEN_256,
     .rxPAC = DWT_PAC16,
     .txCode = 10,
     .rxCode = 10,
     .nsSFD = 1,
     .dataRate = DWT_BR_850K,
     .phrMode = DWT_PHRMODE_STD,
     .sfdTO = (257 + DW_NS_SFD_LEN_850K - 16)}, /* uwb_config5 当前使用的配置，channel 5，baudrate 850K */
};

// Implemented UWB algoritm. The dummy one is at the end of this file.
static uwbAlgorithm_t dummy_Algorithm;
static uwbAlgorithm_t *current_Algorithm = &dummy_Algorithm;
extern uwbAlgorithm_t uwbTwr_AnchorAlgorithm;

struct
{
    uwbAlgorithm_t *algorithm;
    char *name;
} availableAlgorithms[] = {{.algorithm = &uwbTwr_AnchorAlgorithm, .name = " TWR ANCHOR "}, {NULL, NULL}};

static dwDevice_t dw1000_dev;      // 定义dw1000设备，twr使用
static dwDistance_t distance_data; // 定义距离管理

static osMutexId_t heap_accessMutex;
const osMutexAttr_t heap_mutex_attr = {
    .name = "heapMutex", .attr_bits = osMutexRecursive | osMutexPrioInherit, .cb_mem = NULL, .cb_size = 0};

osSemaphoreId_t tagDistClearSem;
StaticSemaphore_t tagDistClearSemCB;
const osSemaphoreAttr_t tagDistClearSem_attr = {
    .name = "tagDistClear", .cb_mem = &tagDistClearSemCB, .cb_size = sizeof(tagDistClearSemCB)};

extern osMessageQueueId_t minDisQueue;
extern osSemaphoreId_t binSem;
static uint32_t timeout;

/*******************************************************静态函数声明********************************************************/
static void distance_init(dwDistance_t *data);
static void dw1000Device_init(dwDevice_t *dev);
static void print_dw1000Config(void);
static void txcallback(const dwt_cb_data_t *cb_data);
static void rxcallback(const dwt_cb_data_t *cb_data);
static void rxTimeoutCallback(const dwt_cb_data_t *cb_data);
static void rxfailedcallback(const dwt_cb_data_t *cb_data);

void uwb_init(void)
{
    static dwt_txconfig_t txconfig_options = {
        .PGdly = 0XC2,    /* PG delay */
        .power = TX_POWER /* TX power */
    };

    dwDistance_t *distance = get_the_local_structure_of_dis();
    dwDevice_t *dev = get_the_local_structure_of_dev();

    distance_init(distance);
    dw1000Device_init(dev);

    /* Target specific drive of RSTn line into DW1000 low for a period. */
    board_dw1000Rst();

    port_set_dw1000_slowrate();
    if (DWT_DEVICE_ID != dwt_readdevid()) // 若读取ID失败，先执行唤醒
    {
        board_dw1000SlowWakeup();
        dwt_softreset();                 // 软件复位
    }
    board_dw1000Rst();

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) // dw1000初始化失败
    {
        while (1)
        ;
    }
    port_set_dw1000_fastrate();

    inst_slot_number = MAX_TAG_NUMBER;
    dwt_configure(&uwb_config[4]);
    inst_dataRate = uwb_config[4].dataRate;
    inst_ch = uwb_config[4].chan;

    /* 配置通信相关时序 */
    if (inst_dataRate == DWT_BR_6M8)
    {
        inst_one_slot_time = ONE_SLOT_TIME_MS_6P8M;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_6P8M;
        inst_resp_rx_timeout = RESP_RX_TIMEOUT_6P8M;
        inst_data_interval = DATA_INTERVAL_TIME_6P8M;
        inst_poll2final_time = ((FIRST_RESP_SEND_6P8M + MAX_AHCHOR_NUMBER * inst_data_interval) * UUS_TO_DWT_TIME);
    }
    else if (inst_dataRate == DWT_BR_110K)
    {
        inst_one_slot_time = ONE_SLOT_TIME_MS_110K;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_110K;
        inst_resp_rx_timeout = RESP_RX_TIMEOUT_110K;
        inst_data_interval = DATA_INTERVAL_TIME_110K;
        inst_poll2final_time = ((FIRST_RESP_SEND_110K + MAX_AHCHOR_NUMBER * inst_data_interval) * UUS_TO_DWT_TIME);
    }
    else if (inst_dataRate == DWT_BR_850K)
    {
        inst_one_slot_time = ONE_SLOT_TIME_MS_850K;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_850K;
        inst_resp_rx_timeout = RESP_RX_TIMEOUT_850K;
        inst_data_interval = DATA_INTERVAL_TIME_850K;
        inst_poll2final_time = ((FIRST_RESP_SEND_850K + MAX_AHCHOR_NUMBER * inst_data_interval) * UUS_TO_DWT_TIME);
    }

    if (uwb_config[4].prf == DWT_PRF_64M)
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
    dwt_configuretxrf(&txconfig_options); // 设置发射功率和pg值

    ant_dly = ANT_DLY;

    dwt_setrxantennadelay(ant_dly); // 设置天线延时
    dwt_settxantennadelay(ant_dly);

    dwt_setpanid(PAN_ID);                                  // 设置PAN ID 组号
    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); // 设置帧过滤模式开启

    dwt_setlnapamode(1, 1);                             // 设置外置PA和LNA控制开启
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK); // 低功耗时可注释掉

    // 设置中断标志
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG |DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | 
                        DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO, 1);

    if (dev->device_mode == ANCHOR)
    {
        dwt_setcallbacks(&txcallback, &rxcallback, &rxTimeoutCallback,
                         &rxfailedcallback); // 设置基站的中断回调函数, 事件驱动版本
    }

    // 按角色初始化设备短地址，设置状态机初始状态
    if (dev->device_mode == ANCHOR)
    {
        /* 设备短地址为2个字节，为了区分A0和T0短地址，基站的最高位为1
         * 如A1短地址=0x8001，T1短地址=0x0001
         */
        uint16_t anc_short_add = 0x8000 | dev->device_id;
        dwt_setaddress16(anc_short_add);
        anc_id = dev->device_id;

        if (anc_id == 0) // A0的group ID最高bit设置为1，为时序校准基站
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

    uint32_t time = inst_one_slot_time * inst_slot_number;
    MX_TIM2_Init(time);                                 // 设置定时器周期为1个TDMA周期，

    current_Algorithm = availableAlgorithms[0].algorithm;
    heap_accessMutex = osMutexNew(&heap_mutex_attr);
    tagDistClearSem = osSemaphoreNew(1, 0, &tagDistClearSem_attr);
}

/**
 * @brief uwb 核心任务，使用中断触发
 * @param  void *arg RTOS要求参数为空指针类型
 * @retval none
 */
void task0_uwb(void *arg)
{
    current_Algorithm->init(&dw1000_dev);
    print_dw1000Config();
    for (;;)
    {
        osStatus_t status = osSemaphoreAcquire(binSem, osWaitForever); // 采用中断触发的方式执行，获取信号量
        if (status == osOK)
        {
            process_deca_irq();
        }
    }
}

void task_tagDistInsertAndUpdate(void *arg)
{
    uint32_t flag_receive = 0;
    for (;;)
    {
        flag_receive = osThreadFlagsWait(0x00000001, osFlagsWaitAll, osWaitForever);
        if (flag_receive & 0x00000001)
        {
            tag_distInsertAndUpdate();
        }
    }
}

void task_tagDistClearInvalid(void *arg)
{
    for (;;)
    {
        osSemaphoreAcquire(tagDistClearSem, osWaitForever);

        tag_distClearInvalid(); // 清除无效距离
    }
}

double calculate_RSSI(dwt_rxdiag_t *rx_diag)
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

/******************************************************Distance
 * Manage************************************************************/
void tag_distInsertAndUpdate(void)
{
    dwDistance_t *distance = get_the_local_structure_of_dis();
    int heap_pos;

    osMutexAcquire(heap_accessMutex, osWaitForever);

    heap_pos = heap_find_index(&distance->dis_min_heap, recv_tag_id);
    if (heap_pos == -1) // 当前有效的标签距离之前没有入堆，插入
    {
        heap_insert(&distance->dis_min_heap, recv_tag_id, distance->sort_distance[recv_tag_id].tag_distance);
    }
    else // 当前有效的标签距离之前已经入堆，更新
    {
        heap_update(
            &distance->dis_min_heap, recv_tag_id,
            distance->sort_distance[recv_tag_id].tag_distance); // 有效距离，将对应的索引存储的数据，更新到堆里面
    }
    osMutexRelease(heap_accessMutex);
}

void tag_distClearInvalid(void)
{
    int current_heap_size;
    heap_node min_dis;
    outDistance_t minSelfDis;
    dwDistance_t *distance = get_the_local_structure_of_dis();
    osMutexAcquire(heap_accessMutex, osWaitForever);

    current_heap_size = distance->dis_min_heap.heap_current_size; // 获取当前的堆大小
    for (uint8_t i = 0; i < 40; i++)
    {
        uint8_t tag_id_in_heap = heap_getNodeIndex(&distance->dis_min_heap, i);

        uint8_t sign = get_tagFinalRecvFlag(tag_id_in_heap);
        if (sign != 0x01) // 标签无效
        {
            heap_remove(&distance->dis_min_heap, tag_id_in_heap); // 假设 heap_remove 需要 distance 指针
        }
    }

    current_heap_size = distance->dis_min_heap.heap_current_size;
    log_d("current_heap_size : %d..", current_heap_size);

    heap_peek_min(&distance->dis_min_heap, &min_dis); // 堆更新完毕后，采集一次堆顶，也就是最小值
    minSelfDis.dis_class = ANCHOR_SELF_DIS;
    minSelfDis.dis_index = min_dis.index;
    minSelfDis.dis_value = min_dis.value;

    osMutexRelease(heap_accessMutex);
    // log_d("Current min distance ----------------------------------: ID %d-dis %.2f", min_dis.index, (float)(min_dis.value) / 1000);

    extern osMessageQueueId_t minDisQueue;
    osMessageQueuePut(minDisQueue, &minSelfDis, 0, 0);
    // 在这个位置发送消息队列
    for (int i = 0; i < MAX_TAG_LIST_SIZE; i++)
    {
        distance->sort_distance[distance->dis_min_heap.nodes[i].index].final_receiveSign = 0;
    }
}

dwDistance_t *get_the_local_structure_of_dis(void)
{
    return &distance_data;
}

dwDevice_t *get_the_local_structure_of_dev(void)
{
    return &dw1000_dev;
}

// 获取标签的有效位
uint8_t get_tagFinalRecvFlag(uint8_t tad_idx)
{
    dwDistance_t *distance = get_the_local_structure_of_dis();
    uint8_t x = distance->sort_distance[tad_idx].final_receiveSign;
    return x;
}

static void distance_init(dwDistance_t *data)
{
    data->dis_idx = 0;
    data->disMsg->dis_class = 0;
    data->min_dis = 2000000;
    for (int i = 0; i < MAX_TAG_LIST_SIZE; i++)
    {
        data->sort_distance[i].final_receiveSign = 0x00;
        data->sort_distance[i].tag_distance = 2000000;
    }
    heap_init(&data->dis_min_heap, 40, 40);
}

/******************************************************Dw1000
 * Device************************************************************/
static void dw1000Device_init(dwDevice_t *dev)
{
    dev->device_mode = ANCHOR;
    dev->twr_mode = LISTENER;
    dev->device_id = dev_getDipVal();
    dev->remainingRespToRx = -1;
    dev->rxResp = 0;
    dev->rxEnIndex = 0;
    board_dw1000Init();
}

/******************************************************Print
 * Config************************************************************/
static void print_dw1000Config(void)
{
    dwDevice_t *dev = get_the_local_structure_of_dev();

    log_i("***********************************************************************");
    log_i("* firmware = %s\r\n* role = %s\r\n* addr = %x", SOFTWARE_VER, (dev->device_mode == TAG) ? "TAG" : "AHCHOR",
          dev->device_id);
    log_i("* max_anc_num = %d\r\n* max_tag_num = %d\r\n* sync = 0", MAX_AHCHOR_NUMBER, inst_slot_number);
    log_i("* baud_rate = %s\r\n* channel = CH%d", (inst_dataRate == DWT_BR_110K) ? "110K" : "6.8M", inst_ch);
    log_i("* data_rate = %dHz\r\n* update_time = %dms", 1000 / (inst_slot_number * inst_one_slot_time),
          inst_slot_number * inst_one_slot_time);
    log_i("* ant_dly  = %d\r\n* tx_power = %08lx", ant_dly, tx_power);
    log_i("***********************************************************************");
}

/******************************************************interrupt use callback
 * function************************************************************/
static void txcallback(const dwt_cb_data_t *cb_data)
{
    timeout = current_Algorithm->onEvent(&dw1000_dev, eventPacketSent);
}

static void rxcallback(const dwt_cb_data_t *cb_data)
{
    timeout = current_Algorithm->onEvent(&dw1000_dev, eventPacketReceived);
}

static void rxTimeoutCallback(const dwt_cb_data_t *cb_data)
{
    timeout = current_Algorithm->onEvent(&dw1000_dev, eventReceiveTimeout);
}

static void rxfailedcallback(const dwt_cb_data_t *cb_data)
{
    timeout = current_Algorithm->onEvent(&dw1000_dev, eventReceiveFailed);
}
