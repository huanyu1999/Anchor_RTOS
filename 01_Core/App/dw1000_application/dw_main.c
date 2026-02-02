#include "dw_sort.h"
#include "dw_instance.h"
#include "dev_can.h"
#include "uthash.h"
#include "cmsis_os.h"
#include "elog.h"

#define ANCHOR_DEV_ID 0x00

uint8_t group_id;                   // 组ID，后续会有用处，不同车务段的人员共同施工，各自跟各自的基站通信？？？？
uint8_t anc_id;                     // 如当前角色是基站，则表示当前基站ID
uint8_t tag_id;                     // 如当前角色是标签，则表示当前标签ID
int32_t distance_report[8];         // 基站测距值数组，用于打包输出
int32_t group_report[8];            // 基站组ID数组，用于打包输出
uint32_t range_time;                // 测距产生时间，串口打包发送
uint8_t frame_seq_nb = 0;           // 每帧数据增加1
uint8_t range_nb = 0;               // 每次range增加1(poll resp1~4 fianl维护一套range_nb)
uint8_t recv_tag_id;                // 如当前角色是基站，则表示当前基站收到标签发送过来数据的标签ID
uint8_t recv_anc_id;                // 如当前角色是标签，则表示当前标签收到基站发送过来数据的基站ID
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
uint16 ant_dly = ANT_DLY;           // 天线延时
uint32 tx_power;                    // 发射增益代码
int32 distance_offset_cm;           // 距离校准，单位cm
int user_data[10];

static dwt_txconfig_t txconfig_options = {
    .PGdly = 0XC2,      /* PG delay */
    .power = TX_POWER   /* TX power */
};

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
    {   /* uwb_config2，channel5 脉冲频率64M 前导码长度1024 数据率 850K ，该配置人体遮挡情况下，表现较好，偶有距离大跳情况，考虑软件优化 */
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

// Implemented UWB algoritm. The dummy one is at the end of this file.
static uwbAlgorithm_t dummy_Algorithm;
static uwbAlgorithm_t *current_Algorithm = &dummy_Algorithm;
extern uwbAlgorithm_t uwbTwr_AnchorAlgorithm;
static dwDevice_t   dw1000_dev;      // 定义dw1000设备，twr使用
static dwDistance_t distance_data; // 定义距离管理

const struct {
    uwbAlgorithm_t *algorithm;
    char *name;
} availableAlgorithms[] = { 
    {.algorithm = &uwbTwr_AnchorAlgorithm, .name = " TWR ANCHOR "}, 
    {NULL, NULL}
};

static osMutexId_t  dis_access_mutex;
const osMutexAttr_t dis_access_mutex_attr = {
    .name = "dis_access_mutex", .attr_bits = osMutexRecursive | osMutexPrioInherit, .cb_mem = NULL, .cb_size = 0
};

osSemaphoreId_t uwbIntSem;                                   // 用于dw1000中断同步
StaticSemaphore_t uwbIntSemCB;
const osSemaphoreAttr_t uwbIntSem_attr = {
    .name = "uwbIntSem", .cb_mem = &uwbIntSemCB, .cb_size = sizeof(StaticSemaphore_t)
};

osSemaphoreId_t dw1000WriteSem;
StaticQueue_t dw1000WriteSemCB;                        
const osSemaphoreAttr_t dw1000WriteSem_attr = {
    .name = "dw1000WriteSem", .cb_mem  = &dw1000WriteSemCB, .cb_size = sizeof(dw1000WriteSemCB)
};

osSemaphoreId_t dw1000ReadSem;
StaticQueue_t dw1000ReadSemCB;                    
const osSemaphoreAttr_t dw1000ReadSem_attr = {
    .name = "dw1000ReadSem", .cb_mem  = &dw1000ReadSemCB, .cb_size = sizeof(dw1000ReadSemCB)
};

osMessageQueueId_t queue_uwbEvent;

osSemaphoreId_t   tagDistClear_sem;        // 该信号量用于周期性采集最小值处理与定时器中断同步
StaticSemaphore_t tagDistClear_sem_cb;
const osSemaphoreAttr_t tagDistClear_sem_attr = {
    .name = "tagDistClear", .cb_mem = &tagDistClear_sem_cb, .cb_size = sizeof(tagDistClear_sem_cb)
};

osMessageQueueId_t queue_processDis;       // 该队列用于传递距离给排序处理用
tag_hashNode_t queue_processDis_buf[16];   // 初始化队列的存储空间
StaticQueue_t queue_processDis_cb;         // 初始化队列控制块存储空间
const osMessageQueueAttr_t queue_processDis_attr = {
    .name = "queue_processDis",
    .cb_mem = &queue_processDis_cb, .cb_size = sizeof(queue_processDis_cb),
    .mq_mem = &queue_processDis_buf, .mq_size = sizeof(queue_processDis_buf)
};

TIM_HandleTypeDef timerForInvaildDistanceClearHandle;
static int32_t prev_rangeTagDis[MAX_TAG_LIST_SIZE];

/*******************************************************静态函数声明********************************************************/
static int get_rxPeakIndex(uint16_t start_index, uint16_t num_samples);
static void distance_init(dwDistance_t *data);
static void dw1000Device_init(dwDevice_t *dev);
static void txcallback(const dwt_cb_data_t *cb_data);
static void rxcallback(const dwt_cb_data_t *cb_data);
static void rxTimeoutCallback(const dwt_cb_data_t *cb_data);
static void rxfailedcallback(const dwt_cb_data_t *cb_data);

void dev_uwbInit(void)
{
    dwt_config_t *current_rfConfig = &uwb_config_channel5[2];
    dwDistance_t *distance = get_the_local_structure_of_dis();
    dwDevice_t *dev = get_the_local_structure_of_dev();

    distance_init(distance);
    dw1000Device_init(dev);
    uwbIntSem       = osSemaphoreNew(1, 0, &uwbIntSem_attr);
    dw1000WriteSem  = osSemaphoreNew(1, 0, &dw1000WriteSem_attr);
    dw1000ReadSem   = osSemaphoreNew(1, 0, &dw1000ReadSem_attr);
    queue_uwbEvent  = osMessageQueueNew(8, sizeof(uwbEvent_t), NULL);
    board_dw1000Rst();      // Target specific drive of RSTn line into DW1000 low for a period.
    port_set_dw1000_slowrate();
    if (0xDECA0130 != dwt_readdevid())
    {   // 若读取ID失败，先执行唤醒
        board_dw1000SlowWakeup();        // dw1000缓慢唤醒
        dwt_softreset();                 // 软件复位
    }
    board_dw1000Rst();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {   // dw1000初始化失败
        // while (1);
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
    dwt_configuretxrf(&txconfig_options); // 设置发射功率和pg值

    ant_dly = ANT_DLY;
    dwt_setrxantennadelay(ant_dly);       // 设置天线延时
    dwt_settxantennadelay(ant_dly);

    dwt_setpanid(PAN_ID);                                      // 设置PAN ID组号
    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);     // 设置帧过滤模式开启

    dwt_setlnapamode(1, 1);                                    // 设置外置PA和LNA控制开启
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);        // 低功耗时可注释掉

    // 设置中断标志
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG |DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_RFTO | DWT_INT_RXPTO, 1);

    if (dev->device_mode == ANCHOR)
    {
        dwt_setcallbacks(&txcallback, &rxcallback, &rxTimeoutCallback, &rxfailedcallback); // 设置基站的中断回调函数, 事件驱动版本
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

        if (anc_id == 0)
        {// A0的group ID最高bit设置为1，为时序校准基站
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

    drv_setTimerForInt(&timerForInvaildDistanceClearHandle, TIM2, 10, 5);  // 硬件定时器，用于清除无效距离

    current_Algorithm = availableAlgorithms[0].algorithm;
    dis_access_mutex  = osMutexNew(&dis_access_mutex_attr);
    tagDistClear_sem  = osSemaphoreNew(1, 0, &tagDistClear_sem_attr);
    queue_processDis  = osMessageQueueNew(16, sizeof(tag_hashNode_t), &queue_processDis_attr);
    osMutexRelease(dis_access_mutex);
    current_Algorithm->init(&dw1000_dev);                                   // 初始化TWR流程，立即打开接收
    // logOut_dw1000Config();
}

void logOut_dw1000Config(void)
{
    dwDevice_t *dev = get_the_local_structure_of_dev();

    log_i("Firmware Ver = %s * Role = %s * addr = %x", SOFTWARE_VER, (dev->device_mode == TAG) ? "TAG" : "AHCHOR",dev->device_id);
    log_i("Max_anc_num = %d * max_tag_num = %d * sync = 0", MAX_AHCHOR_NUMBER, inst_slot_number);
    // log_i("* baud_rate = %s\r\n* channel = CH%d", (inst_dataRate == DWT_BR_110K) ? "110K" : "850K", inst_ch);
    // log_i("* data_rate = %dHz\r\n* update_time = %dms", 1000 / (inst_slot_number * inst_one_slot_time),
    //       inst_slot_number * inst_one_slot_time);
    // log_i("* ant_dly  = %d\r\n* tx_power = %08lx", ant_dly, tx_power);
}

/**
 * @brief uwb中断处理任务
 * @param  void *arg RTOS要求参数为空指针类型
 * @retval none
 */
void task_uwb(void *arg)
{
    UNUSED(arg);
    uwbEvent_t evt;
    dev_uwbInit();
    board_dw1000IRQInit();
    for (;;)
    {
        osStatus_t status = osSemaphoreAcquire(uwbIntSem, osWaitForever); // 采用中断触发的方式执行，获取信号量
        // uint32_t flag = osThreadFlagsWait(0x00000001, osFlagsWaitAll, osWaitForever);
        if (status == osOK)
        {
        // if (flag == 0x00000001)
            process_deca_irq();
        }
        osMessageQueueGet(queue_uwbEvent, &evt, NULL, osWaitForever);
        switch (evt) {
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
        // uint32_t flag = osThreadFlagsWait(EVENT_TX_CPLT_BIT | EVENT_RX_OK_BIT | EVENT_RX_TIMEOUT_BIT | EVENT_RX_FAILED_BIT, osFlagsWaitAny, osWaitForever);

        // if (flag & EVENT_TX_CPLT_BIT)
        // {
        //     (void) current_Algorithm->onEvent(&dw1000_dev, eventPacketSent);
        // }
        // if (flag & EVENT_RX_OK_BIT)
        // {
        //     (void) current_Algorithm->onEvent(&dw1000_dev, eventPacketReceived);
        // }
        // if (flag & EVENT_RX_TIMEOUT_BIT)
        // {
        //     (void) current_Algorithm->onEvent(&dw1000_dev, eventReceiveTimeout);
        // }
        // if (flag & EVENT_RX_FAILED_BIT)
        // {
        //     (void) current_Algorithm->onEvent(&dw1000_dev, eventReceiveFailed);
        // }
        osMessageQueueGet(queue_uwbEvent, &evt, NULL, osWaitForever);
        switch (evt) {
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
}

/**
 * @brief 最小堆管理任务，插入，更新，删除
 * @param  void *arg RTOS要求参数为空指针类型
 * @retval none
 */
void task_minHeapManage(void *arg)
{
    UNUSED(arg);
    osStatus status;

    static tag_hashNode_t recv_processedDis = {0};
    static distance_manager_t task_dis_manage = {0};
    static outDistance_t min_selfDis = {0};
    static uint8_t can_buf[5] = {0};
    
    for (;;)
    {
        osMutexAcquire(dis_access_mutex,osWaitForever);

        status = osMessageQueueGet(queue_processDis, &recv_processedDis, 0, osWaitForever);
        // 接收到计算完成的距离，以及TWR完成时的tick数，进行插入以及更新，同时只要有标签TWR成功，就进行排序，最小距离发送
        if (status == osOK)    
        {
            disManager_update(&task_dis_manage, recv_processedDis.tag_id, recv_processedDis.distance, recv_processedDis.last_updateTick);

            // 堆更新后，清除无效距离，获取当前tick，将超时的标签清除
            uint32_t current_tick = osKernelGetTickCount();
            disManager_purgeExpired(&task_dis_manage, current_tick);

            // 定时获取堆顶最小距离
            status = osSemaphoreAcquire(tagDistClear_sem, osWaitForever);
            if (status == osOK)
            {
                tag_hashNode_t *min_dis_node = disManager_getMin(&task_dis_manage);
                min_selfDis.dis_value = min_dis_node->distance;
                min_selfDis.dis_index = min_dis_node->tag_id;
                min_selfDis.dis_class = ANCHOR_SELF_DIS;

                extern osMessageQueueId_t queue_minimalDis; 
                osMessageQueuePut(queue_minimalDis, &min_selfDis, 0, 0);

                can_buf[4] = min_selfDis.dis_index;
                dev_canSendMsg(CAN_EXT_ID_DIS, can_buf, ARRAY_LENGTH(can_buf));             // 直接发送
                log_d("local min dis :ID %d %.2f.", min_selfDis.dis_index, (float)(min_selfDis.dis_value) / 1000.0f);
            }
        }
        osMutexRelease(dis_access_mutex);
    }
}

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
static void dw1000Device_init(dwDevice_t *dev)
{
    dev->device_mode = ANCHOR;
    dev->twr_mode = LISTENER;
    dev->device_id = ANCHOR_DEV_ID;
    // dev->device_id = dev_getDipVal();
    dev->remainingRespToRx = -1;     // 初始化为 -1
    dev->rxOtherResp = 0;            // 接收其他基站resp帧计数
    dev->respTxIndex = 0;            // 该变量用于决定基站发送resp帧的位置，跟基站自身ID相关
    board_dw1000Init();
}

/******************************************************interrupt use callback function************************************************************/
extern osThreadId_t task_twrRun_handle;
static void txcallback(const dwt_cb_data_t *cb_data)
{
    UNUSED(cb_data);
    osThreadFlagsSet(task_twrRun_handle, EVENT_TX_CPLT_BIT);
    uwbEvent_t evt = eventPacketSent;
    osMessageQueuePut(queue_uwbEvent, &evt, 0, 0);
}

static void rxcallback(const dwt_cb_data_t *cb_data)
{
    UNUSED(cb_data);
    osThreadFlagsSet(task_twrRun_handle, EVENT_RX_OK_BIT);
        uwbEvent_t evt = eventPacketReceived;
    osMessageQueuePut(queue_uwbEvent, &evt, 0, 0);
}

static void rxTimeoutCallback(const dwt_cb_data_t *cb_data)
{
    UNUSED(cb_data);
    osThreadFlagsSet(task_twrRun_handle, EVENT_RX_TIMEOUT_BIT);
        uwbEvent_t evt = eventReceiveTimeout;
    osMessageQueuePut(queue_uwbEvent, &evt, 0, 0);
}

static void rxfailedcallback(const dwt_cb_data_t *cb_data)
{
    UNUSED(cb_data);
    osThreadFlagsSet(task_twrRun_handle, EVENT_RX_FAILED_BIT);
        uwbEvent_t evt = eventReceiveFailed;
    osMessageQueuePut(queue_uwbEvent, &evt, 0, 0);
}

/****************************************************备用配置***********************************************************/
// {    /* uwb_config3，channel5 脉冲频率64M 前导码长度128 数据率 6M8 */
    //     .chan = 5,
    //     .prf = DWT_PRF_64M,
    //     .txPreambLength = DWT_PLEN_128,
    //     .rxPAC = DWT_PAC8,
    //     .txCode = 10,
    //     .rxCode = 10,
    //     .nsSFD = 1,
    //     .dataRate = DWT_BR_6M8,
    //     .phrMode = DWT_PHRMODE_STD,
    //     .sfdTO = (129 + DW_NS_SFD_LEN_6M8 - 8)
    // }, 
    // {    /* uwb_config4，channel5 脉冲频率64M 前导码长度256 数据率 6M8 */
    //     .chan = 5,
    //     .prf = DWT_PRF_64M,
    //     .txPreambLength = DWT_PLEN_256,
    //     .rxPAC = DWT_PAC16,
    //     .txCode = 10,
    //     .rxCode = 10,
    //     .nsSFD = 1,
    //     .dataRate = DWT_BR_6M8,
    //     .phrMode = DWT_PHRMODE_STD,
    //     .sfdTO = (257 + DW_NS_SFD_LEN_6M8 - 16)
    // }, 
    // {    /* uwb_config5，channel5 脉冲频率64M 前导码长度1024 数据率 110K */
    //     .chan = 5,
    //     .prf = DWT_PRF_64M,
    //     .txPreambLength = DWT_PLEN_1024,
    //     .rxPAC = DWT_PAC32,
    //     .txCode = 10,
    //     .rxCode = 10,
    //     .nsSFD = 1,
    //     .dataRate = DWT_BR_110K,
    //     .phrMode = DWT_PHRMODE_STD,
    //     .sfdTO = (1025 + DW_NS_SFD_LEN_110K - 32)
    // }, 
    // {    /* uwb_config6，channel5 脉冲频率64M 前导码长度2048 数据率 110K */
    //     .chan = 5,
    //     .prf = DWT_PRF_64M,
    //     .txPreambLength = DWT_PLEN_2048,
    //     .rxPAC = DWT_PAC64,
    //     .txCode = 10,
    //     .rxCode = 10,
    //     .nsSFD = 1,
    //     .dataRate = DWT_BR_110K,
    //     .phrMode = DWT_PHRMODE_STD,
    //     .sfdTO = (2049 + DW_NS_SFD_LEN_110K - 64)
    // }, 

//static dwt_config_t uwb_config_channel2[6] = {
//    {   /* uwb_config0，channel2 脉冲频率64M 前导码长度1024 数据率 850K */
//        .chan = 2,
//        .prf = DWT_PRF_64M,
//        .txPreambLength = DWT_PLEN_1024,
//        .rxPAC = DWT_PAC32,
//        .txCode = 10,
//        .rxCode = 10,
//        .nsSFD = 1,
//        .dataRate = DWT_BR_850K,
//        .phrMode = DWT_PHRMODE_STD,
//        .sfdTO = (1025 + DW_NS_SFD_LEN_850K - 32)
//    },
//    {   /* uwb_config1，channel2 脉冲频率64M 前导码长度512 数据率 850K */
//        .chan = 2,
//        .prf = DWT_PRF_64M,
//        .txPreambLength = DWT_PLEN_512,
//        .rxPAC = DWT_PAC16,
//        .txCode = 10,
//        .rxCode = 10,
//        .nsSFD = 1,
//        .dataRate = DWT_BR_850K,
//        .phrMode = DWT_PHRMODE_STD,
//        .sfdTO = (513 + DW_NS_SFD_LEN_850K - 16)
//    }, 
//    {   /* uwb_config2，channel2 脉冲频率64M 前导码长度256 数据率 850K */
//        .chan = 2,
//        .prf = DWT_PRF_64M,
//        .txPreambLength = DWT_PLEN_256,
//        .rxPAC = DWT_PAC16,
//        .txCode = 10,
//        .rxCode = 10,
//        .nsSFD = 1,
//        .dataRate = DWT_BR_850K,
//        .phrMode = DWT_PHRMODE_STD,
//        .sfdTO = (257 + DW_NS_SFD_LEN_850K - 16)
//    },
//    {   /* uwb_config3，channel2 脉冲频率16M 前导码长度1024 数据率 850K */
//        .chan = 2,
//        .prf = DWT_PRF_16M,
//        .txPreambLength = DWT_PLEN_1024,
//        .rxPAC = DWT_PAC32,
//        .txCode = 3,
//        .rxCode = 3,
//        .nsSFD = 1,
//        .dataRate = DWT_BR_850K,
//        .phrMode = DWT_PHRMODE_STD,
//        .sfdTO = (1025 + DW_NS_SFD_LEN_850K - 32)
//    },
//    {   /* uwb_config4，channel2 脉冲频率64M 前导码长度1024 数据率 110K */
//        .chan = 2,
//        .prf = DWT_PRF_64M,
//        .txPreambLength = DWT_PLEN_1024,
//        .rxPAC = DWT_PAC32,
//        .txCode = 10,
//        .rxCode = 10,
//        .nsSFD = 1,
//        .dataRate = DWT_BR_110K,
//        .phrMode = DWT_PHRMODE_STD,
//        .sfdTO = (1025 + DW_NS_SFD_LEN_110K - 32)
//    },
//    {   /* uwb_config5，channel2 脉冲频率64M 前导码长度2048 数据率 110K */
//        .chan = 2,
//        .prf = DWT_PRF_64M,
//        .txPreambLength = DWT_PLEN_2048,
//        .rxPAC = DWT_PAC64,
//        .txCode = 10,
//        .rxCode = 10,
//        .nsSFD = 1,
//        .dataRate = DWT_BR_110K,
//        .phrMode = DWT_PHRMODE_STD,
//        .sfdTO = (2049 + DW_NS_SFD_LEN_110K - 64)
//    },
//};

//static dwt_config_t uwb_config_channel7[] = {
//    {   /* uwb_config0，channel7 脉冲频率64M 前导码长度128 数据率 6M8 */
//        .chan = 7,
//        .prf = DWT_PRF_64M,
//        .txPreambLength = DWT_PLEN_128,
//        .rxPAC = DWT_PAC8,
//        .txCode = 19,
//        .rxCode = 20,
//        .nsSFD = 1,
//        .dataRate = DWT_BR_6M8,
//        .phrMode = DWT_PHRMODE_STD,
//        .sfdTO = (129 + DW_NS_SFD_LEN_6M8 - 8)
//    },
//    {   /* uwb_config1，channel7 脉冲频率64M 前导码长度256 数据率 6M8 */
//        .chan = 7,
//        .prf = DWT_PRF_64M,   
//        .txPreambLength = DWT_PLEN_256,
//        .rxPAC = DWT_PAC16,
//        .txCode = 19,
//        .rxCode = 20,
//        .nsSFD = 1,
//        .dataRate = DWT_BR_6M8,
//        .phrMode = DWT_PHRMODE_STD,
//        .sfdTO = (257 + DW_NS_SFD_LEN_6M8 - 16)
//    }, 
//    {   /* uwb_config2，channel7 脉冲频率64M 前导码长度256 数据率 850K */
//        .chan = 7,
//        .prf = DWT_PRF_64M,
//        .txPreambLength = DWT_PLEN_256,
//        .rxPAC = DWT_PAC16,
//        .txCode = 19,
//        .rxCode = 20,
//        .nsSFD = 1,
//        .dataRate = DWT_BR_850K,
//        .phrMode = DWT_PHRMODE_STD,
//        .sfdTO = (257 + DW_NS_SFD_LEN_850K - 16)
//    }, 
//};
