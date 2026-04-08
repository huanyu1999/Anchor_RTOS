#include "app_network.h"
#include <string.h>
#include "main.h"
#include "dev_w5500.h"
#include "cmsis_os.h"
#include "socket.h"
#include "elog.h"

#include "net_protocol.h"
#include "dw_sort.h"
#include "dw_instance.h"

#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/* 标签状态推送间隔 ms */
#define TAG_STATUS_PUSH_INTERVAL_MS  500

/* 标签断联判定阈值 ms（与 DIST_EXPIRE_TICK 对齐） */
#define TAG_OFFLINE_THRESHOLD_MS     2000

/* network information */
wiz_NetInfo default_net_info = {
    .mac  = {0x00, 0x08, 0xdc, 0x12, 0x22, 0x12},
    .ip   = {192, 168, 1, 30},
    .gw   = {192, 168, 1, 1},
    .sn   = {255, 255, 255, 0},
    .dns  = {8, 8, 8, 8},
    .dhcp = NETINFO_STATIC
};

uint8_t ethernet_buf[ETHERNET_BUF_MAX_SIZE] = {0};

uint8_t dest_ip[4] = {192, 168, 1, 10};
uint16_t dest_port = 8080;

extern dev_w5500Handler dev_w5500TcpServer;
extern osSemaphoreId_t sema_w5500Int;
extern w5500_device dev_w5500;

/* ========================= 标签状态推送 ========================= */

/**
 * @brief  构造标签状态推送帧并发送
 *         遍历 distance_manager 中的所有在线标签，打包为 CMD_TAG_STATUS 推送
 */
static void push_tag_status(void)
{
    /* 仅在 socket 已连接时推送 */
    if (getSn_SR(SOCKET_ID) != SOCK_ESTABLISHED)
    {
        return;
    }

    distance_manager_t *mgr = disManager_getHandle();
    uint16_t tag_count = disManager_getNodeNum(mgr);

    if (tag_count == 0)
    {
        return;
    }

    /* 构造推送载荷：[tag_count: 2B] + N * net_tag_entry_t */
    uint8_t payload[2 + MAX_NODE_NUM * sizeof(net_tag_entry_t)];
    uint16_t idx = 0;
    uint32_t now_tick = osKernelGetTickCount();

    payload[idx++] = (uint8_t)(tag_count & 0xFF);
    payload[idx++] = (uint8_t)((tag_count >> 8) & 0xFF);

    for (uint16_t i = 0; i < mgr->heap_size && i < MAX_NODE_NUM; i++)
    {
        tag_hashNode_t *node = mgr->heap[i];
        net_tag_entry_t entry;

        entry.tag_id      = node->tag_id;
        entry.distance_mm = node->distance;
        entry.rx_power_dbm = rx_power;   /* 当前全局信号强度（近似） */

        uint32_t elapsed = now_tick - node->last_updateTick;
        entry.age_ms = (elapsed > 0xFFFF) ? 0xFFFF : (uint16_t)elapsed;

        memcpy(&payload[idx], &entry, sizeof(entry));
        idx += sizeof(entry);
    }

    net_protocol_send_push(CMD_TAG_STATUS, payload, idx);
}

/* ========================= 网络任务 ========================= */

void task_eth(void *arg)
{
    UNUSED(arg);
    wiz_NetInfo net_info;
    dev_w5500Handler *current_handler = &dev_w5500TcpServer;

    dev_w5500Init();
    net_protocol_init();

    current_handler->init(&dev_w5500, ethernet_buf, &default_net_info);
    wizchip_getnetinfo(&net_info);

    uint32_t last_push_tick = osKernelGetTickCount();

    for (;;)
    {
        /* 等待 W5500 中断，但设置超时以便定时推送 */
        osStatus_t status = osSemaphoreAcquire(sema_w5500Int, TAG_STATUS_PUSH_INTERVAL_MS);

        if (status == osOK)
        {
            w5500_isr();
        }

        /* 定时推送标签状态 */
        uint32_t now = osKernelGetTickCount();
        if ((now - last_push_tick) >= TAG_STATUS_PUSH_INTERVAL_MS)
        {
            push_tag_status();
            last_push_tick = now;
        }
    }
}
