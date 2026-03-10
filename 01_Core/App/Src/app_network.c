#include "app_network.h"
// #include "dev_led_buzzer_dip.h"
#include "main.h"
#include "dev_w5500.h"
#include "cmsis_os.h"
// #include "socket.h"

#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/* network information */
wiz_NetInfo default_net_info = {
    .mac  = {0x00, 0x08, 0xdc, 0x12, 0x22, 0x12},
    .ip   = {192, 168, 1, 30},
    .gw   = {192, 168, 1, 1},
    .sn   = {255, 255, 255, 0},
    .dns  = {8, 8, 8, 8},
    .dhcp = NETINFO_STATIC
};        // use static ip

uint8_t ethernet_buf[ETHERNET_BUF_MAX_SIZE] = {0};

uint8_t dest_ip[4] = {192, 168, 1, 10};     // 默认静态IP，后续可通过上位机自由设置IP
uint16_t dest_port = 8080;

extern dev_w5500Handler dev_w5500TcpServer;
extern osSemaphoreId_t sema_w5500Int;       // 后续可以重写这个w5500状态机，适配TCP通信
extern w5500_device dev_w5500;
void task_eth(void* arg)
{
    UNUSED(arg);
    wiz_NetInfo net_info;
    dev_w5500Handler *current_handler = &dev_w5500TcpServer;
    dev_w5500Init();

    current_handler->init(&dev_w5500, ethernet_buf, &default_net_info);
    wizchip_getnetinfo(&net_info);

    // setSn_KPALVTR(SOCKET_ID, 6); 
    for(;;)
    {
        osStatus_t status = osSemaphoreAcquire(sema_w5500Int, osWaitForever); // 采用中断触发的方式执行，获取信号量
        if (status == osOK)
        {
            w5500_isr();
        }
    }
}
