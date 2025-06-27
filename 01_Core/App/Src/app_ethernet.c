#include "app_ethernet.h"
#include "dev_led_buzzer_dip.h"
#include "main.h"
#include "cmsis_os.h"

#include "wizchip_conf.h"
#include "dev_w5500.h"
#include "elog.h"

#define SOCKET_ID 0
#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/* network information */
wiz_NetInfo default_net_info = {
    .mac  = {0x00, 0x08, 0xdc, 0x12, 0x22, 0x12},
    .ip   = {192, 168, 1, 30},
    .gw   = {192, 168, 1, 1},
    .sn   = {255, 255, 255, 0},
    .dns  = {8, 8, 8, 8},
    .dhcp = NETINFO_DHCP};        // static ip

uint8_t ethernet_buf[ETHERNET_BUF_MAX_SIZE] = {0};

uint8_t dest_ip[4] = {192, 168, 1, 20};
uint16_t dest_port = 8080;

void task_eth(void* arg)
{
    wiz_NetInfo net_info;
    
    dev_w5500Initialize();
    log_d("%s network install example\r\n",_WIZCHIP_ID_);

    dev_w5500NetWorkInit(ethernet_buf, &default_net_info);
    wizchip_getnetinfo(&net_info);

    setSn_KPALVTR(SOCKET_ID, 6); // 
    // log_d("please try ping %d.%d.%d.%d\r\n", net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3]);
    for(;;)
    {
        dev_ledBlink(uwb_ok_led);
        dev_w5500TcpClientLoopBack(SOCKET_ID, ethernet_buf, dest_ip, dest_port);
        osDelay(100);
    }
}
