#include <stdio.h>
#include <string.h>
#include "dwt_delay.h"
#include "dev_w5500.h"
#include "board_w5500.h"
#include "spi.h"
#include "drv_timer.h"
#include "cmsis_os.h"

#include "socket.h"
#include "w5500.h"
#include "dhcp.h"
#include "elog.h"
#include "net_protocol.h"

#define W5500_VERSION 0x04
#define KEEPALIVE_ENABLE 1
#define IS_SOCKET_INT(ch)  (0x01 << ch)

TIM_HandleTypeDef dhcp_oneSecondHandle;

w5500_device dev_w5500 = { 0 };
static net_parser_t net_parser = {0};

dev_w5500Handler dev_w5500TcpServer = {
    .init = dev_w5500TcpServerInit,
    .handle_w5500Event = dev_w5500TcpServerHandler
};

dev_w5500Handler dev_w5500TcpCLient = {
    0, 0
};

static uint8_t dev_w5500DhcpProcess(uint8_t sn, uint8_t *buffer);

void dev_w5500Init(void)
{
    // drv_setTimerForInt(&dhcp_oneSecondHandle, TIM5, 1, 9);   // 初始化TIM5定时器中断给dhcp处理用，中断为1s触发一次

    board_w5500Init();

    board_w5500CallbackReg();       // 注册回调函数到W5500

    board_w5500Reset();

    dev_w5500VersionCheck();

    // dev_w5500PhyLinkCheck();
}

void dev_w5500VersionCheck(void)
{
    uint8_t error_count = 0;
    while (1)
    {
        // HAL_Delay(1000);
        DWT_Delay(1000000);
        if (getVERSIONR() != W5500_VERSION)
        {
            error_count++;
            if (error_count > 5)
            {
                log_e("error, %s version is 0x%02x, but read %s version value = 0x%02x\r\n", _WIZCHIP_ID_, W5500_VERSION, _WIZCHIP_ID_, getVERSIONR());
                Error_Handler();
            }
        }
        else
        {
            log_d("w5500 version check success.");
            break;
        }
    }
}

void dev_w5500PhyConfigInit(void)
{   
    wiz_PhyConf phy_conf;
    phy_conf.by = PHY_CONFBY_SW;
    phy_conf.mode = PHY_MODE_MANUAL;
    phy_conf.duplex = PHY_DUPLEX_FULL;
    phy_conf.speed = PHY_SPEED_100;

    ctlwizchip(CW_SET_PHYCONF, &phy_conf);
}

void dev_w5500PhyInfoGet(void)
{
    uint8_t phy_conf;
    phy_conf = getPHYCFGR();
    log_d("The current Mbtis speed : %dMbps\r\n", phy_conf & 0x02 ? 100 : 10);
    log_d("The current Duplex Mode : %s\r\n", phy_conf & 0x04 ? "Full-Duplex" : "Half-Duplex");
}

void dev_w5500PhyLinkCheck(void)
{
    uint8_t phy_link_status;
    dev_w5500PhyConfigInit();
    do
    {
        // HAL_Delay(1000);
        DWT_Delay(1000000);
        ctlwizchip(CW_GET_PHYLINK, (void *)&phy_link_status);
        if (phy_link_status == PHY_LINK_ON)
        {
            log_d("PHY link\r\n");
            dev_w5500PhyInfoGet();
        }
        else
        {
            log_d("PHY no link\r\n");
        }
    } while (phy_link_status == PHY_LINK_OFF);

}

void dev_w5500TcpServerInit(w5500_device* dev, uint8_t *ethernet_buff, wiz_NetInfo *conf_info)
{
    int ret;
    setSIMR(0x01);              // 打开中断
    setSn_IMR(SOCKET_ID, 0x0f);

    dev->dest_port = 8080;
    dev->socket_num = SOCKET_ID;

    wizchip_setnetinfo(conf_info); // Configuring Network Information
    if (conf_info->dhcp == NETINFO_DHCP)
    {
        ret = dev_w5500DhcpProcess(0, ethernet_buff);
        if (ret == 0)
        {
            conf_info->dhcp = NETINFO_STATIC;
            wizchip_setnetinfo(conf_info);
        }
    }
    dev_w5500PrintfNetworkInfo();

    if (socket(dev->socket_num, Sn_MR_TCP, dev->dest_port, 0x00) != SOCKET_ID) {
#ifdef _LOOPBACK_DEBUG_
        log_d("%d:socket open failed.", dev->socket_num);
#endif
    } else {
#ifdef _LOOPBACK_DEBUG_
        log_d("%d:Socket opened\r\n", dev->socket_num);
#endif
        listen(SOCKET_ID);
#ifdef _LOOPBACK_DEBUG_
        log_d("%d:Listen, TCP server, port [%d]\r\n", dev->socket_num, dev->dest_port);
#endif
    }
}

void dev_w5500PrintfNetworkInfo(void)
{
    wiz_NetInfo net_info;
    wizchip_getnetinfo(&net_info);

    if (net_info.dhcp == NETINFO_DHCP)
    {
        log_d("====================================================================================================\r\n");
        log_d(" %s network configuration : DHCP\r\n\r\n", _WIZCHIP_ID_);
    }
    else
    {
        log_d("====================================================================================================\r\n");
        log_d(" %s network configuration : static\r\n\r\n", _WIZCHIP_ID_);
    }

    log_d(" MAC         : %02X:%02X:%02X:%02X:%02X:%02X\r\n", net_info.mac[0], net_info.mac[1], net_info.mac[2], net_info.mac[3], net_info.mac[4], net_info.mac[5]);
    log_d(" IP          : %d.%d.%d.%d\r\n", net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3]);
    log_d(" Subnet Mask : %d.%d.%d.%d\r\n", net_info.sn[0], net_info.sn[1], net_info.sn[2], net_info.sn[3]);
    log_d(" Gateway     : %d.%d.%d.%d\r\n", net_info.gw[0], net_info.gw[1], net_info.gw[2], net_info.gw[3]);
    log_d(" DNS         : %d.%d.%d.%d\r\n", net_info.dns[0], net_info.dns[1], net_info.dns[2], net_info.dns[3]);
    log_d("====================================================================================================");
}

void dev_w5500TcpServerHandler(w5500_device* dev, w5500_event event) {
    uint16_t len = 0;
    // uint16_t sent_size = 0;
    uint8_t destip[4] = {0, 0, 0, 0};
    uint16_t destport;

    switch (event) {
    case EVENT_SENDOK:
#ifdef _LOOPBACK_DEBUG_
            log_d("%d:socket send OK.", dev->socket_num);
#endif
        break;

    case EVENT_TIMEOUT: 
        if (socket(dev->socket_num, Sn_MR_TCP, dev->dest_port, 0x00) != SOCKET_ID) {    // 重新打开socket
#ifdef _LOOPBACK_DEBUG_
            log_d("%d:socket open failed.", dev->socket_num);
#endif
        } else {
#ifdef _LOOPBACK_DEBUG_
            log_d("%d:Socket opened\r\n", dev->socket_num);
#endif
            listen(SOCKET_ID);
#ifdef _LOOPBACK_DEBUG_
            log_d("%d:Listen, TCP server, port [%d]\r\n", dev->socket_num, dev->dest_port);
#endif
        }
        break;

    case EVENT_DISCON:
        if ((len = getSn_RX_RSR(dev->socket_num)) > 0) { // 如果关闭有接收到数据，先接收再关闭
            if (len > DATA_BUF_SIZE) {
                len = DATA_BUF_SIZE;
            }
            recv(dev->socket_num, dev->buffer, len);
            dev->buffer[len] = 0x00;
            log_d("recv_data:%s", dev->buffer);
            int ret = send(dev->socket_num, dev->buffer, len);
        }
        disconnect(dev->socket_num);

        if (getSn_SR(dev->socket_num) == SOCK_CLOSED) {
            if (socket(dev->socket_num, Sn_MR_TCP, dev->dest_port, 0x00) != SOCKET_ID) {
#ifdef _LOOPBACK_DEBUG_
            log_d("%d:socket open failed.", dev->socket_num);
#endif
            } else {
#ifdef _LOOPBACK_DEBUG_
                log_d("%d:Socket opened\r\n", dev->socket_num);
#endif
                listen(SOCKET_ID);
#ifdef _LOOPBACK_DEBUG_
                log_d("%d:Listen, TCP server, port [%d]\r\n", dev->socket_num, dev->dest_port);
#endif
            }
        }
        break;
        
    case EVENT_RECV:
        if ((len = getSn_RX_RSR(dev->socket_num)) > 0)
        {
            if (len > DATA_BUF_SIZE)
            {
                len = DATA_BUF_SIZE;
            }
            recv(dev->socket_num, dev->buffer, len);
            net_protocol_feed(&net_parser, dev->buffer, len);
        }
        break;

    case EVENT_CON:
        /* 新连接建立，重置帧解析器 */
        memset(&net_parser, 0, sizeof(net_parser));
#ifdef _LOOPBACK_DEBUG_
        getSn_DIPR(dev->socket_num, destip);
        destport = getSn_DPORT(dev->socket_num);
        log_d("%d:Connected - %d.%d.%d.%d : %d\r\n", dev->socket_num, destip[0], destip[1], destip[2], destip[3], destport);
#endif
        break;
    }
}

void w5500_isr(void) {
    uint8_t sir_val = 0;
    uint8_t tmp, sn;
    sir_val = getSIR();         // 获取是哪些socket触发了中断
    if (sir_val != 0xff) {
        setSIMR(0x00);          // 暂时关闭Socket 中断，避免中断处理有产生新的中断
        for (sn = 0; sn < _WIZCHIP_SOCK_NUM_; sn++) {
            tmp = 0;
            if (sir_val & IS_SOCKET_INT(sn)) {
                tmp = getSn_IR(sn);
                if(tmp != 0x03) {//error interrupt
                    // I_STATUS[sn] |= tmp;
                    switch (tmp) {
                    case Sn_IR_CON:
                        dev_w5500TcpServer.handle_w5500Event(&dev_w5500, EVENT_CON);
                        break;
                    case Sn_IR_DISCON:
                        dev_w5500TcpServer.handle_w5500Event(&dev_w5500, EVENT_DISCON);
                        break;
                    case Sn_IR_RECV:
                        dev_w5500TcpServer.handle_w5500Event(&dev_w5500, EVENT_RECV);
                        break;
                    case Sn_IR_TIMEOUT:
                        dev_w5500TcpServer.handle_w5500Event(&dev_w5500, EVENT_TIMEOUT);
                        break;
                    case Sn_IR_SENDOK:
                        dev_w5500TcpServer.handle_w5500Event(&dev_w5500, EVENT_SENDOK);
                        break;
                    }
                    tmp &= 0x1f;
                    setSn_IR(sn, tmp); // 清除Sn_IR bit
                }
            }
        }
        setSIMR(0xff);  // 重新打开Socket 中断
    }
}

/**
 * @brief   tcp client loopback test
 * @param   sn:         socket number
 * @param   buf:        Data sending and receiving cache
 * @param   destip:     Destination IP address
 * @param   destport:   Destination port
 * @return  value for SOCK_ERRORs,return 1:no error
 */
int32_t dev_w5500TcpClientLoopBack(uint8_t sn, uint8_t* buf, uint8_t *destip, uint16_t dest_port)
{
    int32_t ret; // return value for SOCK_ERRORs
    uint16_t size = 0, sentsize = 0;

    // Destination (TCP Server) IP info (will be connected)
    // >> loopback_tcpc() function parameter
    // >> Ex)
    uint8_t  dip[4] = {192, 168, 0, 214};
    uint16_t dport = 5000;
    getSn_DIPR(sn, dip);
    dport = getSn_DPORT(sn);
    // Port number for TCP client (will be increased)
    static uint16_t any_port = 50000;

    // Socket Status Transitions
    // Check the W5500 Socket n status register (Sn_SR, The 'Sn_SR' controlled by Sn_CR command or Packet send/recv status)
    switch (getSn_SR(sn))
    {
        case SOCK_ESTABLISHED:
            if (getSn_IR(sn) & Sn_IR_CON) // Socket n interrupt register mask; TCP CON interrupt = connection with peer is successful
            {
#if KEEPALIVE_ENABLE == 1
                // We need to send a packet of data to activate keepalive
                ret = send(sn, (uint8_t *)"", 1); // Data send process
                if (ret < 0)                      // Send Error occurred (sent data length < 0)
                {
                close(sn); // socket close
                return ret;
                }
#endif
#ifdef _LOOPBACK_DEBUG_
                log_d("%d:Connected to - %d.%d.%d.%d : %d\r\n", sn, destip[0], destip[1], destip[2], destip[3], dest_port);
#endif
                setSn_IR(sn, Sn_IR_CON); // this interrupt should be write the bit cleared to '1'
            }

            /* Data Transaction Parts; Handle the [data receive and send] process */ 
            if ((size = getSn_RX_RSR(sn)) > 0) // Sn_RX_RSR: Socket n Received Size Register, Receiving data length
            {
                if (size > DATA_BUF_SIZE)
                    size = DATA_BUF_SIZE;        // DATA_BUF_SIZE means user defined buffer size (array)
                ret       = recv(sn, buf, size); // Data Receive process (H/W Rx socket buffer -> User's buffer)
                buf[size] = 0x00;
                log_d("rece from %d.%d.%d.%d:%d data:%s\r\n", dip[0], dip[1], dip[2], dip[3], dport, buf);
                if (ret <= 0)
                    return ret; // If the received data length <= 0, receive failed and process end
                size     = (uint16_t)ret;
                sentsize = 0;

                // Data sentsize control
                while (size != sentsize)
                {
                    ret = send(sn, buf + sentsize, size - sentsize); // Data send process (User's buffer -> Destination through H/W Tx socket buffer)
                    if (ret < 0)                                     // Send Error occurred (sent data length < 0)
                    {
                        close(sn);                                   // socket close
                        return ret;
                    }
                    sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
                }
            }
            break;

        case SOCK_CLOSE_WAIT:
#ifdef _LOOPBACK_DEBUG_
            log_d("%d:CloseWait\r\n", sn);
#endif
            if ((ret = disconnect(sn)) != SOCK_OK)
                return ret;
#ifdef _LOOPBACK_DEBUG_
            log_d("%d:Socket Closed\r\n", sn);
#endif
            break;

        case SOCK_INIT:
#ifdef _LOOPBACK_DEBUG_
            log_d("%d:Try to connect to the %d.%d.%d.%d : %d\r\n", sn, destip[0], destip[1], destip[2], destip[3], dest_port);
#endif
            if ((ret = connect(sn, destip, dest_port)) != SOCK_OK)
                return ret; //	Try to TCP connect to the TCP server (destination)
            break;

        case SOCK_CLOSED:
            close(sn);
            if ((ret = socket(sn, Sn_MR_TCP, any_port++, 0x00)) != sn)
            {
                if (any_port == 0xffff)
                    any_port = 50000;
                return ret; // TCP socket open with 'any_port' port number
            }
#ifdef _LOOPBACK_DEBUG_
            log_d("%d:TCP client loopback start\r\n", sn);
            log_d("%d:Socket opened\r\n", sn);
#endif
            break;
        
        default:
            break;
    }
    return 1;
}

/**
 * @brief   tcp server loopback test
 * @param   sn:    socket number
 * @param   buf:   Data sending and receiving cache
 * @param   port:  Listen port
 * @return  value for SOCK_ERRORs,return 1:no error
 */
int32_t loopback_tcps(uint8_t sn, uint8_t *buf, uint16_t port) {
    int32_t ret;
    uint16_t size = 0, sentsize = 0;

#ifdef _LOOPBACK_DEBUG_
    uint8_t destip[4];
    uint16_t destport;
#endif

    switch (getSn_SR(sn)) {
    case SOCK_ESTABLISHED:
        if (getSn_IR(sn) & Sn_IR_CON) {
#ifdef _LOOPBACK_DEBUG_
            getSn_DIPR(sn, destip);
            destport = getSn_DPORT(sn);
            log_d("%d:Connected - %d.%d.%d.%d : %d\r\n", sn, destip[0], destip[1], destip[2], destip[3], destport);
#endif
#if KEEPALIVE_ENABLE == 1
            // We need to send a packet of data to activate keepalive
            ret = send(sn, (uint8_t *)"", 1); // Data send process
            if (ret < 0) {                    // Send Error occurred (sent data length < 0)
                close(sn); // socket close
                return ret;
            }
#endif
            setSn_IR(sn, Sn_IR_CON);
        }
        if ((size = getSn_RX_RSR(sn)) > 0) { // Don't need to check SOCKERR_BUSY because it doesn't not occur.
            if (size > DATA_BUF_SIZE) {
                size = DATA_BUF_SIZE;
            }
            ret = recv(sn, buf, size);
            if (ret <= 0) {
                return ret; // check SOCKERR_BUSY & SOCKERR_XXX. For showing the occurrence of SOCKERR_BUSY.
            }
            size = (uint16_t)ret;
            sentsize = 0;
            buf[size] = 0x00;
            log_d("rece data:%s\r\n", buf);
            while (size != sentsize) {
                ret = send(sn, buf + sentsize, size - sentsize);
                if (ret < 0) {
                    close(sn);
                    return ret;
                }
                sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
            }
        }
        break;
    case SOCK_CLOSE_WAIT:
#ifdef _LOOPBACK_DEBUG_
        log_d("%d:CloseWait\r\n", sn);
#endif
        if ((ret = disconnect(sn)) != SOCK_OK) {
            return ret;
        }
#ifdef _LOOPBACK_DEBUG_
        log_d("%d:Socket Closed\r\n", sn);
#endif
        break;
    case SOCK_INIT:
#ifdef _LOOPBACK_DEBUG_
        log_d("%d:Listen, TCP server loopback, port [%d]\r\n", sn, port);
#endif
        if ((ret = listen(sn)) != SOCK_OK) {
            return ret;
        }
        break;
    case SOCK_CLOSED:
#ifdef _LOOPBACK_DEBUG_
        log_d("%d:TCP server loopback start\r\n", sn);
#endif
        if ((ret = socket(sn, Sn_MR_TCP, port, 0x00)) != sn) {
            return ret;
        }
#ifdef _LOOPBACK_DEBUG_
        log_d("%d:Socket opened\r\n", sn);
#endif
        break;
    default:
        break;
    }
    return 1;
}

static uint8_t dev_w5500DhcpProcess(uint8_t sn, uint8_t *buffer)
{   
    wiz_NetInfo conf_info;
    uint8_t dhcp_runFlag = 1;
    uint8_t dhcp_okFlag = 0;

    /* Registration DHCP_time_handler to 1 second timer */
    DHCP_init(sn, buffer);
    log_d("DHCP running\r\n");
    while (1)
    {
        switch (DHCP_run()) // Do the DHCP client
        {
            case DHCP_IP_LEASED: // DHCP Acquiring network information successfully
                if (dhcp_okFlag == 0)
                {
                    dhcp_okFlag = 1;
                    dhcp_runFlag = 0;
                }
                break;
            case DHCP_FAILED:
                dhcp_runFlag = 0;
                break;
        }
        if (dhcp_runFlag == 0)
        {
            log_d("DHCP %s!\r\n", dhcp_okFlag ? "success" : "fail");
            DHCP_stop();

            /*DHCP obtained successfully, cancel the registration DHCP_time_handler*/

            if (dhcp_okFlag)
            {
                getIPfromDHCP(conf_info.ip);
                getGWfromDHCP(conf_info.gw);
                getSNfromDHCP(conf_info.sn);
                getDNSfromDHCP(conf_info.dns);
                conf_info.dhcp = NETINFO_DHCP;
                getSHAR(conf_info.mac);
                wizchip_setnetinfo(&conf_info); // Update network information to network information obtained by DHCP
                return 1;
            }
            return 0;
        }
    }
}
