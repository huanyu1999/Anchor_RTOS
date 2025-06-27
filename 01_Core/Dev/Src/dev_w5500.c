#include <stdio.h>


#include "dev_w5500.h"
#include "board_w5500.h"
#include "spi.h"
#include "timer.h"
#include "socket.h"
#include "w5500.h"
#include "dhcp.h"
#include "elog.h"

#define W5500_VERSION 0x04

TIM_HandleTypeDef dhcp_oneSecondHandle;

static uint8_t dev_w5500DhcpProcess(uint8_t sn, uint8_t *buffer);

void dev_w5500Initialize(void)
{
    drv_setTimerForInt(&dhcp_oneSecondHandle, TIM4, 1, 9);

    board_w5500Init();

    board_w5500CallbackReg();       // 注册回调函数到W5500

    board_w5500Reset();

    dev_w5500VersionCheck();

    dev_w5500PhyLinkCheck();
}

void dev_w5500VersionCheck(void)
{
    uint8_t error_count = 0;
    while (1)
    {
        HAL_Delay(1000);
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
            break;
        }
    }
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
    do
    {
        HAL_Delay(1000);
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

void dev_w5500NetWorkInit(uint8_t *ethernet_buff, wiz_NetInfo *conf_info)
{
    int ret;
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
    uint8_t dip[4] = {192, 168, 0, 214};
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
                printf("rece from %d.%d.%d.%d:%d data:%s\r\n", dip[0], dip[1], dip[2], dip[3], dport, buf);
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
