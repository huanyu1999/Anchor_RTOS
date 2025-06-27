#ifndef __DEV_W5500_H__
#define __DEV_W5500_H__

#include "main.h"
#include "wizchip_conf.h"

/* Loopback test debug message printout enable */
#define _LOOPBACK_DEBUG_

/* DATA_BUF_SIZE define for Loopback example */
#ifndef DATA_BUF_SIZE
#define DATA_BUF_SIZE 2048
#endif

void dev_w5500Initialize(void);
void dev_w5500VersionCheck(void);
void dev_w5500PhyInfoGet(void);
void dev_w5500PhyLinkCheck(void);
void dev_w5500NetWorkInit(uint8_t *ethernet_buff, wiz_NetInfo *conf_info);
int32_t dev_w5500TcpClientLoopBack(uint8_t sn, uint8_t* buf, uint8_t *destip, uint16_t dest_port);
void dev_w5500PrintfNetworkInfo(void);

#endif
