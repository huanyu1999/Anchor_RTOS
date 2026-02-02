#ifndef __DEV_W5500_H__
#define __DEV_W5500_H__

#include "main.h"
#include "wizchip_conf.h"

#define SOCKET_ID 0

/* Loopback test debug message printout enable */
#define _LOOPBACK_DEBUG_

/* DATA_BUF_SIZE define for Loopback example */
#ifndef DATA_BUF_SIZE
#define DATA_BUF_SIZE 2048
#endif

typedef enum {
    EVENT_SENDOK,
    EVENT_TIMEOUT,
    EVENT_RECV,
    EVENT_DISCON,
    EVENT_CON
} w5500_event;

typedef struct {
    uint16_t dest_port;
    uint8_t socket_num;
    uint8_t buffer[DATA_BUF_SIZE];

} w5500_device;

typedef struct {
    void (*init)(w5500_device* dev, uint8_t *ethernet_buff, wiz_NetInfo *conf_info);
    void (*handle_w5500Event)(w5500_device* dev, w5500_event event);
} dev_w5500Handler;

void dev_w5500Init(void);
void dev_w5500VersionCheck(void);
void dev_w5500PhyConfigInit(void);
void dev_w5500PhyInfoGet(void);
void dev_w5500PhyLinkCheck(void);
void dev_w5500TcpServerInit(w5500_device* dev, uint8_t *ethernet_buff, wiz_NetInfo *conf_info);
void dev_w5500PrintfNetworkInfo(void);
void dev_w5500TcpServerHandler(w5500_device* dev, w5500_event event);
void w5500_isr(void);
int32_t loopback_tcps(uint8_t sn, uint8_t *buf, uint16_t port);
int32_t dev_w5500TcpClientLoopBack(uint8_t sn, uint8_t* buf, uint8_t *destip, uint16_t dest_port);




#endif
