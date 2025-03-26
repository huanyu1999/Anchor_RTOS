#ifndef __DEV_CAN_H__
#define __DEV_CAN_H__

void dev_canInit(void);
void dev_canStartRx(void);
void dev_canSendMsg(uint32_t extId, uint8_t* data, uint32_t length);
void dev_canPollingRxMsg(void);
#endif
