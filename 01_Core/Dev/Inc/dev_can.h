#ifndef __DEV_CAN_H__
#define __DEV_CAN_H__

#define  CAN_EXT_ID_BUTTON 0xAAA3U
#define  CAN_EXT_ID_DIS    0xAAA0U

void dev_canInit(void);
void dev_canStartRx(void);
void dev_canSendMsg(uint32_t extId, uint8_t* data, uint32_t length);
void dev_canPollingRxMsg(void);
#endif
