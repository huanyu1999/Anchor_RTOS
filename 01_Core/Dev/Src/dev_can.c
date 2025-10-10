#include "dev_led_buzzer_dip.h"
#include "board_gpio.h"
#include "can.h"
#include "usart.h"
#include "elog.h"

void dev_canInit(void)
{
#if USE_CAN1
    drv_can1Init();
#elif USE_CAN2
    drv_can2Init();
#else
    #error "No CAN interface defined! Define USE_CAN1 or USE_CAN2."
#endif
}

void dev_canStartRx(void)
{
    drv_canEnableReceiveInt();
}

/**
  * @brief  can发送一条指定长度的数据
  * @param  
  *         
  * @retval 
  */
void dev_canSendMsg(uint32_t extId, uint8_t* data, uint32_t length) 
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t TxMailbox = 0;
    tx_header.ExtId = extId;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = length;
    tx_header.TransmitGlobalTime = DISABLE;
    // while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan1 ) == 0 );
    // 替换为中断处理后续
#if USE_CAN1
    if(HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &TxMailbox) == HAL_OK)
#elif USE_CAN2
    if(HAL_CAN_AddTxMessage(&hcan2, &tx_header, data, &TxMailbox) == HAL_OK)
#else
    #error "No CAN interface defined! Define USE_CAN1 or USE_CAN2."
#endif
    {
        // log_i("HAL_CAN_AddTxMessage Ok.");
        // uint32_t err = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
        // log_i("CAN Error: 0x%08lX\r\n", err);
    } 
    else 
    {
        // uint32_t err = HAL_CAN_GetError(&hcan1);
        // log_i("CAN Error: 0x%08lX\r\n", err);
        uint32_t err = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
        log_i("CAN Error: 0x%08lX\r\n", err);
    }
}
