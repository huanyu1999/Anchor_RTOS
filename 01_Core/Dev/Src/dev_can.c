#include "dev.h"
#include "can.h"
#include "elog.h"
#include "usart.h"

CAN_TxHeaderTypeDef tx_header;


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
  * @brief  can发送一条数据，是否就是write device 操作,该函数有问题
  * @param  
  *         
  * @retval 
  */
void dev_canSendMsg(uint32_t extId, uint8_t* data, uint32_t length) 
{
    uint32_t TxMailbox = 0;
    
    tx_header.ExtId = extId;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = length;
    tx_header.TransmitGlobalTime = DISABLE;

    if(HAL_CAN_AddTxMessage(&hcan2, &tx_header, data, &TxMailbox) == HAL_OK)
    {
        // log_i("HAL_CAN_AddTxMessage Ok.");
        // 后续加入log写入，CAN发送成功
    } 
    else 
    {
        // log_i("");
    }
}

/**
  * @brief  
  * @param  
  *         
  * @retval 
  */
void dev_canPollingRxMsg(void)
{
    uint8_t recv_data[8];
    
    CAN_RxHeaderTypeDef *rxHeader;
    while(HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) != 0)
    {
        if(__HAL_CAN_GET_FLAG(&hcan2, CAN_FLAG_FOV0) !=RESET) 
        {
            
        }
        
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, rxHeader, recv_data);

        printf_use_dma( "ExtId ID:%d\n",rxHeader->ExtId);
        printf_use_dma( "CAN IDE:0x%x\n",rxHeader->IDE);
        printf_use_dma( "CAN RTR:0x%x\n",rxHeader->RTR);
        printf_use_dma( "CAN DLC:0x%x\n",rxHeader->DLC);
        printf_use_dma( "RECV DATA:");
        for(int i = 0; i < rxHeader->DLC; i++)
        {
            printf_use_dma( "0x%x ",recv_data[i]);
        }
    }
}
