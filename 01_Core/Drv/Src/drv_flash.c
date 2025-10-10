#include "drv_flash.h"

#include "main.h"

static FLASH_EraseInitTypeDef EraseInitStruct;

uint8_t drv_flashInit(void) 
{
    HAL_FLASH_Unlock();

}

void drv_flashWrite(uint32_t address, uint8_t data, uint8_t data_size)
{
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, data);
    HAL_FLASH_Unlock();
}

uint8_t drv_flashRead(uint32_t address)
{
    return *(uint32_t*)address;
}

uint8_t drv_flashEreaseSector(uint32_t numOfSector, )
{
    EraseInitStruct.Banks = 
    EraseInitStruct.NbSectors = 
    EraseInitStruct.Sector = 
    EraseInitStruct.TypeErase = 
    EraseInitStruct.VoltageRange 


}