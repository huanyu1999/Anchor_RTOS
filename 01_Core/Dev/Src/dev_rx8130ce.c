#include "dev_rx8130ce.h"
#include "board_rx8130ce.h"
#include "main.h"


/* define rx8130ce device address */
#define RX8130CE_ADDR 0x64

void dev_rx8130ceInit(void)
{
    board_rx8130ceInit();
    dev_rx8130ceSoftInit();
}

void dev_rx8130ceSoftInit(void)
{
    // uint8_t ctrl[3];
    
    // // get current extension, flag, control register values
    // dev_rx8130ceRead(RX8130CE_WRITE_ADDR, RX8130CE_READ_ADDR, RX8130_REG_EXT, &ctrl, 3);

    // //set extension register, TE to 0, FSEL1-0 and TSEL2-0 for desired frequency
	// ctrl[0] &= ~RX8130_BIT_EXT_TE;			//set TE to 0
	// ctrl[0] &= ~RX8130_BIT_EXT_FSEL;		//set to 0 (off) for this case
	// ctrl[0] |= 0x02;						//set TSEL for 1Hz 
}

void dev_rx8130ceSetDefaultTime(void)
{
    rx8130ce_time_t now = {
        .year = 99,
        .month = 1,
        .day = 1,
        .hours = 1, 
        .minutes = 0,
        .seconds = 0,
        .week = 1,
    };
    dev_rx8130ceSetDateTime(&now);
}

/**
  * @brief  send a byte to RTC chip register
  * @param  
  * @retval None
  */
void dev_rx8130ceWritebuf(uint16_t devAddr_w, uint8_t memAddr, uint8_t* data, uint16_t length)
{
    //  transmits the RX8130CE's slave address with the R/W bit set to write mode
    // Check for ACK signal from RX8130CE.
    // transmits write address to RX8130CE. 
    // Check for ACK signal from RX8130CE.
    // transfers write data to the address specified behind the address 
    board_rx8130ceBufWrite(devAddr_w, memAddr, data, length);    
}

/**
  * @brief  read a byte from RTC chip register
  * @param  
  * @retval None
  */
void dev_rx8130ceRead(uint16_t devAddr, uint8_t memAddr, uint8_t *buf, uint16_t length)
{
    // transmits the RX8130CE's slave address with the R/W bit set to write mode. 
    // transfers address for reading from RX8130CE. 
    // board_rx8130ceBufWrite1(devAddr, &memAddr, 1);

    // transfers RESTART condition [Sr] (in which case, CPU does not transfer a STOP condition [P]). 
    // transfers RX8130'CEs slave address with the R/W bit set to read mode. 
    // receive data
    // board_rx8130ceBufRead1(devAddr, buf, length);
    board_rx8130ceBufRead(devAddr, memAddr, buf, length);
}

void dev_rx8130ceSetDateTime(volatile rx8130ce_time_t *time)
{
    uint8_t ctrl;

    // enable battery switch
    dev_rx8130ceRead(RX8130CE_ADDR, RX8130_REG_CTRL1, &ctrl, 1);
    ctrl = ctrl | RX8130_BIT_CTRL_INIEN;
    dev_rx8130ceWritebuf(RX8130CE_ADDR, RX8130_REG_CTRL1, &ctrl, 1);

    //set STOP bit before changing clock/calendar
    dev_rx8130ceRead(RX8130CE_ADDR, RX8130_REG_CTRL0, &ctrl, 1);
    ctrl = ctrl | RX8130_BIT_CTRL_STOP;
    dev_rx8130ceWritebuf(RX8130CE_ADDR, RX8130_REG_CTRL0, &ctrl, 1);


    uint8_t tempData[7] = { dev_rx8130ceBin2Bcd(time->seconds), dev_rx8130ceBin2Bcd(time->minutes),
                            dev_rx8130ceBin2Bcd(time->hours), dev_rx8130ceBin2Bcd(time->week),
                            dev_rx8130ceBin2Bcd(time->day), dev_rx8130ceBin2Bcd(time->month),
                            dev_rx8130ceBin2Bcd(time->year) };

    // 写入设定的时间
    dev_rx8130ceWritebuf(RX8130CE_ADDR, RX8130_REG_SEC, tempData, 7);

    // clear STOP bit after changing clock/calendar
    dev_rx8130ceRead(RX8130CE_ADDR, RX8130_REG_CTRL0, &ctrl, 1);
    ctrl = ctrl & ~RX8130_BIT_CTRL_STOP;
    dev_rx8130ceWritebuf(RX8130CE_ADDR, RX8130_REG_CTRL0, &ctrl, 1);
}

uint8_t* dev_rx8130ceGetDateTime(volatile rx8130ce_time_t *time)
{
    static uint8_t temp_data[7];
    dev_rx8130ceRead(RX8130CE_ADDR, RX8130_REG_SEC, temp_data, 7);
    time->year  = dev_rx8130ceBcd2Bin(temp_data[6]);
    time->month = dev_rx8130ceBcd2Bin(temp_data[5]);
    time->day   = dev_rx8130ceBcd2Bin(temp_data[4]);
    time->hours = dev_rx8130ceBcd2Bin(temp_data[2]);
    time->minutes = dev_rx8130ceBcd2Bin(temp_data[1]);
    time->seconds = dev_rx8130ceBcd2Bin(temp_data[0]);

    time->week = 0;
    while ((temp_data[3] & 1) == 0)
    {
        time->week++;
        temp_data[3] >>= 1;
    }

    return temp_data;
}

uint8_t dev_rx8130ceBcd2Bin(uint8_t bcd)
{
    uint8_t dec = 10 * (bcd >> 4);
    dec += bcd & 0xF;
    return dec;
}

uint8_t dev_rx8130ceBin2Bcd(uint8_t bin)
{
    uint8_t low = 0;
    uint8_t high = 0;
    
    high = bin / 10;
    low = bin - (high * 10);
    
    return high << 4 | low;
}
