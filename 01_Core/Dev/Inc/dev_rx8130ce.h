#ifndef __DEV_RX8130CE_H__
#define __DEV_RX8130CE_H__

#include <stdint.h>

/* rx8130ce register definitions */
#define RX8130_REG_SEC		0x10
#define RX8130_REG_MIN		0x11
#define RX8130_REG_HOUR		0x12
#define RX8130_REG_WDAY		0x13
#define RX8130_REG_MDAY		0x14
#define RX8130_REG_MONTH	0x15
#define RX8130_REG_YEAR		0x16

#define RX8130_REG_ALMIN	0x17
#define RX8130_REG_ALHOUR	0x18
#define RX8130_REG_ALWDAY	0x19
#define RX8130_REG_TCOUNT0	0x1A
#define RX8130_REG_TCOUNT1	0x1B
#define RX8130_REG_EXT		0x1C
#define RX8130_REG_FLAG		0x1D
#define RX8130_REG_CTRL0	0x1E
#define RX8130_REG_CTRL1	0x1F

#define RX8130_REG_END		0x23

/* Extension Register (1Ch) bit positions */ 
#define RX8130_BIT_EXT_TSEL		(7 << 0)
#define RX8130_BIT_EXT_WADA		(1 << 3)
#define RX8130_BIT_EXT_TE		(1 << 4)
#define RX8130_BIT_EXT_USEL		(1 << 5)
#define RX8130_BIT_EXT_FSEL		(3 << 6)

/* Flag Register (1Dh) bit positions */ 
#define RX8130_BIT_FLAG_VLF		(1 << 1)
#define RX8130_BIT_FLAG_AF		(1 << 3)
#define RX8130_BIT_FLAG_TF		(1 << 4)
#define RX8130_BIT_FLAG_UF		(1 << 5)

/* Control 0 Register (1Eh) bit positions */ 
#define RX8130_BIT_CTRL_TSTP	(1 << 2)
#define RX8130_BIT_CTRL_AIE		(1 << 3)
#define RX8130_BIT_CTRL_TIE		(1 << 4)
#define RX8130_BIT_CTRL_UIE		(1 << 5)
#define RX8130_BIT_CTRL_STOP	(1 << 6)
#define RX8130_BIT_CTRL_TEST	(1 << 7)

/* Control 1 Register (1Fh) bit positions */
#define RX8130_BIT_CTRL_INIEN   (1 << 4)
#define RX8130_BIT_CTRL_CHGEN   (1 << 5)

typedef struct {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t week;
} rx8130ce_time_t;

void dev_rx8130ceInit(void);
void dev_rx8130ceSoftInit(void);
void dev_rx8130ceSetTimeTest(void);
void dev_rx8130ceWritebuf(uint16_t devAddr_w, uint8_t memAddr, uint8_t* data, uint16_t length);
void dev_rx8130ceRead(uint16_t devAddr, uint8_t memAddr, uint8_t *buf, uint16_t length);
void dev_rx8130ceSetDateTime(volatile rx8130ce_time_t *time);
uint8_t*  dev_rx8130ceGetDateTime(volatile rx8130ce_time_t *time);
uint8_t dev_rx8130ceBcd2Bin(uint8_t bcd);
uint8_t dev_rx8130ceBin2Bcd(uint8_t bin);


#endif

