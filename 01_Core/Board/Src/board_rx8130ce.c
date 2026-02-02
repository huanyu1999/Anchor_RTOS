#include "board_rx8130ce.h"
#include "drv_i2c.h"

static void board_rx8130ceIOInit(void)
{
    // GPIO_InitTypeDef GPIO_InitStructure = {0};
    // 使能RST引脚，INT引脚
}

void board_rx8130ceI2cInit(void)
{
    drv_i2cInit();
}

void board_rx8130ceInit(void)
{
    board_rx8130ceIOInit();         // 暂时先不实现
    board_rx8130ceI2cInit();
}

void board_rx8130ceBufRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint16_t buf_size)
{
    drv_i2cMemRead(addr, reg, buf, buf_size);
}

void board_rx8130ceBufWrite(uint8_t addr, uint8_t reg, uint8_t* val, uint16_t buf_size)
{
    drv_i2cMemWrite(addr, reg, val, buf_size);
}

void board_rx8130ceHardRst(void)
{
}
