#ifndef __BOARD_W5500_H__
#define __BOARD_W5500_H__

#define W5500_RST_PORT    GPIOB
#define W5500_RST_PIN     GPIO_PIN_4

#define W5500_INT_PORT    GPIOA
#define W5500_INT_PIN     GPIO_PIN_10
#define W5500_EXTI_IRQ    EXTI15_10_IRQn       // 跟UWB中断冲突，原理图需要协调修改 

typedef enum {
    W5500_RST = 0,
    W5500_INT,
    W5500_GPIO_NUM
} w5500_gpio_e;


void board_w5500Init(void);
void board_w5500InterfaceInit(void);
void board_w5500RstIntInit(void);
void board_w5500Reset(void);
void board_w5500ChipSel(void);
void board_w5500ChipDeSel(void);
void board_w5500RstHigh(void);
void board_w5500RstLow(void);
void board_w5500WriteByte(uint8_t data);
uint8_t board_w5500ReadByte(void);
void board_w5500WriteBytes(uint8_t* data, uint16_t length);
void board_w5500ReadBytes(uint8_t* data, uint16_t length);
void board_w5500CallbackReg(void);
#endif 
