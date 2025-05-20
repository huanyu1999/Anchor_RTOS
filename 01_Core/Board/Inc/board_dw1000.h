#ifndef __BOARD_DW1000_H__
#define __BOARD_DW1000_H__

#define Dw1000_IRQ_Pin          GPIO_PIN_13
#define Dw1000_IRQ_GPIO_Port    GPIOC
#define Dw1000_IRQ_EXTI_IRQn    EXTI15_10_IRQn

#define Dw1000_RSTn_Pin         GPIO_PIN_4
#define Dw1000_RSTn_GPIO_Port   GPIOC

typedef enum {
    dw1000_reset = 0,
    dw1000_interrupt,
    dw1000_boardNUM
} dw1000Board_enum_t;

// 如果还有其他的外部中断，就在这里添加
typedef enum {
    dw1000_exti,
    exti_num
} dw1000Irq_enum_t;

void board_dw1000Init(void);
void board_dw1000SlowWakeup(void);
void board_dw1000FastWakeup(void);
void board_dw1000SetSignalReset(void);
void board_dw1000Rst(void);

#endif
