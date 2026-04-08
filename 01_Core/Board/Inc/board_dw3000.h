#ifndef __BOARD_DW3000_H__
#define __BOARD_DW3000_H__

#include <stm32f4xx_hal.h>

/* DW3000 核心引脚定义 (与 DW1000 共用相同引脚) */
#define Dw3000_IRQ_Pin          GPIO_PIN_2
#define Dw3000_IRQ_GPIO_Port    GPIOA
#define Dw3000_IRQ_EXTI_IRQn    EXTI2_IRQn

#define Dw3000_RSTn_Pin         GPIO_PIN_4
#define Dw3000_RSTn_GPIO_Port   GPIOC

#define Dw3000_NSS_Pin          GPIO_PIN_4
#define Dw3000_NSS_GPIO_Port    GPIOA

/* TODO: 确认实际的 WAKEUP 和 MEAS_TIME 引脚 */
#define Dw3000_WAKEUP_Pin       GPIO_PIN_0
#define Dw3000_WAKEUP_GPIO_Port GPIOB

#define Dw3000_MEAS_TIME_Pin       GPIO_PIN_0
#define Dw3000_MEAS_TIME_GPIO_Port GPIOC

typedef enum {
    dw3000_reset = 0,
    dw3000_interrupt,
    dw3000_boardNUM
} dw3000Board_enum_t;

typedef enum {
    dw3000_exti,
    dw3000_exti_num
} dw3000Irq_enum_t;

void board_dw3000Init(void);
void board_dw3000IRQInit(void);
void board_dw3000Rst(void);
void board_dw3000SetSignalReset(void);

#endif
