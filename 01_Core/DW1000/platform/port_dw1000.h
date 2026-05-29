/*! ----------------------------------------------------------------------------
 * @file	port.h
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */


#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include "compiler.h"

// #include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal.h"

/* SPI1 read temp buffer. */
#define BUFFLEN     (64)   //(4096+128)
#define BUF_SIZE    (64)

typedef uint64_t        uint64;
typedef int64_t         int64;

extern volatile int32_t sys_time_diff;

#define DECAIRQ_EXTI_IRQn           (EXTI15_10_IRQn)

#define DW1000_RSTn                 Dw1000_RSTn_Pin
#define DW1000_RSTn_GPIO            Dw1000_RSTn_GPIO_Port

#define DECAIRQ                     Dw1000_IRQ_Pin
#define DECAIRQ_GPIO                Dw1000_IRQ_GPIO_Port

/* NSS pin is SW controllable */
#define port_SPIx_set_chip_select()         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define port_SPIx_clear_chip_select()       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)


void Sleep(uint32_t Delay);
unsigned long portGetTickCnt(void);

//int switch_is_on(uint16_t GPIOpin);

void port_wakeup_IC(void);
void port_wakeup_IC_fast(void);
void port_set_signalReset(void);

void port_set_dw1000_slowrate(void);
void port_set_dw1000_fastrate(void);

void process_deca_irq(void);

int  peripherals_init(void);
int usleep(unsigned long usec);

ITStatus EXTI_GetITEnStatus(uint32_t x);

uint32_t port_GetEXT_IRQStatus(void);
uint32_t port_CheckEXT_IRQ(void);
void port_DisableEXT_IRQ(void);
void port_EnableEXT_IRQ(void);
extern uint32_t HAL_GetTick(void);
#ifdef __cplusplus
}
#endif


#endif 

