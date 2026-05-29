/*! ----------------------------------------------------------------------------
 * @file    port.h
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2015-2020 (c) DecaWave Ltd, Dublin, Ireland.
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

#include <stm32f4xx_hal.h>
#include "board_dw3000.h"

/* DW IC IRQ (EXTI) handler type. */
typedef void (*port_dwic_isr_t)(void);

void port_set_dwic_isr(port_dwic_isr_t isr);

/****************************************************************************//**
 *
 *                              MACRO
 *
 *******************************************************************************/

#define DECAIRQ_EXTI_IRQn       (Dw3000_IRQ_EXTI_IRQn)

/* DW3000 driver 使用的引脚别名，映射到 board 层定义 */
#define DW_RSTn                     Dw3000_RSTn_Pin
#define DW_RSTn_GPIO                Dw3000_RSTn_GPIO_Port

#define DW_IRQn_Pin                 Dw3000_IRQ_Pin
#define DW_IRQn_GPIO_Port           Dw3000_IRQ_GPIO_Port
#define DECAIRQ                     Dw3000_IRQ_Pin
#define DECAIRQ_GPIO                Dw3000_IRQ_GPIO_Port

#define DW_RESET_Pin                Dw3000_RSTn_Pin
#define DW_RESET_GPIO_Port          Dw3000_RSTn_GPIO_Port

#define DW_NSS_Pin                  Dw3000_NSS_Pin
#define DW_NSS_GPIO_Port            Dw3000_NSS_GPIO_Port

#define DW_WAKEUP_Pin               Dw3000_WAKEUP_Pin
#define DW_WAKEUP_GPIO_Port         Dw3000_WAKEUP_GPIO_Port

#define DW_MEAS_TIME_Pin            Dw3000_MEAS_TIME_Pin
#define DW_MEAS_TIME_GPIO_Port      Dw3000_MEAS_TIME_GPIO_Port

/****************************************************************************//**
 *
 *                              MACRO function
 *
 *******************************************************************************/

/* NSS pin is SW controllable */
#define port_SPIx_set_chip_select()     HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET)
#define port_SPIx_clear_chip_select()   HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET)

/* Wakeup pin IO control */
#define SET_WAKEUP_PIN_IO_LOW     HAL_GPIO_WritePin(DW_WAKEUP_GPIO_Port, DW_WAKEUP_Pin, GPIO_PIN_RESET)
#define SET_WAKEUP_PIN_IO_HIGH    HAL_GPIO_WritePin(DW_WAKEUP_GPIO_Port, DW_WAKEUP_Pin, GPIO_PIN_SET)

/* Measurement time pin IO control */
#define SET_MEAS_PIN_IO_LOW       HAL_GPIO_WritePin(DW_MEAS_TIME_GPIO_Port, DW_MEAS_TIME_Pin, GPIO_PIN_RESET)
#define SET_MEAS_PIN_IO_HIGH      HAL_GPIO_WritePin(DW_MEAS_TIME_GPIO_Port, DW_MEAS_TIME_Pin, GPIO_PIN_SET)

#define WAIT_500uSEC    Sleep(1) /* This should be a delay of 500uSec at least */

/****************************************************************************//**
 *
 *                              port function prototypes
 *
 *******************************************************************************/

int usleep(unsigned int usec);

void Sleep(uint32_t Delay);
unsigned long portGetTickCnt(void);

void port_set_dw_ic_spi_slowrate(void);
void port_set_dw_ic_spi_fastrate(void);

void process_deca_irq(void);

int  peripherals_init(void);

ITStatus EXTI_GetITEnStatus(IRQn_Type x);

uint32_t port_GetEXT_IRQStatus(void);
uint32_t port_CheckEXT_IRQ(void);
void port_DisableEXT_IRQ(void);
void port_EnableEXT_IRQ(void);
extern uint32_t HAL_GetTick(void);

void wakeup_device_with_io(void);
void make_very_short_wakeup_io(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
