/*! ----------------------------------------------------------------------------
 * @file	port.c
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "deca_port.h"
#include "deca_device_api.h"
#include "stm32f4xx_hal_conf.h"
#include "main.h"

extern          SPI_HandleTypeDef hspi1;
static volatile uint32_t signalResetDone;
volatile        int32_t sys_time_diff = 0;

 
/* @fn 	  portGetTickCnt
 * @brief wrapper for to read a SysTickTimer, which is incremented with
 * 		  CLOCKS_PER_SEC frequency.
 * 		  The resolution of time32_incr is usually 1/1000 sec.
 * */
// __INLINE unsigned long
// portGetTickCnt(void)
unsigned long portGetTickCnt(void)
{
	return HAL_GetTick() - sys_time_diff;
	//return HAL_GetTick();
}


/* @fn	  usleep
 * @brief precise usleep() delay
 * */
#pragma GCC optimize ("O0")
int usleep(unsigned long usec)
{
    int i,j;
#pragma GCC ivdep
    for(i=0;i<usec;i++)
    {
#pragma GCC ivdep
        for(j=0;j<2;j++)
        {
            __NOP();
            __NOP();
        }
    }
    return 0;
}


/* @fn 	  Sleep
 * @brief Sleep delay in ms using SysTick timer
 * */
__INLINE void
Sleep(uint32_t x)
{
    HAL_Delay(x);
}



/* @fn 	  peripherals_init
 * */
int peripherals_init (void)
{
    /* All has been initialized in the CubeMx code, see main.c */
    return 0;
}


/**
  * @brief  Checks whether the specified EXTI line is enabled or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The "enable" state of EXTI_Line (SET or RESET).
  */
ITStatus EXTI_GetITEnStatus(uint32_t x)
{
    return ((NVIC->ISER[(((uint32_t)x) >> 5UL)] &\
            (uint32_t)(1UL << (((uint32_t)x) & 0x1FUL)) ) == (uint32_t)RESET)?(RESET):(SET);
}


/* @fn      reset_DW1000
 * @brief   DW_RESET pin on DW1000 has 2 functions
 *      In general it is output, but it also can be used to reset the digital
 *      part of DW1000 by driving this pin low.
 *      Note, the DW_RESET pin should not be driven high externally.
 * */
void reset_DW1000(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // Enable GPIO used for DW1000 reset as open collector output
    GPIO_InitStruct.Pin   = Dw1000_RSTn_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Dw1000_RSTn_GPIO_Port, &GPIO_InitStruct);

    // drive the RSTn pin low
    HAL_GPIO_WritePin(Dw1000_RSTn_GPIO_Port, Dw1000_RSTn_Pin, GPIO_PIN_RESET);

    usleep(1);
    
    // put the pin back to output open-drain (not active)
    setup_DW1000RSTnIRQ(0);

    Sleep(2);

}

/* @fn      setup_DW1000RSTnIRQ
 * @brief	setup the DW_RESET pin mode
 * 			0 - output Open collector mode
 * 			!0 - input mode with connected EXTI0 IRQ
 * */
void setup_DW1000RSTnIRQ(int enable)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(enable)
    {
        // Enable GPIO used as DECA RESET for interrupt
        GPIO_InitStruct.Pin = Dw1000_RSTn_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(Dw1000_RSTn_GPIO_Port, &GPIO_InitStruct);

        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);             //pin #13 -> EXTI #13
        HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
    }
    else
    {
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);            //pin #13 -> EXTI #13

        // put the pin back to tri-state ... as 
        // output open-drain (not active)
        GPIO_InitStruct.Pin   = Dw1000_RSTn_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(Dw1000_RSTn_GPIO_Port, &GPIO_InitStruct);
        HAL_GPIO_WritePin(Dw1000_RSTn_GPIO_Port, Dw1000_RSTn_Pin, GPIO_PIN_SET);
    }
}

/* @fn		switch_is_on
 * @brief	check the switch status.
 * 			when switch (S1) is 'on' the pin is low
 * @return  1 if ON and 0 for OFF
 * */
/*
int switch_is_on(uint16_t GPIOpin)
{
    return ((GPIO_ReadInputDataBit(SW_GPIO, GPIOpin))?(0):(1));
}
*/

/*
void pa_pwr_on(void)
{
    HAL_GPIO_WritePin(PA_PWR_EN_GPIO_Port, PA_PWR_EN_Pin, GPIO_PIN_SET);
}

void pa_pwr_off(void)
{
    HAL_GPIO_WritePin(PA_PWR_EN_GPIO_Port, PA_PWR_EN_Pin, GPIO_PIN_RESET);
}

void motor_on(void)
{
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
}

void motor_off(void)
{
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
}
*/

/*
void expr_on(void)
{
	//HAL_GPIO_TogglePin(EXPR_GPIO_Port, EXPR_Pin);
	HAL_GPIO_WritePin(EXPR_GPIO_Port, EXPR_Pin, GPIO_PIN_SET);
}
 */

/*
void expr_off(void)
{
	//HAL_GPIO_TogglePin(EXPR_GPIO_Port, EXPR_Pin);
	HAL_GPIO_WritePin(EXPR_GPIO_Port, EXPR_Pin, GPIO_PIN_RESET);
}
 */


/* @fn      port_wakeup_IC
 * @brief   "slow" waking up of DW1000 using DW_CS only
 **/
void port_wakeup_IC(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    Sleep(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    Sleep(7);                   //wait 7ms for DW1000 XTAL to stabilise
}

/* @fn      port_wakeup_IC_fast
 * @brief   waking up of DW1000 using DW_CS and DW_RESET pins.
 *          The DW_RESET signalling that the DW1000 is in the INIT state.
 *          the total fast wakeup takes ~2.2ms and depends on crystal startup time
 **/

void port_wakeup_IC_fast(void)
{
    #define WAKEUP_TMR_MS   (10)

    uint32_t x = 0;
    uint32_t timestamp = HAL_GetTick();    //protection

    setup_DW1000RSTnIRQ(0);         //disable RSTn IRQ
    signalResetDone = 0;            //signalResetDone connected to RST_PIN_IRQ
    setup_DW1000RSTnIRQ(1);         //enable RSTn IRQ
    port_SPIx_clear_chip_select();  //CS low

    //need to poll to check when the DW1000 is in the IDLE, the CPLL interrupt is not reliable
    //when RSTn goes high the DW1000 is in INIT, it will enter IDLE after PLL lock (in 5 us)

    while((signalResetDone == 0) && ((HAL_GetTick() - timestamp) < WAKEUP_TMR_MS))
    {
        x++;    //when DW1000 will switch to an IDLE state RSTn pin will high
    }

    setup_DW1000RSTnIRQ(0);         //disable RSTn IRQ
    port_SPIx_set_chip_select();    //CS high

    //it takes ~35us in total for the DW1000 to lock the PLL, download AON and go to IDLE state
    usleep(35);
}



/* @fn      port_set_dw1000_slowrate
 * @brief   set 2.25MHz
 *          note: hspi1 is clocked from 72MHz
 * */
void port_set_dw1000_slowrate(void)
{
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    HAL_SPI_Init(&hspi1);
}

/* @fn      port_set_dw1000_fastrate
 * @brief   set 18MHz
 *          note: hspi1 is clocked from 32MHz
 * */
void port_set_dw1000_fastrate(void)
{
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    HAL_SPI_Init(&hspi1);
}



/* @fn		HAL_GPIO_EXTI_Callback
 * @brief	IRQ HAL call-back for all EXTI configured lines
 * 			i.e. DW_RESET_Pin and DW_IRQn_Pin
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == Dw1000_RSTn_Pin)
    {
        signalResetDone = 1;
    }
    else if (GPIO_Pin == Dw1000_IRQ_Pin)
    {
        process_deca_irq();
    }
}

/* @fn		process_deca_irq
 * @brief	main call-back for processing of DW1000 IRQ
 * 			it re-enters the IRQ routing and processes all events.
 * 			After processing of all events, DW1000 will clear the IRQ line.
 * */
__INLINE void process_deca_irq(void)
{
    while(port_CheckEXT_IRQ() != 0)
    {

        dwt_isr();

    } //while DW1000 IRQ line active
}


/* @fn		port_DisableEXT_IRQ
 * @brief	wrapper to disable DW_IRQ pin IRQ
 * 			in current implementation it disables all IRQ from lines 5:9
 * */
__INLINE void port_DisableEXT_IRQ(void)
{
    NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn);
}

/* @fn		port_EnableEXT_IRQ
 * @brief	wrapper to enable DW_IRQ pin IRQ
 * 			in current implementation it enables all IRQ from lines 5:9
 * */
__INLINE void port_EnableEXT_IRQ(void)
{
    NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn);
}


/* @fn		port_GetEXT_IRQStatus
 * @brief	wrapper to read a DW_IRQ pin IRQ status
 * */
__INLINE uint32_t port_GetEXT_IRQStatus(void)
{
    return EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn);
}


/* @fn		port_CheckEXT_IRQ
 * @brief	wrapper to read DW_IRQ input pin state
 * */
__INLINE uint32_t port_CheckEXT_IRQ(void)
{
    return HAL_GPIO_ReadPin(DECAIRQ_GPIO, DECAIRQ);
}






 
