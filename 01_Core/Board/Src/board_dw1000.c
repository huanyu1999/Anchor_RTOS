#include "board_dw1000.h"
#include "deca_port.h"
#include "gpio.h"
#include "spi.h"

static volatile uint32_t signalResetDone;
static gpio_config_t dw1000_boardParam[] = {
    {.gpio_port = Dw1000_RSTn_GPIO_Port, .clk_port = GPIO_PORT_C, .gpio_pin = Dw1000_RSTn_Pin, .gpio_mode = GPIO_MODE_OUTPUT_OD, .gpio_pull = GPIO_NOPULL, .gpio_speed = NULL},
    {.gpio_port = Dw1000_IRQ_GPIO_Port, .clk_port = GPIO_PORT_A, .gpio_pin = Dw1000_IRQ_Pin, .gpio_mode = GPIO_MODE_IT_RISING, .gpio_pull = GPIO_PULLDOWN, .gpio_speed = NULL},
};

static exti_irq_t exti_config[exti_num] = {
    {.irq_name = Dw1000_IRQ_EXTI_IRQn, .irq_priority = 4}
};

static void setup_DW1000RSTnIRQ(int enable);

void board_dw1000Init(void)
{   
    // 使能中断脚和reset脚
    GPIO_InitTypeDef GPIO_InitStructure;
    
    drv_gpioInit(&dw1000_boardParam[dw1000_reset]);
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStructure.Pin = GPIO_PIN_2;
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void board_dw1000IRQInit(void)
{
    // 使能dw1000中断
    drv_extiInit(&exti_config[0]);
}

/** @fn      port_wakeup_IC
  * @brief   "slow" waking up of DW1000 using DW_CS only
  */
void board_dw1000SlowWakeup(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    Sleep(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    Sleep(7);                   // wait 7ms for DW1000 XTAL to stabilise
}

void board_dw1000FastWakeup(void)
{
    #define WAKEUP_TMR_MS   (10)

    uint32_t x = 0;
    uint32_t timestamp = HAL_GetTick();    // protection

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

void board_dw1000SetSignalReset(void)
{
    signalResetDone = 1;
}

/** @fn      reset_DW1000
  * @brief   DW_RESET pin on DW1000 has 2 functions
  * @note    RESET pin. Active Low Output, may be pulled low by external open drain driver to 
  *          reset the DW1000
  *      In general it is output, but it also can be used to reset the digital
  *      part of DW1000 by driving this pin low.
  *      Note, the DW_RESET pin should not be driven high externally.
  */
void board_dw1000Rst(void)
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

/** @fn      setup_DW1000RSTnIRQ
  * @brief	setup the DW_RESET pin mode
  * 			0 - output Open collector mode
  * 			!0 - input mode with connected EXTI0 IRQ
  */
static void setup_DW1000RSTnIRQ(int enable)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(enable)
    {
        // Enable GPIO used as DECA RESET for interrupt
        GPIO_InitStruct.Pin = Dw1000_RSTn_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(Dw1000_RSTn_GPIO_Port, &GPIO_InitStruct);

        HAL_NVIC_EnableIRQ(EXTI4_IRQn);             //pin #4 -> EXTI #4
        HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
    }
    else
    {
        HAL_NVIC_DisableIRQ(EXTI4_IRQn);            //EXTI #4 -> pin  

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
