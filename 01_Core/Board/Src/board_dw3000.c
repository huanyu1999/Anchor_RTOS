#include "board_dw3000.h"
#include "gpio.h"
#include "port.h"

static volatile uint32_t signalResetDone;

static gpio_config_t dw3000_boardParam[] = {
    {.gpio_port = Dw3000_RSTn_GPIO_Port, .clk_port = GPIO_PORT_C, .gpio_pin = Dw3000_RSTn_Pin, .gpio_mode = GPIO_MODE_OUTPUT_OD, .gpio_pull = GPIO_NOPULL, .gpio_speed = NULL},
    {.gpio_port = Dw3000_IRQ_GPIO_Port,  .clk_port = GPIO_PORT_A, .gpio_pin = Dw3000_IRQ_Pin,  .gpio_mode = GPIO_MODE_IT_RISING, .gpio_pull = GPIO_PULLDOWN, .gpio_speed = NULL},
};

static exti_irq_t exti_config[] = {
    {.irq_name = Dw3000_IRQ_EXTI_IRQn, .irq_priority = 4}
};

static void setup_DW3000RSTnIRQ(int enable);

void board_dw3000Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    drv_gpioInit(&dw3000_boardParam[dw3000_reset]);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStructure.Pin = Dw3000_IRQ_Pin;
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(Dw3000_IRQ_GPIO_Port, &GPIO_InitStructure);
}

void board_dw3000IRQInit(void)
{
    drv_extiInit(&exti_config[0]);
}

void board_dw3000SetSignalReset(void)
{
    signalResetDone = 1;
}

/** @fn      board_dw3000Rst
  * @brief   DW_RESET pin on DW3000 has 2 functions
  * @note    In general it is output, but it also can be used to reset the digital
  *          part of DW3000 by driving this pin low.
  *          Note, the DW_RESET pin should not be driven high externally.
  */
void board_dw3000Rst(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // Enable GPIO used for DW3000 reset as open collector output
    GPIO_InitStruct.Pin   = Dw3000_RSTn_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Dw3000_RSTn_GPIO_Port, &GPIO_InitStruct);

    // drive the RSTn pin low
    HAL_GPIO_WritePin(Dw3000_RSTn_GPIO_Port, Dw3000_RSTn_Pin, GPIO_PIN_RESET);

    usleep(1);

    // put the pin back to output open-drain (not active)
    setup_DW3000RSTnIRQ(0);

    Sleep(2);
}

/** @fn      setup_DW3000RSTnIRQ
  * @brief   setup the DW_RESET pin mode
  *          0 - output Open collector mode
  *          !0 - input mode with connected EXTI IRQ
  */
static void setup_DW3000RSTnIRQ(int enable)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(enable)
    {
        // Enable GPIO used as DECA RESET for interrupt
        GPIO_InitStruct.Pin = Dw3000_RSTn_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(Dw3000_RSTn_GPIO_Port, &GPIO_InitStruct);

        HAL_NVIC_EnableIRQ(EXTI4_IRQn);
        HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
    }
    else
    {
        HAL_NVIC_DisableIRQ(EXTI4_IRQn);

        // put the pin back to tri-state ... as
        // output open-drain (not active)
        GPIO_InitStruct.Pin   = Dw3000_RSTn_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(Dw3000_RSTn_GPIO_Port, &GPIO_InitStruct);
        HAL_GPIO_WritePin(Dw3000_RSTn_GPIO_Port, Dw3000_RSTn_Pin, GPIO_PIN_SET);
    }
}
