/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 6 de jul de 2022
 *      Author: joaobortotcadore
 */

#include "stm32f407xx_gpio_driver.h"

/**
 * @fn void GPIO_PeriClockControl(GPIO_RegDef_t*, uint8_t)
 * @brief This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in] pGPIOx : base address of the GPIO peripheral
 * @param[in] EnOrDi : ENABLE or DISABLE macros
 *
 * @return none
 * @note none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if(pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if(pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if(pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
        else if(pGPIOx == GPIOI)
        {
            GPIOI_PCLK_EN();
        }
        else if(pGPIOx == GPIOJ)
        {
            GPIOJ_PCLK_EN();
        }
        else if(pGPIOx == GPIOK)
        {
            GPIOK_PCLK_EN();
        }
    }
    else
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        else if(pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }
        else if(pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DI();
        }
        else if(pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DI();
        }
        else if(pGPIOx == GPIOI)
        {
            GPIOI_PCLK_DI();
        }
        else if(pGPIOx == GPIOJ)
        {
            GPIOJ_PCLK_DI();
        }
        else if(pGPIOx == GPIOK)
        {
            GPIOK_PCLK_DI();
        }
    }
}

/*
 * Init and De-init
 */
/**
 * @fn void GPIO_Init(GPIO_Handle_t*)
 * @brief The user application must create a variable of GPIO_Handle_t,
 *  then pass the pointer to this function GPIO_Init to initialize Port
 *  and Pin
 *
 * @param pGPIOHandle
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    //1.configure the mode of gpio pin

    //2.configure the speed

    //3.configure the pupd settings

    //4.configure the optype

    //5.configure the alternate function
}
/**
 * @fn void GPIO_DeInit(GPIO_Handle_t*)
 * @brief Deinitialize means send the register to your reset state/reset value,
 * we can use the peripheral reset register in RCC section to do this in one shot
 *
 * @param pGPIOx
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}

/*
 * Data read/write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) /* return uint8_t because return is 1 or 0 / SET or RESET */
{
	return 1;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) /* uint16_t because we have 16 pins */
{
	return 0;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) /* Value is SET or RESET */
{

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/*
 * IRQ Configuration and ISR handling | IRQ - Interrupt ReQuest, ISR - Interrupt Status Register
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi) //to enable it, to set priority, etc
{

}
void GPIO_IRQHandling(uint8_t PinNumber) //when an interrupt occurs, this function is responsible to process
{

}
