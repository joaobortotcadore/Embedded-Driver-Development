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
	uint32_t temp=0; //temp register
    //1.configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // 2* is because need 2 bits to configure MODERx
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //before SET any bit field, we need ensure it is cleared. 0x3 = 0b11
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else
	{
		//@todo for interrupt mode. Lesson 108,109
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. configure the FTSR (falling trigger selection register)
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. configure both FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding FTSR bit
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR (EXTICR EXTernal Interrupt Control Register)
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3. enable the exti interrupt delivery using IMR (Interrupt Mask Register)
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

    //2.configure the speed
	temp = 0;//reset temp
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // 2* is because need 2 bits to configure OSPEEDR
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //before SET any bit field, we need ensure it is cleared. 0x3 = 0b11
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;


    //3.configure the pupd settings
	temp = 0;//reset temp
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // 2* is because need 2 bits to configure PUPDR
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //before SET any bit field, we need ensure it is cleared. 0x3 = 0b11
	pGPIOHandle->pGPIOx->PUPDR |= temp;

    //4.configure the optype
	temp = 0;//reset temp
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // 1* is because need single bit to configure OTYPER
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //before SET any bit field, we need ensure it is cleared. 0x1 = 0b1
	pGPIOHandle->pGPIOx->OTYPER |= temp;

    //5.configure the alternate function
	temp = 0;//reset temp
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		/* AFR is Low and High (check in 8.4.9, 8.4.10)  */
		uint8_t temp1,temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; //indicates whether it will fall, if into the Low(AFR[0]) or into the High(AFR[1])
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8; //indicates whether it is from Low(0..7) or High((0..7)+8->(8..15))
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); //before SET any bit field, we need ensure it is cleared. 0x1 = 0b1
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); // 4* is because need 4 bits to configure AFR(L/H)
	}
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
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
	else if(pGPIOx == GPIOJ)
	{
		GPIOJ_REG_RESET();
	}
	else if(pGPIOx == GPIOK)
	{
		GPIOK_REG_RESET();
	}
}

/**
 * @fn uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t*, uint8_t)
 * @brief Data read
 * @param pGPIOx
 * @param PinNumber
 * @return 0 or 1
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) /* return uint8_t because return is 1 or 0 / SET or RESET */
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}
/**
 * @fn uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t*)
 * @brief Return all pins of selected GPIOx, x={A,B,C,D,E,F,G,H,I,J}
 * @param pGPIOx
 * @return 16 bits each bit represents a GPIOx port pin
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) /* uint16_t because we have 16 pins */
{
    uint16_t value;
    value = (uint16_t)pGPIOx->IDR;
    return value;
}

/**
 * @fn void GPIO_WriteToOutputPin(GPIO_RegDef_t*, uint8_t, uint8_t)
 * @brief This function write direct to the OutputDataRegister(ODR) at the
 * bitfield corresponding to the pin number

 * @param pGPIOx
 * @param PinNumber
 * @param Value: can be SET or RESET (1 or 0)
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) /* Value is SET or RESET */
{
    if(Value == GPIO_PIN_SET)
    {
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else
    {
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

/**
 * @fn void GPIO_WriteToOutputPort(GPIO_RegDef_t*, uint16_t)
 * @brief Copy Value to the OutputDataRegister(ODR), this function changes all
 * pins of a specific port at once
 *
 * @param pGPIOx
 * @param Value
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}
/**
 * @fn void GPIO_ToggleOutputPin(GPIO_RegDef_t*, uint8_t)
 * @brief Toggle pin output value
 * @param pGPIOx
 * @param PinNumber
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}

/**
 * @fn void GPIO_IRQConfig(uint8_t, uint8_t, uint8_t)
 * @brief IRQ Configuration and ISR handling | IRQ - Interrupt ReQuest, ISR - Interrupt Status Register
 *
 * @param IRQNumber
 * @param IRQPriority
 * @param EnOrDi - 0 has no effect, 1 to enable interrupt
 *
 * @note https://documentation-service.arm.com/static/5f2ac76d60a93e65927bbdc5?token=
 * In Table 4-2 NVIC register summary we need check:
 * ->Interrupt Set-enable Registers on page 4-4
 * ->Interrupt Clear-enable Registers on page 4-5
 * ->Interrupt Priority Registers on page 4-7
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi) //to enable it, to set priority, etc
{
	if(EnOrDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register - ISER (Interrupt Set-Enable Registers)
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}else if(IRQNumber >= 64 && IRQNumber <96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register - ICER (Interrupt Clear-Enable Registers)
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}else if(IRQNumber >= 64 && IRQNumber <96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
}
void GPIO_IRQHandling(uint8_t PinNumber) //when an interrupt occurs, this function is responsible to process
{

}
