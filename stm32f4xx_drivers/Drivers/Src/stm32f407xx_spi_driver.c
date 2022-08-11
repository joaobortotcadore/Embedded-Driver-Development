/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 2 de ago de 2022
 *      Author: joaobortotcadore
*/

#include "stm32f407xx_spi_driver.h"

/* Peripheral Clock Setup */
/**
 * @fn void SPI_PeriClockControl(SPI_RegDef_t*, uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param pGPIOx
 * @param EnOrDi
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if(pSPIx == SPI2)
        {
            SPI3_PCLK_EN();
        }
        else if(pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
//        if(pSPIx == SPI4)
//        {
//            SPI4_PCLK_EN();
//        }
    }
    else
    {
        // if(pSPIx == SPI1)
        // {
        //     SPI1_PCLK_DI();
        // }
        // if(pSPIx == SPI2)
        // {
        //     SPI3_PCLK_DI();
        // }
        // if(pSPIx == SPI3)
        // {
        //     SPI3_PCLK_DI();
        // }
//        if(pSPIx == SPI4)
//        {
//            SPI4_PCLK_DI();
//        }
    }
}

/* Init and De-init */
/**
 * @fn void SPI_Init(SPI_Handle_t*)
 * @brief
 *
 * @pre
 * @post
 * @param pSPIHandle
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//first lets configure the SPI_CR1 register
	uint32_t tempreg = 0;

	//enable the peripheral clock here, because users always forget to enabled it
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);

	}

	//3. Configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
}
/**
 * @fn void SPI_DeInit(SPI_RegDef_t*)
 * @brief
 *
 * @pre
 * @post
 * @param pSPIx
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
 //TODO
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/* Data Send and Receive */
/**
 * @fn void SPI_SendData(SPI_RegDef_t*, uint8_t*, uint32_t)
 * @brief
 * @param pSPIx
 * @param pTxBuffer
 * @param Len
 * @note This is blocking call because we pooling for the TXE_FLAG == SET, and we stay in while loop until LEN > 0
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set == 1: Tx buffer empty
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET); //while( ! (pSPIx->SR & (1 << 1) ) );

		//2. Check the DFF bit in CR1
		if( (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
		{
			//16 bit DFF
			//1. load the data in to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}
/**
 * @fn void SPI_ReceiveData(SPI_RegDef_t*, uint8_t*, uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param pSPIx
 * @param pRxBuffer
 * @param Len
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/* IRQ Configuration and ISR handling | IRQ - Interrupt ReQuest, ISR - Interrupt Status Register */
/**
 * @fn void SPI_IRQInterruptConfig(uint8_t, uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param IRQNumber
 * @param EnOrDi
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
/**
 * @fn void SPI_IRQPriorityConfig(uint8_t, uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param IRQNumber
 * @param IRQPriority
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
/**
 * @fn void SPI_IRQHandling(SPI_Handle_t*)
 * @brief
 *
 * @pre
 * @post
 * @param pHandle
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle);
/**
 * @fn void SPI_PeripheralControl(SPI_RegDef_t*, uint8_t)
 * @brief
 *
 * @param pSPIx
 * @param EnOrDi
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE); //enable peripheral
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE); //disable peripheral
	}
}
/**
 * @fn void SPI_SSIConfig(SPI_RegDef_t*, uint8_t)
 * @brief
 *
 * @param pSPIx
 * @param EnOrDi
 * @note Section 40, lesson 150
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
