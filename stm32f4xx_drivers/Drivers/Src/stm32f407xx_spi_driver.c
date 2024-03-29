/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 2 de ago de 2022
 *      Author: joaobortotcadore
*/

#include "stm32f407xx_spi_driver.h"

//private prototypes, helper functions that the user will not be able to call
static void SPI_TXE_IT_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_IT_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_ERR_IT_Handle(SPI_Handle_t *pSPIHandle);

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
            SPI2_PCLK_EN();
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until RXNE is set == 1: Rx buffer is not empty
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET); //while( ! (pSPIx->SR & (1 << 1) ) );

		//2. Check the DFF bit in CR1
		if( (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
		{
			//16 bit DFF
			//1. load the data from DR to the RxBuffer
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else
		{
			//8 bit DFF
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}
/**
 * @fn uint8_t SPI_SendDataIT(SPI_Handle_t*, uint8_t*, uint32_t)
 * @brief This API doesn't send anything, it just saves the pointers, length info and other data and just
 * activates the TXEIE interrupt and return. So it's a non-blocking API
 *
 * @param pSPIHandle
 * @param pTxBuffer
 * @param Len
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		// 1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		/* 2. Mark the SPI state as busy in transmission so that no other code can
		 * take over same SPI peripheral until transmission is over */
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		// 4. Data Transmission will be handled by the ISR code (will implemented later)
	}

	return state;
}
/**
 * @fn uint8_t SPI_ReceiveDataIT(SPI_Handle_t*, uint8_t*, uint32_t)
 * @brief
 *
 * @param pSPIHandle
 * @param pRxBuffer
 * @param Len
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		// 1. Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		/* 2. Mark the SPI state as busy in receive so that no other code can
		 * take over same SPI peripheral until receive data is over */
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable the RXEIE control bit to get interrupt whenever RXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		// 4. Data Transmission will be handled by the ISR code
	}

	return state;
}

/* IRQ Configuration and ISR handling | IRQ - Interrupt ReQuest, ISR - Interrupt Status Register */
/**
 * @fn void SPI_IRQInterruptConfig(uint8_t, uint8_t)
 * @brief
 *
 * @param IRQNumber
 * @param EnOrDi
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}
/**
 * @fn void SPI_IRQPriorityConfig(uint8_t, uint32_t)
 * @brief
 *
 * @param IRQNumber
 * @param IRQPriority
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}




/**
 * @fn void SPI_IRQHandling(SPI_Handle_t*)
 * @brief
 *
 * @param pHandle
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t	temp1, temp2;
	//first lets check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		SPI_TXE_IT_Handle(pHandle);
	}

	//check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//handle RXNE
		SPI_RXNE_IT_Handle(pHandle);
	}

	//check for OVR - OVerRun flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//handle RXNE
		SPI_OVR_ERR_IT_Handle(pHandle);
	}
}
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

/**
 * @fn void SPI_SSOEConfig(SPI_RegDef_t*, uint8_t)
 * @brief
 *
 * @param pSPIx
 * @param EnOrDi
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp; //to prevent error unused variable
}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

/* PRIVATE FUNCTIONS */

static void SPI_TXE_IT_Handle(SPI_Handle_t *pSPIHandle)
{
	//2. Check the DFF bit in CR1
	if( (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
	{
		//16 bit DFF
		//1. load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if(!pSPIHandle->TxLen)
	{
		//TxLen is zero, so close the spi transmission and inform the application that TX is over

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}

static void SPI_RXNE_IT_Handle(SPI_Handle_t *pSPIHandle)
{
	//2. Check the DFF bit in CR1
	if( (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
	{
		//16 bit DFF
		//1. read the data in to the DR
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer--;
		(uint16_t*)pSPIHandle->pRxBuffer--;
	}else
	{
		//8 bit DFF
		*pSPIHandle->pRxBuffer = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}
	if(!pSPIHandle->RxLen)
	{
		//reception is complete, turn off the rxneie interrupt
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}
static void SPI_OVR_ERR_IT_Handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	// 1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp; //to prevent error unused variable
	// 2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEvent)
{
	//this is a weak implementation. The application may override this function
}
