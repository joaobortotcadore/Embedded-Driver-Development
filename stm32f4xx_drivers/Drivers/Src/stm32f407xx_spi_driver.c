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
        if(pSPIx == SPI2)
        {
            SPI3_PCLK_EN();
        }
        if(pSPIx == SPI3)
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
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
        if(pSPIx == SPI2)
        {
            SPI3_PCLK_DI();
        }
        if(pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }
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
void SPI_Init(SPI_Handle_t *pSPIHandle);
/**
 * @fn void SPI_DeInit(SPI_RegDef_t*)
 * @brief
 *
 * @pre
 * @post
 * @param pSPIx
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/* Data Send and Receive */
/**
 * @fn void SPI_SendData(SPI_RegDef_t*, uint8_t*, uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param pSPIx
 * @param pTxBuffer
 * @param Len
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
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
