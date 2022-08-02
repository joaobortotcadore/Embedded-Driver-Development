/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 2 de ago de 2022
 *      Author: joaobortotcadore
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/* Configuration structure for SPIx peripheral */
typedef struct
{
    uint8_t SPI_DeviceMode; /*  */
    uint8_t SPI_BusConfig; /*  */
    uint8_t SPI_SclkSpeed; /*  */
    uint8_t SPI_DFF; /*  */
    uint8_t SPI_CPOL; /* */
    uint8_t SPI_CPHA; /* */
    uint8_t SPI_SSM; /* */
}SPI_Config_t;

/* Handle structure for a GPIO pin */
typedef  struct
{
    SPI_RegDef_t        *pSPIx;             /* This holds the base address of SPIx(x:0,1,2) peripheral */
    SPI_Config_t        SPIConfig;          /*  */
}SPI_Handle_t;

/**************************************
 *      APIs supported by this driver
 * ************************************/
/* Peripheral Clock Setup */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/* Init and De-init */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/* Data Send and Receive */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/* IRQ Configuration and ISR handling | IRQ - Interrupt ReQuest, ISR - Interrupt Status Register */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
