/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: 6 de jul de 2022
 *      Author: joaobortotcadore
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


/* Handle structure for a GPIO pin */
typedef  struct
{
	GPIO_RegDef_t 		*pGPIOx; 				/* This holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t 	GPIO_PinConfig;			/* This holds GPIO pin configuration settings */
}GPIO_Handle_t;


/**************************************
 * APIs supported by this driver
 * ************************************/
/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

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
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
/**
 * @fn void GPIO_DeInit(GPIO_Handle_t*)
 * @brief Deinitialize means send the register to your reset state/reset value,
 * we can use the peripheral reset register in RCC section to do this in one shot
 *
 * @param pGPIOx
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read/write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber); /* return uint8_t because return is 1 or 0 / SET or RESET */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx); /* uint16_t because we have 16 pins */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value); /* Value is SET or RESET */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling | IRQ - Interrupt ReQuest, ISR - Interrupt Status Register
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi); //to enable it, to set priority, etc
void GPIO_IRQHandling(uint8_t PinNumber); //when an interrupt occurs, this function is responsible to process



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
