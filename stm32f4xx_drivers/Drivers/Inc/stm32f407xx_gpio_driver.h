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
}GPIO_PinCOnfig_t;


/* Handle structure for a GPIO pin */
typedef  struct
{
	GPIO_RegDef_t 		*pGPIOx; 				/* This holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinCOnfig_t 	GPIO_PinConfig;			/* This holds GPIO pin configuration settings */
}GPIO_Handle_t;

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
