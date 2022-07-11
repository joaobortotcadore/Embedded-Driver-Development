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


/**************************************
 * APIs supported by this driver
 * ************************************/
/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(void);

/*
 * Init and De-init
 */
void GPIO_Init(void);
void GPIO_DeInit(void);

/*
 * Data read/write
 */
void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void GPIO_WriteToOutputPin(void);
void GPIO_WriteToOutputPort(void);
void GPIO_ToggleOutputPin(void);

/*
 * IRQ Configuration and ISR handling | IRQ - Interrupt ReQuest, ISR - Interrupt Status Register
 */
void GPIO_IRQConfig(void); //to enable it, to set priority, etc
void GPIO_IRQHandling(void); //when an interrupt occurs, this function is responsible to process



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
